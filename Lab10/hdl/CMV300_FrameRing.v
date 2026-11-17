`timescale 1ns / 1ps
//============================================================================
// CMV300_FrameRing.v - three frame slots in DDR3 and the handshakes around them
//                      (ui_clk domain; docs/CMV300_CAPTURE_PLAN.md §2, mode A).
//
//   Slot i lives at base i << SLOT_SHIFT (1 MiB stride). States FREE ->
//   WRITING -> READY -> READING -> FREE.
//   Writer: when acquisition is enabled, the DDR3 is calibrated, a slot is
//   FREE and the minimum frame period has elapsed, the slot is given to the
//   write DMA and ONE frame_req pulse is issued (the sensor is idle by
//   construction: the previous frame was completely written first). When the
//   DMA reports the frame, the slot becomes READY with the next sequence
//   number. A frame of 0 blocks, or a wait longer than TIMEOUT cycles
//   (250 ms - no sensor clock, no frame), frees the slot and counts a drop.
//   Reader: read_req takes the oldest READY slot (READING), starts the read
//   DMA and publishes its length; read_done pulses when the DMA has pushed
//   everything into the pipe FIFO (the host may already be reading);
//   read_ack frees the slot. A read_req with nothing READY sets read_err.
//   Stopping acquisition keeps READY frames readable; the next start discards them.
//   Free-running mode (free_run = 1, "mode B"): the sensor is programmed for
//   many frames (Number_frames = 65535) and pipelines by itself; the writer
//   hands each free slot to the DMA without requesting anything, the DMA
//   drops the frames that find no slot (frame_dropped -> frames_dropped; they still consume a sequence number, so the host sees the gap),
//   and one FRAME_REQ is issued whenever the sensor has been idle for
//   IDLE_MS_REQ ms (start-up, and after the programmed sequence ends).
//============================================================================
module CMV300_FrameRing #(
    parameter integer AW         = 30,
    parameter integer SLOT_SHIFT = 20,
    parameter integer TIMEOUT    = 25000000,   // ui_clk cycles = 250 ms at 100 MHz
    parameter integer MS_CYCLES  = 100000,     // ui_clk cycles per ms
    parameter integer IDLE_MS_REQ = 100        // free-run: re-request after this many idle ms
)(
    input  wire          clk,
    input  wire          rst,
    input  wire          enable,             // acquisition on (level)
    input  wire          free_run,
    input  wire          stream,            // auto-serve: the reader requests and frees READY frames by itself           // mode B (level)
    input  wire          calib_done,
    input  wire [7:0]    min_period_ms,      // 0 = as fast as the sensor goes
    // write DMA
    output reg           wr_slot_valid,
    output reg  [AW-1:0] wr_slot_base,
    output reg           wr_abort,
    output reg           wr_flush,
    input  wire          wr_busy,
    input  wire          wr_frame_done,
    input  wire          wr_frame_dropped,
    input  wire [15:0]   wr_frame_blocks,
    input  wire [23:0]   wr_frame_pixels,
    output reg           frame_req,          // pulse -> sensor FRAME_REQ (via the sequencer)
    // read DMA
    output reg           rd_start,
    output reg  [AW-1:0] rd_base,
    output reg  [15:0]   rd_blocks,
    input  wire          rd_busy,
    input  wire          rd_done_in,
    // host (pulses already in this clock domain)
    input  wire          read_req,
    input  wire          read_ack,
    output reg           read_done,          // pulse
    output reg           frame_ready,        // pulse: a slot became READY
    output reg  [1:0]    frames_ready,       // READY slots (0..3)
    output reg  [1:0]    read_slot,
    output reg  [7:0]    read_seq,
    output reg  [15:0]   read_blocks,
    output reg  [23:0]   read_pixels,
    output reg  [15:0]   frames_stored,
    output reg  [15:0]   frames_dropped,
    output reg           read_err,
    output wire          acq_active,
    output wire [7:0]    dbg
);
    localparam [1:0] FREE = 2'd0, WRITING = 2'd1, READY = 2'd2, READING = 2'd3;
    localparam [1:0] W_IDLE = 2'd0, W_WAIT = 2'd1, W_ABORT = 2'd2, W_FLUSH = 2'd3;
    localparam [1:0] R_IDLE = 2'd0, R_BUSY = 2'd1, R_ACK = 2'd2;

    reg [1:0]  sst [0:2];                  // slot state
    reg [7:0]  seq [0:2];                  // sequence number of a READY/READING slot
    reg [15:0] blk [0:2];
    reg [23:0] pxl [0:2];
    reg [1:0]  wst = W_IDLE, rst_ = R_IDLE;
    reg        auto_free = 1'b0;
    reg [1:0]  wcur = 2'd0, rcur = 2'd0;
    reg [7:0]  next_seq = 8'd0, exp_seq = 8'd0;
    reg [24:0] wait_cnt = 25'd0;
    reg [16:0] ms_cnt = 17'd0;
    reg [7:0]  ms_since_req = 8'hFF;
    reg [7:0]  idle_ms = 8'hFF;             // ms since the last frame request or frame done (saturating)
    reg        enable_d = 1'b0;

    integer i;
    initial begin
        for (i = 0; i < 3; i = i + 1) begin sst[i] = FREE; seq[i] = 8'd0; blk[i] = 16'd0; pxl[i] = 24'd0; end
        wr_slot_valid = 1'b0; wr_slot_base = {AW{1'b0}}; wr_abort = 1'b0; wr_flush = 1'b0; frame_req = 1'b0;
        rd_start = 1'b0; rd_base = {AW{1'b0}}; rd_blocks = 16'd0; read_done = 1'b0; frame_ready = 1'b0;
        frames_ready = 2'd0; read_slot = 2'd0; read_seq = 8'd0; read_blocks = 16'd0; read_pixels = 24'd0;
        frames_stored = 16'd0; frames_dropped = 16'd0; read_err = 1'b0;
    end

    // which slot is FREE (lowest index), which READY slot is the oldest
    wire        any_free  = (sst[0] == FREE) | (sst[1] == FREE) | (sst[2] == FREE);
    wire [1:0]  free_idx  = (sst[0] == FREE) ? 2'd0 : (sst[1] == FREE) ? 2'd1 : 2'd2;
    wire        r0 = (sst[0] == READY), r1 = (sst[1] == READY), r2 = (sst[2] == READY);
    wire        any_ready = r0 | r1 | r2;
    // oldest = the one whose seq is the expected next one; fall back to the smallest (signed) difference
    wire signed [7:0] d0 = seq[0] - exp_seq, d1 = seq[1] - exp_seq, d2 = seq[2] - exp_seq;
    wire [1:0]  old01 = (!r1 || (r0 && d0 <= d1)) ? 2'd0 : 2'd1;
    wire signed [7:0] d01 = (old01 == 2'd0) ? d0 : d1;
    wire        r01 = r0 | r1;
    wire [1:0]  oldest = (!r2 || (r01 && d01 <= d2)) ? old01 : 2'd2;
    // registered copy: the three sequence compares + the slot mux were the critical path at 100 MHz;
    // read_req is a host pulse, so a one-cycle-old selection is as good as a live one
    reg  [1:0]  oldest_r = 2'd0;
    reg         any_ready_r = 1'b0;
    always @(posedge clk) begin oldest_r <= oldest; any_ready_r <= any_ready; end
    wire        period_ok = (min_period_ms == 8'd0) || (ms_since_req >= min_period_ms);

    assign acq_active = (wst == W_WAIT);
    assign dbg = {sst[2], sst[1], sst[0], wst};

    always @(posedge clk) begin
        frame_req <= 1'b0; rd_start <= 1'b0; read_done <= 1'b0; frame_ready <= 1'b0; wr_flush <= 1'b0;
        frames_ready <= {1'b0, r0} + {1'b0, r1} + {1'b0, r2};
        enable_d <= enable;
        // millisecond counter since the last frame request
        if (ms_cnt == MS_CYCLES - 1) begin
            ms_cnt <= 17'd0;
            if (ms_since_req != 8'hFF) ms_since_req <= ms_since_req + 8'd1;
            if (idle_ms != 8'hFF)      idle_ms <= idle_ms + 8'd1;
        end else ms_cnt <= ms_cnt + 17'd1;
        if (wr_frame_done || wr_frame_dropped) idle_ms <= 8'd0;   // frames arriving (stored or dropped) = sensor alive

        if (rst) begin
            for (i = 0; i < 3; i = i + 1) sst[i] <= FREE;
            wst <= W_IDLE; rst_ <= R_IDLE; wr_slot_valid <= 1'b0; wr_abort <= 1'b0; auto_free <= 1'b0;
            next_seq <= 8'd0; exp_seq <= 8'd0; frames_stored <= 16'd0; frames_dropped <= 16'd0; read_err <= 1'b0;
        end else begin
            if (wr_frame_dropped) begin frames_dropped <= frames_dropped + 16'd1; next_seq <= next_seq + 8'd1; end   // a dropped frame leaves a gap in the sequence numbers
            // free-running sensor: (re)start it when it has been idle for a while
            if (free_run && enable && calib_done && wst != W_FLUSH && idle_ms >= IDLE_MS_REQ) begin
                frame_req <= 1'b1; idle_ms <= 8'd0;
            end
            //------------------------------------------------ writer
            case (wst)
                W_IDLE: begin
                    wr_slot_valid <= 1'b0; wr_abort <= 1'b0;
                    if (enable && !enable_d) begin                                                       // acquisition starts: drop stale pixels and unread frames
                        wr_flush <= 1'b1; wait_cnt <= 25'd0; wst <= W_FLUSH;
                        for (i = 0; i < 3; i = i + 1) if (sst[i] == READY) sst[i] <= FREE;
                    end
                    else if (enable && calib_done && any_free && !wr_busy && (period_ok || free_run)) begin
                        wcur <= free_idx; sst[free_idx] <= WRITING;
                        wr_slot_base <= {{(AW-SLOT_SHIFT-2){1'b0}}, free_idx, {SLOT_SHIFT{1'b0}}};
                        wr_slot_valid <= 1'b1;
                        if (!free_run) begin frame_req <= 1'b1; ms_since_req <= 8'd0; idle_ms <= 8'd0; end   // mode A: one request per slot
                        wait_cnt <= 25'd0; wst <= W_WAIT;
                    end
                end
                W_WAIT: begin
                    if (wr_frame_done) begin
                        wr_slot_valid <= 1'b0;
                        if (wr_frame_blocks == 16'd0) begin sst[wcur] <= FREE; frames_dropped <= frames_dropped + 16'd1; end
                        else begin
                            sst[wcur] <= READY; seq[wcur] <= next_seq; next_seq <= next_seq + 8'd1;
                            blk[wcur] <= wr_frame_blocks; pxl[wcur] <= wr_frame_pixels;
                            frames_stored <= frames_stored + 16'd1; frame_ready <= 1'b1;
                        end
                        wst <= W_IDLE;
                    end else if (wait_cnt == TIMEOUT - 1 || !enable) begin   // no frame, or acquisition switched off: give the slot up
                        wr_slot_valid <= 1'b0; wr_abort <= 1'b1; wst <= W_ABORT;
                    end else wait_cnt <= wait_cnt + 25'd1;
                end
                W_ABORT: begin
                    if (!wr_busy) begin
                        wr_abort <= 1'b0; sst[wcur] <= FREE; frames_dropped <= frames_dropped + 16'd1; wst <= W_IDLE;
                    end
                end
                W_FLUSH: begin                                              // the DMA is draining: no request until it is idle again
                    wait_cnt <= wait_cnt + 25'd1;
                    if (wait_cnt > 25'd4 && !wr_busy) wst <= W_IDLE;
                end
                default: wst <= W_IDLE;
            endcase
            //------------------------------------------------ reader
            case (rst_)
                R_IDLE: begin
                    if (read_req || (stream && any_ready_r && !rd_busy && !(enable && !enable_d))) begin
                        if (any_ready_r && !rd_busy && !(enable && !enable_d)) begin
                            auto_free <= stream & ~read_req;                     // streaming: free the slot on rd_done, no ack needed
                            rcur <= oldest_r; sst[oldest_r] <= READING; read_err <= 1'b0;
                            read_slot <= oldest_r; read_seq <= seq[oldest_r]; read_blocks <= blk[oldest_r]; read_pixels <= pxl[oldest_r];
                            rd_base <= {{(AW-SLOT_SHIFT-2){1'b0}}, oldest_r, {SLOT_SHIFT{1'b0}}};
                            rd_blocks <= blk[oldest_r]; rd_start <= 1'b1;
                            rst_ <= R_BUSY;
                        end else begin read_err <= 1'b1; read_blocks <= 16'd0; end
                    end
                end
                R_BUSY: begin
                    if (rd_done_in) begin
                        read_done <= 1'b1;
                        if (auto_free) begin sst[rcur] <= FREE; exp_seq <= seq[rcur] + 8'd1; rst_ <= R_IDLE; end
                        else rst_ <= R_ACK;
                    end
                    if (read_ack && !auto_free) begin sst[rcur] <= FREE; exp_seq <= seq[rcur] + 8'd1; rst_ <= R_IDLE; end
                end
                R_ACK: begin
                    if (read_ack) begin sst[rcur] <= FREE; exp_seq <= seq[rcur] + 8'd1; rst_ <= R_IDLE; end
                end
                default: rst_ <= R_IDLE;
            endcase
        end
    end
endmodule
