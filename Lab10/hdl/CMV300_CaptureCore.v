`timescale 1ns / 1ps
//============================================================================
// CMV300_CaptureCore.v - CMV300 parallel video front-end  [GIVEN - do not edit] (CMV300.pdf §4.2.1 p.23,
//                    Figures 26-27): counters, a one-frame BRAM snapshot
//                    (Phase 1) and the pixel stream for the DDR3 ring (Phase 2).
//
//   Sensor side. D[9:0], LVAL, DVAL are registered on the FALLING edge of
//   CLK_OUT (the datasheet says to sample there) in the IOB flops, then taken
//   into the rising-edge domain half a period later; everything else - and the
//   pixel FIFO written by this module - runs on the rising edge of CLK_OUT.
//     * pixels per row  = DVAL highs while LVAL is high (648 expected),
//     * rows per frame  = LVAL rising edges; a frame ends when LVAL has been low
//       for GAP clocks (between rows it is low for one clock, between frames
//       for the whole FOT >= 4,875 clocks),
//     * frame counter, OR/AND masks of every pixel of the frame,
//     * stream: pix_valid/pix_data = one FIFO entry per FOUR pixels (12-bit
//       lanes, {px3, px2, px1, px0}, the first pixel in [11:0]; 648 = 4 x 162
//       so a row never leaves a partial entry), pix_eof one clock at the end
//       of a frame (the token that ends a frame in the pixel FIFO); the same
//       entry format as the LVDS receiver (CMV300_Lvds.v), so one write DMA
//       serves both. The stream is gated by `acq`, and the gate opens only
//       between frames, so the FIFO never receives the tail of a frame that
//       was already running,
//     * snapshot: after snap_arm, the next frame goes into a BRAM of SNAP_WORDS
//       x 32 bits, three 10-bit pixels per word ([9:0] first); 244 rows fill
//       it exactly; read through the PipeOut (word valid the clock after po_read).
//   Test pattern (tp_mode): the sensor's LVAL/DVAL/D are replaced by a
//   generator that produces tp_rows rows of 648 pixels, value = (pixel index
//   + frame number) mod 1024, on every tp_start - so the whole chain from the
//   FIFO to the PC can be verified without an image. It still needs the
//   sensor clock (cam_enable + reg 69 = 9).
//
//   Host side (okClk): counters are handed over with toggle synchronisers
//   (they change only at a frame end). clk_out_meas = CLK_OUT cycles per
//   1.000 ms (25,000 at 25 MHz, +/-16).
//============================================================================
module CMV300_CaptureCore #(
    parameter integer SNAP_WORDS  = 52704,   // 244 rows x 216 words
    parameter integer GAP         = 64,      // LVAL-low clocks that end a frame
    parameter integer MEAS_WINDOW = 100800   // okClk cycles = 1.000 ms
)(
    // sensor (CLK_OUT domain)
    input  wire        clk_cmv,
    input  wire [9:0]  d,
    input  wire        lval,
    input  wire        dval,
    // pixel stream (CLK_OUT rising-edge domain) -> pixel FIFO
    output wire        pix_valid,
    output wire [47:0] pix_data,          // {px3, px2, px1, px0}, 12-bit lanes
    output wire        pix_eof,
    output wire        fifo_rst,           // high for the first 16 CLK_OUT cycles after configuration
    input  wire        acq,                // okClk level: acquisition enabled (gate for the stream)
    // test pattern (okClk)
    input  wire        tp_mode,
    input  wire        tp_loop,            // free-run: the generator restarts by itself after every frame
    input  wire        tp_start,           // pulse (the FRAME_REQ of the ring / host)
    input  wire [15:0] tp_rows,            // 0 = 488
    // host (okClk domain)
    input  wire        clk,
    input  wire        rst,
    input  wire        snap_arm,           // pulse: capture the next frame
    input  wire        rewind,             // pulse: PipeOut pointer back to word 0
    input  wire        clear,              // pulse: frame counter to 0
    input  wire        po_read,
    output wire [31:0] po_data,
    output reg  [15:0] frame_count,
    output reg  [15:0] last_row_pixels,
    output reg  [15:0] last_frame_rows,
    output reg  [9:0]  or_mask,
    output reg  [9:0]  and_mask,
    output reg  [15:0] snap_words,
    output reg  [23:0] frame_period,      // CLK_OUT cycles between the last two frame starts
    output reg         snap_armed,
    output reg         snap_done,
    output reg         frame_done_pulse,
    output reg         snap_done_pulse,
    output reg  [31:0] clk_out_meas
);
    //------------------------------------------------------------------------
    // CLK_OUT domain: falling-edge input flops, then the rising-edge domain
    //------------------------------------------------------------------------
    (* IOB = "TRUE" *) reg [9:0] d_n    = 10'd0;
    (* IOB = "TRUE" *) reg       lval_n = 1'b0;
    (* IOB = "TRUE" *) reg       dval_n = 1'b0;
    always @(negedge clk_cmv) begin d_n <= d; lval_n <= lval; dval_n <= dval; end

    // power-on reset for the pixel FIFO: 16 cycles once the sensor clock starts
    reg [4:0] por = 5'd0;
    always @(posedge clk_cmv) if (!por[4]) por <= por + 5'd1;
    assign fifo_rst = ~por[4];

    reg        in_frame  = 1'b0;
    reg        stream_en = 1'b0;
    reg [1:0]  acq_s = 2'd0;

    // test-pattern generator
    reg [1:0]  tpm_s = 2'd0, tpl_s = 2'd0;
    reg        tps_t = 1'b0;                 // okClk toggle
    reg [2:0]  tps_s = 3'd0;
    reg        tp_run = 1'b0, tp_gap = 1'b0, tp_pending = 1'b0;
    reg [8:0]  tp_idle = 9'd0;               // gap between looped frames
    reg [15:0] tp_row = 16'd0, tp_col = 16'd0, tp_frame = 16'd0;
    reg [9:0]  tp_val = 10'd0;
    reg [9:0]  g_d = 10'd0;
    reg        g_lval = 1'b0, g_dval = 1'b0;
    wire [15:0] rows_eff = (tp_rows == 16'd0) ? 16'd488 : tp_rows;

    // (stream_en / in_frame are declared below; the generator, like the sensor, only starts a frame
    //  when the capture is ready for one - a request that arrives earlier is queued, not lost)
    always @(posedge clk_cmv) begin
        tpm_s <= {tpm_s[0], tp_mode};
        tpl_s <= {tpl_s[0], tp_loop};
        tps_s <= {tps_s[1:0], tps_t};
        g_lval <= 1'b0; g_dval <= 1'b0; g_d <= 10'd0;
        if (tps_s[2] ^ tps_s[1]) tp_pending <= 1'b1;
        if (tp_idle != 9'd0) tp_idle <= tp_idle - 9'd1;
        if (!tpm_s[1]) begin tp_run <= 1'b0; tp_pending <= 1'b0; end          // generator only lives in test-pattern mode
        else if (tp_pending && !tp_run && stream_en && !in_frame && tp_idle == 9'd0) begin
            tp_pending <= 1'b0;
            tp_run <= 1'b1; tp_row <= 16'd0; tp_col <= 16'd0; tp_val <= tp_frame[9:0]; tp_gap <= 1'b0;
        end else if (tp_run) begin
            if (tp_gap) tp_gap <= 1'b0;                          // one idle clock between rows
            else begin
                g_lval <= 1'b1; g_dval <= 1'b1; g_d <= tp_val; tp_val <= tp_val + 10'd1;
                if (tp_col == 16'd647) begin
                    tp_col <= 16'd0; tp_gap <= 1'b1;
                    if (tp_row + 16'd1 >= rows_eff) begin                     // >= : a row count lowered mid-frame ends the frame
                        tp_run <= 1'b0; tp_frame <= tp_frame + 16'd1;
                        if (tpl_s[1]) begin tp_pending <= 1'b1; tp_idle <= 9'd300; end   // loop: next frame after a 300-clock gap
                    end
                    else tp_row <= tp_row + 16'd1;
                end else tp_col <= tp_col + 16'd1;
            end
        end
    end

    reg [9:0]  d_r = 10'd0;
    reg        lval_r = 1'b0, dval_r = 1'b0, lval_d = 1'b0;
    reg [15:0] pix_cnt     = 16'd0;
    reg [15:0] row_cnt     = 16'd0;
    reg [7:0]  gap_cnt     = 8'd0;
    reg [15:0] frame_cnt_c = 16'd0, last_row_c = 16'd0, last_rows_c = 16'd0, snap_words_c = 16'd0;
    reg [23:0] period_cnt = 24'd0, period_l = 24'd0;

    reg [9:0]  or_c = 10'd0, and_c = 10'h3FF, or_l = 10'd0, and_l = 10'd0;
    reg        frame_t = 1'b0, snap_t = 1'b0, tick_t = 1'b0;
    reg [3:0]  tick_cnt = 4'd0;
    reg        arm_t = 1'b0, clr_t = 1'b0;             // okClk-side toggles
    reg [2:0]  arm_s = 3'd0, clr_s = 3'd0;             // synchronised into CLK_OUT
    reg        arm_pending = 1'b0, snapping = 1'b0;
    reg [15:0] wr_addr = 16'd0;
    reg [1:0]  pk = 2'd0;
    reg [19:0] pk_buf = 20'd0;
    reg        mem_we = 1'b0;
    reg [15:0] mem_wa = 16'd0;
    reg [31:0] mem_wd = 32'd0;

    wire lval_rise = lval_r & ~lval_d;
    wire lval_fall = ~lval_r & lval_d;
    wire pix       = lval_r & dval_r;
    wire frame_end = in_frame & ~lval_r & (gap_cnt == GAP - 1);

    // The four-pixel packer is CMV300_Capture.v - the module the student writes.
    // Everything it needs is already registered in this rising-edge domain.
    CMV300_Capture u_pack (
        .clk         (clk_cmv),
        .pixel       (d_r),
        .pixel_valid (pix),
        .line_valid  (lval_r),
        .stream_en   (stream_en),
        .frame_end   (frame_end),
        .pix_valid   (pix_valid),
        .pix_data    (pix_data),
        .pix_eof     (pix_eof)
    );

    always @(posedge clk_cmv) begin
        // input mux: sensor or generator
        d_r    <= tpm_s[1] ? g_d    : d_n;
        lval_r <= tpm_s[1] ? g_lval : lval_n;
        dval_r <= tpm_s[1] ? g_dval : dval_n;
        lval_d <= lval_r;
        tick_cnt <= tick_cnt + 4'd1;
        if (tick_cnt == 4'd15) tick_t <= ~tick_t;
        arm_s <= {arm_s[1:0], arm_t};
        clr_s <= {clr_s[1:0], clr_t};
        if (arm_s[2] ^ arm_s[1]) arm_pending <= 1'b1;
        if (clr_s[2] ^ clr_s[1]) frame_cnt_c <= 16'd0;
        gap_cnt <= lval_r ? 8'd0 : ((gap_cnt == GAP - 1) ? gap_cnt : gap_cnt + 8'd1);
        mem_we  <= 1'b0;

        // stream to the pixel FIFO: the gate opens only while no frame is running, closes at once
        acq_s <= {acq_s[0], acq};
        if (!acq_s[1])                 stream_en <= 1'b0;
        else if (!in_frame && !lval_r) stream_en <= 1'b1;
        // the pixel entries and the end-of-frame token come from u_pack above

        // pixels of a row
        if (lval_rise)    pix_cnt <= dval_r ? 16'd1 : 16'd0;
        else if (pix)     pix_cnt <= pix_cnt + 16'd1;
        if (lval_fall)    last_row_c <= pix_cnt;
        if (pix) begin or_c <= or_c | d_r; and_c <= and_c & d_r; end

        // frame period: CLK_OUT cycles from frame start to frame start
        if (period_cnt != 24'hFFFFFF) period_cnt <= period_cnt + 24'd1;
        // rows and frames
        if (lval_rise) begin
            if (!in_frame) begin
                in_frame <= 1'b1; row_cnt <= 16'd1;
                period_l <= period_cnt; period_cnt <= 24'd0;
                or_c  <= pix ? d_r : 10'd0;
                and_c <= pix ? d_r : 10'h3FF;
                if (arm_pending) begin
                    snapping <= 1'b1; arm_pending <= 1'b0; wr_addr <= 16'd0; pk <= 2'd0;
                end
            end else row_cnt <= row_cnt + 16'd1;
        end
        if (frame_end) begin
            in_frame <= 1'b0; last_rows_c <= row_cnt; frame_cnt_c <= frame_cnt_c + 16'd1;
            or_l <= or_c; and_l <= and_c; frame_t <= ~frame_t;
            if (snapping) begin
                snapping <= 1'b0; snap_t <= ~snap_t;
                snap_words_c <= wr_addr + {15'd0, (pk != 2'd0)};
                if (pk != 2'd0) begin mem_we <= 1'b1; mem_wa <= wr_addr; mem_wd <= {12'd0, pk_buf}; end
            end
        end

        // snapshot: pack three pixels per word
        if (snapping && pix) begin
            if (pk == 2'd2) begin
                mem_we <= 1'b1; mem_wa <= wr_addr; mem_wd <= {2'b00, d_r, pk_buf};
                wr_addr <= wr_addr + 16'd1; pk <= 2'd0;
                if (wr_addr == SNAP_WORDS - 1) begin
                    snapping <= 1'b0; snap_words_c <= SNAP_WORDS; snap_t <= ~snap_t;
                end
            end else begin
                if (pk == 2'd0) pk_buf[9:0] <= d_r; else pk_buf[19:10] <= d_r;
                pk <= pk + 2'd1;
            end
        end
    end

    //------------------------------------------------------------------------
    // Snapshot BRAM: written on CLK_OUT, read on okClk
    //------------------------------------------------------------------------
    (* ram_style = "block" *) reg [31:0] mem [0:SNAP_WORDS-1];
    always @(posedge clk_cmv) if (mem_we) mem[mem_wa] <= mem_wd;

    reg [15:0] rd_addr = 16'd0;
    reg [31:0] rd_q    = 32'd0;
    always @(posedge clk) begin
        rd_q <= mem[rd_addr];
        if (rst || rewind)  rd_addr <= 16'd0;
        else if (po_read)   rd_addr <= (rd_addr == SNAP_WORDS - 1) ? 16'd0 : rd_addr + 16'd1;
    end
    assign po_data = rd_q;

    //------------------------------------------------------------------------
    // okClk domain: hand-over of the counters, snapshot control, CLK_OUT measurement
    //------------------------------------------------------------------------
    reg [2:0]  ft_s = 3'd0, st_s = 3'd0, tk_s = 3'd0;
    reg [16:0] win_cnt  = 17'd0;
    reg [15:0] edge_cnt = 16'd0;

    initial begin
        frame_count = 16'd0; last_row_pixels = 16'd0; last_frame_rows = 16'd0;
        or_mask = 10'd0; and_mask = 10'd0; snap_words = 16'd0; frame_period = 24'd0;
        snap_armed = 1'b0; snap_done = 1'b0; frame_done_pulse = 1'b0; snap_done_pulse = 1'b0;
        clk_out_meas = 32'd0;
    end

    always @(posedge clk) begin
        ft_s <= {ft_s[1:0], frame_t};
        st_s <= {st_s[1:0], snap_t};
        tk_s <= {tk_s[1:0], tick_t};
        frame_done_pulse <= ft_s[2] ^ ft_s[1];
        snap_done_pulse  <= st_s[2] ^ st_s[1];
        if (ft_s[2] ^ ft_s[1]) begin
            frame_count <= frame_cnt_c; last_row_pixels <= last_row_c; last_frame_rows <= last_rows_c;
            or_mask <= or_l; and_mask <= and_l; frame_period <= period_l;
        end
        if (st_s[2] ^ st_s[1]) begin snap_words <= snap_words_c; snap_done <= 1'b1; snap_armed <= 1'b0; end
        if (snap_arm)          begin arm_t <= ~arm_t; snap_armed <= 1'b1; snap_done <= 1'b0; end
        if (clear)             begin clr_t <= ~clr_t; frame_count <= 16'd0; end
        if (tp_start)          tps_t <= ~tps_t;

        // CLK_OUT frequency: count tick_t edges (16 CLK_OUT cycles each) per 1 ms
        if (win_cnt == MEAS_WINDOW - 1) begin
            win_cnt <= 17'd0; edge_cnt <= 16'd0;
            clk_out_meas <= {12'd0, edge_cnt, 4'd0};      // x16
        end else begin
            win_cnt <= win_cnt + 17'd1;
            if (tk_s[2] ^ tk_s[1]) edge_cnt <= edge_cnt + 16'd1;
        end

        if (rst) begin snap_armed <= 1'b0; snap_done <= 1'b0; end
    end
endmodule
