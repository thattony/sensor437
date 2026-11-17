`timescale 1ns / 1ps
//============================================================================
// CMV300_WriteDma.v - pixel FIFO -> DDR3 (AXI4 write master, 128-bit, ui_clk)
//
//   Pops the pixel FIFO (first-word-fall-through, 49 bits: [48] = end-of-frame
//   token, [47:0] = four 12-bit pixels, the first in [11:0]) and PACKS the
//   pixels: the 48 payload bits of every entry go into a 176-bit accumulator
//   and a 128-bit beat is emitted whenever one is complete (at most one per
//   pop, so the 1-entry-per-2-clocks pace is unchanged). The stored stream is
//   therefore little-endian packed 12-bit - two pixels per three bytes, pixel
//   0 in the low nibbles first - which is 25 % fewer bytes than 16-bit words
//   and what lets 478.9 fps fit through the USB pipe. Parallel-mode pixels
//   (10-bit) sit in the low bits of their 12-bit field.
//   collects 16 beats in a small buffer and writes them as one INCR burst of
//   256 bytes. At the end-of-frame token the partial beat/burst is padded with
//   zeros and zero bursts are added until the frame occupies a whole number of
//   1024-byte blocks (the BTPipe block size), then all write responses are
//   awaited and frame_done reports the length in blocks and in pixels.
//
//   Control from the frame ring: slot_valid/slot_base (where to write), abort
//   (give up waiting for a frame - only honoured while collecting), flush
//   (throw away whatever is in the FIFO, used when acquisition starts).
//   Without a slot (free-running sensor, all slots full) the DMA keeps popping
//   the FIFO and drops whole frames - `boundary` remembers whether the next
//   entry is the first pixel of a frame, so a slot is only taken up at a frame
//   start; every dropped frame is reported with a frame_dropped pulse.
//   Never touches the AXI channels out of order: AW, then exactly 16 W beats
//   with WLAST on the last, BREADY always high.
//============================================================================
module CMV300_WriteDma #(
    parameter integer AW = 30
)(
    input  wire          clk,
    input  wire          rst,
    // pixel FIFO (fwft)
    input  wire [48:0]   fifo_dout,
    input  wire          fifo_empty,
    output reg           fifo_rd,
    // control
    input  wire          slot_valid,
    input  wire [AW-1:0] slot_base,
    input  wire          abort,
    input  wire          flush,
    output wire          busy,
    output reg           frame_done,       // pulse
    output reg           frame_dropped,    // pulse: a frame went by with no slot to put it in
    output reg  [15:0]   frame_blocks,     // 1024-byte blocks written
    output reg  [23:0]   frame_pixels,
    // AXI4 write master
    output wire [3:0]    m_axi_awid,
    output reg  [AW-1:0] m_axi_awaddr,
    output wire [7:0]    m_axi_awlen,
    output wire [2:0]    m_axi_awsize,
    output wire [1:0]    m_axi_awburst,
    output reg           m_axi_awvalid,
    input  wire          m_axi_awready,
    output wire [127:0]  m_axi_wdata,
    output wire [15:0]   m_axi_wstrb,
    output wire          m_axi_wlast,
    output reg           m_axi_wvalid,
    input  wire          m_axi_wready,
    input  wire [3:0]    m_axi_bid,
    input  wire [1:0]    m_axi_bresp,
    input  wire          m_axi_bvalid,
    output wire          m_axi_bready
);
    localparam [2:0] S_IDLE = 3'd0, S_DRAIN = 3'd1, S_COLLECT = 3'd2, S_AW = 3'd3,
                     S_W = 3'd4, S_PADCHK = 3'd5, S_WAIT_B = 3'd6, S_RELEASE = 3'd7;

    reg [2:0]   st = S_IDLE;
    reg [175:0] acc = 176'd0;                    // packing accumulator: 48 bits per entry, 128-bit beats out
    reg [7:0]   acc_bits = 8'd0;                 // bits held in acc (0..112, multiples of 16)
    reg [4:0]   wi = 5'd0;                       // beats in the burst buffer (0..16)
    reg [3:0]   bi = 4'd0;                       // beat being sent
    reg         eof = 1'b0;
    reg [15:0]  bursts = 16'd0;
    reg [23:0]  pixels = 24'd0;
    reg [7:0]   drain_idle = 8'd0;
    reg [16:0]  idle_cnt = 17'd0;                // S_IDLE cycles with a slot armed but no frame boundary
    reg [3:0]   b_out = 4'd0;                    // write responses outstanding
    reg         boundary = 1'b1;                 // the next FIFO entry starts a frame
    reg [127:0] bb [0:15];                       // burst buffer (distributed RAM)

    assign m_axi_awid    = 4'd0;
    assign m_axi_awlen   = 8'd15;                // 16 beats
    assign m_axi_awsize  = 3'b100;               // 16 bytes per beat
    assign m_axi_awburst = 2'b01;                // INCR
    assign m_axi_wstrb   = 16'hFFFF;
    assign m_axi_wdata   = (bi < wi) ? bb[bi] : 128'd0;
    assign m_axi_wlast   = (bi == 4'd15);
    assign m_axi_bready  = 1'b1;
    assign busy          = (st != S_IDLE);

    // the accumulator with this entry's 48 bits appended at the current fill level
    wire [175:0] acc_full = acc | ({128'd0, fifo_dout[47:0]} << acc_bits);
    wire aw_hs = m_axi_awvalid & m_axi_awready;
    wire w_hs  = m_axi_wvalid  & m_axi_wready;

    initial begin fifo_rd = 1'b0; frame_done = 1'b0; frame_dropped = 1'b0; frame_blocks = 16'd0; frame_pixels = 24'd0;
                  m_axi_awaddr = {AW{1'b0}}; m_axi_awvalid = 1'b0; m_axi_wvalid = 1'b0; end

    always @(posedge clk) begin
        frame_done <= 1'b0;
        frame_dropped <= 1'b0;
        fifo_rd    <= 1'b0;
        // write responses
        if (aw_hs && !m_axi_bvalid)      b_out <= b_out + 4'd1;
        else if (m_axi_bvalid && !aw_hs) b_out <= b_out - 4'd1;

        if (rst) begin
            st <= S_IDLE; m_axi_awvalid <= 1'b0; m_axi_wvalid <= 1'b0; b_out <= 4'd0;
            wi <= 5'd0; acc <= 176'd0; acc_bits <= 8'd0; eof <= 1'b0; boundary <= 1'b1;
        end else case (st)

            S_IDLE: begin
                wi <= 5'd0; acc <= 176'd0; acc_bits <= 8'd0; eof <= 1'b0; bursts <= 16'd0; pixels <= 24'd0;
                if (flush) begin idle_cnt <= 17'd0; drain_idle <= 8'd0; st <= S_DRAIN; end
                else if (slot_valid && boundary) begin idle_cnt <= 17'd0; m_axi_awaddr <= slot_base; st <= S_COLLECT; end
                else if (!fifo_empty && !fifo_rd) begin  // no slot, or mid-frame: drop until the next frame start
                    idle_cnt <= 17'd0;
                    fifo_rd <= 1'b1;
                    if (fifo_dout[48]) begin boundary <= 1'b1; if (!boundary) frame_dropped <= 1'b1; end
                    else boundary <= 1'b0;
                end
                else if (slot_valid && !boundary) begin  // a frame whose end-of-frame token never came (e.g. the
                    idle_cnt <= idle_cnt + 17'd1;        // front end lost its lock mid-frame): after ~1.3 ms of
                    if (idle_cnt[16]) begin              // silence count it as dropped and re-arm - never deadlock
                        boundary <= 1'b1; frame_dropped <= 1'b1; idle_cnt <= 17'd0;
                    end
                end
            end

            S_DRAIN: begin                               // empty the FIFO, leave 16 cycles after it went empty
                if (!fifo_empty) begin fifo_rd <= 1'b1; drain_idle <= 8'd0; end
                else if (drain_idle == 8'd15) begin boundary <= 1'b1; st <= S_IDLE; end
                else drain_idle <= drain_idle + 8'd1;
            end

            S_COLLECT: begin
                if (abort) begin boundary <= eof; st <= S_RELEASE; end
                else if (wi == 5'd16) st <= S_AW;                          // a full burst is ready
                else if (eof) begin
                    if (acc_bits != 8'd0) begin                            // flush the packed tail, zero-padded
                        bb[wi] <= acc[127:0]; wi <= wi + 5'd1; acc <= 176'd0; acc_bits <= 8'd0;
                    end else if (wi != 5'd0) st <= S_AW;                   // partial burst (padded with zero beats)
                    else st <= S_PADCHK;
                end else if (!fifo_empty && !fifo_rd) begin                // pop one entry (fwft, 1 per 2 clocks)
                    fifo_rd <= 1'b1;
                    if (fifo_dout[48]) begin eof <= 1'b1; boundary <= 1'b1; end
                    else begin
                        pixels <= pixels + 24'd4;
                        if (acc_bits >= 8'd80) begin                       // these 48 bits complete a beat
                            bb[wi] <= acc_full[127:0]; wi <= wi + 5'd1;
                            acc <= {128'd0, acc_full[175:128]};
                            acc_bits <= acc_bits - 8'd80;                  // (acc_bits + 48) - 128
                        end else begin
                            acc <= acc_full; acc_bits <= acc_bits + 8'd48;
                        end
                    end
                end
            end

            S_AW: begin
                m_axi_awvalid <= 1'b1;
                if (aw_hs) begin m_axi_awvalid <= 1'b0; bi <= 4'd0; m_axi_wvalid <= 1'b1; st <= S_W; end
            end

            S_W: begin
                if (w_hs) begin
                    if (bi == 4'd15) begin
                        m_axi_wvalid <= 1'b0; bursts <= bursts + 16'd1;
                        m_axi_awaddr <= m_axi_awaddr + 30'd256; wi <= 5'd0;
                        st <= S_COLLECT;                 // COLLECT decides: more pixels, the partial beat, or padding
                    end else bi <= bi + 4'd1;
                end
            end

            S_PADCHK: begin                              // pad with zero bursts to a 1024-byte boundary
                if (bursts[1:0] == 2'd0) st <= S_WAIT_B;
                else begin wi <= 5'd0; st <= S_AW; end
            end

            S_WAIT_B: begin
                if (b_out == 4'd0 && !m_axi_bvalid) begin
                    frame_done <= 1'b1; frame_blocks <= bursts >> 2; frame_pixels <= pixels; st <= S_RELEASE;
                end
            end

            S_RELEASE: begin                             // stay busy until the ring has taken slot_valid away
                if (!slot_valid) st <= S_IDLE;
            end

            default: st <= S_IDLE;
        endcase
    end
endmodule
