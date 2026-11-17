`timescale 1ns / 1ps
//============================================================================
// CMV300_ReadDma.v - DDR3 -> BTPipe FIFO (AXI4 read master, 128-bit, ui_clk)
//
//   On start: reads `blocks` x 1024 bytes from `base` as INCR bursts of 16
//   beats (256 bytes), at most two bursts in flight, a new one only while the
//   output FIFO is not nearly full. Every 128-bit beat is unpacked into four
//   32-bit words, lowest address first (word 0 = beat[31:0]), and written into
//   the 32-bit FIFO that feeds the BTPipeOut - so the host sees the 16-bit
//   little-endian pixels in row-major order with no swapping anywhere.
//   RREADY is held low while a beat is being unpacked (one beat per 5 clocks,
//   320 MB/s, more than USB can take).
//============================================================================
module CMV300_ReadDma #(
    parameter integer AW = 30
)(
    input  wire          clk,
    input  wire          rst,
    input  wire          start,           // pulse
    input  wire [AW-1:0] base,
    input  wire [15:0]   blocks,          // 1024-byte blocks
    output wire          busy,
    output reg           done,            // pulse
    // output FIFO (32-bit write port)
    output reg           fifo_wr,
    output reg  [31:0]   fifo_din,
    input  wire          fifo_prog_full,
    // AXI4 read master
    output wire [3:0]    m_axi_arid,
    output reg  [AW-1:0] m_axi_araddr,
    output wire [7:0]    m_axi_arlen,
    output wire [2:0]    m_axi_arsize,
    output wire [1:0]    m_axi_arburst,
    output reg           m_axi_arvalid,
    input  wire          m_axi_arready,
    input  wire [3:0]    m_axi_rid,
    input  wire [127:0]  m_axi_rdata,
    input  wire [1:0]    m_axi_rresp,
    input  wire          m_axi_rlast,
    input  wire          m_axi_rvalid,
    output wire          m_axi_rready
);
    reg         run = 1'b0;
    reg [17:0]  bursts_left = 18'd0;
    reg [21:0]  beats_left  = 22'd0;
    reg [1:0]   ar_out = 2'd0;            // bursts issued, not yet fully received
    reg [127:0] rbeat = 128'd0;
    reg [2:0]   uc = 3'd0;                // words still to unpack from rbeat

    assign m_axi_arid    = 4'd0;
    assign m_axi_arlen   = 8'd15;
    assign m_axi_arsize  = 3'b100;
    assign m_axi_arburst = 2'b01;
    assign m_axi_rready  = run & (uc == 3'd0);
    assign busy          = run;

    wire ar_hs = m_axi_arvalid & m_axi_arready;
    wire r_hs  = m_axi_rvalid  & m_axi_rready;

    initial begin done = 1'b0; fifo_wr = 1'b0; fifo_din = 32'd0; m_axi_araddr = {AW{1'b0}}; m_axi_arvalid = 1'b0; end

    always @(posedge clk) begin
        done    <= 1'b0;
        fifo_wr <= 1'b0;
        if (rst) begin
            run <= 1'b0; m_axi_arvalid <= 1'b0; ar_out <= 2'd0; uc <= 3'd0;
        end else if (!run) begin
            if (start && blocks != 16'd0) begin
                run <= 1'b1; m_axi_araddr <= base;
                bursts_left <= {blocks, 2'b00}; beats_left <= {blocks, 6'd0};
                ar_out <= 2'd0; uc <= 3'd0;
            end
        end else begin
            // address channel: keep up to two bursts in flight while the FIFO has room
            if (!m_axi_arvalid && bursts_left != 18'd0 && ar_out != 2'd2 && !fifo_prog_full)
                m_axi_arvalid <= 1'b1;
            if (ar_hs) begin
                m_axi_arvalid <= 1'b0; m_axi_araddr <= m_axi_araddr + 30'd256;
                bursts_left <= bursts_left - 18'd1;
            end
            // in-flight accounting (issue and last-beat completion can coincide)
            case ({ar_hs, r_hs & m_axi_rlast})
                2'b10: ar_out <= ar_out + 2'd1;
                2'b01: ar_out <= ar_out - 2'd1;
                default: ;
            endcase
            // data channel -> 4 words per beat
            if (r_hs) begin rbeat <= m_axi_rdata; uc <= 3'd4; beats_left <= beats_left - 22'd1; end
            else if (uc != 3'd0) begin
                fifo_wr  <= 1'b1;
                fifo_din <= rbeat[(3'd4 - uc)*32 +: 32];
                uc <= uc - 3'd1;
            end
            if (beats_left == 22'd0 && uc == 3'd0 && ar_out == 2'd0 && !m_axi_arvalid) begin
                run <= 1'b0; done <= 1'b1;
            end
        end
    end
endmodule
