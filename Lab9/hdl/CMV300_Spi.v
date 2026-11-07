`timescale 1ns / 1ps
//============================================================================
// CMV300_Spi.v - THIS IS A FILE YOU WRITE (2 of 3). SPI master for the CMV300
// register interface.
//
// WHAT IT MUST DO (CMV300.pdf section 3.9 Figures 7-9):
//   * One frame = 16 bits, MSB first: control bit (1 = write, 0 = read),
//     7 address bits, 8 data bits.
//   * The SENSOR samples SPI_IN on the RISING edge of SPI_CLK and launches
//     SPI_OUT on the FALLING edge - so YOUR master changes SPI_IN on falling
//     edges and samples SPI_OUT on rising edges.
//   * SPI_EN high frames the transfer: raise it (with the control bit already
//     on SPI_IN) half a period before the first rising edge, keep it high one
//     period after the last one. SPI_CLK idles low.
//   * SPI_CLK <= 10 MHz (HALF = 5 gives 100.8 / 10 = 10.08 MHz - fine).
//   * burst = 1..3: that many EXTRA frames back to back with SPI_EN held high
//     and addresses addr, addr+1, ... (Figure 8; the temperature registers
//     78/79 are read this way). Read byte i of the burst lands in
//     rd_data[8i+7:8i]. A burst write repeats the same data byte.
//   * done = one-clock pulse when the transfer completes; busy while it runs.
//
// The PORT LIST IS FROZEN (INTERFACE.md). This stub compiles, keeps the SPI
// bus inactive, and fails the testbench with named checks.
//============================================================================
module CMV300_Spi #(
    parameter integer HALF = 5          // okClk cycles per half SPI_CLK period
)(
    input  wire        clk,             // okClk 100.8 MHz
    input  wire        rst,
    input  wire        go,              // one-clock pulse (ignore while busy)
    input  wire        wr,              // 1 = write, 0 = read
    input  wire [6:0]  addr,
    input  wire [7:0]  wr_data,
    input  wire [1:0]  burst,           // number of frames - 1
    output reg         busy,
    output reg         done,            // one-clock pulse at the end of a transfer
    output reg  [31:0] rd_data,         // byte i of a read burst in [8i+7:8i]
    output reg         SPI_CLK,
    output reg         SPI_EN,
    output reg         SPI_IN,          // MOSI
    input  wire        SPI_OUT          // MISO
);
    // ---- STUB: replace everything below with your SPI master ----------------
    initial begin busy = 1'b0; done = 1'b0; rd_data = 32'd0; SPI_CLK = 1'b0; SPI_EN = 1'b0; SPI_IN = 1'b0; end
    always @(posedge clk) begin
        SPI_CLK <= 1'b0;                // safe: bus idle
        SPI_EN  <= 1'b0;
        SPI_IN  <= 1'b0;
        busy    <= 1'b0;
        done    <= 1'b0;
    end
endmodule
