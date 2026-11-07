`timescale 1ns / 1ps
//============================================================================
// CMV300_Counters.v - THIS IS A FILE YOU WRITE (3 of 3). Everything that
// watches the pixel bus: how many rows a frame has, how many valid pixels a
// row carries, and the running bitwise OR and AND of every pixel of a frame
// (a stuck data line shows up there immediately).
//
// WHAT IT MUST DO (CMV300.pdf section 4.2.1, Figures 26-27):
//   * Sample D[9:0], LVAL and DVAL on the FALLING edge of CLK_OUT - the
//     datasheet says the outputs change on the rising edge.
//   * lval_edges: cumulative count of LVAL rising edges (cleared by `clear`).
//   * last_row_pixels: the number of CLK_OUT cycles with DVAL high during the
//     last completed LVAL-high period. Count what DVAL actually says - study
//     Figure 26 closely before assuming anything about the shape of a row.
//   * last_frame_rows: LVAL rising edges of the last completed frame. A frame
//     has ended when LVAL stays low for GAP CLK_OUT cycles (between rows it
//     is low for a single clock; between frames for thousands).
//   * or_mask / and_mask: bitwise OR / AND of every valid pixel of the last
//     completed frame (initialise to 0 / 0x3FF at the frame start).
//   * The okClk-side outputs must be handed across the clock domains so the
//     host never reads a torn value (hand them over at a frame end with a
//     toggle synchroniser; `clear` comes from okClk - synchronise it too).
//
// The PORT LIST IS FROZEN (INTERFACE.md). This stub compiles, drives zeros,
// and fails the testbench with named checks.
//============================================================================
module CMV300_Counters #(
    parameter integer GAP = 64                // LVAL-low CLK_OUT cycles that end a frame
)(
    // sensor (CLK_OUT domain)
    input  wire        clk_cmv,
    input  wire [9:0]  d,
    input  wire        lval,
    input  wire        dval,
    // host (okClk domain)
    input  wire        clk,
    input  wire        rst,
    input  wire        clear,                 // pulse: lval_edges to 0
    output reg  [31:0] lval_edges,
    output reg  [15:0] last_row_pixels,
    output reg  [15:0] last_frame_rows,
    output reg  [9:0]  or_mask,
    output reg  [9:0]  and_mask
);
    // ---- STUB: replace everything below with your counters -------------------
    initial begin
        lval_edges = 32'd0; last_row_pixels = 16'd0; last_frame_rows = 16'd0;
        or_mask = 10'd0; and_mask = 10'd0;
    end
    always @(posedge clk) begin
        lval_edges      <= 32'd0;
        last_row_pixels <= 16'd0;
        last_frame_rows <= 16'd0;
        or_mask         <= 10'd0;
        and_mask        <= 10'd0;
    end
endmodule
