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
// The PORT LIST IS FROZEN (INTERFACE.md).
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
    //========================================================================
    // `clear` crossing okClk -> CLK_OUT: a pulse is too short to sample in the
    // other domain, so turn it into a level change (toggle) and detect the
    // change over there. The toggle also survives CLK_OUT being stopped.
    //========================================================================
    reg clear_t = 1'b0;
    always @(posedge clk) if (clear) clear_t <= ~clear_t;

    (* ASYNC_REG = "TRUE" *) reg [2:0] clear_s = 3'd0;
    wire clear_cmv = clear_s[2] ^ clear_s[1];

    //========================================================================
    // CLK_OUT domain - everything is sampled on the FALLING edge (the sensor
    // changes D/LVAL/DVAL on the rising edge, Figure 26)
    //========================================================================
    reg        lval_q   = 1'b0;           // LVAL one sample ago (edge detector)
    reg        in_frame = 1'b0;           // between the first LVAL rise and the GAP timeout
    reg [15:0] gap_cnt  = 16'd0;          // consecutive LVAL-low samples
    reg [31:0] edges    = 32'd0;          // cumulative LVAL rising edges
    reg [15:0] pix_cnt  = 16'd0;          // DVAL-high samples in the row being received
    reg [15:0] row_pix  = 16'd0;          // ... of the last completed row
    reg [15:0] row_cnt  = 16'd0;          // rows of the frame being received
    reg [9:0]  or_acc   = 10'd0;
    reg [9:0]  and_acc  = 10'h3FF;

    // Snapshot taken at the frame end. It then stays still for a whole frame,
    // so okClk can copy it safely once the toggle has crossed.
    reg [31:0] edges_h   = 32'd0;
    reg [15:0] row_pix_h = 16'd0;
    reg [15:0] rows_h    = 16'd0;
    reg [9:0]  or_h      = 10'd0;
    reg [9:0]  and_h     = 10'd0;
    reg        xfer_t    = 1'b0;

    wire lval_rise = lval & ~lval_q;
    wire lval_fall = ~lval & lval_q;
    // A pixel is valid only while BOTH are high. DVAL drops for one cycle in
    // the middle of every row (after pixel 324) while LVAL stays high: gating
    // with LVAL alone would count 649 and fold that slot into the masks.
    wire pix_valid = lval & dval;

    always @(negedge clk_cmv) begin
        clear_s <= {clear_s[1:0], clear_t};
        lval_q  <= lval;

        // ---- rows -------------------------------------------------------------
        if (lval_rise) begin
            edges <= edges + 32'd1;
            if (!in_frame) begin
                in_frame <= 1'b1;         // first row of a new frame
                row_cnt  <= 16'd1;
            end else
                row_cnt  <= row_cnt + 16'd1;
        end

        // ---- pixels in a row: restart at the LVAL rise, accumulate ACROSS the
        //      invalid slot, latch the total at the LVAL fall --------------------
        if (lval_rise)      pix_cnt <= pix_valid ? 16'd1 : 16'd0;
        else if (pix_valid) pix_cnt <= pix_cnt + 16'd1;
        if (lval_fall)      row_pix <= pix_cnt;

        // ---- masks: 0 / 0x3FF at the frame start, then every valid pixel ---------
        if (lval_rise && !in_frame) begin
            or_acc  <= pix_valid ? d : 10'd0;
            and_acc <= pix_valid ? d : 10'h3FF;
        end else if (pix_valid) begin
            or_acc  <= or_acc  | d;
            and_acc <= and_acc & d;
        end

        // ---- frame end: LVAL low for GAP samples in a row (1 between rows) -------
        if (lval)
            gap_cnt <= 16'd0;
        else if (in_frame) begin
            if (gap_cnt == GAP - 1) begin
                in_frame  <= 1'b0;
                gap_cnt   <= 16'd0;
                edges_h   <= edges;
                row_pix_h <= row_pix;
                rows_h    <= row_cnt;
                or_h      <= or_acc;
                and_h     <= and_acc;
                xfer_t    <= ~xfer_t;     // tell okClk a new snapshot is ready
            end else
                gap_cnt <= gap_cnt + 16'd1;
        end

        if (clear_cmv) edges <= 32'd0;    // last: clear wins
    end

    //========================================================================
    // okClk domain: copy the snapshot when the toggle arrives (3 flops later
    // the *_h registers have been stable for many CLK_OUT cycles)
    //========================================================================
    (* ASYNC_REG = "TRUE" *) reg [2:0] xfer_s = 3'd0;

    initial begin
        lval_edges = 32'd0; last_row_pixels = 16'd0; last_frame_rows = 16'd0;
        or_mask = 10'd0; and_mask = 10'd0;
    end

    always @(posedge clk) begin
        xfer_s <= {xfer_s[1:0], xfer_t};
        if (xfer_s[2] ^ xfer_s[1]) begin
            lval_edges      <= edges_h;
            last_row_pixels <= row_pix_h;
            last_frame_rows <= rows_h;
            or_mask         <= or_h;
            and_mask        <= and_h;
        end
        if (clear) lval_edges <= 32'd0;   // host sees 0 at once, even if no frame follows
        if (rst) begin
            lval_edges      <= 32'd0;
            last_row_pixels <= 16'd0;
            last_frame_rows <= 16'd0;
            or_mask         <= 10'd0;
            and_mask        <= 10'd0;
        end
    end
endmodule
