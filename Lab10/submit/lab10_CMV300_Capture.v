`timescale 1ns / 1ps
//============================================================================
// CMV300_Capture.v - the four-pixel packer (Lab 10).
//
//   In  (CLK_OUT rising-edge domain, registered by CMV300_CaptureCore):
//     pixel, pixel_valid (LVAL & DVAL), line_valid (LVAL), stream_en, frame_end
//   Out (the frozen contract, INTERFACE.md):
//     pix_data[47:0] = {px3, px2, px1, px0}, 12-bit lanes, px0 = FIRST pixel,
//                      a 10-bit pixel sits in the low bits: {2'b00, pixel}
//     pix_valid      one cycle when the fourth pixel of a group arrives
//     pix_eof        one cycle on frame_end, only while stream_en is high
//
//   Row start: the position counter is held at 0 for as long as line_valid is
//   low, so every row starts at lane 0. It is NOT cleared on the rising edge of
//   line_valid: on the test pattern LVAL and DVAL rise together, and a clear
//   there would land on the first pixel of the row and swallow it.
//
//   Mid-row DVAL-low slot (pixel 324 | invalid | 325): line_valid stays high,
//   pixel_valid is low for one cycle, so the packer simply waits - nothing is
//   cleared and nothing is pushed. 648 = 4 x 162: a row never ends mid-entry.
//============================================================================
module CMV300_Capture (
    input  wire        clk,            // CLK_OUT, rising edge
    input  wire [9:0]  pixel,
    input  wire        pixel_valid,
    input  wire        line_valid,
    input  wire        stream_en,
    input  wire        frame_end,
    output reg         pix_valid,
    output reg  [47:0] pix_data,
    output reg         pix_eof
);

    reg [1:0]  pk4     = 2'd0;         // pixels collected so far in this group, 0..3
    reg [35:0] pk4_buf = 36'd0;        // the first three pixels, lanes {px2, px1, px0}

    initial begin pix_valid = 1'b0; pix_data = 48'd0; pix_eof = 1'b0; end

    always @(posedge clk) begin
        pix_valid <= 1'b0;             // one-cycle strobes: default low
        pix_eof   <= 1'b0;

        if (!line_valid) begin
            pk4 <= 2'd0;               // between rows: hold the packer at lane 0
        end else if (pixel_valid && stream_en) begin
            case (pk4)
                2'd0: pk4_buf[11:0]  <= {2'b00, pixel};
                2'd1: pk4_buf[23:12] <= {2'b00, pixel};
                2'd2: pk4_buf[35:24] <= {2'b00, pixel};
                2'd3: begin
                    pix_data  <= {2'b00, pixel, pk4_buf};   // {px3, px2, px1, px0}
                    pix_valid <= 1'b1;
                end
            endcase
            pk4 <= pk4 + 2'd1;         // 3 wraps to 0: the next group starts at lane 0
        end

        if (frame_end && stream_en)
            pix_eof <= 1'b1;           // end-of-frame token, gated by the acquisition
    end

endmodule
