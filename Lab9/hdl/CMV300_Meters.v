`timescale 1ns / 1ps
//============================================================================
// CMV300_Meters.v - the two meters that measure the sensor's own clocking.
// GIVEN AND COMPLETE: do not edit. Both are needed from minute one to debug
// everything else, and neither depends on student logic. The row/pixel
// counters and the pixel masks are NOT here - they are yours
// (hdl/CMV300_Counters.v).
//
//   (a) CLK_OUT cycles per 1.000 ms window (25,000 +/- 16 at 25 MHz),
//       measured by counting a /16 toggle from the CLK_OUT domain.
//   (b) Frame timing with 1 us resolution, three separate intervals:
//         t_req_to_lval_us   FRAME_REQ rising edge -> first LVAL of the frame
//         t_readout_us       first LVAL rise -> last LVAL fall of the frame
//         t_period_us        frame start -> frame start (needs >= 2 frames)
//       Those three are what separate the single-frame regime (exposure,
//       then FOT, then readout, strictly in sequence: ~6.55 ms of silence
//       and 12.69 ms of readout at the defaults) from the pipelined regime
//       (the next exposure overlaps the readout: 12.88 ms frame to frame).
//
//   LVAL and FRAME_REQ are observed in the okClk domain (2-FF synchronised;
//   a row is ~26 us of LVAL - safe). A frame ends when LVAL has been low for
//   50 us (between rows it is low for one 40 ns clock; between frames for
//   the whole FOT, ~195 us). The microsecond counter adds 10/1008 of a us
//   per okClk cycle, which is exact on average at 100.8 MHz.
//============================================================================
module CMV300_Meters #(
    parameter integer MEAS_WINDOW = 100800,   // okClk cycles = 1.000 ms
    parameter integer GAP_US      = 50        // LVAL-low microseconds that end a frame
)(
    // CLK_OUT domain (only the cycle meter)
    input  wire        clk_cmv,
    // okClk domain
    input  wire        clk,
    input  wire        rst,
    input  wire        clear,                 // pulse: frame counter to 0
    input  wire        frame_req,             // the FRAME_REQ pin
    input  wire        lval,                  // LVAL pin (synchronised here)
    output reg  [15:0] frame_count,
    output reg  [23:0] t_req_to_lval_us,
    output reg  [23:0] t_readout_us,
    output reg  [23:0] t_period_us,
    output reg         frame_done_pulse,
    output reg  [31:0] clk_out_meas
);
    //------------------------------------------------------------------------
    // (a) CLK_OUT cycles per millisecond
    //------------------------------------------------------------------------
    reg [3:0] tick_cnt = 4'd0;
    reg       tick_t   = 1'b0;
    always @(posedge clk_cmv) begin
        tick_cnt <= tick_cnt + 4'd1;
        if (tick_cnt == 4'd15) tick_t <= ~tick_t;
    end
    reg [2:0]  tk_s = 3'd0;
    reg [16:0] win_cnt  = 17'd0;
    reg [15:0] edge_cnt = 16'd0;

    //------------------------------------------------------------------------
    // (b) frame timing, 1 us resolution
    //------------------------------------------------------------------------
    reg [1:0]  lv_s = 2'd0, fr_s = 2'd0;
    reg        lv_d = 1'b0, fr_d = 1'b0;
    reg [10:0] us_acc = 11'd0;                 // + 10 per clock; 1008 = one us
    reg        us_tick = 1'b0;
    reg [31:0] us_now = 32'd0;
    reg        in_frame = 1'b0, req_armed = 1'b0;
    reg [31:0] t_req = 32'd0, t_start = 32'd0, t_start_prev = 32'd0, t_last_fall = 32'd0;
    reg [7:0]  gap_us = 8'd0;

    initial begin
        frame_count = 16'd0; t_req_to_lval_us = 24'd0; t_readout_us = 24'd0; t_period_us = 24'd0;
        frame_done_pulse = 1'b0; clk_out_meas = 32'd0;
    end

    always @(posedge clk) begin
        // microsecond time base (exact on average: 10/1008 us per 9.9206 ns clock)
        if (us_acc >= 11'd998) begin us_acc <= us_acc - 11'd998; us_tick <= 1'b1; end
        else                   begin us_acc <= us_acc + 11'd10;  us_tick <= 1'b0; end
        if (us_tick) us_now <= us_now + 32'd1;

        lv_s <= {lv_s[0], lval};
        fr_s <= {fr_s[0], frame_req};
        lv_d <= lv_s[1];
        fr_d <= fr_s[1];
        frame_done_pulse <= 1'b0;

        if (fr_s[1] & ~fr_d) begin t_req <= us_now; req_armed <= 1'b1; end

        if (lv_s[1] & ~lv_d) begin                 // LVAL rise
            gap_us <= 8'd0;
            if (!in_frame) begin                   // frame start
                in_frame <= 1'b1;
                t_period_us  <= us_now - t_start;
                t_start_prev <= t_start;
                t_start      <= us_now;
                if (req_armed) begin t_req_to_lval_us <= us_now - t_req; req_armed <= 1'b0; end
            end
        end
        if (~lv_s[1] & lv_d) t_last_fall <= us_now;  // LVAL fall

        if (in_frame && !lv_s[1] && us_tick) begin
            if (gap_us == GAP_US - 1) begin        // frame end
                in_frame <= 1'b0;
                t_readout_us <= t_last_fall - t_start;
                frame_count  <= frame_count + 16'd1;
                frame_done_pulse <= 1'b1;
            end else gap_us <= gap_us + 8'd1;
        end

        // CLK_OUT meter window
        tk_s <= {tk_s[1:0], tick_t};
        if (win_cnt == MEAS_WINDOW - 1) begin
            win_cnt <= 17'd0; edge_cnt <= 16'd0;
            clk_out_meas <= {12'd0, edge_cnt, 4'd0};   // x16
        end else begin
            win_cnt <= win_cnt + 17'd1;
            if (tk_s[2] ^ tk_s[1]) edge_cnt <= edge_cnt + 16'd1;
        end

        if (clear || rst) frame_count <= 16'd0;
    end
endmodule
