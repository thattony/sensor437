`timescale 1ns / 1ps
//============================================================================
// CMV300_Ctrl.v - THIS IS A FILE YOU WRITE (1 of 3). Power-up / reset
// sequencer and FRAME_REQ pulse generator for the CMV300.
//
// WHAT IT MUST DO (CMV300.pdf section 3.7 Figures 4-5, section 3.8 Figure 6):
//   * Start-up, when cam_enable = 1 and clk_locked = 1:
//       clk_en = 1 (CLK_IN starts) -> wait >= 1 us -> sys_res_n = 1 -> wait
//       >= 1 us -> READY (sensor_ready = 1; SPI traffic is allowed only now).
//   * Reset (cam_enable falls, cam_reset pulses, or the clock unlocks):
//       sys_res_n low WITH THE CLOCK STILL RUNNING for >= 1 us (the falling
//       edge restores every register default), then clk_en = 0. If cam_enable
//       is still high after cam_reset, come back up again.
//   * FRAME_REQ: ONE pulse of at least 4 CLK_IN periods (>= 160 ns at 25 MHz)
//     per frame_req_in trigger, only while READY. frame_req_in may arrive as
//     a level - you must NOT pass a level through: edge-detect it, or the
//     sensor free-runs (the testbench checks exactly this). Same for t_exp1.
//   * state must follow your state register so the host can see where a stuck
//     sequencer sits: 0 OFF, 1 CLK_ON, 2 RES_WAIT, 3 READY, 4 RES_LOW.
//
// The PORT LIST IS FROZEN - the wrapper, the testbench and the Python all
// depend on it (INTERFACE.md). This stub compiles, drives every output to its
// safe inactive level, and fails the testbench with named checks.
//============================================================================
module CMV300_Ctrl #(
    parameter integer T_WAIT = 2016,    // suggested: 20 us at 100.8 MHz (20x the minimum)
    parameter integer T_REQ  = 32       // suggested: FRAME_REQ pulse width in okClk cycles (317 ns)
)(
    input  wire       clk,              // okClk, 100.8 MHz
    input  wire       cam_enable,       // level: 1 = bring the sensor up and keep it up
    input  wire       cam_reset,        // pulse: full reset sequence (then back up if cam_enable)
    input  wire       clk_locked,       // the MMCM that generates CLK_IN is locked
    input  wire       frame_req_in,     // trigger: request one frame sequence (edge-detect this!)
    input  wire       t_exp_in,         // trigger: T_EXP1 external-exposure pulse (edge-detect this!)
    output reg        clk_en,           // 1 = CLK_IN runs (gates the ODDR in the wrapper)
    output reg        sys_res_n,        // CVM300_SYS_RES_N (0 = sensor in reset)
    output reg        frame_req,        // CVM300_FRAME_REQ pin
    output reg        t_exp1,           // CVM300_T_EXP1 pin
    output wire       sensor_ready,     // 1 = start-up complete, SPI allowed
    output wire [3:0] state             // your state register (WireOut 0x24 [27:24])
);
    //========================================================================
    // State encoding - these numbers are what the host sees on WireOut 0x24
    //========================================================================
    localparam [3:0] S_OFF      = 4'd0,   // no clock, sensor in reset
                     S_CLK_ON   = 4'd1,   // clock running, still in reset, waiting T_WAIT
                     S_RES_WAIT = 4'd2,   // reset released, waiting T_WAIT before SPI
                     S_READY    = 4'd3,   // start-up complete: SPI + FRAME_REQ allowed
                     S_RES_LOW  = 4'd4;   // going down: reset low, clock STILL on, waiting T_WAIT

    reg [3:0]  st  = S_OFF;               // the state register
    reg [11:0] cnt = 12'd0;               // wait counter (2^12 = 4096 > T_WAIT = 2016)

    // "should we be up?" - any of these false means go down / stay down
    wire want_up = cam_enable & clk_locked;

    initial begin clk_en = 1'b0; sys_res_n = 1'b0; frame_req = 1'b0; t_exp1 = 1'b0; end

    //========================================================================
    // Power-up / reset sequencer
    //========================================================================
    always @(posedge clk) begin
        case (st)
            S_OFF: begin
                clk_en    <= 1'b0;          // no clock
                sys_res_n <= 1'b0;          // sensor in reset
                if (want_up) begin
                    cnt <= 0;
                    st  <= S_CLK_ON;
                end
            end

            S_CLK_ON: begin
                clk_en    <= 1'b1;          // clock runs in this state no matter what
                sys_res_n <= 1'b0;          // still in reset
                if (!want_up) begin
                    cnt <= 0;
                    st  <= S_RES_LOW;       // clock is on -> proper reset-low, then off
                end else if (cnt == T_WAIT-1) begin
                    cnt <= 0;
                    st  <= S_RES_WAIT;
                end else
                    cnt <= cnt + 1;
            end

            S_RES_WAIT: begin
                clk_en    <= 1'b1;          // clock runs in this state no matter what
                sys_res_n <= 1'b1;          // reset released
                if (!want_up || cam_reset) begin
                    cnt <= 0;
                    st  <= S_RES_LOW;       // clock is on -> proper reset-low, then off
                end else if (cnt == T_WAIT-1) begin
                    cnt <= 0;
                    st  <= S_READY;
                end else
                    cnt <= cnt + 1;
            end

            S_READY: begin
                clk_en    <= 1'b1;          // clock runs in this state no matter what
                sys_res_n <= 1'b1;
                // no timer: stay here until told to go down
                if (!want_up || cam_reset) begin
                    cnt <= 0;
                    st  <= S_RES_LOW;       // clock is on -> proper reset-low, then off
                end
            end

            S_RES_LOW: begin
                clk_en    <= 1'b1;          // clock runs in this state no matter what
                sys_res_n <= 1'b0;          // reset low, clock still running
                if (cnt == T_WAIT-1) begin
                    cnt <= 0;
                    if (want_up) st <= S_CLK_ON;   // was a cam_reset: come back up
                    else         st <= S_OFF;      // was a shutdown: stop the clock
                end else
                    cnt <= cnt + 1;
            end

            default: st <= S_OFF;         // illegal state: recover safely
        endcase
    end

    assign sensor_ready = (st == S_READY);
    assign state        = st;

    //========================================================================
    // FRAME_REQ pulse generator: one rising edge in -> one T_REQ-cycle pulse out
    //========================================================================
    reg        fr_prev = 1'b0;            // frame_req_in last cycle (edge detector)
    reg [7:0]  fr_cnt  = 8'd0;            // pulse-width counter (2^8 = 256 > T_REQ = 32)
    wire       fr_rise = frame_req_in & ~fr_prev;

    always @(posedge clk) begin
        fr_prev <= frame_req_in;
        if (!frame_req) begin                   // pulse OFF: wait for a trigger
            if (fr_rise && sensor_ready) begin
                frame_req <= 1'b1;              // start the pulse
                fr_cnt    <= 8'd0;              // count from 0
            end
        end else begin                          // pulse ON: time it
            if (fr_cnt == T_REQ-1)
                frame_req <= 1'b0;              // T_REQ cycles done -> end
            else
                fr_cnt <= fr_cnt + 1;
        end
    end

    //========================================================================
    // T_EXP1 pulse generator: same pattern, driven by t_exp_in
    //========================================================================
    reg        te_prev = 1'b0;
    reg [7:0]  te_cnt  = 8'd0;
    wire       te_rise = t_exp_in & ~te_prev;

    always @(posedge clk) begin
        te_prev <= t_exp_in;
        if (!t_exp1) begin                      // pulse OFF: wait for a trigger
            if (te_rise && sensor_ready) begin
                t_exp1 <= 1'b1;                 // start the pulse
                te_cnt <= 8'd0;                 // count from 0
            end
        end else begin                          // pulse ON: time it
            if (te_cnt == T_REQ-1)
                t_exp1 <= 1'b0;                 // T_REQ cycles done -> end
            else
                te_cnt <= te_cnt + 1;
        end
    end
endmodule
