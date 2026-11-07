`timescale 1ns / 1ps
//============================================================================
// CMV300_Subsystem.v - everything CMV300 of Lab 9 in one box, instantiated by
// SensorBoard_Top.v next to the I2C machine. GIVEN: do not edit. The three
// modules students write are CMV300_Ctrl.v, CMV300_Spi.v and
// CMV300_Counters.v (see INTERFACE.md).
//
//   Contains: the MMCM that makes the 25 MHz sensor master clock from the
//   200 MHz board oscillator, the ODDR that forwards it to CVM300_CLK_IN
//   (gated by the sequencer's clk_en), the three student modules (u_ctrl,
//   u_spi, u_counters) and the given timing meters (u_meters).
//
// PC <-> FPGA CONTRACT of the camera (I2C endpoints unchanged, top-level header)
//   WireIn  0x04  [0] cam_enable (level: clock on, reset released -> "ready")
//   WireIn  0x05  [7:0] SPI data to write, [14:8] register address,
//                 [15] 1 = write / 0 = read, [17:16] burst = frames - 1
//   TriggerIn  0x41 [0] spi_go, [1] frame_req, [4] cam_reset, [5] clear counters, [8] t_exp1
//   WireOut 0x24  [0] clk_locked, [1] sensor_ready, [3] spi_busy, [10] FRAME_REQ pin (live),
//                 [11] cam_enable echo, [23:16] SPI read byte 0, [27:24] sequencer state
//                 (0 OFF, 1 CLK_ON, 2 RES_WAIT, 3 READY, 4 RES_LOW)
//   WireOut 0x25  [15:0] frame counter (meters), [31:16] LVAL edge counter low 16 bits (counters)
//   WireOut 0x26  [15:0] DVAL-high cycles in the last row (648), [31:16] rows in the last frame (488)
//   WireOut 0x27  CLK_OUT cycles per 1.000 ms (25,000 +/- 16 at 25 MHz)
//   WireOut 0x28  SPI read data, byte i of a burst in [8i+7:8i]
//   WireOut 0x29  frame-to-frame period in us (needs >= 2 frames)
//   WireOut 0x2A  [9:0] OR of every pixel of the last frame, [25:16] AND of every pixel
//   WireOut 0x2B  FRAME_REQ -> first LVAL in us (the exposure + FOT silence of a single frame)
//   WireOut 0x2C  first LVAL rise -> last LVAL fall in us (the readout)
//   TriggerOut 0x61 [0] spi_done, [1] frame_done
//============================================================================
module CMV300_Subsystem (
    input  wire        clk,                // okClk 100.8 MHz
    input  wire        rst,
    input  wire        clk200,             // 200 MHz board oscillator
    // sensor pins
    input  wire        clk_cmv,            // CVM300_CLK_OUT
    input  wire [9:0]  d,
    input  wire        lval,
    input  wire        dval,
    input  wire        spi_out,
    output wire        spi_clk,
    output wire        spi_en,
    output wire        spi_in,
    output wire        cvm_clk_in,         // CVM300_CLK_IN (ODDR output)
    output wire        sys_res_n,
    output wire        enable_lvds,
    output wire        frame_req,
    output wire        t_exp1,
    output wire        t_exp2,
    // host endpoints
    input  wire [31:0] wi04,
    input  wire [31:0] wi05,
    input  wire [31:0] ti41,
    output wire [31:0] to61,
    output wire [31:0] wo24,
    output wire [31:0] wo25,
    output wire [31:0] wo26,
    output wire [31:0] wo27,
    output wire [31:0] wo28,
    output wire [31:0] wo29,
    output wire [31:0] wo2a,
    output wire [31:0] wo2b,
    output wire [31:0] wo2c
);
    //------------------------------------------------------------------------
    // 200 MHz -> 25 MHz sensor master clock (VCO = 200 x 5 = 1000, /40 = 25)
    //------------------------------------------------------------------------
    wire clk25_unbuf, clk25, mmcm_fb_unbuf, mmcm_fb, mmcm_locked;
    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"), .CLKFBOUT_MULT_F(5.0), .CLKFBOUT_PHASE(0.0), .CLKIN1_PERIOD(5.000),
        .CLKOUT0_DIVIDE_F(40.0), .CLKOUT0_DUTY_CYCLE(0.5), .CLKOUT0_PHASE(0.0),
        .CLKOUT1_DIVIDE(1), .CLKOUT2_DIVIDE(1), .CLKOUT3_DIVIDE(1), .CLKOUT4_DIVIDE(1),
        .CLKOUT5_DIVIDE(1), .CLKOUT6_DIVIDE(1), .CLKOUT4_CASCADE("FALSE"),
        .DIVCLK_DIVIDE(1), .REF_JITTER1(0.010), .STARTUP_WAIT("FALSE")
    ) u_mmcm (
        .CLKOUT0(clk25_unbuf), .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
        .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .CLKFBOUT(mmcm_fb_unbuf), .CLKFBOUTB(), .LOCKED(mmcm_locked),
        .CLKIN1(clk200), .PWRDWN(1'b0), .RST(1'b0), .CLKFBIN(mmcm_fb)
    );
    BUFG u_bufg_fb (.I(mmcm_fb_unbuf), .O(mmcm_fb));
    BUFG u_bufg_25 (.I(clk25_unbuf),   .O(clk25));

    // Clock forwarding: CVM300_CLK_IN = clk25 while the sequencer enables it
    wire      clk_en;
    reg [1:0] clk_en_s = 2'b00;
    always @(posedge clk25) clk_en_s <= {clk_en_s[0], clk_en};
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr (
        .Q(cvm_clk_in), .C(clk25), .CE(1'b1), .D1(clk_en_s[1]), .D2(1'b0), .R(1'b0), .S(1'b0)
    );

    //------------------------------------------------------------------------
    // The student modules
    //------------------------------------------------------------------------
    reg [1:0] lock_s = 2'b00;
    always @(posedge clk) lock_s <= {lock_s[0], mmcm_locked};

    wire       sensor_ready;
    wire [3:0] ctrl_state;
    CMV300_Ctrl u_ctrl (
        .clk          (clk),
        .cam_enable   (wi04[0]),
        .cam_reset    (ti41[4]),
        .clk_locked   (lock_s[1]),
        .frame_req_in (ti41[1]),
        .t_exp_in     (ti41[8]),
        .clk_en       (clk_en),
        .sys_res_n    (sys_res_n),
        .frame_req    (frame_req),
        .t_exp1       (t_exp1),
        .sensor_ready (sensor_ready),
        .state        (ctrl_state)
    );
    assign enable_lvds = 1'b0;             // pin B2: parallel CMOS output (LVDS is a later lab)
    assign t_exp2      = 1'b0;

    wire        spi_busy, spi_done;
    wire [31:0] spi_rd;
    CMV300_Spi #(.HALF(5)) u_spi (
        .clk(clk), .rst(rst), .go(ti41[0]), .wr(wi05[15]), .addr(wi05[14:8]), .wr_data(wi05[7:0]),
        .burst(wi05[17:16]), .busy(spi_busy), .done(spi_done), .rd_data(spi_rd),
        .SPI_CLK(spi_clk), .SPI_EN(spi_en), .SPI_IN(spi_in), .SPI_OUT(spi_out)
    );

    //------------------------------------------------------------------------
    // The given timing meters + YOUR pixel-bus counters
    //------------------------------------------------------------------------
    wire [15:0] frame_count, last_row_pixels, last_frame_rows;
    wire [31:0] lval_edges, clk_out_meas;
    wire [23:0] t_req_to_lval_us, t_readout_us, t_period_us;
    wire [9:0]  or_mask, and_mask;
    wire        frame_done_pulse;
    CMV300_Meters #(.MEAS_WINDOW(100800), .GAP_US(50)) u_meters (
        .clk_cmv(clk_cmv),
        .clk(clk), .rst(rst), .clear(ti41[5]),
        .frame_req(frame_req), .lval(lval),
        .frame_count(frame_count),
        .t_req_to_lval_us(t_req_to_lval_us), .t_readout_us(t_readout_us), .t_period_us(t_period_us),
        .frame_done_pulse(frame_done_pulse), .clk_out_meas(clk_out_meas)
    );
    CMV300_Counters #(.GAP(64)) u_counters (
        .clk_cmv(clk_cmv), .d(d), .lval(lval), .dval(dval),
        .clk(clk), .rst(rst), .clear(ti41[5]),
        .lval_edges(lval_edges), .last_row_pixels(last_row_pixels), .last_frame_rows(last_frame_rows),
        .or_mask(or_mask), .and_mask(and_mask)
    );

    //------------------------------------------------------------------------
    // Status words
    //------------------------------------------------------------------------
    assign wo24 = {4'd0, ctrl_state, spi_rd[7:0], 4'd0, wi04[0], frame_req, 6'd0, spi_busy, 1'b0, sensor_ready, lock_s[1]};
    assign wo25 = {lval_edges[15:0], frame_count};
    assign wo26 = {last_frame_rows, last_row_pixels};
    assign wo27 = clk_out_meas;
    assign wo28 = spi_rd;
    assign wo29 = {8'd0, t_period_us};
    assign wo2a = {6'd0, and_mask, 6'd0, or_mask};
    assign wo2b = {8'd0, t_req_to_lval_us};
    assign wo2c = {8'd0, t_readout_us};
    assign to61 = {30'd0, frame_done_pulse, spi_done};
endmodule
