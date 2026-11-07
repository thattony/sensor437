`timescale 1ns / 1ps
//============================================================================
// SensorBoard_Top.v - TOP LEVEL of Lab 9. You do not need to edit this file.
//
// OpalKelly XEM7310-A75 (Artix-7 xc7a75tfgg484-1) on the UIUC ECE 437 Sensor
// Board. Unchanged from the I2C lab except that it also instantiates the
// CMV300 subsystem (hdl/CMV300_Subsystem.v). The files YOU write in Lab 9 are
//   hdl/CMV300_Ctrl.v      (power-up sequencer + FRAME_REQ pulse generator)
//   hdl/CMV300_Spi.v       (16-bit SPI master)
//   hdl/CMV300_Counters.v  (row/pixel counters + OR/AND pixel masks)
// plus hdl/I2C_Transmit.v, which ships as a stub: paste your completed I2C
// machine from the earlier lab (its endpoints here are exactly that lab's;
// temperature.py works against the shipped reference bit file out of the box).
//
// PC <-> FPGA CONTRACT (must match python/sensor_board.py, python/cmv300.py, vivado/build.tcl)
//   WireIn  0x00  ctrl    bit0 = I2C start (rising edge), bit1 = reset (level), [11:8] = sensor-board LEDs
//   WireIn  0x01  param1  [7:0] SAD+W, [8] I2C bus select
//   WireIn  0x02  param2  [7:0] register, [15:8] write data, [16] 1 = write / 0 = read
//   WireIn  0x03  param3  [7:0] bytes to read (1..4)
//   WireIn  0x04  camera control, 0x05 SPI command    (CMV300_Subsystem.v header)
//   WireOut 0x20..0x23    I2C results / status (as in the I2C lab)
//   WireOut 0x24..0x2C    camera status / counters / timing meters (CMV300_Subsystem.v header)
//   WireOut 0x3F  design ID 0xEC437901
//   TriggerIn  0x40 I2C start/reset, 0x41 camera (spi_go, frame_req, cam_reset, clear, t_exp1)
//   TriggerOut 0x60 I2C done, 0x61 camera (spi_done, frame_done)
//
// Clocking: everything host-side runs on okClk (100.8 MHz). The 200 MHz board
// oscillator feeds the MMCM inside the subsystem (25 MHz -> ODDR -> CLK_IN);
// the meters sample on the sensor's returned CLK_OUT. There is no DRAM and no
// pixel storage anywhere in this lab.
//============================================================================
module SensorBoard_Top (
    // FrontPanel host interface (USB 3.0)
    input  wire [4:0]  okUH,
    output wire [2:0]  okHU,
    inout  wire [31:0] okUHU,
    inout  wire        okAA,

    // XEM7310 board
    input  wire        sys_clkp,          // 200 MHz LVDS oscillator (W11/W12)
    input  wire        sys_clkn,
    output wire [7:0]  led,               // XEM7310 LEDs, active LOW

    // Sensor board user I/O
    input  wire [3:0]  button,            // SW0..SW3, pressed = 0
    output wire [3:0]  s_LED,             // D0..D3, 1 = on

    // I2C bus 0 : ADT7420 (0x48), HTS221 (0x5F), LPS35HW (0x5C)
    inout  wire        I2C_SCL_0,
    inout  wire        I2C_SDA_0,
    output wire        ADT7420_A0,
    output wire        ADT7420_A1,
    input  wire        ADT7420_CT,
    input  wire        ADT7420_INT,
    output wire        HTS221_SPIenable,  // 1 = HTS221 in I2C mode
    input  wire        HTS221_DRDY,
    output wire        LPS35_CS,          // 1 = LPS35HW in I2C mode
    output wire        LPS35_SDO,         // = SA0 address bit (0 -> 0x5C)
    input  wire        LPS35_INT_DRDY,

    // I2C bus 1 : LSM303DLHC (accel 0x19, mag 0x1E), AD7156 (0x48)
    inout  wire        I2C_SCL_1,
    inout  wire        I2C_SDA_1,
    input  wire        LSM303_DRDY,
    input  wire        LSM303_INT1,
    input  wire        LSM303_INT2,
    input  wire        AD7156_OUT1,
    input  wire        AD7156_OUT2,

    // CMV300 image sensor (labelled CVM300 on the board): SPI control + status pins
    output wire        CVM300_SPI_EN,
    output wire        CVM300_SPI_CLK,
    output wire        CVM300_SPI_IN,     // FPGA -> sensor (MOSI)
    input  wire        CVM300_SPI_OUT,    // sensor -> FPGA (MISO)
    output wire        CVM300_CLK_IN,     // sensor master clock, from the FPGA (25 MHz)
    input  wire        CVM300_CLK_OUT,    // pixel clock, from the sensor
    output wire        CVM300_SYS_RES_N,
    output wire        CVM300_Enable_LVDS,
    output wire        CVM300_FRAME_REQ,
    output wire        CVM300_T_EXP1,
    output wire        CVM300_T_EXP2,
    input  wire [9:0]  CVM300_D,          // parallel video - measured by counters only in this lab
    input  wire        CVM300_Line_valid,
    input  wire        CVM300_Data_valid
);
    //------------------------------------------------------------------------
    // Clocks
    //------------------------------------------------------------------------
    wire okClk;                       // 100.8 MHz from the FrontPanel IP - the design clock
    wire clk200_i, clk200;            // 200 MHz board oscillator -> the subsystem's MMCM
    IBUFGDS ibuf_sysclk (.I(sys_clkp), .IB(sys_clkn), .O(clk200_i));
    BUFG    u_bufg_200  (.I(clk200_i), .O(clk200));

    //------------------------------------------------------------------------
    // FrontPanel endpoint wires
    //------------------------------------------------------------------------
    wire [31:0] wi00, wi01, wi02, wi03, wi04, wi05;
    wire [31:0] wo20, wo21, wo22, wo23, wo24, wo25, wo26, wo27, wo28, wo29, wo2a, wo2b, wo2c, wo3f;
    wire [31:0] ti40, ti41, to60, to61;

    //------------------------------------------------------------------------
    // Safe defaults: every I2C sensor idles quiet and in I2C mode
    //------------------------------------------------------------------------
    assign ADT7420_A0       = 1'b0;   // address 0x48
    assign ADT7420_A1       = 1'b0;
    assign HTS221_SPIenable = 1'b1;   // HTS221 stays in I2C mode
    assign LPS35_CS         = 1'b1;   // LPS35HW stays in I2C mode
    assign LPS35_SDO        = 1'b0;   // address 0x5C

    //------------------------------------------------------------------------
    // Start / reset conditioning (all in the okClk domain)
    //------------------------------------------------------------------------
    reg  ctrl_start_d = 1'b0;
    reg  done_d       = 1'b0;
    wire i2c_done;
    always @(posedge okClk) begin
        ctrl_start_d <= wi00[0];
        done_d       <= i2c_done;
    end
    wire start = (wi00[0] & ~ctrl_start_d) | ti40[0];   // one-clock pulse
    wire rst   =  wi00[1] | ti40[1];                     // level (wire) or pulse (trigger)

    //------------------------------------------------------------------------
    // I2C buses: open-drain drivers + bus selector (exactly the I2C lab)
    //------------------------------------------------------------------------
    wire i2c_scl_low, i2c_sda_low;
    reg  bus_sel = 1'b0;
    reg  i2c0_scl_low = 1'b0, i2c0_sda_low = 1'b0, i2c1_scl_low = 1'b0, i2c1_sda_low = 1'b0;
    always @(posedge okClk) begin
        i2c0_scl_low <= bus_sel ? 1'b0 : i2c_scl_low;
        i2c0_sda_low <= bus_sel ? 1'b0 : i2c_sda_low;
        i2c1_scl_low <= bus_sel ? i2c_scl_low : 1'b0;
        i2c1_sda_low <= bus_sel ? i2c_sda_low : 1'b0;
    end
    assign I2C_SCL_0 = i2c0_scl_low ? 1'b0 : 1'bz;
    assign I2C_SDA_0 = i2c0_sda_low ? 1'b0 : 1'bz;
    assign I2C_SCL_1 = i2c1_scl_low ? 1'b0 : 1'bz;
    assign I2C_SDA_1 = i2c1_sda_low ? 1'b0 : 1'bz;
    wire i2c_scl_in = bus_sel ? I2C_SCL_1 : I2C_SCL_0;
    wire i2c_sda_in = bus_sel ? I2C_SDA_1 : I2C_SDA_0;

    wire [3:0]  i2c_acks;
    wire        i2c_busy, i2c_err;
    wire [7:0]  i2c_state;
    wire [31:0] i2c_rx_data;

    I2C_Transmit #(.TICK_DIVIDE(252)) u_i2c (
        .clk(okClk), .rst(rst), .start(start),
        .tx_byte(wi01[7:0]), .param2(wi02), .param3(wi03),
        .scl_low(i2c_scl_low), .sda_low(i2c_sda_low), .scl_in(i2c_scl_in), .sda_in(i2c_sda_in),
        .ACK_frames(i2c_acks), .rx_data(i2c_rx_data),
        .busy(i2c_busy), .done(i2c_done), .error_bit(i2c_err), .State(i2c_state)
    );

    //------------------------------------------------------------------------
    // The camera subsystem - hdl/CMV300_Subsystem.v (contains YOUR three modules)
    //------------------------------------------------------------------------
    CMV300_Subsystem u_cam (
        .clk(okClk), .rst(rst), .clk200(clk200),
        .clk_cmv(CVM300_CLK_OUT), .d(CVM300_D), .lval(CVM300_Line_valid), .dval(CVM300_Data_valid),
        .spi_out(CVM300_SPI_OUT), .spi_clk(CVM300_SPI_CLK), .spi_en(CVM300_SPI_EN), .spi_in(CVM300_SPI_IN),
        .cvm_clk_in(CVM300_CLK_IN), .sys_res_n(CVM300_SYS_RES_N), .enable_lvds(CVM300_Enable_LVDS),
        .frame_req(CVM300_FRAME_REQ), .t_exp1(CVM300_T_EXP1), .t_exp2(CVM300_T_EXP2),
        .wi04(wi04), .wi05(wi05), .ti41(ti41), .to61(to61),
        .wo24(wo24), .wo25(wo25), .wo26(wo26), .wo27(wo27), .wo28(wo28), .wo29(wo29), .wo2a(wo2a), .wo2b(wo2b), .wo2c(wo2c)
    );

    //------------------------------------------------------------------------
    // Results, status, LEDs (exactly the I2C lab)
    //------------------------------------------------------------------------
    reg  [7:0]  byte_sent = 8'h00;
    reg  [31:0] cycles    = 32'd0;
    reg  [25:0] blink_cnt = 26'd0;
    reg         blink     = 1'b0;

    always @(posedge okClk) begin
        if (blink_cnt == 26'd50_399_999) begin blink_cnt <= 26'd0; blink <= ~blink; end
        else                                blink_cnt <= blink_cnt + 1'b1;
        if (rst) begin
            bus_sel   <= 1'b0;
            byte_sent <= 8'h00;
            cycles    <= 32'd0;
        end else if (start && !i2c_busy) begin
            bus_sel   <= wi01[8];
            byte_sent <= wi01[7:0];
            cycles    <= 32'd0;
        end else if (i2c_busy) begin
            cycles    <= cycles + 1'b1;
        end
    end

    wire error = i2c_err | (i2c_done & |i2c_acks);

    assign wo20 = {15'd0, bus_sel, byte_sent, 4'd0, i2c_acks};
    assign wo21 = i2c_rx_data;
    assign wo22 = cycles;
    assign wo23 = {12'd0, ~button, i2c_state, 5'd0, error, i2c_done, i2c_busy};
    assign wo3f = 32'hEC43_7901;                            // design ID: Lab 9 control-plane package
    assign to60 = {31'd0, i2c_done & ~done_d};
    assign s_LED = wi00[11:8];
    assign led   = ~{blink, error, i2c_done & ~|i2c_acks, i2c_busy, ~button};

    //------------------------------------------------------------------------
    // FrontPanel Subsystem IP (endpoint set configured in vivado/build.tcl)
    //------------------------------------------------------------------------
    frontpanel_0 frontpanel_inst (
        .okUH             (okUH),
        .okHU             (okHU),
        .okUHU            (okUHU),
        .okAA             (okAA),
        .okClk            (okClk),
        .wi00_ep_dataout  (wi00),
        .wi01_ep_dataout  (wi01),
        .wi02_ep_dataout  (wi02),
        .wi03_ep_dataout  (wi03),
        .wi04_ep_dataout  (wi04),
        .wi05_ep_dataout  (wi05),
        .ti40_ep_clk      (okClk),
        .ti40_ep_trigger  (ti40),
        .ti41_ep_clk      (okClk),
        .ti41_ep_trigger  (ti41),
        .wo20_ep_datain   (wo20),
        .wo21_ep_datain   (wo21),
        .wo22_ep_datain   (wo22),
        .wo23_ep_datain   (wo23),
        .wo24_ep_datain   (wo24),
        .wo25_ep_datain   (wo25),
        .wo26_ep_datain   (wo26),
        .wo27_ep_datain   (wo27),
        .wo28_ep_datain   (wo28),
        .wo29_ep_datain   (wo29),
        .wo2a_ep_datain   (wo2a),
        .wo2b_ep_datain   (wo2b),
        .wo2c_ep_datain   (wo2c),
        .wo3f_ep_datain   (wo3f),
        .to60_ep_clk      (okClk),
        .to60_ep_trigger  (to60),
        .to61_ep_clk      (okClk),
        .to61_ep_trigger  (to61)
    );

endmodule
