`timescale 1ns / 1ps
//============================================================================
// SensorBoard_Top.v - TOP LEVEL. You do not need to edit this file.
//
// OpalKelly XEM7310-A75 (Artix-7 xc7a75tfgg484-1) on the UIUC ECE 437
// Sensor Board. This file owns everything that is fixed by the hardware:
//   * the FrontPanel Subsystem IP (PC <-> FPGA endpoints, 100.8 MHz okClk),
//   * every pin of every sensor and the level each one must idle at,
//   * the open-drain drivers of the two I2C buses and the bus selector,
//   * the start/reset conditioning and the result/status words,
//   * the 25 MHz master clock of the CMV300 (MMCM + ODDR), the DDR3 controller
//     (MIG, vivado/mig_xem7310_a75.prj) and the camera subsystem instance (u_cam).
// The I2C protocol state machine lives in hdl/I2C_Transmit.v - THAT is the
// file you edit for the I2C exercise. The camera lives in hdl/CMV300_*.v
// (docs/CMV300_CAPTURE_PLAN.md).
//
//----------------------------------------------------------------------------
// THE SENSORS (full reference: docs/SENSOR_BOARD.md, datasheets in datasheet/)
//
//  Sensor        Measures            Interface     Addr   Proves it is alive          Side-band pins (this file)
//  ADT7420       temperature         I2C bus 0     0x48   reg 0x0B = 0xCB             A0, A1 (address), CT, INT (alarms)
//  HTS221        humidity + temp     I2C bus 0     0x5F   reg 0x0F = 0xBC             SPIenable (1 = I2C mode), DRDY
//  LPS35HW       pressure + temp     I2C bus 0     0x5C   reg 0x0F = 0xB1             CS (1 = I2C mode), SDO (= SA0 address bit), INT_DRDY
//  LSM303DLHC    accel / magnet      I2C bus 1     0x19 / 0x1E  mag 0x0A..0x0C = 48 34 33   DRDY, INT1, INT2
//  AD7156        capacitance (touch) I2C bus 1     0x48   reg 0x0F default = 0x19     OUT1, OUT2 (proximity)
//  CMV300        648x488 image       SPI + 10-bit parallel video   SPI_EN/CLK/IN/OUT, CLK_IN (25 MHz from this file),
//                (labelled CVM300)                                 CLK_OUT/D[9:0]/Line_valid/Data_valid (from sensor),
//                                                                  SYS_RES_N, Enable_LVDS, FRAME_REQ, T_EXP1/2
//  User I/O      buttons SW0..SW3 (pressed = 0), sensor-board LEDs D0..D3 (1 = on), XEM LEDs (active low)
//
//  I2C bus 0 = I2C_SCL_0 (H3) / I2C_SDA_0 (G3);  bus 1 = I2C_SCL_1 (D2) / I2C_SDA_1 (E2).
//  10 k pull-ups on the board; every part accepts SCL <= 400 kHz. The two
//  0x48 parts (ADT7420, AD7156) are on different buses for that reason.
//
//----------------------------------------------------------------------------
// PC <-> FPGA CONTRACT (must match python/sensor_board.py, python/cmv300.py and vivado/build.tcl)
//   WireIn  0x00  ctrl    bit0 = start (rising edge -> one 'start' pulse)
//                         bit1 = reset (level), bits[11:8] = sensor-board LEDs
//   WireIn  0x01  param1  [7:0] SAD+W = {7-bit slave address, 0}, e.g. 0x90 for the ADT7420
//                         [8]   bus select: 0 = bus 0, 1 = bus 1
//   WireIn  0x02  param2  [7:0] register address, [15:8] data byte to write, [16] 1 = write / 0 = read
//   WireIn  0x03  param3  [7:0] number of bytes to read (1..4)
//   WireIn  0x04..0x07   camera control / SPI command / test-pattern rows / LVDS taps (CMV300_Subsystem.v)
//   WireIn  0x08  sensor sequencer: bit0 enable, bit1 own bus 0, bit2 own bus 1, bit3 frame markers
//   WireIn  0x09..0x0B  sequencer period overrides, 0.1 ms units, two sensors per wire, 0 = default (Sensor_Sequencer.v)
//   WireOut 0x20  result0 [3:0] ACK of frames 1..3 (bit 0 = SAD+W, bit 1 = register, bit 2 = data or SAD+R;
//                         0 = ACK, 1 = NACK), [15:8] SAD+W sent, [16] bus used
//   WireOut 0x21  result1 bytes received, first byte in [31:24]
//   WireOut 0x22  result2 clock cycles from start to done (transaction time; / 100.8 = us)
//   WireOut 0x23  status  bit0 busy, bit1 done, bit2 error (any frame NACKed, or FSM fault),
//                         [15:8] I2C_Transmit.State, [19:16] buttons (1 = pressed)
//   WireOut 0x24..0x2F   camera status / counters / SPI data / ring / frame period / LVDS status (map in CMV300_Subsystem.v)
//   WireOut 0x30  sensor sequencer: [31] present, [23:16] dropped records, [15] overflow sticky, [12:0] FIFO fill in words
//   WireOut 0x3F  design ID 0xEC437007 (lets the PC check the right bit file)
//   TriggerIn  0x40       bit0 = start pulse, bit1 = reset pulse
//   TriggerIn  0x41       camera: spi_go, frame_req, snap_arm, pipe_rewind, cam_reset, clear, read_req, read_ack, t_exp1,
//                         lvds tap load, lvds realign
//   TriggerOut 0x60       bit0 = done (fires once per completed I2C transaction)
//   TriggerOut 0x61       camera: spi_done, frame_done, snap_done, read_done, frame_ready
//   PipeOut    0xA0       camera snapshot: three 10-bit pixels per 32-bit word ([9:0] first)
//   BTPipeOut  0xA1       camera frame from the DDR3 ring: packed little-endian 12-bit pixels (2 per 3 bytes), 1024-byte blocks
//   PipeOut    0xA2       sensor records, 16 bytes each: id/seq/len/flags, timestamp, 8 payload bytes (Sensor_Sequencer.v)
//
// Clocking: the I2C machine and the PC endpoints run on okClk (100.8 MHz,
// from the FrontPanel IP) - no clock-domain crossing between them. The
// 200 MHz board oscillator (clk200) feeds the MMCM that makes the 25 MHz
// CMV300 master clock (clk25 -> ODDR -> CVM300_CLK_IN) and the MIG (DDR3-800,
// ui_clk = 100 MHz for the DMAs). The camera's pixel path runs on the
// sensor's returned CLK_OUT; every crossing is inside CMV300_*.v.
//
// Two builds share this file (the LVDS pairs sit on the same bank-34 pins as
// the parallel bus, so one bit file cannot have both):
//   default          parallel 10-bit port (CVM300_D/Line_valid/Data_valid, LVCMOS25) -> bitfile/SensorBoard_Top.bit
//   `CMV_LVDS        LVDS receiver (CVM300_Out1..4/ctrl/clk P/N, LVDS_25, constraints/lvds/cmv300_lvds.xdc),
//                    Enable_LVDS = 1, ./build.sh --lvds -> bitfile/SensorBoard_Top_lvds.bit
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

    // XEM7310 DDR3 (pins constrained by the MIG IP, vivado/mig_xem7310_a75.prj)
    inout  wire [31:0] ddr3_dq,
    inout  wire [3:0]  ddr3_dqs_p,
    inout  wire [3:0]  ddr3_dqs_n,
    output wire [14:0] ddr3_addr,
    output wire [2:0]  ddr3_ba,
    output wire        ddr3_ras_n,
    output wire        ddr3_cas_n,
    output wire        ddr3_we_n,
    output wire        ddr3_reset_n,
    output wire [0:0]  ddr3_ck_p,
    output wire [0:0]  ddr3_ck_n,
    output wire [0:0]  ddr3_cke,
    output wire [3:0]  ddr3_dm,
    output wire [0:0]  ddr3_odt,

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
    output wire        LPS35_SDO,         // = SA0 address bit (0 -> 0x5C, 1 -> 0x5D)
    input  wire        LPS35_INT_DRDY,

    // I2C bus 1 : LSM303DLHC (accel 0x19, mag 0x1E), AD7156 (0x48)
    inout  wire        I2C_SCL_1,
    inout  wire        I2C_SDA_1,
    input  wire        LSM303_DRDY,
    input  wire        LSM303_INT1,
    input  wire        LSM303_INT2,
    input  wire        AD7156_OUT1,
    input  wire        AD7156_OUT2,

    // CMV300 image sensor (CMOSIS CMV300, labelled CVM300 on the board): SPI control + 10-bit parallel data
    output wire        CVM300_SPI_EN,
    output wire        CVM300_SPI_CLK,
    output wire        CVM300_SPI_IN,     // FPGA -> sensor (MOSI)
    input  wire        CVM300_SPI_OUT,    // sensor -> FPGA (MISO)
    output wire        CVM300_CLK_IN,     // sensor master clock, from FPGA (25 MHz)
    input  wire        CVM300_CLK_OUT,    // pixel clock, from sensor
    output wire        CVM300_SYS_RES_N,
    output wire        CVM300_Enable_LVDS,
    output wire        CVM300_FRAME_REQ,
    output wire        CVM300_T_EXP1,
    output wire        CVM300_T_EXP2,
`ifdef CMV_LVDS
    input  wire        CVM300_Out1_P, CVM300_Out1_N,      // LVDS data lanes (bank 34, same pins as D[9:0])
    input  wire        CVM300_Out2_P, CVM300_Out2_N,
    input  wire        CVM300_Out3_P, CVM300_Out3_N,
    input  wire        CVM300_Out4_P, CVM300_Out4_N,
    input  wire        CVM300_Out_ctrl_P, CVM300_Out_ctrl_N,
    input  wire        CVM300_Out_clk_P, CVM300_Out_clk_N   // 150 MHz DDR clock (MRCC pair R4/T4)
`else
    input  wire [9:0]  CVM300_D,
    input  wire        CVM300_Line_valid,
    input  wire        CVM300_Data_valid
`endif
);

`ifdef CMV_LVDS
    localparam CAM_LVDS = 1;
    wire [9:0] cmv_d = 10'd0;
    wire       cmv_lval = 1'b0, cmv_dval = 1'b0;
    wire [3:0] lv_d_p = {CVM300_Out4_P, CVM300_Out3_P, CVM300_Out2_P, CVM300_Out1_P};
    wire [3:0] lv_d_n = {CVM300_Out4_N, CVM300_Out3_N, CVM300_Out2_N, CVM300_Out1_N};
    wire       lv_ctrl_p = CVM300_Out_ctrl_P, lv_ctrl_n = CVM300_Out_ctrl_N;
    wire       lv_clk_p = CVM300_Out_clk_P, lv_clk_n = CVM300_Out_clk_N;
`else
    localparam CAM_LVDS = 0;
    wire [9:0] cmv_d = CVM300_D;
    wire       cmv_lval = CVM300_Line_valid, cmv_dval = CVM300_Data_valid;
    wire [3:0] lv_d_p = 4'd0, lv_d_n = 4'd0;
    wire       lv_ctrl_p = 1'b0, lv_ctrl_n = 1'b0, lv_clk_p = 1'b0, lv_clk_n = 1'b0;
`endif

    //------------------------------------------------------------------------
    // Clocks
    //------------------------------------------------------------------------
    wire okClk;                       // 100.8 MHz from the FrontPanel IP - the design clock
    wire clk200_i, clk200;            // 200 MHz board oscillator -> MMCM (25 MHz) and MIG
    IBUFGDS ibuf_sysclk (.I(sys_clkp), .IB(sys_clkn), .O(clk200_i));
    BUFG    u_bufg_200  (.I(clk200_i), .O(clk200));

    // 200 MHz -> 25 and 40 MHz: VCO = 200 x 5 = 1000 MHz, CLKOUT0 = 1000/40, CLKOUT1 = 1000/25.
    // 25 MHz is the parallel port's maximum (CMV300.pdf §2.4/§3.5); 40 MHz is for LVDS at 480 Mbit/s
    // (docs/CMV300_CAPTURE_PLAN.md §1.1/§12) and is selected at run time by wi04[2] in the LVDS build,
    // through a glitch-free BUFGCTRL - the Python switches it only while the sensor is held in reset.
    wire clk25_unbuf, clk40_unbuf, clk_cam, mmcm_fb_unbuf, mmcm_fb, mmcm_locked;
    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"), .CLKFBOUT_MULT_F(5.0), .CLKFBOUT_PHASE(0.0), .CLKIN1_PERIOD(5.000),
        .CLKOUT0_DIVIDE_F(40.0), .CLKOUT0_DUTY_CYCLE(0.5), .CLKOUT0_PHASE(0.0),
        .CLKOUT1_DIVIDE(25), .CLKOUT2_DIVIDE(1), .CLKOUT3_DIVIDE(1), .CLKOUT4_DIVIDE(1),
        .CLKOUT5_DIVIDE(1), .CLKOUT6_DIVIDE(1), .CLKOUT4_CASCADE("FALSE"),
        .DIVCLK_DIVIDE(1), .REF_JITTER1(0.010), .STARTUP_WAIT("FALSE")
    ) u_mmcm (
        .CLKOUT0(clk25_unbuf), .CLKOUT0B(), .CLKOUT1(clk40_unbuf), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
        .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .CLKFBOUT(mmcm_fb_unbuf), .CLKFBOUTB(), .LOCKED(mmcm_locked),
        .CLKIN1(clk200), .PWRDWN(1'b0), .RST(1'b0), .CLKFBIN(mmcm_fb)
    );
    BUFG u_bufg_fb (.I(mmcm_fb_unbuf), .O(mmcm_fb));
    wire cam_clk_sel40;
    BUFGCTRL #(.INIT_OUT(0), .PRESELECT_I0("TRUE"), .PRESELECT_I1("FALSE")) u_bufg_cam (
        .I0(clk25_unbuf), .I1(clk40_unbuf), .S0(~cam_clk_sel40), .S1(cam_clk_sel40),
        .CE0(1'b1), .CE1(1'b1), .IGNORE0(1'b0), .IGNORE1(1'b0), .O(clk_cam)
    );

    // Clock forwarding: CVM300_CLK_IN = clk_cam while the camera sequencer enables it (glitch-free through the ODDR)
    wire      cmv_clk_en;
    reg [1:0] clk_en_cam = 2'b00;
    always @(posedge clk_cam) clk_en_cam <= {clk_en_cam[0], cmv_clk_en};
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_oddr_clk_in (
        .Q(CVM300_CLK_IN), .C(clk_cam), .CE(1'b1), .D1(clk_en_cam[1]), .D2(1'b0), .R(1'b0), .S(1'b0)
    );

    //------------------------------------------------------------------------
    // FrontPanel endpoint wires
    //------------------------------------------------------------------------
    wire [31:0] wi00, wi01, wi02, wi03, wi04, wi05, wi06, wi07, wi08, wi09, wi0a, wi0b;
    wire [31:0] wo20, wo21, wo22, wo23, wo24, wo25, wo26, wo27, wo28, wo29, wo2a, wo2b, wo2c, wo2d, wo2e, wo2f, wo30, wo3f;
    wire [31:0] ti40, ti41, to60, to61;
    wire        poa0_read, poa2_read;
    wire [31:0] poa0_data, poa2_data;
    wire        bta1_read, bta1_ready;
    wire [31:0] bta1_data;

    //------------------------------------------------------------------------
    // Safe defaults: every I2C sensor idles quiet and in I2C mode. The CMV300
    // pins are owned by u_cam (its sequencer keeps the sensor in reset with
    // no clock until the PC sets cam_enable).
    //------------------------------------------------------------------------
    assign ADT7420_A0         = 1'b0;   // address 0x48 = 0x48 + A0 + 2*A1 (ADT7420.pdf Table 20, p.17)
    assign ADT7420_A1         = 1'b0;
    assign HTS221_SPIenable   = 1'b1;   // HTS221 stays in I2C mode
    assign LPS35_CS           = 1'b1;   // LPS35HW stays in I2C mode
    assign LPS35_SDO          = 1'b0;   // LPS35HW SA0 = 0 -> address 0x5C
    // Inputs not used today: ADT7420_CT/INT, HTS221_DRDY, LPS35_INT_DRDY,
    // LSM303_DRDY/INT1/INT2, AD7156_OUT1/OUT2 - wire them in when your design needs them.

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
    // I2C buses: open-drain drivers + bus selector.
    // The state machine only says "pull low" (1) or "release" (0) and reads
    // the real pin back (slave ACK / data, clock stretching). param1[8]
    // picks the bus; it is latched on start so the other bus stays released
    // for the whole transaction. Registered -> the drivers sit in pad flops.
    //------------------------------------------------------------------------
    wire i2c_scl_low, i2c_sda_low;               // from I2C_Transmit
    reg  bus_sel = 1'b0;                          // 0 = bus 0, 1 = bus 1
    // The sensor sequencer (u_seq) takes a bus over when the host sets wi08[1]
    // (bus 0) / wi08[2] (bus 1); u_i2c keeps the other one. Hand-over only
    // with both masters idle (the Python does that).
    wire seq_own0 = 1'b0, seq_own1 = 1'b0;   // Lab 10: u_i2c owns both buses
    wire seq_scl0_low, seq_sda0_low, seq_scl1_low, seq_sda1_low;
    reg  i2c0_scl_low = 1'b0, i2c0_sda_low = 1'b0, i2c1_scl_low = 1'b0, i2c1_sda_low = 1'b0;
    always @(posedge okClk) begin
        i2c0_scl_low <= seq_own0 ? seq_scl0_low : (bus_sel ? 1'b0 : i2c_scl_low);
        i2c0_sda_low <= seq_own0 ? seq_sda0_low : (bus_sel ? 1'b0 : i2c_sda_low);
        i2c1_scl_low <= seq_own1 ? seq_scl1_low : (bus_sel ? i2c_scl_low : 1'b0);
        i2c1_sda_low <= seq_own1 ? seq_sda1_low : (bus_sel ? i2c_sda_low : 1'b0);
    end
    assign I2C_SCL_0 = i2c0_scl_low ? 1'b0 : 1'bz;
    assign I2C_SDA_0 = i2c0_sda_low ? 1'b0 : 1'bz;
    assign I2C_SCL_1 = i2c1_scl_low ? 1'b0 : 1'bz;
    assign I2C_SDA_1 = i2c1_sda_low ? 1'b0 : 1'bz;
    wire i2c_scl_in = bus_sel ? I2C_SCL_1 : I2C_SCL_0;   // real levels on the selected bus
    wire i2c_sda_in = bus_sel ? I2C_SDA_1 : I2C_SDA_0;

    //------------------------------------------------------------------------
    // The I2C state machine - hdl/I2C_Transmit.v - THE FILE YOU EDIT.
    // TICK_DIVIDE = 252 -> one state every 2.5 us -> SCL = 100 kHz.
    //------------------------------------------------------------------------
    wire [3:0]  i2c_acks;                    // per-frame ACK bits, 1 = NACK
    wire        i2c_busy, i2c_err;
    wire [7:0]  i2c_state;
    wire [63:0] i2c_rx_data;

    I2C_Transmit #(.TICK_DIVIDE(252)) u_i2c (
        .clk       (okClk),
        .rst       (rst),
        .start     (start),
        .tx_byte   (wi01[7:0]),      // param1[7:0] = SAD+W
        .param2    (wi02),           // register address / write data / write flag
        .param3    (wi03),           // number of bytes to read
        .scl_low   (i2c_scl_low),
        .sda_low   (i2c_sda_low),
        .scl_in    (i2c_scl_in),
        .sda_in    (i2c_sda_in),
        .ACK_frames(i2c_acks),
        .rx_data   (i2c_rx_data),    // -> result1
        .busy      (i2c_busy),
        .done      (i2c_done),
        .error_bit (i2c_err),
        .State     (i2c_state)       // -> status[15:8]
    );

    //------------------------------------------------------------------------
    // The six-sensor scheduler is NOT part of Lab 10 - it is Final Project
    // Part 2a. Its bus hand-over, its PipeOut and its status word are tied off
    // so the rest of this top level is unchanged.
    //------------------------------------------------------------------------
    assign seq_scl0_low = 1'b0;
    assign seq_sda0_low = 1'b0;
    assign seq_scl1_low = 1'b0;
    assign seq_sda1_low = 1'b0;
    assign poa2_data    = 32'd0;
    assign wo30         = 32'd0;

    //------------------------------------------------------------------------
    // DDR3 controller (MIG 7-Series, AXI4 128-bit, ui_clk 100 MHz) - the
    // camera's frame ring lives in it (3 x 1 MiB slots of the 1 GiB).
    //------------------------------------------------------------------------
    wire         ui_clk, ui_clk_sync_rst, init_calib_complete, ddr_sys_rst;
    wire [3:0]   axi_awid, axi_arid, axi_bid, axi_rid;
    wire [29:0]  axi_awaddr, axi_araddr;
    wire [7:0]   axi_awlen, axi_arlen;
    wire [2:0]   axi_awsize, axi_arsize;
    wire [1:0]   axi_awburst, axi_arburst, axi_bresp, axi_rresp;
    wire         axi_awvalid, axi_awready, axi_wvalid, axi_wready, axi_wlast, axi_bvalid, axi_bready;
    wire         axi_arvalid, axi_arready, axi_rvalid, axi_rready, axi_rlast;
    wire [127:0] axi_wdata, axi_rdata;
    wire [15:0]  axi_wstrb;

    mig_7series_0 u_mig (
        .ddr3_dq(ddr3_dq), .ddr3_dqs_n(ddr3_dqs_n), .ddr3_dqs_p(ddr3_dqs_p),
        .ddr3_addr(ddr3_addr), .ddr3_ba(ddr3_ba), .ddr3_ras_n(ddr3_ras_n), .ddr3_cas_n(ddr3_cas_n),
        .ddr3_we_n(ddr3_we_n), .ddr3_reset_n(ddr3_reset_n), .ddr3_ck_p(ddr3_ck_p), .ddr3_ck_n(ddr3_ck_n),
        .ddr3_cke(ddr3_cke), .ddr3_dm(ddr3_dm), .ddr3_odt(ddr3_odt),
        .sys_clk_i(clk200),
        .ui_clk(ui_clk), .ui_clk_sync_rst(ui_clk_sync_rst), .mmcm_locked(),
        .aresetn(~ui_clk_sync_rst),
        .app_sr_req(1'b0), .app_ref_req(1'b0), .app_zq_req(1'b0), .app_sr_active(), .app_ref_ack(), .app_zq_ack(),
        .s_axi_awid(axi_awid), .s_axi_awaddr(axi_awaddr), .s_axi_awlen(axi_awlen), .s_axi_awsize(axi_awsize),
        .s_axi_awburst(axi_awburst), .s_axi_awlock(1'b0), .s_axi_awcache(4'b0011), .s_axi_awprot(3'b000),
        .s_axi_awqos(4'b0000), .s_axi_awvalid(axi_awvalid), .s_axi_awready(axi_awready),
        .s_axi_wdata(axi_wdata), .s_axi_wstrb(axi_wstrb), .s_axi_wlast(axi_wlast), .s_axi_wvalid(axi_wvalid),
        .s_axi_wready(axi_wready), .s_axi_bready(axi_bready), .s_axi_bid(axi_bid), .s_axi_bresp(axi_bresp),
        .s_axi_bvalid(axi_bvalid),
        .s_axi_arid(axi_arid), .s_axi_araddr(axi_araddr), .s_axi_arlen(axi_arlen), .s_axi_arsize(axi_arsize),
        .s_axi_arburst(axi_arburst), .s_axi_arlock(1'b0), .s_axi_arcache(4'b0011), .s_axi_arprot(3'b000),
        .s_axi_arqos(4'b0000), .s_axi_arvalid(axi_arvalid), .s_axi_arready(axi_arready),
        .s_axi_rready(axi_rready), .s_axi_rid(axi_rid), .s_axi_rdata(axi_rdata), .s_axi_rresp(axi_rresp),
        .s_axi_rlast(axi_rlast), .s_axi_rvalid(axi_rvalid),
        .init_calib_complete(init_calib_complete), .device_temp(),
        .sys_rst(ddr_sys_rst)
    );

    //------------------------------------------------------------------------
    // The camera - hdl/CMV300_Subsystem.v (docs/CMV300_CAPTURE_PLAN.md)
    //------------------------------------------------------------------------
    CMV300_Subsystem #(.AW(30), .LVDS(CAM_LVDS)) u_cam (
        .clk(okClk), .rst(rst), .clk_locked(mmcm_locked),
        .clk_cmv(CVM300_CLK_OUT), .d(cmv_d), .lval(cmv_lval), .dval(cmv_dval),
        .clk200(clk200), .lv_clk_p(lv_clk_p), .lv_clk_n(lv_clk_n), .lv_d_p(lv_d_p), .lv_d_n(lv_d_n),
        .lv_ctrl_p(lv_ctrl_p), .lv_ctrl_n(lv_ctrl_n),
        .spi_out(CVM300_SPI_OUT), .spi_clk(CVM300_SPI_CLK), .spi_en(CVM300_SPI_EN), .spi_in(CVM300_SPI_IN),
        .clk_in_en(cmv_clk_en), .clk_sel40(cam_clk_sel40), .sys_res_n(CVM300_SYS_RES_N), .enable_lvds(CVM300_Enable_LVDS),
        .frame_req(CVM300_FRAME_REQ), .t_exp1(CVM300_T_EXP1), .t_exp2(CVM300_T_EXP2),
        .ui_clk(ui_clk), .ui_rst(ui_clk_sync_rst), .calib_done(init_calib_complete), .ddr_sys_rst(ddr_sys_rst),
        .m_axi_awid(axi_awid), .m_axi_awaddr(axi_awaddr), .m_axi_awlen(axi_awlen), .m_axi_awsize(axi_awsize),
        .m_axi_awburst(axi_awburst), .m_axi_awvalid(axi_awvalid), .m_axi_awready(axi_awready),
        .m_axi_wdata(axi_wdata), .m_axi_wstrb(axi_wstrb), .m_axi_wlast(axi_wlast), .m_axi_wvalid(axi_wvalid),
        .m_axi_wready(axi_wready), .m_axi_bid(axi_bid), .m_axi_bresp(axi_bresp), .m_axi_bvalid(axi_bvalid),
        .m_axi_bready(axi_bready),
        .m_axi_arid(axi_arid), .m_axi_araddr(axi_araddr), .m_axi_arlen(axi_arlen), .m_axi_arsize(axi_arsize),
        .m_axi_arburst(axi_arburst), .m_axi_arvalid(axi_arvalid), .m_axi_arready(axi_arready),
        .m_axi_rid(axi_rid), .m_axi_rdata(axi_rdata), .m_axi_rresp(axi_rresp), .m_axi_rlast(axi_rlast),
        .m_axi_rvalid(axi_rvalid), .m_axi_rready(axi_rready),
        .wi04(wi04), .wi05(wi05), .wi06(wi06), .wi07(wi07), .ti41(ti41), .to61(to61),
        .wo24(wo24), .wo25(wo25), .wo26(wo26), .wo27(wo27), .wo28(wo28), .wo29(wo29), .wo2a(wo2a),
        .wo2b(wo2b), .wo2c(wo2c), .wo2d(wo2d), .wo2e(wo2e), .wo2f(wo2f),
        .po_read(poa0_read), .po_data(poa0_data),
        .bt_read(bta1_read), .bt_data(bta1_data), .bt_ready(bta1_ready)
    );

    //------------------------------------------------------------------------
    // Results, status, LEDs
    //------------------------------------------------------------------------
    reg  [7:0]  byte_sent = 8'h00;          // copy of param1[7:0] for result0
    reg  [31:0] cycles    = 32'd0;          // clock cycles busy = transaction time
    reg  [25:0] blink_cnt = 26'd0;          // 1 Hz heartbeat on LED7: toggles every 50,400,000 clocks = 0.5 s
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

    wire error = i2c_err | (i2c_done & |i2c_acks);         // any NACK = failed transaction

    assign wo20 = {15'd0, bus_sel, byte_sent, 4'd0, i2c_acks};
    assign wo21 = i2c_rx_data[63:32];            // first 4 bytes: identical to the old 32-bit rx_data
    assign wo22 = cycles;
    assign wo23 = {12'd0, ~button, i2c_state, 5'd0, error, i2c_done, i2c_busy};
    assign wo3f = 32'hEC43_7007;                            // design ID / version (7007 = sensor sequencer: WireIn 0x08-0x0B, WireOut 0x30, PipeOut 0xA2)
    assign to60 = {31'd0, i2c_done & ~done_d};              // done rising edge
    assign s_LED = wi00[11:8];                              // sensor-board LEDs from ctrl[11:8]
    assign led   = ~{blink, error, i2c_done & ~|i2c_acks, i2c_busy, ~button};   // LED7 blinks at 1 Hz = a live bit file; XEM LEDs are active low

    //------------------------------------------------------------------------
    // FrontPanel Subsystem IP (OpalKelly, v1.0.6). Endpoint set is configured
    // in vivado/build.tcl - keep the two in sync.
    //------------------------------------------------------------------------
    frontpanel_0 frontpanel_inst (
        .okUH             (okUH),
        .okHU             (okHU),
        .okUHU            (okUHU),
        .okAA             (okAA),
        .okClk            (okClk),
        // PC -> FPGA
        .wi00_ep_dataout  (wi00),
        .wi01_ep_dataout  (wi01),
        .wi02_ep_dataout  (wi02),
        .wi03_ep_dataout  (wi03),
        .wi04_ep_dataout  (wi04),
        .wi05_ep_dataout  (wi05),
        .wi06_ep_dataout  (wi06),
        .wi07_ep_dataout  (wi07),
        .wi08_ep_dataout  (wi08),
        .wi09_ep_dataout  (wi09),
        .wi0a_ep_dataout  (wi0a),
        .wi0b_ep_dataout  (wi0b),
        .ti40_ep_clk      (okClk),
        .ti40_ep_trigger  (ti40),
        .ti41_ep_clk      (okClk),
        .ti41_ep_trigger  (ti41),
        // FPGA -> PC
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
        .wo2d_ep_datain   (wo2d),
        .wo2e_ep_datain   (wo2e),
        .wo2f_ep_datain   (wo2f),
        .wo30_ep_datain   (wo30),
        .wo3f_ep_datain   (wo3f),
        .to60_ep_clk      (okClk),
        .to60_ep_trigger  (to60),
        .to61_ep_clk      (okClk),
        .to61_ep_trigger  (to61),
        .poa0_ep_read     (poa0_read),
        .poa0_ep_datain   (poa0_data),
        .poa2_ep_read     (poa2_read),
        .poa2_ep_datain   (poa2_data),
        .btpoa1_ep_read   (bta1_read),
        .btpoa1_ep_datain (bta1_data),
        .btpoa1_ep_blockstrobe (),
        .btpoa1_ep_ready  (bta1_ready)
    );

endmodule
