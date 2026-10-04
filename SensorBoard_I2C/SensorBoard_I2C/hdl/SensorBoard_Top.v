`timescale 1ns / 1ps
//============================================================================
// SensorBoard_Top.v - TOP LEVEL. You do not need to edit this file.
//
// OpalKelly XEM7310-A75 (Artix-7 xc7a75tfgg484-1) on the UIUC ECE 437
// Sensor Board. This file owns everything that is fixed by the hardware:
//   * the FrontPanel Subsystem IP (PC <-> FPGA endpoints, 100.8 MHz okClk),
//   * every pin of every sensor and the level each one must idle at,
//   * the open-drain drivers of the two I2C buses and the bus selector,
//   * the start/reset conditioning and the result/status words.
// The protocol state machine lives in hdl/I2C_Transmit.v - THAT is the file
// you edit. This file instantiates it once (u_i2c, below).
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
//  CMV300        648x488 image       SPI + 10-bit parallel video   SPI_EN/CLK/IN/OUT, CLK_IN (from FPGA, 10-40 MHz),
//                (labelled CVM300)                                 CLK_OUT/D[9:0]/Line_valid/Data_valid (from sensor),
//                                                                  SYS_RES_N, Enable_LVDS, FRAME_REQ, T_EXP1/2
//  User I/O      buttons SW0..SW3 (pressed = 0), sensor-board LEDs D0..D3 (1 = on), XEM LEDs (active low)
//
//  I2C bus 0 = I2C_SCL_0 (H3) / I2C_SDA_0 (G3);  bus 1 = I2C_SCL_1 (D2) / I2C_SDA_1 (E2).
//  10 k pull-ups on the board; every part accepts SCL <= 400 kHz. The two
//  0x48 parts (ADT7420, AD7156) are on different buses for that reason.
//
//----------------------------------------------------------------------------
// PC <-> FPGA CONTRACT (must match python/sensor_board.py and vivado/build.tcl)
//   WireIn  0x00  ctrl    bit0 = start (rising edge -> one 'start' pulse)
//                         bit1 = reset (level), bits[11:8] = sensor-board LEDs
//   WireIn  0x01  param1  [7:0] byte for the I2C state machine = {7-bit address, R/W}
//                         [8]   bus select: 0 = bus 0, 1 = bus 1
//   WireIn  0x02  param2  handed to I2C_Transmit unchanged (free: register address, data ...)
//   WireIn  0x03  param3  handed to I2C_Transmit unchanged (free: byte count ...)
//   WireOut 0x20  result0 [0] ACK bit of the last frame (0 = slave ACKed), [15:8] byte sent, [16] bus used
//   WireOut 0x21  result1 rx_data from I2C_Transmit (the bytes you will read - 0 today)
//   WireOut 0x22  result2 clock cycles from start to done (transaction time; / 100.8 = us)
//   WireOut 0x23  status  bit0 busy, bit1 done, bit2 error (NACK or FSM fault),
//                         [15:8] I2C_Transmit.State, [19:16] buttons (1 = pressed)
//   WireOut 0x3F  design ID 0xEC437001 (lets the PC check the right bit file)
//   TriggerIn  0x40       bit0 = start pulse, bit1 = reset pulse
//   TriggerOut 0x60       bit0 = done (fires once per completed transaction)
//   PipeOut    0xA0       32-bit words (an incrementing counter today - feed it from a FIFO)
//
// Clocking: everything runs on okClk (100.8 MHz, from the FrontPanel IP), so
// there is NO clock-domain crossing between the PC endpoints and the state
// machine. The 200 MHz board oscillator is buffered as clk200 for designs that
// need it (e.g. the CMV300 pixel clock) - crossing to/from it needs synchronisers.
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
    output wire        CVM300_CLK_IN,     // sensor master clock, from FPGA
    input  wire        CVM300_CLK_OUT,    // pixel clock, from sensor
    output wire        CVM300_SYS_RES_N,
    output wire        CVM300_Enable_LVDS,
    output wire        CVM300_FRAME_REQ,
    output wire        CVM300_T_EXP1,
    output wire        CVM300_T_EXP2,
    input  wire [9:0]  CVM300_D,
    input  wire        CVM300_Line_valid,
    input  wire        CVM300_Data_valid
);

    //------------------------------------------------------------------------
    // Clocks
    //------------------------------------------------------------------------
    wire okClk;                       // 100.8 MHz from the FrontPanel IP - the design clock
    wire clk200;                      // 200 MHz board oscillator (unused today)
    IBUFGDS ibuf_sysclk (.I(sys_clkp), .IB(sys_clkn), .O(clk200));

    //------------------------------------------------------------------------
    // FrontPanel endpoint wires
    //------------------------------------------------------------------------
    wire [31:0] wi00, wi01, wi02, wi03;
    wire [31:0] wo20, wo21, wo22, wo23, wo3f;
    wire [31:0] ti40, to60;
    wire        poa0_read;
    wire [31:0] poa0_data;

    //------------------------------------------------------------------------
    // Safe defaults: every sensor idles quiet and in I2C mode. Change a line
    // only when your design takes that sensor over (e.g. the CMV300 needs
    // CLK_IN running and SYS_RES_N = 1 before its SPI answers).
    //------------------------------------------------------------------------
    assign ADT7420_A0         = 1'b0;   // address 0x48 = 0x48 + A0 + 2*A1 (ADT7420.pdf Table 20, p.17)
    assign ADT7420_A1         = 1'b0;
    assign HTS221_SPIenable   = 1'b1;   // HTS221 stays in I2C mode
    assign LPS35_CS           = 1'b1;   // LPS35HW stays in I2C mode
    assign LPS35_SDO          = 1'b0;   // LPS35HW SA0 = 0 -> address 0x5C
    assign CVM300_SPI_EN      = 1'b0;   // CMV300 SPI idle
    assign CVM300_SPI_CLK     = 1'b0;
    assign CVM300_SPI_IN      = 1'b0;
    assign CVM300_CLK_IN      = 1'b0;   // no master clock -> imager idle
    assign CVM300_SYS_RES_N   = 1'b0;   // imager held in reset
    assign CVM300_Enable_LVDS = 1'b0;   // CMOS (parallel) output mode
    assign CVM300_FRAME_REQ   = 1'b0;
    assign CVM300_T_EXP1      = 1'b0;
    assign CVM300_T_EXP2      = 1'b0;
    // Inputs not used today: ADT7420_CT/INT, HTS221_DRDY, LPS35_INT_DRDY,
    // LSM303_DRDY/INT1/INT2, AD7156_OUT1/OUT2, CVM300_SPI_OUT/CLK_OUT/D/
    // Line_valid/Data_valid - wire them in when your design needs them.

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
    wire i2c_scl_in = bus_sel ? I2C_SCL_1 : I2C_SCL_0;   // real levels on the selected bus
    wire i2c_sda_in = bus_sel ? I2C_SDA_1 : I2C_SDA_0;

    //------------------------------------------------------------------------
    // The I2C state machine - hdl/I2C_Transmit.v - THE FILE YOU EDIT.
    // TICK_DIVIDE = 252 -> one state every 2.5 us -> SCL = 100 kHz.
    //------------------------------------------------------------------------
    wire        i2c_ack, i2c_busy, i2c_err;
    wire [7:0]  i2c_state;
    wire [31:0] i2c_rx_data;

    I2C_Transmit #(.TICK_DIVIDE(252)) u_i2c (
        .clk       (okClk),
        .rst       (rst),
        .start     (start),
        .tx_byte   (wi01[7:0]),      // param1[7:0] = {7-bit slave address, R/W}
        .param2    (wi02),           // free for your protocol (register address, data ...)
        .param3    (wi03),           // free for your protocol (byte count ...)
        .scl_low   (i2c_scl_low),
        .sda_low   (i2c_sda_low),
        .scl_in    (i2c_scl_in),
        .sda_in    (i2c_sda_in),
        .ACK_bit   (i2c_ack),
        .rx_data   (i2c_rx_data),    // -> result1
        .busy      (i2c_busy),
        .done      (i2c_done),
        .error_bit (i2c_err),
        .State     (i2c_state)       // -> status[15:8]
    );

    //------------------------------------------------------------------------
    // Results, status, LEDs, pipe
    //------------------------------------------------------------------------
    reg  [7:0]  byte_sent = 8'h00;          // copy of param1[7:0] for result0
    reg  [31:0] cycles    = 32'd0;          // clock cycles busy = transaction time
    reg  [31:0] pipe_word = 32'd0;          // PipeOut test pattern
    reg  [26:0] heartbeat = 27'd0;          // ~0.67 s blink on LED7

    always @(posedge okClk) begin
        heartbeat <= heartbeat + 1'b1;
        if (poa0_read) pipe_word <= pipe_word + 1'b1;
        if (rst) begin
            bus_sel   <= 1'b0;
            byte_sent <= 8'h00;
            cycles    <= 32'd0;
            pipe_word <= 32'd0;
        end else if (start && !i2c_busy) begin
            bus_sel   <= wi01[8];
            byte_sent <= wi01[7:0];
            cycles    <= 32'd0;
        end else if (i2c_busy) begin
            cycles    <= cycles + 1'b1;
        end
    end

    wire error = i2c_err | (i2c_done & i2c_ack);           // NACK = failed transaction

    assign wo20 = {15'd0, bus_sel, byte_sent, 7'd0, i2c_ack};
    assign wo21 = i2c_rx_data;
    assign wo22 = cycles;
    assign wo23 = {12'd0, ~button, i2c_state, 5'd0, error, i2c_done, i2c_busy};
    assign wo3f = 32'hEC43_7001;                            // design ID / version
    assign to60 = {31'd0, i2c_done & ~done_d};              // done rising edge
    assign poa0_data = pipe_word;
    assign s_LED = wi00[11:8];                              // sensor-board LEDs from ctrl[11:8]
    assign led   = ~{heartbeat[26], error, i2c_done & ~i2c_ack, i2c_busy, ~button};   // XEM LEDs are active low

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
        .ti40_ep_clk      (okClk),
        .ti40_ep_trigger  (ti40),
        // FPGA -> PC
        .wo20_ep_datain   (wo20),
        .wo21_ep_datain   (wo21),
        .wo22_ep_datain   (wo22),
        .wo23_ep_datain   (wo23),
        .wo3f_ep_datain   (wo3f),
        .to60_ep_clk      (okClk),
        .to60_ep_trigger  (to60),
        .poa0_ep_read     (poa0_read),
        .poa0_ep_datain   (poa0_data)
    );

endmodule
