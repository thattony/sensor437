`timescale 1ns / 1ps
//============================================================================
// lab8_motor.v - ECE 437 Lab 8: H-bridge motor controller (Pmod DHB1) with
// concurrent LSM303DLHC accelerometer streaming.
//
// One file, four parts:
//   lab8_motor      TOP LEVEL: FrontPanel IP, pins, open-drain I2C pads.
//   lab8_core       everything else, clocked by okClk (100.8 MHz):
//     - motor pulse generator  200 Hz pulse train on EN1, exactly N pulses,
//                              DIR1 changed only while EN1 is low, latched
//                              fault = safe state (EN1 low, coasting)
//     - hardware watchdog      EN1 low if the PC stops talking during a move
//     - accelerometer sampler  free-running: polls STATUS_REG_A, reads X/Y/Z
//                              when ZYXDA = 1, timestamps the sample and pushes
//                              it into a FIFO -> PipeOut 0xA0. It never looks
//                              at the motor pulses (requirement 3).
//     - sample FIFO            8192 x 32 bit block RAM
//   I2C_Transmit    the I2C state machine from the midterm (midterm_p1_i2c.v),
//                   unchanged: burst read 1-8 bytes, register write.
//
// Reused from the midterm (Lab 6 machine -> midterm part 1): the I2C FSM, the
// open-drain pad drivers + bus selector, the host register-access contract on
// WireIn 0x01-0x03 / WireOut 0x20-0x25, the FrontPanel Subsystem IP flow.
//
// ---------------------------------------------------------------------------
// PMOD DHB1 (Digilent, DRV8833).  J1: 1 EN1  2 DIR1  3 S1A  4 S1B  5 GND 6 VCC
//                                     7 EN2  8 DIR2  9 S2A 10 S2B 11 GND 12 VCC
// One motor. EN1 = PWM pulses, DIR1 = direction. EN2/DIR2 copy EN1/DIR1, so the
// motor can be plugged into either DHB1 motor connector (J2 or J3); with only one
// motor connected the other bridge has no load.
// The same signals are driven on BOTH sensor-board PMOD headers (PMOD A = J1
// and PMOD B = J2), so the DHB1 works in either. Pins 3/4/9/10 are DHB1
// outputs (sensor feedback) and are left undriven.
// EN low = both bridge inputs low = coast. The DHB1 has no brake.
//
// ---------------------------------------------------------------------------
// PC <-> FPGA CONTRACT (must match lab8_python.py and fpga/vivado/build.tcl)
//   WireIn  0x00  ctrl    bit1 = reset (level), bits[11:8] unused
//   WireIn  0x01  param1  [7:0] {7-bit I2C address, 0}, [8] bus select     } host I2C
//   WireIn  0x02  param2  [7:0] SUB, [15:8] write data                       } access, only
//   WireIn  0x03  param3  [0] reg txn, [1] write, [7:4] nbytes, [31:16] div  } while sampler off
//   WireIn  0x04  pulses  number of EN1 pulses for the next move (N / 200 s)
//   WireIn  0x05  motor   [0] direction for the next move, [15:8] duty in % (0 -> 50)
//   WireIn  0x06  sampler [0] sampler enable, [1] watchdog enable,
//                         [31:16] watchdog timeout in ms (0 -> 250)
//   TriggerIn 0x40  bit0 host I2C start, bit1 reset, bit2 GO (start a move),
//                   bit3 STALL TRIP (stop + latch fault, code 1), bit4 clear fault,
//                   bit5 FIFO reset, bit6 watchdog kick, bit7 STOP (stop, no fault)
//   WireOut 0x20-0x25  host I2C results, exactly as in the midterm
//   WireOut 0x26  motor status [0] running [1] EN1 [2] DIR1 [3] fault latched
//                 [7:4] fault code (1 stall trip from PC, 2 watchdog)
//                 [8] sampler owns the bus [9] FIFO overflow (sticky)
//                 [10] GO rejected (sticky: fault latched or N = 0)
//                 [11] watchdog enabled [23:16] duty of the current move (%)
//   WireOut 0x27  pulses sent in the current / last move
//   WireOut 0x28  pulses requested for the current / last move
//   WireOut 0x29  FIFO fill, 32-bit words (4 per sample)
//   WireOut 0x2A  timestamp now (0.625 us ticks)
//   WireOut 0x2B  samples pushed since reset
//   WireOut 0x2C  timestamp when the fault latched
//   WireOut 0x2D  pulses sent when the fault latched
//   WireOut 0x2E  [15:0] sampler I2C NACKs, [31:16] ZYXOR overruns seen
//   WireOut 0x3F  design ID 0xEC437008
//   PipeOut 0xA0  samples, 4 words each (read whole samples only):
//      w0 timestamp (0.625 us ticks, when STATUS showed ZYXDA)
//      w1 {OUT_Y (16 b), OUT_X (16 b)}   raw, left-justified: >> 4 = mg
//      w2 {4'hA, fault, running, DIR1, EN1, STATUS_REG_A, OUT_Z}
//      w3 {sequence[7:0], pulses sent[23:0]}
//============================================================================
module lab8_motor (
    // FrontPanel host interface (USB 3.0)
    input  wire [4:0]  okUH,
    output wire [2:0]  okHU,
    inout  wire [31:0] okUHU,
    inout  wire        okAA,

    // XEM7310 board
    input  wire        sys_clkp,          // 200 MHz LVDS oscillator (unused)
    input  wire        sys_clkn,
    output wire [7:0]  led,               // XEM7310 LEDs, active LOW

    // Sensor board user I/O
    input  wire [3:0]  button,            // SW0..SW3, pressed = 0
    output wire [3:0]  s_LED,             // D0..D3, 1 = on

    // Pmod DHB1, on either PMOD header
    output wire        PMOD_A1,           // EN1  (J1 pin 1)
    output wire        PMOD_A2,           // DIR1 (J1 pin 2)
    output wire        PMOD_A7,           // EN2  (copy of EN1)
    output wire        PMOD_A8,           // DIR2
    output wire        PMOD_B1,           // EN1  (J2 pin 1)
    output wire        PMOD_B2,           // DIR1
    output wire        PMOD_B7,           // EN2
    output wire        PMOD_B8,           // DIR2

    // I2C bus 0 : ADT7420, HTS221, LPS35HW (kept quiet)
    inout  wire        I2C_SCL_0,
    inout  wire        I2C_SDA_0,
    output wire        ADT7420_A0,
    output wire        ADT7420_A1,
    output wire        HTS221_SPIenable,
    output wire        LPS35_CS,
    output wire        LPS35_SDO,

    // I2C bus 1 : LSM303DLHC (accel 0x19), AD7156
    inout  wire        I2C_SCL_1,
    inout  wire        I2C_SDA_1,

    // CMV300 image sensor: held idle
    output wire        CVM300_SPI_EN,
    output wire        CVM300_SPI_CLK,
    output wire        CVM300_SPI_IN,
    output wire        CVM300_CLK_IN,
    output wire        CVM300_SYS_RES_N,
    output wire        CVM300_Enable_LVDS,
    output wire        CVM300_FRAME_REQ,
    output wire        CVM300_T_EXP1,
    output wire        CVM300_T_EXP2
);

    wire okClk;                           // 100.8 MHz, the design clock
    wire clk200;
    IBUFGDS ibuf_sysclk (.I(sys_clkp), .IB(sys_clkn), .O(clk200));

    // ---- safe defaults for every sensor this lab does not use (as midterm)
    assign ADT7420_A0         = 1'b0;
    assign ADT7420_A1         = 1'b0;
    assign HTS221_SPIenable   = 1'b1;
    assign LPS35_CS           = 1'b1;
    assign LPS35_SDO          = 1'b0;
    assign CVM300_SPI_EN      = 1'b0;
    assign CVM300_SPI_CLK     = 1'b0;
    assign CVM300_SPI_IN      = 1'b0;
    assign CVM300_CLK_IN      = 1'b0;
    assign CVM300_SYS_RES_N   = 1'b0;
    assign CVM300_Enable_LVDS = 1'b0;
    assign CVM300_FRAME_REQ   = 1'b0;
    assign CVM300_T_EXP1      = 1'b0;
    assign CVM300_T_EXP2      = 1'b0;

    // ---- FrontPanel endpoint wires
    wire [31:0] wi00, wi01, wi02, wi03, wi04, wi05, wi06;
    wire [31:0] wo20, wo21, wo22, wo23, wo24, wo25, wo26, wo27, wo28, wo29,
                wo2a, wo2b, wo2c, wo2d, wo2e, wo3f;
    wire [31:0] ti40, to60;
    wire        poa0_read;
    wire [31:0] poa0_data;

    // ---- open-drain I2C pads
    wire i2c0_scl_low, i2c0_sda_low, i2c1_scl_low, i2c1_sda_low;
    assign I2C_SCL_0 = i2c0_scl_low ? 1'b0 : 1'bz;
    assign I2C_SDA_0 = i2c0_sda_low ? 1'b0 : 1'bz;
    assign I2C_SCL_1 = i2c1_scl_low ? 1'b0 : 1'bz;
    assign I2C_SDA_1 = i2c1_sda_low ? 1'b0 : 1'bz;

    // ---- motor pins
    wire en1, dir1;
    wire [3:0] dbg;                       // {i2c busy, sampler on, fault, running}
    assign PMOD_A1 = en1;   assign PMOD_A2 = dir1;
    assign PMOD_B1 = en1;   assign PMOD_B2 = dir1;
    // bridge 2 copies bridge 1, so the motor runs on either DHB1 motor connector (J2 or J3)
    assign PMOD_A7 = en1;   assign PMOD_A8 = dir1;
    assign PMOD_B7 = en1;   assign PMOD_B8 = dir1;

    lab8_core u_core (
        .clk          (okClk),
        .wi00(wi00), .wi01(wi01), .wi02(wi02), .wi03(wi03),
        .wi04(wi04), .wi05(wi05), .wi06(wi06),
        .ti40(ti40),
        .button_n(button),
        .wo20(wo20), .wo21(wo21), .wo22(wo22), .wo23(wo23), .wo24(wo24),
        .wo25(wo25), .wo26(wo26), .wo27(wo27), .wo28(wo28), .wo29(wo29),
        .wo2a(wo2a), .wo2b(wo2b), .wo2c(wo2c), .wo2d(wo2d), .wo2e(wo2e),
        .to60(to60),
        .po_read      (poa0_read),
        .po_data      (poa0_data),
        .i2c0_scl_low (i2c0_scl_low), .i2c0_sda_low (i2c0_sda_low),
        .i2c1_scl_low (i2c1_scl_low), .i2c1_sda_low (i2c1_sda_low),
        .i2c0_scl_in  (I2C_SCL_0),    .i2c0_sda_in  (I2C_SDA_0),
        .i2c1_scl_in  (I2C_SCL_1),    .i2c1_sda_in  (I2C_SDA_1),
        .en1          (en1),
        .dir1         (dir1),
        .dbg          (dbg)
    );

    assign wo3f = 32'hEC43_7008;          // design ID: Lab 8

    // LEDs: sensor board D0 = EN1, D1 = DIR1, D2 = running, D3 = FAULT
    assign s_LED = {dbg[1], dbg[0], dir1, en1};
    reg [26:0] heartbeat = 27'd0;
    always @(posedge okClk) heartbeat <= heartbeat + 1'b1;
    assign led = ~{heartbeat[26], dbg[1], dbg[0], en1, dir1, dbg[2], dbg[3], 1'b0};   // active low

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
        .wi06_ep_dataout  (wi06),
        .ti40_ep_clk      (okClk),
        .ti40_ep_trigger  (ti40),
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
        .wo3f_ep_datain   (wo3f),
        .to60_ep_clk      (okClk),
        .to60_ep_trigger  (to60),
        .poa0_ep_read     (poa0_read),
        .poa0_ep_datain   (poa0_data)
    );

endmodule


//============================================================================
// lab8_core - motor pulse generator, watchdog, accelerometer sampler, FIFO,
// host I2C access. No FrontPanel IP in here, so the testbench can drive it.
//============================================================================
module lab8_core #(
    parameter integer PERIOD     = 504000,  // clocks per EN1 period: 100.8 MHz / 200 Hz
    parameter integer CLK_PER_PCT= 5040,    // PERIOD / 100: clocks per 1 % duty
    parameter integer GUARD_SAME = 10080,   // EN1-low guard before every move: 100 us
    parameter integer GUARD_DIR  = 100800,  // EN1-low guard after a DIR1 change: 1 ms
    parameter integer MS_DIV     = 100800,  // clocks per ms (watchdog)
    parameter integer TS_DIV     = 63,      // clocks per timestamp tick: 0.625 us
    parameter integer POLL_GAP   = 5040,    // idle between sampler polls: 50 us
    parameter integer I2C_TICK   = 252,     // 2.5 us per I2C state -> 100 kHz SCL
    parameter integer FIFO_AW    = 13       // 8192 words = 2048 samples (~5 s at 400 Hz)
)(
    input  wire        clk,
    input  wire [31:0] wi00, wi01, wi02, wi03, wi04, wi05, wi06,
    input  wire [31:0] ti40,
    input  wire [3:0]  button_n,
    output wire [31:0] wo20, wo21, wo22, wo23, wo24, wo25, wo26, wo27, wo28,
                       wo29, wo2a, wo2b, wo2c, wo2d, wo2e,
    output wire [31:0] to60,
    input  wire        po_read,
    output wire [31:0] po_data,
    output reg         i2c0_scl_low = 1'b0,
    output reg         i2c0_sda_low = 1'b0,
    output reg         i2c1_scl_low = 1'b0,
    output reg         i2c1_sda_low = 1'b0,
    input  wire        i2c0_scl_in, i2c0_sda_in, i2c1_scl_in, i2c1_sda_in,
    output reg         en1  = 1'b0,
    output reg         dir1 = 1'b0,
    output wire [3:0]  dbg
);

    //------------------------------------------------------------------------
    // Reset, triggers
    //------------------------------------------------------------------------
    wire rst       = wi00[1] | ti40[1];
    wire t_i2c     = ti40[0];
    wire t_go      = ti40[2];
    wire t_trip    = ti40[3];
    wire t_clear   = ti40[4];
    wire t_fiforst = ti40[5];
    wire t_kick    = ti40[6];
    wire t_stop    = ti40[7];

    //------------------------------------------------------------------------
    // Timestamp: one tick every TS_DIV clocks (63 / 100.8 MHz = 0.625 us).
    // Free-running, never reset, wraps after 44.7 minutes.
    //------------------------------------------------------------------------
    reg [7:0]  ts_div = 8'd0;
    reg [31:0] ts     = 32'd0;
    always @(posedge clk) begin
        if (ts_div == TS_DIV - 1) begin ts_div <= 8'd0; ts <= ts + 1'b1; end
        else                            ts_div <= ts_div + 1'b1;
    end

    //========================================================================
    // MOTOR PULSE GENERATOR
    //   IDLE  : EN1 = 0. GO latches N (wi04), direction and duty (wi05).
    //           DIR1 is written HERE ONLY, where EN1 is already low.
    //   GUARD : EN1 held low for 100 us (1 ms if DIR1 just changed).
    //   RUN   : EN1 = 1 for duty*PERIOD, 0 for the rest of the 5 ms period.
    //           A pulse is counted at the end of its high time; after the
    //           N-th one EN1 goes low and stays low (motor stopped).
    //   STALL TRIP (from the PC) or WATCHDOG: EN1 low at once, fault latched
    //   with the time and the pulse count. While the fault is latched every
    //   GO is refused, until "clear fault" (after re-homing).
    //========================================================================
    localparam [1:0] M_IDLE = 2'd0, M_GUARD = 2'd1, M_RUN = 2'd2;
    reg [1:0]  mstate     = M_IDLE;
    reg [19:0] pcnt       = 20'd0;      // position inside the 5 ms period
    reg [19:0] high_cyc   = 20'd252000; // EN1-high clocks per period
    reg [7:0]  duty_lat   = 8'd50;
    reg [31:0] target     = 32'd0;      // pulses requested
    reg [31:0] sent       = 32'd0;      // pulses completed
    reg [19:0] guard      = 20'd0;
    reg        fault      = 1'b0;
    reg [3:0]  fault_code = 4'd0;
    reg [31:0] fault_ts   = 32'd0;
    reg [31:0] fault_pul  = 32'd0;
    reg        go_rej     = 1'b0;

    wire [7:0]  duty_req  = wi05[15:8];
    wire [7:0]  duty_eff  = (duty_req == 8'd0) ? 8'd50 : (duty_req > 8'd100) ? 8'd100 : duty_req;
    wire [19:0] pcnt_next = (pcnt == PERIOD - 1) ? 20'd0 : pcnt + 1'b1;
    wire        running   = (mstate != M_IDLE);

    // ---- watchdog: during a move the PC must kick at least every wd_lim ms
    wire        wd_en  = wi06[1];
    wire [15:0] wd_lim = (wi06[31:16] == 16'd0) ? 16'd250 : wi06[31:16];
    reg  [19:0] wd_div = 20'd0;
    reg  [15:0] wd_ms  = 16'd0;
    always @(posedge clk) begin
        if (rst || t_kick || !running) begin
            wd_div <= 20'd0;
            wd_ms  <= 16'd0;
        end else if (wd_div == MS_DIV - 1) begin
            wd_div <= 20'd0;
            if (wd_ms != 16'hFFFF) wd_ms <= wd_ms + 1'b1;
        end else begin
            wd_div <= wd_div + 1'b1;
        end
    end
    wire wd_trip = wd_en && running && (wd_ms >= wd_lim);

    always @(posedge clk) begin
        if (rst) begin
            mstate     <= M_IDLE;
            en1        <= 1'b0;
            dir1       <= 1'b0;          // EN1 goes low in the same clock
            pcnt       <= 20'd0;
            sent       <= 32'd0;
            target     <= 32'd0;
            fault      <= 1'b0;
            fault_code <= 4'd0;
            fault_ts   <= 32'd0;
            fault_pul  <= 32'd0;
            go_rej     <= 1'b0;
        end else begin
            if (t_clear) begin
                fault      <= 1'b0;
                fault_code <= 4'd0;
                go_rej     <= 1'b0;
            end

            if (t_trip || wd_trip) begin
                // SAFE STATE: disabled and coasting
                mstate <= M_IDLE;
                en1    <= 1'b0;
                pcnt   <= 20'd0;
                if (!fault) begin
                    fault      <= 1'b1;
                    fault_code <= t_trip ? 4'd1 : 4'd2;
                    fault_ts   <= ts;
                    fault_pul  <= sent;
                end
            end else if (t_stop) begin
                mstate <= M_IDLE;
                en1    <= 1'b0;
                pcnt   <= 20'd0;
            end else begin
                case (mstate)
                M_IDLE: begin
                    en1 <= 1'b0;
                    if (t_go) begin
                        if (fault || wi04 == 32'd0) begin
                            go_rej <= 1'b1;
                        end else begin
                            target   <= wi04;
                            sent     <= 32'd0;
                            duty_lat <= duty_eff;
                            high_cyc <= duty_eff * CLK_PER_PCT;
                            guard    <= (wi05[0] != dir1) ? GUARD_DIR : GUARD_SAME;
                            dir1     <= wi05[0];          // EN1 is low here
                            mstate   <= M_GUARD;
                        end
                    end
                end
                M_GUARD: begin
                    en1 <= 1'b0;
                    if (guard <= 20'd1) begin
                        mstate <= M_RUN;
                        pcnt   <= 20'd0;
                        en1    <= 1'b1;                   // first pulse starts
                    end else begin
                        guard <= guard - 1'b1;
                    end
                end
                M_RUN: begin
                    if (pcnt == high_cyc - 1'b1) begin    // last high clock of a pulse
                        sent <= sent + 1'b1;
                        if (sent + 1'b1 >= target) begin  // N-th pulse done: stop
                            mstate <= M_IDLE;
                            en1    <= 1'b0;
                            pcnt   <= 20'd0;
                        end else begin
                            pcnt <= pcnt_next;
                            en1  <= (pcnt_next < high_cyc);
                        end
                    end else begin
                        pcnt <= pcnt_next;
                        en1  <= (pcnt_next < high_cyc);
                    end
                end
                default: begin
                    mstate <= M_IDLE;
                    en1    <= 1'b0;
                end
                endcase
            end
        end
    end

    //========================================================================
    // I2C machine, shared by the host (register access while the sampler is
    // off) and the sampler (while it is on).
    //========================================================================
    wire        i2c_ack, i2c_busy, i2c_done, i2c_err;
    wire [7:0]  i2c_state;
    wire [63:0] i2c_rx;
    wire        i2c_marker;
    reg         done_d = 1'b0;
    always @(posedge clk) done_d <= i2c_done;
    wire        done_rise = i2c_done & ~done_d;

    reg         s_start = 1'b0;           // sampler -> I2C
    reg  [31:0] s_p2 = 32'd0, s_p3 = 32'd0;
    wire        samp_owns;

    wire        host_start = t_i2c & ~samp_owns;
    wire        i2c_start  = samp_owns ? s_start : host_start;
    wire [7:0]  i2c_tx     = samp_owns ? 8'h32   : wi01[7:0];   // accel 0x19 + W
    wire [31:0] i2c_p2     = samp_owns ? s_p2    : wi02;
    wire [31:0] i2c_p3     = samp_owns ? s_p3    : wi03;

    wire i2c_scl_low, i2c_sda_low;
    reg  bus_sel = 1'b1;
    always @(posedge clk) begin
        i2c0_scl_low <= bus_sel ? 1'b0 : i2c_scl_low;
        i2c0_sda_low <= bus_sel ? 1'b0 : i2c_sda_low;
        i2c1_scl_low <= bus_sel ? i2c_scl_low : 1'b0;
        i2c1_sda_low <= bus_sel ? i2c_sda_low : 1'b0;
    end
    wire i2c_scl_in = bus_sel ? i2c1_scl_in : i2c0_scl_in;
    wire i2c_sda_in = bus_sel ? i2c1_sda_in : i2c0_sda_in;

    I2C_Transmit #(.TICK_DIVIDE(I2C_TICK)) u_i2c (
        .clk(clk), .rst(rst), .start(i2c_start),
        .tx_byte(i2c_tx), .param2(i2c_p2), .param3(i2c_p3),
        .scl_low(i2c_scl_low), .sda_low(i2c_sda_low),
        .scl_in(i2c_scl_in), .sda_in(i2c_sda_in),
        .ACK_bit(i2c_ack), .rx_data(i2c_rx),
        .busy(i2c_busy), .done(i2c_done), .error_bit(i2c_err),
        .State(i2c_state), .marker(i2c_marker)
    );

    // host-side result words (midterm contract)
    reg  [7:0]  byte_sent  = 8'h00;
    reg  [31:0] cycles     = 32'd0;
    reg  [31:0] xfer_count = 32'd0;
    always @(posedge clk) begin
        if (done_rise) xfer_count <= xfer_count + 1'b1;
        if (rst) begin
            bus_sel   <= 1'b1;
            byte_sent <= 8'h00;
            cycles    <= 32'd0;
        end else if (i2c_start && !i2c_busy) begin
            bus_sel   <= samp_owns ? 1'b1 : wi01[8];
            byte_sent <= i2c_tx;
            cycles    <= 32'd0;
        end else if (i2c_busy) begin
            cycles    <= cycles + 1'b1;
        end
    end

    //========================================================================
    // ACCELEROMETER SAMPLER (free-running, independent of the pulses)
    //   ST_GO/ST_WAIT : 1-byte read of STATUS_REG_A (0x27)     ~0.39 ms
    //   ZYXDA = 0     : wait POLL_GAP, poll again
    //   ZYXDA = 1     : D_GO/D_WAIT: 6-byte burst from 0xA8    ~0.75 ms
    //                   (0x28 | auto-increment), then push 4 words.
    // Reading the data AFTER the status says "ready" means a conversion that
    // lands between the two reads is not lost: we read the newer one. At
    // 400 Hz ODR (2.5 ms) one status+data cycle (~1.2 ms) always fits.
    //========================================================================
    localparam [2:0] SM_IDLE = 3'd0, SM_ST_GO = 3'd1, SM_ST_WAIT = 3'd2,
                     SM_D_GO = 3'd3, SM_D_WAIT = 3'd4, SM_PUSH = 3'd5, SM_GAP = 3'd6;
    reg [2:0]  sm        = SM_IDLE;
    reg [1:0]  push_idx  = 2'd0;
    reg [31:0] sm_ts     = 32'd0;
    reg [7:0]  sm_status = 8'd0;
    reg [47:0] sm_xyz    = 48'd0;
    reg [15:0] gap_cnt   = 16'd0;
    reg [7:0]  seq       = 8'd0;
    reg [31:0] n_samples = 32'd0;
    reg [15:0] n_nack    = 16'd0;
    reg [15:0] n_ovr     = 16'd0;
    reg        fifo_ovf  = 1'b0;
    reg        fifo_we   = 1'b0;
    reg [31:0] fifo_din  = 32'd0;
    wire [FIFO_AW:0] fifo_count;
    wire       samp_en   = wi06[0];
    assign samp_owns = (sm != SM_IDLE);

    always @(posedge clk) begin
        s_start <= 1'b0;
        fifo_we <= 1'b0;
        if (rst) begin
            sm        <= SM_IDLE;
            seq       <= 8'd0;
            n_samples <= 32'd0;
            n_nack    <= 16'd0;
            n_ovr     <= 16'd0;
            fifo_ovf  <= 1'b0;
        end else begin
            if (t_fiforst) fifo_ovf <= 1'b0;
            case (sm)
            SM_IDLE: begin
                if (samp_en && !i2c_busy) sm <= SM_ST_GO;
            end
            SM_ST_GO: begin
                if (!i2c_busy) begin
                    s_p2    <= 32'h0000_0027;                 // STATUS_REG_A, single byte
                    s_p3    <= 32'h0000_0011;                 // register read, 1 byte
                    s_start <= 1'b1;
                    sm      <= SM_ST_WAIT;
                end
            end
            SM_ST_WAIT: begin
                if (done_rise) begin
                    if (i2c_ack || i2c_err) begin
                        if (n_nack != 16'hFFFF) n_nack <= n_nack + 1'b1;
                        sm <= SM_GAP;
                    end else if (i2c_rx[3]) begin             // ZYXDA: new X/Y/Z set
                        sm_status <= i2c_rx[7:0];
                        sm_ts     <= ts;
                        if (i2c_rx[7] && n_ovr != 16'hFFFF) n_ovr <= n_ovr + 1'b1;   // ZYXOR
                        sm        <= SM_D_GO;
                    end else begin
                        sm <= SM_GAP;
                    end
                end
            end
            SM_D_GO: begin
                if (!i2c_busy) begin
                    s_p2    <= 32'h0000_00A8;                 // OUT_X_L_A | auto-increment
                    s_p3    <= 32'h0000_0061;                 // register read, 6 bytes
                    s_start <= 1'b1;
                    sm      <= SM_D_WAIT;
                end
            end
            SM_D_WAIT: begin
                if (done_rise) begin
                    if (i2c_ack || i2c_err) begin
                        if (n_nack != 16'hFFFF) n_nack <= n_nack + 1'b1;
                        sm <= SM_GAP;
                    end else begin
                        sm_xyz   <= i2c_rx[47:0];             // XL XH YL YH ZL ZH
                        push_idx <= 2'd0;
                        sm       <= SM_PUSH;
                    end
                end
            end
            SM_PUSH: begin
                if (push_idx == 2'd0 && fifo_count > (1 << FIFO_AW) - 4) begin
                    fifo_ovf <= 1'b1;                         // no room for a whole sample: drop it
                    sm       <= SM_GAP;
                end else begin
                    fifo_we <= 1'b1;
                    case (push_idx)
                        2'd0: fifo_din <= sm_ts;
                        2'd1: fifo_din <= {sm_xyz[31:16], sm_xyz[15:0]};          // {Y, X}
                        2'd2: fifo_din <= {4'hA, fault, running, dir1, en1, sm_status, sm_xyz[47:32]};
                        2'd3: fifo_din <= {seq, sent[23:0]};
                    endcase
                    push_idx <= push_idx + 1'b1;
                    if (push_idx == 2'd3) begin
                        seq       <= seq + 1'b1;
                        n_samples <= n_samples + 1'b1;
                        sm        <= SM_GAP;
                    end
                end
            end
            SM_GAP: begin
                if (gap_cnt >= POLL_GAP) begin
                    gap_cnt <= 16'd0;
                    sm      <= samp_en ? SM_ST_GO : SM_IDLE;
                end else begin
                    gap_cnt <= gap_cnt + 1'b1;
                end
            end
            default: sm <= SM_IDLE;
            endcase
        end
    end

    sample_fifo #(.AW(FIFO_AW)) u_fifo (
        .clk(clk), .rst(rst | t_fiforst),
        .we(fifo_we), .din(fifo_din),
        .re(po_read), .dout(po_data),
        .count(fifo_count)
    );

    //------------------------------------------------------------------------
    // Result words
    //------------------------------------------------------------------------
    wire i2c_error = i2c_err | (i2c_done & i2c_ack);
    assign wo20 = {15'd0, bus_sel, byte_sent, 7'd0, i2c_ack};
    assign wo21 = i2c_rx[31:0];
    assign wo22 = cycles;
    assign wo23 = {12'd0, ~button_n, i2c_state, 5'd0, i2c_error, i2c_done, i2c_busy};
    assign wo24 = i2c_rx[63:32];
    assign wo25 = xfer_count;
    assign wo26 = {8'd0, duty_lat, 4'd0, wd_en, go_rej, fifo_ovf, samp_owns,
                   fault_code, fault, dir1, en1, running};
    assign wo27 = sent;
    assign wo28 = target;
    assign wo29 = {{(31 - FIFO_AW){1'b0}}, fifo_count};
    assign wo2a = ts;
    assign wo2b = n_samples;
    assign wo2c = fault_ts;
    assign wo2d = fault_pul;
    assign wo2e = {n_ovr, n_nack};
    assign to60 = {31'd0, done_rise};
    assign dbg  = {i2c_busy, samp_owns, fault, running};

endmodule


//============================================================================
// sample_fifo - single-clock FIFO in block RAM, standard (not first-word-
// fall-through) read: dout is valid the clock after re, which is what the
// FrontPanel PipeOut expects (ep_read -> data on the next clock).
//============================================================================
module sample_fifo #(
    parameter integer AW = 13
)(
    input  wire          clk,
    input  wire          rst,
    input  wire          we,
    input  wire [31:0]   din,
    input  wire          re,
    output reg  [31:0]   dout = 32'd0,
    output reg  [AW:0]   count = {(AW+1){1'b0}}
);
    (* ram_style = "block" *) reg [31:0] mem [0:(1 << AW) - 1];
    reg [AW-1:0] wp = {AW{1'b0}};
    reg [AW-1:0] rp = {AW{1'b0}};
    wire do_wr = we && (count != (1 << AW));
    wire do_rd = re && (count != 0);

    always @(posedge clk) begin
        if (do_wr) mem[wp] <= din;
        if (re)    dout    <= mem[rp];
    end

    always @(posedge clk) begin
        if (rst) begin
            wp    <= {AW{1'b0}};
            rp    <= {AW{1'b0}};
            count <= {(AW+1){1'b0}};
        end else begin
            if (do_wr) wp <= wp + 1'b1;
            if (do_rd) rp <= rp + 1'b1;
            case ({do_wr, do_rd})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end
endmodule


//============================================================================
// I2C_Transmit - copied unchanged from midterm/midterm_p1_i2c.v (its header follows).
//============================================================================
// midterm_p1_i2c.v - ECE 437 Midterm Part 1: the I2C state machine (module
// I2C_Transmit), grown from the one generated in Lab 6 so it can talk to the
// LSM303DLHC on I2C bus 1. hdl/SensorBoard_Top.v instantiates it as u_i2c.
//
// Which machine: the Lab 6 generated one (one state per quarter SCL bit,
// advanced by a tick, open-drain scl_low/sda_low). It already did
//     S, addr+W, A, pointer, A, Sr, addr+R, A, ONE data byte, NACK, P
// (ADT7420 register read). The LSM303DLHC needed three changes:
//
//  1. MULTI-BYTE READ (LSM303DLHC.pdf Table 15 p.20): after each received
//     byte the master ACKs (MAK) and clocks the next one; only the last byte
//     gets the NACK (NMAK). States 154-157 are the new master ACK, then back
//     to the receive states 118-149. param3[7:4] = number of bytes (1..8);
//     byte k lands in rx_data[8k+7:8k] (byte 0 in [7:0], as before).
//     The FSM sends the pointer byte exactly as given in param2[7:0]. For the
//     accelerometer the register pointer only advances during a burst if
//     SUB(7) = 1 (p.20: "SUB(7) must be equal to 1"): that bit is the
//     caller's to set (0x28 | 0x80 = 0xA8), so the deliberate fault of 1c is
//     a one-bit change on the host and nothing in here changes.
//
//  2. REGISTER WRITE (Table 11 p.19): S, addr+W, A, SUB, A, DATA, A, P.
//     param3[1] = 1 selects it; the data byte is param2[15:8]. States 158-189
//     send it, 190-193 read its ACK, then the STOP states 39-41. The
//     accelerometer powers up in power-down (CTRL_REG1_A = 0x07, p.24) and the
//     magnetometer in sleep (MR_REG_M = 0x03, p.37), so without a write path
//     neither ever produces data.
//
//  3. SCL RATE AT RUN TIME: param3[31:16] != 0 overrides TICK_DIVIDE for that
//     transaction (latched on start). 0 = the build default: 252 -> 2.5 us per
//     state -> 100 kHz SCL. 63 -> 400 kHz. Used by Part 2 (the bus).
//
//  Plus a scope marker (the Lab 7 idea, now on PMOD_A1 via the top level):
//  `marker` is registered from State and is high for States 110-116, i.e.
//  from the last bit of the READ address through the slave's ACK of it.
//
// TRANSACTIONS (param3[0] = 1):
//   read : S addr+W A SUB A Sr addr+R A D0 MAK D1 MAK ... Dn-1 NMAK P
//   write: S addr+W A SUB A DATA A P
//   param3[0] = 0 keeps the shipped single address frame (S addr A P), so
//   python/smoke_test.py and i2c_first_frame.py still work.
//
// HOW THE LINES WORK (open drain): SCL / SDA below are what WE do to the
// line, 0 = pull low, 1 = release (pull-up or slave decides). sda_in is the
// real wire - always sample that, never SDA.
//
// HANDSHAKE: start (one clock) -> busy = 1 -> ... -> done = 1, busy = 0.
// ACK_bit = 1 if ANY slave ACK slot (address, pointer, read address, write
// data) saw a NACK. error_bit = FSM fell into the default arm.
//============================================================================
module I2C_Transmit #(
    parameter integer TICK_DIVIDE = 252   // clocks per state: 100.8 MHz / 252 = 2.5 us -> 100 kHz SCL
)(
    input  wire        clk,         // 100.8 MHz
    input  wire        rst,         // synchronous, active high
    input  wire        start,       // one-clock pulse: run one transaction (ignored while busy)
    input  wire [7:0]  tx_byte,     // {7-bit slave address, 0}  (param1[7:0]); read address is derived
    input  wire [31:0] param2,      // [7:0] SUB / register pointer (bit 7 = auto-increment on ST parts)
                                    // [15:8] data byte for a register write
    input  wire [31:0] param3,      // [0] register transaction, [1] 1 = write / 0 = read,
                                    // [7:4] bytes to read (1..8, 0 = 1), [31:16] tick divide (0 = default)
    output wire        scl_low,     // 1 = pull SCL low, 0 = release   (open drain)
    output wire        sda_low,     // 1 = pull SDA low, 0 = release
    input  wire        scl_in,      // real level on the SCL wire (for clock stretching)
    input  wire        sda_in,      // real level on the SDA wire (the slave's ACK / data)
    output reg         ACK_bit,     // 0 = every slave ACK slot was an ACK, 1 = a NACK somewhere
    output wire [63:0] rx_data,     // bytes read: byte k in [8k+7:8k]
    output reg         busy,        // 1 while a transaction is on the bus
    output reg         done,        // 1 when the results are valid (until the next start/rst)
    output reg         error_bit,   // 1 = FSM fell into the default arm
    output reg  [7:0]  State,       // 0 idle, 1-2 START, 3-34 address bits, 35-38 ACK, 39-41 STOP,
                                    // 42-73 pointer bits, 74-77 pointer ACK, 78-81 repeated START,
                                    // 82-113 read-address bits, 114-117 read-address ACK,
                                    // 118-149 data bits in, 150-153 master NACK, 154-157 master ACK,
                                    // 158-189 write-data bits, 190-193 write-data ACK
    output reg         marker       // scope marker: high for States 110-116 (last read-address bit + its ACK)
);

    // What we drive on the two lines: 1 = release, 0 = pull low (see header)
    reg SCL = 1'b1;
    reg SDA = 1'b1;
    assign scl_low = ~SCL;
    assign sda_low = ~SDA;
    reg [7:0]  RxByte = 8'h00;         // byte being shifted in (states 118-149)
    reg [63:0] RxBuf  = 64'd0;         // all bytes of the burst
    assign rx_data = RxBuf;

    // Tick generator: one-clock pulse every TickDiv clocks (default 252 = 2.5 us)
    reg [15:0] TickDiv  = TICK_DIVIDE;
    reg [15:0] tick_cnt = 16'd0;
    reg        tick     = 1'b0;
    always @(posedge clk) begin
        if (rst || tick_cnt >= TickDiv - 1'b1) begin
            tick_cnt <= 16'd0;
            tick     <= ~rst;
        end else begin
            tick_cnt <= tick_cnt + 1'b1;
            tick     <= 1'b0;
        end
    end

    // Latched on start
    reg [7:0] SingleByteData = 8'h00;  // address byte {addr7, 0}
    reg       start_pending  = 1'b0;   // start seen, waiting for the next tick
    reg [7:0] RegPointer     = 8'h00;  // SUB byte (param2[7:0])
    reg [7:0] WriteData      = 8'h00;  // data byte of a register write (param2[15:8])
    reg       RegMode        = 1'b0;   // 1 = register transaction (param3[0])
    reg       WriteOp        = 1'b0;   // 1 = register write, 0 = register read (param3[1])
    reg [3:0] NBytes         = 4'd1;   // bytes to read (param3[7:4], 1..8)
    reg [2:0] ByteIdx        = 3'd0;   // byte being received
    reg       ACK_ptr        = 1'b1;   // ACK of the pointer byte: 0 = ACK, 1 = NACK
    reg       ACK_rd         = 1'b1;   // ACK of the read address
    reg       ACK_wr         = 1'b1;   // ACK of the write data byte
    wire [7:0] ReadAddr      = {SingleByteData[7:1], 1'b1};   // 0x32 -> 0x33

    localparam STATE_INIT = 8'd0;

    initial begin
        ACK_bit   = 1'b1;
        busy      = 1'b0;
        done      = 1'b0;
        error_bit = 1'b0;
        State     = STATE_INIT;
        marker    = 1'b0;
    end

    // Scope marker, registered so it is glitch-free: high while State = 110..116
    always @(posedge clk) begin
        marker <= (State >= 8'd110) && (State <= 8'd116);
    end

    always @(posedge clk) begin
        if (rst) begin
            State          <= STATE_INIT;
            SCL            <= 1'b1;
            SDA            <= 1'b1;
            ACK_bit        <= 1'b1;
            busy           <= 1'b0;
            done           <= 1'b0;
            error_bit      <= 1'b0;
            start_pending  <= 1'b0;
            ACK_ptr        <= 1'b1;
            ACK_rd         <= 1'b1;
            ACK_wr         <= 1'b1;
            TickDiv        <= TICK_DIVIDE;
        end else begin
            // ---- accept a start pulse at clock rate, act on it at the next tick
            if (start && !busy) begin
                SingleByteData <= tx_byte;
                RegPointer     <= param2[7:0];
                WriteData      <= param2[15:8];
                RegMode        <= param3[0];
                WriteOp        <= param3[1];
                NBytes         <= (param3[7:4] == 4'd0) ? 4'd1 : (param3[7:4] > 4'd8) ? 4'd8 : param3[7:4];
                TickDiv        <= (param3[31:16] == 16'd0) ? TICK_DIVIDE : (param3[31:16] < 16'd4) ? 16'd4 : param3[31:16];
                ByteIdx        <= 3'd0;
                RxByte         <= 8'h00;
                RxBuf          <= 64'd0;
                ACK_ptr        <= 1'b1;
                ACK_rd         <= 1'b1;
                ACK_wr         <= 1'b1;
                start_pending  <= 1'b1;
                busy           <= 1'b1;
                done           <= 1'b0;
                error_bit      <= 1'b0;
                ACK_bit        <= 1'b1;
            end

            // ---- one state per tick (2.5 us at the default divide)
            if (tick) begin
                case (State)

// ***************************************************
// Idle: both lines released (high). Wait for start.
// ***************************************************
            STATE_INIT : begin  SCL <= 1'b1; SDA <= 1'b1;
                                if (start_pending) begin start_pending <= 1'b0; State <= 8'd1; end   end

// ***************************************************
// Start sequence: SDA falls while SCL is high
// ***************************************************
            8'd1  :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end
            8'd2  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end

// ***************************************************
// Transmit the 1st BYTE: slave address + W (MSB first, 4 states per bit)
// ***************************************************
            8'd3  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[7]; end   // transmit bit 7
            8'd4  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd5  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd6  :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd7  :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[6]; end   // transmit bit 6
            8'd8  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd9  :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd10 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd11 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[5]; end   // transmit bit 5
            8'd12 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd13 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd14 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd15 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[4]; end   // transmit bit 4
            8'd16 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd17 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd18 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd19 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[3]; end   // transmit bit 3
            8'd20 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd21 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd22 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd23 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[2]; end   // transmit bit 2
            8'd24 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd25 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd26 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd27 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[1]; end   // transmit bit 1
            8'd28 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd29 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd30 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd31 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= SingleByteData[0]; end   // transmit bit 0
            8'd32 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd33 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd34 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Release SDA and read the ACK the sensor drives (9th clock)
// ***************************************************
            8'd35 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // RELEASE the line (1 = let go, pull-up/slave decide)
            8'd36 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd37 :  begin  SCL <= 1'b1; State <= State + 1'b1; ACK_bit <= sda_in; end   // read the real wire
            8'd38 :  begin  SCL <= 1'b0; State <= RegMode ? 8'd42 : 8'd39; end   // pointer byte or STOP

// ***************************************************
// Stop sequence: SDA rises while SCL is high
// ***************************************************
            8'd39 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end
            8'd40 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end
            8'd41 :  begin  SCL <= 1'b1; SDA <= 1'b1;                                 // release SDA while SCL is high = STOP
                            busy <= 1'b0; done <= 1'b1; State <= STATE_INIT; end     // transaction finished, back to idle

// ***************************************************
// Transmit the 2nd BYTE: SUB = register pointer (Table 11/15), MSB first.
// For the accelerometer bit 7 is the auto-increment enable - sent as given.
// ***************************************************
            8'd42 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= RegPointer[7]; end   // transmit bit 7
            8'd43 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd44 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd45 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd46 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= RegPointer[6]; end   // transmit bit 6
            8'd47 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd48 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd49 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd50 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= RegPointer[5]; end   // transmit bit 5
            8'd51 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd52 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd53 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd54 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= RegPointer[4]; end   // transmit bit 4
            8'd55 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd56 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd57 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd58 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= RegPointer[3]; end   // transmit bit 3
            8'd59 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd60 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd61 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd62 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= RegPointer[2]; end   // transmit bit 2
            8'd63 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd64 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd65 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd66 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= RegPointer[1]; end   // transmit bit 1
            8'd67 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd68 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd69 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd70 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= RegPointer[0]; end   // transmit bit 0
            8'd71 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd72 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd73 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Release SDA and read the ACK for the pointer byte
// ***************************************************
            8'd74 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // RELEASE the line
            8'd75 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd76 :  begin  SCL <= 1'b1; State <= State + 1'b1;
                            ACK_ptr <= sda_in; ACK_bit <= ACK_bit | sda_in; end    // read the real wire
            8'd77 :  begin  SCL <= 1'b0; State <= WriteOp ? 8'd158 : State + 1'b1; end   // write: data byte; read: Sr

// ***************************************************
// Repeated START: release SDA while SCL is low, raise SCL, then SDA falls
// while SCL is high (= Sr, no STOP before it), SCL falls.
// ***************************************************
            8'd78 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // release SDA, SCL still low
            8'd79 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b1; end   // SCL high, SDA high
            8'd80 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end   // SDA falls while SCL high = Sr
            8'd81 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end

// ***************************************************
// Transmit the 3rd BYTE: the READ address {addr7, 1}, MSB first
// ***************************************************
            8'd82 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= ReadAddr[7]; end   // transmit bit 7
            8'd83 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd84 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd85 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd86 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= ReadAddr[6]; end   // transmit bit 6
            8'd87 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd88 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd89 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd90 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= ReadAddr[5]; end   // transmit bit 5
            8'd91 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd92 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd93 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd94 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= ReadAddr[4]; end   // transmit bit 4
            8'd95 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd96 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd97 :  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd98 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= ReadAddr[3]; end   // transmit bit 3
            8'd99 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd100:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd101:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd102:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= ReadAddr[2]; end   // transmit bit 2
            8'd103:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd104:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd105:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd106:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= ReadAddr[1]; end   // transmit bit 1
            8'd107:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd108:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd109:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd110:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= ReadAddr[0]; end   // transmit bit 0 (R = 1)
            8'd111:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd112:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd113:  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Release SDA and read the ACK for the read address
// ***************************************************
            8'd114:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // RELEASE the line
            8'd115:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd116:  begin  SCL <= 1'b1; State <= State + 1'b1;
                            ACK_rd <= sda_in; ACK_bit <= ACK_bit | sda_in; end     // read the real wire
            8'd117:  begin  SCL <= 1'b0; State <= State + 1'b1; end                // -> receive data byte 0

// ***************************************************
// Receive a DATA BYTE from the slave, MSB first. SDA stays RELEASED so the
// slave can drive it; sample the real wire in the 2nd SCL-high state of
// each bit. Used once per byte of the burst.
// ***************************************************
            8'd118:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // release SDA: slave drives bit 7
            8'd119:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd120:  begin  SCL <= 1'b1; State <= State + 1'b1; RxByte[7] <= sda_in; end   // sample bit 7
            8'd121:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd122:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // slave drives bit 6
            8'd123:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd124:  begin  SCL <= 1'b1; State <= State + 1'b1; RxByte[6] <= sda_in; end   // sample bit 6
            8'd125:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd126:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // slave drives bit 5
            8'd127:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd128:  begin  SCL <= 1'b1; State <= State + 1'b1; RxByte[5] <= sda_in; end   // sample bit 5
            8'd129:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd130:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // slave drives bit 4
            8'd131:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd132:  begin  SCL <= 1'b1; State <= State + 1'b1; RxByte[4] <= sda_in; end   // sample bit 4
            8'd133:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd134:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // slave drives bit 3
            8'd135:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd136:  begin  SCL <= 1'b1; State <= State + 1'b1; RxByte[3] <= sda_in; end   // sample bit 3
            8'd137:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd138:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // slave drives bit 2
            8'd139:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd140:  begin  SCL <= 1'b1; State <= State + 1'b1; RxByte[2] <= sda_in; end   // sample bit 2
            8'd141:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd142:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // slave drives bit 1
            8'd143:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd144:  begin  SCL <= 1'b1; State <= State + 1'b1; RxByte[1] <= sda_in; end   // sample bit 1
            8'd145:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd146:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // slave drives bit 0
            8'd147:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd148:  begin  SCL <= 1'b1; State <= State + 1'b1; RxByte[0] <= sda_in; end   // sample bit 0
            8'd149:  begin  SCL <= 1'b0;                                                    // byte complete: store it
                            RxBuf[{ByteIdx, 3'b000} +: 8] <= RxByte;
                            ByteIdx <= ByteIdx + 1'b1;
                            State <= ({1'b0, ByteIdx} == NBytes - 1'b1) ? 8'd150 : 8'd154; end   // last: NACK, else MAK

// ***************************************************
// Master NACK (9th clock of the LAST byte): SDA stays released - "stop
// sending". The slave lets go of SDA and the STOP can happen.
// ***************************************************
            8'd150:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // NACK = leave SDA released
            8'd151:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd152:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd153:  begin  SCL <= 1'b0; State <= 8'd39; end                       // -> STOP (states 39-41) -> idle

// ***************************************************
// Master ACK (9th clock of every byte but the last, Table 15 MAK): pull
// SDA low so the slave sends the next byte, then receive it (state 118).
// ***************************************************
            8'd154:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end   // MAK = pull SDA low
            8'd155:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd156:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd157:  begin  SCL <= 1'b0; State <= 8'd118; end                      // -> next data byte (118 releases SDA)

// ***************************************************
// Register write: transmit the DATA byte (Table 11), MSB first
// ***************************************************
            8'd158:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[7]; end   // transmit bit 7
            8'd159:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd160:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd161:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd162:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[6]; end   // transmit bit 6
            8'd163:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd164:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd165:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd166:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[5]; end   // transmit bit 5
            8'd167:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd168:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd169:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd170:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[4]; end   // transmit bit 4
            8'd171:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd172:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd173:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd174:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[3]; end   // transmit bit 3
            8'd175:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd176:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd177:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd178:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[2]; end   // transmit bit 2
            8'd179:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd180:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd181:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd182:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[1]; end   // transmit bit 1
            8'd183:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd184:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd185:  begin  SCL <= 1'b0; State <= State + 1'b1; end
            8'd186:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= WriteData[0]; end   // transmit bit 0
            8'd187:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd188:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd189:  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Release SDA and read the ACK for the data byte, then STOP
// ***************************************************
            8'd190:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // RELEASE the line
            8'd191:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd192:  begin  SCL <= 1'b1; State <= State + 1'b1;
                            ACK_wr <= sda_in; ACK_bit <= ACK_bit | sda_in; end     // read the real wire
            8'd193:  begin  SCL <= 1'b0; State <= 8'd39; end                       // -> STOP (states 39-41) -> idle

            // If the FSM ends up here, there was an error in the FSM code:
            // flag it, release the bus and go back to idle.
            default : begin  error_bit <= 1'b1; SCL <= 1'b1; SDA <= 1'b1;
                             busy <= 1'b0; done <= 1'b1; State <= STATE_INIT; end

                endcase
            end
        end
    end

endmodule
