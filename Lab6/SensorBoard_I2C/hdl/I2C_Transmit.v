`timescale 1ns / 1ps
//============================================================================
// I2C_Transmit.v - THIS IS THE FILE YOU EDIT: the first frame of an I2C
// transaction. (hdl/SensorBoard_Top.v instantiates it as u_i2c and owns the
// pins, the bus selector and the PC endpoints - you do not need to touch it.)
//
// What it does (one "frame"):
//
//     START -> 8 bits of tx_byte, MSB first -> 9th clock: read the ACK -> STOP
//
// tx_byte is the 7-bit slave address followed by the R/W bit (ADT7420.pdf
// "Serial Bus Address", p.17, Table 20: address 0x48 -> 0x90 = write,
// 0x91 = read). A slave that recognises its address pulls SDA low during
// the 9th clock (ACK_bit = 0). If nobody answers, the 10 k pull-up keeps
// SDA high (ACK_bit = 1 = NACK) - that is what a wrong address looks like.
//
// That is ALL it does: no register-pointer byte, no data bytes, no repeated
// START. Growing this state machine into the complete transactions of your
// sensor's datasheet is the assignment. For the ADT7420 (ADT7420.pdf):
//   * Figure 14, p.18  register write : S, addr+W, ACK, pointer, ACK, data, ACK, P
//   * Figure 16, p.19  register read  : S, addr+W, ACK, pointer, ACK,
//                                       Sr, addr+R, ACK, data, NACK (by master), P
//   * Figure 17, p.19  2-byte temperature read (master ACKs the MSB, NACKs the LSB)
//   * Table 2 / Figure 2, p.5  timing limits (f_SCL <= 400 kHz, t_LOW >= 1.3 us ...)
// The other sensors' figures/tables are listed in docs/SENSOR_BOARD.md.
//
// Send the WRITE address (0x90) as a stand-alone first frame, not the READ
// address (0x91): after ACKing addr+R the slave immediately starts driving
// data bit 7 on SDA (Figure 17). If that bit is 0 the slave holds SDA low,
// your STOP (SDA rising while SCL is high) cannot happen and the bus stays
// stuck until the byte is clocked out and NACKed. sim/tb_i2c_transmit.v shows
// this happening in the log.
//
// HOW THE TIMING WORKS
//   The state machine advances one state per `tick`, made below from the
//   100.8 MHz clock: TICK_DIVIDE = 252 -> one tick every 2.5 us.
//   One SCL period is four states (low + set SDA, high, high, low) =
//   10 us = 100 kHz SCL, inside the 400 kHz limit of every sensor on the
//   board. t_LOW = t_HIGH = 5 us; SDA changes 2.5 us after SCL fell
//   (t_HD;DAT) and 2.5 us before it rises (t_SU;DAT); START hold and STOP
//   setup are 2.5 us. TICK_DIVIDE (set in SensorBoard_Top.v) scales all of
//   it: 126 -> 200 kHz, 63 -> 400 kHz. A one-clock enable pulse is used
//   instead of a divided clock so the whole design stays on one clock.
//
// HOW THE LINES WORK (open drain, ADT7420.pdf Table 4 p.7)
//   SCL / SDA below are what WE do to the line:
//       0 = pull the line low
//       1 = RELEASE it - the pull-up makes it high unless another device
//           (the slave, during ACK or data) is holding it low.
//   The FPGA never drives a line high. sda_in / scl_in are the real levels
//   on the wires: always read those, never SDA/SCL - while we are released
//   our own register says nothing about what the slave is doing.
//   (SensorBoard_Top.v turns scl_low/sda_low into tristate pads.)
//
// HANDSHAKE with SensorBoard_Top.v (and through it with the PC):
//   start (one clock) -> busy = 1 -> ... 41 states ... -> done = 1, busy = 0
//   ACK_bit is valid when done. error_bit = 1 means the FSM reached an
//   undefined state (the `default:` arm) - a bug in the state list, not a
//   bus error. State is exported so Python can see where a stuck FSM sits.
//   param2 / param3 / rx_data are already routed to WireIn 0x02/0x03 and
//   WireOut 0x21 for the register address, byte count and data bytes you
//   will add.
//
// LAB 6 MILESTONE 2 - STEP 1: FRAME 2, THE REGISTER POINTER (Figure 14/16)
//   param3[0] = 0 : the shipped single frame (S, tx_byte, ACK, P) - unchanged,
//                   so python/smoke_test.py (param2 = param3 = 0) still passes.
//   param3[0] = 1 : S, tx_byte (0x90), ACK, param2[7:0] (pointer, e.g. 0x0B),
//                   ACK, P. States 42-73 send the pointer, 74-77 its ACK, then
//                   the existing STOP states 39-41.
//   ACK_bit = 1 if EITHER frame was NACKed (so the top level's error works);
//   ACK_ptr = the pointer frame's ACK alone.
//
// STEP 2: REPEATED START + READ ADDRESS (Figure 16 p.19)
//   param3[0] = 1 now continues after the pointer ACK: states 78-81 repeated
//   START, 82-113 read address {tx_byte[7:1], 1} (0x90 -> 0x91), 114-117 its
//   ACK (ACK_rd; also OR-ed into ACK_bit). State 117 jumps to the STOP states
//   39-41 for now - receiving the data byte + master NACK is step 3.
//
// STEP 3: DATA BYTE + MASTER NACK + STOP (Figure 16 p.19)
//   States 118-149 receive one byte MSB first with SDA released, sampling
//   sda_in in the 2nd SCL-high state of each bit -> RxByte -> rx_data[7:0]
//   (= result1[7:0]). States 150-153: master NACK (SDA released in the 9th
//   clock), then the STOP states 39-41 and idle.
//   Full transaction: S 0x90 A 0x0B A Sr 0x91 A data N P  (153 ticks).
//============================================================================
module I2C_Transmit #(
    parameter integer TICK_DIVIDE = 252   // clocks per state: 100.8 MHz / 252 = 2.5 us -> 100 kHz SCL
)(
    input  wire        clk,         // 100.8 MHz
    input  wire        rst,         // synchronous, active high
    input  wire        start,       // one-clock pulse: send one frame (ignored while busy)
    input  wire [7:0]  tx_byte,     // byte to send = {7-bit slave address, R/W}  (param1[7:0])
    input  wire [31:0] param2,      // WireIn 0x02 - free for your protocol (unused today)
    input  wire [31:0] param3,      // WireIn 0x03 - free for your protocol (unused today)
    output wire        scl_low,     // 1 = pull SCL low, 0 = release   (open drain)
    output wire        sda_low,     // 1 = pull SDA low, 0 = release
    input  wire        scl_in,      // real level on the SCL wire (for clock stretching)
    input  wire        sda_in,      // real level on the SDA wire (the slave's ACK / data)
    output reg         ACK_bit,     // sampled in the 9th clock: 0 = ACK, 1 = NACK
    output wire [31:0] rx_data,     // WireOut 0x21 - [7:0] = data byte read (param3[0] = 1), else 0
    output reg         busy,        // 1 while a frame is on the bus
    output reg         done,        // 1 when ACK_bit is valid (until the next start/rst)
    output reg         error_bit,   // 1 = FSM fell into the default arm
    output reg  [7:0]  State        // 0 idle, 1-2 START, 3-34 data bits, 35-38 ACK, 39-41 STOP,
                                    // 42-73 pointer bits, 74-77 pointer ACK, 78-81 repeated START,
                                    // 82-113 read-address bits, 114-117 read-address ACK,
                                    // 118-149 data bits in, 150-153 master NACK
);

    // What we drive on the two lines: 1 = release, 0 = pull low (see header)
    reg SCL = 1'b1;
    reg SDA = 1'b1;
    assign scl_low = ~SCL;
    assign sda_low = ~SDA;
    reg [7:0] RxByte = 8'h00;          // data byte read from the slave (states 118-149)
    assign rx_data = {24'd0, RxByte};  // -> result1[7:0]

    // Tick generator: one-clock pulse every TICK_DIVIDE clocks (= 2.5 us)
    localparam integer TW = (TICK_DIVIDE <= 2) ? 1 : $clog2(TICK_DIVIDE);
    reg [TW-1:0] tick_cnt = {TW{1'b0}};
    reg          tick     = 1'b0;
    always @(posedge clk) begin
        if (rst || tick_cnt == TICK_DIVIDE - 1) begin
            tick_cnt <= {TW{1'b0}};
            tick     <= ~rst;
        end else begin
            tick_cnt <= tick_cnt + 1'b1;
            tick     <= 1'b0;
        end
    end

    // The byte being shifted out (latched from tx_byte on start)
    reg [7:0] SingleByteData = 8'h00;
    reg       start_pending  = 1'b0;   // start seen, waiting for the next tick
    reg [7:0] RegPointer     = 8'h00;  // frame 2 byte (latched from param2[7:0] on start)
    reg       RegMode        = 1'b0;   // 1 = send frame 2 (latched from param3[0] on start)
    reg       ACK_ptr        = 1'b1;   // ACK of frame 2: 0 = ACK, 1 = NACK
    reg       ACK_rd         = 1'b1;   // ACK of frame 3 (read address)
    wire [7:0] ReadAddr      = {SingleByteData[7:1], 1'b1};   // 0x90 -> 0x91

    localparam STATE_INIT = 8'd0;

    initial begin
        ACK_bit   = 1'b1;
        busy      = 1'b0;
        done      = 1'b0;
        error_bit = 1'b0;
        State     = STATE_INIT;
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
        end else begin
            // ---- accept a start pulse at clock rate, act on it at the next tick
            if (start && !busy) begin
                SingleByteData <= tx_byte;
                RegPointer     <= param2[7:0];
                RxByte         <= 8'h00;
                RegMode        <= param3[0];
                ACK_ptr        <= 1'b1;
                ACK_rd         <= 1'b1;
                start_pending  <= 1'b1;
                busy           <= 1'b1;
                done           <= 1'b0;
                error_bit      <= 1'b0;
                ACK_bit        <= 1'b1;
            end

            // ---- one state per tick (2.5 us)
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
// Transmit the 1st BYTE to a sensor (MSB first, 4 states per bit)
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
            8'd38 :  begin  SCL <= 1'b0; State <= RegMode ? 8'd42 : 8'd39; end   // frame 2 (pointer) or STOP

// ***************************************************
// Stop sequence: SDA rises while SCL is high
// ***************************************************
            8'd39 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end
            8'd40 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end
            8'd41 :  begin  SCL <= 1'b1; SDA <= 1'b1;                                 // release SDA while SCL is high = STOP
                            busy <= 1'b0; done <= 1'b1; State <= STATE_INIT; end     // frame finished, back to idle

// ***************************************************
// Transmit the 2nd BYTE: the register pointer (ADT7420.pdf Figure 14 p.18 /
// Figure 16 p.19), MSB first, 4 states per bit - same pattern as states 3-34
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
// Release SDA and read the ACK for the pointer byte (9th clock of frame 2)
// ***************************************************
            8'd74 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // RELEASE the line
            8'd75 :  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd76 :  begin  SCL <= 1'b1; State <= State + 1'b1;
                            ACK_ptr <= sda_in; ACK_bit <= ACK_bit | sda_in; end    // read the real wire
            8'd77 :  begin  SCL <= 1'b0; State <= State + 1'b1; end                // -> repeated START

// ***************************************************
// Repeated START (Figure 16 p.19): release SDA while SCL is low, raise SCL,
// then SDA falls while SCL is high (= Sr, no STOP before it), SCL falls.
// t_SU;STA = t_HD;STA = 2.5 us (Table 2 p.5: >= 0.6 us).
// ***************************************************
            8'd78 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // release SDA, SCL still low
            8'd79 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b1; end   // SCL high, SDA high
            8'd80 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end   // SDA falls while SCL high = Sr
            8'd81 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end

// ***************************************************
// Transmit the 3rd BYTE: the READ address {addr7, 1} = 0x91, MSB first
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
// Release SDA and read the ACK for the read address (9th clock of frame 3)
// ***************************************************
            8'd114:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // RELEASE the line
            8'd115:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd116:  begin  SCL <= 1'b1; State <= State + 1'b1;
                            ACK_rd <= sda_in; ACK_bit <= ACK_bit | sda_in; end     // read the real wire
            8'd117:  begin  SCL <= 1'b0; State <= State + 1'b1; end                // -> receive the data byte

// ***************************************************
// Receive the DATA BYTE from the slave (Figure 16 p.19), MSB first.
// SDA stays RELEASED (1) so the slave can drive it; it changes its bit while
// SCL is low, we sample the real wire in the 2nd SCL-high state of each bit
// (same place as the ACK samples in states 37 / 76 / 116).
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
            8'd149:  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Master NACK (9th clock): SDA stays released (high) - "last byte, stop
// sending". The slave sees NACK, stops driving, and the STOP can happen.
// ***************************************************
            8'd150:  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b1; end   // NACK = leave SDA released
            8'd151:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd152:  begin  SCL <= 1'b1; State <= State + 1'b1; end
            8'd153:  begin  SCL <= 1'b0; State <= 8'd39; end                       // -> STOP (states 39-41) -> idle

            // If the FSM ends up here, there was an error in the FSM code:
            // flag it, release the bus and go back to idle.
            default : begin  error_bit <= 1'b1; SCL <= 1'b1; SDA <= 1'b1;
                             busy <= 1'b0; done <= 1'b1; State <= STATE_INIT; end

                endcase
            end
        end
    end

endmodule
