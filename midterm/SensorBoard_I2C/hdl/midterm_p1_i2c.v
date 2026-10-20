`timescale 1ns / 1ps
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
