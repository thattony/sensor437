`timescale 1ns / 1ps
//============================================================================
// I2C_Transmit.v - THIS IS THE FILE YOU BRING: paste your completed I2C
// master from the I2C lab here. The port list is FROZEN
// and identical to that lab's (hdl/SensorBoard_Top.v instantiates it as
// u_i2c and owns the pins, the bus selector and the PC endpoints).
//
// ONE TRANSACTION (ADT7420.pdf Figures 14, 16, 17 p.18-19; same for every
// I2C sensor on the board):
//
//   START
//   frame 1  SAD+W     (param1[7:0], e.g. 0x90)                     slave ACK
//   frame 2  register  (param2[7:0])                                slave ACK
//   -- write (param2[16] = 1) ---------------------------------------------
//   frame 3  data      (param2[15:8])                               slave ACK
//   -- read  (param2[16] = 0) ---------------------------------------------
//   REPEATED START
//   frame 3  SAD+R     ({param1[7:1], 1}, e.g. 0x91)                slave ACK
//   receive byte 1 -> master ACK, ..., receive last byte -> master NACK
//   (param3[7:0] bytes, 1..4; 0 behaves as 1, more than 4 as 4)
//   -----------------------------------------------------------------------
//   STOP
//
// Received bytes land in rx_data, first byte in [31:24] (WireOut 0x21).
// ACK_frames records each frame's ACK as sampled in its 9th clock (0 = ACK,
// 1 = NACK; bit 0 = frame 1); a NACK on any frame aborts straight to STOP,
// so a wrong address never hangs the bus. SCL = 100 kHz (one FSM state per
// tick of TICK_DIVIDE clocks). Open drain: scl_low/sda_low = 1 pulls the
// line low, 0 releases it - the FPGA never drives a line high; read the
// real wire on sda_in. State is your state register (WireOut 0x23 [15:8],
// 0 must mean idle).
//
// sim/tb_i2c_transmit.v (7 transactions, one RESULT line) must pass again
// once your machine is in - run `python build.py --sim`. On hardware,
// python/temperature.py is the acceptance test.
//============================================================================
module I2C_Transmit #(
    parameter integer TICK_DIVIDE = 252   // clocks per state: 100.8 MHz / 252 = 2.5 us -> 100 kHz SCL
)(
    input  wire        clk,         // 100.8 MHz
    input  wire        rst,         // synchronous, active high
    input  wire        start,       // one-clock pulse: run one transaction (ignored while busy)
    input  wire [7:0]  tx_byte,     // param1[7:0] = SAD+W = {7-bit slave address, 0}, e.g. 0x90
    input  wire [31:0] param2,      // [7:0] register address, [15:8] data byte to write, [16] 1 = write / 0 = read
    input  wire [31:0] param3,      // [7:0] number of bytes to read (1..4; 0 -> 1, >4 -> 4)
    output wire        scl_low,     // 1 = pull SCL low, 0 = release   (open drain)
    output wire        sda_low,     // 1 = pull SDA low, 0 = release
    input  wire        scl_in,      // real level on the SCL wire (for clock stretching)
    input  wire        sda_in,      // real level on the SDA wire (the slave's ACK / data)
    output reg  [3:0]  ACK_frames,  // bit n = ACK of frame n+1 as sampled in its 9th clock: 0 = ACK, 1 = NACK
    output reg  [31:0] rx_data,     // bytes received, first byte in [31:24]  (WireOut 0x21)
    output reg         busy,        // 1 while the transaction is on the bus
    output reg         done,        // 1 when rx_data / ACK_frames are valid (until the next start/rst)
    output reg         error_bit,   // 1 = FSM error (fell into the default arm)
    output reg  [7:0]  State        // your state register (WireOut 0x23 [15:8], 0 = idle)
);
    // ---- My I2C master from the midterm (midterm/midterm_p1_i2c.v), moved onto
    // this lab's port list. Same machine: one state per quarter SCL bit,
    // advanced by a tick, open-drain lines. What changed for this port list:
    //   * always a register transaction; param2[16] picks write / read
    //   * param3[7:0] = bytes to read, 1..4 (0 -> 1, more -> 4)
    //   * received bytes go to rx_data with the FIRST byte in [31:24]
    //   * one ACK bit per frame in ACK_frames instead of a single ACK_bit, and
    //     a NACK on any frame jumps straight to the STOP states
    //   * the run-time SCL divider and the scope marker are gone (not ports here)
    //
    // State map: 0 idle, 1-2 START, 3-34 address bits, 35-38 ACK, 39-41 STOP,
    // 42-73 pointer bits, 74-77 pointer ACK, 78-81 repeated START,
    // 82-113 read-address bits, 114-117 read-address ACK, 118-149 data bits in,
    // 150-153 master NACK, 154-157 master ACK, 158-189 write-data bits,
    // 190-193 write-data ACK

    // What we drive on the two lines: 1 = release, 0 = pull low
    reg SCL = 1'b1;
    reg SDA = 1'b1;
    assign scl_low = ~SCL;
    assign sda_low = ~SDA;
    reg [7:0] RxByte = 8'h00;          // byte being shifted in (states 118-149)

    // Tick generator: one-clock pulse every TICK_DIVIDE clocks (252 = 2.5 us)
    reg [15:0] tick_cnt = 16'd0;
    reg        tick     = 1'b0;
    always @(posedge clk) begin
        if (rst || tick_cnt == TICK_DIVIDE - 1) begin
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
    reg [7:0] RegPointer     = 8'h00;  // register address (param2[7:0])
    reg [7:0] WriteData      = 8'h00;  // data byte of a register write (param2[15:8])
    reg       WriteOp        = 1'b0;   // 1 = register write, 0 = register read (param2[16])
    reg [2:0] NBytes         = 3'd1;   // bytes to read (1..4)
    reg [1:0] ByteIdx        = 2'd0;   // byte being received
    wire [7:0] ReadAddr      = {SingleByteData[7:1], 1'b1};   // 0x90 -> 0x91

    localparam STATE_INIT = 8'd0;

    initial begin
        ACK_frames = 4'd0;
        rx_data    = 32'd0;
        busy       = 1'b0;
        done       = 1'b0;
        error_bit  = 1'b0;
        State      = STATE_INIT;
    end

    always @(posedge clk) begin
        if (rst) begin
            State          <= STATE_INIT;
            SCL            <= 1'b1;
            SDA            <= 1'b1;
            ACK_frames     <= 4'd0;
            rx_data        <= 32'd0;
            busy           <= 1'b0;
            done           <= 1'b0;
            error_bit      <= 1'b0;
            start_pending  <= 1'b0;
        end else begin
            // ---- accept a start pulse at clock rate, act on it at the next tick
            if (start && !busy) begin
                SingleByteData <= tx_byte;
                RegPointer     <= param2[7:0];
                WriteData      <= param2[15:8];
                WriteOp        <= param2[16];
                NBytes         <= (param3[7:0] == 8'd0) ? 3'd1 : (param3[7:0] > 8'd4) ? 3'd4 : param3[2:0];
                ByteIdx        <= 2'd0;
                RxByte         <= 8'h00;
                rx_data        <= 32'd0;
                ACK_frames     <= 4'd0;
                start_pending  <= 1'b1;
                busy           <= 1'b1;
                done           <= 1'b0;
                error_bit      <= 1'b0;
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
            8'd37 :  begin  SCL <= 1'b1; State <= State + 1'b1; ACK_frames[0] <= sda_in; end   // read the real wire
            8'd38 :  begin  SCL <= 1'b0; State <= ACK_frames[0] ? 8'd39 : 8'd42; end   // NACK: abort to STOP, else pointer byte

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
                            ACK_frames[1] <= sda_in; end                           // read the real wire
            8'd77 :  begin  SCL <= 1'b0; State <= ACK_frames[1] ? 8'd39 : WriteOp ? 8'd158 : State + 1'b1; end   // NACK: STOP; write: data byte; read: Sr

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
                            ACK_frames[2] <= sda_in; end                           // read the real wire
            8'd117:  begin  SCL <= 1'b0; State <= ACK_frames[2] ? 8'd39 : State + 1'b1; end   // NACK: STOP, else receive byte 0

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
                            rx_data[(5'd31 - {ByteIdx, 3'b000}) -: 8] <= RxByte;       // first byte in [31:24]
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
                            ACK_frames[2] <= sda_in; end                           // read the real wire
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
