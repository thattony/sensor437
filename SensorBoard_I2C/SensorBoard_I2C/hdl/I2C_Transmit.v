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
//   will add - unused today.
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
    output wire [31:0] rx_data,     // WireOut 0x21 - the bytes you will read (0 today)
    output reg         busy,        // 1 while a frame is on the bus
    output reg         done,        // 1 when ACK_bit is valid (until the next start/rst)
    output reg         error_bit,   // 1 = FSM fell into the default arm
    output reg  [7:0]  State        // 0 idle, 1-2 START, 3-34 data bits, 35-38 ACK, 39-41 STOP
);

    // What we drive on the two lines: 1 = release, 0 = pull low (see header)
    reg SCL = 1'b1;
    reg SDA = 1'b1;
    assign scl_low = ~SCL;
    assign sda_low = ~SDA;
    assign rx_data = 32'd0;            // nothing is read yet

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
        end else begin
            // ---- accept a start pulse at clock rate, act on it at the next tick
            if (start && !busy) begin
                SingleByteData <= tx_byte;
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
            8'd38 :  begin  SCL <= 1'b0; State <= State + 1'b1; end

// ***************************************************
// Stop sequence: SDA rises while SCL is high
// ***************************************************
            8'd39 :  begin  SCL <= 1'b0; State <= State + 1'b1; SDA <= 1'b0; end
            8'd40 :  begin  SCL <= 1'b1; State <= State + 1'b1; SDA <= 1'b0; end
            8'd41 :  begin  SCL <= 1'b1; SDA <= 1'b1;                                 // release SDA while SCL is high = STOP
                            busy <= 1'b0; done <= 1'b1; State <= STATE_INIT; end     // frame finished, back to idle

            // If the FSM ends up here, there was an error in the FSM code:
            // flag it, release the bus and go back to idle.
            default : begin  error_bit <= 1'b1; SCL <= 1'b1; SDA <= 1'b1;
                             busy <= 1'b0; done <= 1'b1; State <= STATE_INIT; end

                endcase
            end
        end
    end

endmodule
