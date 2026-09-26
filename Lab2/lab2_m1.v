`timescale 1ns / 1ps

module lab2_example(
        input   wire    [4:0]  okUH,
        output  wire    [2:0]  okHU,
        inout   wire    [31:0] okUHU,
        inout   wire           okAA,
        input   wire           sys_clkn,
        input   wire           sys_clkp,
        input   wire           reset,
        // Your signals go here
        input   [3:0] button,
        output  [7:0] led
    );

    wire okClk;              // FrontPanel wires needed for IO communication
    wire [112:0] okHE;       // FrontPanel wires needed for IO communication
    wire [64:0]  okEH;       // FrontPanel wires needed for IO communication

    // Declare your registers or wires to send or receive data
    wire [31:0] variable_1, variable_2, variable_3;   // outputs from a module must be wires
    wire [31:0] result_wire;              // signals into modules can be wires or registers
    reg  [31:0] result_register;          // signals into modules can be wires or registers
    
    // Milestone
    wire [31:0] mode;
    reg direction;

    // This is the OK host that allows data to be sent or received
    okHost hostIF (
        .okUH(okUH),
        .okHU(okHU),
        .okUHU(okUHU),
        .okClk(okClk),
        .okAA(okAA),
        .okHE(okHE),
        .okEH(okEH)
    );

    // Adjust endPt_count to the number of outgoing endpoints.
    // Here we have 2 output endpoints, hence endPt_count = 2.
    localparam endPt_count = 3;
    wire [endPt_count*65-1:0] okEHx;
    okWireOR # (.N(endPt_count)) wireOR (okEH, okEHx);

    // Clock
    localparam DIVIDE = 20000000;   // fast-clock cycles between slow_clk toggles

    wire clk;
    reg [31:0] clkdiv;
    reg        slow_clk;
    reg [7:0]  counter;

    IBUFGDS osc_clk(
        .O(clk),
        .I(sys_clkp),
        .IB(sys_clkn)
    );

    initial begin
        clkdiv   = 0;
        slow_clk = 0;
        counter  = 8'h00;
        direction = 0;
    end

    // clkdiv counts 0 .. DIVIDE-1, which is exactly DIVIDE cycles per toggle.
    always @(posedge clk) begin
        if (clkdiv == DIVIDE - 1) begin
            clkdiv   <= 0;
            slow_clk <= ~slow_clk;
        end
        else begin
            clkdiv <= clkdiv + 1'b1;
        end
    end
    
    // variable_1 holds data sent from the PC, received at address 0x00
    okWireIn wire00 (   .okHE(okHE),
                        .ep_addr(8'h00),
                        .ep_dataout(variable_1));

    // variable_2 holds data sent from the PC, received at address 0x01
    okWireIn wire01 (   .okHE(okHE),
                        .ep_addr(8'h01),
                        .ep_dataout(variable_2));
    
    okWireIn wire02 (   .okHE(okHE),
                        .ep_addr(8'h02),
                        .ep_dataout(variable_3));
    
    // Milestone
    assign mode = variable_3;
    assign led =
    (mode == 32'd0) ? 8'h00 :   // LED all on
    (mode == 32'd1) ? 8'hFF :   // LED all off
    (mode == 32'd2) ? ~counter :    // show counter val
    (mode == 32'd3) ? ~counter :    // show counter val
                      8'hFF;
    
    // The counter and direction: 0 UP, 1 DOWN
    always @(posedge slow_clk) begin
        if (mode == 32'd2) begin
            counter <= counter + 8'd2;
            direction <= 0;
        end
        else if (mode == 32'd3) begin
            counter <= counter - 8'd2;
            direction <= 1;
        end else begin
            if (direction) begin
                counter <= counter - 8'd2;
            end 
            else begin
                counter <= counter + 8'd2;
            end
        end
    end

    // The sum is stored in a WIRE. A wire needs no clock, so we use assign
    // and the result is available as soon as the inputs change.
    assign result_wire = variable_1 + variable_2;

    // result_wire is transmitted to the PC via address 0x20
    okWireOut wire20 (  .okHE(okHE),
                        .okEH(okEHx[ 0*65 +: 65 ]),
                        .ep_addr(8'h20),
                        .ep_datain(result_wire));

    // The difference is stored in a REGISTER. A register updates only on a clock
    // edge, so this result does not appear until the next rising edge of slow_clk.
    always @(posedge slow_clk) begin
        result_register <= variable_1 - variable_2;
    end

    // result_register is transmitted to the PC via address 0x21
    okWireOut wire21 (  .okHE(okHE),
                        .okEH(okEHx[ 1*65 +: 65 ]),
                        .ep_addr(8'h21),
                        .ep_datain(result_register));
                        
    okWireOut wire22 (  .okHE(okHE),
                        .okEH(okEHx[ 2*65 +: 65 ]),
                        .ep_addr(8'h22),
                        .ep_datain(counter));
endmodule
