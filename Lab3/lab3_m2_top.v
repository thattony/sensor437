`timescale 1ns / 1ps

module lab3_m2_top(
    input  wire [4:0]  okUH,
    output wire [2:0]  okHU,
    inout  wire [31:0] okUHU,
    inout  wire        okAA,
    input  wire        sys_clkn,
    input  wire        sys_clkp,
    output wire [7:0]  led
);

    wire okClk;
    wire [112:0] okHE;
    wire [64:0] okEH;

    okHost hostIF (
        .okUH(okUH),
        .okHU(okHU),
        .okUHU(okUHU),
        .okClk(okClk),
        .okAA(okAA),
        .okHE(okHE),
        .okEH(okEH)
    );

    wire [64:0] okEHx;
    assign okEHx = 65'b0;

    okWireOR #(
        .N(1)
    ) wireOR (
        okEH,
        okEHx
    );

    wire [31:0] control;

    okWireIn wire00 (
        .okHE(okHE),
        .ep_addr(8'h00),
        .ep_dataout(control)
    );

    wire clk;

    IBUFGDS osc_clk (
        .O(clk),
        .I(sys_clkp),
        .IB(sys_clkn)
    );

    wire [7:0] lights;

    lab3_traffic_fsm fsm (
        .clk(clk),
        .ped_request(control[0]),
        .lights(lights)
    );

    assign led = ~lights;

endmodule