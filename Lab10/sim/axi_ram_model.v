`timescale 1ns / 1ps
//============================================================================
// axi_ram_model.v - AXI4 slave standing in for the MIG DDR3 controller in
//                   simulation: 128-bit data, INCR bursts, one write and one
//                   read transaction at a time, 4 MiB of memory (address bits
//                   [21:4] select the beat; the three 1 MiB frame slots fit).
//   Adds a few wait states so the masters' handshakes are exercised.
//============================================================================
module axi_ram_model #(
    parameter integer AW = 30,
    parameter integer MEM_BEATS_LOG2 = 18
)(
    input  wire          clk,
    input  wire          rst,
    input  wire [3:0]    s_axi_awid,
    input  wire [AW-1:0] s_axi_awaddr,
    input  wire [7:0]    s_axi_awlen,
    input  wire [2:0]    s_axi_awsize,
    input  wire [1:0]    s_axi_awburst,
    input  wire          s_axi_awvalid,
    output reg           s_axi_awready,
    input  wire [127:0]  s_axi_wdata,
    input  wire [15:0]   s_axi_wstrb,
    input  wire          s_axi_wlast,
    input  wire          s_axi_wvalid,
    output reg           s_axi_wready,
    output reg  [3:0]    s_axi_bid,
    output reg  [1:0]    s_axi_bresp,
    output reg           s_axi_bvalid,
    input  wire          s_axi_bready,
    input  wire [3:0]    s_axi_arid,
    input  wire [AW-1:0] s_axi_araddr,
    input  wire [7:0]    s_axi_arlen,
    input  wire [2:0]    s_axi_arsize,
    input  wire [1:0]    s_axi_arburst,
    input  wire          s_axi_arvalid,
    output reg           s_axi_arready,
    output reg  [3:0]    s_axi_rid,
    output reg  [127:0]  s_axi_rdata,
    output reg  [1:0]    s_axi_rresp,
    output reg           s_axi_rlast,
    output reg           s_axi_rvalid,
    input  wire          s_axi_rready
);
    reg [127:0] mem [0:(1<<MEM_BEATS_LOG2)-1];

    // ---- write ----
    reg        w_active = 1'b0;
    reg [AW-1:0] w_addr;
    reg [3:0]  w_id;
    reg [7:0]  w_cnt;
    integer    k;
    initial begin s_axi_awready = 1'b0; s_axi_wready = 1'b0; s_axi_bvalid = 1'b0; s_axi_bid = 0; s_axi_bresp = 0; end
    always @(posedge clk) begin
        s_axi_awready <= 1'b0;
        if (rst) begin w_active <= 1'b0; s_axi_wready <= 1'b0; s_axi_bvalid <= 1'b0; end
        else begin
            if (!w_active && !s_axi_bvalid && s_axi_awvalid && !s_axi_awready) begin
                s_axi_awready <= 1'b1;
            end
            if (s_axi_awvalid && s_axi_awready) begin
                w_active <= 1'b1; w_addr <= s_axi_awaddr; w_id <= s_axi_awid; w_cnt <= s_axi_awlen;
                s_axi_wready <= 1'b1;
            end
            if (w_active && s_axi_wvalid && s_axi_wready) begin
                for (k = 0; k < 16; k = k + 1)
                    if (s_axi_wstrb[k]) mem[w_addr[MEM_BEATS_LOG2+3:4]][8*k +: 8] <= s_axi_wdata[8*k +: 8];
                w_addr <= w_addr + 16;
                if (s_axi_wlast || w_cnt == 8'd0) begin
                    w_active <= 1'b0; s_axi_wready <= 1'b0; s_axi_bvalid <= 1'b1; s_axi_bid <= w_id; s_axi_bresp <= 2'b00;
                end else w_cnt <= w_cnt - 8'd1;
            end
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
        end
    end

    // ---- read ----
    reg        r_active = 1'b0;
    reg [AW-1:0] r_addr;
    reg [7:0]  r_cnt;
    reg [2:0]  r_wait;
    initial begin s_axi_arready = 1'b0; s_axi_rvalid = 1'b0; s_axi_rlast = 1'b0; s_axi_rid = 0; s_axi_rresp = 0; s_axi_rdata = 0; end
    always @(posedge clk) begin
        s_axi_arready <= 1'b0;
        if (rst) begin r_active <= 1'b0; s_axi_rvalid <= 1'b0; end
        else begin
            if (!r_active && s_axi_arvalid && !s_axi_arready) s_axi_arready <= 1'b1;
            if (s_axi_arvalid && s_axi_arready) begin
                r_active <= 1'b1; r_addr <= s_axi_araddr; r_cnt <= s_axi_arlen; s_axi_rid <= s_axi_arid; r_wait <= 3'd3;
            end
            if (r_active) begin
                if (!s_axi_rvalid) begin
                    if (r_wait != 3'd0) r_wait <= r_wait - 3'd1;
                    else begin
                        s_axi_rdata <= mem[r_addr[MEM_BEATS_LOG2+3:4]]; s_axi_rresp <= 2'b00;
                        s_axi_rlast <= (r_cnt == 8'd0); s_axi_rvalid <= 1'b1;
                    end
                end else if (s_axi_rready) begin
                    s_axi_rvalid <= 1'b0; r_addr <= r_addr + 16; r_wait <= 3'd1;
                    if (r_cnt == 8'd0) begin r_active <= 1'b0; s_axi_rlast <= 1'b0; end
                    else r_cnt <= r_cnt - 8'd1;
                end
            end
        end
    end
endmodule
