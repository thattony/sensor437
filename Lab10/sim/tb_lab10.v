`timescale 1ns / 1ps
//============================================================================
// tb_cmv300.v - Testbench for hdl/CMV300_Subsystem.v (Phases 0, 1 and 2 of
//               docs/CMV300_CAPTURE_PLAN.md) against sim/cmv300_model.v and
//               sim/axi_ram_model.v (in place of the MIG).
//
// Run:  ./build.sh --sim   (runs tb_i2c_transmit first, then this; log sim/sim.log)
//
// The subsystem is driven exactly like the PC does it: WireIn 0x04/0x05/0x06
// registers, TriggerIn 0x41 pulses, WireOut 0x24..0x2D, PipeOut and BTPipeOut
// reads (a read strobe at edge n -> the IP latches the word at edge n+1; the
// BTPipe reads whole 1024-byte blocks only while ep_ready is high).
//
//   1. cam_enable -> sequencer reaches READY, SYS_RES_N high, CLK_IN running
//   2. SPI read of the datasheet defaults (regs 1, 2, 42, 61, 83)
//   3. SPI write / read-back (reg 1), parallel-mode registers 57/68/69/83
//   4. burst read of the temperature registers 78/79 (0x1464)
//   5. cam_reset -> sequencer cycles, registers back to defaults
//   6. CLK_OUT measurement = 25,000 +/- 16 per ms once reg 69 = 9
//   7. 12-row frame: geometry 648 x 12, counters, OR/AND masks, snapshot of 2,592 words read back
//   8. ring, sensor frames of 12 rows: acquisition fills 3 slots (3 FRAME_REQs), read_req ->
//      16 blocks, BTPipe read, pixels + zero padding checked, read_ack frees the slot, 4th frame
//   9. ring, test pattern of 20 rows: 3 frames read back and checked against the counter pattern
//  10. ring, one 244-row sensor frame (309 blocks) read back completely
//  11. 244-row snapshot (Phase 1 path still intact), TriggerOut counts
//  12. free-run mode (WireIn 0x04[4]): the sensor model is told to send 8 frames of 12 rows per
//      request; the ring stores what fits, drops the rest (counted), sequence numbers show the
//      gaps, the delivered frames are intact; the idle re-request restarts the sensor
//  13. T_EXP1 pulse (TriggerIn 0x41[8])
//
// Parameter LVDS = 1 (xelab -generic_top LVDS=1, needs -L unisims_ver): the subsystem is built with
// the LVDS receiver (CMV300_Lvds.v: IBUFDS/IDELAYE2/ISERDESE2/BUFIO/BUFR) and the model serialises
// its frames on the six pairs. Steps 7, 9 and 11 (snapshot, test pattern) do not exist in that build;
// instead 6b checks the lane lock, a tap load, the eye-scan counter and the counters of a frame,
// and every ring frame is checked in the lane order of Fig. 19.
//============================================================================
// Lab 10 is the parallel port only; the LVDS branches below are inert (LVDS = 0).
module tb_lab10 #(parameter integer LVDS = 0);

    // ---- clocks / reset -----------------------------------------------------
    reg clk = 1'b0;        always #4.96 clk    = ~clk;     // 100.8 MHz okClk
    reg clk25 = 1'b0;      always #20   clk25  = ~clk25;   // 25 MHz "MMCM" output
    reg clk40m = 1'b0;     always #12.5 clk40m = ~clk40m;  // 40 MHz "MMCM" output (LVDS 480 Mbit/s test)
    reg ui_clk = 1'b0;     always #5    ui_clk = ~ui_clk;  // 100 MHz MIG user clock
    reg clk200 = 1'b0;     always #2.5  clk200 = ~clk200;  // 200 MHz board oscillator (IDELAYCTRL)
    reg rst = 1'b1;
    reg ui_rst = 1'b1;
    reg mmcm_locked = 1'b0;
    reg calib_done = 1'b0;

    // ---- endpoints ------------------------------------------------------------
    reg  [31:0] wi04 = 32'd0, wi05 = 32'd0, wi06 = 32'd0, ti41 = 32'd0;
    reg  [31:0] wi07 = 32'h0055_000A;          // LVDS: training pattern 0x055, tap 10, lane 0
    wire [31:0] to61, wo24, wo25, wo26, wo27, wo28, wo29, wo2a, wo2b, wo2c, wo2d, wo2e, wo2f;
    reg         po_read = 1'b0, bt_read = 1'b0;
    wire [31:0] po_data, bt_data;
    wire        bt_ready;

    // ---- sensor pins ------------------------------------------------------------
    wire        spi_clk, spi_en, spi_in, spi_out;
    wire        clk_in_en, clk_sel40, sys_res_n, enable_lvds, frame_req, t_exp1, t_exp2, ddr_sys_rst;
    wire        clk_in = (clk_sel40 ? clk40m : clk25) & clk_in_en;   // the top level's BUFGCTRL, modelled
    wire        clk_out, lval, dval;
    wire [9:0]  d;
    wire        lv_clk_p, lv_clk_n, lv_ctrl_p, lv_ctrl_n;
    wire [3:0]  lv_d_p, lv_d_n;

    // ---- AXI ------------------------------------------------------------------
    wire [3:0]   awid, arid, bid, rid;
    wire [29:0]  awaddr, araddr;
    wire [7:0]   awlen, arlen;
    wire [2:0]   awsize, arsize;
    wire [1:0]   awburst, arburst, bresp, rresp;
    wire         awvalid, awready, wvalid, wready, wlast, bvalid, bready, arvalid, arready, rvalid, rready, rlast;
    wire [127:0] wdata, rdata;
    wire [15:0]  wstrb;

    CMV300_Subsystem #(.AW(30), .IDLE_MS_REQ(3), .LVDS(LVDS)) dut (
        .clk(clk), .rst(rst), .clk_locked(mmcm_locked),
        .clk_cmv(clk_out), .d(d), .lval(lval), .dval(dval), .spi_out(spi_out),
        .clk200(clk200), .lv_clk_p(lv_clk_p), .lv_clk_n(lv_clk_n), .lv_d_p(lv_d_p), .lv_d_n(lv_d_n),
        .lv_ctrl_p(lv_ctrl_p), .lv_ctrl_n(lv_ctrl_n),
        .spi_clk(spi_clk), .spi_en(spi_en), .spi_in(spi_in),
        .clk_in_en(clk_in_en), .clk_sel40(clk_sel40), .sys_res_n(sys_res_n), .enable_lvds(enable_lvds), .frame_req(frame_req),
        .t_exp1(t_exp1), .t_exp2(t_exp2),
        .ui_clk(ui_clk), .ui_rst(ui_rst), .calib_done(calib_done), .ddr_sys_rst(ddr_sys_rst),
        .m_axi_awid(awid), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize), .m_axi_awburst(awburst),
        .m_axi_awvalid(awvalid), .m_axi_awready(awready), .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast),
        .m_axi_wvalid(wvalid), .m_axi_wready(wready), .m_axi_bid(bid), .m_axi_bresp(bresp), .m_axi_bvalid(bvalid),
        .m_axi_bready(bready), .m_axi_arid(arid), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
        .m_axi_arburst(arburst), .m_axi_arvalid(arvalid), .m_axi_arready(arready), .m_axi_rid(rid), .m_axi_rdata(rdata),
        .m_axi_rresp(rresp), .m_axi_rlast(rlast), .m_axi_rvalid(rvalid), .m_axi_rready(rready),
        .wi04(wi04), .wi05(wi05), .wi06(wi06), .wi07(wi07), .ti41(ti41), .to61(to61),
        .wo24(wo24), .wo25(wo25), .wo26(wo26), .wo27(wo27), .wo28(wo28), .wo29(wo29), .wo2a(wo2a),
        .wo2b(wo2b), .wo2c(wo2c), .wo2d(wo2d), .wo2e(wo2e), .wo2f(wo2f),
        .po_read(po_read), .po_data(po_data), .bt_read(bt_read), .bt_data(bt_data), .bt_ready(bt_ready)
    );

    axi_ram_model #(.AW(30)) u_ram (
        .clk(ui_clk), .rst(ui_rst),
        .s_axi_awid(awid), .s_axi_awaddr(awaddr), .s_axi_awlen(awlen), .s_axi_awsize(awsize), .s_axi_awburst(awburst),
        .s_axi_awvalid(awvalid), .s_axi_awready(awready), .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wlast(wlast),
        .s_axi_wvalid(wvalid), .s_axi_wready(wready), .s_axi_bid(bid), .s_axi_bresp(bresp), .s_axi_bvalid(bvalid),
        .s_axi_bready(bready), .s_axi_arid(arid), .s_axi_araddr(araddr), .s_axi_arlen(arlen), .s_axi_arsize(arsize),
        .s_axi_arburst(arburst), .s_axi_arvalid(arvalid), .s_axi_arready(arready), .s_axi_rid(rid), .s_axi_rdata(rdata),
        .s_axi_rresp(rresp), .s_axi_rlast(rlast), .s_axi_rvalid(rvalid), .s_axi_rready(rready)
    );

    cmv300_model u_sensor (
        .clk_in(clk_in), .sys_res_n(sys_res_n), .enable_lvds(enable_lvds), .frame_req(frame_req),
        .spi_clk(spi_clk), .spi_en(spi_en), .spi_in(spi_in), .spi_out(spi_out),
        .clk_out(clk_out), .d(d), .lval(lval), .dval(dval),
        .lv_clk_p(lv_clk_p), .lv_clk_n(lv_clk_n), .lv_d_p(lv_d_p), .lv_d_n(lv_d_n), .lv_ctrl_p(lv_ctrl_p), .lv_ctrl_n(lv_ctrl_n)
    );

    // ---- TriggerOut bookkeeping ----------------------------------------------------
    integer n_spi_done = 0, n_frame_done = 0, n_snap_done = 0, n_read_done = 0, n_frame_ready = 0, n_frame_req = 0, n_texp = 0;
    integer n_spi_ops = 0, n_ring_reads = 0;
    always @(posedge t_exp1) n_texp = n_texp + 1;
    always @(posedge clk) begin
        if (to61[0]) n_spi_done    = n_spi_done + 1;
        if (to61[1]) n_frame_done  = n_frame_done + 1;
        if (to61[2]) n_snap_done   = n_snap_done + 1;
        if (to61[3]) n_read_done   = n_read_done + 1;
        if (to61[4]) n_frame_ready = n_frame_ready + 1;
    end
    always @(posedge frame_req) begin
        n_frame_req = n_frame_req + 1;
        $display("%0t  MON: FRAME_REQ pin rises (#%0d); model busy=%0d pending=%0d; ctrl state %0d", $time, n_frame_req, u_sensor.busy, u_sensor.req_pending, dut.u_ctrl.st);
    end
    always @(posedge dut.u_ring.frame_req) $display("%0t  MON: ring frame_req (ui)", $time);
    always @(posedge dut.ring_req_pulse)  $display("%0t  MON: ring_req_pulse (okClk), req_cnt=%0d ctrl st=%0d", $time, dut.u_ctrl.req_cnt, dut.u_ctrl.st);

    reg [7:0] seq0;
    // pixel FIFO monitor while the write DMA drains (every 10 us)
    integer drain_ticks = 0;
    generate if (LVDS == 0) begin : g_mon_par
        always @(posedge ui_clk) if (dut.u_wdma.st == 3'd1) begin
            drain_ticks = drain_ticks + 1;
            if (drain_ticks % 1000 == 1)
                $display("%0t  MON: DRAIN fifo_rd=%0d pf_empty=%0d rd_rst_busy=%0d wr_rst_busy=%0d rd_count=%0d wr_count=%0d full=%0d drain_idle=%0d stream_en=%0d pix_valid=%0d",
                         $time, dut.pf_rd, dut.pf_empty, dut.pf_rd_rst_busy, dut.pf_wr_rst_busy, dut.u_pix_fifo.rd_data_count, dut.u_pix_fifo.wr_data_count,
                         dut.pf_full, dut.u_wdma.drain_idle, dut.u_cap.stream_en, dut.pix_valid);
            if (drain_ticks % 1000 == 1)
                $display("%0t  MON: generator tp_run=%0d tp_row=%0d tp_frame=%0d rows_eff=%0d tpm=%0d", $time, dut.u_cap.tp_run, dut.u_cap.tp_row, dut.u_cap.tp_frame, dut.u_cap.rows_eff, dut.u_cap.tpm_s[1]);
        end else drain_ticks = 0;
    end else begin : g_mon_lvds
        always @(wo2f[4:0]) $display("%0t  MON: LVDS lanes locked = %b (ctrl,4,3,2,1)", $time, wo2f[4:0]);
        always @(posedge dut.g_lvds.u_lvds.ctrl_tick) if (dut.g_lvds.u_lvds.lane[4].hold == 0 && dut.g_lvds.u_lvds.lane[4].tick && !dut.g_lvds.u_lvds.lane[4].locked && dut.g_lvds.u_lvds.lane[4].miss == 3)
            $display("%0t  MON: control lane word 0x%03h does not fit, slips=%0d half=%0d", $time, dut.g_lvds.u_lvds.lane[4].word, dut.g_lvds.u_lvds.lane[4].slips, dut.g_lvds.u_lvds.lane[4].ph_sel);
    end endgenerate

    // state monitor (ring writer / write DMA / read DMA) - the log shows where a frame got stuck
    always @(dut.u_ring.wst or dut.u_wdma.st or dut.u_ring.rst_)
        $display("%0t  MON: ring writer %0d, write DMA %0d, ring reader %0d, slots %b%b%b, pixfifo empty %0d",
                 $time, dut.u_ring.wst, dut.u_wdma.st, dut.u_ring.rst_, dut.u_ring.sst[2], dut.u_ring.sst[1], dut.u_ring.sst[0], dut.pf_empty);

    // ---- helpers -----------------------------------------------------------------
    integer failures = 0;

    // INTERFACE.md: pix_eof is gated by stream_en - a frame that was already running when acquisition stopped
    // must not emit a token. A pipelined packer may lag frame_end by a few clocks, so pix_eof is accepted only
    // if a frame_end seen with stream_en high occurred within the last 4 CLK_OUT cycles.
    reg [3:0] eof_ok = 4'd0;
    always @(posedge clk_out) begin
        eof_ok <= {eof_ok[2:0], dut.u_cap.frame_end & dut.u_cap.stream_en};
        if (dut.pix_eof && eof_ok == 4'd0) begin
            $display("TB: FAIL pix_eof without a frame_end seen while stream_en was high, at %0t ps", $time);
            failures = failures + 1;
        end
    end

    task check(input [511:0] name, input [31:0] got, input [31:0] want);
        begin
            if (got === want) $display("TB: PASS %0s = %0d", name, got);
            else begin $display("TB: FAIL %0s = %0d (0x%0h), expected %0d (0x%0h)", name, got, got, want, want); failures = failures + 1; end
        end
    endtask

    task trig(input integer bit_no);
        begin @(posedge clk); ti41 <= (32'd1 << bit_no); @(posedge clk); ti41 <= 32'd0; end
    endtask

    task spi_wait;
        integer n;
        begin
            n = 0; @(posedge clk); @(posedge clk);
            while (wo24[3] && n < 100000) begin @(posedge clk); n = n + 1; end
            if (wo24[3]) begin $display("TB: FAIL SPI still busy"); failures = failures + 1; end
        end
    endtask
    task spi_write(input [6:0] a, input [7:0] v); begin wi05 = {14'd0, 2'd0, 1'b1, a, v}; trig(0); spi_wait; n_spi_ops = n_spi_ops + 1; end endtask
    task spi_read(input [6:0] a, input [1:0] burst, output [31:0] v); begin wi05 = {14'd0, burst, 1'b0, a, 8'd0}; trig(0); spi_wait; v = wo28; n_spi_ops = n_spi_ops + 1; end endtask

    task wait_ready(input integer timeout_us);
        integer n; begin n = 0; while (!wo24[1] && n < timeout_us * 101) begin @(posedge clk); n = n + 1; end end
    endtask
    task wait_snap(input integer timeout_ms);
        integer n; begin n = 0; while (!wo24[9] && n < timeout_ms * 100800) begin @(posedge clk); n = n + 1; end
                     if (!wo24[9]) begin $display("TB: FAIL snapshot did not finish"); failures = failures + 1; end end
    endtask
    task wait_frames_ready(input integer want, input integer timeout_ms);
        integer n; begin n = 0; while (wo24[7:5] < want && n < timeout_ms * 100800) begin @(posedge clk); n = n + 1; end
                     if (wo24[7:5] < want) begin $display("TB: FAIL only %0d frame(s) ready after %0d ms", wo24[7:5], timeout_ms); failures = failures + 1; end end
    endtask

    // read `nwords` words from the PipeOut and compare with the sensor pattern of `nrows` rows
    task read_snapshot(input integer nrows, input integer nwords);
        integer w, p, px, exp_px, row, col, bad; reg [31:0] word;
        begin
            bad = 0; trig(3); @(posedge clk);
            for (w = 0; w < nwords; w = w + 1) begin
                po_read <= 1'b1; @(posedge clk); po_read <= 1'b0; #1 word = po_data; @(posedge clk);
                for (p = 0; p < 3; p = p + 1) begin
                    px = (word >> (10*p)) & 32'h3FF; row = (3*w + p) / 648; col = (3*w + p) % 648;
                    exp_px = (col + 3*row) & 32'h3FF;
                    if (px !== exp_px) begin if (bad < 5) $display("TB: FAIL snapshot pixel (%0d,%0d) = %0d, expected %0d", row, col, px, exp_px); bad = bad + 1; end
                end
            end
            if (bad == 0) $display("TB: PASS snapshot of %0d rows: %0d words, all %0d pixels match", nrows, nwords, 3*nwords);
            else begin $display("TB: FAIL snapshot of %0d rows: %0d bad pixels", nrows, bad); failures = failures + 1; end
        end
    endtask

    // BTPipe: read `blocks` x 256 words (a block only after ep_ready), into frame_bytes
    reg [7:0] frame_bytes [0:1024*1024-1];
    task bt_read_frame(input integer blocks);
        integer b, w, n; reg [31:0] word;
        begin
            for (b = 0; b < blocks; b = b + 1) begin
                n = 0; while (!bt_ready && n < 2000000) begin @(posedge clk); n = n + 1; end
                if (!bt_ready) begin $display("TB: FAIL BTPipe never ready for block %0d", b); failures = failures + 1; b = blocks; end
                else for (w = 0; w < 256; w = w + 1) begin
                    bt_read <= 1'b1; @(posedge clk); #1 word = bt_data;
                    frame_bytes[4*(b*256+w)]   = word[7:0];
                    frame_bytes[4*(b*256+w)+1] = word[15:8];
                    frame_bytes[4*(b*256+w)+2] = word[23:16];
                    frame_bytes[4*(b*256+w)+3] = word[31:24];
                end
                bt_read <= 1'b0; @(posedge clk);
            end
        end
    endtask

    // pixel i of the packed little-endian 12-bit stream (two pixels per three bytes)
    function [11:0] packed_px(input integer i);
        integer k;
        begin
            k = (i / 2) * 3;
            packed_px = (i % 2 == 0) ? {frame_bytes[k+1][3:0], frame_bytes[k]}
                                     : {frame_bytes[k+2], frame_bytes[k+1][7:4]};
        end
    endfunction

    // streaming mode: no read_req/ack - just read one fixed-size frame's blocks and check the pixels (LVDS layout)
    task stream_frame(input integer exp_rows);
        integer blocks, pixels, i, bad, row, col, exp_px, npad, b, k, ln;
        begin
            pixels = exp_rows * 648;
            blocks = (pixels * 3 / 2 + 1023) / 1024;
            bt_read_frame(blocks);
            bad = 0;
            for (i = 0; i < pixels; i = i + 1) begin
                b = i / 1296; k = (i % 1296) / 4; ln = i % 4;
                row = 2*b + (ln >= 2 ? 1 : 0); col = k + 324*(ln & 1);
                exp_px = (((col + 3*row) & 32'h3FF) << 2) | ((u_sensor.regs[68] == 8'd1) ? 0 : (row & 3));
                if (packed_px(i) !== exp_px[11:0]) begin if (bad < 5) $display("TB: FAIL stream pixel (%0d,%0d) = %0d, expected %0d", row, col, packed_px(i), exp_px); bad = bad + 1; end
            end
            npad = 0;
            for (i = pixels * 3 / 2; i < blocks * 1024; i = i + 1) if (frame_bytes[i] !== 8'd0) npad = npad + 1;
            if (bad == 0 && npad == 0) $display("TB: PASS stream frame of %0d rows: %0d pixels + padding correct", exp_rows, pixels);
            else begin $display("TB: FAIL stream frame: %0d bad pixels, %0d bad padding words", bad, npad); failures = failures + 1; end
        end
    endtask

    // one ring frame: read_req -> length -> BTPipe -> check -> read_ack
    task ring_frame(input integer exp_rows, input integer pattern /* 0 = sensor (col+3row), 1 = test pattern (idx+frame) */, input integer exp_frame);
        integer blocks, pixels, i, bad, row, col, exp_px, npad, b, k, ln;
        begin
            n_ring_reads = n_ring_reads + 1;
            trig(6); repeat (200) @(posedge clk);
            blocks = wo2b[15:0]; pixels = wo2d[23:0];
            check("ring read: pixels", pixels, exp_rows * 648);
            check("ring read: blocks", blocks, (exp_rows * 972 + 1023) / 1024);   // packed: 1.5 bytes per pixel
            check("ring read: read_err", wo24[15], 0);
            bt_read_frame(blocks);
            bad = 0;
            for (i = 0; i < pixels; i = i + 1) begin
                if (LVDS) begin                       // stored in lane order: word k of burst b = {row 2b px k, row 2b px 324+k, row 2b+1 ...}
                    b = i / 1296; k = (i % 1296) / 4; ln = i % 4;
                    row = 2*b + (ln >= 2 ? 1 : 0); col = k + 324*(ln & 1);
                    exp_px = (((col + 3*row) & 32'h3FF) << 2) | ((u_sensor.regs[68] == 8'd1) ? 0 : (row & 3));
                end else begin
                    row = i / 648; col = i % 648;
                    exp_px = (pattern == 0) ? ((col + 3*row) & 32'h3FF) : ((i + exp_frame) & 32'h3FF);
                end
                if (packed_px(i) !== exp_px[11:0]) begin if (bad < 5) $display("TB: FAIL ring pixel (%0d,%0d) = %0d, expected %0d", row, col, packed_px(i), exp_px); bad = bad + 1; end
            end
            npad = 0;
            for (i = pixels * 3 / 2; i < blocks * 1024; i = i + 1) if (frame_bytes[i] !== 8'd0) npad = npad + 1;
            if (bad == 0 && npad == 0) $display("TB: PASS ring frame of %0d rows: %0d packed pixels + %0d padding bytes all correct", exp_rows, pixels, blocks*1024 - pixels*3/2);
            else begin $display("TB: FAIL ring frame: %0d bad pixels, %0d non-zero padding bytes", bad, npad); failures = failures + 1; end
            trig(7); repeat (100) @(posedge clk);
        end
    endtask

    reg [31:0] v;
    integer meas, i;

    initial begin
        $display("TB: tb_cmv300 start");
        repeat (20) @(posedge clk); rst = 1'b0;
        repeat (20) @(posedge ui_clk); ui_rst = 1'b0;
        repeat (20) @(posedge clk); mmcm_locked = 1'b1; calib_done = 1'b1;

        // 1. power-up sequence
        check("ctrl state before enable", wo24[27:24], 0);
        wi04 = 32'h1;
        wait_ready(100);
        check("sensor_ready", wo24[1], 1);
        check("SYS_RES_N", sys_res_n, 1);
        check("CLK_IN enable", clk_in_en, 1);
        check("Enable_LVDS pin = build type", enable_lvds, LVDS);
        check("ctrl state READY", wo24[27:24], 3);
        check("ddr_calib_done seen", wo24[2], 1);

        // 2. defaults
        spi_read(1,  0, v); check("reg 1 default", v[7:0], 232);
        spi_read(2,  0, v); check("reg 2 default", v[7:0], 1);
        spi_read(42, 0, v); check("reg 42 default", v[7:0], 232);
        spi_read(61, 0, v); check("reg 61 default", v[7:0], 85);
        spi_read(83, 0, v); check("reg 83 default", v[7:0], 155);

        // 3. write / read-back
        spi_write(1, 100);  spi_read(1, 0, v); check("reg 1 after write 100", v[7:0], 100);
        spi_write(57, LVDS ? 0 : 3);   spi_write(68, 1); spi_write(69, 9); spi_write(83, 251);
        spi_read(57, 0, v); check("reg 57 (output mode)", v[7:0], LVDS ? 0 : 3);
        spi_read(68, 0, v); check("reg 68", v[7:0], 1);
        spi_read(69, 0, v); check("reg 69", v[7:0], 9);
        spi_read(83, 0, v); check("reg 83", v[7:0], 251);

        // 4. burst read
        spi_read(78, 1, v); check("temperature regs 78/79 (burst)", v[15:0], 16'h1464);

        // 5. reset
        trig(4); repeat (10) @(posedge clk);
        check("sensor_ready dropped on cam_reset", wo24[1], 0);
        wait_ready(200);
        check("sensor_ready again", wo24[1], 1);
        spi_read(1, 0, v);  check("reg 1 back to default after reset", v[7:0], 232);
        spi_read(69, 0, v); check("reg 69 back to default after reset", v[7:0], 0);

        // 6. CLK_OUT measurement
        spi_write(69, 9);
        repeat (2 * 100800) @(posedge clk);
        meas = wo27;
        if (meas >= 24984 && meas <= 25016) $display("TB: PASS clk_out_meas = %0d per ms", meas);
        else begin $display("TB: FAIL clk_out_meas = %0d, expected 25000 +/- 16", meas); failures = failures + 1; end

        if (LVDS) begin
            // 6b. LVDS: lanes lock on the training pattern / control word, tap load, eye-scan counter, frame counters
            i = 0; while (wo2f[4:0] != 5'h1F && i < 200000) begin @(posedge clk); i = i + 1; end   // <= 2 ms
            $display("TB: LVDS lock after %0d us", i / 101);
            check("LVDS: IDELAYCTRL ready", wo2f[5], 1);
            check("LVDS: clock running", wo2f[6], 1);
            check("LVDS: all 5 lanes locked", wo2f[4:0], 5'h1F);
            check("LVDS: build flag", wo2f[31], 1);
            wi07 = 32'h0055_0000 | (32'd1 << 5) | 32'd20; trig(9); repeat (50) @(posedge clk);
            check("LVDS: tap of lane 1 after load", wo2f[24:20], 20);
            repeat (30000) @(posedge clk);                       // > 255 evaluations at 25 M words/s
            check("LVDS: lane 1 matches 255/255 at tap 20", wo2f[15:8], 255);
            wi07 = 32'h0055_0000 | (32'd1 << 5) | 32'd10; trig(9); repeat (50) @(posedge clk);
            check("LVDS: still locked after the tap change", wo2f[4:0], 5'h1F);
            trig(10); repeat (200) @(posedge clk);                // realign: BUFR CLR + ISERDES reset + hunt again
            check("LVDS: realign in progress / lanes dropped", (wo2f[30] || wo2f[4:0] != 5'h1F), 1);
            i = 0; while (wo2f[4:0] != 5'h1F && i < 200000) begin @(posedge clk); i = i + 1; end
            check("LVDS: all lanes locked again after realign", wo2f[4:0], 5'h1F);
            wi07 = 32'h0055_0000 | 32'h200 | (32'd1 << 5) | 32'd10; repeat (30000) @(posedge clk);   // freeze: still counting
            check("LVDS: hits with the alignment frozen", wo2f[15:8], 255);
            check("LVDS: freeze echo", wo2f[29], 1);
            wi07 = 32'h0055_0000 | (32'd1 << 5) | 32'd10;
            spi_write(1, 12); spi_write(2, 0);
            trig(5); trig(1);
            repeat (30000) @(posedge clk);
            check("LVDS: pixels in the last row", wo26[15:0], 648);
            check("LVDS: rows in the last frame", wo26[31:16], 12);
            check("LVDS: frame_count", wo25[15:0], 1);
            check("LVDS: FVAL low again", wo2f[27], 0);
            check("LVDS: lanes still locked after a frame", wo2f[4:0], 5'h1F);
        end else begin
        // 7. short frame: 12 rows, snapshot
        spi_write(1, 12); spi_write(2, 0);
        trig(5); trig(2); repeat (10) @(posedge clk);
        check("snap_armed", wo24[8], 1);
        trig(1);
        wait_snap(2);
        check("pixels in the last row", wo26[15:0], 648);
        check("rows in the last frame", wo26[31:16], 12);
        check("frame_count", wo25[15:0], 1);
        check("snap_words (12 rows)", wo29[15:0], 12 * 216);
        check("OR mask", wo2a[9:0], 10'h3FF);
        check("AND mask", wo2a[25:16], 0);
        read_snapshot(12, 12 * 216);
        end

        // 8. ring with sensor frames of 12 rows
        n_frame_req = 0;
        wi04 = 32'h3;                                   // cam_enable + acq_enable
        wait_frames_ready(3, 20);
        check("frames_ready", wo24[7:5], 3);
        check("FRAME_REQ pulses for 3 slots", n_frame_req, 3);
        check("frames stored", wo25[31:16], 3);
        check("frames dropped", wo2c[15:0], 0);
        check("acq_active while all slots full", wo24[4], 0);
        ring_frame(12, 0, 0);
        check("frames_ready after one read", wo24[7:5], 2);
        wait_frames_ready(3, 20);                       // the freed slot is refilled
        check("frames stored after refill", wo25[31:16], 4);
        ring_frame(12, 0, 0);
        ring_frame(12, 0, 0);
        wi04 = 32'h1;                                   // stop acquisition
        repeat (2000) @(posedge clk);

        // 9. ring with the test pattern, 20 rows (parallel build only: the generator lives in CMV300_Capture)
        if (!LVDS) begin
        wi06 = 32'd20;
        wi04 = 32'h1 | 32'h8;                           // test pattern on
        repeat (100) @(posedge clk);
        wi04 = 32'h1 | 32'h8 | 32'h2;                   // acq on
        wait_frames_ready(3, 20);
        for (i = 0; i < 3; i = i + 1) ring_frame(20, 1, i);
        wi04 = 32'h1;
        repeat (2000) @(posedge clk);
        end else repeat (30000) @(posedge clk);         // LVDS: let the sensor finish the 12-row frames still in flight

        // 10. one full-size sensor frame through the ring (244 rows)
        spi_write(1, 244);
        wi04 = 32'h3;
        repeat (500) @(posedge clk);                    // the start-up flush discards READY frames of the previous session first
        wait_frames_ready(1, 15);
        wi04 = 32'h1;                                   // stop before the other slots fill (saves simulation time)
        ring_frame(244, 0, 0);

        // 11. Phase-1 snapshot still works on a 244-row frame; TriggerOuts
        if (!LVDS) begin
        trig(2); trig(1);
        wait_snap(12);
        repeat (2000) @(posedge clk);
        check("rows in the last frame (244)", wo26[31:16], 244);
        check("snap_words (244 rows)", wo29[15:0], 52704);
        read_snapshot(244, 52704);
        end
        check("spi_done pulses = SPI operations so far", n_spi_done, n_spi_ops);
        check("read_done pulses = ring reads so far", n_read_done, n_ring_reads);
        $display("TB: frame_ready pulses so far = %0d, frames stored = %0d", n_frame_ready, wo25[31:16]);
        check("frame_ready pulses >= 8 (parallel) / >= 5 (LVDS, no test-pattern frames)", (n_frame_ready >= (LVDS ? 5 : 8)), 1);
        check("pixel FIFO overflow", wo24[14], 0);

        // 12. free-run: 8 frames of 12 rows per request, host reads slowly -> drops, then re-request after idle
        spi_write(1, 12); spi_write(55, 8);
        trig(5);
        n_frame_req = 0;
        wi04 = 32'h1 | 32'h2 | 32'h10;                  // cam_enable + acq_enable + free_run
        repeat (600000) @(posedge clk);                 // ~6 ms: all 8 frames have gone by (3 stored, 5 dropped)
        check("free-run: FRAME_REQ issued", (n_frame_req >= 1), 1);
        check("free-run: frames_ready", wo24[7:5], 3);
        check("free-run: frames dropped >= 5", (wo2c[15:0] >= 5), 1);
        if (LVDS) check("free-run: frame period (12 rows, LVDS) = 300 + 6*325 + 1 + 200 words", (wo2e[23:0] > 2300 && wo2e[23:0] < 2600), 1);
        else      check("free-run: frame period (12 rows) = 300 + 12*650 + 200 clocks", (wo2e[23:0] > 8000 && wo2e[23:0] < 8600), 1);
        ring_frame(12, 0, 0);
        seq0 = wo2b[31:24];
        ring_frame(12, 0, 0);
        check("free-run: sequence number of the second frame", wo2b[31:24], seq0 + 1);
        ring_frame(12, 0, 0);
        check("free-run: sequence number of the third frame", wo2b[31:24], seq0 + 2);
        // slots are free again -> after IDLE_MS_REQ (3 ms in this tb) the ring re-requests, another 8 frames come
        repeat (500000) @(posedge clk);                 // 5 ms
        check("free-run: sensor re-requested after idle", (n_frame_req >= 2), 1);
        wait_frames_ready(3, 10);
        ring_frame(12, 0, 0);
        check("free-run: sequence number advanced by >= 8 (3 stored + >= 5 dropped)", ((wo2b[31:24] - seq0) & 8'hFF) >= 8, 1);
        wi04 = 32'h1;
        repeat (20000) @(posedge clk);
        check("free-run: no pixel FIFO overflow", wo24[14], 0);

        // 13. T_EXP1 pulse
        trig(8); repeat (100) @(posedge clk);
        check("T_EXP1 pulses", n_texp, 1);

        // 14. LVDS only: CLK_IN = 40 MHz -> 480 Mbit/s lanes (wi04[2], switched with the sensor in reset)
        if (LVDS) begin
            wi04 = 32'h0; repeat (5000) @(posedge clk);          // sensor into reset, LVDS clock stops
            wi04 = 32'h1 | 32'h4;                                // up again at 40 MHz
            wait_ready(200);
            spi_write(83, 155);                                  // PLL band for 32-50 MHz (§5.10)
            spi_write(57, 0); spi_write(68, 0); spi_write(1, 12); spi_write(2, 0);
            trig(10);                                            // realign at the new bit rate
            i = 0; while (wo2f[4:0] != 5'h1F && i < 300000) begin @(posedge clk); i = i + 1; end
            check("40 MHz: all 5 lanes locked at 480 Mbit/s", wo2f[4:0], 5'h1F);
            repeat (2 * 100800) @(posedge clk);
            meas = wo27;
            if (meas >= 39984 && meas <= 40016) $display("TB: PASS 40 MHz: %0d words per ms", meas);
            else begin $display("TB: FAIL 40 MHz: %0d words per ms, expected 40000 +/- 16", meas); failures = failures + 1; end
            wi04 = 32'h1 | 32'h4 | 32'h2;                        // one paced 12-row frame through the ring
            wait_frames_ready(1, 10);
            wi04 = 32'h1 | 32'h4;
            ring_frame(12, 0, 0);

            // 15. streaming: the ring auto-serves frames, the host only reads the pipe (still at 40 MHz)
            spi_write(55, 8);                                    // the model pipelines 8 frames per request
            trig(5);
            wi04 = 32'h1 | 32'h4 | 32'h2 | 32'h10 | 32'h20;      // acq + free_run + stream
            stream_frame(12);
            stream_frame(12);
            stream_frame(12);
            check("stream: frames stored >= 3", (wo25[31:16] >= 3), 1);
            wi04 = 32'h1 | 32'h4 | 32'h2 | 32'h10;               // leave stream mode -> the pipe flushes itself
            repeat (20000) @(posedge clk);
            check("stream: pipe flushed after leaving the mode", bt_ready, 0);
            wi04 = 32'h1 | 32'h4;
            repeat (20000) @(posedge clk);
        end

        $display("TB: finished (%0s build), %0d failure(s)  ->  RESULT: %0s", LVDS ? "LVDS" : "parallel", failures, failures == 0 ? "ALL PASS" : "FAIL");
        $finish;
    end

    initial begin #120_000_000; $display("TB: FAIL timeout"); $display("RESULT: FAIL"); $finish; end
endmodule
