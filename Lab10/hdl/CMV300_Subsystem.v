`timescale 1ns / 1ps
//============================================================================
// CMV300_Subsystem.v - everything CMV300 in one box, instantiated once by
//                      SensorBoard_Top.v next to the I2C machine (u_i2c).
//
//   Phase 0 + 1 (docs/CMV300_CAPTURE_PLAN.md): power-up sequencer + SPI master
//   + parallel capture with counters and a one-frame BRAM snapshot (PipeOut 0xA0).
//   Phase 2: pixel FIFO -> write DMA -> DDR3 (AXI4 master exposed to the top
//   level, where the MIG lives) -> three-slot frame ring -> read DMA -> pipe
//   FIFO -> BTPipeOut 0xA1, paced in "mode A" (one FRAME_REQ per free slot).
//
// PC <-> FPGA CONTRACT of the camera (top-level header has the whole map)
//   WireIn  0x04 cam_ctrl   [0] cam_enable (level: clock on, reset released, "ready")
//                           [1] acq_enable (level: frames into the DDR3 ring, one FRAME_REQ per free slot)
//                           [2] CLK_IN = 40 MHz instead of 25 (LVDS build only: 480 Mbit/s lanes, 480 fps;
//                               switch only while cam_enable = 0, and write reg 83 = 155 for the 32-50 MHz band)
//                           [3] test_pattern (generator instead of the sensor's LVAL/DVAL/D; sensor clock still needed)
//                           [4] free_run ("mode B": the sensor pipelines Number_frames frames by itself, the DMA drops
//                               frames that find no free slot, one FRAME_REQ is re-issued after 100 ms of idle)
//                           [5] stream: the ring auto-serves READY frames into the BTPipe (no read_req/read_ack -
//                               the host just reads blocks; frames are fixed-size and arrive oldest first; clearing
//                               the bit flushes whatever is left in the pipe)
//                           [7] ddr_reset (level, active high: MIG sys_rst)
//                           [15:8] minimum frame period in ms (0 = as fast as the sensor goes)
//   WireIn  0x05 spi_cmd    [7:0] data to write, [14:8] register address, [15] 1 = write / 0 = read,
//                           [17:16] burst = frames - 1 (reads of consecutive registers, e.g. 78/79)
//   WireIn  0x06            [15:0] test-pattern rows (0 = 488)
//   WireOut 0x24 cam_status [0] clk_locked, [1] sensor_ready, [2] ddr_calib_done, [3] spi_busy,
//                           [4] acq_active (writer waiting for a frame), [7:5] frames_ready (0..3),
//                           [8] snap_armed, [9] snap_done, [10] FRAME_REQ pin (live), [11] cam_enable echo,
//                           [12] wr_busy, [13] rd_busy, [14] pixel FIFO overflow (sticky), [15] read_err,
//                           [23:16] SPI byte 0, [27:24] ctrl state (0 OFF, 1 CLK_ON, 2 RES_WAIT, 3 READY, 4 RES_LOW),
//                           [28] ui_rst, [29] acq_enable echo, [30] test_pattern echo
//   WireOut 0x25 [15:0] frames seen by the capture, [31:16] frames stored in the ring
//   WireOut 0x26 [15:0] pixels in the last row, [31:16] rows in the last frame
//   WireOut 0x27 CLK_OUT cycles per 1 ms (25,000 +/- 16 at 25 MHz)
//   WireOut 0x28 SPI read data, byte i of a burst in [8i+7:8i]
//   WireOut 0x29 [15:0] words written by the last snapshot (<= 52,704)
//   WireOut 0x2A [9:0] OR of every pixel of the last frame, [25:16] AND of every pixel
//   WireOut 0x2B [15:0] 1024-byte blocks of the frame being read, [17:16] its slot, [23:20] ring writer/slot debug,
//                [31:24] its sequence number (every frame that reached the DMA counts; a gap = a dropped frame)
//   WireOut 0x2C [15:0] frames dropped by the ring, [23:16] ring debug {slot2,slot1,slot0,wstate}
//   WireOut 0x2D [23:0] pixels in the frame being read
//   WireOut 0x2E [23:0] CLK_OUT cycles between the last two frame starts (frame period; / 25 = us at 25 MHz)
//   TriggerIn  0x41 [0] spi_go, [1] frame_req, [2] snap_arm, [3] pipe_rewind, [4] cam_reset, [5] clear counters,
//                   [6] read_req (reserve the oldest READY frame and start streaming it), [7] read_ack (free it),
//                   [8] t_exp1 pulse (external exposure, reg 41 = 1: T_EXP1 starts, FRAME_REQ ends the exposure),
//                   [9] LVDS: load the IDELAY tap of WireIn 0x07 into its lane, [10] LVDS: re-align every lane (BUFR CLR +
//                   ISERDES reset + new alignment search; do it after programming the sensor's PLL band)
//   WireIn  0x07 (LVDS build) [4:0] IDELAY tap (78 ps each), [7:5] lane (0..3 = Out1..4, 4 = control),
//                             [8] reverse the bit order of the halves (bring-up aid), [9] freeze the alignment (eye scan),
//                             [27:16] training pattern (regs 61/62)
//   WireOut 0x2F (LVDS build) [4:0] lanes locked (bit 4 = control), [5] IDELAYCTRL ready, [6] LVDS clock running,
//                             [7] reverse echo, [15:8] matches of the selected lane in its last 255 evaluations (eye scan, 255 = all),
//                             [18:16] its bitslips, [19] its half-select, [24:20] its tap, [25] DVAL, [26] LVAL, [27] FVAL
//                             (live), [28] in_frame, [29] freeze echo, [30] realign in progress, [31] 1 = LVDS build (0 in the parallel build)
//   TriggerOut 0x61 [0] spi_done, [1] frame_done, [2] snap_done, [3] read_done, [4] frame_ready
//   PipeOut   0xA0 snapshot words: three 10-bit pixels per 32-bit word ([9:0] first), row-major (parallel build only)
//   BTPipeOut 0xA1 ring frame: PACKED little-endian 12-bit pixels, two per three bytes (pixel n even:
//                  byte0 | (byte1[3:0] << 8); odd: byte1[7:4] | (byte2 << 4)), padded with zeros to blocks x
//                  1024 bytes; frame bytes = pixels x 3 / 2, read exactly blocks x 1024 with block size 1024.
//                  Parallel build: 10-bit values in the 12-bit fields, row-major. LVDS build: the lanes' 12-bit
//                  words in lane order - word k of burst b = {row 2b px k, row 2b px 324+k, row 2b+1 px k,
//                  row 2b+1 px 324+k} (Fig. 19); python/cmv300.py unpacks and de-interleaves.
//============================================================================
module CMV300_Subsystem #(
    parameter integer AW = 30,
    parameter integer IDLE_MS_REQ = 100,         // free-run: re-request the sensor after this many idle ms
    parameter integer LVDS = 0,                  // 1 = LVDS receiver on the Out* pairs instead of the parallel port
    parameter         DIFF_TERM = "TRUE"         // LVDS: on-chip 100 ohm (bank 34 must be at 2.5 V)
)(
    input  wire          clk,               // okClk
    input  wire          rst,               // global reset (SPI + snapshot pointers; not the sensor)
    input  wire          clk_locked,        // MMCM of CLK_IN locked (synchronised here)
    // sensor pins
    input  wire          clk_cmv,           // CVM300_CLK_OUT
    input  wire [9:0]    d,
    input  wire          lval,
    input  wire          dval,
    input  wire          clk200,            // 200 MHz: IDELAYCTRL reference (LVDS build)
    input  wire          lv_clk_p, lv_clk_n,     // CVM300_Out_clk (LVDS build; tie to 0 otherwise)
    input  wire [3:0]    lv_d_p, lv_d_n,         // CVM300_Out1..4
    input  wire          lv_ctrl_p, lv_ctrl_n,   // CVM300_Out_ctrl
    input  wire          spi_out,
    output wire          spi_clk,
    output wire          spi_en,
    output wire          spi_in,
    output wire          clk_in_en,         // to the ODDR that drives CVM300_CLK_IN
    output wire          clk_sel40,         // 1 = the 40 MHz MMCM output feeds that ODDR (LVDS build, wi04[2])
    output wire          sys_res_n,
    output wire          enable_lvds,
    output wire          frame_req,
    output wire          t_exp1,
    output wire          t_exp2,
    // DDR3 side (MIG in the top level)
    input  wire          ui_clk,
    input  wire          ui_rst,            // ui_clk_sync_rst
    input  wire          calib_done,
    output wire          ddr_sys_rst,       // -> MIG sys_rst (active high)
    output wire [3:0]    m_axi_awid,
    output wire [AW-1:0] m_axi_awaddr,
    output wire [7:0]    m_axi_awlen,
    output wire [2:0]    m_axi_awsize,
    output wire [1:0]    m_axi_awburst,
    output wire          m_axi_awvalid,
    input  wire          m_axi_awready,
    output wire [127:0]  m_axi_wdata,
    output wire [15:0]   m_axi_wstrb,
    output wire          m_axi_wlast,
    output wire          m_axi_wvalid,
    input  wire          m_axi_wready,
    input  wire [3:0]    m_axi_bid,
    input  wire [1:0]    m_axi_bresp,
    input  wire          m_axi_bvalid,
    output wire          m_axi_bready,
    output wire [3:0]    m_axi_arid,
    output wire [AW-1:0] m_axi_araddr,
    output wire [7:0]    m_axi_arlen,
    output wire [2:0]    m_axi_arsize,
    output wire [1:0]    m_axi_arburst,
    output wire          m_axi_arvalid,
    input  wire          m_axi_arready,
    input  wire [3:0]    m_axi_rid,
    input  wire [127:0]  m_axi_rdata,
    input  wire [1:0]    m_axi_rresp,
    input  wire          m_axi_rlast,
    input  wire          m_axi_rvalid,
    output wire          m_axi_rready,
    // host endpoints
    input  wire [31:0]   wi04,
    input  wire [31:0]   wi05,
    input  wire [31:0]   wi06,
    input  wire [31:0]   wi07,
    input  wire [31:0]   ti41,
    output wire [31:0]   to61,
    output wire [31:0]   wo24,
    output wire [31:0]   wo25,
    output wire [31:0]   wo26,
    output wire [31:0]   wo27,
    output wire [31:0]   wo28,
    output wire [31:0]   wo29,
    output wire [31:0]   wo2a,
    output wire [31:0]   wo2b,
    output wire [31:0]   wo2c,
    output wire [31:0]   wo2d,
    output wire [31:0]   wo2e,
    output wire [31:0]   wo2f,
    input  wire          po_read,
    output wire [31:0]   po_data,
    input  wire          bt_read,
    output wire [31:0]   bt_data,
    output reg           bt_ready
);
    //------------------------------------------------------------------------
    // Sequencer, SPI (okClk)
    //------------------------------------------------------------------------
    reg [1:0] lock_s = 2'b00;
    always @(posedge clk) lock_s <= {lock_s[0], clk_locked};

    wire        sensor_ready;
    wire [3:0]  ctrl_state;
    wire        ring_req_pulse;                 // frame request from the ring (already in okClk)
    CMV300_Ctrl u_ctrl (
        .clk          (clk),
        .cam_enable   (wi04[0]),
        .cam_reset    (ti41[4]),
        .clk_locked   (lock_s[1]),
        .frame_req_in (ti41[1] | ring_req_pulse),
        .t_exp_in     (ti41[8]),
        .clk_en       (clk_in_en),
        .sys_res_n    (sys_res_n),
        .frame_req    (frame_req),
        .t_exp1       (t_exp1),
        .sensor_ready (sensor_ready),
        .state        (ctrl_state)
    );
    assign enable_lvds = (LVDS != 0);          // pin B2: 1 = LVDS outputs, 0 = parallel CMOS (§4 p.18)
    assign clk_sel40   = (LVDS != 0) & wi04[2];   // the parallel port is a 25 MHz port on every variant
    assign t_exp2      = 1'b0;
    assign ddr_sys_rst = wi04[7];

    wire        spi_busy, spi_done;
    wire [31:0] spi_rd;
    CMV300_Spi #(.HALF(5)) u_spi (
        .clk(clk), .rst(rst), .go(ti41[0]), .wr(wi05[15]), .addr(wi05[14:8]), .wr_data(wi05[7:0]),
        .burst(wi05[17:16]), .busy(spi_busy), .done(spi_done), .rd_data(spi_rd),
        .SPI_CLK(spi_clk), .SPI_EN(spi_en), .SPI_IN(spi_in), .SPI_OUT(spi_out)
    );

    //------------------------------------------------------------------------
    // Capture (CLK_OUT domain) + snapshot (Phase 1) + pixel stream (Phase 2)
    //------------------------------------------------------------------------
    wire [15:0] frame_count, last_row_pixels, last_frame_rows, snap_words;
    wire [23:0] frame_period;
    wire [9:0]  or_mask, and_mask;
    wire        snap_armed, snap_done, frame_done_pulse, snap_done_pulse;
    wire [31:0] clk_out_meas;
    wire        pix_valid, pix_eof, pix_fifo_rst, pix_clk;
    wire [47:0] pix_data;                        // four 12-bit pixels per FIFO entry (both front ends)
    wire [31:0] lvds_status;
    // Lab 10 is the parallel port only. (The LVDS receiver is the Final Project.)
    CMV300_CaptureCore #(.SNAP_WORDS(52704), .GAP(64), .MEAS_WINDOW(100800)) u_cap (
        .clk_cmv(clk_cmv), .d(d), .lval(lval), .dval(dval),
        .pix_valid(pix_valid), .pix_data(pix_data), .pix_eof(pix_eof), .fifo_rst(pix_fifo_rst), .acq(wi04[1]),
        .tp_mode(wi04[3]), .tp_loop(wi04[4]), .tp_start(ti41[1] | ring_req_pulse), .tp_rows(wi06[15:0]),
        .clk(clk), .rst(rst), .snap_arm(ti41[2]), .rewind(ti41[3]), .clear(ti41[5]),
        .po_read(po_read), .po_data(po_data),
        .frame_count(frame_count), .last_row_pixels(last_row_pixels), .last_frame_rows(last_frame_rows),
        .or_mask(or_mask), .and_mask(and_mask), .snap_words(snap_words), .frame_period(frame_period),
        .snap_armed(snap_armed), .snap_done(snap_done),
        .frame_done_pulse(frame_done_pulse), .snap_done_pulse(snap_done_pulse), .clk_out_meas(clk_out_meas)
    );
    assign pix_clk     = clk_cmv;
    assign lvds_status = 32'd0;

    //------------------------------------------------------------------------
    // Pixel FIFO: pixel clock -> ui_clk, 49 bits ({eof, four 12-bit pixels}), 4096 deep, fwft
    //------------------------------------------------------------------------
    wire [48:0] pf_dout;
    wire        pf_empty, pf_full, pf_overflow, pf_rd_rst_busy, pf_wr_rst_busy;
    wire        pf_rd;
    // pixels enter the FIFO only while acquisition is enabled (the capture opens its gate between frames)
    wire        pf_wr = (pix_valid | pix_eof) & ~pix_fifo_rst & ~pf_wr_rst_busy;
    xpm_fifo_async #(
        .CDC_SYNC_STAGES(2), .DOUT_RESET_VALUE("0"), .ECC_MODE("no_ecc"), .FIFO_MEMORY_TYPE("block"),
        .FIFO_READ_LATENCY(0), .FIFO_WRITE_DEPTH(4096), .FULL_RESET_VALUE(0), .PROG_EMPTY_THRESH(10),
        .PROG_FULL_THRESH(4000), .RD_DATA_COUNT_WIDTH(13), .READ_DATA_WIDTH(49), .READ_MODE("fwft"),
        .RELATED_CLOCKS(0), .SIM_ASSERT_CHK(0), .USE_ADV_FEATURES("0707"), .WAKEUP_TIME(0),
        .WRITE_DATA_WIDTH(49), .WR_DATA_COUNT_WIDTH(13)
    ) u_pix_fifo (
        .sleep(1'b0), .rst(pix_fifo_rst), .wr_clk(pix_clk), .wr_en(pf_wr), .din({pix_eof, pix_data}),
        .full(pf_full), .prog_full(), .wr_data_count(), .overflow(pf_overflow), .wr_rst_busy(pf_wr_rst_busy),
        .almost_full(), .wr_ack(), .rd_clk(ui_clk), .rd_en(pf_rd), .dout(pf_dout), .empty(pf_empty),
        .prog_empty(), .rd_data_count(), .underflow(), .rd_rst_busy(pf_rd_rst_busy), .almost_empty(),
        .data_valid(), .injectsbiterr(1'b0), .injectdbiterr(1'b0), .sbiterr(), .dbiterr()
    );
    // sticky overflow flag (okClk)
    reg [2:0] ovf_s = 3'd0; reg pf_ovf_sticky = 1'b0;
    always @(posedge clk) begin ovf_s <= {ovf_s[1:0], pf_overflow}; if (ovf_s[2]) pf_ovf_sticky <= 1'b1; if (ti41[5]) pf_ovf_sticky <= 1'b0; end

    //------------------------------------------------------------------------
    // okClk <-> ui_clk control crossings
    //------------------------------------------------------------------------
    // levels okClk -> ui_clk
    reg [1:0] acq_s = 2'd0, fr_s = 2'd0, strm_s = 2'd0;
    always @(posedge ui_clk) begin acq_s <= {acq_s[0], wi04[1]}; fr_s <= {fr_s[0], wi04[4]}; strm_s <= {strm_s[0], wi04[5]}; end
    // pulses okClk -> ui_clk (toggle)
    reg req_t = 1'b0, ack_t = 1'b0;
    reg [2:0] req_s = 3'd0, ack_s = 3'd0;
    // read_req: first drain whatever is left in the pipe FIFO, then forward
    wire pipe_empty; wire [14:0] pipe_rd_count; wire pipe_rd_rst_busy;
    reg  drain = 1'b0, drain_fwd = 1'b0, stream_d = 1'b0;
    reg  [1:0] rd_busy_ok = 2'd0;
    wire rd_busy_ui;
    always @(posedge clk) begin
        rd_busy_ok <= {rd_busy_ok[0], rd_busy_ui};
        stream_d <= wi04[5];
        if (ti41[6]) begin drain <= 1'b1; drain_fwd <= 1'b1; end
        else if (stream_d && !wi04[5]) begin drain <= 1'b1; drain_fwd <= 1'b0; end   // leaving stream mode: flush the pipe
        else if (drain && pipe_empty && !rd_busy_ok[1]) begin drain <= 1'b0; if (drain_fwd) req_t <= ~req_t; end
        if (ti41[7]) ack_t <= ~ack_t;
    end
    always @(posedge ui_clk) begin req_s <= {req_s[1:0], req_t}; ack_s <= {ack_s[1:0], ack_t}; end
    wire read_req_ui = req_s[2] ^ req_s[1];
    wire read_ack_ui = ack_s[2] ^ ack_s[1];
    // pulses ui_clk -> okClk (toggle)
    reg frq_t = 1'b0, rdd_t = 1'b0, frr_t = 1'b0;
    reg [2:0] frq_s = 3'd0, rdd_s = 3'd0, frr_s = 3'd0;
    wire ring_frame_req, ring_read_done, ring_frame_ready;
    always @(posedge ui_clk) begin
        if (ring_frame_req)   frq_t <= ~frq_t;
        if (ring_read_done)   rdd_t <= ~rdd_t;
        if (ring_frame_ready) frr_t <= ~frr_t;
    end
    always @(posedge clk) begin frq_s <= {frq_s[1:0], frq_t}; rdd_s <= {rdd_s[1:0], rdd_t}; frr_s <= {frr_s[1:0], frr_t}; end
    assign ring_req_pulse = frq_s[2] ^ frq_s[1];
    wire read_done_ok   = rdd_s[2] ^ rdd_s[1];
    wire frame_ready_ok = frr_s[2] ^ frr_s[1];
    // status levels/values ui_clk -> okClk (values change only around the pulses above)
    wire [1:0]  frames_ready_ui, read_slot_ui;
    wire [7:0]  read_seq_ui;
    wire [15:0] read_blocks_ui, frames_stored_ui, frames_dropped_ui;
    wire [23:0] read_pixels_ui;
    wire        acq_active_ui, read_err_ui, wr_busy_ui, calib_s_ui;
    wire [7:0]  ring_dbg_ui;
    reg [1:0] fr_gray_s0 = 2'd0, fr_gray_s1 = 2'd0;
    wire [1:0] fr_gray = {frames_ready_ui[1], frames_ready_ui[1] ^ frames_ready_ui[0]};
    reg  [5:0] lvl_s0 = 6'd0, lvl_s1 = 6'd0;
    reg  [15:0] read_blocks_ok = 16'd0, frames_stored_ok = 16'd0, frames_dropped_ok = 16'd0;
    reg  [23:0] read_pixels_ok = 24'd0;
    reg  [1:0]  read_slot_ok = 2'd0;
    reg  [7:0]  read_seq_ok = 8'd0;
    reg  [7:0]  ring_dbg_ok = 8'd0;
    always @(posedge clk) begin
        fr_gray_s0 <= fr_gray; fr_gray_s1 <= fr_gray_s0;
        lvl_s0 <= {ui_rst, calib_done, acq_active_ui, read_err_ui, wr_busy_ui, rd_busy_ui}; lvl_s1 <= lvl_s0;
        // quasi-static values: sample continuously (they change only at ring events, far apart)
        read_blocks_ok <= read_blocks_ui; read_pixels_ok <= read_pixels_ui; read_slot_ok <= read_slot_ui; read_seq_ok <= read_seq_ui;
        frames_stored_ok <= frames_stored_ui; frames_dropped_ok <= frames_dropped_ui; ring_dbg_ok <= ring_dbg_ui;
    end
    wire [1:0] frames_ready_ok = {fr_gray_s1[1], fr_gray_s1[1] ^ fr_gray_s1[0]};

    //------------------------------------------------------------------------
    // Ring + DMAs (ui_clk)
    //------------------------------------------------------------------------
    wire          wr_slot_valid, wr_abort, wr_flush, wr_frame_done, wr_frame_dropped;
    wire [AW-1:0] wr_slot_base, rd_base;
    wire [15:0]   wr_frame_blocks, rd_blocks;
    wire [23:0]   wr_frame_pixels;
    wire          rd_start, rd_done;
    reg  [1:0]    calib_ui = 2'd0;
    always @(posedge ui_clk) calib_ui <= {calib_ui[0], calib_done};

    CMV300_FrameRing #(.AW(AW), .SLOT_SHIFT(20), .TIMEOUT(25000000), .MS_CYCLES(100000), .IDLE_MS_REQ(IDLE_MS_REQ)) u_ring (
        .clk(ui_clk), .rst(ui_rst), .enable(acq_s[1]), .free_run(fr_s[1]), .stream(strm_s[1]), .calib_done(calib_ui[1]), .min_period_ms(wi04[15:8]),
        .wr_slot_valid(wr_slot_valid), .wr_slot_base(wr_slot_base), .wr_abort(wr_abort), .wr_flush(wr_flush),
        .wr_busy(wr_busy_ui), .wr_frame_done(wr_frame_done), .wr_frame_dropped(wr_frame_dropped), .wr_frame_blocks(wr_frame_blocks), .wr_frame_pixels(wr_frame_pixels),
        .frame_req(ring_frame_req),
        .rd_start(rd_start), .rd_base(rd_base), .rd_blocks(rd_blocks), .rd_busy(rd_busy_ui), .rd_done_in(rd_done),
        .read_req(read_req_ui), .read_ack(read_ack_ui), .read_done(ring_read_done), .frame_ready(ring_frame_ready),
        .frames_ready(frames_ready_ui), .read_slot(read_slot_ui), .read_seq(read_seq_ui), .read_blocks(read_blocks_ui), .read_pixels(read_pixels_ui),
        .frames_stored(frames_stored_ui), .frames_dropped(frames_dropped_ui), .read_err(read_err_ui),
        .acq_active(acq_active_ui), .dbg(ring_dbg_ui)
    );

    CMV300_WriteDma #(.AW(AW)) u_wdma (
        .clk(ui_clk), .rst(ui_rst),
        .fifo_dout(pf_dout), .fifo_empty(pf_empty | pf_rd_rst_busy), .fifo_rd(pf_rd),
        .slot_valid(wr_slot_valid), .slot_base(wr_slot_base), .abort(wr_abort), .flush(wr_flush),
        .busy(wr_busy_ui), .frame_done(wr_frame_done), .frame_dropped(wr_frame_dropped), .frame_blocks(wr_frame_blocks), .frame_pixels(wr_frame_pixels),
        .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen), .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst), .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb), .m_axi_wlast(m_axi_wlast), .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready), .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready)
    );

    wire        pipe_wr, pipe_prog_full;
    wire [31:0] pipe_din;
    CMV300_ReadDma #(.AW(AW)) u_rdma (
        .clk(ui_clk), .rst(ui_rst), .start(rd_start), .base(rd_base), .blocks(rd_blocks),
        .busy(rd_busy_ui), .done(rd_done),
        .fifo_wr(pipe_wr), .fifo_din(pipe_din), .fifo_prog_full(pipe_prog_full),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready)
    );

    //------------------------------------------------------------------------
    // Pipe FIFO: ui_clk -> okClk, 32 bits, 16384 deep (64 KiB: deep enough to keep ep_ready high across the
    // inter-frame handover of the streaming mode - a shallow FIFO dropped ready mid-transfer and every stall
    // cost USB turnaround time), standard read (word valid the clock after rd_en)
    //------------------------------------------------------------------------
    wire pipe_wr_rst_busy;
    wire pipe_rd = bt_read | (drain & ~pipe_empty & ~pipe_rd_rst_busy);
    xpm_fifo_async #(
        .CDC_SYNC_STAGES(2), .DOUT_RESET_VALUE("0"), .ECC_MODE("no_ecc"), .FIFO_MEMORY_TYPE("block"),
        .FIFO_READ_LATENCY(1), .FIFO_WRITE_DEPTH(16384), .FULL_RESET_VALUE(0), .PROG_EMPTY_THRESH(10),
        .PROG_FULL_THRESH(16000), .RD_DATA_COUNT_WIDTH(15), .READ_DATA_WIDTH(32), .READ_MODE("std"),
        .RELATED_CLOCKS(0), .SIM_ASSERT_CHK(0), .USE_ADV_FEATURES("0707"), .WAKEUP_TIME(0),
        .WRITE_DATA_WIDTH(32), .WR_DATA_COUNT_WIDTH(15)
    ) u_pipe_fifo (
        .sleep(1'b0), .rst(ui_rst), .wr_clk(ui_clk), .wr_en(pipe_wr & ~pipe_wr_rst_busy), .din(pipe_din),
        .full(), .prog_full(pipe_prog_full), .wr_data_count(), .overflow(), .wr_rst_busy(pipe_wr_rst_busy),
        .almost_full(), .wr_ack(), .rd_clk(clk), .rd_en(pipe_rd), .dout(bt_data), .empty(pipe_empty),
        .prog_empty(), .rd_data_count(pipe_rd_count), .underflow(), .rd_rst_busy(pipe_rd_rst_busy), .almost_empty(),
        .data_valid(), .injectsbiterr(1'b0), .injectdbiterr(1'b0), .sbiterr(), .dbiterr()
    );
    // one BTPipe block = 1024 bytes = 256 words
    always @(posedge clk) bt_ready <= (pipe_rd_count >= 15'd256) & ~drain;

    //------------------------------------------------------------------------
    // Status words
    //------------------------------------------------------------------------
    assign wo24 = {wi04[4], wi04[3], wi04[1], lvl_s1[5], ctrl_state, spi_rd[7:0],
                   lvl_s1[2], pf_ovf_sticky, lvl_s1[0], lvl_s1[1], wi04[0], frame_req, snap_done, snap_armed,
                   1'b0, frames_ready_ok, lvl_s1[3], spi_busy, lvl_s1[4], sensor_ready, lock_s[1]};
    assign wo25 = {frames_stored_ok, frame_count};
    assign wo26 = {last_frame_rows, last_row_pixels};
    assign wo27 = clk_out_meas;
    assign wo28 = spi_rd;
    assign wo29 = {16'd0, snap_words};
    assign wo2a = {6'd0, and_mask, 6'd0, or_mask};
    assign wo2b = {read_seq_ok, ring_dbg_ok[7:4], 2'd0, read_slot_ok, read_blocks_ok};
    assign wo2c = {8'd0, ring_dbg_ok, frames_dropped_ok};
    assign wo2d = {8'd0, read_pixels_ok};
    assign wo2e = {8'd0, frame_period};
    assign wo2f = lvds_status;
    assign to61 = {27'd0, frame_ready_ok, read_done_ok, snap_done_pulse, frame_done_pulse, spi_done};
endmodule
