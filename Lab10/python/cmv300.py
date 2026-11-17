#!/usr/bin/env python3
"""
cmv300.py - Talk to the CMV300 image sensor: power it up, program its registers over SPI,
read the on-chip temperature, grab a snapshot, stream frames through the DDR3 ring (Phases 0-2 of docs/CMV300_CAPTURE_PLAN.md).

Usage:
    python cmv300.py [file.bit] test            # Phase-0 acceptance: power-up, register defaults, write/read/reset, temperature, CLK_OUT
    python cmv300.py [file.bit] snap [out.pgm]  # configure (244 rows, every 2nd row), grab one frame, print statistics, save PGM (+ .npy)
    python cmv300.py [file.bit] regs            # dump all 128 registers
    python cmv300.py [file.bit] temp            # on-chip temperature (regs 78/79, burst read)
    python cmv300.py [file.bit] read REG | write REG VAL
    python cmv300.py [file.bit] ring [N] [ROWS]   # N frames through the DDR3 ring + BTPipeOut: fps, MB/s, sensor period, gaps
    python cmv300.py [file.bit] tp [N] [ROWS]     # N test-pattern frames, every pixel verified
    python cmv300.py [file.bit] live [ROWS]       # live view (matplotlib), close the window to stop
    python cmv300.py [file.bit] bench [ROWS] [N]  # mode A vs free-running mode B: delivered fps, sensor fps, drops
    python cmv300.py [file.bit] extexp [MS]       # external-exposure experiment (T_EXP1 ... FRAME_REQ, reg 41 = 1)
    python cmv300.py [file.bit] stream [N] [ROWS]  # N frames in streaming mode (ring auto-serves, multi-frame reads)
    python cmv300.py --lvds lvds [status|scan|tap LANE TAP|realign]   # LVDS build: lane lock, eye scan, IDELAY taps
    add --clk40 to any --lvds command: CLK_IN = 40 MHz -> 480 Mbit/s lanes, ~480 fps (40 MHz sensor variant only)
  options for ring/tp/live/bench:  --free-run  --exp MS  --pga 0..7  --adc 0..63  --y0 ROW  --skip N  --period MS
  --lvds loads bitfile/SensorBoard_Top_lvds.bit (the LVDS receiver build); the class detects the build from
  WireOut 0x2F and then programs 4 LVDS outputs / 12-bit words, waits for the lane lock in acq_start, and
  de-interleaves the two rows per burst (Fig. 19) in grab_frame; pixels are 12-bit (info["bits"] = 12).

Endpoints (hdl/CMV300_Subsystem.v): WireIn 0x04 cam_ctrl, 0x05 spi_cmd; WireOut 0x24 cam_status,
0x25 frames, 0x26 geometry, 0x27 CLK_OUT per ms, 0x28 SPI data, 0x29 snapshot words, 0x2A OR/AND masks;
TriggerIn 0x41; TriggerOut 0x61; PipeOut 0xA0 = snapshot, three 10-bit pixels per 32-bit word.

Register facts (CMV300.pdf v2.1 / v2.11, see the plan §1): parallel mode needs 57 = 3, 69 = 9 (CLK_OUT),
68 = 1 (10-bit), 83 = 251 (PLL band for 25 MHz); Number_lines = regs 1/2; Sub_s = regs 35/36 skips rows;
temperature = regs 78/79 burst, about 4900 + 13 x T(degC) at 25 MHz.
"""
import json
import os
import sys
import time

from sensor_board import SensorBoard, parse_bitfile_arg, LVDS_BITFILE

EP_CAM_CTRL, EP_SPI_CMD = 0x04, 0x05
EP_CAM_STATUS, EP_FRAMES, EP_GEOMETRY, EP_CLK_MEAS, EP_SPI_DATA, EP_SNAP, EP_MASKS = 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A
EP_TP_ROWS, EP_READ_LEN, EP_RING, EP_READ_PIX, EP_FRAME_PERIOD = 0x06, 0x2B, 0x2C, 0x2D, 0x2E
EP_TRIG_CAM, EP_TRIG_CAM_OUT, EP_PIPE, EP_BTPIPE = 0x41, 0x61, 0xA0, 0xA1
T_SPI_GO, T_FRAME_REQ, T_SNAP_ARM, T_REWIND, T_CAM_RESET, T_CLEAR, T_READ_REQ, T_READ_ACK, T_EXP1 = 0, 1, 2, 3, 4, 5, 6, 7, 8
T_TAP_LOAD, T_REALIGN = 9, 10
EP_LVDS_CTRL, EP_LVDS_STATUS = 0x07, 0x2F
LVDS_REGS = {57: 0, 68: 0, 81: 126}     # Output_mode = 4 LVDS outputs (§5.7), 12-bit words (§5.9), every channel on (§5.12)
LVDS_LANES = ("Out1", "Out2", "Out3", "Out4", "ctrl")
LVDS_TAPS_FILE = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "build", "lvds_taps.json"))
CLK_IN_HZ = 25.0e6
LINE_CLKS = 325.0                     # exposure unit: Exp_time x 325 x clk_per (CMV300.pdf §5.1)
RING_STATES = {0: "FREE", 1: "WRITING", 2: "READY", 3: "READING"}

COLS = 648
SNAP_WORDS = 52704          # 244 rows x 216 words (three pixels per word)
CTRL_STATES = {0: "OFF", 1: "CLK_ON", 2: "RES_WAIT", 3: "READY", 4: "RES_LOW"}

# datasheet defaults used as the Phase-0 check (CMV300.pdf p.36-38)
DEFAULTS = {1: 232, 2: 1, 42: 232, 43: 1, 61: 85, 63: 12, 77: 128, 81: 126, 83: 155, 85: 3, 99: 255, 127: 255}

# recommended values (v2.1 p.37-38) + the parallel-mode registers; 83 per v2.11 §5.10 for 25 MHz
RECOMMENDED = {57: 3, 68: 1, 69: 9, 83: 251,
               58: 44, 59: 240, 60: 10, 80: 2, 97: 240, 98: 10, 100: 112, 101: 98, 102: 34, 103: 64,
               106: 94, 107: 110, 108: 91, 109: 82, 110: 80, 117: 91}


class CMV300(SensorBoard):
    """SensorBoard plus the camera endpoints. Everything I2C keeps working through the base class."""

    clk_in_hz = CLK_IN_HZ                    # 25 MHz unless set_clk40() switched the LVDS build to 40 MHz

    # ---- low level ------------------------------------------------------------
    def cam_trigger(self, bit):
        self.dev.ActivateTriggerIn(EP_TRIG_CAM, bit)

    @property
    def lvds(self):
        """True when the loaded bit file is the LVDS build (WireOut 0x2F bit 31)."""
        if getattr(self, "_lvds", None) is None:
            self._lvds = bool((self.wire_out(EP_LVDS_STATUS) >> 31) & 1)
        return self._lvds

    def cam_status(self):
        s = self.wire_out(EP_CAM_STATUS)
        return {"clk_locked": bool(s & 1), "ready": bool(s & 2), "calib_done": bool(s & 4), "spi_busy": bool(s & 8),
                "acq_active": bool(s & 0x10), "frames_ready": (s >> 5) & 7,
                "snap_armed": bool(s & 0x100), "snap_done": bool(s & 0x200), "frame_req": bool(s & 0x400),
                "cam_enable": bool(s & 0x800), "wr_busy": bool(s & 0x1000), "rd_busy": bool(s & 0x2000),
                "fifo_overflow": bool(s & 0x4000), "read_err": bool(s & 0x8000), "spi_byte0": (s >> 16) & 0xFF,
                "state": CTRL_STATES.get((s >> 24) & 0xF, str((s >> 24) & 0xF)), "ui_rst": bool(s & (1 << 28)),
                "acq_enable": bool(s & (1 << 29)), "test_pattern": bool(s & (1 << 30)), "raw": s}

    def set_clk40(self, on=True):
        """CLK_IN = 40 MHz (LVDS 480 Mbit/s per lane, ~480 fps; needs the 40 MHz sensor variant) or back to
        25 MHz. LVDS build only - the parallel port is a 25 MHz port on every variant. The MMCM output is
        switched through a glitch-free mux while the sensor is held in reset; configure() then writes the
        matching PLL band (reg 83 = 155 above 32 MHz, 251 below)."""
        if on and not self.lvds:
            raise RuntimeError("40 MHz CLK_IN is for the LVDS build (the parallel port is capped at 25 MHz)")
        want = 40.0e6 if on else CLK_IN_HZ
        if want != self.clk_in_hz and (self.wire_out(EP_CAM_STATUS) & 0x800):
            self.power_down()                # never switch the clock under a running sensor
            time.sleep(0.002)
        self.wire_in(EP_CAM_CTRL, 4 if on else 0, 4)
        self.clk_in_hz = want

    # ---- power ----------------------------------------------------------------
    def power_up(self, timeout=0.5):
        """cam_enable = 1: CLK_IN on, 20 us, SYS_RES_N released, 20 us -> READY (CMV300.pdf §3.7)."""
        self.wire_in(EP_CAM_CTRL, 1, 1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = self.cam_status()
            if st["ready"]:
                return st
            time.sleep(0.001)
        raise TimeoutError("CMV300 sequencer did not reach READY: %s" % self.cam_status())

    def power_down(self):
        self.wire_in(EP_CAM_CTRL, 0, 1)

    def cam_reset(self, timeout=0.5):
        """Full reset sequence (SYS_RES_N low with the clock running -> every register back to default)."""
        self.cam_trigger(T_CAM_RESET)
        time.sleep(0.001)
        return self.power_up(timeout) if self.cam_status()["cam_enable"] else self.cam_status()

    # ---- SPI ------------------------------------------------------------------
    def _spi(self, addr, data=0, write=False, burst=0, timeout=0.1):
        cmd = ((burst & 3) << 16) | ((1 if write else 0) << 15) | ((addr & 0x7F) << 8) | (data & 0xFF)
        self.wire_in(EP_SPI_CMD, cmd)
        self.cam_trigger(T_SPI_GO)
        t0 = time.time()
        while self.cam_status()["spi_busy"]:
            if time.time() - t0 > timeout:
                raise TimeoutError("SPI transfer did not finish")
        return self.wire_out(EP_SPI_DATA)

    def spi_write(self, addr, value):
        self._spi(addr, value, write=True)

    def spi_read(self, addr, n=1):
        """Read n (1..4) consecutive registers in one burst (SPI_EN held high); returns a list."""
        v = self._spi(addr, burst=n - 1)
        return [(v >> (8 * i)) & 0xFF for i in range(n)]

    def spi_read16(self, addr):
        lo, hi = self.spi_read(addr, 2)
        return lo | (hi << 8)

    def spi_write16(self, addr, value):
        self.spi_write(addr, value & 0xFF)
        self.spi_write(addr + 1, (value >> 8) & 0xFF)

    # ---- configuration -----------------------------------------------------------
    def configure(self, rows=244, skip=1, exposure=None, extra=None):
        """Parallel 10-bit mode with the recommended analogue settings; rows = Number_lines,
        skip = Sub_s/Sub_a (1 = every second row -> 244 rows cover the whole 488-row field of view).
        Waits 2 ms for the PLL (§3.7) afterwards."""
        want = dict(RECOMMENDED)
        if self.lvds:
            want.update(LVDS_REGS)              # 4 LVDS outputs, 12-bit words, every channel on
            rows += rows & 1                    # a burst carries two rows
            if self.clk_in_hz > 32e6:
                want[83] = 155                  # PLL band 32-50 MHz (§5.10) - 251 is the 10-32 MHz band
        for reg, val in want.items():
            self.spi_write(reg, val)
        self.spi_write16(1, rows)
        self.spi_write16(35, skip)
        self.spi_write16(37, skip)
        if exposure is not None:
            self.spi_write(42, exposure & 0xFF)
            self.spi_write(43, (exposure >> 8) & 0xFF)
            self.spi_write(44, (exposure >> 16) & 0xFF)
        for reg, val in (extra or {}).items():
            self.spi_write(reg, val)
        time.sleep(0.002)
        if self.lvds:
            self.lvds_select(0)                 # WireIn 0x07 carries the training pattern the data lanes align on: 0x000 after a load
            self.lvds_realign()                 # the PLL band (reg 83) changed the LVDS clock: BUFR + ISERDES need a reset after it
            time.sleep(0.002)
            self.lvds_load_taps()               # a bit-file load resets the IDELAYs to tap 10, which sits almost on the data
                                                # transition of this board - restore the taps of the last eye scan
        bad = {r: self.spi_read(r)[0] for r, v in want.items() if self.spi_read(r)[0] != v}
        if bad:
            raise RuntimeError("register read-back mismatch after configure: %s" % bad)

    # ---- sensor settings (any time; the sensor applies them to the next frame) -----------
    def set_exposure_ms(self, ms):
        """Exp_time (regs 42-44) = ms * 25e6 / 325 (§5.1); returns the value written."""
        v = max(1, min(0xFFFFFF, int(round(ms * 1e-3 * self.clk_in_hz / LINE_CLKS))))
        self.spi_write(42, v & 0xFF); self.spi_write(43, (v >> 8) & 0xFF); self.spi_write(44, (v >> 16) & 0xFF)
        return v

    def set_gain(self, pga=None, adc=None):
        """PGA_gain reg 80 (0: x1, 1: x1.25, 2: x1.5, 3: x1.75, 4: x2, 5: x2.5, 6: x3, 7: x3.5);
        ADC_gain reg 100 = 0x40 | value (unity ~ 51 below 30 MHz, §5.13.2)."""
        if pga is not None: self.spi_write(80, pga & 7)
        if adc is not None: self.spi_write(100, 0x40 | (adc & 0x3F))

    def set_window(self, y0=0, rows=488, skip=0):
        """Rows y0..y0+rows-1 (regs 3/4, 1/2), every (skip+1)-th row (regs 35-38). The ring adapts."""
        self.spi_write16(3, y0); self.spi_write16(1, rows)
        self.spi_write16(35, skip); self.spi_write16(37, skip)
        self.wire_in(EP_TP_ROWS, rows)

    def frame_period_us(self):
        """FPGA-measured CLK_OUT cycles between the last two frame starts, in us (0 = fewer than two frames)."""
        return (self.wire_out(EP_FRAME_PERIOD) & 0xFFFFFF) / (self.clk_in_hz / 1e6)

    # ---- LVDS receiver (LVDS build only: hdl/CMV300_Lvds.v) -------------------------
    def lvds_status(self):
        """WireOut 0x2F: lane lock, IDELAYCTRL, live control bits, and tap / bitslips / eye-scan hits of the selected lane."""
        s = self.wire_out(EP_LVDS_STATUS)
        return {"lvds_build": bool((s >> 31) & 1), "locked": [bool((s >> i) & 1) for i in range(5)], "all_locked": (s & 0x1F) == 0x1F,
                "idelayctrl_ready": bool(s & 0x20), "clock_running": bool(s & 0x40), "reversed": bool(s & 0x80),
                "hits": (s >> 8) & 0xFF, "slips": (s >> 16) & 7, "half": (s >> 19) & 1, "tap": (s >> 20) & 0x1F,
                "dval": bool(s & (1 << 25)), "lval": bool(s & (1 << 26)), "fval": bool(s & (1 << 27)),
                "in_frame": bool(s & (1 << 28)), "frozen": bool(s & (1 << 29)), "realigning": bool(s & (1 << 30)), "raw": s}

    def lvds_select(self, lane, tap=0, reverse=False, train=0x055, freeze=False):
        """WireIn 0x07: lane 0..3 = Out1..Out4, 4 = control; tap 0..31 (78 ps each); reverse = bit order of the
        halves; freeze = keep the alignment while scanning (a tap beyond a data transition then reads as a miss)."""
        v = ((train & 0xFFF) << 16) | ((1 if freeze else 0) << 9) | ((1 if reverse else 0) << 8) | ((lane & 7) << 5) | (tap & 0x1F)
        self.wire_in(EP_LVDS_CTRL, v)

    def lvds_set_tap(self, lane, tap, **kw):
        self.lvds_select(lane, tap, **kw)
        self.cam_trigger(T_TAP_LOAD)

    def lvds_realign(self):
        """BUFR clear + ISERDES reset + a new alignment search (needed after the sensor's clock changed: reg 83)."""
        self.cam_trigger(T_REALIGN)

    def lvds_load_taps(self, path=LVDS_TAPS_FILE):
        """Load the IDELAY taps saved by the last eye scan (build/lvds_taps.json). True when a file was applied."""
        try:
            with open(path) as f:
                taps = {int(k): int(v) for k, v in json.load(f).items()}
        except (OSError, ValueError):
            return False
        for lane, tap in sorted(taps.items()):
            self.lvds_set_tap(lane, tap)
            time.sleep(0.002)
        return True

    def lvds_wait_lock(self, timeout=0.5, retry_ms=100):
        """Wait for all five lanes; realign and try again every retry_ms (the search is a few microseconds, so a lane
        that has not locked after that is stuck on a bad ISERDES reset, which the realign clears)."""
        self.lvds_select(0)                     # make sure the training pattern (0x055) is on WireIn 0x07
        t0 = time.time(); t_retry = t0
        while time.time() - t0 < timeout:
            st = self.lvds_status()
            if st["all_locked"]:
                return st
            if time.time() - t_retry > retry_ms / 1e3:
                self.lvds_realign(); t_retry = time.time()
            time.sleep(0.002)
        raise RuntimeError("LVDS lanes did not lock: %s  (try `cmv300.py --lvds lvds scan`; check reg 57 = 0, reg 83 = 251, "
                           "Enable_LVDS = 1 and that bank 34 / R8 sits at 2.5 V)" % self.lvds_status())

    def lvds_scan(self, lanes=range(5), settle=0.004, verbose=True):
        """Eye scan while the sensor idles (training pattern on the data lanes). The lanes lock first, then the
        alignment is frozen and every IDELAY tap (78 ps) is tried: matches out of 255 evaluations, 255 = open. With
        the alignment frozen a data transition shows as a closed region (without it a lane simply re-locks one bit
        later on the far side and everything looks open). The tap is then put a quarter period (1.67 ns = 21 taps) away
        from the transition, or in the middle of the eye when both edges are inside the 2.4 ns range, and the lanes
        re-lock. Returns {lane: {"hits", "tap", "closed"}}."""
        BIT_TAPS = max(4, int(round(1e12 / (12 * self.clk_in_hz) / 2 / 78.0)))   # half a bit in 78 ps taps: 21 at 25 MHz, 13 at 40
        self.lvds_wait_lock()
        result = {}
        for lane in lanes:
            hits = []
            for tap in range(32):
                self.lvds_select(lane, tap, freeze=True)
                self.cam_trigger(T_TAP_LOAD)
                time.sleep(settle)
                hits.append(self.lvds_status()["hits"])
            closed, t = [], 0                                  # runs of taps where the frozen alignment misses
            while t < 32:
                if hits[t] < 250:
                    t0 = t
                    while t < 32 and hits[t] < 250:
                        t += 1
                    closed.append((t0, t - 1))
                else:
                    t += 1
            if not closed:
                tap = 15                                       # the whole 2.4 ns range is open: nothing to go on, take the middle
            elif len(closed) == 1:
                c = (closed[0][0] + closed[0][1]) / 2.0        # one transition in range: sit half a bit away from it
                if c + BIT_TAPS <= 31:   tap = int(round(c + BIT_TAPS))
                elif c - BIT_TAPS >= 0:  tap = int(round(c - BIT_TAPS))
                else:                    tap = 31 if (31 - c) >= c else 0
            else:                                              # two transitions: the eye between them
                tap = int(round((closed[0][1] + closed[1][0]) / 2.0))
            self.lvds_select(lane, tap, freeze=False)
            self.cam_trigger(T_TAP_LOAD)
            time.sleep(settle)
            result[lane] = {"hits": hits, "tap": tap, "closed": closed}
            try:
                os.makedirs(os.path.dirname(LVDS_TAPS_FILE), exist_ok=True)
                with open(LVDS_TAPS_FILE, "w") as f:
                    json.dump({str(k): v["tap"] for k, v in result.items()}, f)
            except OSError:
                pass
            if verbose:
                bar = "".join("#" if h >= 250 else ("+" if h >= 128 else ".") for h in hits)
                margin = min([abs(tap - (a + b) / 2.0) for a, b in closed] + [99]) * 0.078
                print("lane %d %-4s |%s| transition(s) %s -> tap %2d (%.2f ns, %s from the nearest transition)" %
                      (lane, LVDS_LANES[lane], bar, closed or "none in range", tap, tap * 0.078,
                       "%.2f ns" % margin if closed else "> 2.4 ns"))
        st = self.lvds_wait_lock()
        if verbose:
            print("all lanes locked: %s" % st["all_locked"])
        return result

    def read_temperature(self):
        """Raw 16-bit temperature (regs 78/79 in one burst). ~4900 + 13/degC at 25 MHz, uncalibrated."""
        return self.spi_read16(78)

    # ---- video ----------------------------------------------------------------
    def clk_out_per_ms(self):
        return self.wire_out(EP_CLK_MEAS)

    def geometry(self):
        g, f, m, w = self.wire_outs(EP_GEOMETRY, EP_FRAMES, EP_MASKS, EP_SNAP)
        return {"pixels_per_row": g & 0xFFFF, "rows_per_frame": g >> 16, "frames": f & 0xFFFF,
                "or_mask": m & 0x3FF, "and_mask": (m >> 16) & 0x3FF, "snap_words": w & 0xFFFF}

    def frame_request(self):
        self.cam_trigger(T_FRAME_REQ)

    def snapshot(self, rows=244, timeout=1.0):
        """Arm the snapshot, request a frame, wait, read the buffer. Returns (pixels, info):
        pixels = list of rows (each a list of 648 ints) or a numpy uint16 array if numpy is available."""
        if self.lvds:
            raise RuntimeError("the LVDS build has no snapshot BRAM: use acq_start() + grab_frame() (cmv300.py snap does)")
        self.cam_trigger(T_SNAP_ARM)
        self.frame_request()
        t0 = time.time()
        while not self.cam_status()["snap_done"]:
            if time.time() - t0 > timeout:
                raise TimeoutError("snapshot did not complete: %s %s" % (self.cam_status(), self.geometry()))
            time.sleep(0.001)
        info = self.geometry()
        nwords = info["snap_words"]
        nbytes = (nwords * 4 + 15) // 16 * 16
        self.cam_trigger(T_REWIND)
        buf = self.read_pipe(nbytes)
        return unpack_words(buf, nwords, rows), info

    # ---- DDR3 frame ring (Phase 2) ----------------------------------------------
    def wait_ddr3(self, timeout=3.0):
        """MIG calibration takes a few hundred ms after configuration (measured ~0.5 s on this board)."""
        t0 = time.time()
        while not self.cam_status()["calib_done"]:
            if time.time() - t0 > timeout:
                raise RuntimeError("DDR3 did not calibrate within %.1f s: %s" % (timeout, self.cam_status()))
            time.sleep(0.02)

    def ddr3_reset(self):
        """Pulse the MIG's sys_rst (WireIn 0x04 bit 7) and wait for calibration again."""
        self.wire_in(EP_CAM_CTRL, 0x80, 0x80); time.sleep(0.01)
        self.wire_in(EP_CAM_CTRL, 0x00, 0x80)
        self.wait_ddr3()

    def ring_status(self):
        a, b, c = self.wire_outs(EP_FRAMES, EP_RING, EP_READ_LEN)
        dbg = (b >> 16) & 0xFF
        return {"frames_seen": a & 0xFFFF, "frames_stored": a >> 16, "frames_dropped": b & 0xFFFF,
                "slots": [RING_STATES[(dbg >> (2 + 2 * i)) & 3] for i in range(3)], "writer_state": dbg & 3,
                "read_blocks": c & 0xFFFF, "read_slot": (c >> 16) & 3, "read_seq": (c >> 24) & 0xFF,
                "frame_period_us": self.frame_period_us()}

    def acq_start(self, rows=488, skip=0, period_ms=0, test_pattern=False, exposure=None, free_run=False, exposure_ms=None, clk40=False):
        """Configure the sensor for `rows` lines (skip = Sub_s) and start acquisition into the ring.
        Mode A (default): one FRAME_REQ per free slot, at most one every period_ms -> no frame is
        ever dropped, sequential cadence. Mode B (free_run): Number_frames = 65535, the sensor
        pipelines at its own rate (~77 fps full frame), frames that find no free slot are dropped
        and counted, sequence numbers (WireOut 0x2B[31:24]) show the gaps.
        test_pattern replaces the sensor's pixels by the counter pattern; the sensor clock is still required."""
        self.wait_ddr3()
        if clk40 != (self.clk_in_hz > 32e6):
            self.set_clk40(clk40)
        self.power_up()
        self.configure(rows=rows, skip=skip, exposure=exposure)
        if self.lvds:
            if not os.path.exists(LVDS_TAPS_FILE):
                print("no LVDS tap calibration yet - running the eye scan once (build/lvds_taps.json)")
                self.lvds_scan()
            self.lvds_wait_lock()               # the lanes align on the training pattern while the sensor idles
        if exposure_ms is not None:
            self.set_exposure_ms(exposure_ms)
        self.spi_write16(55, 65535 if free_run else 1)
        self.wire_in(EP_TP_ROWS, rows)
        ctrl = 1 | (8 if test_pattern else 0) | (0x10 if free_run else 0) | ((period_ms & 0xFF) << 8)
        self.wire_in(EP_CAM_CTRL, ctrl, 0xFFFF)
        time.sleep(0.001)
        self.wire_in(EP_CAM_CTRL, ctrl | 2, 0xFFFF)

    def acq_stop(self):
        self.wire_in(EP_CAM_CTRL, 0, 0x2 | 0x8 | 0x10 | 0xFF00)
        time.sleep(0.02)
        self.spi_write16(55, 1)                     # back to one frame per request

    def grab_frame(self, timeout=1.0):
        """Ring protocol: wait for a READY frame, read_req, read blocks x 1024 B over the BTPipe, read_ack.
        Frames arrive as a packed little-endian 12-bit stream (two pixels per three bytes, 1.5 B/px).
        Returns (pixels as rows x 648 uint16 (numpy) or list of rows, info)."""
        # two USB wire reads per frame: one to see a READY frame, one to fetch its length after read_req
        t0 = time.time()
        while (self.wire_out(EP_CAM_STATUS) >> 5) & 7 == 0:
            if time.time() - t0 > timeout:
                raise TimeoutError("no frame ready: %s %s" % (self.cam_status(), self.ring_status()))
            time.sleep(0.0005)
        self.cam_trigger(T_READ_REQ)
        blocks, pixels, seq, err = 0, 0, 0, False
        for _ in range(200):
            ln, pxn, st = self.wire_outs(EP_READ_LEN, EP_READ_PIX, EP_CAM_STATUS)
            blocks, pixels, seq, err = ln & 0xFFFF, pxn & 0xFFFFFF, (ln >> 24) & 0xFF, bool(st & 0x8000)
            if blocks or err:
                break
        if err or blocks == 0:
            raise RuntimeError("read_req refused: %s %s" % (self.cam_status(), self.ring_status()))
        buf = self.read_btpipe(EP_BTPIPE, blocks * 1024)
        self.cam_trigger(T_READ_ACK)
        rows = pixels // COLS
        try:
            import numpy as np
            raw8 = np.frombuffer(bytes(buf), dtype=np.uint8)
            flat = self._unpack12(np, raw8, rows * COLS)
            px = self._frame_from_flat(np, flat, rows)
        except ImportError:
            data = bytes(buf)
            flat = []
            for k in range(0, (pixels // 2) * 3, 3):
                flat.append(data[k] | ((data[k + 1] & 0x0F) << 8))
                flat.append((data[k + 1] >> 4) | (data[k + 2] << 4))
            px = [flat[i:i + COLS] for i in range(0, rows * COLS, COLS)]
        return px, {"blocks": blocks, "pixels": pixels, "rows": rows, "bytes": len(buf), "seq": seq, "bits": 12 if self.lvds else 10}

    @staticmethod
    def _unpack12(np, raw8, n_px):
        """Packed little-endian 12-bit stream (two pixels per three bytes) -> uint16 pixels."""
        b = raw8[: (n_px // 2) * 3].reshape(-1, 3).astype(np.uint16)
        out = np.empty(b.shape[0] * 2, dtype=np.uint16)
        out[0::2] = b[:, 0] | ((b[:, 1] & 0x0F) << 8)
        out[1::2] = (b[:, 1] >> 4) | (b[:, 2] << 4)
        return out[:n_px]

    def _frame_from_flat(self, np, flat, rows):
        """One frame from its stored 16-bit words: LVDS lane order (Fig. 19) -> raster, else a straight reshape."""
        if self.lvds and rows % 2 == 0:
            g = flat.reshape(rows // 2, 324, 4)
            px = np.empty((rows, COLS), dtype=np.uint16)
            px[0::2, :324] = g[:, :, 0]; px[0::2, 324:] = g[:, :, 1]
            px[1::2, :324] = g[:, :, 2]; px[1::2, 324:] = g[:, :, 3]
            return px
        return flat.reshape(rows, COLS)

    # ---- streaming (wi04[5]): the ring auto-serves frames, the host only reads blocks ---------
    def frame_blocks(self, rows):
        return (rows * COLS * 3 // 2 + 1023) // 1024      # packed 12-bit: 1.5 bytes per pixel

    def stream_on(self):
        """After acq_start(free_run=True): frames arrive in the BTPipe by themselves, oldest first, fixed
        size (frame_blocks(rows) x 1024 bytes each) - no read_req/read_ack, so the per-frame host cost is
        just its share of one large ReadFromBlockPipeOut. Drops are still counted in ring_status()."""
        self.wire_in(EP_CAM_CTRL, 0x20, 0x20)

    def stream_off(self):
        """Leave streaming; the FPGA flushes whatever is left in the pipe by itself."""
        self.wire_in(EP_CAM_CTRL, 0, 0x20)
        time.sleep(0.05)

    def read_stream(self, n, rows, unpack=True):
        """Read n whole frames from the stream in one BTPipe call. unpack=True returns an n x rows x 648
        uint16 array; unpack=False returns the raw packed bytes as an n x frame_bytes uint8 array - unpacking
        costs a few ms of numpy per frame, so at hundreds of fps decode only the frames you will actually use
        (unpack_frame) or the ring overflows while the CPU crunches."""
        import numpy as np
        fb = self.frame_blocks(rows) * 1024
        buf = self.read_btpipe(EP_BTPIPE, n * fb)
        if len(buf) != n * fb:
            raise RuntimeError("stream read returned %d of %d bytes: %s" % (len(buf), n * fb, self.ring_status()))
        raw8 = np.frombuffer(bytes(buf), dtype=np.uint8).reshape(n, fb)
        if not unpack:
            return raw8
        out = np.empty((n, rows, COLS), dtype=np.uint16)
        for i in range(n):
            out[i] = self.unpack_frame(raw8[i], rows)
        return out

    def unpack_frame(self, raw8, rows):
        """One packed frame (uint8 vector, from read_stream(unpack=False)) -> rows x 648 uint16."""
        import numpy as np
        return self._frame_from_flat(np, self._unpack12(np, raw8, rows * COLS), rows)

    def frames(self, n=None, timeout=1.0):
        """Generator: successive ring frames until n (or forever)."""
        k = 0
        while n is None or k < n:
            yield self.grab_frame(timeout)
            k += 1


def tp_check(px, rows):
    """Verify a test-pattern frame: pixel i == (i + first) mod 1024. Returns (bad, first)."""
    flat = [int(p) for row in px for p in row] if not hasattr(px, "reshape") else px.reshape(-1).tolist()
    first = flat[0]
    bad = sum(1 for i, p in enumerate(flat) if p != ((i + first) & 0x3FF))
    return bad, first


def unpack_words(buf, nwords, rows):
    """Three 10-bit pixels per little-endian 32-bit word, [9:0] first; row-major, 648 per row."""
    npix = min(3 * nwords, rows * COLS)
    try:
        import numpy as np
        w = np.frombuffer(bytes(buf[:nwords * 4]), dtype="<u4")
        px = np.stack([w & 0x3FF, (w >> 10) & 0x3FF, (w >> 20) & 0x3FF], axis=1).reshape(-1)[:npix]
        nrows = npix // COLS
        return px[:nrows * COLS].astype("uint16").reshape(nrows, COLS)
    except ImportError:
        import struct
        words = struct.unpack("<%dI" % nwords, bytes(buf[:nwords * 4]))
        flat = []
        for wd in words:
            flat.extend((wd & 0x3FF, (wd >> 10) & 0x3FF, (wd >> 20) & 0x3FF))
        flat = flat[:npix]
        return [flat[i:i + COLS] for i in range(0, len(flat) - COLS + 1, COLS)]


def save_pgm(path, pixels):
    """16-bit binary PGM (maxval 1023, or 4095 for 12-bit LVDS data) - opens in Preview/GIMP/ImageJ, no library needed."""
    rows = len(pixels)
    maxval = 4095 if max(int(max(row)) for row in pixels) > 1023 else 1023
    with open(path, "wb") as f:
        f.write(("P5\n%d %d\n%d\n" % (COLS, rows, maxval)).encode())
        for row in pixels:
            f.write(b"".join(int(p).to_bytes(2, "big") for p in row))


def stats(pixels):
    flat = [int(p) for row in pixels for p in row]
    n = len(flat)
    lsb_nonzero = sum(1 for p in flat if p & 3)
    return {"rows": len(pixels), "min": min(flat), "max": max(flat), "mean": sum(flat) / n,
            "lsb_nonzero_pct": 100.0 * lsb_nonzero / n}


def phase0_test(cam):
    print("Board %s, design ID 0x%08X" % (cam.model, cam.design_id()))
    ok = True
    st = cam.power_up()
    print("power-up      : %s" % st["state"])
    cam.cam_reset()                              # start from the datasheet defaults even if the sensor was configured before
    time.sleep(0.005)
    print("CLK_OUT/ms    : %d (expected 0: reg 69 is 0 after reset)" % cam.clk_out_per_ms())
    for reg, want in DEFAULTS.items():
        got = cam.spi_read(reg)[0]
        flag = "ok" if got == want else "MISMATCH"
        ok &= got == want
        print("reg %3d default = %3d (want %3d) %s" % (reg, got, want, flag))
    cam.spi_write(1, 100)
    got = cam.spi_read(1)[0]
    print("write reg 1 = 100, read back %d %s" % (got, "ok" if got == 100 else "MISMATCH")); ok &= got == 100
    cam.cam_reset()
    got = cam.spi_read(1)[0]
    print("after cam_reset reg 1 = %d %s" % (got, "ok" if got == 232 else "MISMATCH")); ok &= got == 232
    cam.spi_write(69, 9)
    time.sleep(0.005)
    wrong = cam.clk_out_per_ms()
    cam.spi_write(83, 251)                       # PLL band for 25 MHz (v2.11 §5.10) - the sequencer clock depends on it
    time.sleep(0.005)
    meas = cam.clk_out_per_ms()
    print("reg 69 = 9, reg 83 = 155 (default band) -> CLK_OUT/ms = %d (off-band, measured 36,7xx on this board)" % wrong)
    print("reg 69 = 9, reg 83 = 251               -> CLK_OUT/ms = %d %s" % (meas, "ok" if 24900 <= meas <= 25100 else "UNEXPECTED (want 25000 +/- 16)"))
    ok &= 24900 <= meas <= 25100
    for i in range(3):
        t = cam.read_temperature()
        print("temperature raw = %d  (~%.1f degC uncalibrated)" % (t, (t - 4900) / 13.0))
        time.sleep(0.2)
    print("PHASE 0: %s" % ("PASS" if ok else "FAIL"))
    return ok


def _opts(args):
    """--free-run, --exp MS, --pga N, --adc N, --window Y0 ROWS, --skip N, --period MS pulled out of the CLI args."""
    o = {"free_run": False, "exp": None, "pga": None, "adc": None, "y0": 0, "skip": 0, "period": 0, "clk40": False}
    rest = []
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--free-run": o["free_run"] = True
        elif a == "--exp": o["exp"] = float(args[i + 1]); i += 1
        elif a == "--pga": o["pga"] = int(args[i + 1]); i += 1
        elif a == "--adc": o["adc"] = int(args[i + 1]); i += 1
        elif a == "--y0": o["y0"] = int(args[i + 1]); i += 1
        elif a == "--skip": o["skip"] = int(args[i + 1]); i += 1
        elif a == "--period": o["period"] = int(args[i + 1]); i += 1
        elif a == "--lvds": pass                       # bit file choice, handled in main()
        elif a == "--parallel": pass                   # the default bit file, stated explicitly
        elif a == "--clk25": pass                      # the LVDS default clock, stated explicitly (implies --lvds in main)
        elif a == "--clk40": o["clk40"] = True         # LVDS build: CLK_IN = 40 MHz -> 480 Mbit/s, ~480 fps
        else: rest.append(a)
        i += 1
    return o, rest


def _start(cam, rows, o, test_pattern=False):
    cam.warn_if_usb2()
    cam.acq_start(rows=rows, skip=o["skip"], period_ms=o["period"], test_pattern=test_pattern, free_run=o["free_run"], exposure_ms=o["exp"], clk40=o["clk40"])
    if o["pga"] is not None or o["adc"] is not None: cam.set_gain(o["pga"], o["adc"])
    if o["y0"]: cam.set_window(o["y0"], rows, o["skip"])


def main():
    bit = parse_bitfile_arg()
    if bit is None and ("--lvds" in sys.argv or "--clk25" in sys.argv or "--clk40" in sys.argv):
        bit = LVDS_BITFILE                             # --clk25/--clk40 imply the LVDS build
    o, args = _opts([a for a in sys.argv[1:] if not a.endswith(".bit")])
    cmd = args[0] if args else "test"
    with CMV300(bit) as cam:
        if cmd == "test":
            phase0_test(cam)
        elif cmd == "regs":
            cam.power_up()
            for base in range(0, 128, 8):
                print("reg %3d..%3d: %s" % (base, base + 7, " ".join("%3d" % cam.spi_read(r)[0] for r in range(base, base + 8))))
        elif cmd == "temp":
            cam.power_up(); cam.spi_write(83, 251); time.sleep(0.002)
            while True:
                t = cam.read_temperature()
                print("temperature raw = %d  (~%.1f degC uncalibrated)" % (t, (t - 4900) / 13.0))
                time.sleep(0.5)
        elif cmd == "read":
            cam.power_up(); print("reg %d = %d" % (int(args[1], 0), cam.spi_read(int(args[1], 0))[0]))
        elif cmd == "write":
            cam.power_up(); cam.spi_write(int(args[1], 0), int(args[2], 0))
            print("reg %d = %d" % (int(args[1], 0), cam.spi_read(int(args[1], 0))[0]))
        elif cmd == "snap":
            out = args[1] if len(args) > 1 else "cmv300_snapshot.pgm"
            cam.power_up()
            if cam.lvds:                                 # no snapshot BRAM: one frame through the ring
                cam.acq_start(rows=244, skip=1)
                print("LVDS build: 244 rows, every second row, through the DDR3 ring; lanes locked %s; words/ms = %d" %
                      (cam.lvds_status()["locked"], cam.clk_out_per_ms()))
                pixels, info = cam.grab_frame()
                cam.acq_stop()
            else:
                cam.configure(rows=244, skip=1)
                print("configured: 244 rows, every second row; CLK_OUT/ms = %d" % cam.clk_out_per_ms())
                pixels, info = cam.snapshot(rows=244)
            print("geometry: %s" % info)
            s = stats(pixels)
            print("snapshot: rows=%d min=%d max=%d mean=%.1f  D[1:0] non-zero in %.1f%% of pixels" %
                  (s["rows"], s["min"], s["max"], s["mean"], s["lsb_nonzero_pct"]))
            save_pgm(out, pixels)
            print("saved %s" % out)
            try:
                import numpy as np
                np.save(out.rsplit(".", 1)[0] + ".npy", np.asarray(pixels, dtype="uint16"))
                print("saved %s" % (out.rsplit(".", 1)[0] + ".npy"))
            except ImportError:
                pass
        elif cmd == "ring":
            n = int(args[1]) if len(args) > 1 else 10
            rows = int(args[2]) if len(args) > 2 else 488
            _start(cam, rows, o)
            print("acquisition on: %d rows, %s, ring %s" % (rows, "free-run" if o["free_run"] else "one request per slot", cam.ring_status()))
            t0 = time.time(); last = None; seqs = []
            for i, (px, info) in enumerate(cam.frames(n, timeout=2.0)):
                a = px
                m = float(a.mean()) if hasattr(a, "mean") else sum(sum(r) for r in a) / (info["rows"] * COLS)
                if i < 3 or i == n - 1:
                    print("frame %3d: seq %3d, %d rows, %d blocks, mean %.1f" % (i, info["seq"], info["rows"], info["blocks"], m))
                last = px; seqs.append(info["seq"])
            dt = time.time() - t0
            gaps = sum(((seqs[i + 1] - seqs[i]) & 0xFF) - 1 for i in range(len(seqs) - 1))
            rs = cam.ring_status()
            print("%d frames in %.2f s = %.1f fps delivered, %.1f MB/s; sensor frame period %.2f ms (%.1f fps); "
                  "sequence gaps %d, ring dropped %d, overflow %s" %
                  (n, dt, n / dt, n * info["bytes"] / dt / 1e6, rs["frame_period_us"] / 1e3,
                   1e6 / rs["frame_period_us"] if rs["frame_period_us"] else 0, gaps, rs["frames_dropped"], cam.cam_status()["fifo_overflow"]))
            cam.acq_stop()
            if last is not None and hasattr(last, "shape"):
                import numpy as np
                np.save("cmv300_ring_last.npy", last); save_pgm("cmv300_ring_last.pgm", last); print("saved cmv300_ring_last.pgm/.npy")
        elif cmd == "stream":
            n = int(args[1]) if len(args) > 1 else 200
            rows = int(args[2]) if len(args) > 2 else 488
            rows += rows & 1 if cam.lvds else 0
            _start(cam, rows, dict(o, free_run=True))
            cam.stream_on()
            chunk = max(1, min(48, (24 << 20) // (cam.frame_blocks(rows) * 1024)))  # ~24 MB per USB call: amortise the per-call cost
            t0 = time.time(); got = 0; last = None
            while got < n:
                k = min(chunk, n - got)
                last = cam.read_stream(k, rows, unpack=False)      # raw: unpacking every frame would stall the ring
                got += k
            dt = time.time() - t0
            rs = cam.ring_status()
            cam.stream_off(); cam.acq_stop()
            mb = got * cam.frame_blocks(rows) * 1024 / 1e6
            print("%d frames in %.2f s = %.1f fps streamed, %.1f MB/s (%d-frame reads); sensor period %.2f ms (%.1f fps); ring dropped %d; last frame mean %.1f" %
                  (got, dt, got / dt, mb / dt, chunk, rs["frame_period_us"] / 1e3,
                   1e6 / max(rs["frame_period_us"], 1), rs["frames_dropped"], float(cam.unpack_frame(last[-1], rows).mean())))
        elif cmd == "tp" and cam.lvds:
            print("test pattern: the generator lives in the parallel front end - not available in the LVDS build")
        elif cmd == "lvds":
            sub = args[1] if len(args) > 1 else "status"
            if o["clk40"]:
                cam.set_clk40(True)
            cam.power_up()
            cam.configure(rows=488, skip=0)              # PLL band, 4 outputs, 12-bit; the sensor then idles with the training pattern
            if not cam.lvds:
                print("this is the parallel build (WireOut 0x2F = 0); load bitfile/SensorBoard_Top_lvds.bit with --lvds")
            elif sub == "scan":
                cam.lvds_scan()
                print("after the scan: %s" % cam.lvds_status())
            elif sub == "tap":
                cam.lvds_set_tap(int(args[2]), int(args[3]))
                time.sleep(0.01)
                print("lane %s tap %s -> %s" % (args[2], args[3], cam.lvds_status()))
            elif sub == "realign":
                cam.lvds_realign(); time.sleep(0.05)
                print(cam.lvds_status())
            else:
                try:
                    cam.lvds_wait_lock()
                except RuntimeError as exc:
                    print(exc)
                st = cam.lvds_status()
                print("LVDS build: IDELAYCTRL ready %s, clock running %s (words/ms = %d), lanes locked %s, DVAL %d LVAL %d FVAL %d" %
                      (st["idelayctrl_ready"], st["clock_running"], cam.clk_out_per_ms(), st["locked"], st["dval"], st["lval"], st["fval"]))
                for lane in range(5):
                    cam.lvds_select(lane); time.sleep(0.01)          # select only (no load): status shows that lane
                    st = cam.lvds_status()
                    print("  lane %d %-4s: %s  tap %2d  bitslips %d  half %d  hits %3d/255" %
                          (lane, LVDS_LANES[lane], "locked" if st["locked"][lane] else "hunting", st["tap"], st["slips"], st["half"], st["hits"]))
        elif cmd == "tp":
            n = int(args[1]) if len(args) > 1 else 100
            rows = int(args[2]) if len(args) > 2 else 488
            _start(cam, rows, o, test_pattern=True)
            t0 = time.time(); total_bad = 0; firsts = []
            for i, (px, info) in enumerate(cam.frames(n, timeout=2.0)):
                bad, first = tp_check(px, info["rows"])
                total_bad += bad; firsts.append(first)
                if bad or i < 3 or i == n - 1:
                    print("frame %3d: %d px, %d blocks, first=%d, bad=%d" % (i, info["pixels"], info["blocks"], first, bad))
            dt = time.time() - t0
            skipped = sum(((firsts[i + 1] - firsts[i]) & 0x3FF) - 1 for i in range(len(firsts) - 1))
            if o["free_run"]:
                verdict = "%d generator frames skipped (free-run: frames the host did not keep up with, ring dropped %d)" % (skipped, cam.ring_status()["frames_dropped"])
            else:
                verdict = "frame sequence %s" % ("OK" if skipped == 0 else "BROKEN (%d skipped)" % skipped)
            print("%d test-pattern frames in %.2f s = %.1f fps, %.1f MB/s: %d bad pixels, %s, overflow %s" %
                  (n, dt, n / dt, n * info["bytes"] / dt / 1e6, total_bad, verdict, cam.cam_status()["fifo_overflow"]))
            cam.acq_stop()
        elif cmd == "live":
            import numpy as np
            import matplotlib.pyplot as plt
            rows = int(args[1]) if len(args) > 1 else 488
            _start(cam, rows, o)
            fig, ax = plt.subplots(); im = ax.imshow(np.zeros((rows, COLS)), cmap="gray", vmin=0, vmax=1023)
            fig.canvas.manager.set_window_title("CMV300 live - close the window to stop")
            t0 = time.time(); k = 0
            try:
                while plt.fignum_exists(fig.number):
                    px, info = cam.grab_frame(timeout=2.0); k += 1
                    im.set_data(px)
                    ax.set_title("frame %d (seq %d)  host %.1f fps, sensor %.1f fps, dropped %d" %
                                 (k, info["seq"], k / (time.time() - t0), 1e6 / max(cam.frame_period_us(), 1), cam.ring_status()["frames_dropped"]))
                    plt.pause(0.001)
            finally:
                cam.acq_stop()
        elif cmd == "bench":
            rows = int(args[1]) if len(args) > 1 else 488
            n = int(args[2]) if len(args) > 2 else 100
            for label, fr in (("mode A (one request per slot)", False), ("mode B (free-running sensor)", True)):
                oo = dict(o); oo["free_run"] = fr
                _start(cam, rows, oo)
                for _ in cam.frames(3, timeout=2.0): pass          # warm up
                t0 = time.time(); seqs = []
                for px, info in cam.frames(n, timeout=2.0): seqs.append(info["seq"])
                dt = time.time() - t0; rs = cam.ring_status()
                gaps = sum(((seqs[i + 1] - seqs[i]) & 0xFF) - 1 for i in range(len(seqs) - 1))
                print("%-30s %d x %d rows: delivered %.1f fps (%.1f MB/s); sensor period %.2f ms = %.1f fps; gaps %d, dropped %d" %
                      (label, n, rows, n / dt, n * info["bytes"] / dt / 1e6, rs["frame_period_us"] / 1e3,
                       1e6 / rs["frame_period_us"] if rs["frame_period_us"] else 0, gaps, rs["frames_dropped"]))
                cam.acq_stop()
        elif cmd == "extexp":
            # external exposure through the Phase-1 snapshot: reg 41 = 1, T_EXP1 starts, FRAME_REQ ends the exposure (§3.10.2)
            ms = float(args[1]) if len(args) > 1 else 20.0
            cam.power_up(); cam.configure(rows=244, skip=1); cam.spi_write(41, 1); time.sleep(0.002)
            cam.cam_trigger(T_SNAP_ARM)
            cam.cam_trigger(T_EXP1); time.sleep(ms / 1e3); cam.frame_request()
            t0 = time.time()
            while not cam.cam_status()["snap_done"]:
                if time.time() - t0 > 1.0: raise TimeoutError("no frame after T_EXP1 + FRAME_REQ: %s" % cam.geometry())
            info = cam.geometry(); nwords = info["snap_words"]
            cam.cam_trigger(T_REWIND); px = unpack_words(cam.read_pipe((nwords * 4 + 15) // 16 * 16), nwords, 244)
            s = stats(px); cam.spi_write(41, 0)
            print("external exposure %.1f ms (host-timed): %s, mean %.1f max %d" % (ms, info, s["mean"], s["max"]))
        else:
            print(__doc__)


if __name__ == "__main__":
    main()
