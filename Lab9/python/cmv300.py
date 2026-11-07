#!/usr/bin/env python3
"""
cmv300.py - Lab 9 host side: power the CMV300 up, program its registers over
SPI, and measure everything it does with the FPGA's counters. There is no
image capture in this lab - the counters ARE the instrument.

    python cmv300.py [file.bit] test        # power-up, defaults, write/read/reset, CLK_OUT, temperature
    python cmv300.py [file.bit] regs        # dump all 128 registers
    python cmv300.py [file.bit] read N | write N V
    python cmv300.py [file.bit] clk         # CLK_OUT cycles per ms (25,000 +/- 16 when configured)
    python cmv300.py [file.bit] geom        # pixels/row, rows/frame, frames, after a frame request
    python cmv300.py [file.bit] period      # frame period in ms (needs Number_frames >= 2 per request)
    python cmv300.py [file.bit] expsweep    # exposure sweep: measured period vs exposure
    python cmv300.py [file.bit] masks       # OR / AND of every pixel of the last frame
    python cmv300.py [file.bit] temp        # on-chip temperature (regs 78/79, burst read)

Endpoints (hdl/CMV300_Subsystem.v): WireIn 0x04 cam_ctrl, 0x05 spi_cmd; WireOut
0x24 status, 0x25 frames/LVAL edges, 0x26 geometry, 0x27 CLK_OUT per ms, 0x28 SPI
data, 0x29 frame-to-frame period (us), 0x2A OR/AND masks, 0x2B FRAME_REQ->first
LVAL (us), 0x2C readout (us); TriggerIn 0x41.

Register facts (CMV300.pdf v2.1/v2.11): parallel mode needs 57 = 3, 69 = 9
(CLK_OUT), 68 = 1 (10-bit), 83 = 251 (PLL band for 25 MHz - the 155 default
locks off-band at ~36.7 MHz); Number_lines = regs 1/2; exposure regs 42-44 in
units of 325 CLK_IN periods; temperature = regs 78/79, ~4900 + 13/degC.
"""
import sys
import time

from sensor_board import SensorBoard, parse_bitfile_arg

EP_CAM_CTRL, EP_SPI_CMD = 0x04, 0x05
EP_CAM_STATUS, EP_FRAMES, EP_GEOMETRY, EP_CLK_MEAS, EP_SPI_DATA, EP_PERIOD, EP_MASKS = 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A
EP_T_REQ_LVAL, EP_T_READOUT = 0x2B, 0x2C
EP_TRIG_CAM = 0x41
T_SPI_GO, T_FRAME_REQ, T_CAM_RESET, T_CLEAR, T_EXP1 = 0, 1, 4, 5, 8
OKCLK_MHZ = 100.8
CLK_IN_HZ = 25.0e6
LINE_CLKS = 325.0                      # exposure unit: Exp_time x 325 CLK_IN periods (section 5.1)
CTRL_STATES = {0: "OFF", 1: "CLK_ON", 2: "RES_WAIT", 3: "READY", 4: "RES_LOW"}

DEFAULTS = {1: 232, 2: 1, 42: 232, 61: 85, 63: 12, 77: 128, 81: 126, 83: 155, 85: 3, 99: 255}
RECOMMENDED = {57: 3, 68: 1, 69: 9, 83: 251,
               58: 44, 59: 240, 60: 10, 80: 2, 97: 240, 98: 10, 100: 112, 101: 98, 102: 34, 103: 64,
               106: 94, 107: 110, 108: 91, 109: 82, 110: 80, 117: 91}


class CMV300(SensorBoard):
    """SensorBoard plus the Lab 9 camera endpoints. Everything I2C keeps working."""

    def cam_trigger(self, bit):
        self.dev.ActivateTriggerIn(EP_TRIG_CAM, bit)

    def cam_status(self):
        s = self.wire_out(EP_CAM_STATUS)
        return {"clk_locked": bool(s & 1), "ready": bool(s & 2), "spi_busy": bool(s & 8),
                "frame_req": bool(s & 0x400), "cam_enable": bool(s & 0x800),
                "spi_byte0": (s >> 16) & 0xFF,
                "state": CTRL_STATES.get((s >> 24) & 0xF, str((s >> 24) & 0xF)), "raw": s}

    # ---- power ------------------------------------------------------------
    def power_up(self, timeout=0.5):
        """cam_enable = 1: CLK_IN on, >= 1 us, SYS_RES_N released, >= 1 us -> READY (section 3.7)."""
        self.wire_in(EP_CAM_CTRL, 1, 1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.cam_status()["ready"]:
                return self.cam_status()
            time.sleep(0.001)
        raise TimeoutError("sequencer did not reach READY: %s" % self.cam_status())

    def power_down(self):
        self.wire_in(EP_CAM_CTRL, 0, 1)

    def cam_reset(self, timeout=0.5):
        self.cam_trigger(T_CAM_RESET)
        time.sleep(0.001)
        return self.power_up(timeout) if self.cam_status()["cam_enable"] else self.cam_status()

    # ---- SPI --------------------------------------------------------------
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
        v = self._spi(addr, burst=n - 1)
        return [(v >> (8 * i)) & 0xFF for i in range(n)]

    def spi_read16(self, addr):
        lo, hi = self.spi_read(addr, 2)
        return lo | (hi << 8)

    def spi_write16(self, addr, value):
        self.spi_write(addr, value & 0xFF)
        self.spi_write(addr + 1, (value >> 8) & 0xFF)

    # ---- configuration ------------------------------------------------------
    def configure(self, rows=488, exposure_ms=None, extra=None):
        """Parallel 10-bit mode with the recommended analogue settings; waits 2 ms for the PLL."""
        for reg, val in RECOMMENDED.items():
            self.spi_write(reg, val)
        self.spi_write16(1, rows)
        if exposure_ms is not None:
            self.set_exposure_ms(exposure_ms)
        for reg, val in (extra or {}).items():
            self.spi_write(reg, val)
        time.sleep(0.002)
        bad = {r: self.spi_read(r)[0] for r, v in RECOMMENDED.items() if self.spi_read(r)[0] != v}
        if bad:
            raise RuntimeError("register read-back mismatch after configure: %s" % bad)

    def set_exposure_ms(self, ms):
        v = max(1, min(0xFFFFFF, int(round(ms * 1e-3 * CLK_IN_HZ / LINE_CLKS))))
        self.spi_write(42, v & 0xFF); self.spi_write(43, (v >> 8) & 0xFF); self.spi_write(44, (v >> 16) & 0xFF)
        return v

    # ---- the meters ---------------------------------------------------------
    def clk_out_per_ms(self):
        return self.wire_out(EP_CLK_MEAS)

    def geometry(self):
        g, f = self.wire_outs(EP_GEOMETRY, EP_FRAMES)
        return {"pixels_per_row": g & 0xFFFF, "rows_per_frame": g >> 16,
                "frames": f & 0xFFFF, "lval_edges": f >> 16}

    def pixel_masks(self):
        m = self.wire_out(EP_MASKS)
        return {"or": m & 0x3FF, "and": (m >> 16) & 0x3FF}

    def frame_period_us(self):
        """Frame-to-frame period in us (needs Number_frames >= 2 in one request)."""
        return self.wire_out(EP_PERIOD) & 0xFFFFFF

    def frame_timing_us(self):
        """The three intervals of the given meters, all in us: FRAME_REQ -> first LVAL
        (= exposure + FOT of a single frame), first LVAL -> last LVAL (readout), and
        frame-to-frame period (pipelined regime; needs >= 2 frames)."""
        return {"req_to_lval": self.wire_out(EP_T_REQ_LVAL) & 0xFFFFFF,
                "readout": self.wire_out(EP_T_READOUT) & 0xFFFFFF,
                "period": self.wire_out(EP_PERIOD) & 0xFFFFFF}

    def frame_req(self):
        self.cam_trigger(T_FRAME_REQ)

    def clear_counters(self):
        self.cam_trigger(T_CLEAR)

    def read_temperature(self):
        """Raw 16-bit temperature (regs 78/79 in one burst). ~4900 + 13/degC at 25 MHz, uncalibrated."""
        return self.spi_read16(78)


def main():
    bit = parse_bitfile_arg()
    args = [a for a in sys.argv[1:] if not a.endswith(".bit")]
    cmd = args[0] if args else "test"

    with CMV300(bit) as cam:
        if cmd == "test":
            ok = True
            print("Board %s, design ID 0x%08X" % (cam.model, cam.design_id()))
            st = cam.power_up(); print("power-up      : %s" % st["state"])
            cam.cam_reset()
            print("CLK_OUT/ms    : %d (expected 0: reg 69 is 0 after reset)" % cam.clk_out_per_ms())
            for reg, want in DEFAULTS.items():
                got = cam.spi_read(reg)[0]
                ok &= got == want
                print("reg %3d default = %3d (want %3d) %s" % (reg, got, want, "ok" if got == want else "MISMATCH"))
            cam.spi_write(1, 100); got = cam.spi_read(1)[0]
            print("write reg 1 = 100, read back %d %s" % (got, "ok" if got == 100 else "MISMATCH")); ok &= got == 100
            cam.cam_reset(); got = cam.spi_read(1)[0]
            print("after cam_reset reg 1 = %d %s" % (got, "ok" if got == 232 else "MISMATCH")); ok &= got == 232
            cam.spi_write(69, 9); time.sleep(0.005)
            wrong = cam.clk_out_per_ms()
            cam.spi_write(83, 251); time.sleep(0.005)
            meas = cam.clk_out_per_ms()
            print("reg 69 = 9, band 155 (default) -> CLK_OUT/ms = %d (off-band, ~36,7xx on this board)" % wrong)
            print("reg 69 = 9, band 251           -> CLK_OUT/ms = %d %s" % (meas, "ok" if 24984 <= meas <= 25016 else "UNEXPECTED"))
            ok &= 24984 <= meas <= 25016
            t = cam.read_temperature()
            print("temperature raw = %d (~%.1f degC uncalibrated)" % (t, (t - 4900) / 13.0))
            print("TEST: %s" % ("PASS" if ok else "FAIL"))
        elif cmd == "regs":
            cam.power_up()
            for r in range(128):
                v = cam.spi_read(r)[0]
                print("%3d: %3d (0x%02X)%s" % (r, v, v, "\n" if r % 8 == 7 else ""), end="  ")
            print()
        elif cmd == "read":
            cam.power_up(); print(cam.spi_read(int(args[1]))[0])
        elif cmd == "write":
            cam.power_up(); cam.spi_write(int(args[1]), int(args[2]))
            print("reg %s = %d" % (args[1], cam.spi_read(int(args[1]))[0]))
        elif cmd == "clk":
            cam.power_up(); cam.configure(); time.sleep(0.005)
            print("CLK_OUT cycles/ms = %d" % cam.clk_out_per_ms())
        elif cmd == "geom":
            cam.power_up(); cam.configure(); cam.clear_counters()
            cam.frame_req(); time.sleep(0.05)
            print(cam.geometry())
        elif cmd == "period":
            cam.power_up(); cam.configure()
            cam.clear_counters()
            cam.frame_req(); time.sleep(0.1)            # ONE frame: the sequential regime
            t1 = cam.frame_timing_us()
            print("single frame  : FRAME_REQ->LVAL %.2f ms (exposure+FOT), readout %.2f ms, end to end %.2f ms" %
                  (t1["req_to_lval"] / 1e3, t1["readout"] / 1e3, (t1["req_to_lval"] + t1["readout"]) / 1e3))
            cam.spi_write(55, 4)                        # pipelined: the next exposure overlaps the readout
            cam.frame_req(); time.sleep(0.3)
            t2 = cam.frame_timing_us()
            print("pipelined     : frame-to-frame period %.2f ms (FOT + max(readout, exposure))" % (t2["period"] / 1e3))
        elif cmd == "expsweep":
            cam.power_up(); cam.configure()
            cam.spi_write(55, 2)                        # pipelined regime: period = FOT + max(readout, exposure)
            print("exposure_ms   period_ms   (flat at ~12.9 while the readout hides the exposure; knee at Exp_time ~= 976 = 12.7 ms)")
            for ms in (1, 6, 12, 20, 40):
                cam.set_exposure_ms(ms)
                cam.frame_req(); time.sleep(2 * ms / 1000.0 + 0.15)
                print("  %6.1f     %8.3f" % (ms, cam.frame_period_us() / 1000.0))
        elif cmd == "masks":
            cam.power_up(); cam.configure()
            cam.frame_req(); time.sleep(0.05)
            m = cam.pixel_masks()
            print("OR = 0x%03X  AND = 0x%03X   (a stuck data line shows here: a bit that is 1 in AND or 0 in OR)" % (m["or"], m["and"]))
        elif cmd == "temp":
            cam.power_up()
            t = cam.read_temperature()
            print("raw %d  ~%.1f degC (uncalibrated)" % (t, (t - 4900) / 13.0))
        else:
            print(__doc__)


if __name__ == "__main__":
    main()
