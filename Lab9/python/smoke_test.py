#!/usr/bin/env python3
"""
smoke_test.py - is the toolchain, the USB link, the bit file and the sensor
alive? Run this FIRST, against the shipped reference bit file, before touching
any HDL: a pass here means every later problem is in your modules.

    python smoke_test.py [file.bit]      -> one RESULT: ALL PASS line
"""
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from sensor_board import DESIGN_ID, parse_bitfile_arg   # noqa: E402
from cmv300 import CMV300                               # noqa: E402

fails = 0
def check(name, cond, detail=""):
    global fails
    print("%s %s%s" % ("PASS" if cond else "FAIL", name, (" - " + str(detail)) if detail else ""))
    if not cond:
        fails += 1

def main():
    with CMV300(parse_bitfile_arg()) as cam:
        did = cam.design_id()
        check("design ID", did == DESIGN_ID, "0x%08X (want 0x%08X) - a stale bit file fails here" % (did, DESIGN_ID))
        if did != DESIGN_ID:
            print("RESULT: 1 FAILED"); return 1
        st = cam.power_up()
        check("power-up reaches READY", st["ready"], st["state"])
        cam.cam_reset()
        v = cam.spi_read(1)[0]
        check("SPI default reg 1 = 232", v == 232, v)
        cam.spi_write(1, 100)
        check("SPI write/read-back", cam.spi_read(1)[0] == 100)
        cam.cam_reset()
        check("reset restores defaults", cam.spi_read(1)[0] == 232)
        cam.configure()
        time.sleep(0.005)
        clk = cam.clk_out_per_ms()
        check("CLK_OUT = 25,000/ms +/- 16", 24984 <= clk <= 25016, clk)
        cam.clear_counters()
        cam.frame_req()                         # ONE frame: the sequential regime
        time.sleep(0.1)
        g = cam.geometry()
        check("648 pixels per row", g["pixels_per_row"] == 648, g)
        check("488 rows per frame", g["rows_per_frame"] == 488, g)
        check("1 frame counted", g["frames"] == 1, g)
        t = cam.frame_timing_us()
        check("FRAME_REQ->LVAL ~6.5 ms (exposure + FOT)", 6000 < t["req_to_lval"] < 7200, t)
        check("readout ~12.7 ms", 12300 < t["readout"] < 13100, t)
        cam.spi_write(55, 4)                    # pipelined regime
        cam.frame_req()
        time.sleep(0.3)
        p = cam.frame_period_us() / 1000.0
        check("pipelined period 12.88 ms +/- 2%", 12.6 < p < 13.2, "%.3f ms" % p)
        check("pipelined < sequential end-to-end", p * 1000 < t["req_to_lval"] + t["readout"], p)
        m = cam.pixel_masks()
        check("pixel masks plausible (OR != 0, AND != 0x3FF)", m["or"] != 0 and m["and"] != 0x3FF, m)
        t = cam.read_temperature()
        check("temperature plausible (4500..6500 raw)", 4500 < t < 6500, t)
    print("RESULT: %s" % ("ALL PASS" if fails == 0 else "%d FAILED" % fails))
    return fails

if __name__ == "__main__":
    sys.exit(1 if main() else 0)
