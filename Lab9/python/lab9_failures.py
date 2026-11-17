#!/usr/bin/env python3
"""
lab9_failures.py - a closer look at the three deliberate failures of Milestone 3c
and at the masks of 3d. Saves results/lab9_failures_<timestamp>.json.

    python lab9_failures.py [file.bit]
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sensor_board import parse_bitfile_arg          # noqa: E402
from cmv300 import CMV300, RECOMMENDED, EP_SPI_CMD, EP_TRIG_CAM, T_SPI_GO, T_FRAME_REQ   # noqa: E402
from lab9_measure import counters, exp_ms_of, RESULTS   # noqa: E402


def snap(cam, label, log):
    c = counters(cam)
    c["label"] = label
    log.append(c)
    print("  %-34s frames %3d  lval_edges %5d  rows %3d  px/row %3d  clk %5d  req->lval %6d us" %
          (label, c["frames"], c["lval_edges"], c["rows_per_frame"], c["pixels_per_row"], c["clk_out_per_ms"], c["req_to_lval"]))
    return c


def main():
    R = {"started": time.strftime("%Y-%m-%d %H:%M:%S")}
    with CMV300(parse_bitfile_arg()) as cam:
        cam.power_up(); cam.cam_reset(); cam.configure(); time.sleep(0.01)

        # ---- failure 1: reg 69 = 0 ----
        print("failure 1: reg 69 = 0")
        log = []
        cam.clear_counters(); cam.frame_req(); time.sleep(0.15); snap(cam, "good frame first", log)
        cam.spi_write(69, 0); time.sleep(0.01)
        cam.clear_counters(); time.sleep(0.01); snap(cam, "reg 69 = 0, cleared, no request", log)
        cam.frame_req(); time.sleep(0.15); snap(cam, "reg 69 = 0, after FRAME_REQ", log)
        cam.spi_write(69, 9); time.sleep(0.01)
        R["failure1"] = log

        # ---- failure 2: Number_frames = 0 ----
        print("failure 2: Number_frames = 0")
        log = []
        cam.spi_write(55, 0); cam.spi_write(56, 0)
        R["nf_readback"] = cam.spi_read(55, 2)
        cam.clear_counters(); time.sleep(0.1); snap(cam, "Number_frames = 0, no request, 100 ms", log)
        cam.frame_req()
        for ms in (100, 300, 600):
            time.sleep(0.1 if ms == 100 else (0.2 if ms == 300 else 0.3))
            snap(cam, "Number_frames = 0, %d ms after REQ" % ms, log)
        cam.spi_write(55, 1); time.sleep(0.2)
        a = snap(cam, "Number_frames back to 1, +200 ms", log)
        time.sleep(0.3)
        b = snap(cam, "Number_frames back to 1, +500 ms", log)
        R["nf0_still_running_after_restore"] = b["frames"] != a["frames"]
        cam.cam_reset(); cam.configure(); time.sleep(0.01)
        cam.clear_counters(); cam.frame_req(); time.sleep(0.15); snap(cam, "after reset + configure, one REQ", log)
        R["failure2"] = log

        # ---- failure 3: FRAME_REQ straight after the SPI upload ----
        print("failure 3: FRAME_REQ straight after the upload")
        log = []
        for last in (69, 83):
            for trial in range(3):
                cam.cam_reset()
                cam.clear_counters()
                for r in [x for x in RECOMMENDED if x != last]:
                    cam.spi_write(r, RECOMMENDED[r])
                # last write and the frame request back to back, no status poll in between
                cam.wire_in(EP_SPI_CMD, (1 << 15) | (last << 8) | RECOMMENDED[last])
                t0 = time.perf_counter()
                cam.dev.ActivateTriggerIn(EP_TRIG_CAM, T_SPI_GO)
                cam.dev.ActivateTriggerIn(EP_TRIG_CAM, T_FRAME_REQ)
                dt = (time.perf_counter() - t0) * 1e3
                time.sleep(0.15)
                c = snap(cam, "reg %d last, REQ %.2f ms later (host)" % (last, dt), log)
                c["host_ms"] = round(dt, 3); c["last_reg"] = last
                time.sleep(0.01)
                cam.clear_counters(); cam.frame_req(); time.sleep(0.15)
                snap(cam, "  same, second REQ after waiting", log)
        R["failure3"] = log

        # ---- masks against exposure ----
        print("masks")
        cam.cam_reset(); cam.configure(); time.sleep(0.01)
        R["masks"] = []
        for ms in (0.5, 1, 2, 4, 6.344, 10, 20, 40, 80):
            v = cam.set_exposure_ms(ms)
            cam.frame_req(); time.sleep(ms / 1000.0 + 0.1)
            m = cam.pixel_masks()
            R["masks"].append({"exposure_ms": round(exp_ms_of(v), 3), "or": "0x%03X" % m["or"], "and": "0x%03X" % m["and"]})
            print("  exposure %7.3f ms  OR 0x%03X  AND 0x%03X" % (exp_ms_of(v), m["or"], m["and"]))
        R["temp_raw"] = cam.read_temperature()
        cam.power_down()

    os.makedirs(RESULTS, exist_ok=True)
    out = os.path.join(RESULTS, "lab9_failures_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w") as f:
        json.dump(R, f, indent=1)
    print("saved", out)


if __name__ == "__main__":
    main()
