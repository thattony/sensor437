#!/usr/bin/env python3
"""
lab9_measure.py - take every Lab 9 milestone measurement in one run and save
the raw numbers to results/lab9_<timestamp>.json (the post lab is written from
that file).

    python lab9_measure.py [file.bit] [--warm SECONDS]

Order (one script, as Milestone 2 asks):
  M1  register defaults, Number_lines = 488, write/read-back, reset, temperature
  M2  CLK_OUT meter at three points: after power-up, reg 69 = 9, reg 83 = 251
  M3a one frame from idle: FRAME_REQ->LVAL, readout, LVAL edges, pixels per row
  M3b pipelined period, exposure sweep (coarse + fine around the knee)
  M3c three deliberate failures and what the counters say
  M3d OR / AND masks at a short and a long exposure
  then the sensor free-runs for --warm seconds (default 300) with the
  temperature logged every 30 s, for the start-vs-warm comparison.
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sensor_board import parse_bitfile_arg          # noqa: E402
from cmv300 import CMV300, RECOMMENDED, CLK_IN_HZ, LINE_CLKS   # noqa: E402

RESULTS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results"))


def exp_ms_of(v):
    return v * LINE_CLKS / CLK_IN_HZ * 1e3


def counters(cam):
    """Everything the FPGA measures, in one dict."""
    d = dict(cam.geometry())
    d.update(cam.frame_timing_us())
    d["clk_out_per_ms"] = cam.clk_out_per_ms()
    m = cam.pixel_masks()
    d["or"], d["and"] = m["or"], m["and"]
    return d


def main():
    bit = parse_bitfile_arg()
    warm_s = 300
    if "--warm" in sys.argv:
        warm_s = int(sys.argv[sys.argv.index("--warm") + 1])
    R = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "bitfile": bit or "default"}

    with CMV300(bit) as cam:
        R["design_id"] = "0x%08X" % cam.design_id()
        t_session = time.time()

        # ---------------- M1: the control plane ----------------
        st = cam.power_up()
        R["power_up_state"] = st["state"]
        R["temp_start_raw"] = cam.read_temperature()
        R["temp_log"] = [[0.0, R["temp_start_raw"]]]
        R["defaults"] = {str(r): cam.spi_read(r)[0] for r in (1, 2, 42, 61, 83)}
        lo, hi = cam.spi_read(1, 2)
        R["number_lines_burst"] = {"reg1": lo, "reg2": hi, "value": lo + 256 * hi}
        cam.spi_write(1, 100)
        R["write100_readback"] = cam.spi_read(1)[0]
        cam.cam_reset()
        R["after_reset_reg1"] = cam.spi_read(1)[0]
        print("M1:", R["defaults"], R["number_lines_burst"], R["write100_readback"], R["after_reset_reg1"],
              "temp", R["temp_start_raw"])

        # ---------------- M2: the clock ----------------
        cam.cam_reset(); time.sleep(0.01)
        a = cam.clk_out_per_ms()
        ra = {"69": cam.spi_read(69)[0], "83": cam.spi_read(83)[0]}
        cam.spi_write(69, 9); time.sleep(0.01)
        b = [cam.clk_out_per_ms() for _ in range(20) if not time.sleep(0.002)]
        rb = {"69": cam.spi_read(69)[0], "83": cam.spi_read(83)[0]}
        cam.spi_write(83, 251); time.sleep(0.01)
        c = [cam.clk_out_per_ms() for _ in range(50) if not time.sleep(0.002)]
        rc = {"69": cam.spi_read(69)[0], "83": cam.spi_read(83)[0]}
        R["clk"] = {"after_power_up": {"regs": ra, "meter": a},
                    "reg69_9_band155": {"regs": rb, "meter": b[0], "samples": b},
                    "reg69_9_band251": {"regs": rc, "meter": c[0], "samples": c}}
        print("M2: %d -> %d -> %d  (band-251 samples min %d max %d)" % (a, b[0], c[0], min(c), max(c)))

        # ---------------- M3a: one frame from idle ----------------
        cam.cam_reset()
        cam.configure()                       # parallel 10-bit, reg 69 = 9, reg 83 = 251, default exposure
        time.sleep(0.01)
        R["exp_time_default"] = cam.spi_read16(42)
        R["number_frames_default"] = cam.spi_read(55)[0]
        cam.clear_counters()
        cam.frame_req(); time.sleep(0.15)
        R["single"] = counters(cam)
        print("M3a:", R["single"])

        # ---------------- M3b: pipelined + exposure law ----------------
        cam.spi_write(55, 8)
        cam.frame_req(); time.sleep(0.25)
        R["pipelined"] = counters(cam)
        print("M3b pipelined period: %.3f ms" % (R["pipelined"]["period"] / 1e3))

        cam.spi_write(55, 3)
        sweep = []
        fine = [x / 4.0 for x in range(40, 65)]                 # 10.0 .. 16.0 ms in 0.25 ms steps
        for ms in sorted(set([1, 2, 4, 6, 8, 20, 25, 30, 40] + fine)):
            v = cam.set_exposure_ms(ms)
            cam.frame_req(); time.sleep(3 * max(13.0, ms) / 1000.0 + 0.08)
            p = cam.frame_period_us()
            sweep.append({"exp_time": v, "exposure_ms": round(exp_ms_of(v), 4), "period_us": p})
            print("  exposure %7.3f ms (Exp_time %4d) -> period %7.3f ms" % (exp_ms_of(v), v, p / 1e3))
        R["sweep"] = sweep

        # ---------------- M3d: masks at two exposures ----------------
        cam.spi_write(55, 1)
        R["masks"] = []
        for ms in (6.344, 40):               # 1 ms is too dark for the top bits with the room light
            v = cam.set_exposure_ms(ms)
            cam.frame_req(); time.sleep(ms / 1000.0 + 0.1)
            m = cam.pixel_masks()
            R["masks"].append({"exposure_ms": round(exp_ms_of(v), 3), "or": "0x%03X" % m["or"], "and": "0x%03X" % m["and"]})
        print("M3d:", R["masks"])
        cam.spi_write16(42, 488); cam.spi_write(44, 0)

        # ---------------- M3c: three deliberate failures ----------------
        fails = {}
        # control: everything correct
        cam.clear_counters(); cam.frame_req(); time.sleep(0.15)
        fails["control_good_frame"] = counters(cam)

        # 1. reg 69 = 0, pulse FRAME_REQ
        cam.spi_write(69, 0); time.sleep(0.01)
        cam.clear_counters(); cam.frame_req(); time.sleep(0.15)
        fails["1_reg69_cleared"] = counters(cam)
        cam.spi_write(69, 9); time.sleep(0.01)

        # 2. Number_frames = 0, pulse FRAME_REQ
        cam.spi_write(55, 0); cam.spi_write(56, 0)
        cam.clear_counters(); cam.frame_req(); time.sleep(0.15)
        fails["2_number_frames_0"] = counters(cam)
        cam.spi_write(55, 1)

        # 3. FRAME_REQ less than 1 ms after the SPI upload: reset, upload, trigger at once
        cam.cam_reset()
        cam.clear_counters()
        order = [r for r in RECOMMENDED if r not in (69, 83)] + [83, 69]
        for r in order:
            cam.spi_write(r, RECOMMENDED[r])
        t0 = time.perf_counter()
        cam.frame_req()
        dt = time.perf_counter() - t0
        time.sleep(0.15)
        fails["3_req_right_after_upload"] = counters(cam)
        fails["3_req_right_after_upload"]["host_ms_upload_to_req"] = round(dt * 1e3, 3)
        # and the same thing done properly, for comparison
        time.sleep(0.01)
        cam.clear_counters(); cam.frame_req(); time.sleep(0.15)
        fails["3_same_after_waiting"] = counters(cam)
        R["failures"] = fails
        for k, v in fails.items():
            print("M3c %-26s frames %d  lval_edges %d  rows %d  px/row %d  clk %d" %
                  (k, v["frames"], v["lval_edges"], v["rows_per_frame"], v["pixels_per_row"], v["clk_out_per_ms"]))

        # ---------------- warm-up: free-run and log the temperature ----------------
        cam.configure()
        cam.spi_write(55, 200)
        next_log = 0.0
        t_warm0 = time.time()
        while time.time() - t_warm0 < warm_s:
            cam.frame_req()                       # 200 frames ~ 2.6 s
            time.sleep(2.7)
            if time.time() - t_warm0 >= next_log:
                t = cam.read_temperature()
                R["temp_log"].append([round(time.time() - t_session, 1), t])
                print("  t = %5.0f s  temperature raw %d" % (time.time() - t_session, t))
                next_log += 30.0
        R["temp_end_raw"] = cam.read_temperature()
        R["temp_end_s"] = round(time.time() - t_session, 1)
        R["temp_log"].append([R["temp_end_s"], R["temp_end_raw"]])
        print("temperature: start %d -> after %.0f s %d" % (R["temp_start_raw"], R["temp_end_s"], R["temp_end_raw"]))
        cam.power_down()

    os.makedirs(RESULTS, exist_ok=True)
    out = os.path.join(RESULTS, "lab9_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
    with open(out, "w") as f:
        json.dump(R, f, indent=1)
    print("saved", out)


if __name__ == "__main__":
    main()
