#!/usr/bin/env python3
"""
lab10_host.py - ECE 437 Lab 10 host code: every milestone measurement in one script.

    python lab10_host.py [file.bit] m1     # first light: test pattern, snapshot image, geometry, masks, exposure
    python lab10_host.py [file.bit] m2     # full 648 x 488 frames through DDR3: test pattern, then real frames
    python lab10_host.py [file.bit] m3a    # mode A (paced) vs free-run: delivered fps, sensor period, drops
    python lab10_host.py [file.bit] m3b    # free-run with the host slowed down on purpose, drop fraction vs prediction
    python lab10_host.py [file.bit] m3c    # the ceiling: host wait time, ring occupancy, USB time per frame
    python lab10_host.py [file.bit] all    # m1, m2, m3a, m3b, m3c in order (one bit-file load)
    python lab10_host.py plot              # redraw the drop-fraction plot from results/lab10_m3b.json (no board)

Options:  --rates 20,40,60,75   host rates for m3b (fps)      --frames N   frames per measurement (default 100)
          --exp-short MS / --exp-long MS   the two exposures of m1 (default 1 and 10 ms)

Everything is saved to ../results/ (JSON per milestone, PGM + NPY images, PNG plots when matplotlib is there).
The board is driven through python/cmv300.py (the given host API); nothing here touches the FPGA design.
"""
import json
import os
import sys
import time

from cmv300 import (CMV300, COLS, EP_CAM_STATUS, EP_READ_LEN, EP_READ_PIX, EP_BTPIPE, T_READ_REQ, T_READ_ACK,
                    T_SNAP_ARM, save_pgm, stats, tp_check)
from sensor_board import parse_bitfile_arg

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.normpath(os.path.join(HERE, "..", "results"))
PRELAB_SENSOR_FPS = 77.6                 # pre lab 3(c): s = 1 / 12.88 ms, measured in Lab 9
PRELAB_RATES = (40, 60, 70)              # the three host rates the pre lab evaluated


# ---------------------------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------------------------
def save_json(name, data):
    os.makedirs(RESULTS, exist_ok=True)
    path = os.path.join(RESULTS, name)
    with open(path, "w") as f:
        json.dump(data, f, indent=1)
    print("saved %s" % path)
    return path


def save_image(name, px):
    """PGM (opens anywhere) + NPY (exact numbers) + PNG if matplotlib is installed."""
    os.makedirs(RESULTS, exist_ok=True)
    base = os.path.join(RESULTS, name)
    save_pgm(base + ".pgm", px)
    try:
        import numpy as np
        np.save(base + ".npy", np.asarray(px, dtype="uint16"))
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        a = np.asarray(px)
        fig, ax = plt.subplots(figsize=(6.48, 6.48 * a.shape[0] / a.shape[1] + 0.4))
        ax.imshow(a, cmap="gray", vmin=0, vmax=1023, interpolation="nearest")
        ax.set_title("%s  (%d x %d, mean %.1f DN)" % (name, a.shape[1], a.shape[0], float(a.mean())))
        ax.set_xlabel("column (pixel)"); ax.set_ylabel("row")
        fig.tight_layout(); fig.savefig(base + ".png", dpi=120); plt.close(fig)
    except ImportError:
        pass
    print("saved %s.pgm" % base)


def seq_gaps(seqs):
    """Frames missing between consecutive sequence numbers (8-bit, wraps at 256)."""
    return sum(((seqs[i + 1] - seqs[i]) & 0xFF) - 1 for i in range(len(seqs) - 1))


def timed_grab(cam, timeout=2.0):
    """cmv300.grab_frame() with timing: returns (pixels, info) where info also has
    wait_s (host idle, no READY frame), xfer_s (read_req .. last BTPipe byte) and ready (frames_ready seen)."""
    import numpy as np
    t0 = time.perf_counter()
    while True:
        st = cam.wire_out(EP_CAM_STATUS)
        ready = (st >> 5) & 7
        if ready:
            break
        if time.perf_counter() - t0 > timeout:
            raise TimeoutError("no frame ready: %s %s" % (cam.cam_status(), cam.ring_status()))
    t1 = time.perf_counter()
    cam.cam_trigger(T_READ_REQ)
    blocks = pixels = seq = 0
    err = False
    for _ in range(200):
        ln, pxn, s2 = cam.wire_outs(EP_READ_LEN, EP_READ_PIX, EP_CAM_STATUS)
        blocks, pixels, seq, err = ln & 0xFFFF, pxn & 0xFFFFFF, (ln >> 24) & 0xFF, bool(s2 & 0x8000)
        if blocks or err:
            break
    if err or blocks == 0:
        raise RuntimeError("read_req refused: %s %s" % (cam.cam_status(), cam.ring_status()))
    buf = cam.read_btpipe(EP_BTPIPE, blocks * 1024)
    cam.cam_trigger(T_READ_ACK)
    t2 = time.perf_counter()
    rows = pixels // COLS
    raw8 = np.frombuffer(bytes(buf), dtype=np.uint8)
    px = cam._frame_from_flat(np, cam._unpack12(np, raw8, rows * COLS), rows)
    return px, {"blocks": blocks, "pixels": pixels, "rows": rows, "bytes": len(buf), "seq": seq,
                "wait_s": t1 - t0, "xfer_s": t2 - t1, "ready": ready}


def sensor_fps(cam):
    us = cam.frame_period_us()
    return (1e6 / us if us else 0.0), us / 1e3


def snapshot_frame(cam, rows=244):
    """Snapshot path (block RAM, PipeOut 0xA0): one frame of `rows` rows."""
    px, info = cam.snapshot(rows=rows)
    return px, info


# ---------------------------------------------------------------------------------------------
# Milestone 1: first light
# ---------------------------------------------------------------------------------------------
def m1(cam, o):
    out = {}
    print("\n=== Milestone 1: first light ===")
    # 1. test pattern first: the counter pattern through YOUR packer (it only runs with acquisition on)
    rows = 244
    cam.acq_start(rows=rows, test_pattern=True)
    total_bad, firsts = 0, []
    for i in range(5):
        px, info = timed_grab(cam)
        bad, first = tp_check(px, info["rows"])
        total_bad += bad; firsts.append(first)
        print("test pattern frame %d: %d x %d, first pixel %d, mismatched pixels %d" % (i, COLS, info["rows"], first, bad))
    cam.acq_stop()
    out["test_pattern"] = {"frames": 5, "rows": rows, "mismatched_pixels": total_bad, "first_pixels": firsts}
    print("TEST PATTERN: %d mismatched pixels in 5 frames of %d x %d  -> %s" % (total_bad, COLS, rows, "PASS" if total_bad == 0 else "FAIL"))

    # 2. first light: a real image through the snapshot block RAM, 244 rows (every second row of the field)
    cam.power_up()
    cam.configure(rows=rows, skip=1)                       # default exposure (reg 42-44 = 232 lines)
    snapshot_frame(cam, rows)                              # discard the first frame after configuration
    px, geo = snapshot_frame(cam, rows)
    s = stats(px)
    save_image("lab10_m1_first_light", px)
    print("geometry: %d pixels per row, %d rows; OR mask 0x%03X, AND mask 0x%03X; mean %.1f DN" %
          (geo["pixels_per_row"], geo["rows_per_frame"], geo["or_mask"], geo["and_mask"], s["mean"]))
    out["first_light"] = {"geometry": geo, "stats": s}
    ok_geo = geo["pixels_per_row"] == 648 and geo["rows_per_frame"] == rows
    ok_mask = geo["or_mask"] == 0x3FF and geo["and_mask"] == 0
    print("GEOMETRY 648 x %d: %s   MASKS OR=0x3FF AND=0: %s" % (rows, "PASS" if ok_geo else "FAIL", "PASS" if ok_mask else "CHECK (point the camera at a scene with dark and bright parts)"))

    # 3. exposure responds: short vs long
    means = {}
    for label, ms in (("short", o["exp_short"]), ("long", o["exp_long"])):
        cam.set_exposure_ms(ms)
        snapshot_frame(cam, rows)                          # the new exposure applies to the next frame: discard one
        px, _ = snapshot_frame(cam, rows)
        means[label] = {"exposure_ms": ms, "mean_dn": stats(px)["mean"]}
        save_image("lab10_m1_exposure_%s" % label, px)
        print("exposure %-5s %5.1f ms -> mean %.1f DN" % (label, ms, means[label]["mean_dn"]))
    out["exposure"] = means
    print("EXPOSURE RESPONDS: %s" % ("PASS" if means["long"]["mean_dn"] > means["short"]["mean_dn"] + 5 else "CHECK (more light, or a longer --exp-long)"))
    save_json("lab10_m1.json", out)
    return out


# ---------------------------------------------------------------------------------------------
# Milestone 2: full frames through DDR3, module unchanged
# ---------------------------------------------------------------------------------------------
def m2(cam, o):
    out = {}
    n = max(30, o["frames"] if o["frames"] >= 30 else 30)
    rows = 488
    print("\n=== Milestone 2: full frames through DDR3 ===")
    cam.acq_start(rows=rows, test_pattern=True)
    total_bad, firsts, seqs = 0, [], []
    t0 = time.time()
    for i in range(n):
        px, info = timed_grab(cam)
        bad, first = tp_check(px, info["rows"])
        total_bad += bad; firsts.append(first); seqs.append(info["seq"])
        if bad or i < 2 or i == n - 1:
            print("tp frame %3d: seq %3d, %d x %d, %d blocks, first %4d, bad %d" % (i, info["seq"], COLS, info["rows"], info["blocks"], first, bad))
    dt = time.time() - t0
    skipped = sum(((firsts[i + 1] - firsts[i]) & 0x3FF) - 1 for i in range(len(firsts) - 1))
    cam.acq_stop()
    out["test_pattern"] = {"frames": n, "rows": rows, "mismatched_pixels": total_bad, "generator_frames_skipped": skipped,
                           "seq_gaps": seq_gaps(seqs), "fps": n / dt, "blocks": info["blocks"]}
    print("TEST PATTERN %d frames of 648 x 488: %d mismatched pixels, %d frames skipped, %d sequence gaps -> %s" %
          (n, total_bad, skipped, seq_gaps(seqs), "PASS" if total_bad == 0 and skipped == 0 and seq_gaps(seqs) == 0 else "FAIL"))

    # real frames, mode A: the frame counter must step by exactly one per frame
    cam.acq_start(rows=rows)
    seqs, means = [], []
    rs0 = cam.ring_status()
    t0 = time.time()
    for i in range(n):
        px, info = timed_grab(cam)
        seqs.append(info["seq"]); means.append(float(px.mean()))
    dt = time.time() - t0
    rs1 = cam.ring_status()
    cam.acq_stop()
    save_image("lab10_m2_full_frame", px)
    out["real_frames"] = {"frames": n, "rows": rows, "seq_first": seqs[0], "seq_last": seqs[-1], "seq_gaps": seq_gaps(seqs),
                          "dropped": rs1["frames_dropped"] - rs0["frames_dropped"], "fps": n / dt,
                          "mean_dn_first_last": [means[0], means[-1]]}
    print("REAL FRAMES mode A: %d frames, seq %d..%d, gaps %d, dropped %d, %.1f fps -> %s" %
          (n, seqs[0], seqs[-1], seq_gaps(seqs), out["real_frames"]["dropped"], n / dt,
           "PASS" if seq_gaps(seqs) == 0 and out["real_frames"]["dropped"] == 0 else "FAIL"))
    print("BRAM used: read build/post_route_utilization.rpt (Block RAM Tile) and compare with the pre lab (a 16-bit frame = 130.7%)")
    save_json("lab10_m2.json", out)
    return out


# ---------------------------------------------------------------------------------------------
# Milestone 3a: mode A vs free-run
# ---------------------------------------------------------------------------------------------
def run_rate(cam, n, free_run, host_fps=None, test_pattern=False, check_tp=False):
    """n frames; host_fps paces the read loop (None = as fast as possible). Returns a result dict."""
    rows = 488
    cam.acq_start(rows=rows, free_run=free_run, test_pattern=test_pattern)
    for _ in range(3):
        timed_grab(cam)                                    # warm up: the first frames come from the start-up flush
    rs0 = cam.ring_status()
    seqs, waits, xfers, readies, bad_total = [], [], [], [], 0
    period = 1.0 / host_fps if host_fps else 0.0
    t0 = time.perf_counter(); t_next = t0
    for i in range(n):
        if period:
            now = time.perf_counter()
            if t_next > now:
                time.sleep(t_next - now)                   # the deliberate slowdown
            t_next += period
        px, info = timed_grab(cam)
        seqs.append(info["seq"]); waits.append(info["wait_s"]); xfers.append(info["xfer_s"]); readies.append(info["ready"])
        if check_tp:
            bad, _ = tp_check(px, info["rows"])
            bad_total += bad
    dt = time.perf_counter() - t0
    rs1 = cam.ring_status()
    fps_s, period_ms = sensor_fps(cam)
    cam.acq_stop()
    dropped = (rs1["frames_dropped"] - rs0["frames_dropped"]) & 0xFFFF
    gaps = seq_gaps(seqs)
    res = {"free_run": free_run, "host_target_fps": host_fps, "frames": n, "delivered_fps": n / dt,
           "sensor_period_ms": period_ms, "sensor_fps": fps_s, "dropped": dropped, "seq_gaps": gaps,
           "drop_fraction": gaps / float(gaps + n),       # sensor frames in the run = received + lost
           "seqs": seqs, "host_wait_ms_mean": 1e3 * sum(waits) / n, "host_wait_fraction": sum(waits) / dt,
           "xfer_ms_mean": 1e3 * sum(xfers) / n, "frames_ready_max": max(readies),
           "mb_per_s": n * 475136 / dt / 1e6}
    if check_tp:
        res["tp_mismatched_pixels"] = bad_total
    return res


def m3a(cam, o):
    print("\n=== Milestone 3a: the two acquisition modes ===")
    out = {}
    for label, fr in (("mode_A", False), ("free_run", True)):
        r = run_rate(cam, o["frames"], fr)
        out[label] = r
        print("%-9s delivered %.1f fps; sensor period %.2f ms (%.1f fps); dropped %d; sequence gaps %d" %
              (label, r["delivered_fps"], r["sensor_period_ms"], r["sensor_fps"], r["dropped"], r["seq_gaps"]))
    print("Mode A must show 0 dropped. Mode A runs one frame at a time (exposure, then readout); free-run pipelines")
    print("the next exposure under the current readout - the two Lab 9 timing regimes.")
    save_json("lab10_m3a.json", out)
    return out


# ---------------------------------------------------------------------------------------------
# Milestone 3b: make it drop on purpose
# ---------------------------------------------------------------------------------------------
def m3b(cam, o):
    print("\n=== Milestone 3b: free-run with a slowed-down host ===")
    out = {"rates": [], "prelab_sensor_fps": PRELAB_SENSOR_FPS}
    for h in o["rates"]:
        r = run_rate(cam, o["frames"], True, host_fps=h)
        s = r["sensor_fps"] or PRELAB_SENSOR_FPS
        r["predicted_drop_fraction"] = max(0.0, 1.0 - r["delivered_fps"] / s)
        r["prelab_drop_fraction"] = max(0.0, 1.0 - h / PRELAB_SENSOR_FPS)
        out["rates"].append(r)
        print("host %4.0f fps: delivered %.1f fps, sensor %.1f fps, dropped %d, gaps %d -> drop fraction %.3f "
              "(pre lab %.3f, from measured s %.3f)" % (h, r["delivered_fps"], r["sensor_fps"], r["dropped"], r["seq_gaps"],
                                                        r["drop_fraction"], r["prelab_drop_fraction"], r["predicted_drop_fraction"]))
        print("   sequence numbers received: %s ..." % r["seqs"][:20])
    # integrity while dropping: the test pattern in free-run at the slowest rate, every pixel checked
    h = min(o["rates"])
    r = run_rate(cam, max(30, o["frames"] // 2), True, host_fps=h, test_pattern=True, check_tp=True)
    out["integrity"] = {"host_target_fps": h, "frames": r["frames"], "seq_gaps": r["seq_gaps"],
                        "tp_mismatched_pixels": r["tp_mismatched_pixels"], "seqs": r["seqs"]}
    print("INTEGRITY at %d fps (test pattern, free-run): %d frames received, %d frames lost as sequence gaps, "
          "%d mismatched pixels -> %s" % (h, r["frames"], r["seq_gaps"], r["tp_mismatched_pixels"],
                                          "whole frames lost, received frames exact" if r["tp_mismatched_pixels"] == 0 else "FAIL"))
    save_json("lab10_m3b.json", out)
    plot_m3b(out)
    return out


def plot_m3b(out=None):
    if out is None:
        with open(os.path.join(RESULTS, "lab10_m3b.json")) as f:
            out = json.load(f)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed: plot skipped (numbers are in lab10_m3b.json)")
        return
    pts = out["rates"]
    s_meas = sum(p["sensor_fps"] for p in pts) / len(pts)
    hs = [x * 0.5 for x in range(0, 2 * int(max(85, s_meas + 5)))]
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    ax.plot(hs, [max(0, 1 - h / PRELAB_SENSOR_FPS) for h in hs], "k--", lw=1, label="pre lab: 1 - h/s, s = %.1f fps" % PRELAB_SENSOR_FPS)
    ax.plot([p["delivered_fps"] for p in pts], [p["drop_fraction"] for p in pts], "o", ms=7, color="#B06A3B", label="measured")
    for p in pts:
        ax.annotate("%.0f" % p["host_target_fps"], (p["delivered_fps"], p["drop_fraction"]), textcoords="offset points", xytext=(6, 4), fontsize=8)
    ax.set_xlabel("host read rate h (frames/s)")
    ax.set_ylabel("fraction of sensor frames dropped")
    ax.set_xlim(0, max(hs)); ax.set_ylim(-0.02, 1.0)
    ax.grid(alpha=0.3); ax.legend()
    ax.set_title("Free-run drop fraction vs host rate (sensor measured %.1f fps)" % s_meas)
    fig.tight_layout()
    path = os.path.join(RESULTS, "lab10_m3b_drop_fraction.png")
    fig.savefig(path, dpi=150); plt.close(fig)
    print("saved %s" % path)


# ---------------------------------------------------------------------------------------------
# Milestone 3c: name the ceiling
# ---------------------------------------------------------------------------------------------
def m3c(cam, o):
    print("\n=== Milestone 3c: the ceiling ===")
    r = run_rate(cam, o["frames"], True)
    usb_fps = 1e3 / r["xfer_ms_mean"] if r["xfer_ms_mean"] else 0
    # USB capacity on its own: streaming mode reads many frames per call (no per-frame handshake)
    cam.acq_start(rows=488, free_run=True)
    cam.stream_on()
    t0 = time.perf_counter(); n = 0
    while n < 60:
        cam.read_stream(10, 488, unpack=False); n += 10
    st_dt = time.perf_counter() - t0
    rs = cam.ring_status()
    cam.stream_off(); cam.acq_stop()
    out = {"free_run": r, "usb_only_fps_from_xfer_time": usb_fps,
           "stream_fps": n / st_dt, "stream_mb_s": n * 475136 / st_dt / 1e6, "stream_dropped_total": rs["frames_dropped"]}
    print("free-run, host as fast as it can: delivered %.1f fps, sensor %.1f fps, dropped %d" % (r["delivered_fps"], r["sensor_fps"], r["dropped"]))
    print("host waited for a READY frame %.0f%% of the time (mean %.2f ms per frame); ring held at most %d frame(s)" %
          (100 * r["host_wait_fraction"], r["host_wait_ms_mean"], r["frames_ready_max"]))
    print("one frame over USB (read_req .. read_ack): %.2f ms -> the link alone could carry %.0f fps; streaming mode: %.1f fps, %.0f MB/s" %
          (r["xfer_ms_mean"], usb_fps, out["stream_fps"], out["stream_mb_s"]))
    limit = "the sensor" if r["dropped"] == 0 and r["host_wait_fraction"] > 0.1 and usb_fps > r["sensor_fps"] * 1.2 else "check the numbers"
    out["verdict"] = limit
    print("CEILING: %s (delivered = sensor rate, no drops, and the host sits idle waiting for frames)" % limit)
    save_json("lab10_m3c.json", out)
    return out


# ---------------------------------------------------------------------------------------------
def parse(argv):
    o = {"rates": [20, 40, 60, 75], "frames": 100, "exp_short": 1.0, "exp_long": 10.0}
    rest, i = [], 0
    while i < len(argv):
        a = argv[i]
        if a == "--rates": o["rates"] = [float(x) for x in argv[i + 1].split(",")]; i += 1
        elif a == "--frames": o["frames"] = int(argv[i + 1]); i += 1
        elif a == "--exp-short": o["exp_short"] = float(argv[i + 1]); i += 1
        elif a == "--exp-long": o["exp_long"] = float(argv[i + 1]); i += 1
        elif not a.endswith(".bit"): rest.append(a)
        i += 1
    return o, rest


def main():
    o, args = parse(sys.argv[1:])
    cmd = args[0] if args else "all"
    if cmd == "plot":
        plot_m3b(); return
    steps = {"m1": [m1], "m2": [m2], "m3a": [m3a], "m3b": [m3b], "m3c": [m3c], "all": [m1, m2, m3a, m3b, m3c]}
    if cmd not in steps:
        print(__doc__); return
    with CMV300(parse_bitfile_arg()) as cam:
        cam.warn_if_usb2()
        print("Board %s, design ID 0x%08X, bit file %s" % (cam.model, cam.design_id(), cam.bitfile))
        cam.wait_ddr3()
        for step in steps[cmd]:
            step(cam, o)


if __name__ == "__main__":
    main()
