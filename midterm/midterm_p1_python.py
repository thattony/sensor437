# -*- coding: utf-8 -*-
"""
ECE 437 Midterm Part 1 - the LSM303DLHC accelerometer over I2C, and proof that
the numbers are real.

FPGA: SensorBoard_I2C/hdl/midterm_p1_i2c.v (the Lab 6 generated I2C machine,
extended with multi-byte burst reads, register writes and a run-time SCL
divide) + SensorBoard_I2C/hdl/SensorBoard_Top.v. Build: SensorBoard_I2C/build.sh

Every mode logs what it measured to results/ (CSV = raw samples, JSON =
summary) so Report 1 is written from logged numbers, not from the terminal.

    python midterm_p1_python.py kat        1a  known-answer test (run first after power-up)
    python midterm_p1_python.py live       1b  wake up, print ax ay az |a| continuously (Ctrl-C)
    python midterm_p1_python.py six        1c  six-orientation gravity table (Enter per orientation)
    python midterm_p1_python.py six --auto 1c  same, captures each new still orientation by itself
    python midterm_p1_python.py broken     1c  two orientations with SUB(7) cleared
    python midterm_p1_python.py rate       1d  100+ samples/s for 30 s, independence, ODR sweep
    python midterm_p1_python.py mag            magnetometer: ID registers + field while rotating
    python midterm_p1_python.py demo           kat, then live (what the TA watches)

Register facts used (LSM303DLHC.pdf, Doc ID 018771 Rev 1):
    accelerometer 7-bit address 0x19 -> 0x32 write / 0x33 read          (Table 14 p.20)
    magnetometer  7-bit address 0x1E -> 0x3C write / 0x3D read          (Table 16 p.21)
    SUB(7) = 1 -> register pointer auto-increments during a burst      (p.19-20)
    CTRL_REG1_A 0x20, power-on 0x07 = power-down, X/Y/Z enabled       (Table 17 p.22, Table 19-20 p.24)
    CTRL_REG4_A 0x23: BDU, BLE, FS1-0, HR                               (Table 26-27 p.26)
    STATUS_REG_A 0x27: ZYXOR (bit 7), ZYXDA (bit 3)                     (Table 34-35 p.28)
    OUT_X_L_A .. OUT_Z_H_A 0x28..0x2D, little-endian (BLE = 0), X Y Z  (p.28)
    +-2 g, high resolution: 12-bit left-justified, 1 mg/LSB            (Table 3 p.9)
    magnetometer OUT 0x03..0x08 = X_H X_L Z_H Z_L Y_H Y_L (big-endian, X Z Y order) (Table 17 p.23)
    IRA/IRB/IRC_REG_M 0x0A..0x0C = 0x48 0x34 0x33                      (Table 81-83 p.38-39)
"""
import sys
import os
import time
import json
import csv
import math
import argparse
import statistics

ok_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64'
ok_dll_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64'
sys.path.append(ok_loc)
os.add_dll_directory(ok_dll_loc)
import ok  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
BITFILE = os.path.join(HERE, "SensorBoard_I2C", "bitfile", "SensorBoard_Top.bit")
RESULTS = os.path.join(HERE, "results")
DESIGN_ID = 0xEC437002
CLK_MHZ = 100.8                      # okClk, the FSM clock
TICK_DIVIDE = 252                    # default: 2.5 us per state -> 100 kHz SCL
SCL_HZ = CLK_MHZ * 1e6 / (4 * TICK_DIVIDE)

# ---- FrontPanel endpoints (SensorBoard_Top.v) ----
EP_CTRL, EP_PARAM1, EP_PARAM2, EP_PARAM3 = 0x00, 0x01, 0x02, 0x03
EP_RESULT0, EP_RX_LO, EP_CYCLES, EP_STATUS, EP_RX_HI, EP_COUNT, EP_ID = 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x3F
EP_TRIG_IN = 0x40
BUS1 = 1 << 8                        # param1[8]: the LSM303 is on I2C bus 1

# ---- LSM303DLHC ----
ACC_ADDR, MAG_ADDR = 0x19, 0x1E
AUTO_INC = 0x80                      # SUB(7)
CTRL_REG1_A, CTRL_REG4_A, STATUS_REG_A, OUT_X_L_A = 0x20, 0x23, 0x27, 0x28
CTRL_REG1_A_DEFAULT = 0x07
CRA_REG_M, CRB_REG_M, MR_REG_M, OUT_X_H_M, IRA_REG_M = 0x00, 0x01, 0x02, 0x03, 0x0A
MAG_ID = (0x48, 0x34, 0x33)

ODR_BITS = {1: 0x1, 10: 0x2, 25: 0x3, 50: 0x4, 100: 0x5, 200: 0x6, 400: 0x7}
ODR_HZ = 200                         # chosen output data rate (see rate mode)
CTRL1_RUN = (ODR_BITS[ODR_HZ] << 4) | 0x07          # 0x67: 200 Hz, normal mode, X Y Z on
CTRL4_RUN = 0x88                     # BDU = 1, BLE = 0 (little-endian), FS = +-2 g, HR = 1
MG_PER_LSB = 1.0                     # +-2 g, 12-bit: 1 mg/LSB (Table 3)
ZERO_G_OFFSET_MG = 60                # typical zero-g level offset, +-60 mg (Table 3 p.10)

KAT_VALUES = (0x37, 0x57)            # recognisable values written in 1a step 2 (25 Hz, then 100 Hz)


def fail(msg):
    print("FAIL:", msg)
    sys.exit(1)


def save_json(name, obj):
    os.makedirs(RESULTS, exist_ok=True)
    obj = dict(obj, saved=time.strftime("%Y-%m-%d %H:%M:%S"), bitfile=BITFILE, scl_hz=SCL_HZ)
    p = os.path.join(RESULTS, name)
    with open(p, "w") as f:
        json.dump(obj, f, indent=2)
    print("  logged ->", p)


def save_csv(name, header, rows):
    os.makedirs(RESULTS, exist_ok=True)
    p = os.path.join(RESULTS, name)
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print("  logged ->", p, "(%d rows)" % len(rows))


# ---------------------------------------------------------------------------
# Conversions (checked against hand-made vectors before any hardware is used)
# ---------------------------------------------------------------------------
def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def acc_counts(lo, hi):
    """OUT_x_L_A / OUT_x_H_A -> signed 12-bit count (left-justified in 16 bits)."""
    return s16((hi << 8) | lo) >> 4


def acc_g(lo, hi):
    return acc_counts(lo, hi) * MG_PER_LSB / 1000.0


TEST_VECTORS = [   # (L, H, expected g)
    (0x00, 0x00, 0.000), (0x10, 0x00, 0.001), (0x80, 0x3E, 1.000), (0xF0, 0x7F, 2.047),
    (0x00, 0x80, -2.048), (0xF0, 0xFF, -0.001), (0x80, 0xC1, -1.000),
]


def self_test():
    good = True
    for lo, hi, exp in TEST_VECTORS:
        got = acc_g(lo, hi)
        good &= abs(got - exp) < 1e-9
        if abs(got - exp) >= 1e-9:
            print("  L=0x%02X H=0x%02X expected %.3f got %.3f  WRONG" % (lo, hi, exp, got))
    if not good:
        fail("acceleration conversion does not match the test vectors")


# ---------------------------------------------------------------------------
# FPGA / I2C access
# ---------------------------------------------------------------------------
class Board:
    def __init__(self, load=True):
        self.dev = ok.okCFrontPanel()
        if self.dev.OpenBySerial("") != 0:
            sys.exit("No Opal Kelly board found. Is FrontPanel still open? Close it.")
        if load:
            if not os.path.exists(BITFILE):
                sys.exit("No bit file at %s - run SensorBoard_I2C/build.sh first." % BITFILE)
            if self.dev.ConfigureFPGA(BITFILE) != 0:
                sys.exit("Bitfile did not load: " + BITFILE)
        if not self.dev.IsFrontPanelEnabled():
            fail("FrontPanel is not enabled in the FPGA design.")
        self.dev.UpdateWireOuts()
        did = self.dev.GetWireOutValue(EP_ID) & 0xFFFFFFFF
        if did != DESIGN_ID:
            fail("design ID 0x%08X, expected 0x%08X (wrong bit file?)" % (did, DESIGN_ID))
        self.params = None

    def close(self):
        self.dev.Close()

    def _set(self, p1, p2, p3):
        if self.params != (p1, p2, p3):        # skip the USB write when nothing changed
            self.dev.SetWireInValue(EP_PARAM1, p1)
            self.dev.SetWireInValue(EP_PARAM2, p2)
            self.dev.SetWireInValue(EP_PARAM3, p3)
            self.dev.UpdateWireIns()
            self.params = (p1, p2, p3)

    def transact(self, p1, p2, p3, timeout=0.1):
        """Trigger one transaction; return (rx 8 bytes, cycles, ack_error).

        The FPGA counts completed transactions (WireOut 0x25). Waiting for that
        count to step by exactly one means the bytes returned come from THIS
        transaction and that one trigger produced one transaction.
        """
        self._set(p1, p2, p3)
        d = self.dev
        d.UpdateWireOuts()
        before = d.GetWireOutValue(EP_COUNT)
        d.ActivateTriggerIn(EP_TRIG_IN, 0)
        t0 = time.perf_counter()
        while True:
            d.UpdateWireOuts()
            n = (d.GetWireOutValue(EP_COUNT) - before) & 0xFFFFFFFF
            if n:
                break
            if time.perf_counter() - t0 > timeout:
                fail("no I2C transaction completed within %.0f ms" % (timeout * 1e3))
        if n != 1:
            fail("one trigger produced %d transactions" % n)
        lo = d.GetWireOutValue(EP_RX_LO) & 0xFFFFFFFF
        hi = d.GetWireOutValue(EP_RX_HI) & 0xFFFFFFFF
        rx = (lo | (hi << 32)).to_bytes(8, "little")
        ack_err = bool(d.GetWireOutValue(EP_RESULT0) & 1)
        return rx, d.GetWireOutValue(EP_CYCLES) & 0xFFFFFFFF, ack_err

    # ---- register-level helpers ------------------------------------------
    def read(self, addr7, sub, n=1, divide=0):
        rx, cyc, err = self.transact(BUS1 | (addr7 << 1), sub & 0xFF, (divide << 16) | (n << 4) | 0x1)
        if err:
            fail("NACK reading 0x%02X from device 0x%02X" % (sub, addr7))
        self.last_cycles = cyc
        return list(rx[:n])

    def write(self, addr7, sub, value):
        rx, cyc, err = self.transact(BUS1 | (addr7 << 1), ((value & 0xFF) << 8) | (sub & 0xFF), 0x3)
        if err:
            fail("NACK writing 0x%02X to 0x%02X of device 0x%02X" % (value, sub, addr7))
        return cyc


def configure_acc(b, ctrl1=CTRL1_RUN):
    b.write(ACC_ADDR, CTRL_REG4_A, CTRL4_RUN)
    b.write(ACC_ADDR, CTRL_REG1_A, ctrl1)
    r1, r4 = b.read(ACC_ADDR, CTRL_REG1_A)[0], b.read(ACC_ADDR, CTRL_REG4_A)[0]
    if (r1, r4) != (ctrl1, CTRL4_RUN):
        fail("configuration read back CTRL_REG1_A=0x%02X CTRL_REG4_A=0x%02X" % (r1, r4))
    time.sleep(0.05)                       # first conversion after power-down (turn-on 7/ODR)


def read_xyz(b, sub=AUTO_INC | OUT_X_L_A):
    """One 6-byte burst -> (ax, ay, az) in g, plus the raw bytes."""
    r = b.read(ACC_ADDR, sub, 6)
    return (acc_g(r[0], r[1]), acc_g(r[2], r[3]), acc_g(r[4], r[5])), r


def read_status_xyz(b):
    """One 7-byte burst from STATUS_REG_A: (status, (ax, ay, az), raw counts)."""
    r = b.read(ACC_ADDR, AUTO_INC | STATUS_REG_A, 7)
    c = (acc_counts(r[1], r[2]), acc_counts(r[3], r[4]), acc_counts(r[5], r[6]))
    return r[0], tuple(v * MG_PER_LSB / 1000.0 for v in c), c


def new_sample(b, timeout=0.1):
    """Poll until ZYXDA = 1, return (status, g, counts, polls)."""
    t0 = time.perf_counter()
    polls = 0
    while True:
        st, g, c = read_status_xyz(b)
        polls += 1
        if st & 0x08:
            return st, g, c, polls
        if time.perf_counter() - t0 > timeout:
            fail("ZYXDA never set - accelerometer not converting (CTRL_REG1_A?)")


def mag(v):
    return math.sqrt(sum(x * x for x in v))


# ---------------------------------------------------------------------------
# 1a. Known-answer test
# ---------------------------------------------------------------------------
def run_kat(b):
    print("=" * 64)
    print("1a  KNOWN-ANSWER TEST  (accelerometer 0x19 on I2C bus 1)")
    print("=" * 64)
    log = {"register": "CTRL_REG1_A", "address": "0x20", "documented_default": "0x07"}
    v = b.read(ACC_ADDR, CTRL_REG1_A)[0]
    log["step1_read"] = "0x%02X" % v
    print("Step 1: read CTRL_REG1_A (0x20). Documented power-on default 0x07, read 0x%02X  -> %s"
          % (v, "PASS" if v == CTRL_REG1_A_DEFAULT else "FAIL"))
    step1 = v == CTRL_REG1_A_DEFAULT
    if not step1:
        print("        (0x%02X is not the power-on value: an earlier run already wrote this register.\n"
              "         Power-cycle the board - unplug its power - and run 'kat' first.)" % v)
    log["step1_pass"] = step1
    log["step2"] = []
    step2 = True
    for w in KAT_VALUES:
        b.write(ACC_ADDR, CTRL_REG1_A, w)
        r = b.read(ACC_ADDR, CTRL_REG1_A)[0]
        good = r == w
        step2 &= good
        log["step2"].append({"wrote": "0x%02X" % w, "read_back": "0x%02X" % r, "pass": good})
        print("Step 2: wrote 0x%02X to CTRL_REG1_A, read back 0x%02X  -> %s" % (w, r, "PASS" if good else "FAIL"))
    log["step2_pass"] = step2
    # a neighbouring register must NOT have changed: proves the write hit 0x20 only
    r4 = b.read(ACC_ADDR, CTRL_REG4_A)[0]
    log["ctrl_reg4_a_untouched"] = "0x%02X" % r4
    print("        CTRL_REG4_A (0x23) after the writes: 0x%02X (we never wrote it this power cycle -> 0x00 expected"
          " unless a previous run configured it)" % r4)

    ida = b.read(MAG_ADDR, IRA_REG_M, 3)
    log["mag_id"] = ["0x%02X" % x for x in ida]
    print("Extra:  magnetometer IRA/IRB/IRC (0x1E, 0x0A..0x0C) = %s, expected 48 34 33 -> %s"
          % (" ".join("%02X" % x for x in ida), "PASS" if tuple(ida) == MAG_ID else "FAIL"))
    print("        (proves the MAGNETOMETER address and read path only - not the accelerometer)")
    log["mag_id_pass"] = tuple(ida) == MAG_ID
    save_json("kat.json", log)
    return step1 and step2


# ---------------------------------------------------------------------------
# 1b. Live three-axis read
# ---------------------------------------------------------------------------
def run_live(b, seconds=None):
    configure_acc(b)
    print("CTRL_REG1_A = 0x%02X (%d Hz, normal, XYZ), CTRL_REG4_A = 0x%02X (BDU, +-2 g, HR) -> 1 mg/LSB"
          % (CTRL1_RUN, ODR_HZ, CTRL4_RUN))
    print("Tilt the board. Ctrl-C to stop.\n     ax (g)    ay (g)    az (g)    |a| (g)    raw bytes XL XH YL YH ZL ZH")
    rows = []
    t0 = time.perf_counter()
    try:
        while seconds is None or time.perf_counter() - t0 < seconds:
            g, r = read_xyz(b)
            rows.append([round(time.perf_counter() - t0, 4)] + [round(x, 4) for x in g] + [round(mag(g), 4)]
                        + ["%02X" % x for x in r])
            print("\r  %+8.3f  %+8.3f  %+8.3f   %7.3f     %s" % (g + (mag(g), " ".join("%02X" % x for x in r))),
                  end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print()
    save_csv("live_tilt.csv", ["t_s", "ax_g", "ay_g", "az_g", "mag_g", "XL", "XH", "YL", "YH", "ZL", "ZH"], rows)


# ---------------------------------------------------------------------------
# 1c. Six orientations
# ---------------------------------------------------------------------------
N_AVG = 400            # new samples per orientation = 2 s at 200 Hz


def capture(b, n=N_AVG, sub=None):
    """Average n NEW conversions (ZYXDA-gated). sub given = the broken read path instead."""
    xs = []
    raw = []
    t0 = time.perf_counter()
    while len(xs) < n:
        if sub is None:
            st, g, c, _ = new_sample(b)
            xs.append(g)
        else:
            g, r = read_xyz(b, sub)
            xs.append(g)
            raw.append(r)
    dur = time.perf_counter() - t0
    mean = tuple(statistics.mean(v[i] for v in xs) for i in range(3))
    std = tuple(statistics.stdev(v[i] for v in xs) for i in range(3))
    return mean, std, xs, raw, dur


def label_of(m):
    i = max(range(3), key=lambda k: abs(m[k]))
    return ("+" if m[i] > 0 else "-") + "XYZ"[i]


def run_six(b, auto=False):
    configure_acc(b)
    print("Six orientations: rest the board so gravity is along each sensor axis, both directions.")
    print("Hold it still on the bench; each capture averages %d new conversions (%.1f s).\n" % (N_AVG, N_AVG / ODR_HZ))
    rows, raw_rows, summary = [], [], []
    seen = set()
    k = 0
    while len(summary) < 6:
        if auto:
            # wait until still (0.5 s window, std < 15 mg per axis) and dominant axis not yet captured
            win = []
            while True:
                _, g, _, _ = new_sample(b)
                win.append(g)
                win = win[-ODR_HZ // 2:]
                if len(win) == ODR_HZ // 2:
                    m = tuple(statistics.mean(v[i] for v in win) for i in range(3))
                    s = max(statistics.stdev(v[i] for v in win) for i in range(3))
                    lab = label_of(m)
                    print("\r  waiting: now %s  |a|=%.3f  std=%.3f g  captured %s    " % (lab, mag(m), s, sorted(seen)),
                          end="", flush=True)
                    if s < 0.015 and lab not in seen and max(abs(x) for x in m) > 0.8:
                        print()
                        break
            desc = "auto"
        else:
            desc = input("Orientation %d - describe it (e.g. 'component side up') and press Enter: " % (len(summary) + 1)).strip()
            time.sleep(1.0)                         # hands off the board
        mean, std, xs, _, dur = capture(b)
        lab = label_of(mean)
        if lab in seen:
            print("  gravity along %s was already captured - turn the board to a new face." % lab)
            continue
        seen.add(lab)
        k += 1
        a = mag(mean)
        summary.append({"n": k, "description": desc, "gravity_axis": lab, "ax_g": mean[0], "ay_g": mean[1],
                        "az_g": mean[2], "mag_g": a, "std_x_g": std[0], "std_y_g": std[1], "std_z_g": std[2],
                        "samples": len(xs), "seconds": dur,
                        "mean_of_sample_mag_g": statistics.mean(mag(v) for v in xs)})
        for i, v in enumerate(xs):
            raw_rows.append([k, lab, i] + [round(x, 4) for x in v])
        print("  %d  %-3s  ax %+7.4f  ay %+7.4f  az %+7.4f  |a| %.4f g   (std %.4f %.4f %.4f g, %d samples)"
              % (k, lab, mean[0], mean[1], mean[2], a, std[0], std[1], std[2], len(xs)))
    mags = [s["mag_g"] for s in summary]
    spread = max(mags) - min(mags)
    print("\n  |a| mean %.4f g, spread (max - min) %.4f g  -> %s (limit 0.15 g)"
          % (statistics.mean(mags), spread, "PASS" if spread < 0.15 else "FAIL"))
    # per-axis offset and gain from the +/- pairs (the data, not the datasheet, gives the axis mapping)
    axes = {}
    for ax in "XYZ":
        p = next((s for s in summary if s["gravity_axis"] == "+" + ax), None)
        n = next((s for s in summary if s["gravity_axis"] == "-" + ax), None)
        if p and n:
            key = "a%s_g" % ax.lower()
            axes[ax] = {"plus_g": p[key], "minus_g": n[key], "offset_g": (p[key] + n[key]) / 2,
                        "gain": (p[key] - n[key]) / 2}
            print("  %s axis: +1g reads %+.4f, -1g reads %+.4f -> offset %+.4f g, scale %.4f"
                  % (ax, p[key], n[key], axes[ax]["offset_g"], axes[ax]["gain"]))
    save_csv("six_orientations.csv", list(summary[0].keys()), [list(s.values()) for s in summary])
    save_csv("six_raw_samples.csv", ["orientation", "gravity_axis", "i", "ax_g", "ay_g", "az_g"], raw_rows)
    save_json("six_summary.json", {"orientations": summary, "mag_mean_g": statistics.mean(mags),
                                   "spread_g": spread, "pass": spread < 0.15, "axes": axes,
                                   "odr_hz": ODR_HZ, "ctrl_reg1_a": "0x%02X" % CTRL1_RUN,
                                   "ctrl_reg4_a": "0x%02X" % CTRL4_RUN, "n_avg": N_AVG})


# ---------------------------------------------------------------------------
# 1c. Broken on purpose: SUB(7) cleared, nothing else changed
# ---------------------------------------------------------------------------
def predict_broken(xl):
    """No auto-increment: all six bytes are OUT_X_L_A. Each axis = (XL<<8 | XL) as a 12-bit count."""
    return acc_g(xl, xl)


def run_broken(b):
    configure_acc(b)
    print("Deliberate fault: burst read with SUB = 0x28 (bit 7 cleared) instead of 0xA8.")
    out = []
    for k in range(2):
        input("Broken orientation %d: put the board in one of your six orientations, Enter: " % (k + 1))
        time.sleep(1.0)
        good_mean, _, _, _, _ = capture(b, 200)                       # correct read, same orientation
        good_bytes = b.read(ACC_ADDR, AUTO_INC | OUT_X_L_A, 6)
        mean, std, xs, raw, _ = capture(b, 200, sub=OUT_X_L_A)       # the fault
        same = all(r[0] == r[1] == r[2] == r[3] == r[4] == r[5] for r in raw)
        xl_vals = sorted(set(r[0] for r in raw))
        preds = [predict_broken(r[0]) for r in raw]
        match = all(abs(xs[i][j] - preds[i]) < 1e-9 for i in range(len(xs)) for j in range(3))
        rec = {"orientation": label_of(good_mean), "correct_ax_g": good_mean[0], "correct_ay_g": good_mean[1],
               "correct_az_g": good_mean[2], "correct_mag_g": mag(good_mean),
               "correct_burst_bytes": ["%02X" % x for x in good_bytes],
               "broken_ax_g": mean[0], "broken_ay_g": mean[1], "broken_az_g": mean[2], "broken_mag_g": mag(mean),
               "broken_std_g": std[0], "broken_example_bytes": ["%02X" % x for x in raw[0]],
               "all_six_bytes_identical_every_read": same, "distinct_xl_values": ["%02X" % x for x in xl_vals],
               "every_axis_equals_prediction_from_xl": match,
               "mag_over_abs_ax": mag(mean) / abs(mean[0]) if mean[0] else None}
        out.append(rec)
        print("  correct (0xA8):  ax %+.4f ay %+.4f az %+.4f |a| %.4f g   bytes %s"
              % (good_mean + (mag(good_mean), " ".join(rec["correct_burst_bytes"]))))
        print("  broken  (0x28):  ax %+.4f ay %+.4f az %+.4f |a| %.4f g   bytes %s"
              % (mean + (mag(mean), " ".join(rec["broken_example_bytes"]))))
        print("    six bytes identical in every read: %s; every axis = (XL<<8|XL)>>4 mg: %s; XL seen: %s"
              % (same, match, " ".join(rec["distinct_xl_values"])))
    save_json("broken_autoinc.json", {"sub_correct": "0xA8", "sub_broken": "0x28", "runs": out})
    print("Put the bit back: the normal read path uses SUB = 0xA8 again (nothing is left changed).")


# ---------------------------------------------------------------------------
# 1d. Sustained rate
# ---------------------------------------------------------------------------
def run_rate(b, seconds=30.0):
    configure_acc(b)
    print("1d: %d Hz ODR, one 7-byte burst (STATUS_REG_A + 6 output bytes) per poll, SCL %.1f kHz, %.0f s"
          % (ODR_HZ, SCL_HZ / 1e3, seconds))
    rows = []
    polls = new = overruns = repeats = 0
    prev = None
    host_t = []
    cyc = []
    t0 = time.perf_counter()
    last = t0
    while True:
        now = time.perf_counter()
        if now - t0 >= seconds:
            break
        st, g, c = read_status_xyz(b)
        t1 = time.perf_counter()
        host_t.append(t1 - now)
        cyc.append(b.last_cycles)
        polls += 1
        if st & 0x08:                                # ZYXDA: a new conversion since the last read
            new += 1
            if st & 0x80:
                overruns += 1                        # ZYXOR: a conversion was overwritten unread
            if prev is not None and c == prev:
                repeats += 1
            prev = c
            rows.append([round(t1 - t0, 6), polls, "%02X" % st, c[0], c[1], c[2]])
        if now - last > 1.0:
            print("\r  t=%5.1f s  polls %6d  new samples %6d (%.1f/s)  overruns %d"
                  % (now - t0, polls, new, new / (now - t0), overruns), end="", flush=True)
            last = now
    el = time.perf_counter() - t0
    print()
    ts = [r[0] for r in rows]
    gaps = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    per_sec = {}
    for t in ts:
        per_sec[int(t)] = per_sec.get(int(t), 0) + 1
    sec_rates = [per_sec.get(i, 0) for i in range(int(seconds))]
    summary = {
        "seconds": el, "polls": polls, "new_samples": new, "sample_rate_hz": new / el, "poll_rate_hz": polls / el,
        "polls_per_new_sample": polls / max(new, 1), "overruns_zyxor": overruns,
        "consecutive_identical_new_samples": repeats,
        "min_samples_in_any_1s": min(sec_rates), "max_samples_in_any_1s": max(sec_rates),
        "max_gap_ms": max(gaps) * 1e3, "mean_gap_ms": statistics.mean(gaps) * 1e3,
        "scl_hz": SCL_HZ, "bytes_per_transaction_on_bus": 3 + 7,
        "data_bytes_per_transaction": 7,
        "i2c_cycles_mean": statistics.mean(cyc), "i2c_time_us_mean": statistics.mean(cyc) / CLK_MHZ,
        "host_time_per_poll_us_mean": statistics.mean(host_t) * 1e6,
        "host_time_per_poll_us_median": statistics.median(host_t) * 1e6,
        "odr_hz": ODR_HZ, "ctrl_reg1_a": "0x%02X" % CTRL1_RUN,
    }
    summary["overhead_us_per_poll"] = summary["host_time_per_poll_us_mean"] - summary["i2c_time_us_mean"]
    print("  %.1f s: %d polls (%.0f/s), %d NEW samples = %.1f samples/s; slowest second %d, overruns %d"
          % (el, polls, polls / el, new, new / el, min(sec_rates), overruns))
    print("  I2C transaction %.0f us (FPGA cycle counter), whole poll %.0f us -> %.0f us outside the bus"
          % (summary["i2c_time_us_mean"], summary["host_time_per_poll_us_mean"], summary["overhead_us_per_poll"]))
    print("  %s: >= 100 samples/s for %.0f s, every sample a new conversion (ZYXDA = 1)"
          % ("PASS" if min(sec_rates) >= 100 else "FAIL", seconds))
    save_csv("rate_30s_samples.csv", ["t_s", "poll", "status", "x_counts", "y_counts", "z_counts"], rows)

    # --- the other approach: six separately addressed one-byte transactions per sample ---
    print("  comparison: six separate 1-byte register reads per sample, 5 s ...")
    t0 = time.perf_counter()
    n6 = 0
    cyc6 = []
    while time.perf_counter() - t0 < 5.0:
        for reg in range(OUT_X_L_A, OUT_X_L_A + 6):
            b.read(ACC_ADDR, reg, 1)
            cyc6.append(b.last_cycles)
        n6 += 1
    el6 = time.perf_counter() - t0
    summary["six_single_reads"] = {"samples": n6, "seconds": el6, "rate_hz": n6 / el6,
                                   "i2c_time_us_per_read": statistics.mean(cyc6) / CLK_MHZ,
                                   "i2c_time_us_per_sample": 6 * statistics.mean(cyc6) / CLK_MHZ}
    print("    %.1f samples/s (each 1-byte read %.0f us on the bus, %.0f us per sample)"
          % (n6 / el6, statistics.mean(cyc6) / CLK_MHZ, 6 * statistics.mean(cyc6) / CLK_MHZ))

    # --- the trap: reading as fast as possible WITHOUT checking ZYXDA ---
    t0 = time.perf_counter()
    raw = []
    while time.perf_counter() - t0 < 2.0:
        _, g, c = read_status_xyz(b)
        raw.append(c)
    distinct = sum(1 for i in range(1, len(raw)) if raw[i] != raw[i - 1]) + 1
    summary["ungated_2s"] = {"reads": len(raw), "reads_per_s": len(raw) / 2.0,
                             "changes_between_consecutive_reads": distinct,
                             "changes_per_s": distinct / 2.0}
    print("  ungated: %d reads in 2 s (%.0f/s) but only %d changes (%.0f/s) - the rest are re-reads"
          % (len(raw), len(raw) / 2.0, distinct, distinct / 2.0))

    # --- ODR sweep: the new-sample rate must follow the ODR setting ---
    sweep = []
    for odr in (50, 100, 200, 400):
        configure_acc(b, (ODR_BITS[odr] << 4) | 0x07)
        new_n = polls_n = ovr = 0
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 3.0:
            st, _, _ = read_status_xyz(b)
            polls_n += 1
            if st & 0x08:
                new_n += 1
                ovr += bool(st & 0x80)
        el2 = time.perf_counter() - t0
        sweep.append({"odr_hz": odr, "new_per_s": new_n / el2, "polls_per_s": polls_n / el2, "overruns": ovr})
        print("  ODR %3d Hz -> %6.1f new samples/s  (%.0f polls/s, %d overruns)" % (odr, new_n / el2, polls_n / el2, ovr))
    configure_acc(b)
    summary["odr_sweep"] = sweep
    save_json("rate_summary.json", summary)


# ---------------------------------------------------------------------------
# Magnetometer (not graded)
# ---------------------------------------------------------------------------
MAG_GAIN_XY, MAG_GAIN_Z = 1100.0, 980.0     # LSB/gauss at GN = 001 (Table 75 p.37)


def run_mag(b, seconds=20.0):
    ida = b.read(MAG_ADDR, IRA_REG_M, 3)
    print("IRA/IRB/IRC = %s (expect 48 34 33)" % " ".join("%02X" % x for x in ida))
    b.write(MAG_ADDR, CRA_REG_M, 0x14)        # DO = 101: 30 Hz
    b.write(MAG_ADDR, CRB_REG_M, 0x20)        # GN = 001: +-1.3 gauss
    b.write(MAG_ADDR, MR_REG_M, 0x00)         # continuous conversion
    cfg = b.read(MAG_ADDR, CRA_REG_M, 3)
    print("CRA/CRB/MR read back: %s (expect 14 20 00)" % " ".join("%02X" % x for x in cfg))
    configure_acc(b)
    print("Rotate the board slowly through all orientations for %.0f s." % seconds)
    rows = []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        r = b.read(MAG_ADDR, OUT_X_H_M, 6)    # X_H X_L Z_H Z_L Y_H Y_L: big-endian, X Z Y order
        mx = s16((r[0] << 8) | r[1]) / MAG_GAIN_XY
        mz = s16((r[2] << 8) | r[3]) / MAG_GAIN_Z
        my = s16((r[4] << 8) | r[5]) / MAG_GAIN_XY
        g, _ = read_xyz(b)
        m = math.sqrt(mx * mx + my * my + mz * mz)
        rows.append([round(time.perf_counter() - t0, 3), mx, my, mz, m] + list(g) + [mag(g)])
        print("\r  B = (%+.3f %+.3f %+.3f) gauss |B| = %.3f   |a| = %.3f g   " % (mx, my, mz, m, mag(g)),
              end="", flush=True)
        time.sleep(0.05)
    print()
    ms = [r[4] for r in rows]
    gs = [r[8] for r in rows]
    summ = {"id": ["%02X" % x for x in ida], "config": ["%02X" % x for x in cfg], "samples": len(rows),
            "B_mag_min_gauss": min(ms), "B_mag_max_gauss": max(ms), "B_mag_mean_gauss": statistics.mean(ms),
            "B_mag_std_gauss": statistics.stdev(ms), "a_mag_min_g": min(gs), "a_mag_max_g": max(gs),
            "x_range": [min(r[1] for r in rows), max(r[1] for r in rows)],
            "y_range": [min(r[2] for r in rows), max(r[2] for r in rows)],
            "z_range": [min(r[3] for r in rows), max(r[3] for r in rows)]}
    print("  |B| %.3f .. %.3f gauss (mean %.3f), while |a| stayed %.3f .. %.3f g"
          % (summ["B_mag_min_gauss"], summ["B_mag_max_gauss"], summ["B_mag_mean_gauss"], min(gs), max(gs)))
    save_csv("mag_rotation.csv", ["t_s", "Bx_G", "By_G", "Bz_G", "B_G", "ax_g", "ay_g", "az_g", "a_g"], rows)
    save_json("mag_summary.json", summ)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["kat", "live", "six", "broken", "rate", "mag", "demo"])
    ap.add_argument("--auto", action="store_true", help="six: capture each new still orientation automatically")
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--no-load", action="store_true", help="keep the bitstream already in the FPGA")
    args = ap.parse_args()
    self_test()
    b = Board(load=not args.no_load)
    try:
        if args.mode == "kat":
            sys.exit(0 if run_kat(b) else 1)
        elif args.mode == "live":
            run_live(b, args.seconds)
        elif args.mode == "six":
            run_six(b, args.auto)
        elif args.mode == "broken":
            run_broken(b)
        elif args.mode == "rate":
            run_rate(b, args.seconds or 30.0)
        elif args.mode == "mag":
            run_mag(b, args.seconds or 20.0)
        elif args.mode == "demo":
            run_kat(b)
            run_live(b, args.seconds)
    finally:
        b.close()


if __name__ == "__main__":
    main()
