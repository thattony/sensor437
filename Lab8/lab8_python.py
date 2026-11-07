# -*- coding: utf-8 -*-
"""
ECE 437 Lab 8 - H-bridge motor controller (Pmod DHB1) + LSM303DLHC arm angle.

FPGA: lab8_motor.v (bit file fpga/bitfile/lab8_motor.bit, build: fpga/build.ps1)
  - 200 Hz pulse train on EN1, exactly N pulses, DIR1 changed only while EN1 low
  - free-running accelerometer sampler (400 Hz ODR, STATUS-gated) -> FIFO -> PipeOut
  - stall trip / watchdog -> EN1 low (coast), fault latched until cleared

Every mode logs what it measured to results/ (CSV = samples, JSON = summary,
PNG = plot). The arm configuration (axes, home, offsets, tilt, EXPECTED) lives
in results/lab8_config.json and is reused by every later mode.

    py -3.14 lab8_python.py selftest            offline: conversions + stall detector on synthetic data
    py -3.14 lab8_python.py check               bit file, accelerometer, sample rate, axis means
    py -3.14 lab8_python.py pulsetest           10 pulses -> check count/timing (scope on PMOD pin 1)
    py -3.14 lab8_python.py interactive         type direction + pulse count, move -> settle -> read
    py -3.14 lab8_python.py home                arm at the centre of travel: record phi_home
    py -3.14 lab8_python.py tilt                arm stopped: beta from the in-plane axes    (TA check 1)
    py -3.14 lab8_python.py calibrate           free run: deg/pulse, direction sign, EXPECTED per 100 ms
    py -3.14 lab8_python.py sweep               stop-and-read around the travel, circle fit -> beta, offsets
    py -3.14 lab8_python.py resolution          100 stationary readings of phi -> std   (TA check 3)
    py -3.14 lab8_python.py move N DIR          one protected move -> settle -> read (log for 3b/4a)
    py -3.14 lab8_python.py goto ANGLE          move the arm to ANGLE degrees (stop-and-read loop)
    py -3.14 lab8_python.py stall DIR           drive into the post with protection     (TA check 8)
    py -3.14 lab8_python.py reset               clear the latched fault (then re-home with goto 0)
    py -3.14 lab8_python.py analyze FILE...     ringing (3b) + torque (4a/4c) from move logs

Common options: --duty PCT (default 50), --label TEXT (goes in the file names).

Bench supply: the E3631A +6V output feeds J4 (VM / GND). interactive, calibrate, sweep,
move, goto and stall set it to --volts (default 6.0, max 6.0) with a 1.5 A limit, switch
the output ON, and switch it OFF when they finish (also on Ctrl-C or an error).
pulsetest only powers the motor with --power. --no-supply leaves the E3631A alone.
"""
import sys
import os
import time
import json
import csv
import math
import struct
import argparse

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")
CONFIG = os.path.join(RESULTS, "lab8_config.json")
BITFILE = os.path.join(HERE, "fpga", "bitfile", "lab8_motor.bit")
DESIGN_ID = 0xEC437008

# ---- FrontPanel endpoints (lab8_motor.v header) ----
EP_CTRL, EP_P1, EP_P2, EP_P3, EP_PULSES, EP_MOTOR, EP_SAMPLER = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
EP_RESULT0, EP_RX_LO, EP_STATUS, EP_RX_HI, EP_XFER = 0x20, 0x21, 0x23, 0x24, 0x25
EP_MSTAT, EP_SENT, EP_TARGET, EP_FIFO, EP_TS, EP_NSAMP, EP_FTS, EP_FPUL, EP_ERRS, EP_ID = \
    0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x3F
EP_TRIG = 0x40
T_I2C, T_RESET, T_GO, T_TRIP, T_CLEAR, T_FIFORST, T_KICK, T_STOP = range(8)
EP_PIPE = 0xA0
TS_TICK = 63 / 100.8e6               # 0.625 us per timestamp tick
PULSE_HZ = 200.0
WATCHDOG_MS = 250

# ---- LSM303DLHC accelerometer (same register facts as the midterm) ----
ACC_ADDR = 0x19
BUS1 = 1 << 8
CTRL_REG1_A, CTRL_REG4_A = 0x20, 0x23
ODR_HZ = 400
CTRL1 = 0x77                         # 400 Hz, normal mode, X Y Z on
CTRL4 = {2: 0x88, 4: 0x98}           # BDU, little-endian, +-2 g / +-4 g, high resolution
MG_PER_LSB = {2: 1.0, 4: 2.0}

# ---- the rig (pre lab) ----
R_IMU = 0.10                         # m, IMU distance from the shaft
J_ARM = 0.3 * 0.28 ** 2 / 12         # kg m^2 = 1.96e-3 (pre lab 4a)
G0 = 9.80665
TILT_NOMINAL = 15.0
PRELAB_RES_DEG = math.degrees(0.001 / math.sin(math.radians(15.0)))   # 0.22 deg

# ---- stall detector (pre lab) ----
WINDOW = 0.100                       # s
MIN_FRAC = 0.25
STRIKES = 2
NAVG = 40                            # samples averaged for phi_now: the whole 100 ms window. The pre lab says 4-8,
                                     # but pinned against the post the arm vibrates and 8-sample means wander 10+ deg
GRACE_WINDOWS = 1                    # first window = spin-up, no strike
FLOOR_DEG = 2.0                      # never trip on less than this per window if EXPECTED unknown

DEFAULT_CFG = {
    "radial": "x", "radial_sign": 1, "tangential": "z", "tangential_sign": 1,
    "fs": 2, "offset_r": 0.0, "offset_t": 0.0, "G": math.sin(math.radians(15.0)),
    "beta_deg": None, "phi_home_deg": None, "expected_deg": None,
    "deg_per_pulse": None, "dir_sign": None,
}


# ===========================================================================
# small helpers
# ===========================================================================
def stamp():
    return time.strftime("%Y%m%d_%H%M%S")


def ensure_results():
    os.makedirs(RESULTS, exist_ok=True)


def save_json(name, obj):
    ensure_results()
    p = os.path.join(RESULTS, name)
    with open(p, "w") as f:
        json.dump(obj, f, indent=2, default=float)
    print("  logged ->", p)
    return p


def save_csv(name, header, rows):
    ensure_results()
    p = os.path.join(RESULTS, name)
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print("  logged ->", p, "(%d rows)" % len(rows))
    return p


def load_cfg():
    cfg = dict(DEFAULT_CFG)
    if os.path.exists(CONFIG):
        with open(CONFIG) as f:
            cfg.update(json.load(f))
    return cfg


def save_cfg(cfg):
    ensure_results()
    with open(CONFIG, "w") as f:
        json.dump(cfg, f, indent=2)
    print("  config ->", CONFIG)


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def counts(raw16):
    """OUT_x register pair (left-justified 12 bit) -> signed count."""
    return s16(raw16 & 0xFFFF) >> 4


def plt_module():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


# ===========================================================================
# sample words <-> values (PipeOut format, lab8_motor.v header)
# ===========================================================================
def parse_words(words, fs=2):
    """4 words per sample -> list of dicts. Raises on a lost alignment marker."""
    out = []
    k = MG_PER_LSB[fs] / 1000.0
    for i in range(0, len(words) - 3, 4):
        w0, w1, w2, w3 = words[i:i + 4]
        if (w2 >> 28) != 0xA:
            raise ValueError("sample %d: bad marker in word 2 (0x%08X)" % (i // 4, w2))
        out.append({
            "ts": w0,
            "ax": counts(w1) * k, "ay": counts(w1 >> 16) * k, "az": counts(w2) * k,
            "status": (w2 >> 16) & 0xFF,
            "en1": (w2 >> 24) & 1, "dir1": (w2 >> 25) & 1,
            "running": (w2 >> 26) & 1, "fault": (w2 >> 27) & 1,
            "pulses": w3 & 0xFFFFFF, "seq": w3 >> 24,
        })
    return out


def make_words(ts, ax_mg, ay_mg, az_mg, status=0x0F, en1=0, dir1=0, running=0, fault=0, pulses=0, seq=0):
    """Inverse of parse_words for one sample (used by selftest)."""
    def raw(mg):
        return (int(mg) << 4) & 0xFFFF
    w1 = (raw(ay_mg) << 16) | raw(ax_mg)
    w2 = (0xA << 28) | (fault << 27) | (running << 26) | (dir1 << 25) | (en1 << 24) | (status << 16) | raw(az_mg)
    w3 = ((seq & 0xFF) << 24) | (pulses & 0xFFFFFF)
    return [ts & 0xFFFFFFFF, w1, w2, w3]


# ===========================================================================
# arm geometry: axes -> (a_radial, a_tangential) -> phi
# ===========================================================================
class Arm:
    def __init__(self, cfg):
        self.cfg = cfg

    def rt(self, s, corrected=True):
        c = self.cfg
        ar = s["a" + c["radial"]] * c["radial_sign"]
        at = s["a" + c["tangential"]] * c["tangential_sign"]
        if corrected:
            ar -= c["offset_r"]
            at -= c["offset_t"]
        return ar, at

    def axis_comp(self, s):
        c = self.cfg
        other = ({"x", "y", "z"} - {c["radial"], c["tangential"]}).pop()
        return s["a" + other]

    @staticmethod
    def phi_of(ar, at):
        return math.degrees(math.atan2(-at, ar))          # phi = atan2(-a_t, a_r)

    def phi(self, s):
        return self.phi_of(*self.rt(s))

    def phi_rel(self, s):
        h = self.cfg.get("phi_home_deg")
        p = self.phi(s)
        return wrap180(p - h) if h is not None else p


# ===========================================================================
# stall detector (pre lab pseudocode, with one change explained in the docstring)
# ===========================================================================
class StallDetector:
    """Every WINDOW of FPGA time while EN1 is pulsing:
         v_now  = mean (a_r, a_t) of the last NAVG samples
         moved  = angle the gravity vector turned since the last window
         strike if moved < MIN_FRAC * EXPECTED (or < FLOOR_DEG if not calibrated)
         trip after STRIKES strikes in a row.

    'moved' is measured from the chord, 2 asin(|v_now - v_prev| / 2G), not from
    atan2. While the arm turns, w^2 r adds to a_r (here ~0.6 g at 45 deg / 100 ms,
    more than G = g sin(beta) = 0.26 g), so atan2 is wrong while moving - it can
    even run backwards and fake a stall. The centripetal term is the same in
    both readings of a window pair at constant speed, so it cancels in the
    difference; at a stall w = 0 and both methods agree. The atan2 change is
    still computed and logged for every window.
    """

    def __init__(self, arm, expected_deg=None, learn=False):
        self.arm = arm
        self.G = max(arm.cfg["G"], 0.05)
        self.expected = expected_deg
        self.learn = learn
        thr = FLOOR_DEG if (expected_deg is None or learn) else max(MIN_FRAC * expected_deg, FLOOR_DEG)
        self.threshold = thr
        self.buf = []
        self.v_prev = None
        self.phi_prev = None
        self.t_start = None
        self.next_end = None
        self.k = 0
        self.strikes = 0
        self.tripped = False
        self.trip_info = None
        self.windows = []

    def prime(self, samples):
        """Reference vector from samples taken at rest before the move."""
        rts = [self.arm.rt(s) for s in samples[-NAVG:]]
        if rts:
            self.v_prev = np.mean(np.array(rts), axis=0)
            self.phi_prev = Arm.phi_of(*self.v_prev)

    def feed(self, s, t):
        """s = parsed sample, t = its time in s. Returns True on the sample that trips."""
        if self.tripped or not s["running"]:
            return False
        if self.t_start is None:
            self.t_start = t
            self.next_end = t + WINDOW
        tripped_now = False
        while t >= self.next_end and not self.tripped:
            tripped_now |= self._evaluate(self.next_end, s)
            self.next_end += WINDOW
        self.buf.append(self.arm.rt(s))
        self.buf = self.buf[-NAVG:]
        return tripped_now

    def _evaluate(self, t_end, s):
        self.k += 1
        if not self.buf:
            return False
        v_now = np.mean(np.array(self.buf), axis=0)
        phi_now = Arm.phi_of(*v_now)
        if self.v_prev is None:
            self.v_prev, self.phi_prev = v_now, phi_now
            return False
        chord = float(np.hypot(*(v_now - self.v_prev)))
        moved = math.degrees(2 * math.asin(min(1.0, chord / (2 * self.G))))
        moved_atan = wrap180(phi_now - self.phi_prev)
        if self.k <= GRACE_WINDOWS:
            verdict = "grace"
        elif moved < self.threshold:
            self.strikes += 1
            verdict = "STRIKE %d" % self.strikes
        else:
            self.strikes = 0
            verdict = "ok"
        home = self.arm.cfg.get("phi_home_deg")
        row = {"window": self.k, "t_end": t_end - self.t_start,
               "phi_now": wrap180(phi_now - home) if home is not None else phi_now,     # relative to home, as in 'read'
               "moved_deg": moved, "moved_atan_deg": moved_atan, "threshold_deg": self.threshold,
               "strikes": self.strikes, "verdict": verdict, "pulses": s["pulses"]}
        self.windows.append(row)
        self.v_prev, self.phi_prev = v_now, phi_now
        if self.strikes >= STRIKES and not self.learn:
            self.tripped = True
            self.trip_info = dict(row)
            row["verdict"] = "TRIP"
            return True
        return False


# ===========================================================================
# FPGA access
# ===========================================================================
def import_ok():
    ok_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64'
    ok_dll_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64'
    sys.path.append(ok_loc)
    if hasattr(os, "add_dll_directory") and os.path.isdir(ok_dll_loc):
        os.add_dll_directory(ok_dll_loc)
    import ok
    return ok


class Board:
    def __init__(self, cfg, force_load=False):
        ok = import_ok()
        self.cfg = cfg
        self.dev = ok.okCFrontPanel()
        if self.dev.OpenBySerial("") != 0:
            sys.exit("No Opal Kelly board found. Is FrontPanel (or another script) still open?")
        loaded = False
        if force_load or self._design_id() != DESIGN_ID:
            if not os.path.exists(BITFILE):
                sys.exit("No bit file at %s - build it: fpga\\build.ps1" % BITFILE)
            if self.dev.ConfigureFPGA(BITFILE) != 0:
                sys.exit("Bit file did not load: " + BITFILE)
            loaded = True
        if self._design_id() != DESIGN_ID:
            sys.exit("Design ID 0x%08X, expected 0x%08X (wrong bit file?)" % (self._design_id(), DESIGN_ID))
        print("FPGA: lab8_motor.bit %s (ID 0x%08X)" % ("loaded" if loaded else "already running", DESIGN_ID))
        self.wd_ms = WATCHDOG_MS
        self.st = {}
        self.fs = cfg.get("fs", 2)
        self.ts_wraps = 0
        self.ts_last = None
        self.last_seq = None
        self.drops = 0
        if loaded:
            self.trigger(T_RESET)
        self.configure_accel()

    # ---- raw endpoint access ----
    def _design_id(self):
        self.dev.UpdateWireOuts()
        return self.dev.GetWireOutValue(EP_ID) & 0xFFFFFFFF

    def trigger(self, bit):
        self.dev.ActivateTriggerIn(EP_TRIG, bit)

    def wire(self, ep, value):
        self.dev.SetWireInValue(ep, value & 0xFFFFFFFF)
        self.dev.UpdateWireIns()

    def update(self):
        d = self.dev
        d.UpdateWireOuts()
        m = d.GetWireOutValue(EP_MSTAT)
        self.st = {
            "running": m & 1, "en1": (m >> 1) & 1, "dir1": (m >> 2) & 1, "fault": (m >> 3) & 1,
            "fault_code": (m >> 4) & 0xF, "sampler": (m >> 8) & 1, "fifo_ovf": (m >> 9) & 1,
            "go_rejected": (m >> 10) & 1, "wd_en": (m >> 11) & 1, "duty": (m >> 16) & 0xFF,
            "sent": d.GetWireOutValue(EP_SENT), "target": d.GetWireOutValue(EP_TARGET),
            "fifo_words": d.GetWireOutValue(EP_FIFO), "ts": d.GetWireOutValue(EP_TS),
            "nsamples": d.GetWireOutValue(EP_NSAMP), "fault_ts": d.GetWireOutValue(EP_FTS),
            "fault_pulses": d.GetWireOutValue(EP_FPUL), "nack": d.GetWireOutValue(EP_ERRS) & 0xFFFF,
            "zyxor": d.GetWireOutValue(EP_ERRS) >> 16,
        }
        return self.st

    # ---- host I2C (only while the sampler is off) ----
    def _transact(self, p1, p2, p3, timeout=0.1):
        d = self.dev
        d.SetWireInValue(EP_P1, p1)
        d.SetWireInValue(EP_P2, p2)
        d.SetWireInValue(EP_P3, p3)
        d.UpdateWireIns()
        d.UpdateWireOuts()
        before = d.GetWireOutValue(EP_XFER)
        self.trigger(T_I2C)
        t0 = time.perf_counter()
        while True:
            d.UpdateWireOuts()
            if (d.GetWireOutValue(EP_XFER) - before) & 0xFFFFFFFF:
                break
            if time.perf_counter() - t0 > timeout:
                raise RuntimeError("I2C transaction did not complete")
        rx = ((d.GetWireOutValue(EP_RX_LO) & 0xFFFFFFFF) | ((d.GetWireOutValue(EP_RX_HI) & 0xFFFFFFFF) << 32))
        nack = bool(d.GetWireOutValue(EP_RESULT0) & 1)
        return rx.to_bytes(8, "little"), nack

    def i2c_read(self, sub, n=1):
        rx, nack = self._transact(BUS1 | (ACC_ADDR << 1), sub & 0xFF, (n << 4) | 0x1)
        if nack:
            raise RuntimeError("accelerometer NACK reading 0x%02X" % sub)
        return rx[:n]

    def i2c_write(self, sub, val):
        _, nack = self._transact(BUS1 | (ACC_ADDR << 1), (sub & 0xFF) | ((val & 0xFF) << 8), 0x3)
        if nack:
            raise RuntimeError("accelerometer NACK writing 0x%02X" % sub)

    def sampler(self, on, watchdog=True):
        v = (self.wd_ms << 16) | ((1 if watchdog else 0) << 1) | (1 if on else 0)
        self.wire(EP_SAMPLER, v)
        if not on:
            t0 = time.perf_counter()
            while self.update()["sampler"] and time.perf_counter() - t0 < 0.1:
                pass

    def configure_accel(self):
        self.sampler(False)
        c1 = self.i2c_read(CTRL_REG1_A)[0]
        self.i2c_write(CTRL_REG1_A, CTRL1)
        self.i2c_write(CTRL_REG4_A, CTRL4[self.fs])
        r1, r4 = self.i2c_read(CTRL_REG1_A)[0], self.i2c_read(CTRL_REG4_A)[0]
        if (r1, r4) != (CTRL1, CTRL4[self.fs]):
            sys.exit("LSM303 config read back 0x%02X/0x%02X, expected 0x%02X/0x%02X" % (r1, r4, CTRL1, CTRL4[self.fs]))
        self.ctrl1_before = c1
        print("LSM303: CTRL_REG1_A 0x%02X -> 0x%02X (%d Hz), CTRL_REG4_A = 0x%02X (BDU, +-%d g, HR)" %
              (c1, r1, ODR_HZ, r4, self.fs))
        self.sampler(True)
        self.flush()

    # ---- streaming ----
    def flush(self):
        self.trigger(T_FIFORST)
        self.last_seq = None

    def _t(self, ts):
        if self.ts_last is not None and ts < self.ts_last:
            self.ts_wraps += 1
        self.ts_last = ts
        return (ts + self.ts_wraps * (1 << 32)) * TS_TICK

    def poll(self, kick=True):
        """Read every whole sample waiting in the FIFO. Returns list of samples (with 't')."""
        if kick:
            self.trigger(T_KICK)
        st = self.update()
        n = min(st["fifo_words"] // 4, 1024)
        if n == 0:
            return []
        buf = bytearray(16 * n)
        got = self.dev.ReadFromPipeOut(EP_PIPE, buf)
        if got != len(buf):
            raise RuntimeError("PipeOut returned %d of %d bytes" % (got, len(buf)))
        words = struct.unpack("<%dI" % (4 * n), bytes(buf))
        samples = parse_words(words, self.fs)
        for s in samples:
            s["t"] = self._t(s["ts"])
            if self.last_seq is not None and s["seq"] != ((self.last_seq + 1) & 0xFF):
                self.drops += 1
            self.last_seq = s["seq"]
        return samples

    def collect(self, seconds, kick=True):
        out = []
        t_end = time.perf_counter() + seconds
        while time.perf_counter() < t_end:
            out += self.poll(kick)
            time.sleep(0.004)
        return out

    def collect_n(self, n, timeout=10.0):
        out = []
        t_end = time.perf_counter() + timeout
        while len(out) < n and time.perf_counter() < t_end:
            out += self.poll()
            time.sleep(0.004)
        return out

    # ---- motor ----
    def start_move(self, pulses, direction, duty):
        st = self.update()
        if st["fault"]:
            raise RuntimeError("fault latched (code %d) - run 'reset' and re-home first" % st["fault_code"])
        if st["running"]:
            raise RuntimeError("motor already running")
        self.dev.SetWireInValue(EP_PULSES, int(pulses))
        self.dev.SetWireInValue(EP_MOTOR, ((int(duty) & 0xFF) << 8) | (int(direction) & 1))
        self.dev.UpdateWireIns()
        self.trigger(T_GO)

    def close(self):
        try:
            self.trigger(T_STOP)
            self.sampler(False, watchdog=False)
        finally:
            self.dev.Close()


# ===========================================================================
# the measurement cycle: command the move -> stop -> wait to settle -> read
# ===========================================================================
SETTLE_BLOCK = 20          # samples (50 ms at 400 Hz)
SETTLE_TOL_DEG = 0.3       # 3 consecutive block means within this span = settled
SETTLE_MAX = 4.0           # s
READ_N = 100               # samples averaged for the stopped reading (0.25 s)


def sample_rows(samples, arm, t0):
    rows = []
    for s in samples:
        ar, at = arm.rt(s)
        rows.append(["%.6f" % (s["t"] - t0), "%.4f" % s["ax"], "%.4f" % s["ay"], "%.4f" % s["az"],
                     "%.4f" % ar, "%.4f" % at, "%.3f" % arm.phi_rel(s),
                     s["en1"], s["dir1"], s["running"], s["fault"], s["pulses"], "0x%02X" % s["status"]])
    return rows


SAMPLE_HEADER = ["t_s", "ax_g", "ay_g", "az_g", "a_radial_g", "a_tangential_g", "phi_rel_deg",
                 "en1", "dir1", "running", "fault", "pulses", "status"]


def stopped_read(b, arm, n=READ_N):
    s = b.collect_n(n)
    phis = np.array([arm.phi_rel(x) for x in s])
    rt = np.array([arm.rt(x) for x in s])
    return {"phi": float(np.mean(phis)), "phi_std": float(np.std(phis, ddof=1)) if len(phis) > 1 else 0.0,
            "ar": float(rt[:, 0].mean()), "at": float(rt[:, 1].mean()), "n": len(s), "samples": s}


def do_move(b, cfg, pulses, direction, duty=50, protect=True, learn=False, label="move", save=True, quiet=False):
    arm = Arm(cfg)
    pre = b.collect(0.3)
    if not pre:
        raise RuntimeError("no samples from the accelerometer")
    phi_before = float(np.mean([arm.phi_rel(s) for s in pre[-READ_N:]]))
    det = StallDetector(arm, cfg.get("expected_deg"), learn=learn)
    det.prime(pre)
    t0 = pre[-1]["t"]
    log = list(pre)
    b.start_move(pulses, direction, duty)
    t_host_go = time.perf_counter()
    if not quiet:
        print("  GO: %d pulses (%.2f s), DIR1 = %d, duty %d %%, protection %s" %
              (pulses, pulses / PULSE_HZ, direction, duty, "ON" if protect else "off"))
    trip_host = None
    t_limit = t_host_go + pulses / PULSE_HZ + 2.0
    # ---- motor running ----
    while True:
        new = b.poll()
        for s in new:
            log.append(s)
            if protect and det.feed(s, s["t"]) and trip_host is None:
                b.trigger(T_TRIP)                       # FPGA: EN1 low, fault latched
                trip_host = time.perf_counter()
        st = b.st
        if trip_host is not None or (not st["running"] and (st["sent"] >= st["target"] or st["fault"])):
            break
        if time.perf_counter() > t_limit:
            b.trigger(T_STOP)
            print("  host timeout: STOP sent")
            break
        time.sleep(0.003)
    st = b.update()
    running_t = [s["t"] for s in log if s["running"]]
    t_stop = (st["fault_ts"] * TS_TICK + b.ts_wraps * (1 << 32) * TS_TICK) if trip_host else \
        (running_t[-1] if running_t else log[-1]["t"])
    # ---- settle ----
    post_phi, settled_at = [], None
    t_settle_limit = time.perf_counter() + SETTLE_MAX
    while time.perf_counter() < t_settle_limit:
        new = b.poll()
        for s in new:
            log.append(s)
            if s["t"] > t_stop and not s["running"]:
                post_phi.append((s["t"], arm.phi_rel(s)))
        nb = len(post_phi) // SETTLE_BLOCK
        if nb >= 3:
            means = [np.mean([p for _, p in post_phi[i * SETTLE_BLOCK:(i + 1) * SETTLE_BLOCK]]) for i in range(nb - 3, nb)]
            if max(means) - min(means) < SETTLE_TOL_DEG:
                settled_at = post_phi[(nb - 3) * SETTLE_BLOCK][0]
                break
        time.sleep(0.003)
    rd = stopped_read(b, arm)
    log += rd["samples"]
    res = {
        "label": label, "pulses": pulses, "dir": direction, "duty": duty, "protect": protect,
        "phi_before": phi_before, "phi_after": rd["phi"], "phi_after_std": rd["phi_std"],
        "moved_deg": wrap180(rd["phi"] - phi_before),
        "sent": st["sent"], "target": st["target"], "tripped": trip_host is not None,
        "fault": st["fault"], "fault_code": st["fault_code"], "fault_pulses": st["fault_pulses"],
        "t_stop_rel": t_stop - t0, "t_stop_abs": t_stop, "settle_s": (settled_at - t_stop) if settled_at else None,
        "settled": settled_at is not None, "windows": det.windows, "trip_info": det.trip_info,
        "en1_after": st["en1"], "running_after": st["running"], "drops": b.drops,
        "supply": dict(SUPPLY_INFO),
    }
    if trip_host is not None:
        res["trip_latency_ms"] = (st["fault_ts"] * TS_TICK + b.ts_wraps * (1 << 32) * TS_TICK -
                                  (det.t_start + det.trip_info["t_end"])) * 1e3
    if not quiet:
        print("  %s: sent %d/%d pulses, phi %.2f -> %.2f deg (moved %+.2f), settle %s" %
              ("TRIPPED" if res["tripped"] else "done", st["sent"], st["target"], phi_before, rd["phi"],
               res["moved_deg"], ("%.0f ms" % (res["settle_s"] * 1e3)) if res["settled"] else "NOT settled in %.0f s" % SETTLE_MAX))
    if save:
        tag = "%s_%s" % (label, stamp())
        res["csv"] = save_csv(tag + ".csv", SAMPLE_HEADER, sample_rows(log, arm, t0))
        if det.windows:
            save_csv(tag + "_windows.csv", list(det.windows[0].keys()),
                     [[("%.4f" % v) if isinstance(v, float) else v for v in w.values()] for w in det.windows])
        save_json(tag + ".json", {k: v for k, v in res.items() if k != "windows"})
    res["log"] = log
    return res


GOTO_STEP_DEG = 45.0      # at most this far per goto move, so a downhill run cannot reach the post


def goto(b, cfg, target, duty=50, tol=3.0, max_iter=10, label="goto"):
    if cfg.get("deg_per_pulse") is None or cfg.get("dir_sign") is None:
        sys.exit("run 'calibrate' first (needs deg/pulse and direction sign)")
    if abs(target) > 135:
        sys.exit("target %.0f deg is too close to the post (keep within +-135)" % target)
    arm = Arm(cfg)
    for _ in range(max_iter):
        now = stopped_read(b, arm)["phi"]
        err = wrap180(target - now)
        if abs(err) < tol:
            print("  at %.1f deg (target %.1f)" % (now, target))
            return now
        n = max(1, int(round(min(abs(err), GOTO_STEP_DEG) / cfg["deg_per_pulse"] * 0.9)))   # small steps: downhill it runs on
        d = 1 if (err > 0) == (cfg["dir_sign"] > 0) else 0
        r = do_move(b, cfg, n, d, duty, label=label, save=False, quiet=True)
        print("  goto %.0f: at %.1f, %d pulses DIR %d -> %.1f" % (target, now, n, d, r["phi_after"]))
        if r["tripped"]:
            sys.exit("stall detector tripped during goto - check the arm, then 'reset'")
    return stopped_read(b, arm)["phi"]


# ===========================================================================
# bench supply (E3631A, +6V output -> J4 VM / GND on the DHB1)
# ===========================================================================
PS_CHANNEL = "P6V"
V_MAX = 6.0                          # motor rating
V_DEFAULT = 6.0                      # the motor rating; 4-5 V cannot reliably start the tilted arm uphill
I_LIMIT_A = 1.5                      # DRV8833: 1.5 A RMS per bridge (the motor draws 1.8 A stalled)
MOTOR_MODES = ("interactive", "calibrate", "sweep", "move", "goto", "stall")
SUPPLY_INFO = {}                     # last set/measured values, copied into every move log
# pyvisa hands every caller the same ResourceManager: closing it closes the supply AND the scope
# sessions, so Supply and Scope only ever close their own instrument.


class Supply:
    """E3631A found by *IDN?, as in Lab 7. Output ON only for the command that drives the motor."""

    def __init__(self, cfg):
        import pyvisa
        self.rm = pyvisa.ResourceManager()
        self.ps = None
        known = cfg.get("supply_resource")
        resources = list(self.rm.list_resources())
        if known in resources:                       # the port that answered last time goes first
            resources.remove(known)
            resources.insert(0, known)
        for res in resources:
            inst = None
            try:
                inst = self.rm.open_resource(res)
                inst.timeout = 1500
                if "E3631A" in inst.query("*IDN?"):
                    self.ps, inst = inst, None       # keep this one open
                    self.ps.timeout = 5000
                    if res != known:
                        cfg["supply_resource"] = res
                        save_cfg(cfg)
                    break
            except Exception:
                pass
            finally:
                if inst is not None:
                    inst.close()
        if self.ps is None:
            raise RuntimeError("E3631A not found. Is it on and connected? "
                               "(or set it by hand and add --no-supply)")
        self.serial = res.startswith("ASRL")
        if self.serial:
            self.ps.write("SYSTem:REMote")           # RS-232 needs remote mode before anything else

    def errors(self):
        errs = []
        for _ in range(20):
            e = self.ps.query("SYSTem:ERRor?").strip()
            if e.startswith("+0") or e.startswith("0,") or "No error" in e:
                break
            errs.append(e)
        return errs

    def write(self, cmd):
        self.ps.write(cmd)
        errs = self.errors()
        if errs:
            raise RuntimeError("E3631A error after '%s': %s" % (cmd, "; ".join(errs)))

    def measure(self):
        return (float(self.ps.query("MEASure:VOLTage? %s" % PS_CHANNEL)),
                float(self.ps.query("MEASure:CURRent? %s" % PS_CHANNEL)))

    def on(self, volts):
        if not 0 < volts <= V_MAX:
            raise RuntimeError("--volts %.2f refused: VMOTOR must be above 0 and at most %.1f V" % (volts, V_MAX))
        self.errors()
        self.write("*CLS")
        self.write("OUTPut:STATe OFF")
        # OUTPut ON switches all three outputs on the E3631A, so zero the unused ones
        for ch in ("P25V", "N25V"):
            self.write("INSTrument:SELect %s" % ch)
            self.write("VOLTage 0")
        self.write("INSTrument:SELect %s" % PS_CHANNEL)
        self.write("VOLTage %.3f" % volts)
        self.write("CURRent %.3f" % I_LIMIT_A)
        v_set, i_set = float(self.ps.query("VOLTage?")), float(self.ps.query("CURRent?"))
        if abs(v_set - volts) > 0.01 or abs(i_set - I_LIMIT_A) > 0.01:
            raise RuntimeError("E3631A read-back %.3f V / %.3f A, wanted %.3f V / %.3f A" % (v_set, i_set, volts, I_LIMIT_A))
        self.write("OUTPut:STATe ON")
        time.sleep(0.5)
        v, i = self.measure()
        SUPPLY_INFO.update({"v_set": v_set, "i_limit": i_set, "v_meas_on": v, "i_meas_on": i})
        print("SUPPLY: E3631A %s ON, set %.3f V / limit %.3f A, measured %.3f V %.3f A" % (PS_CHANNEL, v_set, i_set, v, i))
        if v < 0.9 * volts:
            raise RuntimeError("VMOTOR measured %.2f V with the motor stopped: short or wrong wiring on J4?" % v)

    def off(self):
        """Output OFF, no questions asked. Never raises."""
        try:
            self.ps.write("OUTPut:STATe OFF")
            state = self.ps.query("OUTPut:STATe?").strip()
            print("SUPPLY: output %s" % ("OFF" if state in ("0", "OFF") else
                                        "reads '%s' - SWITCH IT OFF BY HAND" % state))
        except Exception as e:
            print("WARNING: could not confirm the supply is OFF (%s) - switch it off by hand!" % e)

    def close(self):
        try:
            if self.serial:
                self.ps.write("SYSTem:LOCal")        # give the front panel back
            self.ps.close()
        except Exception:
            pass


# ===========================================================================
# oscilloscope (Keysight InfiniiVision over USB): CH1 = EN1 (PMOD pin 1), CH2 = DIR1 (pin 2)
# ===========================================================================
class Scope:
    def __init__(self):
        import pyvisa
        self.rm = pyvisa.ResourceManager()
        self.sc = None
        for res in self.rm.list_resources("USB?*"):
            try:
                inst = self.rm.open_resource(res)
                inst.timeout = 5000
                idn = inst.query("*IDN?").strip()
                if "-X" in idn.split(",")[1]:            # DSO-X / MSO-X
                    self.sc, self.idn = inst, idn
                    break
                inst.close()
            except Exception:
                pass
        if self.sc is None:
            raise RuntimeError("oscilloscope not found on USB (replug its rear USB-B cable)")
        self.sc.timeout = 10000
        print("SCOPE: %s" % self.idn.split(",")[1])

    def w(self, cmd):
        self.sc.write(cmd)

    def arm(self, t_div=0.010):
        """CH1/CH2 at 1 V/div, single shot on the first EN1 rising edge, one division of pre-trigger."""
        self.w("*CLS")
        self.w(":STOP")
        for ch in (1, 2):
            self.w(":CHANnel%d:DISPlay ON; :CHANnel%d:SCALe 1; :CHANnel%d:OFFSet 2.5" % (ch, ch, ch))
        self.w(":TIMebase:SCALe %g; :TIMebase:REFerence LEFT; :TIMebase:POSition %g" % (t_div, -t_div))
        self.w(":TRIGger:MODE EDGE; :TRIGger:EDGE:SOURce CHANnel1; :TRIGger:EDGE:SLOPe POSitive; "
               ":TRIGger:EDGE:LEVel 1.65; :TRIGger:SWEep NORMal")
        self.w(":ACQuire:TYPE NORMal")
        self.w(":SINGle")
        t_end = time.time() + 3
        while time.time() < t_end and int(self.sc.query(":AER?").strip() or 0) != 1:
            time.sleep(0.05)

    def triggered(self, timeout=3.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if int(self.sc.query(":TER?").strip() or 0) == 1:
                time.sleep(0.2)                         # let the single acquisition finish
                return True
            time.sleep(0.05)
        return False

    def wave(self, ch):
        self.w(":WAVeform:SOURce CHANnel%d; :WAVeform:FORMat BYTE; :WAVeform:UNSigned 1; "
               ":WAVeform:POINts:MODE NORMal; :WAVeform:POINts 2000" % ch)
        pre = [float(x) for x in self.sc.query(":WAVeform:PREamble?").split(",")]
        xinc, xorg, xref, yinc, yorg, yref = pre[4:10]
        raw = np.array(self.sc.query_binary_values(":WAVeform:DATA?", datatype="B", container=list), dtype=float)
        t = (np.arange(len(raw)) - xref) * xinc + xorg
        return t, (raw - yref) * yinc + yorg

    def close(self):
        try:
            self.sc.close()
        except Exception:
            pass


def edges(t, v, thr=1.65):
    hi = v > thr
    d = np.diff(hi.astype(int))
    return t[1:][d == 1], t[1:][d == -1]


def scope_report(sc, n, direction, duty):
    t, en1 = sc.wave(1)
    _, dir1 = sc.wave(2)
    rise, fall = edges(t, en1)
    out = {"scope": sc.idn, "pulses_seen": len(rise), "requested": n}
    if len(rise) >= 2:
        out["period_ms"] = float(np.mean(np.diff(rise)) * 1e3)
        out["freq_hz"] = 1e3 / out["period_ms"]
    if len(rise) and len(fall):
        f = fall[fall > rise[0]][:len(rise)]
        out["high_ms"] = float(np.mean(f - rise[:len(f)]) * 1e3)
        if "period_ms" in out:
            out["duty_pct"] = 100 * out["high_ms"] / out["period_ms"]
    out["en1_low_after"] = bool(en1[-1] < 1.65)
    dir_hi = dir1 > 1.65
    out["dir1_level"] = int(np.median(dir_hi[t >= 0]))
    # DIR1 must not change while EN1 is high
    out["dir1_changes_while_en1_high"] = int(np.sum((np.diff(dir_hi.astype(int)) != 0) & (en1[1:] > 1.65)))
    dr, df = edges(t, dir1)
    dir_edges = np.concatenate([dr, df])
    if len(dir_edges) and len(rise):
        out["dir1_edge_before_first_pulse_ms"] = float((rise[0] - dir_edges.max()) * 1e3)
    print("  SCOPE CH1 EN1: %d pulses seen (requested %d), period %.3f ms (%.1f Hz), high %.3f ms (%.1f %%), low after: %s"
          % (out["pulses_seen"], n, out.get("period_ms", float("nan")), out.get("freq_hz", float("nan")),
             out.get("high_ms", float("nan")), out.get("duty_pct", float("nan")), out["en1_low_after"]))
    print("  SCOPE CH2 DIR1: level %d (requested %d), changes while EN1 high: %d%s"
          % (out["dir1_level"], direction, out["dir1_changes_while_en1_high"],
             (", DIR1 edge %.2f ms before the first pulse" % out["dir1_edge_before_first_pulse_ms"])
             if "dir1_edge_before_first_pulse_ms" in out else ""))
    ok = (out["pulses_seen"] == n and abs(out.get("freq_hz", 0) - PULSE_HZ) < 2 and
          abs(out.get("duty_pct", 0) - duty) < 2 and out["en1_low_after"] and
          out["dir1_level"] == direction and out["dir1_changes_while_en1_high"] == 0)
    print("  SCOPE: %s" % ("PASS" if ok else "CHECK"))
    tag = "pulsetest_scope_%s" % stamp()
    save_csv(tag + ".csv", ["t_s", "en1_v", "dir1_v"], [["%.6f" % a, "%.3f" % b_, "%.3f" % c] for a, b_, c in zip(t, en1, dir1)])
    save_json(tag + ".json", dict(out, pass_=ok, duty_set=duty, dir_set=direction))
    plt = plt_module()
    fig, ax = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
    ax[0].plot(t * 1e3, en1, lw=1)
    ax[0].set_ylabel("EN1 (V)")
    ax[0].set_title("%d pulses: %.1f Hz, %.1f %% duty" % (out["pulses_seen"], out.get("freq_hz", float("nan")),
                                                          out.get("duty_pct", float("nan"))))
    ax[1].plot(t * 1e3, dir1, lw=1, color="C1")
    ax[1].set_ylabel("DIR1 (V)")
    ax[1].set_xlabel("time from the first EN1 edge (ms)")
    fig.tight_layout()
    p = os.path.join(RESULTS, tag + ".png")
    fig.savefig(p, dpi=120)
    print("  plot ->", p)
    return ok


# ===========================================================================
# modes
# ===========================================================================
def mode_check(b, cfg, args):
    print("\nStreaming 2 s with the motor stopped ...")
    st0 = b.update()
    s = b.collect(2.0)
    st1 = b.update()
    if len(s) < 10:
        sys.exit("FAIL: only %d samples" % len(s))
    t = np.array([x["t"] for x in s])
    rate = (len(s) - 1) / (t[-1] - t[0])
    m = {k: float(np.mean([x[k] for x in s])) for k in ("ax", "ay", "az")}
    sd = {k: float(np.std([x[k] for x in s], ddof=1) * 1e3) for k in ("ax", "ay", "az")}
    mag = math.sqrt(sum(v * v for v in m.values()))
    print("  samples: %d in %.2f s = %.1f /s (ODR %d Hz), sequence gaps %d, NACKs %d, FIFO overflow %d" %
          (len(s), t[-1] - t[0], rate, ODR_HZ, b.drops, st1["nack"], st1["fifo_ovf"]))
    for k in ("ax", "ay", "az"):
        print("  %s = %+.4f g   (noise %.2f mg rms)" % (k, m[k], sd[k]))
    print("  |a| = %.4f g" % mag)
    # axis assignment: rotation axis = biggest |mean| (g cos beta); the other two are in the arm's plane
    axis = max(m, key=lambda k: abs(m[k]))[1]
    inplane = [a for a in "xyz" if a != axis]
    tang = "z" if "z" in inplane else inplane[1]
    rad = [a for a in inplane if a != tang][0]
    print("  rotation axis ~ %s (|a| = %.3f g); in-plane: radial = %s, tangential = %s (through the board face)"
          % (axis, abs(m["a" + axis]), rad, tang))
    if args.radial or args.tangential:
        rad = (args.radial or "+" + rad)
        tang = (args.tangential or "+" + tang)
        cfg["radial"], cfg["radial_sign"] = rad[-1], (-1 if rad[0] == "-" else 1)
        cfg["tangential"], cfg["tangential_sign"] = tang[-1], (-1 if tang[0] == "-" else 1)
        print("  axes overridden: radial %s, tangential %s" % (rad, tang))
    elif cfg.get("phi_home_deg") is None:
        cfg["radial"], cfg["tangential"] = rad, tang
    print("  using radial = %s%s, tangential = %s%s" % ("-" if cfg["radial_sign"] < 0 else "+", cfg["radial"],
                                                         "-" if cfg["tangential_sign"] < 0 else "+", cfg["tangential"]))
    save_cfg(cfg)
    ok_rate = rate > 0.9 * ODR_HZ
    print("  %s: %.0f samples/s streamed while the FPGA owns the bus" % ("PASS" if ok_rate else "CHECK", rate))
    save_json("check_%s.json" % stamp(), {"rate": rate, "means_g": m, "noise_mg": sd, "abs_a": mag,
                                         "drops": b.drops, "nack": st1["nack"], "axis": axis,
                                         "radial": cfg["radial"], "tangential": cfg["tangential"],
                                         "ctrl_reg1_before": b.ctrl1_before})


def mode_pulsetest(b, cfg, args):
    n = args.n if args.n else 10
    print("\n%d pulses, DIR1 = %d, duty %d %%. Scope: PMOD pin 1 = EN1, pin 2 = DIR1, pin 5 = GND." % (n, args.dir, args.duty))
    sc = None
    if args.scope:
        sc = Scope()
        sc.arm(t_div=max(0.001, n / PULSE_HZ / 8))      # the whole burst on screen
    b.flush()
    b.start_move(n, args.dir, args.duty)
    s = b.collect(n / PULSE_HZ + 0.3)
    st = b.update()
    run = [x["t"] for x in s if x["running"]]
    dur = (run[-1] - run[0]) if len(run) > 1 else 0.0
    print("  FPGA counted %d pulses (requested %d); EN1 now %d; samples during the run: %d (%.0f ms)"
          % (st["sent"], st["target"], st["en1"], len(run), dur * 1e3))
    print("  expected run time N / 200 Hz = %.0f ms" % (n / PULSE_HZ * 1e3))
    print("  %s" % ("PASS" if st["sent"] == n and st["en1"] == 0 else "FAIL"))
    if sc is not None:
        try:
            if sc.triggered():
                scope_report(sc, n, args.dir, args.duty)
            else:
                print("  SCOPE: no trigger on CH1 - check the CH1 probe is on PMOD pin 1 and its ground on pin 5")
        finally:
            sc.close()


def mode_home(b, cfg, args):
    arm = Arm(dict(cfg, phi_home_deg=None))
    r = stopped_read(b, arm, 200)
    cfg["phi_home_deg"] = r["phi"]
    save_cfg(cfg)
    print("\nHome recorded: raw phi = %.2f deg (std %.2f). All angles are now relative to this." % (r["phi"], r["phi_std"]))


def tilt_from(samples, cfg):
    arm = Arm(cfg)
    rt = np.array([arm.rt(s, corrected=True) for s in samples])
    ax = np.array([arm.axis_comp(s) for s in samples])
    ar, at, aa = rt[:, 0].mean(), rt[:, 1].mean(), ax.mean()
    inplane = math.hypot(ar, at)
    g_meas = math.sqrt(inplane ** 2 + aa ** 2)
    return {"a_radial": ar, "a_tangential": at, "a_axis": aa, "inplane_g": inplane, "g_meas": g_meas,
            "beta_deg": math.degrees(math.atan2(inplane, abs(aa))),
            "beta_nominal_g_deg": math.degrees(math.asin(min(1.0, inplane / 1.0)))}


def mode_tilt(b, cfg, args):
    print("\nArm stopped: averaging 4 s (1600 samples) ...")
    s = b.collect(4.0)
    r = tilt_from(s, cfg)
    print("  a_radial = %+.4f g, a_tangential = %+.4f g, a_axis = %+.4f g" % (r["a_radial"], r["a_tangential"], r["a_axis"]))
    print("  sqrt(a_r^2 + a_t^2) = g sin(beta) = %.4f g;  |a| = %.4f g" % (r["inplane_g"], r["g_meas"]))
    print("  TILT beta = %.2f deg  (using the measured |a|; with g = 1.000 exactly: %.2f deg)" %
          (r["beta_deg"], r["beta_nominal_g_deg"]))
    if cfg.get("offset_r") or cfg.get("offset_t"):
        print("  (offsets from the sweep circle fit removed: r %.4f g, t %.4f g)" % (cfg["offset_r"], cfg["offset_t"]))
    else:
        print("  (no offset correction yet - 'sweep' fits the circle and removes the zero-g offsets)")
    cfg["beta_deg"] = r["beta_deg"]
    cfg["G"] = r["inplane_g"]
    save_cfg(cfg)
    save_json("tilt_%s.json" % stamp(), dict(r, n=len(s), offsets=[cfg["offset_r"], cfg["offset_t"]]))


def mode_resolution(b, cfg, args):
    arm = Arm(cfg)
    n_read, spacing = 100, 0.05
    print("\nArm stopped. Waiting 1 s, then 100 readings of phi, one every %.0f ms ..." % (spacing * 1e3))
    b.collect(1.0)
    s = b.collect(n_read * spacing + 0.5)
    t = np.array([x["t"] for x in s])
    t = t - t[0]
    phis = np.array([arm.phi_rel(x) for x in s])
    rt = np.array([arm.rt(x) for x in s])
    idx = [int(np.argmin(np.abs(t - k * spacing))) for k in range(n_read)]
    readings = phis[idx]
    sd = float(np.std(readings, ddof=1))
    sd_all = float(np.std(phis, ddof=1))
    sr, stg = float(np.std(rt[:, 0], ddof=1)), float(np.std(rt[:, 1], ddof=1))
    ar, at = rt[:, 0].mean(), rt[:, 1].mean()
    G = math.hypot(ar, at)
    ph = math.atan2(-at, ar)
    pred = math.degrees(math.sqrt((math.cos(ph) * stg) ** 2 + (math.sin(ph) * sr) ** 2) / G)
    print("  100 readings: phi = %.3f deg, STANDARD DEVIATION = %.3f deg" % (readings.mean(), sd))
    print("  all %d samples: std = %.3f deg" % (len(phis), sd_all))
    print("  measured axis noise: radial %.2f mg, tangential %.2f mg rms; g sin(beta) = %.4f g" % (sr * 1e3, stg * 1e3, G))
    print("  predicted: pre lab (1 mg, beta = 15 deg) = %.3f deg; from the measured noise and beta = %.3f deg" %
          (PRELAB_RES_DEG, pred))
    tag = "resolution_%s" % stamp()
    save_csv(tag + ".csv", ["reading", "t_s", "phi_rel_deg"], [[k, "%.4f" % t[i], "%.4f" % phis[i]] for k, i in enumerate(idx)])
    save_json(tag + ".json", {"std_100_deg": sd, "mean_deg": float(readings.mean()), "std_all_deg": sd_all,
                              "n_all": len(phis), "noise_radial_mg": sr * 1e3, "noise_tangential_mg": stg * 1e3,
                              "G_g": G, "predicted_prelab_deg": PRELAB_RES_DEG, "predicted_measured_noise_deg": pred,
                              "spacing_s": spacing, "odr_hz": ODR_HZ, "fs_g": cfg["fs"]})
    plt = plt_module()
    fig, axs = plt.subplots(1, 2, figsize=(11, 4))
    axs[0].plot(range(n_read), readings, ".-")
    axs[0].set_xlabel("reading #  (one every %.0f ms)" % (spacing * 1e3))
    axs[0].set_ylabel("phi (deg)")
    axs[0].set_title("100 stationary readings, std = %.3f deg" % sd)
    axs[1].hist(readings - readings.mean(), bins=20)
    axs[1].set_xlabel("phi - mean (deg)")
    axs[1].set_title("predicted %.3f (pre lab) / %.3f (measured noise)" % (PRELAB_RES_DEG, pred))
    fig.tight_layout()
    p = os.path.join(RESULTS, tag + ".png")
    fig.savefig(p, dpi=120)
    print("  plot ->", p)


def mode_calibrate(b, cfg, args):
    if cfg.get("phi_home_deg") is None:
        sys.exit("record home first: put the arm at the centre of travel and run 'home'")
    arm = Arm(cfg)
    duty = args.duty
    print("\nStep 1: probe move, 20 pulses DIR1 = 1 (protection on, floor %.0f deg/window)" % FLOOR_DEG)
    r = do_move(b, dict(cfg, expected_deg=None), 20, 1, duty, learn=False, label="cal_probe", save=False)
    if r["tripped"]:
        sys.exit("tripped on the probe - is the arm against the post? Move it to the centre.")
    d = r["moved_deg"]
    if abs(d) < 2:
        sys.exit("arm moved only %.2f deg in 20 pulses: check VMOTOR, J4 polarity and the PMOD header" % d)
    dir_sign = 1 if d > 0 else -1
    dpp = abs(d) / 20
    cfg["dir_sign"], cfg["deg_per_pulse"] = dir_sign, dpp
    print("  DIR1 = 1 turns phi %s; %.2f deg/pulse (probe)" % ("UP" if dir_sign > 0 else "DOWN", dpp))
    start = -80.0
    print("Step 2: go to %.0f deg" % start)
    goto(b, cfg, start, duty)
    span = 160.0
    n = max(20, int(span / dpp * 0.8))
    d_up = 1 if dir_sign > 0 else 0
    print("Step 3: free run, %d pulses toward +, learning EXPECTED" % n)
    r = do_move(b, dict(cfg, expected_deg=None), n, d_up, duty, label="calibrate_free_%dpct" % duty)
    if r["tripped"]:
        sys.exit("free run tripped (arm reached the post?). Run 'reset', 'goto 0', then calibrate with less travel.")
    w = [x["moved_deg"] for x in r["windows"]]
    steady = w[1:-1] if len(w) > 3 else w
    if not steady:
        sys.exit("too few windows to learn EXPECTED - the run was too short")
    expected = float(np.median(steady))
    cfg["expected_deg"] = expected
    cfg["deg_per_pulse"] = abs(r["moved_deg"]) / n
    print("  windows (deg moved per 100 ms): " + " ".join("%.1f" % x for x in w))
    print("  EXPECTED = %.1f deg / 100 ms -> trip below %.1f deg (25 %%)" % (expected, MIN_FRAC * expected))
    print("  deg/pulse over the run = %.3f" % cfg["deg_per_pulse"])
    save_cfg(cfg)
    print("Step 4: back home")
    goto(b, cfg, 0.0, duty)


def circle_fit(x, y):
    A = np.column_stack([x, y, np.ones_like(x)])
    bvec = -(x ** 2 + y ** 2)
    D, E, F = np.linalg.lstsq(A, bvec, rcond=None)[0]
    xc, yc = -D / 2, -E / 2
    R = math.sqrt(max(xc ** 2 + yc ** 2 - F, 0.0))
    return xc, yc, R


def mode_sweep(b, cfg, args):
    cfg0 = dict(cfg, offset_r=0.0, offset_t=0.0)
    arm0 = Arm(cfg0)
    targets = list(np.arange(-110, 111, 20.0))
    rows, xs, ys, axs_ = [], [], [], []
    print("\nStop-and-read at %d positions from -110 to +110 deg ..." % len(targets))
    for tg in targets:
        goto(b, cfg, tg, args.duty)
        rd = stopped_read(b, arm0, 200)
        aa = float(np.mean([arm0.axis_comp(s) for s in rd["samples"]]))
        rows.append([tg, "%.3f" % rd["phi"], "%.5f" % rd["ar"], "%.5f" % rd["at"], "%.5f" % aa])
        xs.append(rd["ar"]); ys.append(rd["at"]); axs_.append(aa)
        print("  target %+5.0f: phi %+7.2f  a_r %+.4f  a_t %+.4f" % (tg, rd["phi"], rd["ar"], rd["at"]))
    goto(b, cfg, 0.0, args.duty)
    x, y = np.array(xs), np.array(ys)
    xc, yc, R = circle_fit(x, y)
    aa = float(np.mean(axs_))
    beta = math.degrees(math.atan2(R, abs(aa)))
    print("  circle fit: centre (offset) a_r %.4f g, a_t %.4f g; radius g sin(beta) = %.4f g" % (xc, yc, R))
    print("  TILT beta (offset-free) = %.2f deg" % beta)
    resid = np.hypot(x - xc, y - yc) - R
    print("  fit residual rms %.2f mg" % (np.sqrt(np.mean(resid ** 2)) * 1e3))
    cfg["offset_r"], cfg["offset_t"], cfg["G"], cfg["beta_deg"] = float(xc), float(yc), float(R), beta
    # home angle changes once offsets are removed: re-reference at the current (home) position
    arm1 = Arm(dict(cfg, phi_home_deg=None))
    cfg["phi_home_deg"] = stopped_read(b, arm1, 200)["phi"]
    save_cfg(cfg)
    tag = "sweep_%s" % stamp()
    save_csv(tag + ".csv", ["target_deg", "phi_raw_rel_deg", "a_radial_g", "a_tangential_g", "a_axis_g"], rows)
    save_json(tag + ".json", {"offset_r": xc, "offset_t": yc, "G": R, "beta_deg": beta, "a_axis": aa,
                              "resid_rms_mg": float(np.sqrt(np.mean(resid ** 2)) * 1e3)})
    plt = plt_module()
    fig, ax = plt.subplots(figsize=(5, 5))
    th = np.linspace(0, 2 * np.pi, 300)
    ax.plot(xc + R * np.cos(th), yc + R * np.sin(th), "-", lw=1)
    ax.plot(x, y, "o")
    ax.plot([xc], [yc], "+", ms=12)
    ax.set_aspect("equal")
    ax.set_xlabel("a_radial (g)")
    ax.set_ylabel("a_tangential (g)")
    ax.set_title("beta = %.2f deg, offset (%.3f, %.3f) g" % (beta, xc, yc))
    fig.tight_layout()
    p = os.path.join(RESULTS, tag + ".png")
    fig.savefig(p, dpi=120)
    print("  plot ->", p)


def mode_move(b, cfg, args):
    r = do_move(b, cfg, args.n, args.dir, args.duty, label=args.label or "move")
    if r["windows"]:
        print_windows(r["windows"])
    print("  -> analyze it:  py -3.14 lab8_python.py analyze \"%s\"" % r["csv"])


def print_windows(ws):
    print("  %-4s %8s %9s %10s %12s %8s  %s" % ("win", "t_end(s)", "phi_now", "moved", "moved_atan2", "thresh", "verdict"))
    for w in ws:
        print("  %-4d %8.2f %9.2f %10.2f %12.2f %8.2f  %s" % (w["window"], w["t_end"], w["phi_now"], w["moved_deg"],
                                                             w["moved_atan_deg"], w["threshold_deg"], w["verdict"]))


def mode_stall(b, cfg, args):
    if cfg.get("expected_deg") is None:
        print("WARNING: EXPECTED not calibrated - detector uses the %.0f deg/window floor" % FLOOR_DEG)
    n = args.n if args.n else 1000
    print("\nSTALL TEST: %d pulses (%.1f s max) DIR1 = %d into the post, protection ON" % (n, n / PULSE_HZ, args.dir))
    print("  EXPECTED %s deg/100 ms, trip if moved < %.1f deg for %d windows in a row" %
          (("%.1f" % cfg["expected_deg"]) if cfg.get("expected_deg") else "n/a",
           max(MIN_FRAC * (cfg.get("expected_deg") or 0), FLOOR_DEG), STRIKES))
    r = do_move(b, cfg, n, args.dir, args.duty, label=args.label or "stall")
    print_windows(r["windows"])
    st = b.update()
    after = [s for s in r["log"] if s["t"] > r["t_stop_abs"] + 0.01]
    print("\n================ STALL LOG ================")
    if r["tripped"]:
        ti = r["trip_info"]
        print("  DETECTOR TRIPPED at t = %.3f s after GO (window %d): phi = %.2f deg, moved %.2f deg < %.2f"
              % (ti["t_end"], ti["window"], ti["phi_now"], ti["moved_deg"], ti["threshold_deg"]))
        print("  FPGA fault latched: code %d (1 = stall trip), pulses sent at trip %d of %d, latency %.1f ms after the window"
              % (st["fault_code"], st["fault_pulses"], st["target"], r.get("trip_latency_ms", float("nan"))))
        print("  SAFE STATE: EN1 = %d, running = %d (EN1 low = coasting); fault stays latched: no moves until 'reset' + re-home"
              % (st["en1"], st["running"]))
        print("  samples after the trip with EN1/running high: %d of %d" %
              (sum(1 for s in after if s["running"] or s["en1"]), len(after)))
    else:
        print("  no trip: the move ended after %d pulses. Did the arm reach the post? Use more pulses or the other DIR." % st["sent"])
    print("  full sample log + window table: %s" % r["csv"])


def mode_reset(b, cfg, args):
    st = b.update()
    print("\nFault before: %d (code %d). Clearing." % (st["fault"], st["fault_code"]))
    b.trigger(T_CLEAR)
    st = b.update()
    print("Fault now: %d. Re-home: 'py -3.14 lab8_python.py goto 0' (or jog in 'interactive' and run 'home')." % st["fault"])


def mode_goto(b, cfg, args):
    goto(b, cfg, args.angle, args.duty)


def mode_interactive(b, cfg, args):
    arm = Arm(cfg)
    print("\nEnter a move as:  <direction 0/1 or cw/ccw> <pulses>   e.g.  1 40   (N/200 s of running)")
    print("Other commands: read | home | goto <deg> | reset | q")
    while True:
        try:
            line = input("lab8> ").strip().lower()
        except EOFError:
            break
        if not line:
            continue
        p = line.split()
        try:
            if p[0] in ("q", "quit", "exit"):
                break
            if p[0] == "read":
                r = stopped_read(b, arm)
                print("  phi = %.2f deg (std %.2f over %d samples)" % (r["phi"], r["phi_std"], r["n"]))
            elif p[0] == "home":
                mode_home(b, cfg, args)
                arm = Arm(cfg)
            elif p[0] == "goto":
                goto(b, cfg, float(p[1]), args.duty)
            elif p[0] == "reset":
                mode_reset(b, cfg, args)
            else:
                d = {"0": 0, "1": 1, "cw": 1, "ccw": 0}[p[0]]
                n = int(p[1])
                if not 1 <= n <= 4000:
                    print("  pulses must be 1..4000")
                    continue
                do_move(b, cfg, n, d, args.duty, label="interactive")
        except (KeyError, IndexError, ValueError):
            print("  ? e.g. '1 40', 'read', 'goto -30'")
        except RuntimeError as e:
            print("  refused:", e)


# ===========================================================================
# offline analysis of move logs: ringing (3b) and torque (4a / 4c)
# ===========================================================================
def load_log(path):
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding=None)
    return d


def analyze_one(path, plt, band_deg=None, t_fit=0.040):
    d = load_log(path)
    t, phi, at, ar = d["t_s"], d["phi_rel_deg"], d["a_tangential_g"], d["a_radial_g"]
    run = d["running"].astype(int)
    out = {"file": os.path.basename(path)}
    idx_run = np.where(run == 1)[0]
    if len(idx_run) == 0:
        print("  %s: no running samples" % path)
        return out
    i0, i1 = idx_run[0], idx_run[-1]
    t_go, t_stop = t[i0], t[i1]
    # ---- torque: alpha from the tangential axis during spin-up from rest
    pre = (t < t_go) & (t > t_go - 0.25)
    at0 = at[pre].mean() if pre.any() else at[max(i0 - 1, 0)]
    fit = (t >= t_go) & (t <= t_go + t_fit)
    da = (at[fit] - at0) * G0                                  # m/s^2 = alpha r
    alpha_mean = float(np.mean(da) / R_IMU)
    k = 4                                                      # 10 ms running mean
    sm = np.convolve((at - at0) * G0 / R_IMU, np.ones(k) / k, mode="same")
    sl = (t >= t_go) & (t <= t_go + 0.15)
    alpha_peak = float(sm[sl][np.argmax(np.abs(sm[sl]))]) if sl.any() else float("nan")
    out.update({"alpha_mean_first_%dms" % int(t_fit * 1e3): abs(alpha_mean), "alpha_peak": abs(alpha_peak),
                "torque_mean_Nm": abs(alpha_mean) * J_ARM, "torque_peak_Nm": abs(alpha_peak) * J_ARM,
                "a_t_step_g": float(np.mean(at[fit] - at0)), "J": J_ARM, "r": R_IMU})
    # ---- ringing: phi after EN1 goes low
    after = t > t_stop
    tail = t > (t[-1] - 0.25)
    final = float(np.mean(phi[tail]))
    noise = float(np.std(phi[tail], ddof=1))
    band = band_deg if band_deg else max(0.5, 3 * noise)
    move_sign = np.sign(final - np.mean(phi[pre])) if pre.any() else 1.0
    pa, ta = phi[after], t[after]
    # phi is meaningless while w^2 r is large: measure the overshoot only once phi is
    # back within 20 deg of its final value for good (the arm is then nearly still)
    far = np.where(np.abs(pa - final) > 20.0)[0]
    j0 = (far[-1] + 1) if len(far) else 0
    over = float(np.max((pa[j0:] - final) * move_sign)) if len(pa) > j0 else float("nan")
    at_after = at[after][j0:]
    ring_pp_g = float(np.ptp(at_after[:int(0.3 * ODR_HZ)])) if len(at_after) else float("nan")
    out["ring_pp_at_g"] = ring_pp_g
    out["overshoot_from_t_s"] = float(ta[j0] - t_stop) if len(ta) > j0 else float("nan")
    outside = np.where(np.abs(pa - final) > band)[0]
    t_settle = float(ta[outside[-1]] - t_stop) if len(outside) else 0.0
    # tangential-axis ringing (a_t carries alpha*r: oscillation after the stop)
    at_final = float(np.mean(at[tail]))
    at_out = np.where(np.abs(at[after] - at_final) > 3 * float(np.std(at[tail], ddof=1)) + 0.002)[0]
    t_settle_at = float(ta[at_out[-1]] - t_stop) if len(at_out) else 0.0
    moved = final - (np.mean(phi[pre]) if pre.any() else phi[0])
    out.update({"move_deg": float(moved), "final_deg": final, "overshoot_deg": over,
                "overshoot_pct": 100 * over / abs(moved) if abs(moved) > 1e-6 else float("nan"),
                "band_deg": band, "settle_s": t_settle, "settle_at_s": t_settle_at,
                "run_s": float(t_stop - t_go)})
    tread = READ_N / ODR_HZ
    out["max_rate_per_s"] = 1.0 / (max(t_settle, t_settle_at) + tread)
    out["rate_definition"] = "1 / (settling time + %.2f s reading); add the move time for a moving cycle" % tread
    # ---- plot
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axs[0].plot(t, phi, lw=1)
    axs[0].axhline(final, color="k", lw=0.6)
    axs[0].axhspan(final - band, final + band, color="g", alpha=0.15)
    axs[0].set_ylabel("phi (deg)\n(wrong while moving)")
    axs[1].plot(t, at, lw=1, label="a_tangential")
    axs[1].plot(t, ar, lw=1, label="a_radial")
    axs[1].legend(loc="best")
    axs[1].set_ylabel("g")
    axs[2].plot(t, sm, lw=1)
    axs[2].set_ylabel("alpha = d_at*g/r (rad/s^2)")
    axs[2].set_xlabel("t (s)")
    for a in axs:
        a.axvline(t_go, color="r", lw=0.7)
        a.axvline(t_stop, color="m", lw=0.7)
    axs[0].set_title("%s: overshoot %.2f deg, settle %.0f ms (phi) / %.0f ms (a_t), tau %.3f N m"
                     % (os.path.basename(path), over, t_settle * 1e3, t_settle_at * 1e3, abs(alpha_mean) * J_ARM))
    fig.tight_layout()
    png = os.path.splitext(path)[0] + "_analysis.png"
    fig.savefig(png, dpi=120)
    plt.close(fig)
    out["plot"] = png
    return out


def mode_analyze(args):
    plt = plt_module()
    rows = []
    for f in args.files:
        r = analyze_one(f, plt)
        rows.append(r)
        print("\n%s" % f)
        for k, v in r.items():
            if k != "file":
                print("  %-28s %s" % (k, ("%.4g" % v) if isinstance(v, float) else v))
    if len(rows) > 1:
        print("\nTorque comparison (4c):")
        for r in rows:
            print("  %-45s tau_mean %.4f N m   tau_peak %.4f N m" % (r["file"], r.get("torque_mean_Nm", float("nan")),
                                                                   r.get("torque_peak_Nm", float("nan"))))
    save_json("analysis_%s.json" % stamp(), {"results": rows})


# ===========================================================================
# offline self-test (no hardware)
# ===========================================================================
def mode_selftest(args):
    bad = 0

    def ok(cond, what):
        nonlocal bad
        print(("PASS  " if cond else "FAIL  ") + what)
        bad += 0 if cond else 1

    # conversions (same vectors as the midterm)
    for lo, hi, exp in [(0x00, 0x00, 0), (0x10, 0x00, 1), (0x80, 0x3E, 1000), (0xF0, 0x7F, 2047), (0x00, 0x80, -2048),
                        (0xF0, 0xFF, -1), (0x80, 0xC1, -1000)]:
        ok(counts((hi << 8) | lo) == exp, "L=0x%02X H=0x%02X -> %d mg" % (lo, hi, exp))
    w = make_words(123456, 12, -34, 1000, en1=1, dir1=1, running=1, pulses=77, seq=5)
    s = parse_words(w)[0]
    ok(abs(s["ax"] - 0.012) < 1e-9 and abs(s["ay"] + 0.034) < 1e-9 and abs(s["az"] - 1.0) < 1e-9, "PipeOut word parsing (X/Y/Z)")
    ok(s["en1"] == 1 and s["dir1"] == 1 and s["running"] == 1 and s["fault"] == 0 and s["pulses"] == 77 and s["seq"] == 5,
       "PipeOut word parsing (flags, pulses, seq)")
    ok(abs(PRELAB_RES_DEG - 0.2214) < 0.001, "pre lab resolution 0.001/sin15 = %.3f deg" % PRELAB_RES_DEG)
    ok(abs(J_ARM - 1.96e-3) < 1e-5, "J = m L^2 / 12 = %.3e kg m^2" % J_ARM)
    ok(abs(circle_fit(np.array([1.1, 0.1, -0.9, 0.1]), np.array([0.2, 1.2, 0.2, -0.8]))[2] - 1.0) < 1e-9, "circle fit")

    # stall detector on a synthetic arm: tilt 15 deg, r = 0.1 m, 45 deg / 100 ms, then the post
    rng = np.random.default_rng(1)
    beta = math.radians(15)
    Gs = math.sin(beta)
    cfg = dict(DEFAULT_CFG, G=Gs, phi_home_deg=0.0, expected_deg=45.0)
    arm = Arm(cfg)

    def synth(t_hit, w_rad=math.radians(450), noise=0.0015, t_end=2.0, phi0=-120.0):
        out, dt = [], 1 / ODR_HZ
        phi, w_now = math.radians(phi0), 0.0
        for i in range(int(t_end / dt)):
            t = i * dt
            running = 1
            if t < t_hit:
                a_ang = (w_rad - w_now) / 0.03                  # 30 ms spin-up
            else:
                a_ang = 0.0
                w_now = 0.0                                     # jammed against the post
            w_now += a_ang * dt if t < t_hit else 0.0
            phi += w_now * dt
            ar = Gs * math.cos(phi) - w_now ** 2 * R_IMU / G0 + rng.normal(0, noise)   # centripetal on radial
            at = -Gs * math.sin(phi) + a_ang * R_IMU / G0 + rng.normal(0, noise)
            out.append({"t": t, "ax": ar, "az": at, "ay": math.cos(beta), "running": running, "en1": 1, "dir1": 1,
                        "fault": 0, "pulses": int(t * 200), "status": 0x0F, "seq": i & 0xFF})
        return out

    data = synth(t_hit=1.0)
    det = StallDetector(arm, 45.0)
    det.prime([dict(data[0], running=0)] * NAVG)
    trip_t = None
    for x in data:
        if det.feed(x, x["t"]):
            trip_t = x["t"]
            break
    ok(trip_t is not None and 1.0 < trip_t <= 1.31, "chord detector: no trip while turning, trips %.0f ms after the post"
       % ((trip_t - 1.0) * 1e3 if trip_t else -1))
    ok(all(wn["verdict"] in ("ok", "grace") for wn in det.windows if wn["t_end"] < 1.0),
       "chord detector: every window before the post is 'ok' (w^2 r = %.2f g > G = %.2f g)" % (math.radians(450) ** 2 * R_IMU / G0, Gs))
    # the same data through a plain atan2 detector would have false-tripped while turning
    phis = [Arm.phi_of(x["ax"], x["az"]) for x in data]
    win = int(WINDOW * ODR_HZ)
    moves = [abs(wrap180(np.mean(phis[k + win - 8:k + win]) - np.mean(phis[k - 8:k]))) for k in range(win, 400 - win, win)]   # pre lab: 8 samples
    ok(min(moves[1:]) < MIN_FRAC * 45.0, "naive atan2 detector WOULD false-trip while moving (min moved %.1f deg)" % min(moves[1:]))
    det2 = StallDetector(arm, 45.0)
    det2.prime([dict(data[0], running=0)] * NAVG)
    free = synth(t_hit=99.0, t_end=1.0)
    ok(not any(det2.feed(x, x["t"]) for x in free), "free run of 1 s: no trip")
    print("ALL PASSED" if bad == 0 else "%d FAILED" % bad)
    return bad


# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["selftest", "check", "pulsetest", "interactive", "home", "tilt", "calibrate",
                                     "sweep", "resolution", "move", "goto", "stall", "reset", "analyze"])
    ap.add_argument("rest", nargs="*", help="move: N DIR | goto: ANGLE | stall: DIR [N] | analyze: FILE...")
    ap.add_argument("--duty", type=int, default=50, help="EN1 duty in %% (default 50)")
    ap.add_argument("--label", default=None, help="name tag for the log files (e.g. 6V)")
    ap.add_argument("--load", action="store_true", help="reload the bit file even if it is already running")
    ap.add_argument("--radial", default=None, help="check: override radial axis, e.g. +x or -y")
    ap.add_argument("--tangential", default=None, help="check: override tangential axis, e.g. +z")
    ap.add_argument("--fs", type=int, choices=[2, 4], default=None, help="accelerometer full scale (g)")
    ap.add_argument("--volts", type=float, default=V_DEFAULT, help="VMOTOR from the E3631A +6V output (default 6.0, max 6.0)")
    ap.add_argument("--power", action="store_true", help="pulsetest: switch VMOTOR on (default: off, scope only)")
    ap.add_argument("--no-supply", action="store_true", help="do not touch the E3631A (set it by hand)")
    ap.add_argument("--scope", action="store_true", help="pulsetest: capture EN1 (CH1) and DIR1 (CH2) on the USB scope")
    args = ap.parse_args()

    if args.mode == "selftest":
        sys.exit(mode_selftest(args))
    if args.mode == "analyze":
        args.files = args.rest
        if not args.files:
            sys.exit("analyze needs one or more CSV logs from results/")
        mode_analyze(args)
        return

    args.n, args.dir, args.angle = 0, 1, 0.0
    try:
        if args.mode == "move":
            args.n, args.dir = int(args.rest[0]), int(args.rest[1])
        elif args.mode == "goto":
            args.angle = float(args.rest[0])
        elif args.mode == "stall":
            args.dir = int(args.rest[0])
            args.n = int(args.rest[1]) if len(args.rest) > 1 else 0
        elif args.mode == "pulsetest":
            args.n = int(args.rest[0]) if args.rest else 10
            args.dir = int(args.rest[1]) if len(args.rest) > 1 else 1
    except (IndexError, ValueError):
        sys.exit("usage: move N DIR | goto ANGLE | stall DIR [N] | pulsetest [N [DIR]]")

    cfg = load_cfg()
    if args.fs:
        cfg["fs"] = args.fs
    b = Board(cfg, force_load=args.load)
    ps = None
    try:
        # the FPGA is up with EN1 low before VMOTOR comes on; the finally below switches it off again
        if not args.no_supply and (args.mode in MOTOR_MODES or (args.mode == "pulsetest" and args.power)):
            ps = Supply(cfg)
            ps.on(args.volts)
        {"check": mode_check, "pulsetest": mode_pulsetest, "interactive": mode_interactive, "home": mode_home,
         "tilt": mode_tilt, "calibrate": mode_calibrate, "sweep": mode_sweep, "resolution": mode_resolution,
         "move": mode_move, "goto": mode_goto, "stall": mode_stall, "reset": mode_reset}[args.mode](b, cfg, args)
    except KeyboardInterrupt:
        print("\nCtrl-C: STOP")
    except RuntimeError as e:
        print("\nREFUSED / ERROR:", e)
    finally:
        if ps is not None:
            ps.off()
            ps.close()
        b.close()


if __name__ == "__main__":
    main()
