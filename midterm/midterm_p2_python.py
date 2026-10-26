# -*- coding: utf-8 -*-
"""
ECE 437 Midterm Part 2 - the bus. The same SDA edge on both I2C buses,
captured 100+ times each, fitted in Python, turned into bus capacitance.

FPGA: the Part 1 bit file (SensorBoard_I2C/bitfile/SensorBoard_Top.bit), no
new firmware. param1[8] picks the bus per transaction, and the scope marker
on PMOD_A1 is high for FSM states 110-116 on whichever bus is selected, so
the trigger is the same event on both buses.

    python midterm_p2_python.py check              both buses answer (ADT7420 on 0, LSM303 on 1)
    python midterm_p2_python.py capture --bus 0    2a/2b  N edges on bus 0 (default N = 120)
    python midterm_p2_python.py capture --bus 1    2a/2b  N edges on bus 1
    python midterm_p2_python.py analyze            2b/2c  fit every edge, histograms, C, max SCL (no hardware)
    python midterm_p2_python.py run                check, bus 0, (move probe), bus 1, analyze
    python midterm_p2_python.py sweep              3e  raise SCL until the ACK stops (no scope)
    python midterm_p2_python.py selftest           edge fit on a synthetic waveform (no hardware)

Wiring (MSO-X 3024T, 10x probes):
    CH1 tip on SDA_0 for bus 0, then on SDA_1 for bus 1 (or --sda1-ch 3 with a
        second probe left on SDA_1, so nothing moves between runs)
    CH2 tip on PMOD A pin 1 (marker), ground on PMOD pin 5 (the ground next to it)
    every probe ground on a PMOD GND pin

The event (t = 0 at the marker's rising edge, Ts = one FSM state = 2.5 us):
    state 110  t = 0      last bit of the READ address (R = 1, SDA high)
    state 113  t = 4 Ts   SCL falls -> the slave pulls SDA low:      ACK FALL (slave's transistor)
    state 115  t = 5-7 Ts SCL high: the ACK clock;   marker ends at t = 7 Ts
    state 117  t = 8 Ts   SCL falls -> the slave lets SDA go:        ACK RISE (10 k pull-up only)
               t = 8-12 Ts data bit 7, which must be 1 for the rise to exist
The read on each bus is chosen so that data bit 7 is 1:
    bus 0: ADT7420 (0x48) ID register 0x0B = 0xCB
    bus 1: LSM303 accelerometer (0x19) CTRL_REG4_A 0x23 = 0x88 (written first; Part 1's value)

Capacitance: the rise is an RC charge, v = Vh - (Vh - Vl) e^(-t/tau), so
    t_rise(10-90 %) = tau ln 9,   C = tau / R,   R = R4..R7 = 10.00 k (schematic)
I2C rise-time limits (NXP UM10204 Table 10) are measured 30 % -> 70 % VDD,
i.e. t_r(spec) = tau ln(7/3); that is what is compared with the limits.
"""
import sys
import os
import time
import json
import csv
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
BITFILE = os.path.join(HERE, "SensorBoard_I2C", "bitfile", "SensorBoard_Top.bit")
RESULTS = os.path.join(HERE, "results")
OK_PY_DIR = r"C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64"
OK_DLL_DIR = r"C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64"
DESIGN_ID = 0xEC437002
TS_NOMINAL = 252 / 100.8e6           # one FSM state: 2.5 us (100 kHz SCL)

# ---- FrontPanel endpoints (SensorBoard_Top.v) ----
EP_PARAM1, EP_PARAM2, EP_PARAM3 = 0x01, 0x02, 0x03
EP_RESULT0, EP_RX_LO, EP_COUNT, EP_ID = 0x20, 0x21, 0x25, 0x3F
EP_TRIG_IN = 0x40

# ---- the read on each bus: (name, 7-bit address, register, expected byte, bit 7 must be 1) ----
BUS_READ = {
    0: ("ADT7420 ID", 0x48, 0x0B, 0xCB),
    1: ("LSM303 CTRL_REG4_A", 0x19, 0x23, 0x88),
}
BUS_DEVICES = {0: "ADT7420, HTS221, LPS35HW (3 devices)", 1: "LSM303 accel + mag, AD7156 (2 devices)"}

# ---- board ----
R_PULLUP = {0: 10.00e3, 1: 10.00e3}  # R4..R7, one per line, all 10.00 k (Sensor_Board_Schematic.pdf)
LN9 = np.log(9.0)
LN_SPEC = np.log(7.0 / 3.0)          # 30 % -> 70 %
I2C_MODES = [                         # (name, f_SCL max, t_r max)  UM10204 Table 10
    ("Fast-mode Plus", 1000e3, 120e-9),
    ("Fast-mode", 400e3, 300e-9),
    ("Standard-mode", 100e3, 1000e-9),
]

# ---- scope ----
SCOPE_MODEL = "MSO-X 3024T"
SDA_VDIV, MARK_VDIV, OFFSET_V, TRIG_LEVEL_V = 0.5, 1.0, 1.65, 1.65
TIME_RANGE_S = 32e-6                  # whole screen: -1 us .. 31 us (12 Ts = 30 us)
TIME_POS_S = 15e-6
N_DEFAULT = 120
RISE_SMOOTH_S = 25e-9                 # moving average applied to the slow rise only (see analyze_edge)


def fail(msg):
    print("FAIL:", msg)
    sys.exit(1)


def results_path(name):
    os.makedirs(RESULTS, exist_ok=True)
    return os.path.join(RESULTS, name)


def save_json(name, obj):
    p = results_path(name)
    with open(p, "w") as f:
        json.dump(dict(obj, saved=time.strftime("%Y-%m-%d %H:%M:%S")), f, indent=2)
    print("  logged ->", p)


def save_csv(name, header, rows):
    p = results_path(name)
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print("  logged ->", p, "(%d rows)" % len(rows))


# ---------------------------------------------------------------------------
# FPGA / I2C access (Part 1 Board, with the bus as an argument)
# ---------------------------------------------------------------------------
class Board:
    def __init__(self, load=True):
        sys.path.append(OK_PY_DIR)
        os.add_dll_directory(OK_DLL_DIR)
        import ok
        self.dev = ok.okCFrontPanel()
        if self.dev.OpenBySerial("") != 0:
            sys.exit("No Opal Kelly board found. Is FrontPanel still open? Close it.")
        if load:
            if self.dev.ConfigureFPGA(BITFILE) != 0:
                sys.exit("Bitfile did not load: " + BITFILE)
        if not self.dev.IsFrontPanelEnabled():
            fail("FrontPanel is not enabled in the FPGA design.")
        self.dev.UpdateWireOuts()
        did = self.dev.GetWireOutValue(EP_ID) & 0xFFFFFFFF
        if did != DESIGN_ID:
            fail("design ID 0x%08X, expected 0x%08X (wrong bit file?)" % (did, DESIGN_ID))

    def close(self):
        self.dev.Close()

    def transact(self, p1, p2, p3, timeout=0.1):
        """One transaction; waits for the FPGA's completed-transaction count to step by one."""
        d = self.dev
        d.SetWireInValue(EP_PARAM1, p1)
        d.SetWireInValue(EP_PARAM2, p2)
        d.SetWireInValue(EP_PARAM3, p3)
        d.UpdateWireIns()
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
        r0 = d.GetWireOutValue(EP_RESULT0)
        return d.GetWireOutValue(EP_RX_LO) & 0xFF, bool(r0 & 1), (r0 >> 16) & 1

    def read(self, bus, addr7, sub):
        rx, nack, bus_used = self.transact((bus << 8) | (addr7 << 1), sub, (1 << 4) | 0x1)
        if bus_used != bus:
            fail("FPGA reports bus %d, asked for bus %d" % (bus_used, bus))
        if nack:
            fail("NACK reading 0x%02X from device 0x%02X on bus %d" % (sub, addr7, bus))
        return rx

    def write(self, bus, addr7, sub, value):
        _, nack, _ = self.transact((bus << 8) | (addr7 << 1), (value << 8) | sub, 0x3)
        if nack:
            fail("NACK writing 0x%02X to 0x%02X of device 0x%02X on bus %d" % (value, sub, addr7, bus))


def prepare_bus(b, bus):
    """Make the bus's marker read return a byte with bit 7 = 1, and check it."""
    name, addr, sub, want = BUS_READ[bus]
    if bus == 1:
        b.write(1, addr, sub, want)
    got = b.read(bus, addr, sub)
    ok = got == want
    print("  bus %d  %-20s device 0x%02X reg 0x%02X: read 0x%02X, expect 0x%02X  -> %s"
          % (bus, name, addr, sub, got, want, "PASS" if ok else "FAIL"))
    return ok


def run_check(b):
    print("=" * 64)
    print("BUS CHECK  (same bit file, param1[8] picks the bus)")
    print("=" * 64)
    good = all([prepare_bus(b, 0), prepare_bus(b, 1)])
    # the bus select really switches: a device only on one bus must NACK on the other
    # (0x48 is no use here: the AD7156 on bus 1 has the ADT7420's address)
    for addr, home, name in ((0x19, 1, "LSM303 accel"), (0x5C, 0, "LPS35HW")):
        other = 1 - home
        _, nack, _ = b.transact((other << 8) | (addr << 1), 0x0F, (1 << 4) | 0x1)
        print("  0x%02X %s addressed on bus %d (it is on bus %d): %s  -> %s"
              % (addr, name, other, home, "NACK" if nack else "ACK", "PASS" if nack else "FAIL"))
        good &= nack
    print("  both buses: %s" % ("PASS" if good else "FAIL"))
    return good


# ---------------------------------------------------------------------------
# 3e: the other route to the limit - raise SCL until the ACK stops appearing
# ---------------------------------------------------------------------------
SWEEP_DIVIDES = [252, 168, 126, 100, 84, 72, 63, 56, 50, 45, 42, 38, 36, 32, 30, 28, 26, 24, 22, 20,
                 18, 16, 14, 12, 10, 8, 6, 4]     # f_SCL = 100.8 MHz / (4 x divide); the FSM floor is 4
SWEEP_READS = 100


def run_sweep(b, reads=SWEEP_READS):
    """Same marker read as the captures, at rising SCL: count ACKs and correct bytes."""
    print("=" * 64)
    print("SCL SWEEP  (%d reads per step; ACK = every ACK slot ACKed; data = byte correct)" % reads)
    print("=" * 64)
    rows, summary = [], {}
    for bus in (0, 1):
        name, addr, sub, want = BUS_READ[bus]
        if not prepare_bus(b, bus):
            fail("bus %d does not read correctly at 100 kHz" % bus)
        last_ok = None
        for div in SWEEP_DIVIDES:
            f = 100.8e6 / (4 * div)
            n_ack = n_data = 0
            for _ in range(reads):
                rx, nack, _ = b.transact((bus << 8) | (addr << 1), sub, (div << 16) | (1 << 4) | 0x1)
                n_ack += not nack
                n_data += (not nack) and rx == want
            rows.append([bus, div, "%.0f" % f, reads, n_ack, n_data])
            print("  bus %d  divide %3d  SCL %7.1f kHz   ACK %3d/%d   data 0x%02X %3d/%d"
                  % (bus, div, f / 1e3, n_ack, reads, want, n_data, reads))
            if n_data == reads:
                last_ok = f
            # back at 100 kHz: a slave confused by a too-fast frame must not leave the bus stuck
            for _ in range(50):
                rx, nack, _ = b.transact((bus << 8) | (addr << 1), sub, (1 << 4) | 0x1)
                if not nack and rx == want:
                    break
            else:
                print("  bus %d did not recover at 100 kHz - stopping this bus" % bus)
                break
            if n_ack == 0 and n_data == 0:
                break
        first_fail = next((float(r[2]) for r in rows if r[0] == bus and r[5] < reads), None)
        summary["bus%d" % bus] = {"highest_all_correct_Hz": last_ok, "first_step_with_errors_Hz": first_fail}
        print("  bus %d: every read correct up to %.0f kHz; first errors at %s"
              % (bus, last_ok / 1e3, "%.0f kHz" % (first_fail / 1e3) if first_fail else "none (FSM floor)"))
    save_csv("p2_scl_sweep.csv", ["bus", "divide", "scl_Hz", "reads", "ack_ok", "data_ok"], rows)
    save_json("p2_scl_sweep.json", summary)


# ---------------------------------------------------------------------------
# Scope (Keysight InfiniiVision, SCPI over USB)
# ---------------------------------------------------------------------------
def drain_errors(inst, max_reads=20):
    errors = []
    for _ in range(max_reads):
        err = inst.query(":SYSTem:ERRor?").strip()
        if err.startswith("+0") or err.startswith("0,") or "No error" in err:
            break
        errors.append(err)
    return errors


def scpi_write(inst, cmd):
    inst.write(cmd)
    errors = drain_errors(inst)
    if errors:
        raise RuntimeError("SCPI error after '%s': %s" % (cmd, "; ".join(errors)))


def scpi_query(inst, cmd):
    resp = inst.query(cmd).strip()
    errors = drain_errors(inst)
    if errors:
        raise RuntimeError("SCPI error after '%s': %s" % (cmd, "; ".join(errors)))
    return resp


def open_scope():
    import pyvisa as visa
    rm = visa.ResourceManager()
    for res in rm.list_resources():
        if not res.startswith("USB"):
            continue
        try:
            inst = rm.open_resource(res)
            inst.timeout = 5000
            if SCOPE_MODEL in inst.query("*IDN?"):
                return rm, inst
            inst.close()
        except Exception:
            pass
    rm.close()
    sys.exit("No %s found on USB. Is it on and plugged into this PC?" % SCOPE_MODEL)


def scope_configure(scope, sda_ch, mark_ch):
    """SDA and marker, DC, full bandwidth, NORMAL acquisition, NORMAL trigger on the marker's rising edge."""
    scpi_write(scope, "*CLS")
    scpi_write(scope, ":STOP")
    for ch in range(1, 5):
        scpi_write(scope, ":CHANnel%d:DISPlay %s" % (ch, "ON" if ch in (sda_ch, mark_ch) else "OFF"))
    for ch, vdiv in ((sda_ch, SDA_VDIV), (mark_ch, MARK_VDIV)):
        c = ":CHANnel%d" % ch
        scpi_write(scope, c + ":COUPling DC")
        scpi_write(scope, c + ":BWLimit OFF")          # a 20 MHz limit would slow the ~8 ns fall
        scpi_write(scope, c + ":SCALe %g" % vdiv)
        scpi_write(scope, c + ":OFFSet %g" % OFFSET_V)
    scpi_write(scope, ":TIMebase:MODE MAIN")
    scpi_write(scope, ":TIMebase:RANGe %g" % TIME_RANGE_S)
    scpi_write(scope, ":TIMebase:REFerence CENTer")
    scpi_write(scope, ":TIMebase:POSition %g" % TIME_POS_S)
    scpi_write(scope, ":ACQuire:TYPE NORMal")          # every capture is one real edge, never averaged
    scpi_write(scope, ":TRIGger:MODE EDGE")
    scpi_write(scope, ":TRIGger:EDGE:SOURce CHANnel%d" % mark_ch)
    scpi_write(scope, ":TRIGger:EDGE:SLOPe POSitive")
    scpi_write(scope, ":TRIGger:EDGE:LEVel %g" % TRIG_LEVEL_V)
    scpi_write(scope, ":TRIGger:SWEep NORMal")         # only the marker triggers
    return {"idn": scpi_query(scope, "*IDN?"),
            "probe_sda": float(scpi_query(scope, ":CHANnel%d:PROBe?" % sda_ch)),
            "probe_marker": float(scpi_query(scope, ":CHANnel%d:PROBe?" % mark_ch)),
            "time_range": float(scpi_query(scope, ":TIMebase:RANGe?")),
            "trigger": "CH%d rising %.2f V" % (mark_ch, TRIG_LEVEL_V)}


def scope_fetch(scope, ch):
    """Stopped acquisition of one channel: RAW, maximum points, 16-bit codes. Returns (codes, preamble)."""
    scpi_write(scope, ":WAVeform:SOURce CHANnel%d" % ch)
    scpi_write(scope, ":WAVeform:FORMat WORD")
    scpi_write(scope, ":WAVeform:BYTeorder LSBFirst")
    scpi_write(scope, ":WAVeform:UNSigned ON")
    scpi_write(scope, ":WAVeform:POINts:MODE RAW")
    scpi_write(scope, ":WAVeform:POINts MAXimum")
    pre = [float(x) for x in scpi_query(scope, ":WAVeform:PREamble?").split(",")]
    old = scope.timeout
    scope.timeout = 60000
    scope.chunk_size = 1024 * 1024
    try:
        codes = scope.query_binary_values(":WAVeform:DATA?", datatype="H", is_big_endian=False,
                                          container=np.array)
    finally:
        scope.timeout = old
    if len(codes) != int(pre[2]):
        raise RuntimeError("got %d points, preamble says %d" % (len(codes), int(pre[2])))
    return codes.astype(np.uint16), np.array(pre)


def to_volts(codes, pre):
    """Preamble = format, type, points, count, xinc, xorg, xref, yinc, yorg, yref."""
    i = np.arange(len(codes), dtype=float)
    t = (i - pre[6]) * pre[4] + pre[5]
    v = (np.asarray(codes, dtype=float) - pre[9]) * pre[7] + pre[8]
    return t, v


def acquire(scope, b, bus, sda_ch, mark_ch, max_reads=50):
    """Arm one acquisition, then run the bus's marker read until the scope has triggered."""
    name, addr, sub, want = BUS_READ[bus]
    scpi_query(scope, ":TER?")                       # clear the trigger event register
    scpi_write(scope, ":SINGle")
    time.sleep(0.05)
    for _ in range(max_reads):
        got = b.read(bus, addr, sub)
        if got != want:
            fail("bus %d read 0x%02X, expected 0x%02X (bit 7 must be 1 for the ACK rise)" % (bus, got, want))
        if int(scpi_query(scope, ":TER?")) == 1:
            break
        time.sleep(0.005)
    else:
        raise RuntimeError("scope never triggered after %d reads - is CH%d on PMOD A pin 1?" % (max_reads, mark_ch))
    t0 = time.time()
    while int(scpi_query(scope, ":OPERegister:CONDition?")) & 0x8:   # bit 3: still running
        if time.time() - t0 > 5:
            raise RuntimeError("acquisition did not finish after the trigger")
        time.sleep(0.01)
    sda, pre_s = scope_fetch(scope, sda_ch)
    mark, pre_m = scope_fetch(scope, mark_ch)
    if len(sda) != len(mark):
        raise RuntimeError("SDA and marker records differ in length")
    return sda, pre_s, mark, pre_m


# ---------------------------------------------------------------------------
# Edge fit: one capture -> rise and fall, computed from the array
# ---------------------------------------------------------------------------
def crossing(t, v, i, level):
    """Time v crosses level between samples i and i + 1 (linear interpolation)."""
    v0, v1 = v[i], v[i + 1]
    return t[i] if v1 == v0 else t[i] + (level - v0) * (t[i + 1] - t[i]) / (v1 - v0)


def analyze_edge(t, sda, mark):
    """ACK fall (t ~ 4 Ts) and ACK release rise (t ~ 8 Ts) of one capture.

    Time zero and the state period come from the marker: it is high for the
    7 states 110..116. Levels are medians over flat stretches of the bit
    timeline; 10 % / 90 % are of each edge's own swing. Noise-safe crossings:
    for the rise, the first sample at/above 90 %, then the last sample at/below
    10 % before it (mirror image for the fall).
    """
    mhalf = 0.5 * (mark.min() + mark.max())
    up = np.flatnonzero((mark[:-1] < mhalf) & (mark[1:] >= mhalf))
    dn = np.flatnonzero((mark[:-1] >= mhalf) & (mark[1:] < mhalf))
    if len(up) == 0 or len(dn[dn > up[0]]) == 0:
        raise RuntimeError("marker pulse not complete inside the record")
    t_on = crossing(t, mark, up[0], mhalf)
    ts = (crossing(t, mark, dn[dn > up[0]][0], mhalf) - t_on) / 7.0
    if abs(ts / TS_NOMINAL - 1) > 0.05:
        raise RuntimeError("marker width gives Ts = %.3f us, expected %.3f us" % (ts * 1e6, TS_NOMINAL * 1e6))
    tt = (t - t_on) / ts                              # time in FSM states
    med = lambda a, b: float(np.median(sda[(tt > a) & (tt < b)]))
    v_addr_high = med(3.0, 3.95)                      # R bit, SCL low again, before the ACK
    v_ack_low = med(5.0, 7.9)                         # slave holding the ACK
    v_d7_high = med(10.0, 11.9)                       # data bit 7 = 1, pull-up only
    if v_addr_high - v_ack_low < 1.0 or v_d7_high - v_ack_low < 1.0:
        raise RuntimeError("SDA swing %.2f / %.2f V - is the SDA probe on the right bus, and did the slave ACK?"
                           % (v_addr_high - v_ack_low, v_d7_high - v_ack_low))
    n = np.arange(len(sda))

    # fall: the slave pulls the ACK low after SCL falls at 4 Ts
    sw = v_addr_high - v_ack_low
    f90, f10 = v_ack_low + 0.9 * sw, v_ack_low + 0.1 * sw
    j10 = np.flatnonzero((tt > 3.9) & (tt < 5.0) & (sda <= f10))[0]
    j90 = np.flatnonzero((n < j10) & (sda >= f90))[-1]
    t_f90, t_f10 = crossing(t, sda, j90, f90), crossing(t, sda, j10 - 1, f10)

    # rise: the slave lets go after SCL falls at 8 Ts; only the pull-up charges the bus.
    # Near 90 % the RC edge is slow (~1 mV/ns), so a few mV of noise would cross the
    # level tens of ns early. A 25 ns moving average (< 3 % of the edge) removes that;
    # on an exponential it only shifts both levels' times equally.
    w = max(1, int(round(RISE_SMOOTH_S / (t[1] - t[0]))))
    sm = np.convolve(sda, np.ones(w) / w, mode="same")
    sw = v_d7_high - v_ack_low
    r10, r90 = v_ack_low + 0.1 * sw, v_ack_low + 0.9 * sw
    k90 = np.flatnonzero((tt > 7.9) & (tt < 10.0) & (sm >= r90))[0]
    k10 = np.flatnonzero((n < k90) & (sm <= r10))[-1]
    t_r10, t_r90 = crossing(t, sm, k10, r10), crossing(t, sm, k90 - 1, r90)

    # RC fit of the rise between 10 % and 90 %: ln((Vh - v) / swing) = -(t - t0) / tau
    sel = (t >= t_r10) & (t <= t_r90)
    tau_fit = float("nan")
    if np.count_nonzero(sel) >= 5:
        y = np.log(np.clip((v_d7_high - sda[sel]) / sw, 1e-6, None))
        tau_fit = -1.0 / np.polyfit(t[sel], y, 1)[0]
    return {"rise": t_r90 - t_r10, "fall": t_f10 - t_f90, "tau_fit": tau_fit, "ts": ts, "t_on": t_on,
            "v_high": v_d7_high, "v_low": v_ack_low, "v_addr_high": v_addr_high,
            "t_r10": t_r10, "t_r90": t_r90, "t_f90": t_f90, "t_f10": t_f10,
            "fall_after_scl": t_f90 - t_on - 4 * ts, "rise_after_scl": t_r10 - t_on - 8 * ts,
            "levels": (r10, r90, f90, f10)}


# ---------------------------------------------------------------------------
# 2a / 2b: capture N edges on one bus
# ---------------------------------------------------------------------------
def raw_name(bus):
    return "p2_bus%d_raw.npz" % bus


def run_capture(b, scope, bus, n, sda_ch, mark_ch):
    print("=" * 64)
    print("CAPTURE  bus %d (%s), %d edges" % (bus, BUS_DEVICES[bus], n))
    print("  SDA_%d on CH%d, marker (PMOD A pin 1) on CH%d" % (bus, sda_ch, mark_ch))
    print("=" * 64)
    if not prepare_bus(b, bus):
        fail("bus %d marker read does not return the expected byte" % bus)
    settings = scope_configure(scope, sda_ch, mark_ch)
    if settings["probe_sda"] < 10:
        print("  WARNING: SDA probe set to %gx - a 1x probe adds ~100 pF" % settings["probe_sda"])
    sda_all, mark_all, pre_s_all, pre_m_all = [], [], [], []
    k, tries = 0, 0
    while k < n:
        tries += 1
        if tries > n + 20:
            fail("too many captures could not be fitted")
        sda, pre_s, mark, pre_m = acquire(scope, b, bus, sda_ch, mark_ch)
        t, v = to_volts(sda, pre_s)
        try:
            e = analyze_edge(t, v, to_volts(mark, pre_m)[1])
        except (RuntimeError, IndexError) as ex:
            print("  capture skipped: %s" % ex)
            continue
        k += 1
        sda_all.append(sda); mark_all.append(mark); pre_s_all.append(pre_s); pre_m_all.append(pre_m)
        if k == 1:
            print("  %d points, %.3f ns/sample, Ts = %.3f us" % (len(t), pre_s[4] * 1e9, e["ts"] * 1e6))
        print("  edge %3d/%d: rise %6.1f ns   fall %5.2f ns   tau(fit) %6.1f ns"
              % (k, n, e["rise"] * 1e9, e["fall"] * 1e9, e["tau_fit"] * 1e9))
    np.savez_compressed(results_path(raw_name(bus)), sda=np.array(sda_all), marker=np.array(mark_all),
                        pre_sda=np.array(pre_s_all), pre_marker=np.array(pre_m_all), bus=bus,
                        sda_ch=sda_ch, mark_ch=mark_ch, settings=json.dumps(settings),
                        when=time.strftime("%Y-%m-%d %H:%M:%S"))
    print("  logged ->", results_path(raw_name(bus)), "(%d captures, raw scope codes)" % n)


# ---------------------------------------------------------------------------
# 2b / 2c: fit every saved capture, histograms, capacitance, max SCL
# ---------------------------------------------------------------------------
def fit_bus(bus):
    p = results_path(raw_name(bus))
    if not os.path.exists(p):
        return None
    d = np.load(p)
    edges = []
    for sda, mark, ps, pm in zip(d["sda"], d["marker"], d["pre_sda"], d["pre_marker"]):
        t, v = to_volts(sda, ps)
        edges.append(analyze_edge(t, v, to_volts(mark, pm)[1]))
    first = (to_volts(d["sda"][0], d["pre_sda"][0]), to_volts(d["marker"][0], d["pre_marker"][0])[1])
    return edges, first


def max_scl(tr_spec):
    for name, f, limit in I2C_MODES:
        if tr_spec <= limit:
            return name, f, limit
    return "none (slower than Standard-mode allows)", 0.0, I2C_MODES[-1][2]


def run_analyze():
    fits = {bus: fit_bus(bus) for bus in (0, 1)}
    missing = [bus for bus, f in fits.items() if f is None]
    if missing:
        fail("no captures for bus %s yet - run: capture --bus %d" % (missing, missing[0]))
    print("=" * 64)
    print("ANALYSIS  (every edge fitted from the saved arrays)")
    print("=" * 64)
    rows, summary = [], {}
    for bus, (edges, _) in fits.items():
        rise = np.array([e["rise"] for e in edges])
        fall = np.array([e["fall"] for e in edges])
        fit = np.array([e["tau_fit"] for e in edges])
        tau = rise.mean() / LN9
        c = tau / R_PULLUP[bus]
        tr_spec = tau * LN_SPEC
        mode, f, limit = max_scl(tr_spec)
        summary[bus] = {"devices": BUS_DEVICES[bus], "edges": len(edges),
                        "rise_mean_ns": rise.mean() * 1e9, "rise_std_ns": rise.std(ddof=1) * 1e9,
                        "fall_mean_ns": fall.mean() * 1e9, "fall_std_ns": fall.std(ddof=1) * 1e9,
                        "tau_from_rise_ns": tau * 1e9, "tau_logfit_ns": float(np.nanmean(fit)) * 1e9,
                        "R_pullup_ohm": R_PULLUP[bus], "C_bus_pF": c * 1e12,
                        "tr_spec_30_70_ns": tr_spec * 1e9, "max_mode": mode, "max_scl_Hz": f,
                        "mode_tr_limit_ns": limit * 1e9}
        s = summary[bus]
        print("  bus %d  %s, %d edges" % (bus, BUS_DEVICES[bus], len(edges)))
        print("    rise 10-90 %%: %7.1f +/- %5.1f ns     fall 90-10 %%: %5.2f +/- %4.2f ns"
              % (s["rise_mean_ns"], s["rise_std_ns"], s["fall_mean_ns"], s["fall_std_ns"]))
        print("    tau = rise / ln 9 = %.1f ns  (log fit %.1f ns)   C = tau / %.2f k = %.1f pF"
              % (s["tau_from_rise_ns"], s["tau_logfit_ns"], R_PULLUP[bus] / 1e3, s["C_bus_pF"]))
        print("    I2C t_r (30-70 %%) = tau ln(7/3) = %.0f ns  ->  max SCL: %s, %.0f kHz (t_r <= %.0f ns)"
              % (s["tr_spec_30_70_ns"], mode, f / 1e3, limit * 1e9))
        for k, e in enumerate(edges):
            rows.append([bus, k + 1, "%.4e" % e["rise"], "%.4e" % e["fall"], "%.4e" % e["tau_fit"],
                         "%.4f" % e["v_low"], "%.4f" % e["v_high"], "%.4e" % e["ts"]])
    dc = summary[0]["C_bus_pF"] - summary[1]["C_bus_pF"]
    print("  C(bus 0) - C(bus 1) = %.1f pF   (the probe's own ~10-15 pF is in both and cancels here)" % dc)
    save_csv("p2_edges.csv", ["bus", "capture", "rise_10_90_s", "fall_90_10_s", "tau_fit_s",
                              "v_ack_low_V", "v_high_V", "Ts_s"], rows)
    save_json("p2_summary.json", {"bus0": summary[0], "bus1": summary[1], "C_difference_pF": dc,
                                  "rise_definition": "10-90 % of the ACK-release edge (pull-up only)",
                                  "fall_definition": "90-10 % of the ACK edge (slave pulling low)",
                                  "spec": "UM10204 Table 10 t_r, 30 %-70 % VDD"})
    plot_histograms(fits, summary)
    plot_edges(fits)


def plot_histograms(fits, summary):
    colors = {0: "tab:blue", 1: "tab:orange"}
    for key, title, fname in (("rise", "10-90 % rise time (ACK release, pull-up only)", "p2_rise_hist.png"),
                              ("fall", "90-10 % fall time (slave pulling the ACK low)", "p2_fall_hist.png")):
        data = {bus: np.array([e[key] for e in fits[bus][0]]) * 1e9 for bus in fits}
        lo = min(x.min() for x in data.values())
        hi = max(x.max() for x in data.values())
        pad = 0.05 * (hi - lo) + 0.5
        bins = np.linspace(lo - pad, hi + pad, 60)       # one set of bins = common axes
        fig, ax = plt.subplots(figsize=(9, 4.5))
        for bus, x in data.items():
            s = summary[bus]
            ax.hist(x, bins=bins, alpha=0.6, color=colors[bus],
                    label="bus %d (%d devices), N = %d: %.1f +/- %.1f ns"
                    % (bus, 3 if bus == 0 else 2, len(x), s[key + "_mean_ns"], s[key + "_std_ns"]))
            ax.axvline(s[key + "_mean_ns"], color=colors[bus], ls="--", lw=1)
        ax.set_xlabel("%s time [ns]" % key)
        ax.set_ylabel("edges")
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
        fig.tight_layout()
        fig.savefig(results_path(fname), dpi=150)
        plt.close(fig)
        print("  saved ->", results_path(fname))


def plot_edges(fits):
    """One captured edge per bus: the whole ACK, then zooms on the fall and the rise."""
    fig, axs = plt.subplots(3, 2, figsize=(12, 10))
    for col, bus in enumerate((0, 1)):
        edges, ((t, sda), mark) = fits[bus]
        e = edges[0]
        us = (t - e["t_on"]) * 1e6
        r10, r90, f90, f10 = e["levels"]
        ax = axs[0, col]
        ax.plot(us, sda, lw=0.8, label="SDA_%d" % bus)
        ax.plot(us, mark, lw=0.8, alpha=0.7, label="marker (PMOD A1)")
        for k, name in ((4, "SCL falls: ACK"), (8, "SCL falls: release")):
            ax.axvline(k * e["ts"] * 1e6, color="gray", ls=":", lw=0.8)
            ax.text(k * e["ts"] * 1e6, 3.75, " " + name, fontsize=8, va="top")
        ax.set_xlim(-1, 31)
        ax.set_ylim(-0.4, 3.9)
        ax.set_title("Bus %d: SDA through the read-address ACK" % bus)
        ax.set_xlabel("time from marker rising edge [us]")
        ax.set_ylabel("V")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc="lower right")

        ax = axs[1, col]
        span = max(e["fall"], 5e-9)
        w = (t > e["t_f90"] - 4 * span) & (t < e["t_f10"] + 4 * span)
        ax.plot((t[w] - e["t_f90"]) * 1e9, sda[w], ".-", lw=0.8, ms=2)
        for lev in (f90, f10):
            ax.axhline(lev, color="gray", ls="--", lw=0.8)
        ax.axvline(0, color="red", ls=":")
        ax.axvline(e["fall"] * 1e9, color="red", ls=":")
        ax.set_title("Bus %d fall (90-10 %%): %.2f ns" % (bus, e["fall"] * 1e9), fontsize=10)
        ax.set_xlabel("time from 90 % point [ns]")
        ax.set_ylabel("V")
        ax.grid(True, alpha=0.3)

        ax = axs[2, col]
        w = (t > e["t_r10"] - e["rise"]) & (t < e["t_r90"] + e["rise"])
        ax.plot((t[w] - e["t_r10"]) * 1e9, sda[w], lw=0.8)
        for lev in (r10, r90):
            ax.axhline(lev, color="gray", ls="--", lw=0.8)
        ax.axvline(0, color="red", ls=":")
        ax.axvline(e["rise"] * 1e9, color="red", ls=":")
        ax.set_title("Bus %d rise (10-90 %%): %.0f ns, tau = %.0f ns" % (bus, e["rise"] * 1e9, e["rise"] / LN9 * 1e9),
                     fontsize=10)
        ax.set_xlabel("time from 10 % point [ns]")
        ax.set_ylabel("V")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(results_path("p2_edge_per_bus.png"), dpi=150)
    plt.close(fig)
    print("  saved ->", results_path("p2_edge_per_bus.png"))


# ---------------------------------------------------------------------------
# Self-test: the fit on a synthetic capture with known edges
# ---------------------------------------------------------------------------
def synthetic(tau_rise, tau_fall, dt=0.2e-9, noise=0.01, seed=0):
    ts = TS_NOMINAL
    t = np.arange(-1e-6, 31e-6, dt)
    mark = np.where((t >= 0) & (t < 7 * ts), 3.3, 0.0)
    tf, tr = 4 * ts + 0.15e-6, 8 * ts + 0.10e-6       # slave reacts a little after SCL falls
    sda = np.full_like(t, 3.3)
    a = t >= tf
    sda[a] = 3.3 * np.exp(-(t[a] - tf) / tau_fall)
    b = t >= tr
    sda[b] = 3.3 - (3.3 - sda[b][0]) * np.exp(-(t[b] - tr) / tau_rise)
    sda[t > 12 * ts] = 3.3
    sda += np.random.default_rng(seed).normal(0, noise, len(t))
    return t, sda, mark


def self_test(verbose=False):
    good = True
    for tau_r, tau_f in ((420e-9, 3.7e-9), (300e-9, 3.0e-9)):
        e = analyze_edge(*synthetic(tau_r, tau_f))
        er, ef = e["rise"] / (tau_r * LN9) - 1, e["fall"] / (tau_f * LN9) - 1
        ok = abs(er) < 0.02 and abs(ef) < 0.05 and abs(e["tau_fit"] / tau_r - 1) < 0.02
        good &= ok
        if verbose or not ok:
            print("  tau_r %.0f ns tau_f %.1f ns: rise %.1f ns (expect %.1f), fall %.2f ns (expect %.2f), "
                  "tau fit %.1f ns -> %s" % (tau_r * 1e9, tau_f * 1e9, e["rise"] * 1e9, tau_r * LN9 * 1e9,
                                             e["fall"] * 1e9, tau_f * LN9 * 1e9, e["tau_fit"] * 1e9,
                                             "PASS" if ok else "FAIL"))
    if not good:
        fail("edge fit does not recover the synthetic edges")
    return good


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["check", "capture", "analyze", "run", "sweep", "selftest"])
    ap.add_argument("--bus", type=int, choices=[0, 1])
    ap.add_argument("--n", type=int, default=N_DEFAULT, help="edges per bus (at least 100)")
    ap.add_argument("--sda0-ch", type=int, default=1, help="scope channel on SDA_0")
    ap.add_argument("--sda1-ch", type=int, default=1, help="scope channel on SDA_1")
    ap.add_argument("--mark-ch", type=int, default=2, help="scope channel on PMOD A pin 1")
    ap.add_argument("--no-load", action="store_true", help="keep the bitstream already in the FPGA")
    args = ap.parse_args()
    self_test(verbose=args.mode == "selftest")
    if args.mode == "selftest":
        print("  self-test PASS")
        return
    if args.mode == "analyze":
        run_analyze()
        return
    if args.mode == "capture" and args.bus is None:
        fail("capture needs --bus 0 or --bus 1")
    if args.n < 100:
        print("  NOTE: Part 2 asks for at least 100 edges per bus; --n is %d" % args.n)
    sda_ch = {0: args.sda0_ch, 1: args.sda1_ch}
    b = Board(load=not args.no_load)
    try:
        if args.mode == "check":
            sys.exit(0 if run_check(b) else 1)
        if args.mode == "sweep":
            run_sweep(b)
            return
        rm, scope = open_scope()
        try:
            if args.mode == "capture":
                run_capture(b, scope, args.bus, args.n, sda_ch[args.bus], args.mark_ch)
            else:
                if not run_check(b):
                    fail("bus check")
                input("\nCH%d on SDA_0, CH%d on PMOD A pin 1, grounds on PMOD GND. Enter... "
                      % (sda_ch[0], args.mark_ch))
                run_capture(b, scope, 0, args.n, sda_ch[0], args.mark_ch)
                if sda_ch[0] == sda_ch[1]:
                    input("\nMove the CH%d probe tip to SDA_1 (same ground). Enter... " % sda_ch[1])
                run_capture(b, scope, 1, args.n, sda_ch[1], args.mark_ch)
                run_analyze()
        finally:
            try:
                scope.write(":RUN")
                scope.close()
                rm.close()
            except Exception:
                pass
    finally:
        b.close()


if __name__ == "__main__":
    main()
