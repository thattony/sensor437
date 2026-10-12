# -*- coding: utf-8 -*-
"""
ECE 437 - Lab 7 Milestone 1: the sensor's floor.

    python lab7_m1.py --sensor-test    FPGA + ADT7420 checks only (no instruments)
    python lab7_m1.py --m1b            averaging law: fast run + 300 ms run (heater stays OFF)
    python lab7_m1.py --supply-test    E3631A P6V safety setup + 2 V known-resistor check, then OFF
    python lab7_m1.py --demo-cutoff    show the software cutoff tripping (low DEMO threshold)
    python lab7_m1.py --m1a            five-level heating sweep, 100 readings per level
    python lab7_m1.py --plot-only      redraw every plot from the saved CSVs (no hardware)

With no mode the script prints this help and does nothing.

The FPGA side is the verified Lab 7 design (I2C_Transmit.v): it puts the
ADT7420 in 16-bit mode after reset, and one rising edge on WireIn 0x00 bit 0
gives exactly one temperature read. The endpoint map below must match
lab7_python.py.
"""
import sys
import os
import time
import csv
import math
import argparse
import statistics
import numpy as np
import matplotlib
matplotlib.use("Agg")                    # save PNGs; --show reopens them at the end
import matplotlib.pyplot as plt

ok_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64'
ok_dll_loc = r'C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64'

sys.path.append(ok_loc)
os.add_dll_directory(ok_dll_loc)

import ok

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")

# FPGA (same endpoints as lab7_python.py)
LOAD_BITFILE = True     # --no-bitfile to keep the bitstream Vivado loaded over JTAG
BITFILE = os.path.join(HERE, "Lab7.runs", "impl_1", "JTAG_Test_File.bit")

WIRE_GO = 0x00          # WireIn: rising edge on bit 0 = one temperature read
WIRE_TEMP_MSB = 0x20    # WireOut: last temperature word, MSB
WIRE_TEMP_LSB = 0x21    # WireOut: last temperature word, LSB
WIRE_CONFIG   = 0x22    # WireOut: config register read back after power-up write
WIRE_ID       = 0x23    # WireOut: ID register read during power-up
WIRE_STATUS   = 0x24    # WireOut: bit 0 init_done, bit 1 init_ack_error, bit 2 temp_ack_error
WIRE_COUNT    = 0x25    # WireOut: number of completed temperature reads (16 bit)

CONFIG_16BIT = 0x80     # config register 0x03, bit 7 = 1 -> 16-bit resolution
ADT7420_ID   = 0xCB
LSB_C = 0.0078125       # 16-bit mode: 1 count = 1/128 deg C

ROOM_MIN_C = 10.0       # startup plausibility window (heater off)
ROOM_MAX_C = 40.0
SENSOR_TEMP_LIMIT_C = 85.0   # extra trip: sensor reading this hot during M1a -> output OFF

# Power resistor and supply limits
R_NOMINAL = 47.0        # ohm, 5 %
R_MIN = 44.65
R_MAX = 49.35
P_RATING_W = 0.5
V_ABS_MAX = 4.72        # sqrt(0.5 W * 44.65 ohm): never exceed, even measured
V_MAX_PLANNED = 4.4     # never command above this
I_LIMIT_A = 0.100       # hardware current limit; I^2 * R_MAX = 0.4935 W < 0.5 W
POWER_CUTOFF_W = 0.45   # software cutoff on measured |V * I|
CHANNEL = "P6V"

# M1a sweep: five evenly spaced MEASURED-power targets (supply |V x I|).
# The supply does not see a plain 47 ohm load: V/I fell from 47.5 ohm at 0.5 V
# to 37.5 ohm at 2.0 V (a diode-like path in parallel with the 46.7 ohm
# resistor). With V = sqrt(P * 47) the 0.30 / 0.40 W levels would need
# ~105 / ~122 mA and hit the 0.100 A limit, so the targets stop at 0.28 W and
# the voltages come from the measured I(V) curve (predicted I <= ~88 mA).
M1A_TARGET_POWERS_W = [0.00, 0.07, 0.14, 0.21, 0.28]
M1A_COMMAND_V       = [0.000, 1.637, 2.276, 2.768, 3.182]
M1A_N_READINGS = 100
M1A_READ_INTERVAL_S = 0.300   # >= one 240 ms conversion, so the 100 readings are independent
M1A_POWER_CHECK_EVERY = 10    # check supply power every this many readings (and after the last)

# Fixed settling wait: after each voltage change (and its immediate power
# check) wait LEVEL_WAIT_S before collecting the 100 readings. The wait is not
# a blind sleep: supply power and sensor temperature are checked every
# SETTLE_CHECK_S and logged to m1a_level<k>_settling.csv. Any remaining drift
# over the 100 readings is saved per level (trend_C_per_min, delta_C).
LEVEL_WAIT_S = 180.0
SETTLE_CHECK_S = 2.0
DRIFT_EDGE_N = 10             # first/last this many readings compared for drift

# Supply known-value check: 2.0 V across 47 ohm = 42.6 mA, 85 mW.
# 2 V (not 1 V) so the current is above a stale 30 mA limit and well above
# the supply's few-mA current readback error.
SANITY_V = 2.0
SANITY_V_TOL = 0.10           # measured V must be within this of commanded (constant-voltage mode)
SANITY_R_MIN = 35.0           # wide window: lead resistance + readback accuracy
SANITY_R_MAX = 60.0
SUPPLY_SETTLE_S = 0.5         # wait after a voltage change before measuring
CC_TRIP_A = 0.098             # current at/near the 0.100 A limit = supply current-limiting -> trip

# Cutoff demonstration: ramp to at most 2.0 V (85 mW) against a 50 mW DEMO threshold
DEMO_VOLTAGES = [0.5, 1.0, 1.5, 2.0]
DEMO_CUTOFF_W = 0.050

# M1b
FAST_N_DEFAULT = 2048         # >= 1000; 2048 leaves 8 blocks at N = 256
SLOW_N = 100
SLOW_INTERVAL_S = 0.300
MIN_BLOCKS = 8                # only use N with at least this many block means
DEPARTURE_RATIO = 1.5         # "departed" = measured sigma > 1.5 x the 1/sqrt(N) line


class PowerCutoff(Exception):
    """Raised after the software cutoff has switched the supply output off."""


dev = None              # FrontPanel device, opened in open_fpga()
commanded_v = None      # last P6V voltage commanded, for the current-limiting check


# ---------------------------------------------------------------------------
# FPGA / ADT7420 (same interface as lab7_python.py)
# ---------------------------------------------------------------------------
def open_fpga(load_bitfile):
    global dev
    dev = ok.okCFrontPanel()
    if dev.OpenBySerial("") != 0:
        dev = None
        raise RuntimeError("No Opal Kelly board found. Is FrontPanel still open? Close it.")
    print("FrontPanel host interface initialized.")
    if load_bitfile:
        if dev.ConfigureFPGA(BITFILE) != 0:
            raise RuntimeError("Bitfile did not load: " + BITFILE)
        print("Bitfile loaded:", BITFILE)
    else:
        print("Using the bitstream already in the FPGA (loaded over JTAG).")
    if not dev.IsFrontPanelEnabled():
        raise RuntimeError("FrontPanel is not enabled in the FPGA design.")
    print("FPGA communication: PASS")
    dev.SetWireInValue(WIRE_GO, 0)      # make sure the next 0 -> 1 is a real edge
    dev.UpdateWireIns()


def close_fpga():
    if dev is not None:
        dev.Close()


def read_wires():
    """Take one snapshot of all WireOuts and return them as a dict."""
    dev.UpdateWireOuts()                 # without this you read stale data
    status = dev.GetWireOutValue(WIRE_STATUS)
    return {
        "msb":            dev.GetWireOutValue(WIRE_TEMP_MSB) & 0xFF,
        "lsb":            dev.GetWireOutValue(WIRE_TEMP_LSB) & 0xFF,
        "config":         dev.GetWireOutValue(WIRE_CONFIG) & 0xFF,
        "id":             dev.GetWireOutValue(WIRE_ID) & 0xFF,
        "init_done":      bool(status & 0x1),
        "init_ack_error": bool(status & 0x2),
        "temp_ack_error": bool(status & 0x4),
        "count":          dev.GetWireOutValue(WIRE_COUNT) & 0xFFFF,
    }


def start_fsm():
    """Pulse GO high then low. The FSM acts on the rising edge only.

    No sleep between the two updates (lab7_python.py has 1 ms): each
    UpdateWireIns is a USB transaction, far longer than the two 2.5 us FSM
    clocks the synchroniser needs, and a missed edge is caught by the counter
    check in read_temperature() anyway.
    """
    dev.SetWireInValue(WIRE_GO, 1)
    dev.UpdateWireIns()
    dev.SetWireInValue(WIRE_GO, 0)
    dev.UpdateWireIns()


def read_temperature(timeout=0.5):
    """Trigger one read and return the raw 16-bit word from THAT transaction.

    The FPGA bumps the read counter only after the new word is latched, so a
    counter step of exactly one means one new transaction finished and the data
    is not left over from the previous read.
    """
    before = read_wires()["count"]
    start_fsm()
    t0 = time.time()
    while read_wires()["count"] == before:
        if time.time() - t0 > timeout:
            raise RuntimeError("no I2C transaction completed within %.1f s of the trigger" % timeout)
    w = read_wires()                     # second snapshot: data has been stable since the count stepped
    step = (w["count"] - before) & 0xFFFF
    if step != 1:
        raise RuntimeError("one trigger produced %d transactions" % step)
    if w["temp_ack_error"]:
        raise RuntimeError("sensor did not ACK during the temperature read")
    return (w["msb"] << 8) | w["lsb"]


def to_celsius(raw):
    """16-bit mode: two's complement, 1 LSB = 1/128 = 0.0078125 deg C."""
    if raw & 0x8000:
        raw -= (1 << 16)
    return raw * LSB_C


def sensor_self_test():
    """Same checks as lab7_python.py. Raises RuntimeError on any failure."""
    t0 = time.time()
    while True:
        w = read_wires()
        if w["init_done"]:
            break
        if time.time() - t0 > 1.0:
            raise RuntimeError("ADT7420 initialization did not complete (init_done never set) - "
                               "is the bitstream current?")
    if w["init_ack_error"]:
        raise RuntimeError("sensor did not ACK during initialization")
    print("ADT7420 initialization: PASS")
    if w["config"] != CONFIG_16BIT:
        raise RuntimeError("config register reads 0x%02X, expected 0x%02X" % (w["config"], CONFIG_16BIT))
    print("Resolution mode: 16-bit   (config register 0x03 = 0x%02X)" % w["config"])
    if w["id"] != ADT7420_ID:
        raise RuntimeError("ID register reads 0x%02X, expected 0x%02X" % (w["id"], ADT7420_ID))
    print("Sensor ID: PASS           (ID register 0x0B = 0x%02X)" % w["id"])

    time.sleep(0.25)                     # let the first 16-bit conversion finish
    temps = []
    for _ in range(3):
        raw = read_temperature()
        t = to_celsius(raw)
        if not (ROOM_MIN_C <= t <= ROOM_MAX_C):
            raise RuntimeError("temperature %.4f C (raw 0x%04X) outside %.0f to %.0f C"
                               % (t, raw, ROOM_MIN_C, ROOM_MAX_C))
        temps.append(t)
        time.sleep(0.25)
    print("Temperature: %.4f C   (raw 0x%04X)" % (temps[-1], raw))
    print("Sensor self-test: PASS")
    print("-" * 52)


# ---------------------------------------------------------------------------
# Power supply (E3631A, P6V)
# ---------------------------------------------------------------------------
def find_instruments():
    """Return (resource_manager, power_supply). Other instruments are closed again."""
    import pyvisa as visa
    rm = visa.ResourceManager()
    ps = None
    for res in rm.list_resources():
        inst = None
        try:
            inst = rm.open_resource(res)
            inst.timeout = 5000
            idn = inst.query("*IDN?").strip()
            print("  %s : %s" % (res, idn))
            if "E3631A" in idn and ps is None:
                ps, inst = inst, None        # keep this one open
        except Exception:
            print("  %s : no response (powered off, or not an instrument)" % res)
        finally:
            if inst is not None:
                inst.close()
    if ps is None:
        rm.close()
        raise RuntimeError("E3631A power supply not found. Is it on and connected?")
    print("Power supply: E3631A")
    return rm, ps


def drain_errors(inst, max_reads=20):
    """Read SYSTem:ERRor? until the queue is empty. Returns the errors found."""
    errors = []
    for _ in range(max_reads):
        err = inst.query("SYSTem:ERRor?").strip()
        if err.startswith("+0") or err.startswith("0,") or "No error" in err:
            break
        errors.append(err)
    return errors


def scpi_write(inst, cmd):
    """Send a command, then ask the instrument whether it liked it. Raise if not."""
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


def safe_off(ps):
    """Output OFF first, no questions asked. Never raises."""
    if ps is None:
        return
    try:
        ps.write("OUTPut:STATe OFF")
        state = ps.query("OUTPut:STATe?").strip()
        if state in ("0", "OFF"):
            print("Supply output: OFF (confirmed)")
        else:
            print("WARNING: supply output state reads '%s' - switch it off by hand!" % state)
        ps.write("APPLy %s, 0, %.3f" % (CHANNEL, I_LIMIT_A))     # leave 0 V programmed
    except Exception as e:
        print("WARNING: could not confirm supply output OFF (%s) - switch it off by hand!" % e)


def setup_supply(ps):
    """Output OFF, all channels 0 V, P6V current limit 0.100 A, verified by read-back."""
    stale = drain_errors(ps)
    if stale:
        print("Cleared stale instrument errors:", "; ".join(stale))
    scpi_write(ps, "*CLS")
    scpi_write(ps, "OUTPut:STATe OFF")
    # OUTPut ON switches all three outputs on the E3631A, so zero the unused ones too
    for ch in ("P25V", "N25V"):
        scpi_write(ps, "INSTrument:SELect %s" % ch)
        scpi_write(ps, "VOLTage 0")
    scpi_write(ps, "INSTrument:SELect %s" % CHANNEL)
    scpi_write(ps, "VOLTage 0")
    scpi_write(ps, "CURRent %.3f" % I_LIMIT_A)

    v_set = float(scpi_query(ps, "VOLTage?"))
    i_set = float(scpi_query(ps, "CURRent?"))
    out = scpi_query(ps, "OUTPut:STATe?")
    print("Channel: %s" % CHANNEL)
    print("Output: %s" % ("OFF" if out in ("0", "OFF") else out))
    print("Voltage setting: %.3f V" % v_set)
    print("Current limit: %.3f A" % i_set)
    if out not in ("0", "OFF"):
        raise RuntimeError("output did not switch off")
    if abs(v_set) > 0.001:
        raise RuntimeError("P6V voltage reads back %.3f V, expected 0" % v_set)
    if abs(i_set - I_LIMIT_A) > 0.001:
        raise RuntimeError("P6V current limit reads back %.3f A, expected %.3f A" % (i_set, I_LIMIT_A))
    print("Safety setup: PASS")


def output_on(ps):
    scpi_write(ps, "OUTPut:STATe ON")
    if scpi_query(ps, "OUTPut:STATe?") not in ("1", "ON"):
        raise RuntimeError("output did not switch on")
    print("Supply output: ON")


def read_supply_vi(ps):
    v = float(scpi_query(ps, "MEASure:VOLTage:DC? %s" % CHANNEL))
    i = float(scpi_query(ps, "MEASure:CURRent:DC? %s" % CHANNEL))
    return v, i


def log_cutoff(v, i, p, limit, reason):
    os.makedirs(RESULTS, exist_ok=True)
    path = os.path.join(RESULTS, "power_cutoff_log.csv")
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        wr = csv.writer(f)
        if new:
            wr.writerow(["time", "V", "I_A", "P_W", "limit_W", "reason"])
        wr.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), "%.4f" % v, "%.5f" % i, "%.4f" % p,
                     "%.3f" % limit, reason])


def trip(ps, v, i, p, limit, reason):
    """Software cutoff: output OFF immediately, then say why and log it."""
    safe_off(ps)
    print("")
    print("POWER CUTOFF")
    print("V = %.4f V" % v)
    print("I = %.5f A" % i)
    print("P = %.4f W   limit %.3f W" % (p, limit))
    print("Reason: %s" % reason)
    print("SUPPLY OUTPUT OFF")
    log_cutoff(v, i, p, limit, reason)
    raise PowerCutoff(reason)


def check_power(ps, limit=POWER_CUTOFF_W):
    """Measure V and I, compute P = |V * I|, trip the cutoff if too high. Returns (V, I, P)."""
    v, i = read_supply_vi(ps)
    p = abs(v * i)
    if p >= limit:
        trip(ps, v, i, p, limit, "P = %.4f W >= %.3f W" % (p, limit))
    if abs(v) > V_ABS_MAX:
        trip(ps, v, i, p, limit, "V = %.3f V > absolute limit %.2f V" % (v, V_ABS_MAX))
    # Supply in constant-current mode: current at the limit, or voltage sagging below the setting
    if abs(i) >= CC_TRIP_A:
        trip(ps, v, i, p, limit, "I = %.4f A at the %.3f A hardware current limit" % (i, I_LIMIT_A))
    if commanded_v is not None and commanded_v > 0.5 and v < commanded_v - SANITY_V_TOL:
        trip(ps, v, i, p, limit, "measured %.3f V < commanded %.3f V: supply current-limiting"
             % (v, commanded_v))
    return v, i, p


def set_voltage_checked(ps, voltage, limit=POWER_CUTOFF_W):
    """Program P6V (current limit re-sent every time), wait, then check power."""
    if not (0.0 <= voltage <= V_MAX_PLANNED):
        raise ValueError("refusing to command %.3f V (allowed 0 to %.2f V)" % (voltage, V_MAX_PLANNED))
    global commanded_v
    scpi_write(ps, "APPLy %s, %.3f, %.3f" % (CHANNEL, voltage, I_LIMIT_A))
    commanded_v = voltage
    time.sleep(SUPPLY_SETTLE_S)
    return check_power(ps, limit)


def sanity_check(ps):
    """Known value first: 2.0 V across the 47 ohm resistor. Leaves P6V at 0 V, output ON."""
    print("Known-value check: %.1f V across the power resistor" % SANITY_V)
    set_voltage_checked(ps, 0.0)
    output_on(ps)
    vs, cs = [], []
    for _ in range(3):
        v, i, p = set_voltage_checked(ps, SANITY_V) if not vs else check_power(ps)
        vs.append(v)
        cs.append(i)
    v, i = statistics.mean(vs), statistics.mean(cs)
    p = abs(v * i)
    r = v / i if abs(i) > 1e-6 else float("inf")
    print("  Measured voltage: %.4f V" % v)
    print("  Measured current: %.5f A" % i)
    print("  Measured power:   %.4f W" % p)
    print("  Estimated R:      %.1f ohm   (expect %.0f ohm, accept %.0f to %.0f)"
          % (r, R_NOMINAL, SANITY_R_MIN, SANITY_R_MAX))
    set_voltage_checked(ps, 0.0)
    if abs(v - SANITY_V) > SANITY_V_TOL:
        raise RuntimeError("measured %.3f V for %.1f V commanded: supply is current-limiting, "
                           "or wrong channel/wiring" % (v, SANITY_V))
    if not (SANITY_R_MIN <= r <= SANITY_R_MAX):
        raise RuntimeError("estimated R = %.1f ohm: resistor missing, wrong channel or bad wiring" % r)
    print("Known-value check: PASS")


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
def write_csv(name, header, rows):
    os.makedirs(RESULTS, exist_ok=True)
    path = os.path.join(RESULTS, name)
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(header)
        wr.writerows(rows)
    return path


def read_csv(name):
    path = os.path.join(RESULTS, name)
    if not os.path.exists(path):
        return None
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def save_fig(fig, name):
    os.makedirs(RESULTS, exist_ok=True)
    path = os.path.join(RESULTS, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print("Saved", path)


def fmt(x, spec):
    return "" if x is None else spec % x


# ---------------------------------------------------------------------------
# M1a: heat the sensor (fixed settling wait per level)
# ---------------------------------------------------------------------------
def check_sensor_temp(ps, t):
    if t > SENSOR_TEMP_LIMIT_C:
        v, i = read_supply_vi(ps)
        trip(ps, v, i, abs(v * i), POWER_CUTOFF_W,
             "sensor at %.2f C > %.0f C limit" % (t, SENSOR_TEMP_LIMIT_C))


def collect_readings(ps, level):
    """M1A_N_READINGS readings, M1A_READ_INTERVAL_S apart, power checked every few.

    Returns (temps, powers, vs, is_). Every reading is saved, nothing discarded.
    """
    rows, temps, vs, cs, ps_w = [], [], [], [], []
    print("Collecting %d readings (%.0f ms apart)..." % (M1A_N_READINGS, M1A_READ_INTERVAL_S * 1000))
    t0 = time.perf_counter()
    next_t = t0
    try:
        for k in range(M1A_N_READINGS):
            raw = read_temperature()
            t = time.perf_counter() - t0
            temp = to_celsius(raw)
            check_sensor_temp(ps, temp)
            temps.append(temp)
            v = i = p = None
            if k % M1A_POWER_CHECK_EVERY == 0 or k == M1A_N_READINGS - 1:
                v, i, p = check_power(ps)
                vs.append(v)
                cs.append(i)
                ps_w.append(p)
            rows.append([k + 1, "%.3f" % t, raw, "0x%04X" % raw, "%.6f" % temp,
                         fmt(v, "%.4f"), fmt(i, "%.5f"), fmt(p, "%.4f")])
            next_t = max(next_t + M1A_READ_INTERVAL_S, time.perf_counter())
            time.sleep(max(0.0, next_t - time.perf_counter()))
    finally:
        write_csv("m1a_level%d_readings.csv" % level,
                  ["sample", "t_s", "raw", "raw_hex", "temp_C", "V", "I_A", "P_W"], rows)
    return temps, ps_w, vs, cs


def settle_wait(ps, level):
    """Wait LEVEL_WAIT_S with the heater on, checking power + sensor every SETTLE_CHECK_S.

    Returns the actual elapsed wait in seconds. The trace is saved even on a trip.
    """
    print("Settling wait %.0f s (power and temperature checked every %.0f s)..."
          % (LEVEL_WAIT_S, SETTLE_CHECK_S))
    rows = []
    t0 = time.perf_counter()
    next_t = t0
    try:
        while True:
            t = time.perf_counter() - t0
            if t >= LEVEL_WAIT_S:
                break
            v, i, p = check_power(ps)
            temp = to_celsius(read_temperature())
            check_sensor_temp(ps, temp)
            rows.append(["%.2f" % t, "%.6f" % temp, "%.4f" % v, "%.5f" % i, "%.4f" % p])
            if len(rows) % 10 == 1:
                print("  t = %5.1f s   T = %.4f C   P = %.4f W" % (t, temp, p))
            next_t += SETTLE_CHECK_S
            time.sleep(max(0.0, min(next_t, t0 + LEVEL_WAIT_S) - time.perf_counter()))
    finally:
        write_csv("m1a_level%d_settling.csv" % level, ["t_s", "temp_C", "V", "I_A", "P_W"], rows)
    actual = time.perf_counter() - t0
    print("Settling wait done: %.1f s actual" % actual)
    return actual


def run_m1a(ps):
    levels = list(zip(M1A_TARGET_POWERS_W, M1A_COMMAND_V))
    for p, v in levels:                  # check the plan before anything is switched on
        if v > V_MAX_PLANNED or p >= POWER_CUTOFF_W:
            raise ValueError("planned level %.3f W / %.3f V exceeds the limits" % (p, v))

    header = ["level", "target_W", "command_V", "mean_V", "mean_I_A", "mean_P_W",
              "mean_T_C", "std_T_C", "n", "level_wait_s", "actual_wait_s", "trend_C_per_min",
              "first%d_mean_C" % DRIFT_EDGE_N, "last%d_mean_C" % DRIFT_EDGE_N, "delta_C",
              "detrended_std_C"]
    t_start = time.perf_counter()
    summary = []
    for k, (p_target, v_cmd) in enumerate(levels, 1):
        print("=" * 52)
        print("Level %d/%d" % (k, len(levels)))
        print("Target power: %.3f W" % p_target)
        print("Command voltage: %.3f V" % v_cmd)
        v, i, p = set_voltage_checked(ps, v_cmd)
        print("Measured voltage: %.4f V" % v)
        print("Measured current: %.5f A" % i)
        print("Measured power: %.4f W" % p)

        actual_wait = settle_wait(ps, k)
        temps, pw, vs, cs = collect_readings(ps, k)
        if len(temps) != M1A_N_READINGS:
            raise RuntimeError("level %d: %d readings, expected %d" % (k, len(temps), M1A_N_READINGS))
        tt = np.arange(len(temps)) * M1A_READ_INTERVAL_S
        slope, icpt = np.polyfit(tt, temps, 1)
        trend = slope * 60.0
        detr_std = float(np.std(np.array(temps) - (slope * tt + icpt), ddof=2))
        first_m = statistics.mean(temps[:DRIFT_EDGE_N])
        last_m = statistics.mean(temps[-DRIFT_EDGE_N:])

        mean_t = statistics.mean(temps)
        std_t = statistics.stdev(temps)          # sample standard deviation (ddof = 1)
        print("Mean temperature: %.4f C" % mean_t)
        print("Standard deviation: %.4f C   (%.2f LSB)   detrended: %.4f C"
              % (std_t, std_t / LSB_C, detr_std))
        print("Mean measured power: %.4f W" % statistics.mean(pw))
        print("Drift during collection: %+.3f C/min   (first %d mean %.4f C, last %d mean %.4f C, delta %+.4f C)"
              % (trend, DRIFT_EDGE_N, first_m, DRIFT_EDGE_N, last_m, last_m - first_m))
        summary.append([k, "%.3f" % p_target, "%.3f" % v_cmd, "%.4f" % statistics.mean(vs),
                        "%.5f" % statistics.mean(cs), "%.4f" % statistics.mean(pw),
                        "%.5f" % mean_t, "%.5f" % std_t, len(temps), "%.1f" % LEVEL_WAIT_S,
                        "%.1f" % actual_wait, "%.4f" % trend, "%.5f" % first_m, "%.5f" % last_m,
                        "%.5f" % (last_m - first_m), "%.5f" % detr_std])
        write_csv("m1a_summary.csv", header, summary)    # rewritten every level: survives an abort

    safe_off(ps)
    print("=" * 52)
    print("M1a sweep complete in %.0f s (%.0f s settling wait per level)."
          % (time.perf_counter() - t_start, LEVEL_WAIT_S))


def plot_m1a():
    rows = read_csv("m1a_summary.csv")
    if not rows:
        print("No results/m1a_summary.csv - skipping M1a plots.")
        return False
    p = np.array([float(r["mean_P_W"]) for r in rows])
    t = np.array([float(r["mean_T_C"]) for r in rows])
    s = np.array([float(r["std_T_C"]) for r in rows])

    fig, ax = plt.subplots()
    ax.errorbar(p, t, yerr=s, fmt="o", capsize=4, label="mean of 100 readings (bars: 1 std)")
    if len(p) >= 2:
        k, c = np.polyfit(p, t, 1)
        xx = np.linspace(0, p.max() * 1.05, 50)
        ax.plot(xx, k * xx + c, "--", label="linear fit: %.1f C/W" % k)
    ax.set_title("M1a: mean temperature vs applied power\n(%.0f s settling wait per level)" % LEVEL_WAIT_S)
    ax.set_xlabel("Measured supply output power, |V x I| on P6V [W]")
    ax.set_ylabel("Mean temperature [deg C]")
    ax.grid(True)
    ax.legend()
    save_fig(fig, "m1a_mean_temperature_vs_power.png")

    fig, ax = plt.subplots()
    ax.plot(t, s, "o-", label="measured (100 readings, 300 ms apart)")
    ax.axhline(LSB_C, color="gray", ls="--", label="1 LSB = %.5f C" % LSB_C)
    ax.axhline(LSB_C / math.sqrt(12), color="gray", ls=":", label="LSB / sqrt(12) = %.5f C" % (LSB_C / math.sqrt(12)))
    ax.set_title("M1a: temperature std vs mean temperature\n(%.0f s settling wait per level)" % LEVEL_WAIT_S)
    ax.set_xlabel("Mean temperature [deg C]")
    ax.set_ylabel("Standard deviation of temperature [deg C]")
    ax.set_ylim(bottom=0)
    ax.grid(True)
    ax.legend()
    save_fig(fig, "m1a_std_vs_mean_temperature.png")
    return True


# ---------------------------------------------------------------------------
# M1b: the averaging law
# ---------------------------------------------------------------------------
def fast_run(n):
    """n readings back to back, no sleep. One trigger per sample."""
    rows, temps = [], []
    print("Fast run: %d readings, no delay..." % n)
    t0 = time.perf_counter()
    for k in range(n):
        raw = read_temperature()
        t = time.perf_counter() - t0
        temp = to_celsius(raw)
        temps.append(temp)
        rows.append([k + 1, "%.6f" % t, raw, "0x%04X" % raw, "%.6f" % temp])
    elapsed = time.perf_counter() - t0
    write_csv("m1b_fast.csv", ["sample", "t_s", "raw", "raw_hex", "temp_C"], rows)
    print("%d readings completed in %.3f s" % (n, elapsed))
    print("%.1f readings/s" % (n / elapsed))
    return temps, elapsed


def slow_run(n=SLOW_N, interval=SLOW_INTERVAL_S):
    """n readings, one request every `interval` seconds."""
    rows, temps = [], []
    print("Slow run: %d readings, one every %.0f ms..." % (n, interval * 1000))
    t0 = time.perf_counter()
    for k in range(n):
        target = t0 + k * interval
        time.sleep(max(0.0, target - time.perf_counter()))
        raw = read_temperature()
        t = time.perf_counter() - t0
        temp = to_celsius(raw)
        temps.append(temp)
        rows.append([k + 1, "%.6f" % t, raw, "0x%04X" % raw, "%.6f" % temp])
    elapsed = time.perf_counter() - t0
    write_csv("m1b_slow300ms.csv", ["sample", "t_s", "raw", "raw_hex", "temp_C"], rows)
    print("%d readings completed in %.1f s" % (n, elapsed))
    return temps


def averaging_analysis(temps):
    """Std of the means of non-overlapping blocks of N samples, N = 1, 2, 4, ...

    Only N with at least MIN_BLOCKS complete blocks are used. Reference line:
    sigma_1 / sqrt(N). Departure: the first N >= 2 where the measured value is
    more than DEPARTURE_RATIO times the reference. This is a marker, not a
    physical constant: a different ratio moves it by a factor of 2 or so.
    """
    x = np.asarray(temps, dtype=float)
    ns, sig = [], []
    n = 1
    while len(x) // n >= MIN_BLOCKS:
        blocks = x[: (len(x) // n) * n].reshape(-1, n).mean(axis=1)
        ns.append(n)
        sig.append(np.std(blocks, ddof=1))
        n *= 2
    ns, sig = np.array(ns), np.array(sig)
    ref = sig[0] / np.sqrt(ns)
    departure = None
    if sig[0] > 0:
        for k in range(1, len(ns)):
            if sig[k] > DEPARTURE_RATIO * ref[k]:
                departure = int(ns[k])
                break
    return ns, sig, ref, departure


def plot_m1b():
    fast = read_csv("m1b_fast.csv")
    if not fast:
        print("No results/m1b_fast.csv - skipping M1b plots.")
        return False
    slow = read_csv("m1b_slow300ms.csv")
    tf = [float(r["temp_C"]) for r in fast]
    ns, sig, ref, departure = averaging_analysis(tf)

    fig, ax = plt.subplots()
    ax.loglog(ns, sig, "o-", label="measured: std of N-sample averages")
    ax.loglog(ns, ref, "--", label="ideal: sigma_1 / sqrt(N)")
    if departure is not None:
        ax.axvline(departure, color="red", ls=":", label="departs from 1/sqrt(N) at N = %d" % departure)
    ax.set_title("M1b: averaging law, %d fast readings" % len(tf))
    ax.set_xlabel("N, readings averaged")
    ax.set_ylabel("Std of averaged temperature [deg C]")
    ax.grid(True, which="both")
    ax.legend()
    save_fig(fig, "m1b_averaging.png")

    if slow:
        ts = [float(r["temp_C"]) for r in slow]
        fig, ax = plt.subplots()
        ax.plot([float(r["t_s"]) for r in fast[:len(ts)]], tf[:len(ts)], ".-",
                label="first %d fast readings" % len(ts))
        ax.plot([float(r["t_s"]) for r in slow], ts, ".-", label="%d readings, 300 ms apart" % len(ts))
        ax.set_xscale("symlog", linthresh=0.1)
        ax.set_title("M1b: fast vs 300 ms readings")
        ax.set_xlabel("Time since first reading [s]")
        ax.set_ylabel("Temperature [deg C]")
        ax.grid(True)
        ax.legend()
        save_fig(fig, "m1b_traces.png")
    return True


def report_m1b(fast_temps, slow_temps):
    ns, sig, ref, departure = averaging_analysis(fast_temps)
    print("")
    print("    N   std of averages [C]   ideal [C]   ratio")
    for n, s, r in zip(ns, sig, ref):
        print("  %4d     %.6f          %.6f   %5.2f" % (n, s, r, s / r if r > 0 else float("nan")))
    rows = [[int(n), "%.7f" % s, "%.7f" % r] for n, s, r in zip(ns, sig, ref)]
    write_csv("m1b_averaging.csv", ["N", "std_of_averages_C", "ideal_C"], rows)
    if sig[0] == 0:
        print("WARNING: sigma_1 is exactly zero - every reading identical. 13-bit mode?")
    if departure is not None:
        print("Averaging-law departure: N = %d   (first N where measured > %.1f x ideal)"
              % (departure, DEPARTURE_RATIO))
    else:
        print("Averaging-law departure: not found up to N = %d" % ns[-1])

    f100 = fast_temps[:SLOW_N]
    s_fast = statistics.stdev(f100)                 # sample standard deviation (ddof = 1)
    s_slow = statistics.stdev(slow_temps)
    print("")
    print("First %d fast readings stdev:    %.5f C   (%d distinct codes)"
          % (len(f100), s_fast, len(set(f100))))
    print("%d readings at 300 ms stdev:     %.5f C   (%d distinct codes)"
          % (len(slow_temps), s_slow, len(set(slow_temps))))
    write_csv("m1b_summary.csv", ["quantity", "value"], [
        ["fast_n", len(fast_temps)],
        ["fast_std_first100_C", "%.6f" % s_fast],
        ["slow300ms_std_C", "%.6f" % s_slow],
        ["departure_N", departure if departure is not None else ""],
    ])


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------
def mode_sensor_test(args):
    open_fpga(not args.no_bitfile)
    try:
        sensor_self_test()
    finally:
        close_fpga()


def mode_m1b(args):
    if args.fast_n < 1000:
        raise ValueError("--fast-n must be at least 1000")
    print("Heater must be OFF and the bench steady. This mode never touches the supply.")
    open_fpga(not args.no_bitfile)
    try:
        sensor_self_test()
        t_wall = time.strftime("%H:%M:%S")
        fast, elapsed = fast_run(args.fast_n)
        slow = slow_run()
    finally:
        close_fpga()
    write_csv("m1b_fast_runinfo.csv", ["quantity", "value"],
              [["start_time", t_wall], ["n", len(fast)], ["elapsed_s", "%.4f" % elapsed],
               ["readings_per_s", "%.1f" % (len(fast) / elapsed)]])
    report_m1b(fast, slow)
    plot_m1b()


def with_supply(body):
    """Connect to the E3631A, run body(ps), and ALWAYS switch the output off."""
    print("Looking for instruments:")
    rm, ps = find_instruments()
    try:
        setup_supply(ps)
        body(ps)
    except PowerCutoff:
        print("Sweep aborted.")
        raise
    finally:
        safe_off(ps)
        try:
            ps.close()
            rm.close()
        except Exception:
            pass


def mode_supply_test(args):
    with_supply(sanity_check)
    print("Supply test complete.")


def mode_demo_cutoff(args):
    def body(ps):
        print("*** DEMO threshold %.3f W in use (real cutoff is %.3f W) ***" % (DEMO_CUTOFF_W, POWER_CUTOFF_W))
        set_voltage_checked(ps, 0.0, DEMO_CUTOFF_W)
        output_on(ps)
        try:
            for v in DEMO_VOLTAGES:
                vm, im, pm = set_voltage_checked(ps, v, DEMO_CUTOFF_W)
                print("  %.1f V commanded: V = %.4f V, I = %.5f A, P = %.4f W" % (v, vm, im, pm))
        except PowerCutoff:
            print("Software cutoff demonstration: PASS")
            return
        raise RuntimeError("cutoff never tripped up to %.1f V - check the wiring" % DEMO_VOLTAGES[-1])
    with_supply(body)


def mode_m1a(args):
    open_fpga(not args.no_bitfile)
    try:
        sensor_self_test()               # sensor must be good before any heater power
        def body(ps):
            sanity_check(ps)
            run_m1a(ps)
        with_supply(body)
    finally:
        close_fpga()
    plot_m1a()


def mode_plot_only(args):
    a = plot_m1a()
    b = plot_m1b()
    if b:
        fast = [float(r["temp_C"]) for r in read_csv("m1b_fast.csv")]
        slow = read_csv("m1b_slow300ms.csv")
        if slow:
            report_m1b(fast, [float(r["temp_C"]) for r in slow])
    if not (a or b):
        print("Nothing to plot yet.")


def main():
    parser = argparse.ArgumentParser(
        description="ECE 437 Lab 7 Milestone 1. Choose exactly one mode; with none, nothing runs.")
    g = parser.add_mutually_exclusive_group()
    g.add_argument("--sensor-test", action="store_true", help="FPGA + ADT7420 checks only")
    g.add_argument("--m1b", action="store_true", help="averaging law (heater OFF, no supply)")
    g.add_argument("--supply-test", action="store_true", help="P6V safety setup + 2 V known-resistor check")
    g.add_argument("--demo-cutoff", action="store_true", help="trip the software cutoff at a DEMO threshold")
    g.add_argument("--m1a", action="store_true", help="five-level heating sweep")
    g.add_argument("--plot-only", action="store_true", help="redraw plots from saved CSVs")
    parser.add_argument("--fast-n", type=int, default=FAST_N_DEFAULT, help="M1b fast-run length (>= 1000)")
    parser.add_argument("--no-bitfile", action="store_true", help="do not load the bitfile (JTAG/ILA use)")
    parser.add_argument("--show", action="store_true", help="open the saved plots at the end")
    args = parser.parse_args()

    modes = [("sensor_test", mode_sensor_test), ("m1b", mode_m1b), ("supply_test", mode_supply_test),
             ("demo_cutoff", mode_demo_cutoff), ("m1a", mode_m1a), ("plot_only", mode_plot_only)]
    chosen = [fn for name, fn in modes if getattr(args, name)]
    if not chosen:
        parser.print_help()
        return 0

    try:
        chosen[0](args)
    except PowerCutoff:
        return 2
    except KeyboardInterrupt:
        print("\nInterrupted by user. (If the supply was in use, its output was switched off on the way out.)")
        return 1
    except Exception as e:
        print("FAIL:", e)
        return 1

    if args.show:
        for name in sorted(os.listdir(RESULTS)) if os.path.isdir(RESULTS) else []:
            if name.endswith(".png"):
                os.startfile(os.path.join(RESULTS, name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
