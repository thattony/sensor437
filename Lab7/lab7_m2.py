# -*- coding: utf-8 -*-
"""
ECE 437 - Lab 7 Milestone 2: the instrument's floor, and the bus's.

    python lab7_m2.py --m2a            34461A NPLC sweep: 200 readings per setting on one quiet node
    python lab7_m2.py --m2b-short      scope capture of the node, SHORT ground (ground spring)
    python lab7_m2.py --m2b-long       scope capture of the same node, LONG ground wire (big loop)
    python lab7_m2.py --m2b-analyze    FFT both captures, compare 60 Hz amplitudes (no hardware)
    python lab7_m2.py --scope-check    capture a 3.3 V logic line, check volts scaling
    python lab7_m2.py --m2c            I2C SDA edge through the ACK: 10-90 % rise/fall, RC, bus C
    python lab7_m2.py --plot-only      redraw every M2 plot from saved files (no hardware)

With no mode the script prints this help and does nothing.
Loop areas for the comparison:  --short-area-cm2 A  --long-area-cm2 B
M2c wiring: CH1 tip on SDA_0 (TP10), ground on the nearest board GND; CH2 tip on PMOD J1
pin 1 (marker), ground on J1 pin 5. 10x probes. Needs the bitstream with the PMOD marker.
"""
import sys
import os
import time
import csv
import argparse
import statistics
import numpy as np
import matplotlib
matplotlib.use("Agg")                    # save PNGs; --show opens them at the end
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")

LINE_HZ = 60.0                # one power line cycle = 16.7 ms

# M2a: 34461A DC volts
DMM_MODEL = "34461A"
M2A_N = 200                   # readings per NPLC setting
# Candidate NPLC values. The script keeps only those inside the meter's
# NPLC MIN..MAX, then keeps whatever the meter actually accepts (it rounds to
# its own steps; a 34461A supports 0.02, 0.2, 1, 10, 100).
NPLC_CANDIDATES = [0.001, 0.002, 0.006, 0.02, 0.06, 0.2, 1, 10, 100]
DMM_POLL_S = 1.0              # progress poll interval while a setting is running

# M2b: MSO-X 3024T
SCOPE_MODEL = "MSO-X 3024T"
SCOPE_CHANNEL = 1
SCOPE_VDIV = 0.020            # V/div, AC coupled; same for both captures
SCOPE_TIME_RANGE_S = 1.0      # full-screen time = record length; 60 cycles, 1 Hz FFT bins
SCOPE_COUPLING = "AC"         # node is DC; AC coupling lets a small V/div show the 60 Hz
F_LINE_BAND_HZ = 5.0          # search 55..65 Hz for the line peak
F_PLOT_MAX_HZ = 1000.0        # FFT plot range (shows the 120/180 Hz harmonics too)

# --scope-check: a 3.3 V logic line, DC coupled
CHECK_VDIV = 1.0
CHECK_OFFSET_V = 2.0
CHECK_TIME_RANGE_S = 0.001
LOGIC_HIGH_V = 3.3

# M2c: I2C SDA edge through the read-address ACK
# FPGA (same bitstream and endpoints as lab7_python.py / lab7_m1.py)
BITFILE = os.path.join(HERE, "Lab7.runs", "impl_1", "JTAG_Test_File.bit")
OK_PY_DIR = r"C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\x64"
OK_DLL_DIR = r"C:\Program Files\Opal Kelly\FrontPanelUSB\API\lib\x64"
WIRE_GO = 0x00          # WireIn: rising edge on bit 0 = one temperature read
WIRE_STATUS = 0x24      # WireOut: bit 0 init_done, bit 1 init_ack_error, bit 2 temp_ack_error
WIRE_COUNT = 0x25       # WireOut: number of completed temperature reads (16 bit)
# The marker (PMOD J1 pin 1) is high while State = 106..110 of every temperature read.
# Each state's assignments appear at the clock edge that LEAVES the state, so with
# Ts = one FSM state (~2.48 us) and t = 0 at the marker's rising edge:
#   t = 1 Ts  SDA released for the last address bit (0x91 bit 0 = 1): pull-up charges SDA (rise)
#   t = 2-4 Ts SCL high (bit 0 clocked);  t = 4 Ts SCL falls: the sensor pulls SDA low (ACK, fall)
#   t = 5 Ts  marker falls; the ACK clock follows.
M2C_SDA_CH = 1          # SDA_0 test point (TP10), ground on the nearest board GND
M2C_MARK_CH = 2         # PMOD J1 pin 1, ground on J1 pin 5
M2C_SDA_VDIV = 0.5      # V/div; offset 1.65 V puts 0..3.3 V across most of the screen
M2C_MARK_VDIV = 1.0
M2C_OFFSET_V = 1.65
M2C_TRIG_LEVEL_V = 1.65
M2C_TIME_RANGE_S = 20e-6    # full screen: about 2 us before the marker to 18 us after
M2C_TIME_POS_S = 8e-6       # screen centre relative to the trigger
M2C_N_DEFAULT = 10      # captures; the edge times are averaged over these
R_PULLUP_OHM = 10.0e3   # SDA_0 pull-up R6, 10.00 kohm (sensor board schematic)
I2C_TR_MAX_S = {"standard mode (100 kHz)": 1000e-9, "fast mode (400 kHz)": 300e-9}


# ---------------------------------------------------------------------------
# Instruments
# ---------------------------------------------------------------------------
def find_instruments(models):
    """Open every USB VISA resource, keep the ones whose *IDN? names a wanted model.

    Only USB resources are probed, so the E3631A on its serial port is never touched.
    Returns (resource_manager, {model: instrument}).
    """
    import pyvisa as visa
    rm = visa.ResourceManager()
    found = {}
    for res in rm.list_resources():
        if not res.startswith("USB"):
            continue
        inst = None
        try:
            inst = rm.open_resource(res)
            inst.timeout = 5000
            idn = inst.query("*IDN?").strip()
            print("  %s : %s" % (res, idn))
            for m in models:
                if m in idn and m not in found:
                    found[m], inst = inst, None      # keep this one open
                    break
        except Exception:
            print("  %s : no response" % res)
        finally:
            if inst is not None:
                inst.close()
    missing = [m for m in models if m not in found]
    if missing:
        for inst in found.values():
            inst.close()
        rm.close()
        raise RuntimeError("not found: %s. Is it on and connected by USB?" % ", ".join(missing))
    return rm, found


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


def safe_close(rm, insts):
    """Leave the instruments usable from the front panel and release VISA. Never raises."""
    for model, inst in insts.items():
        try:
            if model == DMM_MODEL:
                inst.write("ABORt")              # stop any measurement still running
                inst.write("SYSTem:LOCal")       # front panel back to the user
            elif model == SCOPE_MODEL:
                inst.write(":RUN")               # :DIGitize leaves the scope stopped
        except Exception as e:
            print("WARNING: could not reset %s (%s)" % (model, e))
        try:
            inst.close()
        except Exception:
            pass
    try:
        rm.close()
    except Exception:
        pass


def with_instruments(models, body):
    print("Looking for instruments:")
    rm, insts = find_instruments(models)
    try:
        return body(insts)
    finally:
        safe_close(rm, insts)


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
def results_path(name):
    os.makedirs(RESULTS, exist_ok=True)
    return os.path.join(RESULTS, name)


def write_csv(name, header, rows):
    path = results_path(name)
    with open(path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(header)
        wr.writerows(rows)
    print("Saved", path)
    return path


def read_csv(name):
    path = os.path.join(RESULTS, name)
    if not os.path.exists(path):
        return None
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def save_fig(fig, name):
    path = results_path(name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("Saved", path)


def wait_for_user(lines):
    print("")
    for line in lines:
        print(line)
    input("Press Enter when ready (Ctrl+C to cancel)...")
    print("")


# ---------------------------------------------------------------------------
# M2a: DMM integration time
# ---------------------------------------------------------------------------
def dmm_setup(dmm, args):
    """DC volts, fixed range, 10 Mohm input, sample count M2A_N. Returns the range used."""
    print("DMM: %s" % DMM_MODEL)
    terminals = scpi_query(dmm, "ROUTe:TERMinals?")
    print("Input terminals in use: %s panel" % ("FRONT" if terminals.startswith("FRON") else "REAR"))
    scpi_write(dmm, "*CLS")
    scpi_write(dmm, "CONFigure:VOLTage:DC AUTO")
    scpi_write(dmm, "VOLTage:DC:IMPedance:AUTO OFF")       # fixed 10 Mohm on every range
    print("Function: DC voltage")

    level = float(scpi_query(dmm, "READ?"))                # one autoranged look at the node
    if abs(level) > 1e30:
        raise RuntimeError("DMM reads overload - check the leads are on V HI / LO")
    if args.range is not None:
        rng = args.range
    else:
        rng = float(scpi_query(dmm, "VOLTage:DC:RANGe?"))  # the range autorange picked
    scpi_write(dmm, "VOLTage:DC:RANGe %g" % rng)          # fixed from here on (autorange off)
    if scpi_query(dmm, "VOLTage:DC:RANGe:AUTO?") not in ("0", "OFF"):
        raise RuntimeError("autorange did not switch off")
    scpi_write(dmm, "VOLTage:DC:ZERO:AUTO %s" % ("OFF" if args.autozero_off else "ON"))
    scpi_write(dmm, "TRIGger:SOURce IMMediate")
    scpi_write(dmm, "TRIGger:COUNt 1")
    scpi_write(dmm, "SAMPle:COUNt %d" % M2A_N)
    print("Node level: %.6f V   range fixed at %g V   autozero %s"
          % (level, rng, "OFF" if args.autozero_off else "ON"))
    return level, rng


def nplc_plan(dmm, args):
    """NPLC values to sweep: the candidates inside the meter's MIN..MAX (or --nplc)."""
    lo = float(scpi_query(dmm, "VOLTage:DC:NPLC? MIN"))
    hi = float(scpi_query(dmm, "VOLTage:DC:NPLC? MAX"))
    wanted = args.nplc if args.nplc else NPLC_CANDIDATES
    plan = [n for n in wanted if lo * 0.999 <= n <= hi * 1.001]
    print("Meter NPLC range: %g to %g" % (lo, hi))
    return plan


def take_readings(dmm, nplc, autozero):
    """INITiate M2A_N readings, poll progress, FETCh them. Timeout from the integration time."""
    per = nplc / LINE_HZ * (2.0 if autozero else 1.0)     # autozero doubles each reading
    deadline_s = M2A_N * per * 1.5 + 30.0
    scpi_write(dmm, "INITiate")
    t0 = time.perf_counter()
    last = -1
    while True:
        n = int(float(scpi_query(dmm, "DATA:POINts?")))
        if n >= M2A_N:
            break
        if per * M2A_N > 10 and n // 20 != last // 20:     # progress for the slow settings
            print("    %3d / %d readings" % (n, M2A_N))
        last = n
        if time.perf_counter() - t0 > deadline_s:
            raise RuntimeError("NPLC %g: only %d of %d readings after %.0f s" % (nplc, n, M2A_N, deadline_s))
        time.sleep(DMM_POLL_S if per * M2A_N > 2 else 0.1)
    elapsed = time.perf_counter() - t0
    vals = [float(x) for x in scpi_query(dmm, "FETCh?").split(",")]
    if len(vals) != M2A_N:
        raise RuntimeError("NPLC %g: fetched %d readings, expected %d" % (nplc, len(vals), M2A_N))
    if any(abs(v) > 1e30 for v in vals):
        raise RuntimeError("NPLC %g: overload reading - range too small" % nplc)
    return vals, elapsed


def run_m2a(insts, args):
    dmm = insts[DMM_MODEL]
    level, rng = dmm_setup(dmm, args)
    plan = nplc_plan(dmm, args)
    az = not args.autozero_off
    est = sum(M2A_N * n / LINE_HZ * (2 if az else 1) for n in plan)
    print("Planned NPLC: %s   (estimated %.0f s of integration, about %.1f min)"
          % (", ".join("%g" % n for n in plan), est, est / 60))

    raw_rows, summary, done = [], [], set()
    for req in plan:
        scpi_write(dmm, "VOLTage:DC:NPLC %g" % req)
        nplc = float(scpi_query(dmm, "VOLTage:DC:NPLC?"))   # what the meter really uses
        if nplc in done:
            print("NPLC %g requested -> meter uses %g (already measured), skipped" % (req, nplc))
            continue
        done.add(nplc)
        t_int = nplc / LINE_HZ
        print("")
        print("NPLC = %g" % nplc)
        print("Integration time = %.6f s" % t_int)
        vals, elapsed = take_readings(dmm, nplc, az)
        sd = statistics.stdev(vals)                          # sample standard deviation (ddof = 1)
        print("%d readings collected in %.1f s" % (len(vals), elapsed))
        print("Mean = %.7f V   Std dev = %.3e V" % (statistics.mean(vals), sd))
        raw_rows += [["%g" % nplc, k + 1, "%.9e" % v] for k, v in enumerate(vals)]
        summary.append(["%g" % nplc, "%.6e" % t_int, len(vals), "%.9e" % statistics.mean(vals),
                        "%.6e" % sd, "%.6e" % (max(vals) - min(vals)), "%.2f" % elapsed,
                        "%g" % rng, "ON" if az else "OFF"])
        # rewritten after every setting, so an interrupted sweep keeps what it measured
        write_csv("m2a_nplc_raw.csv", ["nplc", "reading", "volts"], raw_rows)
        write_csv("m2a_nplc_summary.csv", ["nplc", "t_int_s", "n", "mean_V", "std_V", "range_V",
                                           "elapsed_s", "dmm_range_V", "autozero"], summary)
    plot_m2a()


def plot_m2a():
    rows = read_csv("m2a_nplc_summary.csv")
    if not rows:
        print("No results/m2a_nplc_summary.csv - skipping M2a plot.")
        return False
    t = np.array([float(r["t_int_s"]) for r in rows])
    sd = np.array([float(r["std_V"]) for r in rows])
    nplc = [r["nplc"] for r in rows]
    fig, ax = plt.subplots()
    ax.loglog(t, sd, "o-", label="std of %d readings" % M2A_N)
    for x, y, n in zip(t, sd, nplc):
        ax.annotate("NPLC %s" % n, (x, y), textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax.axvline(1.0 / LINE_HZ, color="red", ls=":", label="1 PLC = %.1f ms" % (1000 / LINE_HZ))
    ax.set_title("M2a: 34461A noise vs integration time (range %s V, autozero %s)"
                 % (rows[0]["dmm_range_V"], rows[0]["autozero"]))
    ax.set_xlabel("Integration time, NPLC / %.0f Hz [s]" % LINE_HZ)
    ax.set_ylabel("Standard deviation of readings [V]")
    ax.grid(True, which="both")
    ax.legend()
    save_fig(fig, "m2a_nplc.png")

    # The raw readings at each setting: 60 Hz pickup is visible at short integration times
    raw = read_csv("m2a_nplc_raw.csv")
    if raw:
        fig, axs = plt.subplots(len(rows), 1, sharex=True, figsize=(7, 1.6 * len(rows) + 1))
        axs = np.atleast_1d(axs)
        for ax, r in zip(axs, rows):
            v = np.array([float(x["volts"]) for x in raw if x["nplc"] == r["nplc"]]) * 1e6
            ax.plot(np.arange(1, len(v) + 1), v, ".-", lw=0.6, ms=2)
            ax.set_ylabel("uV")
            ax.set_title("NPLC %s (%.3g ms integration): std %.1f uV"
                         % (r["nplc"], float(r["t_int_s"]) * 1000, float(r["std_V"]) * 1e6), fontsize=9)
            ax.grid(True)
        axs[-1].set_xlabel("Reading number")
        fig.suptitle("M2a: raw DMM readings at each NPLC (DC volts, range %s V)" % rows[0]["dmm_range_V"])
        fig.tight_layout()
        save_fig(fig, "m2a_raw_readings.png")
    return True


# ---------------------------------------------------------------------------
# M2b: scope capture
# ---------------------------------------------------------------------------
def parse_preamble(text):
    """:WAVeform:PREamble? -> dict. Ten fields, InfiniiVision order."""
    f = [float(x) for x in text.split(",")]
    keys = ["format", "type", "points", "count", "xincrement", "xorigin", "xreference",
            "yincrement", "yorigin", "yreference"]
    return dict(zip(keys, f))


def codes_to_wave(codes, pre):
    """Raw codes -> (time [s], volts [V]) using the preamble.

    time  = (i    - xreference) * xincrement + xorigin
    volts = (code - yreference) * yincrement + yorigin
    yincrement / yorigin are already in volts at the probe tip, i.e. they
    include the channel's probe attenuation setting.
    """
    codes = np.asarray(codes, dtype=float)
    i = np.arange(len(codes), dtype=float)
    t = (i - pre["xreference"]) * pre["xincrement"] + pre["xorigin"]
    v = (codes - pre["yreference"]) * pre["yincrement"] + pre["yorigin"]
    return t, v


def scope_configure(scope, vdiv, offset, coupling, time_range, ch):
    """Identical front-end setup for every capture. NORMAL acquisition, never AVERAGE."""
    c = ":CHANnel%d" % ch
    scpi_write(scope, "*CLS")
    scpi_write(scope, ":STOP")
    for other in range(1, 5):                # one channel only: it gets all the memory
        scpi_write(scope, ":CHANnel%d:DISPlay %s" % (other, "ON" if other == ch else "OFF"))
    scpi_write(scope, c + ":COUPling " + coupling)
    scpi_write(scope, c + ":SCALe %g" % vdiv)
    scpi_write(scope, c + ":OFFSet %g" % offset)
    scpi_write(scope, c + ":BWLimit ON")         # 20 MHz limit: irrelevant at 60 Hz, cuts HF noise
    scpi_write(scope, ":TIMebase:MODE MAIN")
    scpi_write(scope, ":TIMebase:RANGe %g" % time_range)
    scpi_write(scope, ":TIMebase:REFerence CENTer")
    scpi_write(scope, ":TIMebase:POSition 0")
    scpi_write(scope, ":ACQuire:TYPE NORMal")    # averaging would suppress the pickup we want
    scpi_write(scope, ":TRIGger:SWEep AUTO")     # a quiet node may never cross a trigger level
    acq = scpi_query(scope, ":ACQuire:TYPE?")
    if not acq.startswith("NORM"):
        raise RuntimeError("acquisition type is %s, expected NORMal" % acq)
    probe = float(scpi_query(scope, c + ":PROBe?"))
    return {"channel": ch, "vdiv": float(scpi_query(scope, c + ":SCALe?")),
            "offset": float(scpi_query(scope, c + ":OFFSet?")),
            "coupling": scpi_query(scope, c + ":COUPling?"),
            "time_range": float(scpi_query(scope, ":TIMebase:RANGe?")),
            "probe": probe, "acquire": acq}


def scope_capture(scope, ch):
    """One :DIGitize, then RAW, maximum points, 16-bit words. Returns (t, v, preamble, srate, codes)."""
    old = scope.timeout
    scope.timeout = 60000                        # digitize of a 1 s record + auto trigger
    try:
        scpi_write(scope, ":DIGitize CHANnel%d" % ch)
        scpi_query(scope, "*OPC?")
    finally:
        scope.timeout = old
    return scope_fetch(scope, ch)


def scope_fetch(scope, ch):
    """Transfer the stopped acquisition of one channel: RAW, maximum points, 16-bit words.

    Returns (t, v, preamble, srate, codes).
    """
    old = scope.timeout
    try:
        srate = float(scpi_query(scope, ":ACQuire:SRATe?"))
        scpi_write(scope, ":WAVeform:SOURce CHANnel%d" % ch)
        scpi_write(scope, ":WAVeform:FORMat WORD")
        scpi_write(scope, ":WAVeform:BYTeorder LSBFirst")
        scpi_write(scope, ":WAVeform:UNSigned ON")
        scpi_write(scope, ":WAVeform:POINts:MODE RAW")
        scpi_write(scope, ":WAVeform:POINts MAXimum")
        pre = parse_preamble(scpi_query(scope, ":WAVeform:PREamble?"))   # after all settings
        if int(pre["format"]) != 1:
            raise RuntimeError("waveform format code %d, expected 1 (WORD)" % pre["format"])
        scope.timeout = 120000                   # several MB over USB
        scope.chunk_size = 1024 * 1024
        codes = scope.query_binary_values(":WAVeform:DATA?", datatype="H",
                                          is_big_endian=False, container=np.array)
        errors = drain_errors(scope)
        if errors:
            raise RuntimeError("SCPI error after waveform transfer: " + "; ".join(errors))
    finally:
        scope.timeout = old
    if len(codes) != int(pre["points"]):
        raise RuntimeError("got %d points, preamble says %d" % (len(codes), int(pre["points"])))
    t, v = codes_to_wave(codes, pre)
    return t, v, pre, srate, codes


def save_wave(name, t, v, pre, settings, srate, idn, kind):
    path = results_path(name)
    np.savez_compressed(path, t=t, v=v, preamble=np.array(list(pre.values())),
                        preamble_keys=np.array(list(pre.keys())),
                        settings_keys=np.array(list(settings.keys())),
                        settings_vals=np.array([str(x) for x in settings.values()]),
                        srate=srate, idn=idn, kind=kind, when=time.strftime("%Y-%m-%d %H:%M:%S"))
    print("Saved", path)


def load_wave(name):
    path = os.path.join(RESULTS, name)
    if not os.path.exists(path):
        return None
    d = np.load(path, allow_pickle=False)
    settings = dict(zip(d["settings_keys"].tolist(), d["settings_vals"].tolist()))
    return {"t": d["t"], "v": d["v"], "settings": settings, "srate": float(d["srate"]),
            "kind": str(d["kind"]), "when": str(d["when"])}


def check_clipping(v, settings):
    """The screen spans offset +/- 4 div; samples pinned at either edge are clipped."""
    vdiv, off = float(settings["vdiv"]), float(settings["offset"])
    edge = 0.02 * vdiv
    clipped = np.mean((v >= off + 4 * vdiv - edge) | (v <= off - 4 * vdiv + edge))
    if clipped > 0.001:
        print("WARNING: %.1f %% of samples at the screen edge - signal clipped, raise --vdiv"
              % (100 * clipped))


def run_m2b_capture(insts, args, kind):
    scope = insts[SCOPE_MODEL]
    idn = scpi_query(scope, "*IDN?")
    print("Scope: %s" % SCOPE_MODEL)
    print("Capture type: %s LOOP" % kind.upper())
    settings = scope_configure(scope, args.vdiv, 0.0, SCOPE_COUPLING, args.time_range, args.channel)
    print("CH%d: %s coupled, %g V/div, probe %gx, record %g s, acquisition %s"
          % (settings["channel"], settings["coupling"], settings["vdiv"], settings["probe"],
             settings["time_range"], settings["acquire"]))
    if kind == "short":
        wiring = ["Connect the probe tip to the fixed quiet node.",
                  "Use the SHORT ground connection: the ground spring on the nearest ground.",
                  "Keep the probe and its cable where they are for the whole capture."]
    else:
        wiring = ["Probe tip on the SAME quiet node as the short-loop capture.",
                  "Remove the ground spring. Run a LONG wire from the probe ground to the",
                  "power supply's ground terminal, laid out to enclose the largest loop you can.",
                  "Measure/photograph the loop so you can estimate its area."]
    print("Waiting for user wiring confirmation...")
    wait_for_user(wiring)
    t, v, pre, srate, codes = scope_capture(scope, args.channel)
    dur = t[-1] - t[0] + pre["xincrement"]
    print("Waveform captured")
    print("Samples: %d   (%.3g Sa/s, %g s per sample)" % (len(v), 1 / pre["xincrement"], pre["xincrement"]))
    print("Duration: %.4f s" % dur)
    print("Mean %.3e V   Vpp %.3e V   RMS (AC) %.3e V" % (v.mean(), np.ptp(v), np.std(v)))
    if dur < 0.5:
        print("WARNING: record shorter than 0.5 s - 60 Hz resolution will be coarse")
    check_clipping(v, settings)
    f, amp = amplitude_spectrum(v, pre["xincrement"])
    f60, a60 = line_amplitude(f, amp)
    print("Quick look: %.3e V peak at %.2f Hz" % (a60, f60))
    save_wave("m2b_%s_wave.npz" % kind, t, v, pre, settings, srate, idn, kind)


def run_scope_check(insts, args):
    scope = insts[SCOPE_MODEL]
    idn = scpi_query(scope, "*IDN?")
    print("Scope: %s   (volts-scaling check)" % SCOPE_MODEL)
    settings = scope_configure(scope, CHECK_VDIV, CHECK_OFFSET_V, "DC", CHECK_TIME_RANGE_S, args.channel)
    print("CH%d: DC coupled, %g V/div, offset %g V, probe %gx"
          % (settings["channel"], settings["vdiv"], settings["offset"], settings["probe"]))
    wait_for_user(["Probe a known 3.3 V logic signal (e.g. a 3.3 V supply pin or a PMOD 3V3 pin),",
                   "with the probe ground on the board ground."])
    t, v, pre, srate, codes = scope_capture(scope, args.channel)
    high = np.percentile(v, 95)
    print("Samples %d   min %.3f V   max %.3f V   95th percentile %.3f V" % (len(v), v.min(), v.max(), high))
    ok = abs(high - LOGIC_HIGH_V) < 0.25
    print("3.3 V logic level reads %.3f V: %s" % (high, "PASS" if ok else
          "CHECK - wrong probe attenuation setting, or not a 3.3 V signal"))
    save_wave("m2b_scope_check.npz", t, v, pre, settings, srate, idn, "check")


# ---------------------------------------------------------------------------
# M2c: FPGA access (self-contained copy of the lab7_m1.py helpers)
# ---------------------------------------------------------------------------
class Fpga:
    """Opal Kelly board running the Lab 7 I2C bitstream. One trigger = one temperature read."""

    def __init__(self, load_bitfile=True):
        sys.path.append(OK_PY_DIR)
        os.add_dll_directory(OK_DLL_DIR)
        import ok
        self.dev = ok.okCFrontPanel()
        if self.dev.OpenBySerial("") != 0:
            self.dev = None
            raise RuntimeError("No Opal Kelly board found. Is FrontPanel still open? Close it.")
        if load_bitfile:
            if self.dev.ConfigureFPGA(BITFILE) != 0:
                raise RuntimeError("Bitfile did not load: " + BITFILE)
            print("Bitfile loaded:", BITFILE)
        if not self.dev.IsFrontPanelEnabled():
            raise RuntimeError("FrontPanel is not enabled in the FPGA design.")
        self.dev.SetWireInValue(WIRE_GO, 0)
        self.dev.UpdateWireIns()
        t0 = time.time()
        while not (self.status() & 0x1):         # init_done: 16-bit config write + ID read finished
            if time.time() - t0 > 1.0:
                raise RuntimeError("ADT7420 initialization did not complete (init_done never set)")
        if self.status() & 0x2:
            raise RuntimeError("sensor did not ACK during the power-up sequence")
        print("FPGA communication: PASS   ADT7420 initialization: PASS")

    def status(self):
        self.dev.UpdateWireOuts()
        return self.dev.GetWireOutValue(WIRE_STATUS)

    def count(self):
        self.dev.UpdateWireOuts()
        return self.dev.GetWireOutValue(WIRE_COUNT) & 0xFFFF

    def read_once(self, timeout=0.5):
        """Rising edge on GO -> exactly one I2C temperature read (one marker pulse)."""
        before = self.count()
        self.dev.SetWireInValue(WIRE_GO, 1)
        self.dev.UpdateWireIns()
        self.dev.SetWireInValue(WIRE_GO, 0)
        self.dev.UpdateWireIns()
        t0 = time.time()
        while self.count() == before:
            if time.time() - t0 > timeout:
                raise RuntimeError("no I2C transaction completed within %.1f s of the trigger" % timeout)
        if self.status() & 0x4:
            raise RuntimeError("sensor did not ACK during the temperature read")

    def close(self):
        if self.dev is not None:
            self.dev.Close()


# ---------------------------------------------------------------------------
# M2c: SDA edge capture through the ACK
# ---------------------------------------------------------------------------
def m2c_configure(scope):
    """CH1 = SDA, CH2 = marker, DC coupled, full bandwidth, NORMAL trigger on the marker's rising edge."""
    scpi_write(scope, "*CLS")
    scpi_write(scope, ":STOP")
    for ch in range(1, 5):
        scpi_write(scope, ":CHANnel%d:DISPlay %s" % (ch, "ON" if ch in (M2C_SDA_CH, M2C_MARK_CH) else "OFF"))
    for ch, vdiv in ((M2C_SDA_CH, M2C_SDA_VDIV), (M2C_MARK_CH, M2C_MARK_VDIV)):
        c = ":CHANnel%d" % ch
        scpi_write(scope, c + ":COUPling DC")
        scpi_write(scope, c + ":BWLimit OFF")        # a 20 MHz limit would slow the fast fall
        scpi_write(scope, c + ":SCALe %g" % vdiv)
        scpi_write(scope, c + ":OFFSet %g" % M2C_OFFSET_V)
    scpi_write(scope, ":TIMebase:MODE MAIN")
    scpi_write(scope, ":TIMebase:RANGe %g" % M2C_TIME_RANGE_S)
    scpi_write(scope, ":TIMebase:REFerence CENTer")
    scpi_write(scope, ":TIMebase:POSition %g" % M2C_TIME_POS_S)
    scpi_write(scope, ":ACQuire:TYPE NORMal")
    scpi_write(scope, ":TRIGger:MODE EDGE")
    scpi_write(scope, ":TRIGger:EDGE:SOURce CHANnel%d" % M2C_MARK_CH)
    scpi_write(scope, ":TRIGger:EDGE:SLOPe POSitive")
    scpi_write(scope, ":TRIGger:EDGE:LEVel %g" % M2C_TRIG_LEVEL_V)
    scpi_write(scope, ":TRIGger:SWEep NORMal")       # wait for the marker; never auto-trigger
    probes = {ch: float(scpi_query(scope, ":CHANnel%d:PROBe?" % ch)) for ch in (M2C_SDA_CH, M2C_MARK_CH)}
    return {"sda_vdiv": float(scpi_query(scope, ":CHANnel%d:SCALe?" % M2C_SDA_CH)),
            "time_range": float(scpi_query(scope, ":TIMebase:RANGe?")),
            "acquire": scpi_query(scope, ":ACQuire:TYPE?"),
            "probe_sda": probes[M2C_SDA_CH], "probe_marker": probes[M2C_MARK_CH]}


def m2c_acquire(scope, fpga, max_reads=50):
    """Arm a single acquisition, then trigger I2C reads until the scope has triggered."""
    scpi_query(scope, ":TER?")                       # clear the trigger event register
    scpi_write(scope, ":SINGle")
    time.sleep(0.05)                                 # let the acquisition arm
    for _ in range(max_reads):
        fpga.read_once()
        if int(scpi_query(scope, ":TER?")) == 1:
            break
        time.sleep(0.01)
    else:
        raise RuntimeError("scope never triggered on the marker after %d reads - check CH%d "
                           "(PMOD J1 pin 1) and that the new bitstream is loaded" % (max_reads, M2C_MARK_CH))
    t0 = time.time()
    while int(scpi_query(scope, ":OPERegister:CONDition?")) & 0x8:   # bit 3: still running
        if time.time() - t0 > 5:
            raise RuntimeError("acquisition did not finish after the trigger")
        time.sleep(0.01)
    t, sda, pre, srate, _ = scope_fetch(scope, M2C_SDA_CH)
    tm, mark, _, _, _ = scope_fetch(scope, M2C_MARK_CH)
    if len(tm) != len(t):
        raise RuntimeError("SDA and marker records differ in length")
    return t, sda, mark, pre, srate


def crossing(t, v, i, level):
    """Linear interpolation of the time v crosses level between samples i and i + 1."""
    v0, v1 = v[i], v[i + 1]
    return t[i] if v1 == v0 else t[i] + (level - v0) * (t[i + 1] - t[i]) / (v1 - v0)


def analyze_edge(t, sda, mark):
    """10-90 % rise (pull-up only) and fall (sensor pulling the ACK low) of one capture.

    Time zero is the marker's rising edge. The marker is 5 FSM states long, so
    its width gives the state period Ts (see the M2c constants for the timeline):
      rise: SDA released at t = Ts;  plateau: [3 Ts, 4 Ts), just before SCL falls
      fall: the sensor's ACK, after SCL falls at t = 4 Ts.
    Levels: low before the rise, high on the plateau, low after the fall;
    10 % / 90 % of each edge's own swing. Noise-safe crossings: the 90 % time
    is the first sample at/above 90 %, the 10 % time is the last sample at/below
    10 % before it (mirror image for the fall).
    """
    mhalf = 0.5 * (mark.min() + mark.max())
    up = np.flatnonzero((mark[:-1] < mhalf) & (mark[1:] >= mhalf))
    dn = np.flatnonzero((mark[:-1] >= mhalf) & (mark[1:] < mhalf))
    if len(up) == 0 or len(dn[dn > up[0]]) == 0:
        raise RuntimeError("marker pulse not complete inside the record")
    t_on = crossing(t, mark, up[0], mhalf)
    t_off = crossing(t, mark, dn[dn > up[0]][0], mhalf)
    ts = (t_off - t_on) / 5.0
    tt = t - t_on

    v_low = np.median(sda[(tt > -1.5e-6) & (tt < -0.1e-6)])
    v_high = np.median(sda[(tt > 3 * ts) & (tt < 4 * ts)])
    swing = v_high - v_low
    if swing < 1.0:
        raise RuntimeError("SDA swing only %.2f V around the marker - is CH%d on SDA_0?" % (swing, M2C_SDA_CH))

    # rise
    r10, r90 = v_low + 0.1 * swing, v_low + 0.9 * swing
    k90 = np.flatnonzero((tt > -0.1e-6) & (sda >= r90))[0]
    k10 = np.flatnonzero((np.arange(len(sda)) < k90) & (sda <= r10))[-1]
    t_r10, t_r90 = crossing(t, sda, k10, r10), crossing(t, sda, k90 - 1, r90)

    # fall: low level after the edge, from 0.5 us after the 50 % point to the marker end
    k_after = np.flatnonzero((tt > 4 * ts - 0.2e-6) & (sda <= v_low + 0.5 * swing))
    if len(k_after) == 0:
        raise RuntimeError("SDA never went low after SCL fell: no ACK from the sensor?")
    t_half = tt[k_after[0]]
    v_low2 = np.median(sda[(tt > t_half + 0.5e-6) & (tt < 5 * ts)])
    swing2 = v_high - v_low2
    f90, f10 = v_low2 + 0.9 * swing2, v_low2 + 0.1 * swing2
    j10 = np.flatnonzero((tt > 4 * ts - 0.2e-6) & (sda <= f10))[0]
    j90 = np.flatnonzero((np.arange(len(sda)) < j10) & (sda >= f90))[-1]
    t_f90, t_f10 = crossing(t, sda, j90, f90), crossing(t, sda, j10 - 1, f10)

    # RC time constant from the rise samples between 10 % and 90 %: ln((Vh - v)/swing) = -(t - t0)/tau
    sel = (tt >= t_r10 - t_on) & (tt <= t_r90 - t_on)
    tau_fit = float("nan")
    if np.count_nonzero(sel) >= 5:
        y = np.log(np.clip((v_high - sda[sel]) / swing, 1e-6, None))
        tau_fit = -1.0 / np.polyfit(t[sel], y, 1)[0]
    rise = t_r90 - t_r10
    return {"ts": ts, "v_low": v_low, "v_high": v_high, "v_ack_low": v_low2,
            "rise": rise, "fall": t_f10 - t_f90, "tau_10_90": rise / np.log(9.0), "tau_fit": tau_fit,
            "t_on": t_on, "t_r10": t_r10, "t_r90": t_r90, "t_f90": t_f90, "t_f10": t_f10,
            "ack_delay": t_f90 - t_on - 4 * ts,
            "levels": (r10, r90, f90, f10)}


def run_m2c(insts, args):
    scope = insts[SCOPE_MODEL]
    idn = scpi_query(scope, "*IDN?")
    settings = m2c_configure(scope)
    print("Scope: CH%d = SDA_0, CH%d = marker; DC, full bandwidth, %g s screen, acquisition %s"
          % (M2C_SDA_CH, M2C_MARK_CH, settings["time_range"], settings["acquire"]))
    print("Probe attenuation setting: CH%d %gx, CH%d %gx   (use 10x probes, matching switch)"
          % (M2C_SDA_CH, settings["probe_sda"], M2C_MARK_CH, settings["probe_marker"]))
    if settings["probe_sda"] < 10:
        print("WARNING: SDA probe set to %gx - a 1x probe adds ~100 pF and slows the rise" % settings["probe_sda"])
    fpga = Fpga(not args.no_bitfile)
    rows, first = [], None
    try:
        for k in range(args.m2c_n):
            t, sda, mark, pre, srate = m2c_acquire(scope, fpga)
            e = analyze_edge(t, sda, mark)
            if first is None:
                first = (t, sda, mark, pre, srate, e)
                print("Sample interval %.3g ns (%.3g Sa/s), %d points, FSM state %.3f us"
                      % (pre["xincrement"] * 1e9, 1 / pre["xincrement"], len(t), e["ts"] * 1e6))
                print("SDA levels: low %.3f V, high %.3f V, ACK low %.3f V"
                      % (e["v_low"], e["v_high"], e["v_ack_low"]))
            print("  capture %2d: rise %7.1f ns   fall %6.2f ns   tau(fit) %7.1f ns   ACK %.2f us after SCL fell"
                  % (k + 1, e["rise"] * 1e9, e["fall"] * 1e9, e["tau_fit"] * 1e9, e["ack_delay"] * 1e6))
            rows.append([k + 1, "%.4e" % e["rise"], "%.4e" % e["fall"], "%.4e" % e["tau_10_90"],
                         "%.4e" % e["tau_fit"], "%.4f" % e["v_low"], "%.4f" % e["v_high"],
                         "%.4f" % e["v_ack_low"], "%.4e" % e["ack_delay"], "%.4e" % pre["xincrement"]])
    finally:
        fpga.close()
    write_csv("m2c_edges.csv", ["capture", "rise_10_90_s", "fall_90_10_s", "tau_from_rise_s", "tau_fit_s",
                                "v_low_V", "v_high_V", "v_ack_low_V", "ack_delay_s", "dt_s"], rows)
    t, sda, mark, pre, srate, e = first
    np.savez_compressed(results_path("m2c_ack_edge.npz"), t=t, sda=sda, marker=mark, t_on=e["t_on"],
                        preamble=np.array(list(pre.values())), preamble_keys=np.array(list(pre.keys())),
                        srate=srate, idn=idn, when=time.strftime("%Y-%m-%d %H:%M:%S"),
                        probe_sda=settings["probe_sda"])
    print("Saved", results_path("m2c_ack_edge.npz"))
    report_m2c()


def report_m2c():
    rows = read_csv("m2c_edges.csv")
    if not rows:
        print("No results/m2c_edges.csv yet - run --m2c.")
        return False
    col = lambda k: np.array([float(r[k]) for r in rows])
    rise, fall, tau = col("rise_10_90_s"), col("fall_90_10_s"), col("tau_from_rise_s")
    tau_fit = col("tau_fit_s")
    sd = lambda x: np.std(x, ddof=1) if len(x) > 1 else 0.0
    c_bus = tau.mean() / R_PULLUP_OHM
    print("")
    print("M2c result over %d captures (mean +/- std):" % len(rows))
    print("  10-90 %% rise time: %.1f +/- %.1f ns   (pull-up %.0f kohm charging the bus)"
          % (rise.mean() * 1e9, sd(rise) * 1e9, R_PULLUP_OHM / 1e3))
    print("  90-10 %% fall time: %.2f +/- %.2f ns   (sensor's open-drain transistor pulling down)"
          % (fall.mean() * 1e9, sd(fall) * 1e9))
    print("  rise / fall = %.0f" % (rise.mean() / fall.mean()))
    print("  RC time constant: tau = t_rise / ln 9 = %.1f ns   (log-fit of the rise: %.1f ns)"
          % (tau.mean() * 1e9, np.nanmean(tau_fit) * 1e9))
    print("  bus capacitance C = tau / R = %.1f pF   (includes the scope probe)" % (c_bus * 1e12))
    for mode, limit in I2C_TR_MAX_S.items():
        print("  %-24s max rise %4.0f ns: %s" % (mode, limit * 1e9, "OK" if rise.mean() <= limit else "TOO SLOW"))
    write_csv("m2c_summary.csv", ["quantity", "value"], [
        ["captures", len(rows)], ["rise_mean_s", "%.4e" % rise.mean()], ["rise_std_s", "%.4e" % sd(rise)],
        ["fall_mean_s", "%.4e" % fall.mean()], ["fall_std_s", "%.4e" % sd(fall)],
        ["tau_from_rise_s", "%.4e" % tau.mean()], ["tau_fit_s", "%.4e" % np.nanmean(tau_fit)],
        ["R_pullup_ohm", "%.0f" % R_PULLUP_OHM], ["C_bus_F", "%.4e" % c_bus]])
    plot_m2c()
    return True


def plot_m2c():
    path = os.path.join(RESULTS, "m2c_ack_edge.npz")
    if not os.path.exists(path):
        return False
    d = np.load(path, allow_pickle=False)
    t, sda, mark = d["t"], d["sda"], d["marker"]
    e = analyze_edge(t, sda, mark)
    us = lambda x: (x - e["t_on"]) * 1e6
    r10, r90, f90, f10 = e["levels"]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(2, 1, 1)
    ax.plot(us(t), sda, lw=0.8, label="SDA_0 (CH%d)" % M2C_SDA_CH)
    ax.plot(us(t), mark, lw=0.8, alpha=0.7, label="marker, PMOD J1 pin 1 (CH%d)" % M2C_MARK_CH)
    for n, name in ((1, "SDA released"), (2, "SCL rises"), (4, "SCL falls"), (5, "marker ends")):
        ax.axvline(n * e["ts"] * 1e6, color="gray", ls=":", lw=0.8)
        ax.text(n * e["ts"] * 1e6, 3.75, " " + name, fontsize=8, va="top")
    ax.set_xlim(-2, 15)
    ax.set_ylim(-0.4, 3.9)
    ax.set_xlabel("Time from marker rising edge [us]")
    ax.set_ylabel("Voltage [V]")
    ax.set_title("M2c: SDA through the read-address ACK (rise: pull-up only; fall: sensor ACK)")
    ax.grid(True)
    ax.legend(loc="center right", fontsize=8)

    ax = fig.add_subplot(2, 2, 3)
    w = (t > e["t_r10"] - 1.0 * e["rise"]) & (t < e["t_r90"] + 1.0 * e["rise"])
    ax.plot((t[w] - e["t_r10"]) * 1e9, sda[w], lw=0.8)
    for lev, lab in ((r10, "10 %"), (r90, "90 %")):
        ax.axhline(lev, color="gray", ls="--", lw=0.8)
    ax.axvline(0, color="red", ls=":")
    ax.axvline((e["t_r90"] - e["t_r10"]) * 1e9, color="red", ls=":")
    ax.set_title("Rise (10-90 %%): %.0f ns, tau = %.0f ns" % (e["rise"] * 1e9, e["tau_10_90"] * 1e9), fontsize=10)
    ax.set_xlabel("Time from 10 % point [ns]")
    ax.set_ylabel("SDA [V]")
    ax.grid(True)

    ax = fig.add_subplot(2, 2, 4)
    span = max(e["fall"], 5e-9)
    w = (t > e["t_f90"] - 4 * span) & (t < e["t_f10"] + 4 * span)
    ax.plot((t[w] - e["t_f90"]) * 1e9, sda[w], ".-", lw=0.8, ms=2)
    for lev in (f90, f10):
        ax.axhline(lev, color="gray", ls="--", lw=0.8)
    ax.axvline(0, color="red", ls=":")
    ax.axvline((e["t_f10"] - e["t_f90"]) * 1e9, color="red", ls=":")
    ax.set_title("Fall (90-10 %%): %.1f ns" % (e["fall"] * 1e9), fontsize=10)
    ax.set_xlabel("Time from 90 % point [ns]")
    ax.set_ylabel("SDA [V]")
    ax.grid(True)
    fig.tight_layout()
    save_fig(fig, "m2c_ack_edge.png")
    return True


# ---------------------------------------------------------------------------
# M2b: FFT analysis
# ---------------------------------------------------------------------------
def amplitude_spectrum(v, dt):
    """Mean removed, Hann window, rFFT. Returns (f [Hz], peak amplitude [V]).

    Scaled by 2 / sum(window) so a sinusoid A*sin(2 pi f t) on a bin centre
    reads A. DC and Nyquist bins are not doubled.
    """
    x = np.asarray(v, dtype=float)
    x = x - x.mean()
    w = np.hanning(len(x))
    amp = 2.0 * np.abs(np.fft.rfft(x * w)) / w.sum()
    amp[0] /= 2.0
    if len(x) % 2 == 0:
        amp[-1] /= 2.0
    return np.fft.rfftfreq(len(x), dt), amp


def line_amplitude(f, amp, f0=LINE_HZ, half_band=F_LINE_BAND_HZ):
    """Largest amplitude in f0 +/- half_band. Returns (frequency, amplitude)."""
    sel = (f >= f0 - half_band) & (f <= f0 + half_band)
    k = np.argmax(amp[sel])
    return f[sel][k], amp[sel][k]


def noise_floor(f, amp, f0=LINE_HZ):
    """Median amplitude in the bands either side of the line search band (40-55, 65-80 Hz)."""
    sel = ((f >= f0 - 20) & (f < f0 - F_LINE_BAND_HZ)) | ((f > f0 + F_LINE_BAND_HZ) & (f <= f0 + 20))
    return float(np.median(amp[sel]))


def analyze_m2b(args):
    short = load_wave("m2b_short_wave.npz")
    long_ = load_wave("m2b_long_wave.npz")
    if short is None and long_ is None:
        print("No M2b captures yet (results/m2b_short_wave.npz, m2b_long_wave.npz).")
        return False

    res = {}
    for name, w in (("short", short), ("long", long_)):
        if w is None:
            print("Missing results/m2b_%s_wave.npz" % name)
            continue
        dt = w["t"][1] - w["t"][0]
        f, amp = amplitude_spectrum(w["v"], dt)
        f60, a60 = line_amplitude(f, amp)
        floor = noise_floor(f, amp)
        res[name] = {"f": f, "amp": amp, "f60": f60, "a60": a60, "floor": floor, "w": w, "dt": dt}
        print("%s-loop 60 Hz amplitude: %.4e V peak (%.4e V rms) at %.2f Hz   "
              "noise floor %.2e V  (%.0fx above)   captured %s"
              % (name.capitalize(), a60, a60 / np.sqrt(2), f60, floor, a60 / floor if floor > 0 else 0, w["when"]))

    rows = [[k, "%.3f" % r["f60"], "%.6e" % r["a60"], "%.6e" % r["floor"], len(r["w"]["v"]),
             "%.6g" % (1 / r["dt"]), r["w"]["settings"].get("vdiv"), r["w"]["when"]] for k, r in res.items()]
    summary = [["loop", "f_peak_Hz", "amp60_Vpeak", "noise_floor_V", "samples", "srate_Sa_s", "vdiv", "captured"]] + rows

    if len(res) == 2:
        s, l = res["short"]["w"]["settings"], res["long"]["w"]["settings"]
        for key in ("vdiv", "coupling", "time_range", "probe", "channel", "acquire"):
            if s.get(key) != l.get(key):
                print("WARNING: captures differ in %s (short %s, long %s)" % (key, s.get(key), l.get(key)))
        ratio = res["long"]["a60"] / res["short"]["a60"]
        print("")
        print("Amplitude ratio (long / short): %.2f" % ratio)
        if res["short"]["a60"] < 3 * res["short"]["floor"]:
            print("NOTE: short-loop 60 Hz is within 3x of the noise floor, so the ratio is a LOWER bound")
        summary.append(["amplitude_ratio_long_over_short", "%.4f" % ratio])
        if args.short_area_cm2 and args.long_area_cm2:
            aratio = args.long_area_cm2 / args.short_area_cm2
            print("Area ratio (long / short):      %.2f   (%.3g cm^2 / %.3g cm^2)"
                  % (aratio, args.long_area_cm2, args.short_area_cm2))
            print("Amplitude ratio / area ratio:   %.2f   (1.0 would be a perfect match; expect only rough agreement)"
                  % (ratio / aratio))
            summary += [["short_area_cm2", "%g" % args.short_area_cm2], ["long_area_cm2", "%g" % args.long_area_cm2],
                        ["area_ratio_long_over_short", "%.4f" % aratio]]
        else:
            print("Area ratio: not given (add --short-area-cm2 A --long-area-cm2 B)")

    path = results_path("m2b_summary.csv")
    with open(path, "w", newline="") as fh:
        csv.writer(fh).writerows(summary)
    print("Saved", path)

    fig, ax = plt.subplots(figsize=(8, 5))
    for name, r in res.items():
        sel = r["f"] <= F_PLOT_MAX_HZ
        line, = ax.semilogy(r["f"][sel], r["amp"][sel], lw=0.8,
                            label="%s loop: %.2e V at %.1f Hz" % (name, r["a60"], r["f60"]))
        ax.plot(r["f60"], r["a60"], "o", color=line.get_color())
    for h in (1, 2, 3):
        ax.axvline(h * LINE_HZ, color="gray", ls=":", lw=0.8)
    ax.set_title("M2b: FFT of the quiet node, short vs long ground loop (Hann window)")
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("Amplitude [V peak]")
    ax.set_xlim(0, F_PLOT_MAX_HZ)
    ax.grid(True, which="both")
    ax.legend()
    save_fig(fig, "m2b_fft.png")

    fig, ax = plt.subplots(figsize=(8, 4))
    for name, r in res.items():
        t, v = r["w"]["t"], r["w"]["v"]
        sel = t <= t[0] + 0.1                       # first 100 ms = 6 line cycles
        ax.plot((t[sel] - t[0]) * 1000, v[sel] * 1000, lw=0.6, label="%s loop" % name)
    ax.set_title("M2b: quiet node, first 100 ms of each capture")
    ax.set_xlabel("Time [ms]")
    ax.set_ylabel("Voltage, AC coupled [mV]")
    ax.grid(True)
    ax.legend()
    save_fig(fig, "m2b_waveforms.png")
    return True


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------
def mode_m2a(args):
    def body(insts):
        wait_for_user(["M2a: DMM on one fixed quiet DC node.",
                       "Leads in the DMM's V HI and LO jacks - NOT the current jacks.",
                       "Leave the setup untouched for the whole sweep."])
        run_m2a(insts, args)
    with_instruments([DMM_MODEL], body)


def mode_m2b_short(args):
    with_instruments([SCOPE_MODEL], lambda insts: run_m2b_capture(insts, args, "short"))


def mode_m2b_long(args):
    with_instruments([SCOPE_MODEL], lambda insts: run_m2b_capture(insts, args, "long"))


def mode_scope_check(args):
    with_instruments([SCOPE_MODEL], lambda insts: run_scope_check(insts, args))


def mode_m2b_analyze(args):
    if not analyze_m2b(args):
        raise RuntimeError("nothing to analyze yet")


def mode_m2c(args):
    with_instruments([SCOPE_MODEL], lambda insts: run_m2c(insts, args))


def mode_plot_only(args):
    a = plot_m2a()
    b = analyze_m2b(args)
    c = os.path.exists(os.path.join(RESULTS, "m2c_edges.csv")) and report_m2c()
    if not (a or b or c):
        print("Nothing to plot yet.")


def main():
    parser = argparse.ArgumentParser(
        description="ECE 437 Lab 7 Milestone 2. Choose exactly one mode; with none, nothing runs.")
    g = parser.add_mutually_exclusive_group()
    g.add_argument("--m2a", action="store_true", help="DMM NPLC sweep")
    g.add_argument("--m2b-short", action="store_true", help="scope capture, short ground (spring)")
    g.add_argument("--m2b-long", action="store_true", help="scope capture, long ground wire")
    g.add_argument("--m2b-analyze", action="store_true", help="FFT + 60 Hz comparison from saved captures")
    g.add_argument("--scope-check", action="store_true", help="check volts scaling on a 3.3 V logic line")
    g.add_argument("--m2c", action="store_true", help="I2C SDA edge through the ACK (scope + FPGA)")
    g.add_argument("--plot-only", action="store_true", help="redraw plots from saved files")
    parser.add_argument("--nplc", type=lambda s: [float(x) for x in s.split(",")],
                        help="M2a: comma-separated NPLC list instead of the full range")
    parser.add_argument("--range", type=float, help="M2a: fixed DMM range in V (default: what autorange picks)")
    parser.add_argument("--autozero-off", action="store_true", help="M2a: autozero OFF (halves the sweep time)")
    parser.add_argument("--channel", type=int, default=SCOPE_CHANNEL, help="M2b: scope channel")
    parser.add_argument("--vdiv", type=float, default=SCOPE_VDIV, help="M2b: V/div (use the same for both)")
    parser.add_argument("--time-range", type=float, default=SCOPE_TIME_RANGE_S, help="M2b: record length [s]")
    parser.add_argument("--short-area-cm2", type=float, help="M2b: short ground loop area [cm^2]")
    parser.add_argument("--long-area-cm2", type=float, help="M2b: long ground loop area [cm^2]")
    parser.add_argument("--m2c-n", type=int, default=M2C_N_DEFAULT, help="M2c: number of captures")
    parser.add_argument("--no-bitfile", action="store_true", help="M2c: keep the bitstream already loaded")
    parser.add_argument("--show", action="store_true", help="open the saved plots at the end")
    args = parser.parse_args()

    modes = [("m2a", mode_m2a), ("m2b_short", mode_m2b_short), ("m2b_long", mode_m2b_long),
             ("m2b_analyze", mode_m2b_analyze), ("scope_check", mode_scope_check),
             ("m2c", mode_m2c), ("plot_only", mode_plot_only)]
    chosen = [fn for name, fn in modes if getattr(args, name)]
    if not chosen:
        parser.print_help()
        return 0

    try:
        chosen[0](args)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Instrument sessions were closed on the way out.")
        return 1
    except Exception as e:
        print("FAIL:", e)
        return 1

    if args.show and os.path.isdir(RESULTS):
        for name in sorted(os.listdir(RESULTS)):
            if name.startswith(("m2a", "m2b", "m2c")) and name.endswith(".png"):
                os.startfile(os.path.join(RESULTS, name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
