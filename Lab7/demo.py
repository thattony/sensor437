# -*- coding: utf-8 -*-
"""
ECE 437 - Lab 7 demo (M1b + M1a + M2a NPLC + M2b FFTs + M2c ACK edge), from the SAVED data only.

Live demo steps, run before this script:
    1. 4.72 V. My code never commands more than 4.4 V.
    2. python lab7_m1.py --supply-test
    3. python lab7_m1.py --demo-cutoff
    4. python lab7_m1.py --sensor-test

    python demo.py                 print the results, save the PDF, show the plots
    python demo.py --no-show       same, but do not open plot windows

Never connects to the FPGA or any instrument and never takes a measurement.
Reads (written earlier by lab7_m1.py --m1b):
    results/m1b_fast.csv           fast run   (sample, t_s, raw, raw_hex, temp_C)
    results/m1b_slow300ms.csv      100 readings ~300 ms apart (same columns)
    results/m1a_summary.csv        one row per M1a power level (mean_P_W, mean_T_C, std_T_C, n, ...)
    results/m2a_nplc_summary.csv   one row per NPLC (lab7_m2.py --m2a); skipped if missing
    results/m2b_summary.csv        M2b 60 Hz amplitudes, ratios, loop areas (lab7_m2.py --m2b-analyze)
    results/m2b_short_wave.npz     M2b captures (t, v), short and long ground loop
    results/m2b_long_wave.npz
    results/m2c_edges.csv          one row per M2c capture (lab7_m2.py --m2c); skipped if missing
    results/m2c_ack_edge.npz       first M2c capture: t, SDA, marker
    lab7_m1.py                     source text only, for the 16-bit setting
Writes:
    results/m1b_demo_report.pdf
"""
import os
import re
import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")
FAST_CSV = os.path.join(RESULTS, "m1b_fast.csv")
SLOW_CSV = os.path.join(RESULTS, "m1b_slow300ms.csv")
M1_SOURCE = os.path.join(HERE, "lab7_m1.py")
M1A_CSV = os.path.join(RESULTS, "m1a_summary.csv")
M2A_CSV = os.path.join(RESULTS, "m2a_nplc_summary.csv")
OUT_PDF = os.path.join(RESULTS, "m1b_demo_report.pdf")
LINE_HZ = 60.0                # 1 PLC = 1/60 s
M2B_CSV = os.path.join(RESULTS, "m2b_summary.csv")
M2B_NPZ = {k: os.path.join(RESULTS, "m2b_%s_wave.npz" % k) for k in ("short", "long")}
F_PLOT_MAX_HZ = 1000.0        # M2b FFT plot range
M2C_CSV = os.path.join(RESULTS, "m2c_edges.csv")
M2C_NPZ = os.path.join(RESULTS, "m2c_ack_edge.npz")
R_PULLUP_OHM = 10.0e3         # SDA_0 pull-up R6 (sensor board schematic)
I2C_TR_MAX_S = [("standard mode, 100 kHz", 1000e-9), ("fast mode, 400 kHz", 300e-9)]

LSB_C = 0.0078125             # 16-bit mode: 1 count = 1/128 deg C
N_TIMED = 1000                # wall-clock time is reported for the first 1000 readings
N_COMPARE = 100               # first 100 fast vs 100 slow
N_LIST = [1, 2, 4, 8, 16, 32, 64, 128, 256]
MIN_BLOCKS = 8                # need >= 8 block means for a std (same rule as lab7_m1.py)
DEPARTURE_RATIO = 1.5         # departed when measured std > 1.5 x the 1/sqrt(N) line


def load_run(path):
    """Return (t_s, raw, temp_C) arrays. Stops on a missing file or a non-numeric value."""
    if not os.path.exists(path):
        raise SystemExit("Missing %s - run 'python lab7_m1.py --m1b' first." % path)
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    try:
        t = np.array([float(r["t_s"]) for r in rows])
        raw = np.array([int(r["raw"]) for r in rows])
        temp = np.array([float(r["temp_C"]) for r in rows])
    except (KeyError, TypeError, ValueError) as e:
        raise SystemExit("%s: bad or missing value (%s)" % (path, e))
    if not (np.all(np.isfinite(t)) and np.all(np.isfinite(temp))):
        raise SystemExit("%s contains NaN/inf" % path)
    return t, raw, temp


def sample_std(raw):
    """Sample std (ddof = 1) in deg C, on the integer codes so a constant set gives exactly 0."""
    return float(np.std(raw.astype(float), ddof=1)) * LSB_C


def averaging(temps):
    """Std of the means of non-overlapping blocks of N readings."""
    ns, sig = [], []
    for n in N_LIST:
        blocks = len(temps) // n
        if blocks < MIN_BLOCKS:
            break
        ns.append(n)
        sig.append(np.std(temps[: blocks * n].reshape(blocks, n).mean(axis=1), ddof=1))
    ns, sig = np.array(ns), np.array(sig)
    return ns, sig, sig[0] / np.sqrt(ns)


def check_16bit(raw_all):
    """16-bit evidence: the config write in lab7_m1.py, and the saved raw codes."""
    text = open(M1_SOURCE, encoding="utf-8").read() if os.path.exists(M1_SOURCE) else ""
    m = re.search(r"^CONFIG_16BIT\s*=\s*(0x[0-9A-Fa-f]+)", text, re.M)
    config = m.group(1) if m else None
    checked = 'w["config"] != CONFIG_16BIT' in text
    # 13-bit data always has bits 2:0 = 0 (flag bits), so a code that is not a multiple of 8 is 16-bit
    fine_codes = int(np.count_nonzero(raw_all % 8))
    ok = config is not None and int(config, 16) == 0x80 and checked and fine_codes > 0
    return ok, config, fine_codes


def load_m1a():
    """M1a summary: measured power, mean temperature, std, readings per level."""
    if not os.path.exists(M1A_CSV):
        raise SystemExit("Missing %s - run 'python lab7_m1.py --m1a' first." % M1A_CSV)
    with open(M1A_CSV, newline="") as f:
        rows = list(csv.DictReader(f))
    try:
        cols = {k: np.array([float(r[k]) for r in rows])
                for k in ("level", "mean_P_W", "mean_T_C", "std_T_C", "n", "level_wait_s")}
    except (KeyError, TypeError, ValueError) as e:
        raise SystemExit("%s: bad or missing value (%s)" % (M1A_CSV, e))
    if not all(np.all(np.isfinite(v)) for v in cols.values()):
        raise SystemExit("%s contains NaN/inf" % M1A_CSV)
    return cols


def load_m2a():
    """M2a NPLC sweep summary, or None if it has not been run yet (the M1 part still runs)."""
    if not os.path.exists(M2A_CSV):
        print("(No %s yet - M2a section skipped. Run 'python lab7_m2.py --m2a'.)"
              % os.path.relpath(M2A_CSV, HERE))
        return None
    with open(M2A_CSV, newline="") as f:
        rows = list(csv.DictReader(f))
    try:
        cols = {k: np.array([float(r[k]) for r in rows])
                for k in ("nplc", "t_int_s", "n", "mean_V", "std_V")}
    except (KeyError, TypeError, ValueError) as e:
        raise SystemExit("%s: bad or missing value (%s)" % (M2A_CSV, e))
    cols["range_V"] = rows[0]["dmm_range_V"]
    cols["autozero"] = rows[0]["autozero"]
    return cols


def load_m2b():
    """M2b summary (per-loop 60 Hz amplitude + ratios) and both captures, or None if not run yet."""
    if not (os.path.exists(M2B_CSV) and all(os.path.exists(f) for f in M2B_NPZ.values())):
        print("(No M2b results yet - M2b section skipped. Run lab7_m2.py --m2b-short/--m2b-long/--m2b-analyze.)")
        return None
    out = {"loops": {}, "info": {}}
    with open(M2B_CSV, newline="") as f:
        for row in csv.reader(f):
            if row and row[0] in ("short", "long"):
                out["loops"][row[0]] = {"f60": float(row[1]), "a60": float(row[2]), "floor": float(row[3]),
                                        "vdiv": row[6]}
            elif len(row) == 2:
                out["info"][row[0]] = row[1]
    for k, path in M2B_NPZ.items():
        d = np.load(path, allow_pickle=False)
        out["loops"].setdefault(k, {}).update({"t": d["t"], "v": d["v"]})
    return out


def spectrum(v, dt):
    """Mean removed, Hann window, rFFT scaled to peak volts (same as lab7_m2.py)."""
    x = np.asarray(v, dtype=float) - np.mean(v)
    w = np.hanning(len(x))
    return np.fft.rfftfreq(len(x), dt), 2.0 * np.abs(np.fft.rfft(x * w)) / w.sum()


def load_m2c():
    """M2c per-capture edge times plus the first capture's waveforms, or None if not run yet."""
    if not (os.path.exists(M2C_CSV) and os.path.exists(M2C_NPZ)):
        print("(No M2c results yet - M2c section skipped. Run 'python lab7_m2.py --m2c'.)")
        return None
    with open(M2C_CSV, newline="") as f:
        rows = list(csv.DictReader(f))
    try:
        cols = {k: np.array([float(r[k]) for r in rows])
                for k in ("rise_10_90_s", "fall_90_10_s", "tau_from_rise_s", "tau_fit_s")}
    except (KeyError, TypeError, ValueError) as e:
        raise SystemExit("%s: bad or missing value (%s)" % (M2C_CSV, e))
    d = np.load(M2C_NPZ, allow_pickle=False)
    cols["t"], cols["sda"], cols["marker"] = d["t"], d["sda"], d["marker"]
    return cols


def edge_points(t, sda, mark):
    """10/90 % crossing times of the saved capture (same rules as lab7_m2.py).

    t = 0 at the marker's rising edge; the marker lasts 5 FSM states (Ts each);
    SDA is released at Ts, SCL falls at 4 Ts and the sensor pulls SDA low (ACK).
    """
    def cross(i, lev):
        return t[i] + (lev - sda[i]) * (t[i + 1] - t[i]) / (sda[i + 1] - sda[i])
    half = 0.5 * (mark.min() + mark.max())
    up = np.flatnonzero((mark[:-1] < half) & (mark[1:] >= half))[0]
    dn = np.flatnonzero((mark[:-1] >= half) & (mark[1:] < half))
    t_on, t_off = t[up], t[dn[dn > up][0]]
    ts = (t_off - t_on) / 5.0
    tt = t - t_on
    lo = np.median(sda[(tt > -1.5e-6) & (tt < -0.1e-6)])
    hi = np.median(sda[(tt > 3 * ts) & (tt < 4 * ts)])
    r10, r90 = lo + 0.1 * (hi - lo), lo + 0.9 * (hi - lo)
    k90 = np.flatnonzero((tt > -0.1e-6) & (sda >= r90))[0]
    k10 = np.flatnonzero(sda[:k90] <= r10)[-1]
    j50 = np.flatnonzero((tt > 4 * ts - 0.2e-6) & (sda <= lo + 0.5 * (hi - lo)))[0]
    lo2 = np.median(sda[(tt > tt[j50] + 0.5e-6) & (tt < 5 * ts)])
    f90, f10 = lo2 + 0.9 * (hi - lo2), lo2 + 0.1 * (hi - lo2)
    j10 = np.flatnonzero((tt > 4 * ts - 0.2e-6) & (sda <= f10))[0]
    j90 = np.flatnonzero(sda[:j10] >= f90)[-1]
    return {"t_on": t_on, "ts": ts, "r10": r10, "r90": r90, "f90": f90, "f10": f10,
            "t_r10": cross(k10, r10), "t_r90": cross(k90 - 1, r90),
            "t_f90": cross(j90, f90), "t_f10": cross(j10 - 1, f10)}


def main():
    parser = argparse.ArgumentParser(description="Lab 7 M1 + M2 demo from saved data (no hardware).")
    parser.add_argument("--no-show", action="store_true", help="do not open plot windows")
    args = parser.parse_args()

    tf, rawf, tempf = load_run(FAST_CSV)
    ts, raws, temps = load_run(SLOW_CSV)
    if len(tf) < N_TIMED:
        raise SystemExit("Only %d fast readings; at least %d required." % (len(tf), N_TIMED))
    if len(ts) != N_COMPARE:
        raise SystemExit("m1b_slow300ms.csv has %d readings, expected %d." % (len(ts), N_COMPARE))

    # 1. Wall-clock time of the first 1000 consecutive fast readings
    t1000 = tf[N_TIMED - 1]          # timer starts just before reading 1

    # 2/3. Averaging law and departure
    ns, sig, ideal = averaging(tempf)
    dep = next((k for k in range(1, len(ns)) if sig[k] > DEPARTURE_RATIO * ideal[k]), None)

    # 4. 300 ms vs first 100 fast
    s_fast = sample_std(rawf[:N_COMPARE])
    s_slow = sample_std(raws)
    spacing_ms = np.mean(np.diff(ts)) * 1e3

    m1a = load_m1a()
    ok16, config, fine_codes = check_16bit(np.concatenate([rawf, raws]))

    lines = [
        "1. WALL-CLOCK TIME",
        "   First %d consecutive fast readings: %.4f s  (%.2f ms per reading)"
        % (N_TIMED, t1000, t1000 / N_TIMED * 1e3),
        "   (the saved run has %d consecutive readings; all are used for the averaging plot)" % len(tf),
        "",
        "2. AVERAGING LAW  (std of the average of N readings, non-overlapping groups)",
        "       N    groups   measured std [C]   1/sqrt(N) line [C]",
    ]
    lines += ["   %5d   %6d      %.6f           %.6f" % (n, len(tempf) // n, s, i)
              for n, s, i in zip(ns, sig, ideal)]
    lines += ["", "3. DEPARTURE N"]
    if dep is not None:
        lines.append("   N = %d  (measured %.6f C vs line %.6f C)" % (ns[dep], sig[dep], ideal[dep]))
    else:
        lines.append("   NOT DETERMINED - the data never exceeded %.1fx the line" % DEPARTURE_RATIO)
    lines += [
        "   Rule: first N where the measured std is more than %.1fx the sigma_1/sqrt(N) line." % DEPARTURE_RATIO,
        "",
        "4. 300 ms COMPARISON  (100 readings each)",
        "   First 100 fast readings:        std = %.6f C" % s_fast,
        "   100 readings ~%.0f ms apart:    std = %.6f C" % (spacing_ms, s_slow),
        "   The fast readings repeat one sensor conversion; the 300 ms readings are independent.",
        "",
        "16-BIT MODE: %s" % ("CONFIRMED" if ok16 else "NOT CONFIRMED"),
        "   lab7_m1.py writes config register 0x03 = %s and aborts unless it reads back 0x80;" % config,
        "   %d of %d saved raw codes use the 1/128 C steps that only 16-bit mode gives."
        % (fine_codes, len(rawf) + len(raws)),
        "",
        "M1a  (%d power levels, %.0f s settling wait per level, sample std ddof=1)"
        % (len(m1a["level"]), m1a["level_wait_s"].max()),
        "   level   power [W]   mean T [C]   std T [C]     n",
    ]
    lines += ["   %5d    %.4f      %.4f      %.5f    %4d" % (lv, p, t, sd, n) for lv, p, t, sd, n in
              zip(m1a["level"], m1a["mean_P_W"], m1a["mean_T_C"], m1a["std_T_C"], m1a["n"])]
    lines.append("   power = measured |V x I| from the supply, not the target")

    m2a = load_m2a()
    if m2a is not None:
        lines += [
            "",
            "M2a  (34461A DC volts across the unpowered resistor, range %s V, autozero %s)"
            % (m2a["range_V"], m2a["autozero"]),
            "    NPLC   integration [ms]    n    mean [uV]   std [uV]",
        ]
        lines += ["   %5g   %12.3f      %4d   %8.1f   %8.1f" % (p, t * 1e3, n, m * 1e6, s * 1e6)
                  for p, t, n, m, s in zip(m2a["nplc"], m2a["t_int_s"], m2a["n"],
                                           m2a["mean_V"], m2a["std_V"])]
        lines.append("   1 PLC = %.1f ms integrates exactly one 60 Hz cycle, so the pickup averages to zero"
                     % (1e3 / LINE_HZ))

    m2b = load_m2b()
    if m2b is not None:
        info = m2b["info"]
        lines += ["", "M2b  (scope FFT of the same resistor node, 1 s records, Hann window)",
                  "   loop    60 Hz amplitude [V peak]   noise floor [V]   peak / floor"]
        lines += ["   %-6s     %.3e at %.1f Hz        %.2e          %5.0fx"
                  % (k, L["a60"], L["f60"], L["floor"], L["a60"] / L["floor"])
                  for k, L in ((k, m2b["loops"][k]) for k in ("short", "long"))]
        lines.append("   amplitude ratio long / short = %.1f" % float(info["amplitude_ratio_long_over_short"]))
        if "area_ratio_long_over_short" in info:
            lines.append("   loop areas %s cm^2 (spring) and %s cm^2 (long wire): area ratio %.0f"
                         % (info["short_area_cm2"], info["long_area_cm2"],
                            float(info["area_ratio_long_over_short"])))
        lines.append("   60 Hz + odd harmonics = interference: fixed by a smaller loop, not by averaging")

    m2c = load_m2c()
    if m2c is not None:
        sd = lambda x: np.std(x, ddof=1) if len(x) > 1 else 0.0
        rise, fall, tau = m2c["rise_10_90_s"], m2c["fall_90_10_s"], m2c["tau_from_rise_s"]
        c_bus = tau.mean() / R_PULLUP_OHM
        lines += [
            "",
            "M2c  (SDA_0 through the read-address ACK, %d captures, triggered on the PMOD marker)" % len(rise),
            "   10-90 %% rise:  %7.1f +/- %.1f ns   (10 kohm pull-up charging the bus)"
            % (rise.mean() * 1e9, sd(rise) * 1e9),
            "   90-10 %% fall:  %7.2f +/- %.2f ns   (sensor's open-drain transistor pulling low)"
            % (fall.mean() * 1e9, sd(fall) * 1e9),
            "   rise / fall = %.0f;  tau = t_rise / ln 9 = %.0f ns (fit %.0f ns)"
            % (rise.mean() / fall.mean(), tau.mean() * 1e9, np.nanmean(m2c["tau_fit_s"]) * 1e9),
            "   bus capacitance C = tau / R = %.1f pF (R = 10 kohm, includes the probe)" % (c_bus * 1e12),
        ]
        lines += ["   %-23s max rise %4.0f ns: %s" % (m, lim * 1e9, "OK" if rise.mean() <= lim else "TOO SLOW")
                  for m, lim in I2C_TR_MAX_S]
    print("\n".join(lines))

    # Plot 1: averaging law
    fig1, ax = plt.subplots(figsize=(7.5, 5.5))
    ax.loglog(ns, sig, "o-", label="measured")
    ax.loglog(ns, ideal, "--", label=r"$\sigma_1/\sqrt{N}$ reference")
    if dep is not None:
        ax.axvline(ns[dep], color="red", ls=":")
        ax.plot(ns[dep], sig[dep], "o", ms=12, mfc="none", mec="red", mew=2,
                label="departure: N = %d" % ns[dep])
    ax.set_title("M1b: averaging law (%d consecutive fast readings, 16-bit)" % len(tf))
    ax.set_xlabel("N, number of readings averaged")
    ax.set_ylabel("Std of the N-reading average [deg C]")
    ax.grid(True, which="both", alpha=0.4)
    ax.legend()
    fig1.tight_layout()

    # Plot 2: 100 readings ~300 ms apart
    fig2, ax = plt.subplots(figsize=(7.5, 5))
    ax.plot(ts, temps, ".-", color="C2")
    ax.set_title("M1b: 100 readings ~300 ms apart\n"
                 "std = %.6f C   (first 100 fast readings: std = %.6f C)" % (s_slow, s_fast))
    ax.set_xlabel("Elapsed time since first reading [s]")
    ax.set_ylabel("Temperature [deg C]")
    ax.grid(True, alpha=0.4)
    fig2.tight_layout()

    # Plot 3: M1a mean temperature vs applied (measured) power
    wait = m1a["level_wait_s"].max()
    fig3, ax = plt.subplots(figsize=(7.5, 5))
    ax.plot(m1a["mean_P_W"], m1a["mean_T_C"], "o-")
    ax.set_title("M1a: mean temperature vs applied power\n(100 readings per level, %.0f s settling wait)" % wait)
    ax.set_xlabel("Applied power, measured |V x I| [W]")
    ax.set_ylabel("Mean temperature [deg C]")
    ax.grid(True, alpha=0.4)
    fig3.tight_layout()

    # Plot 4: M1a std vs mean temperature
    fig4, ax = plt.subplots(figsize=(7.5, 5))
    ax.plot(m1a["mean_T_C"], m1a["std_T_C"], "o")
    ax.set_title("M1a: temperature standard deviation vs mean temperature\n"
                 "(100 readings per level, %.0f s settling wait)" % wait)
    ax.set_xlabel("Mean temperature [deg C]")
    ax.set_ylabel("Standard deviation of temperature [deg C]")
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.4)
    fig4.tight_layout()

    # Plot 5: M2a std vs integration time, log axes
    figs = [fig1, fig2, fig3, fig4]
    if m2a is not None:
        fig5, ax = plt.subplots(figsize=(7.5, 5))
        ax.loglog(m2a["t_int_s"], m2a["std_V"], "o-", label="std of %d readings" % m2a["n"][0])
        for t, s, p in zip(m2a["t_int_s"], m2a["std_V"], m2a["nplc"]):
            ax.annotate("NPLC %g" % p, (t, s), textcoords="offset points", xytext=(5, 5), fontsize=8)
        ax.axvline(1.0 / LINE_HZ, color="red", ls=":", label="1 PLC = %.1f ms" % (1e3 / LINE_HZ))
        ax.set_title("M2a: DMM standard deviation vs integration time\n"
                     "(34461A, DC volts, range %s V, autozero %s)" % (m2a["range_V"], m2a["autozero"]))
        ax.set_xlabel("Integration time, NPLC / 60 Hz [s]")
        ax.set_ylabel("Standard deviation of readings [V]")
        ax.grid(True, which="both", alpha=0.4)
        ax.legend()
        fig5.tight_layout()
        figs.append(fig5)

    # Plot 6: M2b FFTs of both ground loops on common axes
    if m2b is not None:
        fig6b, ax = plt.subplots(figsize=(8, 5))
        for k in ("short", "long"):
            L = m2b["loops"][k]
            f, amp = spectrum(L["v"], L["t"][1] - L["t"][0])
            sel = (f > 0) & (f <= F_PLOT_MAX_HZ)
            line, = ax.semilogy(f[sel], amp[sel], lw=0.8,
                                label="%s loop: %.2e V at %.0f Hz" % (k, L["a60"], L["f60"]))
            ax.plot(L["f60"], L["a60"], "o", color=line.get_color())
        for h in (60, 120, 180):
            ax.axvline(h, color="gray", ls=":", lw=0.8)
        ax.set_xlim(0, F_PLOT_MAX_HZ)
        ax.set_title("M2b: FFT of the resistor node, short vs long ground loop\n"
                     "(amplitude ratio long/short = %.1f)" % float(m2b["info"]["amplitude_ratio_long_over_short"]))
        ax.set_xlabel("Frequency [Hz]")
        ax.set_ylabel("Amplitude [V peak]")
        ax.grid(True, which="both", alpha=0.4)
        ax.legend()
        fig6b.tight_layout()
        figs.append(fig6b)

    # Plot 7: M2c SDA edge through the ACK
    if m2c is not None:
        t, sda, mark = m2c["t"], m2c["sda"], m2c["marker"]
        e = edge_points(t, sda, mark)
        fig6 = plt.figure(figsize=(10, 8))
        ax = fig6.add_subplot(2, 1, 1)
        ax.plot((t - e["t_on"]) * 1e6, sda, lw=0.8, label="SDA_0 (CH1)")
        ax.plot((t - e["t_on"]) * 1e6, mark, lw=0.8, alpha=0.7, label="marker, PMOD J1 pin 1 (CH2)")
        for n, name in ((1, "SDA released"), (2, "SCL rises"), (4, "SCL falls"), (5, "marker ends")):
            ax.axvline(n * e["ts"] * 1e6, color="gray", ls=":", lw=0.8)
            ax.text(n * e["ts"] * 1e6, 3.75, " " + name, fontsize=8, va="top")
        ax.set_xlim(-2, 15)
        ax.set_ylim(-0.4, 3.9)
        ax.set_title("M2c: SDA through the read-address ACK (rise: pull-up only; fall: sensor ACK)")
        ax.set_xlabel("Time from marker rising edge [us]")
        ax.set_ylabel("Voltage [V]")
        ax.grid(True, alpha=0.4)
        ax.legend(loc="center right", fontsize=8)

        rise1 = e["t_r90"] - e["t_r10"]
        ax = fig6.add_subplot(2, 2, 3)
        w = (t > e["t_r10"] - rise1) & (t < e["t_r90"] + rise1)
        ax.plot((t[w] - e["t_r10"]) * 1e9, sda[w], lw=0.8)
        for lev in (e["r10"], e["r90"]):
            ax.axhline(lev, color="gray", ls="--", lw=0.8)
        for x in (0, rise1):
            ax.axvline(x * 1e9, color="red", ls=":")
        ax.set_title("Rise, 10-90 %%: %.0f ns   (mean of %d: %.0f ns)"
                     % (rise1 * 1e9, len(rise), rise.mean() * 1e9), fontsize=10)
        ax.set_xlabel("Time from 10 % point [ns]")
        ax.set_ylabel("SDA [V]")
        ax.grid(True, alpha=0.4)

        fall1 = e["t_f10"] - e["t_f90"]
        ax = fig6.add_subplot(2, 2, 4)
        span = max(fall1, 5e-9)
        w = (t > e["t_f90"] - 4 * span) & (t < e["t_f10"] + 4 * span)
        ax.plot((t[w] - e["t_f90"]) * 1e9, sda[w], ".-", lw=0.8, ms=2)
        for lev in (e["f90"], e["f10"]):
            ax.axhline(lev, color="gray", ls="--", lw=0.8)
        for x in (0, fall1):
            ax.axvline(x * 1e9, color="red", ls=":")
        ax.set_title("Fall, 90-10 %%: %.1f ns   (mean of %d: %.2f ns)"
                     % (fall1 * 1e9, len(fall), fall.mean() * 1e9), fontsize=10)
        ax.set_xlabel("Time from 90 % point [ns]")
        ax.set_ylabel("SDA [V]")
        ax.grid(True, alpha=0.4)
        fig6.tight_layout()
        figs.append(fig6)

    with PdfPages(OUT_PDF) as pdf:
        page = plt.figure(figsize=(8.5, 11))
        page.text(0.07, 0.95, "Lab 7 Milestones 1 and 2 - results", fontsize=14, weight="bold", va="top")
        page.text(0.07, 0.91, "\n".join(lines), fontsize=9, family="monospace", va="top")
        pdf.savefig(page)
        plt.close(page)
        for fig in figs:
            pdf.savefig(fig)
    print("\nSaved", os.path.relpath(OUT_PDF, HERE))

    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
