# -*- coding: utf-8 -*-
"""make_report1_figs.py - figures for Report 1, drawn only from results/."""
import csv
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.join(HERE, "results")
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3})

# --- Figure 1: |a| in the six orientations ---------------------------------
six = json.load(open(os.path.join(R, "six_summary.json")))
o = six["orientations"]
labels = ["%s\n(%s)" % (s["gravity_axis"], s["description"]) for s in o]
mags = [s["mag_g"] for s in o]
fig, ax = plt.subplots(figsize=(6.4, 2.8))
ax.bar(labels, mags, color="#4a7ab5", width=0.6)
ax.axhline(1.0, color="k", lw=1, label="1 g")
ax.axhline(six["mag_mean_g"], color="#c0504d", lw=1, ls="--", label="mean %.4f g" % six["mag_mean_g"])
for i, m in enumerate(mags):
    ax.text(i, m + 0.003, "%.4f" % m, ha="center", fontsize=8)
ax.set_ylim(0.95, 1.07)
ax.set_ylabel("|a| (g)")
ax.set_xlabel("orientation (sensor axis pointing up, as logged)")
ax.set_title("Magnitude of the 400-sample average, six orientations: spread %.4f g" % six["spread_g"])
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(R, "fig1_six_orientations.png"), dpi=170)

# --- Figure 2: new samples per second over the 30 s run --------------------
rows = list(csv.DictReader(open(os.path.join(R, "rate_30s_samples.csv"))))
t = [float(r["t_s"]) for r in rows]
per = [0] * 30
for x in t:
    if int(x) < 30:
        per[int(x)] += 1
gaps = [(t[i + 1] - t[i]) * 1e3 for i in range(len(t) - 1)]
fig, (a1, a2) = plt.subplots(1, 2, figsize=(6.8, 2.6))
a1.bar(range(30), per, color="#4a7ab5", width=0.8)
a1.axhline(100, color="#c0504d", lw=1.2, label="requirement 100/s")
a1.set_xlabel("time into run (s)")
a1.set_ylabel("new samples in that second")
a1.set_ylim(0, 200)
a1.legend(fontsize=8, loc="lower right")
a1.set_title("30 s run: %.1f samples/s" % (len(t) / 30.0))
a2.hist(gaps, bins=60, color="#4a7ab5")
a2.set_xlabel("time between counted samples (ms)")
a2.set_ylabel("count")
a2.set_title("gaps (ODR period 5 ms, poll 1.46 ms)")
fig.tight_layout()
fig.savefig(os.path.join(R, "fig2_rate.png"), dpi=170)

# --- Figure 3: ODR setting vs measured conversion rate vs gated count ------
true = json.load(open(os.path.join(R, "odr_true_rate.json")))["runs"]
rate = json.load(open(os.path.join(R, "rate_summary.json")))
nom = [r["odr_nominal_hz"] for r in true]
fig, ax = plt.subplots(figsize=(5.0, 3.0))
ax.plot([0, 420], [0, 420], color="k", lw=0.8, label="measured = setting")
ax.plot(nom, [r["event_rate_hz"] for r in true], "o-", color="#4a7ab5",
        label="conversions/s (time between data changes)")
ax.plot([s["odr_hz"] for s in rate["odr_sweep"]], [s["new_per_s"] for s in rate["odr_sweep"]], "s--",
        color="#c0504d", label="counted samples/s (ZYXDA = 1)")
ax.axhline(100, color="gray", lw=0.8, ls=":")
ax.set_xlabel("ODR setting in CTRL_REG1_A (Hz)")
ax.set_ylabel("rate (1/s)")
ax.set_xlim(0, 420)
ax.set_ylim(0, 420)
ax.legend(fontsize=7.5, loc="upper left")
ax.set_title("Output data rate: setting vs measured")
fig.tight_layout()
fig.savefig(os.path.join(R, "fig3_odr.png"), dpi=170)
print("figures written to", R)
