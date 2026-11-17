# -*- coding: utf-8 -*-
"""
make_postlab_figs.py - figures for the Lab 9 post lab.

    py -3.14 make_postlab_figs.py

Reads results/lab9_20261110_150354.json (python/lab9_measure.py) and writes
  results/postlab_3c_sweep.png   frame period against exposure, full range + the knee
"""
import json
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
RUN = "lab9_20261110_150354.json"

FOT_MS, READOUT_MS = 0.195, 12.688            # pre lab: 4875 and 488 x 650 clocks of 40 ns


def predicted(exp_ms):
    return FOT_MS + max(READOUT_MS, exp_ms)


d = json.load(open(os.path.join(RES, RUN)))
exp = [s["exposure_ms"] for s in d["sweep"]]
per = [s["period_us"] / 1000.0 for s in d["sweep"]]

fig, (a1, a2) = plt.subplots(1, 2, figsize=(9, 3.6))
for ax, lo, hi in ((a1, 0, 41), (a2, 11.5, 14.5)):
    xs = [lo + (hi - lo) * i / 400.0 for i in range(401)]
    ax.plot(xs, [predicted(x) for x in xs], color="0.55", lw=1.2, label="pre lab: 0.195 ms + max(12.688 ms, exposure)")
    ax.plot(exp, per, "o", ms=4, color="#1f5fa8", label="measured")
    ax.set_xlim(lo, hi)
    ax.set_xlabel("Exposure (ms)")
    ax.set_ylabel("Frame-to-frame period (ms)")
    ax.grid(True, color="0.9", lw=0.6)
a1.set_ylim(0, 42)
a1.set_title("1 ms to 40 ms", fontsize=10)
a1.legend(fontsize=8, loc="upper left", frameon=False)
a2.set_ylim(12.6, 14.8)
a2.axvline(12.714, color="#c0392b", lw=1, ls="--")
a2.text(12.76, 12.66, "knee 12.71 ms\n(Exp_time 978)", color="#c0392b", fontsize=8, va="bottom")
a2.set_title("Around the knee (0.25 ms steps)", fontsize=10)
fig.tight_layout()
out = os.path.join(RES, "postlab_3c_sweep.png")
fig.savefig(out, dpi=170)
print("wrote", out)
