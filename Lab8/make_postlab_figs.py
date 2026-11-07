# -*- coding: utf-8 -*-
"""
make_postlab_figs.py - the 4(c) comparison figure for the Lab 8 post lab.

    py -3.14 make_postlab_figs.py

Reads the 6 V and 4 V move logs in results/ and plots alpha = d(a_t) g / r against
time from GO, both runs on the same axes, with the 40 ms fit window shaded.
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
RUNS = [("6V_20261103_154455.csv", "6 V"), ("4V_20261103_154510.csv", "4 V")]
G0, R_IMU, J_ARM = 9.80665, 0.10, 0.3 * 0.28 ** 2 / 12

fig, ax = plt.subplots(figsize=(8, 3.6))
for name, lab in RUNS:
    d = np.genfromtxt(os.path.join(RES, name), delimiter=",", names=True)
    t, at, run = d["t_s"], d["a_tangential_g"], d["running"].astype(int)
    i0 = np.where(run == 1)[0][0]
    t = t - t[i0]
    at0 = at[(t < 0) & (t > -0.25)].mean()
    alpha = np.convolve((at - at0) * G0 / R_IMU, np.ones(4) / 4, mode="same")
    w = (t >= 0) & (t <= 0.040)
    a_mean = float(np.mean((at[w] - at0) * G0 / R_IMU))
    sel = (t > -0.05) & (t < 0.30)
    ax.plot(t[sel] * 1e3, alpha[sel], lw=1.2,
            label="%s: mean alpha (0-40 ms) = %.1f rad/s$^2$, tau = %.3f N m" % (lab, a_mean, a_mean * J_ARM))
ax.axvspan(0, 40, color="0.85", zorder=0)
ax.axhline(0, color="k", lw=0.6)
ax.set_xlabel("time from GO (ms)")
ax.set_ylabel("alpha = d(a_t) g / r  (rad/s$^2$)")
ax.set_title("Spin-up from rest at -62 deg, 60 pulses, 50 % duty (grey = 40 ms fit window)")
ax.legend(loc="upper right", fontsize=8)
fig.tight_layout()
p = os.path.join(RES, "postlab_4c_alpha.png")
fig.savefig(p, dpi=150)
print("wrote", p)

# 3(b): the ringing log around the stop, phi and the tangential axis only
d = np.genfromtxt(os.path.join(RES, "ring_20261103_154409.csv"), delimiter=",", names=True)
t, phi, at, run = d["t_s"], d["phi_rel_deg"], d["a_tangential_g"], d["running"].astype(int)
idx = np.where(run == 1)[0]
t_go, t_stop = t[idx[0]], t[idx[-1]]
final = phi[t > t[-1] - 0.25].mean()
sel = (t > t_go - 0.2) & (t < t_stop + 0.7)
fig, axs = plt.subplots(2, 1, figsize=(8, 3.8), sharex=True)
axs[0].plot((t[sel] - t_stop) * 1e3, phi[sel], lw=1)
axs[0].axhspan(final - 1.75, final + 1.75, color="g", alpha=0.2)
axs[0].set_ylim(-120, 140)
axs[0].set_ylabel("phi (deg)")
axs[0].set_title("Overshoot 19.5 deg; settled in the green band 0.40 s after the last pulse")
axs[1].plot((t[sel] - t_stop) * 1e3, at[sel], lw=1, color="C1")
axs[1].set_ylabel("a_tangential (g)")
axs[1].set_xlabel("time from the last pulse (ms)")
for a in axs:
    a.axvline(0, color="m", lw=0.8)
    a.axvline(400, color="k", lw=0.8, ls="--")
fig.tight_layout()
p = os.path.join(RES, "postlab_3b_ring.png")
fig.savefig(p, dpi=150)
print("wrote", p)
