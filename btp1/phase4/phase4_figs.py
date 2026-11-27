import json
import os
import sys
import numpy as np
from scipy.io import loadmat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, os.path.join(ROOT, "plotting"))
from plot_style import plt, save

RES = "phase4/results"
OUT = "phase4/figures/report"

plt.rcParams.update({
    "axes.grid": True, "grid.alpha": 0.35,
    "legend.fontsize": 6.5, "legend.frameon": True, "legend.framealpha": 1.0,
    "legend.edgecolor": "0.3", "legend.fancybox": False,
    "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
})
COL = {"arctan": "#1f3fff", "arccos": "#e01010"}
LS = {"arctan": "-.", "arccos": "--"}

rows = json.load(open("phase4/phase4_rows.json"))
runs = {(r["field"], r["rmin"], r["ey0"]): loadmat(os.path.join(RES, r["tag"] + ".mat"), squeeze_me=True, simplify_cells=True)
        for r in rows}

fig, axs = plt.subplots(2, 2, figsize=(7.16, 5.4))
fig.subplots_adjust(hspace=0.5, wspace=0.28, left=0.08, right=0.98, top=0.98, bottom=0.11)
ax = axs.flat

for f in ("arctan", "arccos"):
    for ey0, lw in ((5.0, 1.2), (15.0, 0.9)):
        m = runs[(f, 20.0, ey0)]
        ax[0].plot(m["t"], np.abs(m["ey"]), color=COL[f], ls=LS[f], lw=lw, alpha=1 if ey0 == 5 else 0.55,
                   label=f"{f}, $e_{{y0}}$ = {ey0:g} m")
ax[0].axhline(0.2, color="0.4", lw=0.6, ls=":")
ax[0].set_yscale("log")
ax[0].set_ylim(1e-5, 30)
ax[0].set_xlim(0, 20)
ax[0].set_xlabel("time (s)")
ax[0].set_ylabel(r"$|e_y|$ (m)")
ax[0].legend(loc="upper right", fontsize=6)

labels = ["$r_{\\min}$ = 10 m", "20 m", "30 m", "20 m,\n15 m start"]
keys = [(10.0, 5.0), (20.0, 5.0), (30.0, 5.0), (20.0, 15.0)]
x = np.arange(len(keys))
for j, f in enumerate(("arctan", "arccos")):
    ts = [next(r["ts"] for r in rows if r["field"] == f and r["rmin"] == k[0] and r["ey0"] == k[1]) for k in keys]
    ax[1].bar(x + (j - 0.5) * 0.38, ts, 0.38, color=COL[f], label=f"{f} field")
ax[1].set_xticks(x)
ax[1].set_xticklabels(labels)
ax[1].set_ylabel("follower settling time (s)")
ax[1].legend(loc="upper left")

for f in ("arctan", "arccos"):
    for rmin, alpha in ((10.0, 1.0), (30.0, 0.5)):
        m = runs[(f, rmin, 5.0)]
        ax[2].plot(m["t"], np.abs(m["ey"]), color=COL[f], ls=LS[f], lw=1.1, alpha=alpha,
                   label=f"{f}, $r_{{\\min}}$ = {rmin:g} m")
ax[2].axhline(0.2, color="0.4", lw=0.6, ls=":")
ax[2].set_yscale("log")
ax[2].set_ylim(1e-5, 10)
ax[2].set_xlim(0, 20)
ax[2].set_xlabel("time (s)")
ax[2].set_ylabel(r"$|e_y|$ (m), $e_{y0}$ = 5 m")
ax[2].legend(loc="upper right", fontsize=6)

m0 = runs[("arctan", 20.0, 15.0)]
clip = (m0["v_i"] <= 0.5 + 1e-9) | (m0["v_i"] >= 8.0 - 1e-9)
ax[3].fill_between(m0["t"], -0.4, 0.4, where=clip, color="0.85", lw=0, label="speed at a limit")
for f in ("arctan", "arccos"):
    m = runs[(f, 20.0, 15.0)]
    ax[3].plot(m["t"], m["chidot_i"] / m["v_i"], color=COL[f], ls=LS[f], lw=1.2, label=f"{f} field")
ax[3].axhline(0.05, color="k", lw=0.8, ls=(0, (6, 2, 1, 2)), label=r"$\pm 1/r_{\min}$")
ax[3].axhline(-0.05, color="k", lw=0.8, ls=(0, (6, 2, 1, 2)))
ax[3].set_xlim(0, 12)
ax[3].set_ylim(-0.4, 0.4)
ax[3].set_xlabel("time (s)")
ax[3].set_ylabel(r"flown curvature $\dot\chi_i / v_i$ (1/m)")
ax[3].legend(loc="upper right")

for k, a in enumerate(ax):
    a.text(0.5, -0.25, f"({'abcd'[k]})", transform=a.transAxes, ha="center", va="top", fontsize=8)
save(fig, f"{OUT}/fig_p4_follower_field")
