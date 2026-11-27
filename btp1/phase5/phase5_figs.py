import json
import os
import sys
import numpy as np
from scipy.io import loadmat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, os.path.join(ROOT, "plotting"))
from plot_style import plt, save

RES = "phase5/results"
OUT = "phase5/figures/report"

plt.rcParams.update({
    "axes.grid": True, "grid.alpha": 0.35,
    "legend.fontsize": 6.5, "legend.frameon": True, "legend.framealpha": 1.0,
    "legend.edgecolor": "0.3", "legend.fancybox": False,
    "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
})
COL = {"sign_pow": "#1f3fff", "sat": "#e01010"}
LS = {"sign_pow": "-.", "sat": "--"}
NAME = {"sign_pow": "sign–power", "sat": "saturation"}

rows = json.load(open("phase5/phase5_rows.json"))
runs = {(r["law"], r["gain"], r["noise_deg"]): loadmat(os.path.join(RES, r["tag"] + ".mat"), squeeze_me=True, simplify_cells=True)
        for r in rows}

fig, axs = plt.subplots(2, 2, figsize=(7.16, 5.4))
fig.subplots_adjust(hspace=0.5, wspace=0.3, left=0.08, right=0.98, top=0.98, bottom=0.11)
ax = axs.flat

for law in ("sign_pow", "sat"):
    for g, a in (("pi/8", 0.45), ("pi/4", 1.0), ("pi/2", 0.7)):
        m = runs[(law, g, 0.0)]
        ax[0].plot(m["t"], np.abs(m["chi_t_i"]), color=COL[law], ls=LS[law], lw=1.1 if g == "pi/4" else 0.8, alpha=a,
                   label=f"{NAME[law]}, gain {g.replace('pi', 'π')}")
ax[0].set_yscale("log")
ax[0].set_ylim(1e-7, 1)
ax[0].set_xlim(0, 20)
ax[0].set_xlabel("time (s)")
ax[0].set_ylabel(r"$|\tilde\chi_i|$ (rad), no noise")
ax[0].legend(loc="upper right", fontsize=5.8, ncol=2)

for law in ("sign_pow", "sat"):
    m = runs[(law, "pi/4", 0.0)]
    ax[1].plot(m["t"], np.abs(m["ey"]), color=COL[law], ls=LS[law], lw=1.1, label=f"{NAME[law]}, no noise")
    m = runs[(law, "pi/4", 1.0)]
    ax[1].plot(m["t"], np.abs(m["ey"]), color=COL[law], ls="-", lw=0.6, alpha=0.45, label=rf"{NAME[law]}, $\sigma$ = 1$^\circ$")
ax[1].axhline(0.2, color="0.4", lw=0.6, ls=":")
ax[1].set_yscale("log")
ax[1].set_ylim(1e-5, 10)
ax[1].set_xlim(0, 60)
ax[1].set_xlabel("time (s)")
ax[1].set_ylabel(r"$|e_y|$ (m)")
ax[1].legend(loc="upper right", fontsize=6)

for law in ("sign_pow", "sat"):
    m = runs[(law, "pi/4", 1.0)]
    ax[2].plot(m["t"], m["chidot_i"], color=COL[law], lw=0.6, alpha=0.8, label=rf"{NAME[law]}, $\sigma$ = 1$^\circ$")
m = runs[("sat", "pi/4", 0.0)]
ax[2].plot(m["t"], m["chidot_i"], color="k", lw=1.2, label="no noise")
ax[2].set_xlim(30, 35)
ax[2].set_ylim(-0.35, 0.35)
ax[2].set_xlabel("time (s)")
ax[2].set_ylabel(r"turn rate $\dot\chi_i$ (rad/s)")
ax[2].legend(loc="upper right", fontsize=6, ncol=1)

sig = [0.5, 1.0, 2.0]
a2 = ax[3].twinx()
for law in ("sign_pow", "sat"):
    jit = [next(r["jitter"] for r in rows if r["law"] == law and r["noise_deg"] == s) for s in sig]
    eff = [next(r["effort"] for r in rows if r["law"] == law and r["noise_deg"] == s) for s in sig]
    ax[3].plot(sig, jit, color=COL[law], marker="o", ms=3.5, lw=1.2, label=f"{NAME[law]}: jitter")
    a2.plot(sig, eff, color=COL[law], marker="s", ms=3.5, lw=1.0, ls=":", label=f"{NAME[law]}: effort")
ax[3].set_xlabel(r"heading noise $\sigma$ (deg)")
ax[3].set_ylabel("turn-rate jitter (rad/s)")
a2.set_ylabel("control effort")
a2.tick_params(labelsize=7)
a2.grid(False)
h1, l1 = ax[3].get_legend_handles_labels()
h2, l2 = a2.get_legend_handles_labels()
ax[3].legend(h1 + h2, l1 + l2, loc="upper left", fontsize=6)
ax[3].set_xticks(sig)

for k, a in enumerate(ax):
    a.text(0.5, -0.25, f"({'abcd'[k]})", transform=a.transAxes, ha="center", va="top", fontsize=8)
save(fig, f"{OUT}/fig_p5_reaching_law")
