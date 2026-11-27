import json
import os
import sys
import numpy as np
from scipy.io import loadmat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, os.path.join(ROOT, "plotting"))
sys.path.insert(0, os.path.join(ROOT, "controllers"))
from plot_style import plt, save
import params_nominal as P

RES = "final/results"
OUT = "final/figures/report"

plt.rcParams.update({
    "axes.grid": True, "grid.alpha": 0.35,
    "legend.fontsize": 6.5, "legend.frameon": True, "legend.framealpha": 1.0,
    "legend.edgecolor": "0.3", "legend.fancybox": False,
    "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
})
COL = {"baseline": "#e01010", "chosen": "#1f3fff"}
NAME = {"baseline": "Basak baseline", "chosen": "chosen combination"}

rows = json.load(open("final/final_rows.json"))
runs = {(r["controller"], r["case"]): loadmat(os.path.join(RES, r["tag"] + ".mat"), squeeze_me=True, simplify_cells=True)
        for r in rows}

fig, axs = plt.subplots(2, 2, figsize=(7.16, 5.4))
fig.subplots_adjust(hspace=0.5, wspace=0.28, left=0.08, right=0.98, top=0.98, bottom=0.11)
ax = axs.flat

xp = np.linspace(-5, 310, 800)
ax[0].plot(xp, P.A * np.sin(2 * np.pi * xp / P.L), color="k", lw=0.9, label="desired path")
for c in ("baseline", "chosen"):
    m = runs[(c, "nominal")]
    ax[0].plot(m["xl"], m["yl"], color=COL[c], lw=1.0, ls="-", label=f"leader, {NAME[c]}")
    ax[0].plot(m["xi"], m["yi"], color=COL[c], lw=0.9, ls="--", label=f"follower, {NAME[c]}")
ax[0].plot(P.XL0, P.YL0, "o", color="k", ms=3.5)
ax[0].set_xlabel(r"$X$ (m)")
ax[0].set_ylabel(r"$Y$ (m)")
ax[0].set_ylim(-110, 60)
ax[0].legend(loc="upper right", fontsize=5.8, ncol=1)
ins = ax[0].inset_axes([0.55, 0.07, 0.41, 0.28])
for c in ("baseline", "chosen"):
    m = runs[(c, "nominal")]
    xs = m["xl"] + P.GX * np.cos(m["chi_l"]) - P.GY * np.sin(m["chi_l"])
    ys = m["yl"] + P.GX * np.sin(m["chi_l"]) + P.GY * np.cos(m["chi_l"])
    ins.plot(xs, ys, color=COL[c], lw=2.2, alpha=0.25)
    ins.plot(m["xi"], m["yi"], color=COL[c], lw=0.9, ls="--")
ins.set_xlim(221, 229)
ins.set_ylim(-27.8, -26.6)
ins.set_title("slot (thick) vs follower, x ≈ 225 m", fontsize=5.5, pad=2)
ins.tick_params(labelsize=5)
ins.grid(alpha=0.3)

for c in ("baseline", "chosen"):
    m = runs[(c, "nominal")]
    ax[1].plot(m["t"], m["ey"], color=COL[c], lw=1.0, label=NAME[c])
ax[1].axhline(0, color="0.4", lw=0.6)
ax[1].set_xlim(0, 60)
ax[1].set_ylim(-0.6, 0.6)
ax[1].set_xlabel("time (s)")
ax[1].set_ylabel(r"$e_y$ (m)")
ax[1].legend(loc="lower left")

for c in ("baseline", "chosen"):
    m = runs[(c, "nominal")]
    ax[2].plot(m["t"], np.abs(m["ey"]), color=COL[c], lw=1.0, label=NAME[c])
    m = runs[(c, "noise1")]
    ax[2].plot(m["t"], np.abs(m["ey"]), color=COL[c], lw=0.5, alpha=0.4, label=rf"{NAME[c]}, $\sigma$ = 1$^\circ$")
ax[2].axhline(0.2, color="0.4", lw=0.6, ls=":")
ax[2].set_yscale("log")
ax[2].set_ylim(1e-6, 10)
ax[2].set_xlim(0, 60)
ax[2].set_xlabel("time (s)")
ax[2].set_ylabel(r"$|e_y|$ (m)")
ax[2].legend(loc="lower left", fontsize=5.8, ncol=2)

for c in ("baseline", "chosen"):
    m = runs[(c, "nominal")]
    ax[3].plot(m["t"], m["v_i"], color=COL[c], lw=0.7 if c == "baseline" else 1.1, label=NAME[c])
ax[3].axhline(P.V_MIN, color="0.4", lw=0.6, ls=":")
ax[3].axhline(P.V_MAX, color="0.4", lw=0.6, ls=":")
ax[3].set_xlim(0, 60)
ax[3].set_ylim(0, 8.8)
ax[3].set_xlabel("time (s)")
ax[3].set_ylabel(r"follower speed $v_i$ (m/s)")
ax[3].legend(loc="upper right")

for k, a in enumerate(ax):
    a.text(0.5, -0.25, f"({'abcd'[k]})", transform=a.transAxes, ha="center", va="top", fontsize=8)
save(fig, f"{OUT}/fig_final_comparison")
