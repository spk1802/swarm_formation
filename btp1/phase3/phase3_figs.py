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

RES = "phase3/results"
OUT = "phase3/figures/report"

plt.rcParams.update({
    "axes.grid": True, "grid.alpha": 0.35,
    "legend.fontsize": 6.5, "legend.frameon": True, "legend.framealpha": 1.0,
    "legend.edgecolor": "0.3", "legend.fancybox": False,
    "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
})

STY = {
    "uncorrected": dict(color="#e01010", ls="-", lw=1.0, label=r"uncorrected ($\Delta = 0$)"),
    "pointwise": dict(color="#e67e22", ls="--", lw=1.2, label=r"pointwise ($\Delta$)"),
    "full": dict(color="#1f3fff", ls="-.", lw=1.2, label=r"full ($\Delta$, $\dot\Delta$)"),
}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


rows = json.load(open("phase3/phase3_rows.json"))
runs = {}
for r in rows:
    m = loadmat(os.path.join(RES, r["tag"] + ".mat"), squeeze_me=True, simplify_cells=True)
    runs[r["level"]] = m

fig, axs = plt.subplots(2, 2, figsize=(7.16, 5.4))
fig.subplots_adjust(hspace=0.5, wspace=0.28, left=0.08, right=0.98, top=0.98, bottom=0.11)
ax = axs.flat

for lv, m in runs.items():
    ax[0].plot(m["t"], m["ey"], **STY[lv])
    ax[1].plot(m["t"], np.abs(m["ey"]), **STY[lv])
ax[0].axhline(0, color="0.4", lw=0.6)
ax[0].set_xlim(0, 60)
ax[0].set_ylim(-0.6, 0.6)
ax[0].set_xlabel("time (s)")
ax[0].set_ylabel(r"Lateral error $e_y$ (m)")
ax[0].legend(loc="lower right")

ax[1].set_yscale("log")
ax[1].set_ylim(1e-6, 10)
ax[1].set_xlim(0, 60)
ax[1].set_xlabel("time (s)")
ax[1].set_ylabel(r"$|e_y|$ (m)")
ax[1].legend(loc="upper right")

m = runs["full"]
pred = np.arctan2(m["omega"] * P.GX, P.V_GL - m["omega"] * P.GY)
ax[2].plot(m["t"], np.rad2deg(pred), color="k", lw=2.4, alpha=0.3, label=r"predicted $\Delta$ from $\omega(t)$")
for lv, mm in runs.items():
    ax[2].plot(mm["t"], np.rad2deg(wrap(mm["chi_i"] - mm["chi_l"])), **STY[lv])
ax[2].axhline(0, color="0.4", lw=0.6)
ax[2].set_xlim(10, 60)
ax[2].set_ylim(-3.2, 3.2)
ax[2].set_xlabel("time (s)")
ax[2].set_ylabel(r"$\chi_i - \chi_l$ (deg)")
ax[2].legend(loc="upper left", fontsize=6)

m = runs["full"]
xs = m["xl"] + P.GX * np.cos(m["chi_l"]) - P.GY * np.sin(m["chi_l"])
ys = m["yl"] + P.GX * np.sin(m["chi_l"]) + P.GY * np.cos(m["chi_l"])
ax[3].plot(xs, ys, color="k", lw=2.4, alpha=0.3, label="slot")
for lv, mm in runs.items():
    ax[3].plot(mm["xi"], mm["yi"], **STY[lv])
xc = 225.0
w = np.abs(xs - xc) < 6
yc = ys[w].mean()
ax[3].set_xlim(xc - 5, xc + 5)
ax[3].set_ylim(yc - 1.8, yc + 1.8)
ax[3].set_xlabel(r"$X$ (m)")
ax[3].set_ylabel(r"$Y$ (m)")
ax[3].set_title("sharpest bend (x ≈ 225 m)", fontsize=8)
ax[3].legend(loc="upper right", fontsize=6)

for k, a in enumerate(ax):
    a.text(0.5, -0.25, f"({'abcd'[k]})", transform=a.transAxes, ha="center", va="top", fontsize=8)
save(fig, f"{OUT}/fig_p3_heading_correction")
