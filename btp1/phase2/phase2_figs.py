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

RES = "phase2/results"
OUT = "phase2/figures/report"

plt.rcParams.update({
    "axes.grid": True, "grid.alpha": 0.35, "grid.linestyle": "-",
    "legend.fontsize": 6.5, "legend.frameon": True, "legend.framealpha": 1.0,
    "legend.edgecolor": "0.3", "legend.fancybox": False,
    "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
})

GUARD = dict(color="#e01010", ls="-", lw=1.0)
EPSF = dict(color="#1f3fff", ls="--", lw=1.2)
REDS = ["#f4a3a3", "#e01010", "#7a0000"]
BLUES = ["#9fb4ff", "#1f3fff", "#001080"]
KD_STY = [dict(color="#9fb4ff", ls=":", lw=1.5), dict(color="#1f3fff", ls="--", lw=1.2), dict(color="#001080", ls="-", lw=1.0)]
LIM = dict(color="k", ls="--", lw=0.8)

rows = json.load(open("phase2/phase2_rows.json"))


def pick(fn, law, kd, eps):
    hit = [r for r in rows if r["file"] == fn and r["law"] == law and r["kd"] == kd and abs(r["eps"] - eps) < 1e-12]
    assert len(hit) == 1, (fn, law, kd, eps)
    m = loadmat(os.path.join(RES, hit[0]["tag"] + ".mat"), squeeze_me=True, simplify_cells=True)
    m["D"] = np.hypot(m["ex"], m["ey"])
    return m


def panel_label(ax, k, y):
    ax.text(0.5, y, f"({'abcd'[k]})", transform=ax.transAxes, ha="center", va="top", fontsize=8)


def speed_limits(ax):
    ax.axhline(P.V_MIN, **LIM, label=r"$V_{min}$, $V_{max}$")
    ax.axhline(P.V_MAX, **LIM)


EPS_G, EPS_F = P.EPS_DEN, P.EPS_FIX

fig, axs = plt.subplots(2, 2, figsize=(7.16, 5.4))
fig.subplots_adjust(hspace=0.5, wspace=0.28, left=0.08, right=0.98, top=0.98, bottom=0.11)
ax = axs.flat

g, e = pick("basak.py", "guard", 1.0, EPS_G), pick("basak.py", "epsfix", 1.0, EPS_F)
ax[0].plot(g["t"], g["v_i"], label=rf"guard, $\varepsilon$ = {EPS_G:g}", **GUARD)
ax[0].plot(e["t"], e["v_i"], label=rf"$\varepsilon$-fix, $\varepsilon$ = {EPS_F:g}", **EPSF)
speed_limits(ax[0])
ax[0].set_ylim(0, 9.5)
ax[0].set_xlabel("time (s)")
ax[0].set_ylabel(r"Follower speed $v_i$ (m/s)")
ax[0].set_title("basak.py (no $\\Delta$)", fontsize=8)
ax[0].legend(loc="lower right")

g, e = pick("run_ours.py", "guard", 1.0, EPS_G), pick("run_ours.py", "epsfix", 1.0, EPS_F)
ax[1].plot(g["t"], g["ex"], label=rf"guard, $\varepsilon$ = {EPS_G:g}", **GUARD)
ax[1].plot(e["t"], e["ex"], label=rf"$\varepsilon$-fix, $\varepsilon$ = {EPS_F:g}", **EPSF)
ax[1].axhline(0, color="0.4", lw=0.6)
ax[1].set_xlabel("time (s)")
ax[1].set_ylabel(r"Along-track error $e_x$ (m)")
ax[1].set_title("run_ours.py (with $\\Delta$)", fontsize=8)
ax[1].legend(loc="upper right")
ins = ax[1].inset_axes([0.42, 0.25, 0.53, 0.42])
ins.plot(g["t"], g["ex"], **GUARD)
ins.plot(e["t"], e["ex"], **EPSF)
ins.axhline(0, color="0.4", lw=0.6)
ins.set_xlim(10, 60)
ins.set_ylim(-0.02, 0.005)
ins.tick_params(labelsize=6)
ins.grid(True, alpha=0.3)

for col, eps in zip(REDS, (0.1, 0.01, 0.001)):
    m = pick("run_ours.py", "guard", 1.0, eps)
    ax[2].plot(m["t"], m["D"], color=col, ls="-", lw=1.1, label=rf"guard, $\varepsilon$ = {eps:g}")
for col, eps in zip(BLUES, (0.1, 0.05, 0.01)):
    m = pick("run_ours.py", "epsfix", 1.0, eps)
    ax[2].plot(m["t"], m["D"], color=col, ls="--", lw=1.1, label=rf"$\varepsilon$-fix, $\varepsilon$ = {eps:g}")
ax[2].set_yscale("log")
ax[2].set_xlabel("time (s)")
ax[2].set_ylabel(r"Formation error $D$ (m)")
ax[2].legend(loc="upper right", ncol=2)
ax[2].set_ylim(1e-4, 30)

g = pick("run_ours.py", "guard", 1.0, EPS_G)
ax[3].plot(g["t"], g["den"], label=r"denominator $Q$", **GUARD)
ax[3].axhspan(-EPS_G, EPS_G, color="0.85", lw=0)
ax[3].axhline(EPS_G, **LIM, label=rf"$\pm\varepsilon$ = $\pm${EPS_G:g} m")
ax[3].axhline(-EPS_G, **LIM)
ax[3].set_xlim(0, 20)
ax[3].set_xlabel("time (s)")
ax[3].set_ylabel(r"$Q = e_x\cos(\chi_i-\chi_l)+e_y\sin(\chi_i-\chi_l)$ (m)")
ax[3].set_title(rf"run_ours.py, guard, $\varepsilon$ = {EPS_G:g}", fontsize=8)
ax[3].legend(loc="upper right")
ins = ax[3].inset_axes([0.42, 0.18, 0.53, 0.4])
ins.plot(g["t"], g["den"], **GUARD)
ins.axhspan(-EPS_G, EPS_G, color="0.85", lw=0)
ins.axhline(EPS_G, **LIM)
ins.axhline(-EPS_G, **LIM)
ins.set_xlim(0, 20)
ins.set_ylim(-0.05, 0.05)
ins.tick_params(labelsize=6)
ins.grid(True, alpha=0.3)

for k, a in enumerate(ax):
    panel_label(a, k, -0.25)
save(fig, f"{OUT}/fig_p2_speed_law")

fig, axs = plt.subplots(1, 2, figsize=(7.16, 2.8))
fig.subplots_adjust(wspace=0.25, left=0.08, right=0.98, top=0.97, bottom=0.25)
for sty, kd in zip(KD_STY, (0.5, 1.0, 2.0)):
    m = pick("run_ours.py", "epsfix", kd, EPS_F)
    axs[0].plot(m["t"], m["D"], label=rf"$K_D$ = {kd:g}", **sty)
    axs[1].plot(m["t"], m["v_i"], label=rf"$K_D$ = {kd:g}", **sty)
axs[0].set_yscale("log")
axs[0].set_xlabel("time (s)")
axs[0].set_ylabel(r"Formation error $D$ (m)")
axs[0].legend(loc="upper right")
speed_limits(axs[1])
axs[1].set_ylim(0, 9.5)
axs[1].set_xlim(0, 20)
axs[1].set_xlabel("time (s)")
axs[1].set_ylabel(r"Follower speed $v_i$ (m/s)")
axs[1].legend(loc="upper right")
for k, a in enumerate(axs):
    panel_label(a, k, -0.27)
save(fig, f"{OUT}/fig_p2_kd_sweep")
