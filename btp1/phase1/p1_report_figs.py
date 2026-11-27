import glob
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
import params as P

RES = "phase1/results"
OUT = "phase1/figures/report"

plt.rcParams.update({
    "axes.grid": True, "grid.alpha": 0.35, "grid.linestyle": "-",
    "legend.fontsize": 6.5, "legend.frameon": True, "legend.framealpha": 1.0,
    "legend.edgecolor": "0.3", "legend.fancybox": False,
    "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
})

A, L = 30.0, 300.0
T_MAX = 30.0
S_MAX = 150.0
INSET_TICK = 6
STY3 = [dict(color="#1f3fff", ls="--", lw=1.2),
        dict(color="#e000e0", ls="-.", lw=1.2),
        dict(color="#e01010", ls=":", lw=1.5)]
STY2 = {"arccos": dict(color="#e01010", ls="--", lw=1.2),
        "arctan": dict(color="#1f3fff", ls="-.", lw=1.2)}
NAME = {"arccos": "arccos field", "arctan": "arctan field"}
LIM_LS = (0, (6, 2, 1, 2))
SAT_K = P.BETA / P.V_GL


def find(pattern):
    hits = glob.glob(os.path.join(RES, pattern))
    if len(hits) != 1:
        raise SystemExit(f"{pattern}: expected 1 file, found {len(hits)}")
    return hits[0]


def load(pattern):
    m = loadmat(find(pattern), squeeze_me=True, simplify_cells=True)
    v = float(m["params"]["V_GL"])
    dt = float(m["params"]["DT"])
    assert abs(v - P.V_GL) < 1e-12 and abs(float(m["params"]["BETA"]) - P.BETA) < 1e-12
    chi_d = np.unwrap(m["chi_l"] - m["chi_t_l"])
    m["kappa_vf"] = np.gradient(chi_d, dt) / v
    m["kappa_path"] = m["omega"] / v
    m["s"] = v * m["t"]
    return m


def run_a(field, rmin):
    return load(f"basak_none_L-{field}_*rmin{rmin}_chi0_off5.mat")


def run_b(field, y, h):
    if (y, h) == (30, 0):
        return run_a(field, 20)
    return load(f"basak_none_L-{field}_*rmin20_chi0_off5_y{y}_h{h}.mat")


def panel_label(ax, k, y=-0.3):
    ax.text(0.5, y, f"({'abcdef'[k]})", transform=ax.transAxes, ha="center", va="top", fontsize=8)


def limit_lines(ax, rmins, label=None):
    for r, st in rmins:
        for sgn in (1, -1):
            ax.axhline(sgn / r, color=st["color"], ls=LIM_LS, lw=0.7,
                       label=label if (label and sgn == 1) else None)


def sat_lines(ax):
    for sgn in (1, -1):
        ax.axhline(sgn * SAT_K, color="0.35", ls=":", lw=1.0,
                   label=r"$\beta/v_{gl}$" if sgn == 1 else None)


def panel_xy(ax, runs, labels, styles, inset_xlim=None):
    xs = np.linspace(-5, max(m["xl"].max() for m in runs) + 5, 800)
    ax.plot(xs, A * np.sin(2 * np.pi * xs / L), "k-", lw=1.0, label="Desired path")
    for m, lab, st in zip(runs, labels, styles):
        ax.plot(m["xl"], m["yl"], label=lab, **st)
    starts = {(round(m["xl"][0], 3), round(m["yl"][0], 3)) for m in runs}
    for i, (x0, y0) in enumerate(starts):
        ax.plot(x0, y0, "o", ms=4, mfc="#e01010", mec="#e01010", ls="none",
                label="Initial position" if i == 0 else None)
    ax.set_xlabel(r"$X$ (m)")
    ax.set_ylabel(r"$Y$ (m)")
    ylo = min(m["yl"].min() for m in runs) - 10
    ax.set_ylim(ylo, 35 + (35 - ylo) * 1.1)
    ax.legend(loc="upper right")
    ins = ax.inset_axes([0.13, 0.6, 0.33, 0.36])
    ins.plot(xs, A * np.sin(2 * np.pi * xs / L), "k-", lw=0.8)
    for m, st in zip(runs, styles):
        ins.plot(m["xl"], m["yl"], **{**st, "lw": 0.9})
    ins.set_xlim(*(inset_xlim or (-2, 70)))
    ins.set_ylim(-max(abs(m["yl"][0]) for m in runs) - 8, 35)
    ins.tick_params(labelsize=INSET_TICK)
    ins.grid(True, alpha=0.3)


def panel_d(ax, runs, labels, styles):
    for m, lab, st in zip(runs, labels, styles):
        ax.plot(m["t"], m["d"], label=lab, **st)
    ax.set_xlim(0, T_MAX)
    ax.set_xlabel("time (s)")
    ax.set_ylabel(r"Cross-track error $d$ (m)")
    ax.legend(loc="lower right")
    ins = ax.inset_axes([0.45, 0.48, 0.5, 0.42])
    for m, st in zip(runs, styles):
        ins.plot(m["t"], m["d"], **{**st, "lw": 0.9})
    ins.axhspan(-0.2, 0.2, color="0.85", lw=0)
    ins.set_xlim(5, 20)
    ins.set_ylim(-1.5, 1.5)
    ins.tick_params(labelsize=INSET_TICK)
    ins.grid(True, alpha=0.3)


def panel_kvf(ax, runs, labels, styles, rmins, lim_label=None):
    for m, lab, st in zip(runs, labels, styles):
        ax.plot(m["d"], m["kappa_vf"], label=lab, **st)
    limit_lines(ax, rmins, lim_label)
    ax.set_xlabel(r"Cross-track error $d$ (m)")
    ax.set_ylabel(r"$\kappa_{VF}$ (1/m)")
    ax.legend(loc="upper right")


def panel_kpath(ax, runs, labels, styles, rmins):
    for m, lab, st in zip(runs, labels, styles):
        ax.plot(m["s"], m["kappa_path"], label=lab, **st)
    limit_lines(ax, rmins, r"$\pm 1/r_{min}$")
    sat_lines(ax)
    ax.set_xlim(0, S_MAX)
    ax.set_xlabel("Path length (m)")
    ax.set_ylabel("Flown curvature (1/m)")
    ax.legend(loc="upper right")


# fig_p1_fields_rmin20
fields = ("arccos", "arctan")
runs = [run_a(f, 20) for f in fields]
labs = [NAME[f] for f in fields]
sty = [STY2[f] for f in fields]
fig, axs = plt.subplots(2, 2, figsize=(7.16, 5.4))
fig.subplots_adjust(hspace=0.5, wspace=0.28, left=0.08, right=0.98, top=0.98, bottom=0.11)
ax = axs.flat
panel_xy(ax[0], runs, labs, sty)
panel_d(ax[1], runs, labs, sty)
panel_kvf(ax[2], runs, labs, sty, [(20, dict(color="k"))], r"$\pm 1/r_{min}$")
panel_kpath(ax[3], runs, labs, sty, [(20, dict(color="k"))])
for k, a in enumerate(ax):
    panel_label(a, k, -0.25)
save(fig, f"{OUT}/fig_p1_fields_rmin20")

# fig_p1_rmin_sweep
RM = (10, 20, 30)
rl = [rf"$r_{{min}}$ = {r} m" for r in RM]
fig, axs = plt.subplots(1, 3, figsize=(7.16, 2.7))
fig.subplots_adjust(wspace=0.38, left=0.07, right=0.99, top=0.9, bottom=0.27)
panel_d(axs[0], [run_a("arctan", r) for r in RM], rl, STY3)
for k, f in enumerate(("arctan", "arccos"), 1):
    panel_kvf(axs[k], [run_a(f, r) for r in RM], rl, STY3, list(zip(RM, STY3)))
    axs[k].set_title(f, fontsize=9)
    axs[k].legend(loc="upper left", bbox_to_anchor=(0.0, 0.93))
for k, a in enumerate(axs):
    panel_label(a, k, -0.3)
save(fig, f"{OUT}/fig_p1_rmin_sweep")

# fig_p1_heading_sweep
HD = (0, 90, 180)
runs = [run_b("arctan", 30, h) for h in HD]
hl = [rf"$\chi_{{l0}}$ = {h}$^\circ$" for h in HD]
xmin = min(m["xl"].min() for m in runs)
fig, axs = plt.subplots(1, 2, figsize=(7.16, 2.9))
fig.subplots_adjust(wspace=0.25, left=0.08, right=0.98, top=0.97, bottom=0.25)
panel_xy(axs[0], runs, hl, STY3, inset_xlim=(xmin - 2, 60))
panel_kpath(axs[1], runs, hl, STY3, [(20, dict(color="k"))])
for k, a in enumerate(axs):
    panel_label(a, k, -0.27)
save(fig, f"{OUT}/fig_p1_heading_sweep")

# fig_p1_ic_settling (same data and layout as the overview bar chart, field colours as above)
rows = json.load(open(os.path.join(RES, "phase1_rows.json")))
A20 = {r["field"]: r for r in rows if r["part"] == "A" and r["rmin"] == 20.0}
B = [r for r in rows if r["part"] == "B"] + list(A20.values())
cases = [(y, h) for y in (15, 30, 60) for h in (0, 90, 180)]
fig, ax = plt.subplots(figsize=(7.16, 2.6))
x = np.arange(len(cases)); wbar = 0.38
for k, fld in enumerate(("arctan", "arccos")):
    ts = [next(r["leader_ts"] for r in B if r["field"] == fld and r["y0"] == y and r["h0"] == h) for y, h in cases]
    ax.bar(x + (k - 0.5) * wbar, ts, wbar, color=STY2[fld]["color"], label=NAME[fld])
ax.set_xticks(x, [f"{y} m\n{h}°" for y, h in cases], fontsize=8)
ax.set_ylabel("leader settling time (s)"); ax.set_xlabel(r"start offset $|y_{l0}|$ and heading $\chi_{l0}$")
ax.legend(loc="upper left")
save(fig, f"{OUT}/fig_p1_ic_settling")
