import glob
import json
import os
import subprocess
import sys
import numpy as np
from scipy.io import loadmat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, os.path.join(ROOT, "plotting"))
from plot_style import plt, save

RES = "phase1/results"
FIG_OVERVIEW = "phase1/figures/overview"
FIG_PAPER = "phase1/figures/paper"

A, L = 30.0, 300.0
C = {"arctan": "#c0392b", "arccos": "#2980b9"}


def find(pattern):
    hits = glob.glob(os.path.join(RES, pattern))
    if len(hits) != 1:
        raise SystemExit(f"{pattern}: expected 1 file, found {len(hits)}")
    return hits[0]


def load_raw(path):
    return loadmat(path, squeeze_me=True, simplify_cells=True)


# ---------- overview figures ----------

for r, lim in ((10, "0.1"), (20, "0.05"), (30, "0.0333")):
    pair = [find(f"basak_none_L-{f}_*rmin{r}_chi0_off5.mat") for f in ("arctan", "arccos")]
    for var, extra in (("d", ["--abs", "--log"]), ("kappa_l", ["--hline", lim, "0.157", "--xlim", "0", "20"])):
        name = "d" if var == "d" else "kappa"
        subprocess.run([sys.executable, "plotting/plot_compare.py", var, *pair, "--labels", "arctan", "arccos",
                        *extra, "--out", f"{FIG_OVERVIEW}/p1_{name}_rmin{r}"], check=True)

xs = np.linspace(-5, 300, 800)
fig, ax = plt.subplots(figsize=(7.16, 2.6))
ax.plot(xs, A * np.sin(2 * np.pi * xs / L), "k:", lw=1, label="reference path")
for fld in ("arctan", "arccos"):
    m = load_raw(find(f"basak_none_L-{fld}_*rmin20_chi0_off5.mat"))
    ax.plot(m["xl"], m["yl"], color=C[fld], label=f"leader, {fld}")
ax.set_xlabel(r"$x$ (m)"); ax.set_ylabel(r"$y$ (m)"); ax.set_aspect("equal"); ax.legend(fontsize=7)
save(fig, f"{FIG_OVERVIEW}/p1_xy_rmin20")

fig, ax = plt.subplots(figsize=(3.5, 2.6))
ax.set_xlim(-2, 60); ax.set_ylim(-32, 30)
for fld in ("arctan", "arccos"):
    m = load_raw(find(f"basak_none_L-{fld}_*rmin20_chi0_off5.mat"))
    ax.plot(m["xl"], m["yl"], color=C[fld], label=fld)
ax.plot(xs, A * np.sin(2 * np.pi * xs / L), "k:", lw=1, label="path")
ax.set_xlabel(r"$x$ (m)"); ax.set_ylabel(r"$y$ (m)"); ax.set_aspect("equal"); ax.legend(fontsize=7)
save(fig, f"{FIG_OVERVIEW}/p1_xy_zoom_rmin20")

rows = json.load(open(os.path.join(RES, "phase1_rows.json")))
A20 = {r["field"]: r for r in rows if r["part"] == "A" and r["rmin"] == 20.0}
B = [r for r in rows if r["part"] == "B"] + list(A20.values())
cases = [(y, h) for y in (15, 30, 60) for h in (0, 90, 180)]
fig, ax = plt.subplots(figsize=(7.16, 2.6))
x = np.arange(len(cases)); wbar = 0.38
for k, fld in enumerate(("arctan", "arccos")):
    ts = [next(r["leader_ts"] for r in B if r["field"] == fld and r["y0"] == y and r["h0"] == h) for y, h in cases]
    ax.bar(x + (k - 0.5) * wbar, ts, wbar, color=C[fld], label=fld)
ax.set_xticks(x, [f"{y} m\n{h}°" for y, h in cases], fontsize=8)
ax.set_ylabel("leader settling time (s)"); ax.set_xlabel(r"start offset $|y_{l0}|$ and heading $\chi_{l0}$"); ax.legend()
save(fig, f"{FIG_OVERVIEW}/p1_ic_settling")

# ---------- paper-style six-panel figures ----------

plt.rcParams.update({
    "axes.grid": True, "grid.alpha": 0.35, "grid.linestyle": "-",
    "legend.fontsize": 6.5, "legend.frameon": True, "legend.framealpha": 1.0,
    "legend.edgecolor": "0.3", "legend.fancybox": False,
    "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
})

T_MAX = 30.0
S_MAX = 150.0
STY3 = [dict(color="#1f3fff", ls="--", lw=1.2),
        dict(color="#e000e0", ls="-.", lw=1.2),
        dict(color="#e01010", ls=":", lw=1.5)]
STY2 = {"arccos": dict(color="#e01010", ls="--", lw=1.2),
        "arctan": dict(color="#1f3fff", ls="-.", lw=1.2)}
NAME = {"arccos": "arccos field", "arctan": "arctan field"}


def load(pattern):
    m = load_raw(find(pattern))
    v = float(m["params"]["V_GL"])
    dt = float(m["params"]["DT"])
    chi_d = np.unwrap(m["chi_l"] - m["chi_t_l"])
    m["course_deg"] = np.rad2deg(np.unwrap(m["chi_l"]))
    m["kappa_vf"] = np.gradient(chi_d, dt) / v
    m["kappa_path"] = m["omega"] / v
    m["s"] = v * m["t"]
    m["a_lat"] = v * m["omega"]
    return m


def run_a(field, rmin):
    return load(f"basak_none_L-{field}_*rmin{rmin}_chi0_off5.mat")


def run_b(field, y, h):
    if (y, h) == (30, 0):
        return run_a(field, 20)
    return load(f"basak_none_L-{field}_*rmin20_chi0_off5_y{y}_h{h}.mat")


def panel_label(ax, k):
    ax.text(0.5, -0.36, f"({'abcdef'[k]})", transform=ax.transAxes, ha="center", va="top", fontsize=8)


def limit_lines(ax, rmins):
    for r, st in rmins:
        for sgn in (1, -1):
            ax.axhline(sgn / r, color=st["color"], ls=(0, (6, 2, 1, 2)), lw=0.7)


def six_panel(runs, labels, styles, rmins, out):
    fig, axs = plt.subplots(3, 2, figsize=(7.16, 7.4))
    fig.subplots_adjust(hspace=0.62, wspace=0.28, left=0.08, right=0.98, top=0.98, bottom=0.08)
    ax = axs.flat

    xs = np.linspace(-5, max(m["xl"].max() for m in runs) + 5, 800)
    ax[0].plot(xs, A * np.sin(2 * np.pi * xs / L), "k-", lw=1.0, label="Desired path")
    for m, lab, st in zip(runs, labels, styles):
        ax[0].plot(m["xl"], m["yl"], label=lab, **st)
    starts = {(round(m["xl"][0], 3), round(m["yl"][0], 3)) for m in runs}
    for i, (x0, y0) in enumerate(starts):
        ax[0].plot(x0, y0, "o", ms=4, mfc="#e01010", mec="#e01010", ls="none",
                   label="Initial position" if i == 0 else None)
    ax[0].set_xlabel(r"$X$ (m)")
    ax[0].set_ylabel(r"$Y$ (m)")
    ylo = min(m["yl"].min() for m in runs) - 10
    ax[0].set_ylim(ylo, 35 + (35 - ylo) * 1.1)
    ax[0].legend(loc="upper right")
    ins = ax[0].inset_axes([0.13, 0.6, 0.33, 0.36])
    ins.plot(xs, A * np.sin(2 * np.pi * xs / L), "k-", lw=0.8)
    for m, st in zip(runs, styles):
        ins.plot(m["xl"], m["yl"], **{**st, "lw": 0.9})
    ins.set_xlim(-2, 70)
    ins.set_ylim(-max(abs(m["yl"][0]) for m in runs) - 8, 35)
    ins.tick_params(labelsize=5)
    ins.grid(True, alpha=0.3)

    for m, lab, st in zip(runs, labels, styles):
        ax[1].plot(m["t"], m["course_deg"], label=lab, **st)
    ax[1].set_xlim(0, T_MAX)
    ax[1].set_xlabel("time (s)")
    ax[1].set_ylabel("Course angle (deg)")
    ax[1].legend(loc="upper right")

    for m, lab, st in zip(runs, labels, styles):
        ax[2].plot(m["t"], m["d"], label=lab, **st)
    ax[2].set_xlim(0, T_MAX)
    ax[2].set_xlabel("time (s)")
    ax[2].set_ylabel(r"Cross-track error $d$ (m)")
    ax[2].legend(loc="lower right")
    ins = ax[2].inset_axes([0.45, 0.48, 0.5, 0.42])
    for m, st in zip(runs, styles):
        ins.plot(m["t"], m["d"], **{**st, "lw": 0.9})
    ins.axhspan(-0.2, 0.2, color="0.85", lw=0)
    ins.set_xlim(5, 20)
    ins.set_ylim(-1.5, 1.5)
    ins.tick_params(labelsize=5)
    ins.grid(True, alpha=0.3)

    for m, lab, st in zip(runs, labels, styles):
        ax[3].plot(m["d"], m["kappa_vf"], label=lab, **st)
    limit_lines(ax[3], rmins)
    ax[3].set_xlabel(r"Cross-track error $d$ (m)")
    ax[3].set_ylabel(r"$\kappa_{VF}$ (1/m)")
    ax[3].legend(loc="upper right")

    for m, lab, st in zip(runs, labels, styles):
        ax[4].plot(m["s"], m["kappa_path"], label=lab, **st)
    limit_lines(ax[4], rmins)
    ax[4].set_xlim(0, S_MAX)
    ax[4].set_xlabel("Path length (m)")
    ax[4].set_ylabel("Path curvature (1/m)")
    ax[4].legend(loc="upper right")

    for m, lab, st in zip(runs, labels, styles):
        ax[5].plot(m["t"], m["a_lat"], label=lab, **st)
    ax[5].set_xlim(0, T_MAX)
    ax[5].set_xlabel("time (s)")
    ax[5].set_ylabel(r"Lateral acceleration (m/s$^2$)")
    ax[5].legend(loc="upper right")

    for k, a in enumerate(ax):
        panel_label(a, k)
    save(fig, f"{FIG_PAPER}/{out}")


for field in ("arccos", "arctan"):
    runs = [run_a(field, r) for r in (10, 20, 30)]
    six_panel(runs, [rf"$r_{{min}}$ = {r} m" for r in (10, 20, 30)], STY3,
              [(r, st) for r, st in zip((10, 20, 30), STY3)], f"p1_paper_rmin_{field}")

for field in ("arccos", "arctan"):
    runs = [run_b(field, 30, h) for h in (0, 90, 180)]
    six_panel(runs, [rf"$\chi_{{l0}}$ = {h}$^\circ$" for h in (0, 90, 180)], STY3,
              [(20, dict(color="k"))], f"p1_paper_heading_{field}")

for r in (10, 20, 30):
    runs = [run_a(f, r) for f in ("arccos", "arctan")]
    six_panel(runs, [NAME[f] for f in ("arccos", "arctan")], [STY2[f] for f in ("arccos", "arctan")],
              [(r, dict(color="k"))], f"p1_paper_compare_rmin{r}")

get = {(r["field"], r["rmin"], r["y0"], r["h0"]): r for r in rows}
lines = ["| Scenario | Control effort (m²/s³), arccos | Control effort (m²/s³), arctan | Settling time (s), arccos | Settling time (s), arctan |",
         "| --- | --- | --- | --- | --- |"]
tcases = [(f"r_min = {r:g} m, y_l0 = −30 m, χ_l0 = 0°", r, 30, 0) for r in (10.0, 20.0, 30.0)]
tcases += [(f"r_min = 20 m, y_l0 = −{y} m, χ_l0 = {h}°", 20.0, y, h)
           for y in (15, 30, 60) for h in (0, 90, 180) if (y, h) != (30, 0)]
for name, r, y, h in tcases:
    c, t = get[("arccos", r, y, h)], get[("arctan", r, y, h)]
    lines.append(f"| {name} | {c['leader_effort']:.2f} | {t['leader_effort']:.2f} | "
                 f"{c['leader_ts']:.2f} | {t['leader_ts']:.2f} |")
open("phase1/table_paper.md", "w").write("\n".join(lines) + "\n")
print("saved phase1/table_paper.md")
