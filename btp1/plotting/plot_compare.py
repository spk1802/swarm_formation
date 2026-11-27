import argparse
import os
import numpy as np
from scipy.io import loadmat
from plot_style import plt, save, COLORS, CYCLE, LABELS

DERIVED = {
    "D": (lambda m: np.hypot(m["ex"], m["ey"]), r"$D$ (m)"),
    "kappa_i": (lambda m: m["chidot_i"] / m["v_i"], r"$\dot\chi_i / v_i$ (1/m)"),
    "kappa_l": (lambda m: m["omega"] / m["params"]["V_GL"], r"$\dot\chi_l / v_{gl}$ (1/m)"),
    "rel": (lambda m: (m["chi_i"] - m["chi_l"] + np.pi) % (2 * np.pi) - np.pi, r"$\chi_i - \chi_l$ (rad)"),
}


def load(path):
    raw = loadmat(path, squeeze_me=True, simplify_cells=True)
    return {k: v for k, v in raw.items() if not k.startswith("__")}


p = argparse.ArgumentParser()
p.add_argument("var")
p.add_argument("files", nargs="+")
p.add_argument("--labels", nargs="+")
p.add_argument("--log", action="store_true")
p.add_argument("--abs", action="store_true")
p.add_argument("--xlim", nargs=2, type=float)
p.add_argument("--ylim", nargs=2, type=float)
p.add_argument("--hline", nargs="+", type=float)
p.add_argument("--overlay", help="second variable drawn dashed in the same colour, e.g. Delta")
p.add_argument("--out", required=True)
a = p.parse_args()

runs = [load(f) for f in a.files]
labels = a.labels or [os.path.splitext(os.path.basename(f))[0] for f in a.files]

if a.var == "xy":
    fig, ax = plt.subplots(figsize=(7.16, 2.6))
    m0 = runs[0]
    xs = np.linspace(min(m["xl"].min() for m in runs) - 5, max(m["xl"].max() for m in runs) + 5, 800)
    A, L = m0["params"]["A"], m0["params"]["L"]
    ax.plot(xs, A * np.sin(2 * np.pi * xs / L), "k:", lw=1, label="path")
    ax.plot(m0["xl"], m0["yl"], color="0.4", lw=1.2, label="leader")
    for i, (m, lab) in enumerate(zip(runs, labels)):
        c = COLORS.get(str(m.get("name")), CYCLE[i % len(CYCLE)])
        ax.plot(m["xi"], m["yi"], "--", color=c, lw=1.0, label=lab)
    ax.set_xlabel(r"$x$ (m)")
    ax.set_ylabel(r"$y$ (m)")
    ax.set_aspect("equal")
else:
    fig, ax = plt.subplots()
    for i, (m, lab) in enumerate(zip(runs, labels)):
        c = COLORS.get(str(m.get("name")), CYCLE[i % len(CYCLE)]) if len(set(r.get("name") for r in runs)) == len(runs) else CYCLE[i % len(CYCLE)]
        if a.var in DERIVED:
            y, ylab = DERIVED[a.var][0](m), DERIVED[a.var][1]
        else:
            y, ylab = m[a.var], LABELS.get(a.var, a.var)
        y = np.abs(y) if a.abs else y
        ax.plot(m["t"], y, color=c, label=lab)
        if a.overlay:
            ax.plot(m["t"], m[a.overlay], "--", color=c, lw=1.0)
    ax.set_xlabel(LABELS["t"])
    ax.set_ylabel(("|" + ylab.split(" (")[0] + "|" + " (" + ylab.split(" (")[1]) if a.abs and " (" in ylab else ylab)
    if a.log:
        ax.set_yscale("log")
for h in a.hline or []:
    ax.axhline(h, color="k", ls="--", lw=0.8)
if a.xlim:
    ax.set_xlim(*a.xlim)
if a.ylim:
    ax.set_ylim(*a.ylim)
ax.legend()
save(fig, a.out)
