import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif", "mathtext.fontset": "cm",
    "font.size": 10, "axes.labelsize": 10, "legend.fontsize": 8,
    "xtick.labelsize": 9, "ytick.labelsize": 9,
    "lines.linewidth": 1.4, "axes.grid": True, "grid.alpha": 0.3,
    "figure.figsize": (3.5, 2.6),
    "savefig.bbox": "tight", "savefig.dpi": 300,
})

COLORS = {
    "basak": "#c0392b",
    "ours": "#27ae60",
    "pal": "#2980b9",
    "pal_ours": "#8e44ad",
}
CYCLE = ["#c0392b", "#27ae60", "#2980b9", "#8e44ad", "#e67e22", "#2c3e50", "#16a085"]

LABELS = {
    "t": r"$t$ (s)", "ey": r"$e_y$ (m)", "ex": r"$e_x$ (m)", "d": r"$d$ (m)",
    "chi_t_i": r"$\tilde\chi_i$ (rad)", "chi_t_l": r"$\tilde\chi_l$ (rad)",
    "v_i": r"$v_i$ (m/s)", "u_i": r"$u_i$ (rad/s)", "omega": r"$\omega$ (rad/s)",
    "Delta": r"$\Delta$ (rad)", "den": r"speed-law denominator $Q$ (m)",
    "chidot_i": r"$\dot\chi_i$ (rad/s)",
}

FIG_DIR = "figures"


def save(fig, name):
    path = (name if os.path.dirname(name) else os.path.join(FIG_DIR, name)) + ".png"
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    print("saved", path)
