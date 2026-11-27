import os
import sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, os.path.join(ROOT, "plotting"))
sys.path.insert(0, os.path.join(ROOT, "controllers"))
from plot_style import plt, save
import params_nominal as P

OUT = "carousel/figures/report"
W_DEMO = 0.5

plt.rcParams.update({
    "legend.fontsize": 6.5, "legend.frameon": True, "legend.framealpha": 1.0,
    "legend.edgecolor": "0.3", "legend.fancybox": False,
    "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
})


def arrow(ax, p, v, color, label, lw=1.6, ls="-"):
    ax.annotate("", xy=(p[0] + v[0], p[1] + v[1]), xytext=p,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, ls=ls, mutation_scale=10))
    ax.plot([], [], color=color, lw=lw, ls=ls, label=label)


def vector_panel(ax, w):
    gx, gy, v = P.GX, P.GY, P.V_GL
    fwd, side = v - w * gy, w * gx
    delta, vreq = np.arctan2(side, fwd), np.hypot(fwd, side)
    ax.plot([0, gx], [0, gy], color="0.5", lw=0.8, ls="--")
    ax.text(gx / 2 + 0.15, gy / 2 - 0.35, r"$G$", fontsize=8)
    ax.plot(0, 0, "o", color="#1f3fff", ms=5)
    ax.plot(gx, gy, "s", color="#27ae60", ms=5)
    ax.text(-0.3, -0.55, "leader", fontsize=7, ha="center")
    ax.text(gx + 0.1, gy - 0.55, "slot", fontsize=7)

    s = 1.0
    p = np.array([gx, gy])
    arrow(ax, (0, 0), (s * v, 0), "#1f3fff", r"leader velocity $v_{gl}$")
    arrow(ax, p, (s * v, 0), "0.55", r"$v_{gl}$ at the slot", lw=1.0, ls="--")
    arrow(ax, p, (s * fwd, 0), "k", r"forward part $v_{gl}-\omega g_y$", lw=1.2)
    arrow(ax, p + (s * fwd, 0), (0, s * side), "#e67e22", r"sideways part $\omega g_x$", lw=1.4)
    arrow(ax, p, (s * fwd, s * side), "#27ae60", r"required velocity $v_{req}$", lw=1.8)

    rr = 1.6
    a = np.linspace(0, delta, 40)
    ax.plot(p[0] + rr * np.cos(a), p[1] + rr * np.sin(a), color="#27ae60", lw=0.9)
    ax.text(p[0] + rr + 0.1, p[1] + 0.2, r"$\Delta$", fontsize=9, color="#27ae60")

    ax.set_aspect("equal")
    ax.set_xlim(-1.0, 9.0)
    ax.set_ylim(-1.0, 7.5)
    ax.set_xlabel(r"leader body $x_L$ (m)")
    ax.set_ylabel(r"leader body $y_L$ (m)")
    ax.legend(loc="upper left", fontsize=6)
    ax.text(0.98, 0.17, rf"$\omega$ = {w:g} rad/s (illustrative)" "\n"
            rf"$\Delta$ = {np.rad2deg(delta):.1f}$^\circ$, $v_{{req}}$ = {vreq:.2f} m/s",
            transform=ax.transAxes, va="bottom", ha="right", fontsize=6.5,
            bbox=dict(fc="white", ec="0.3", lw=0.6))
    return delta, vreq


fig, ax = plt.subplots(figsize=(4.2, 3.4))
d_demo, v_demo = vector_panel(ax, W_DEMO)
save(fig, f"{OUT}/carousel_vector_diagram")

lines = [
    "carousel_vector_diagram",
    f"Carousel effect in the leader body frame, slot G = ({P.GX:g}, {P.GY:g}) m, v_gl = {P.V_GL:g} m/s, "
    f"illustrative turn rate omega = {W_DEMO:g} rad/s: the slot moves forward at v_gl - omega g_y and sideways at "
    f"omega g_x, so the follower must fly at Delta = {np.rad2deg(d_demo):.1f} deg to the leader's course at "
    f"v_req = {v_demo:.2f} m/s.",
]
os.makedirs(OUT, exist_ok=True)
open(f"{OUT}/captions.txt", "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
