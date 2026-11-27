import os
import sys
import numpy as np
import pandas as pd
from scipy.io import loadmat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, os.path.join(ROOT, "plotting"))
from plot_style import plt, save

LOGS = "sitl/logs"
OUT = "sitl/figures/report"
PY = "final/results"
PY_BASE = "basak_none_L-arctan_guard_sign_pow_rmin20_chi0_off5_final_baseline_nominal.mat"
PY_CHOSEN = "pal_ours_full_L-arccos_epsfix_sat_rmin20_chi0_off5_final_chosen_nominal.mat"
RUNS = {
    "baseline": ("baseline", "#e01010"),
    "chosen": ("chosen", "#1f3fff"),
    "chosen_repeat": ("chosen (repeat)", "#7f8cff"),
    "chosen_lead": ("chosen + velocity lead", "#1a9850"),
    "chosen_course": ("chosen + course feedback", "#9e9e9e"),
}
W0, W1 = 20.0, 88.0

plt.rcParams.update({
    "axes.grid": True, "grid.alpha": 0.35,
    "legend.fontsize": 6.5, "legend.frameon": True, "legend.framealpha": 1.0,
    "legend.edgecolor": "0.3", "legend.fancybox": False,
    "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
})


def load(name):
    d = pd.read_csv(os.path.join(LOGS, name + ".csv"))
    t = d.t.values
    d["tt"] = t - t[np.argmax(d.leader_speed.values > 1e-3)]
    return d[d.tt >= -1.0].reset_index(drop=True)


def leader_d(d, A=30.0, L=300.0):
    k = 2 * np.pi / L
    x, y = d.leader_x.values, d.leader_y.values
    u = x.copy()
    for _ in range(20):
        f, fp, fpp = A * np.sin(k * u), A * k * np.cos(k * u), -A * k * k * np.sin(k * u)
        u -= ((u - x) + (f - y) * fp) / (1 + fp ** 2 + (f - y) * fpp)
    cp = np.arctan(A * k * np.cos(k * u))
    return -(x - u) * np.sin(cp) + (y - A * np.sin(k * u)) * np.cos(cp)


def settle(t, e, thr=0.2):
    out = np.nonzero((np.abs(e) > thr) & (t > 0))[0]
    return float(t[out[-1] + 1]) if len(out) and out[-1] + 1 < len(t) else float("nan")


def parts(d):
    E = {i: d[f"f{i}_eyi"].values for i in range(1, 5)}
    front, rear = (E[1] + E[3]) / 2, (E[2] + E[4]) / 2
    return (front - rear) / 2, (front + rear) / 2


runs = {k: load(k) for k in RUNS}

rows = []
for k, d in runs.items():
    t = d.tt.values
    w = (t > W0) & (t < min(W1, t[-1]))
    car, lag = parts(d)
    om = d.leader_chidot.values
    r = {"run": k, "flight": t[-1], "leader_ts": settle(t, leader_d(d)),
         "carousel": np.sqrt(np.mean(car[w] ** 2)), "lag": np.sqrt(np.mean(lag[w] ** 2)),
         "lag_slope": np.polyfit(om[w], lag[w], 1)[0]}
    for i in range(1, 5):
        ey = d[f"f{i}_eyi"].values
        r[f"ts{i}"] = settle(t, ey)
        r[f"rms{i}"] = np.sqrt(np.mean(ey[w] ** 2))
        r[f"max{i}"] = np.abs(ey[w]).max()
    r["D"] = np.mean([d[f"f{i}_Di"].values[w].mean() for i in range(1, 5)])
    r["vstd"] = np.mean([d[f"f{i}_speed"].values[(t > 12) & (t < 30)].std() for i in range(1, 5)])
    rows.append(r)

hdr = ("| Run | Flight (s) | Leader settling (s) | Settling d1–d4 (s) | RMS e_y d1–d4 (m) | Max \\|e_y\\| d1–d4 (m) | "
       "Mean D (m) | Carousel part (m) | Lag part (m) | Lag slope (m per rad/s) | Speed std 12–30 s (m/s) |")
lines = [hdr, "|" + " --- |" * 11]
for r in rows:
    f = lambda key, fmt: "/".join(("–" if np.isnan(r[f"{key}{i}"]) else format(r[f"{key}{i}"], fmt)) for i in range(1, 5))
    lines.append(f"| {r['run']} | {r['flight']:.0f} | {r['leader_ts']:.2f} | {f('ts', '.1f')} | {f('rms', '.3f')} | {f('max', '.3f')} | "
                 f"{r['D']:.3f} | {r['carousel']:.3f} | {r['lag']:.3f} | {r['lag_slope']:.2f} | {r['vstd']:.2f} |")
note = (f"\nAll metrics for {W0:g} < t < {W1:g} s after the leader starts moving (course run: until it was stopped at 28 s), "
        "except settling (|e| stays within 0.2 m) and speed std (12–30 s). Drones: d1 (3,3), d2 (−3,3), d3 (3,−3), d4 (−3,−3). "
        "Carousel part = (front − rear)/2 of e_y, lag part = mean e_y of all four. Speed is the measured ground speed.\n")
open("sitl/sitl_table.md", "w").write("\n".join(lines) + "\n" + note)
print("\n".join(lines))

plt.rcParams.update({"legend.fontsize": 8, "axes.labelsize": 10, "xtick.labelsize": 9, "ytick.labelsize": 9})

pb = loadmat(os.path.join(PY, PY_BASE), squeeze_me=True)
pc = loadmat(os.path.join(PY, PY_CHOSEN), squeeze_me=True)

K_F, C_F = 1.0 / (2 / (3 * np.sqrt(3)) * 20.0), 1.0 / (0.300 * 20.0)
FIELD = {"baseline": "arctan", "chosen": "arccos", "chosen_repeat": "arccos", "chosen_lead": "arccos", "chosen_course": "arccos"}


def phi(e, field):
    if field == "arccos":
        u = np.exp(-C_F * np.abs(e))
        return -np.sign(e) * np.arcsin(1 - u)
    return -np.arctan(K_F * e)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def leader_errors(d, field, A=30.0, L=300.0):
    k = 2 * np.pi / L
    x, y = d.leader_x.values, d.leader_y.values
    u = x.copy()
    for _ in range(20):
        f, fp, fpp = A * np.sin(k * u), A * k * np.cos(k * u), -A * k * k * np.sin(k * u)
        u -= ((u - x) + (f - y) * fp) / (1 + fp ** 2 + (f - y) * fpp)
    cp = np.arctan(A * k * np.cos(k * u))
    dd = -(x - u) * np.sin(cp) + (y - A * np.sin(k * u)) * np.cos(cp)
    return dd, wrap(d.leader_chi.values - (cp + phi(dd, field)))


def follower_errors(d, k):
    ey = np.array([d[f"f{i}_eyi"].values for i in range(1, 5)])
    rel = np.array([d[f"f{i}_chi_err"].values - (d[f"f{i}_delta"].values if k != "baseline" else 0.0) for i in range(1, 5)])
    ct = wrap(rel - phi(ey, FIELD[k]))
    return np.sqrt(np.mean(ey ** 2, axis=0)), np.sqrt(np.mean(ct ** 2, axis=0))


fig, axs = plt.subplots(2, 2, figsize=(7.16, 5.6))
fig.subplots_adjust(hspace=0.48, wspace=0.3, left=0.1, right=0.98, top=0.98, bottom=0.11)
ax = axs.flat
for k in ("baseline", "chosen", "chosen_lead"):
    d = runs[k]
    t = d.tt.values
    dl, cl = leader_errors(d, FIELD[k])
    ey, ct = follower_errors(d, k)
    if k != "chosen_lead":
        ax[0].plot(t, np.abs(dl), color=RUNS[k][1], lw=1.4, label=f"SITL, {FIELD[k]} leader")
        ax[1].plot(t, np.abs(cl), color=RUNS[k][1], lw=1.4, label=f"SITL, {FIELD[k]} leader")
    ax[2].plot(t, ey, color=RUNS[k][1], lw=1.3, label=f"SITL, {RUNS[k][0]}")
    ax[3].plot(t, ct, color=RUNS[k][1], lw=1.0, label=f"SITL, {RUNS[k][0]}")
for m, c, lab in ((pb, "#e01010", "baseline"), (pc, "#1f3fff", "chosen")):
    lf = "arctan" if lab == "baseline" else "arccos"
    ax[0].plot(m["t"], np.abs(m["d"]), color=c, lw=1.1, ls=":", label=f"Python, {lf} leader")
    ax[1].plot(m["t"], np.abs(m["chi_t_l"]), color=c, lw=1.1, ls=":", label=f"Python, {lf} leader")
    ax[2].plot(m["t"], np.abs(m["ey"]), color=c, lw=1.1, ls=":", label=f"Python, {lab}")
    ax[3].plot(m["t"], np.abs(m["chi_t_i"]), color=c, lw=1.1, ls=":", label=f"Python, {lab}")
lim = [(1e-5, 50), (1e-5, 3), (1e-4, 10), (1e-5, 3)]
lab = [r"leader $|d|$ (m)", r"leader $|\tilde\chi_l|$ (rad)", r"follower $e_y$, RMS of 4 (m)", r"follower $\tilde\chi_i$, RMS of 4 (rad)"]
for j, a in enumerate(ax):
    a.set_yscale("log")
    a.set_ylim(*lim[j])
    a.set_xlim(0, 88)
    a.set_xlabel("time since the leader starts (s)")
    a.set_ylabel(lab[j])
    a.axhline(0.2 if j in (0, 2) else np.nan, color="0.4", lw=0.6, ls="--")
    a.legend(loc="upper right" if j < 2 else "lower right", fontsize=6.2)
    a.text(0.5, -0.27, f"({'abcd'[j]})", transform=a.transAxes, ha="center", va="top", fontsize=9)
save(fig, f"{OUT}/fig_sitl_overlay")

fig, axs = plt.subplots(1, 2, figsize=(7.16, 3.2))
fig.subplots_adjust(wspace=0.32, left=0.09, right=0.98, top=0.97, bottom=0.25)
for k in ("baseline", "chosen", "chosen_lead"):
    d = runs[k]
    t = d.tt.values
    w = (t > W0) & (t < W1)
    car, lag = parts(d)
    om = d.leader_chidot.values[w]
    axs[0].plot(om, lag[w], ".", ms=1.0, color=RUNS[k][1], alpha=0.3)
    s = np.polyfit(om, lag[w], 1)
    oo = np.linspace(om.min(), om.max(), 2)
    axs[0].plot(oo, np.polyval(s, oo), color=RUNS[k][1], lw=2.0, label=f"{RUNS[k][0]}: {s[0]:+.1f} m per rad/s")
axs[0].set_xlabel(r"leader turn rate $\omega$ (rad/s)")
axs[0].set_ylabel(r"common lateral bias (m)")
axs[0].set_xticks([-0.06, -0.03, 0.0, 0.03, 0.06])
axs[0].legend(loc="lower left", fontsize=7)

d = runs["chosen_course"]
D = np.mean([d[f"f{i}_Di"].values for i in range(1, 5)], axis=0)
axs[1].plot(d.tt, D, color="#9e9e9e", lw=1.6, label="course feedback")
d = runs["chosen_lead"]
D = np.mean([d[f"f{i}_Di"].values for i in range(1, 5)], axis=0)
axs[1].plot(d.tt, D, color="#1a9850", lw=1.6, label="velocity lead")
axs[1].set_xlim(0, 28)
axs[1].set_xlabel("time (s)")
axs[1].set_ylabel(r"formation error $D$ (m)")
axs[1].legend(loc="upper right", fontsize=7)
for k, a in enumerate(axs):
    a.text(0.5, -0.27, f"({'ab'[k]})", transform=a.transAxes, ha="center", va="top", fontsize=9)
save(fig, f"{OUT}/fig_sitl_lag")
