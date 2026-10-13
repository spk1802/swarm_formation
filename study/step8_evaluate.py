#!/usr/bin/env python3
"""
step8_evaluate.py -- controller performance + effort from Gazebo logger CSVs.

    python3 study/step8_evaluate.py none=<csv> full=<csv> [pointwise=<csv>] \
        [--path sinusoid|figure8|circle|straight_line] [--offset 5 | --offset N,E]
        [--t0 onset|engaged] [--out-dir study/results_step8]

Time zero (--t0):
  onset    leader motion onset (first leader_speed > 1e-3). Default; the only
           option for runs before the /formation_engaged start gate.
  engaged  the moment the follower published /formation_engaged (all drones
           in OFFBOARD). Read from swarm_launch.log next to the CSV:
             CSV t_engaged = onset - [(leader rx stamp - FORMATION ENGAGED
                                       stamp) + engage_delay]
           (same ROS clock; the leader counts engage_delay in timer ticks, so
           the mapping is good to a few ms). Transient windows then start at
           engagement; leader convergence is still timed from leader onset.

Writes <out-dir>/metrics.md (tables)
and <out-dir>/step8_evaluation.png. Harness predictions for the same mode are
printed alongside the accuracy metrics.

Metric groups
  leader      t(|d|<0.1), |d| at end
  accuracy    transient max D (t<60s), steady RMS D / e_x / e_y (t>settle),
              time until all D_i stay < 0.1 m
  effort      measured follower yaw rate: RMS, max, integral |chi_dot| dt;
              speed: RMS deviation from leader speed, RMS accel, % time >= 1.15 m/s
  oscillation fraction of yaw-rate power in 0.6-1.0 Hz (the PX4 0.79 Hz cycle)
  eq21        % time |den/D| < 0.2 (near the Eq. 21 singularity)
  safety      min pairwise follower distance; % time each follower keeps its
              initial quadrant in the leader frame (paper Sec. III-C)
"""
import argparse
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "swarm_formation"))
sys.path.insert(0, HERE)
from path_leader import PATHS, make_path, ExactPathLeader, VFLeader  # noqa: E402
from step8_ros_harness import run as harness_run, formation_D  # noqa: E402

TRAPZ = getattr(np, "trapezoid", None) or np.trapz  # numpy 1.26 (Ubuntu 24.04) has only trapz

G = {1: (3.0, 3.0), 2: (-3.0, 3.0), 3: (3.0, -3.0), 4: (-3.0, -3.0)}
COL = {"none": "r", "pointwise": "orange", "full": "g"}


def engagement(csv_path):
    """Start-gate events from swarm_launch.log beside the CSV, or None.

    Returns dict(shift=engagement->leader-onset [s], fallback=bool,
    offboard=[drone engage times rel. to engagement, s]).
    """
    log = os.path.join(os.path.dirname(os.path.expanduser(csv_path)), "swarm_launch.log")
    if not os.path.isfile(log):
        return None
    stamp = lambda line: float(re.search(r"\[(\d+\.\d+)\]", line).group(1))  # noqa: E731
    eng = rx = delay = None
    fallback, offb = False, []
    for line in open(log, errors="replace"):
        if "FORMATION ENGAGED" in line:
            eng = stamp(line)
        elif "[start] /formation_engaged received" in line:
            rx = stamp(line)
            delay = float(re.search(r"moving in ([0-9.]+) s", line).group(1))
        elif "[start] FALLBACK" in line:
            fallback = True
        elif "OFFBOARD ENABLED" in line:
            offb.append(stamp(line))
    if eng is None or rx is None:
        return None
    return dict(shift=(rx - eng) + delay, fallback=fallback,
                offboard=[o - eng for o in offb])


def load(path, sp, t0="onset"):
    a = np.genfromtxt(os.path.expanduser(path), delimiter=",", names=True)
    i_on = np.argmax(a["leader_speed"] > 1e-3)
    t_onset = 0.0
    if t0 == "engaged":
        ev = engagement(path)
        if ev is None:
            raise SystemExit(f"--t0 engaged: no FORMATION ENGAGED / [start] lines "
                             f"in the swarm_launch.log beside {path}")
        if ev["fallback"]:
            print(f"WARNING {path}: leader was released by the 60 s FALLBACK, "
                  "not by /formation_engaged")
        t_onset = ev["shift"]
        t_eng = a["t"][i_on] - t_onset
        a = a[np.argmax(a["t"] >= t_eng):]
        t = a["t"] - t_eng
    else:
        a = a[i_on:]
        t = a["t"] - a["t"][0]
    u, d = 0.0, []
    for x, y in zip(a["leader_x"], a["leader_y"]):
        u, di, _ = sp.project(x, y, u)
        d.append(di)
    f = lambda q: np.vstack([a[f"f{i}_{q}"] for i in G]).T  # noqa: E731
    return dict(t=t, t_onset=t_onset, d=np.array(d), a=a, D=f("Di"), ex=f("exi"), ey=f("eyi"),
                spd=f("speed"), chidot=f("chidot"), x=f("x"), y=f("y"),
                xL=f("xL"), yL=f("yL"), den=f("den_norm") if "f1_den_norm" in a.dtype.names else None)


def rms(x):
    return float(np.sqrt(np.nanmean(np.asarray(x) ** 2)))


def band_fraction(t, sig, lo=0.6, hi=1.0):
    dt = np.median(np.diff(t))
    s = sig - sig.mean()
    P = np.abs(np.fft.rfft(s)) ** 2
    fr = np.fft.rfftfreq(len(s), dt)
    tot = P[fr > 0.05].sum()
    return float(P[(fr >= lo) & (fr <= hi)].sum() / tot) if tot > 0 else np.nan


def metrics(g, settle):
    t, D = g["t"], g["D"]
    late, early = t > settle, t < 60
    hit = (np.abs(g["d"]) < 0.1) & (t >= g["t_onset"])
    above = np.where((D > 0.1).any(1))[0]
    m = {
        "t0 -> leader start [s]": g["t_onset"],
        "leader t(|d|<0.1) from leader start [s]": t[np.argmax(hit)] - g["t_onset"] if hit.any() else np.nan,
        "leader |d| end [m]": abs(g["d"][-1]),
        "max D transient [m]": D[early].max(),
        "RMS D steady [m]": rms(D[late]),
        "RMS e_x steady [m]": rms(g["ex"][late]),
        "RMS e_y steady [m]": rms(g["ey"][late]),
        "all D<0.1 after [s]": t[above[-1]] if len(above) else 0.0,
        "RMS yaw rate [rad/s]": rms(g["chidot"]),
        "max |yaw rate| [rad/s]": np.nanmax(np.abs(g["chidot"])),
        "heading effort int|chidot| [rad]": float(TRAPZ(np.abs(g["chidot"]), t, axis=0).mean()),
        "RMS speed - v_gl [m/s]": rms(g["spd"] - g["a"]["leader_speed"][:, None]),
        "RMS accel [m/s^2]": rms(np.gradient(g["spd"], t, axis=0)),
        "% time speed>=1.15": 100 * float((g["spd"] >= 1.15).mean()),
        "0.6-1.0 Hz yaw power [%]": 100 * np.nanmean([band_fraction(t, g["chidot"][:, j]) for j in range(4)]),
        "% time |den/D|<0.2": 100 * float((np.abs(g["den"]) < 0.2).mean()) if g["den"] is not None else np.nan,
    }
    P = np.stack([g["x"], g["y"]], -1)
    dmin = min(np.linalg.norm(P[:, i] - P[:, j], axis=1).min()
               for i in range(4) for j in range(i + 1, 4))
    quad = [100 * float(((np.sign(g["xL"][:, j]) == np.sign(gx)) &
                         (np.sign(g["yL"][:, j]) == np.sign(gy))).mean())
            for j, (gx, gy) in enumerate(G.values())]
    m["min follower separation [m]"] = dmin
    m["min % time in own quadrant"] = min(quad)
    return m


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--path", choices=PATHS, default="sinusoid")
    ap.add_argument("--offset", default="5.0",
                    help="path shift: E (East) or N,E (North,East) [m]")
    ap.add_argument("--settle", type=float, default=120.0)
    ap.add_argument("--t0", choices=("onset", "engaged"), default="onset")
    ap.add_argument("--out-dir", default=os.path.join(HERE, "results_step8"))
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    off_n, off_e = ((float(v) for v in args.offset.split(",")) if "," in args.offset
                    else (0.0, float(args.offset)))
    sp = make_path(args.path, off_e, x_off=off_n)

    def harness_leader():
        """The leader the node runs for this --path/--offset (fresh state)."""
        p = make_path(args.path, off_e, x_off=off_n)
        return VFLeader(p) if (off_e != 0.0 or off_n != 0.0) else ExactPathLeader(p)

    runs, M, H = {}, {}, {}
    for spec in args.runs:
        mode, path = spec.split("=", 1)
        runs[mode] = load(path, sp, args.t0)
        M[mode] = metrics(runs[mode], args.settle)
        h = harness_run(mode, T=float(runs[mode]["t"][-1]), leader=harness_leader())
        hD = formation_D(h)
        H[mode] = {"max D transient [m]": hD[h[:, 0] < 60].max(),
                   "RMS D steady [m]": rms(hD[h[:, 0] > args.settle]),
                   "leader t(|d|<0.1) from leader start [s]": h[np.argmax(np.abs(h[:, 3]) < 0.1), 0]}

    modes = list(runs)
    lines = ["| metric | " + " | ".join(f"{m} (gazebo)" for m in modes)
             + " | " + " | ".join(f"{m} (harness)" for m in modes) + " |",
             "|---|" + "---|" * (2 * len(modes))]
    for k in M[modes[0]]:
        g = " | ".join(f"{M[m][k]:.4g}" for m in modes)
        h = " | ".join(f"{H[m][k]:.4g}" if k in H[m] else "" for m in modes)
        lines.append(f"| {k} | {g} | {h} |")
    if "none" in M and "full" in M:
        r = M["none"]["max D transient [m]"] / max(M["full"]["max D transient [m]"], 1e-9)
        rs = M["none"]["RMS D steady [m]"] / max(M["full"]["RMS D steady [m]"], 1e-9)
        hr = H["none"]["max D transient [m]"] / H["full"]["max D transient [m]"]
        lines += ["", f"**none/full transient max-D ratio: gazebo {r:.1f}x vs harness {hr:.1f}x; "
                  f"steady RMS-D ratio: gazebo {rs:.1f}x**"]
    md = "\n".join(lines)
    open(os.path.join(args.out_dir, "metrics.md"), "w").write(md + "\n")
    print(md)

    fig, ax = plt.subplots(2, 3, figsize=(17, 9))
    for mode, g in runs.items():
        c = COL.get(mode, "b")
        ax[0, 0].semilogy(g["t"], np.abs(g["d"]) + 1e-5, color=c, label=mode)
        ax[0, 1].semilogy(g["t"], g["D"].max(1) + 1e-5, color=c, label=f"{mode} gazebo")
        h = harness_run(mode, T=float(g["t"][-1]), leader=harness_leader())
        ax[0, 1].semilogy(h[:, 0] + g["t_onset"], formation_D(h).max(1) + 1e-5, color=c, ls="--", lw=1, label=f"{mode} harness")
        ax[0, 2].plot(g["t"], g["ey"][:, 0], color=c, label=mode)
        ax[1, 0].plot(g["t"], g["chidot"][:, 0], color=c, lw=.7, label=mode)
        ax[1, 1].plot(g["t"], g["spd"][:, 0], color=c, lw=.8, label=mode)
        ax[1, 2].plot(g["a"]["leader_y"], g["a"]["leader_x"], "k", lw=1.5)
        for j in range(4):
            ax[1, 2].plot(g["y"][:, j], g["x"][:, j], color=c, lw=.7)
    u0, u1 = sp.u_marker
    ref = np.array([sp.point(u0 + (u1 - u0) * i / 600) for i in range(601)])
    ax[1, 2].plot(ref[:, 1], ref[:, 0], "k:", lw=1)
    titles = [("leader |d| [m]", "leader cross-track"), ("max_i D_i [m]", "worst formation error"),
              ("e_y F1 [m]", "F1 lateral error"), ("yaw rate F1 [rad/s]", "F1 measured yaw rate (effort)"),
              ("speed F1 [m/s]", "F1 speed (effort)"), ("North [m]", "XY (East on x-axis)")]
    for q, (yl, tt) in zip(ax.flat, titles):
        q.set(ylabel=yl, title=tt, xlabel="t [s]"); q.grid(alpha=.3); q.legend(fontsize=7)
    ax[1, 2].set_xlabel("East [m]")
    fig.tight_layout()
    fp = os.path.join(args.out_dir, "step8_evaluation.png")
    fig.savefig(fp, dpi=130)
    print("saved", fp, "and", os.path.join(args.out_dir, "metrics.md"))
