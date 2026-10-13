#!/usr/bin/env python3
"""
step8_ros_harness.py -- kinematic stand-in for the Gazebo run of step 6.

Leader: path_leader.VFLeader (the code the node runs when path_offset != 0), 100 Hz.
Followers: a line-for-line mirror of follower_controller.py at 10 Hz --
  Eq. 14 + Delta (with the fwd_min_frac guard), Eq. 15 + dDelta/dt,
  chi_c slew (max_yaw_rate), regularized Eq. 21 blending to v_req
  (leader_speed when Delta is off),
  alignment gate, [-0.3, 1.2] clamp, 1.5 m/s^2 speed slew.
Plant: first-order yaw (alpha, paper Eq. 4) + perfect velocity tracking, so
any Gazebo excess over these numbers is PX4/physics, not guidance.

Modes  none      = enable_delta:=false
       pointwise = enable_delta:=true  enable_delta_dot:=false
       full      = enable_delta:=true  enable_delta_dot:=true

    python3 study/step8_ros_harness.py [--plot] [--offset 5]
"""
import argparse
import os
import sys
from math import atan, atan2, cos, sin, pi, hypot

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "swarm_formation"))
from path_leader import SinusoidPath, VFLeader, wrap  # noqa: E402

G = {1: (3.0, 3.0), 2: (-3.0, 3.0), 3: (3.0, -3.0), 4: (-3.0, -3.0)}
# follower_controller.py constants
CHI_INF, K_Y, K_D = pi / 2, 1.2, 0.4
ALPHA, ETA, N, M = 1.65, pi / 4, 3.0, 5.0
EPS_DEN, V_MIN, V_MAX, A_MAX = 0.05, -0.3, 1.2, 1.5
MAX_YAW_RATE, FWD_MIN_FRAC, DT_CTRL = 0.8, 0.25, 0.1


def spow(x, p):
    return abs(x) ** p * (1 if x >= 0 else -1)


def carousel(v, w, gx, gy):
    """follower_controller.compute_carousel (incl. guard)."""
    if v < 1e-3:
        return 0.0, 0.0
    v_fwd, v_side = v - w * gy, w * gx
    floor = FWD_MIN_FRAC * v
    if v_fwd <= floor:
        v_fwd = floor if gy >= 0 else -floor
    return atan2(v_side, v_fwd), hypot(v_fwd, v_side)


def run(mode, T=200.0, path_offset=5.0, leader_kw=None, ramp=6.0, v=0.5,
        vdot_term=True,    # True = patched follower; False = old dDelta/dt
        leader=None):      # any object with VFLeader's interface (e.g.
                           # path_leader.ExactPathLeader); default VF leader
    if leader is None:
        path = SinusoidPath(10.0, 130.0, y_off=path_offset)
        L = VFLeader(path, v=v, **(leader_kw or {}))
    else:
        L = leader
    dt_l, every = 0.01, 10
    F = {i: dict(x=g[0], y=g[1], chi=0.0, spd=0.0, ey_prev=None, chi_c=0.0,
                 chi_c_prev=0.0) for i, g in G.items()}
    use_d, use_dd = mode != "none", mode == "full"
    log = []
    for k in range(int(T / dt_l)):
        t = k * dt_l
        s = min(t / ramp, 1.0)
        L.step(dt_l, v * (3 * s * s - 2 * s ** 3))
        if k % every == 0:
            cl, sl = cos(L.chi), sin(L.chi)
            for i, f in F.items():
                gx, gy = G[i]
                dx, dy = f["x"] - L.x, f["y"] - L.y
                ex = dx * cl + dy * sl - gx
                ey = -dx * sl + dy * cl - gy
                ey_dot = 0.0 if f["ey_prev"] is None else (ey - f["ey_prev"]) / DT_CTRL
                f["ey_prev"] = ey
                dl, req = carousel(L.v, L.chi_dot, gx, gy)
                if not use_d:
                    dl = 0.0
                dd = 0.0
                if use_dd and req > 1e-3:
                    dd = L.chi_ddot * L.v * gx / req ** 2
                    if vdot_term:
                        dd -= L.chi_dot * L.v_dot * gx / req ** 2
                # patched follower: blend to v_req when Delta is on
                bl = req if (use_d and req > 1e-3) else L.v
                chi_d = L.chi + dl - CHI_INF * (2 / pi) * atan(K_Y * ey)
                chi_t = wrap(f["chi"] - chi_d)
                chi_c = wrap(f["chi"] + L.chi_dot / ALPHA + dd / ALPHA
                             - (2 * CHI_INF / (ALPHA * pi)) * K_Y / (1 + (K_Y * ey) ** 2) * ey_dot
                             - (ETA / ALPHA) * spow(chi_t, N / M))
                step = MAX_YAW_RATE * DT_CTRL
                dcc = max(-step, min(step, wrap(chi_c - f["chi_c_prev"])))
                chi_c = wrap(f["chi_c_prev"] + dcc)
                f["chi_c_prev"] = f["chi_c"] = chi_c
                num = L.v * ex - K_D * (ex * ex + ey * ey) - L.chi_dot * (ex * gy - ey * gx)
                den = ex * cos(f["chi"] - L.chi) + ey * sin(f["chi"] - L.chi)
                spd = (num * den + bl * EPS_DEN ** 2) / (den * den + EPS_DEN ** 2)
                align = max(0.0, cos(wrap(f["chi"] - chi_d)))
                spd = bl + (spd - bl) * align
                spd = max(V_MIN, min(spd, V_MAX))
                spd = max(f["spd"] - A_MAX * DT_CTRL, min(spd, f["spd"] + A_MAX * DT_CTRL))
                f["spd"] = spd
                f.update(ex=ex, ey=ey)
            log.append([t, L.x, L.y, L.d, L.chi_t, L.chi_dot, float(L.clamped)]
                       + [F[i][q] for i in G for q in ("x", "y", "ex", "ey", "spd")])
        for f in F.values():
            f["chi"] = wrap(f["chi"] + ALPHA * wrap(f["chi_c"] - f["chi"]) * dt_l)
            f["x"] += f["spd"] * cos(f["chi"]) * dt_l
            f["y"] += f["spd"] * sin(f["chi"]) * dt_l
    return np.array(log)


def formation_D(a):
    return np.sqrt(a[:, 9::5][:, :4] ** 2 + a[:, 10::5][:, :4] ** 2)


def summarize(a, settle=120.0):
    t, D = a[:, 0], formation_D(a)
    ey = a[:, 10::5][:, :4]
    spd = a[:, 11::5][:, :4]
    late = t > settle
    hit = np.abs(a[:, 3]) < 0.1
    return dict(t_d=t[np.argmax(hit)] if hit.any() else np.nan,
                clamp=a[:, 6].mean(), max_D=D.max(),
                rms_D=np.sqrt((D[late] ** 2).mean()),
                rms_ey=np.sqrt((ey[late] ** 2).mean()),
                spd=(spd.min(), spd.max()))


def fmt(mode, s):
    return (f"{mode:9s} t(|d|<0.1)={s['t_d']:5.1f}s clamp={100*s['clamp']:4.1f}% "
            f"maxD={s['max_D']:.3f} rmsD(t>120)={s['rms_D']:.4f} "
            f"rms_ey={s['rms_ey']:.4f} spd=[{s['spd'][0]:.2f},{s['spd'][1]:.2f}]")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--offset", type=float, default=5.0)
    args = ap.parse_args()
    res = {}
    for mode in ("none", "pointwise", "full"):
        res[mode] = run(mode, path_offset=args.offset)
        print(fmt(mode, summarize(res[mode])))
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 2, figsize=(12, 8))
        a = res["full"]
        p = SinusoidPath(10.0, 130.0, y_off=args.offset)
        xs = np.linspace(-5, a[:, 1].max() + 5, 600)
        ax[0, 0].plot([p.point(x)[1] for x in xs], xs, "k--", lw=1, label="reference path")
        ax[0, 0].plot(a[:, 2], a[:, 1], "k", lw=2, label="virtual leader")
        for j in range(4):
            ax[0, 0].plot(a[:, 8 + 5 * j], a[:, 7 + 5 * j], lw=1, label=f"F{j+1}")
        ax[0, 0].set(xlabel="East [m]", ylabel="North [m]", title="XY, full correction")
        ax[0, 0].legend(fontsize=7)
        ax[0, 1].semilogy(a[:, 0], np.abs(a[:, 3]) + 1e-6, "k")
        ax[0, 1].set(xlabel="t [s]", ylabel="|d| [m]", title="leader cross-track (asymptotic)")
        for mode, c in zip(("none", "pointwise", "full"), ("r", "orange", "g")):
            b = res[mode]
            ax[1, 0].plot(b[:, 0], b[:, 10], c, label=mode)
            ax[1, 1].semilogy(b[:, 0], formation_D(b).max(1) + 1e-6, c, label=mode)
        ax[1, 0].set(xlabel="t [s]", ylabel="e_y F1 [m]", title="F1 lateral error")
        ax[1, 1].set(xlabel="t [s]", ylabel="max_i D_i [m]", title="worst formation error")
        for q in ax.flat:
            q.grid(alpha=.3)
        ax[1, 0].legend(); ax[1, 1].legend()
        fig.tight_layout()
        out = os.path.join(os.path.dirname(__file__), "figures", "step8_harness.png")
        fig.savefig(out, dpi=130)
        print("saved", out)
