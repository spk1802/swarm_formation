# Virtual Leader-based Multi-UAV Formation Path Following

A ROS 2 / PX4 SITL / Gazebo implementation of the vector-field + Lyapunov hybrid guidance strategy from:

> **S. Basak and S. Ghosh**, *"Virtual Leader-based Multi-UAV Formation Path Following Guidance Using Vector Field,"* 2025 Eleventh Indian Control Conference (ICC), IISc Bengaluru, India. DOI: 10.1109/ICC69100.2025.11372370

The system drives a swarm of PX4 SITL quadrotors to converge to and hold a rigid geometric formation around a **virtual leader**, while the virtual leader follows a reference path using a vector-field guidance law. Followers regulate their heading and speed through the paper's guidance laws (Eqs. 14, 15, 21), giving finite-time heading convergence and asymptotic formation-error convergence. On top of the paper, the follower adds a **carousel correction Δ** (and its feedforward Δ̇) that removes the steady formation error the paper's laws leave on curved paths.

---

## Table of Contents

- [Overview](#overview)
- [What's Implemented](#whats-implemented)
- [System Architecture](#system-architecture)
- [Guidance Laws](#guidance-laws)
- [Prerequisites](#prerequisites)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [Configuration & Parameters](#configuration--parameters)
- [Diagnostics & Visualization](#diagnostics--visualization)
- [Offline Study Series](#offline-study-series)
- [Post-processing](#post-processing)
- [Results](#results)
- [Design Notes & Lessons Learned](#design-notes--lessons-learned)
- [Known Limitations](#known-limitations)
- [Progress Log](#progress-log)
- [Citation](#citation)

---

## Overview

The guidance strategy combines two ideas:

1. **Virtual structure / virtual leader.** A constant-speed virtual leader follows the reference path via a vector-field path-following law. Because the leader's forward speed is a known constant, followers never depend on noisy or delayed leader-velocity estimates.
2. **Hybrid follower guidance.** Each follower uses a vector field to generate its *desired heading* and a Lyapunov-based controller to generate its *commanded ground speed*, so that it converges to a fixed offset `G_i = (g_xi, g_yi)` in the leader's body frame.

The result is precise formation path-following with quadrant-separated initial partitions that provide inter-agent collision avoidance.

---

## What's Implemented

- **One leader node for every path** (`path_leader`): sinusoid, figure-8, circle, straight line, with closed-form curvature, χ̇_l and χ̈_l (nothing finite-differenced).
  - *Exact mode* (`path_offset = 0`): the leader is generated exactly on the path.
  - *Vector-field mode* (`path_offset ≠ 0`): the path is shifted and the leader flies onto it with the paper's vector field (Eq. 5) and Lemma-1 saturation law (Eq. 6), moved along the exact unicycle arc.
- Follower desired-heading vector field (Eq. 14), finite-time heading controller (Eq. 15) and ε-regularised Lyapunov ground-speed law (Eq. 21).
- **Carousel correction Δ** and its **Δ̇ feedforward** in the follower (switchable for A/B tests).
- **Start gate:** the follower publishes a latched `/formation_engaged` once all drones are in OFFBOARD; the leader holds at the origin until then.
- **50 Hz yaw-rate** commanding (`TrajectorySetpoint.yawspeed`) instead of a 10 Hz yaw-angle setpoint.
- Multi-vehicle PX4 SITL orchestration for ROS 2 Jazzy / Gazebo Harmonic / PX4 v1.17 (`scripts/start_swarm.sh`).
- Single-CSV formation logger (6-decimal positions) + per-run `run_config.txt`.
- RViz swarm visualization.
- Offline numpy study series (steps 1–7), a kinematic ROS harness and a Gazebo evaluation pipeline (step 8).
- MATLAB convergence dashboards (`study/analyze_formation_csv.m`).

---

## System Architecture

The stack is a set of ROS 2 nodes started by `launch/swarm.launch.py`, which `scripts/start_swarm.sh` calls after bringing up Gazebo, PX4 and the DDS agent:

| Node | Responsibility |
|------|----------------|
| **Virtual leader** (`path_leader`) | One node for every path: `path:=sinusoid` (default) / `figure8` / `circle` / `straight_line`. Publishes leader pose, velocity (`angular.z = χ̇_l`) and acceleration (`angular.z = χ̈_l`). Two modes on any path: `path_offset = 0` generates the leader exactly on the path; `path_offset ≠ 0` shifts the path and the leader flies onto it with the paper's vector field (Eq. 5/6). Holds at the origin until `/formation_engaged`. |
| **Follower controller** | One node for all 4 drones. Holds the formation offsets `G_i`, computes formation errors `(e_xi, e_yi)`, the desired heading (Eq. 14, + carousel Δ), the commanded course rate (Eq. 15, + Δ̇) and ground speed (Eq. 21); sends 50 Hz setpoints to PX4. Publishes `/formation_engaged`. |
| **Formation logger** | One CSV per run: leader + follower states and formation errors. |
| **Swarm visualizer** | Publishes markers/paths for RViz. |

Data flow (simplified):

```
reference path ─▶ [virtual leader] ─▶ leader state (pose, χ_l, χ̇_l, χ̈_l)
                        ▲                 │
                        │                 ▼
   /formation_engaged ──┘   odometry ─▶ [follower controller] ─▶ PX4 setpoints ×4
                                          │
                     ┌────────────────────┴────────────────────┐
                     ▼                                         ▼
             [formation logger] ─▶ CSV                [swarm visualizer] ─▶ RViz
```

Topics (NED): `/virtual_leader_pose`, `/virtual_leader_velocity`, `/virtual_leader_accel`, `/reference_path_marker`, `/formation_engaged`.

---

## Guidance Laws

The core equations reproduced from the paper (see Section III):

**Leader vector field (Eq. 5):**
```
χ_l^d = χ^p − χ^∞ · (2/π) · atan(k·d)
```

**Follower desired heading (Eq. 14):**
```
χ_i^d = χ_l − χ^∞ · (2/π) · atan(k_y · e_yi)
```

**Follower commanded course, finite-time convergence (Eq. 15):**
```
χ_i^c = χ_i + (χ̇_l / α_i)
        − χ^∞ · (2 / (α_i·π)) · (k_y / (1 + (k_y·e_yi)^2)) · ė_yi
        − (η / α_i) · (χ̃_i)^(n/m)
```

**Follower ground-speed law, asymptotic formation-error convergence (Eq. 21):**
```
v_gi = ( v_gl·e_xi − k_D·D_i^2 − χ̇_l·[e_xi·g_yi − e_yi·g_xi] )
       / ( e_xi·cos(χ_i − χ_l) + e_yi·sin(χ_i − χ_l) )
```

where `D_i = sqrt(e_xi² + e_yi²)` is the formation error magnitude. The denominator is regularised with `ε_den = 0.05 m`.

**Carousel correction (this implementation).** On a curved path the follower's slot does not move parallel to the leader: by rigid-body kinematics `v_slot = v_leader + ω × G_i`. Substituting `χ_i = χ_l` into the paper's Eq. 13 gives `ė_yi = −χ̇_l·g_xi ≠ 0`, so `(χ_i = χ_l, e_y = 0)` is not an equilibrium and the plain laws settle at `e_y_ss = −tan(Δ)/k_y`. The follower therefore steers to the slot's own heading:

```
Δ_i = atan2( χ̇_l·g_xi , v_gl − χ̇_l·g_yi )        χ_i^d = χ_l + Δ_i − χ^∞·(2/π)·atan(k_y·e_yi)
```

with `Δ̇_i` (from the leader's analytic χ̈_l) fed forward in Eq. 15. `enable_delta:=false` recovers the paper's laws; `enable_delta_dot:=false` gives the pointwise variant.

---

## Prerequisites

| Component | Version used |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy |
| Simulator | Gazebo Sim (Harmonic) 8.x |
| Autopilot | PX4-Autopilot v1.17.0 SITL, airframe 4001 (`gz_x500`) |
| DDS bridge | Micro-XRCE-DDS-Agent v2.4.3 (UDP 8888) |
| Messages | `px4_msgs` matching PX4 v1.17, in the same workspace |
| Python | Python 3, `numpy`, `matplotlib` (study + evaluation scripts) |
| Optional | MATLAB (Online works) for `analyze_formation_csv.m`; `rviz2`, `rqt_plot` |

The original Humble / Gazebo Classic version lives on the `main` branch; this branch (`jazzy-harmonic`) is the current one.

---

## Repository Structure

```
.
├── README.md
├── package.xml / setup.py / setup.cfg
├── scripts/
│   └── start_swarm.sh             # Gazebo + 4× PX4 SITL + DDS agent + ROS stack
├── launch/
│   └── swarm.launch.py            # leader, follower, logger, visualizer (+ rviz)
├── swarm_formation/
│   ├── path_leader.py             # The leader: any path, exact or vector-field mode
│   ├── follower_controller.py     # Eqs. 14, 15, 21 + carousel Δ / Δ̇, start gate
│   ├── formation_logger.py        # CSV per run
│   └── swarm_visualizer.py        # RViz markers/paths
├── study/
│   ├── step1_finite_time.py … step7_initial_conditions.py   # offline numpy series
│   ├── step8_ros_harness.py       # kinematic mirror of the ROS nodes (perfect plant)
│   ├── step8_evaluate.py          # metrics + figures from Gazebo CSVs
│   ├── analyze_formation_csv.m    # MATLAB convergence dashboards
│   ├── figures/                   # figures from steps 1–8
│   └── results_step8/             # Step 8 report, metrics and figures
│       ├── STEP8_RESULTS.md
│       ├── v2/                    # gated runs, 5 m offset
│       └── v2_n10e10/             # gated runs, (10 N, 10 E) offset
└── data/                          # CSV logs cited in reports
```

Runtime logs go to `~/swarm_logs/<timestamp>/` (CSV, `run_config.txt`, per-process logs) and are not tracked.

---

## Installation

1. **Clone into your ROS 2 workspace** (alongside `px4_msgs`):
   ```bash
   cd ~/ros2_ws/src
   git clone -b jazzy-harmonic https://github.com/SPK2005/swarm_formation.git
   ```

2. **Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **PX4 location.** `start_swarm.sh` uses `PX4_DIR` (default `~/tools/PX4-Autopilot`) and `ROS_WS` (default `~/ros2_ws`); override them via the environment if yours differ.

---

## Running the Simulation

```bash
./scripts/start_swarm.sh                                  # sinusoid, exact mode, rviz on
./scripts/start_swarm.sh --leader circle --no-rviz
./scripts/start_swarm.sh --leader figure8 --path-offset 5 # VF mode on any path
./scripts/start_swarm.sh --path-offset 10,10              # shift path North 10, East 10
./scripts/start_swarm.sh --n 1 --leader straight_line     # single drone
```

Follower A/B modes (Step 8):

```bash
./scripts/start_swarm.sh --path-offset 5 --delta off                --tag none
./scripts/start_swarm.sh --path-offset 5 --delta on --delta-dot off --tag pw
./scripts/start_swarm.sh --path-offset 5 --delta on                 --tag full
```

The script:

1. Starts the Micro-XRCE-DDS agent, Gazebo Harmonic and 4 PX4 SITL instances (spawned at their formation offsets, all facing North).
2. Waits for each vehicle's `vehicle_local_position(_v1)` topic.
3. Launches `swarm.launch.py`. The leader holds at the origin until every drone is in OFFBOARD, waits 3 s, then ramps to speed over 6 s.

`Ctrl-C` stops everything, including the Gazebo server and GUI. If gz-transport reports `Network is unreachable`, run with `GZ_IP=127.0.0.1`.

Options: `--n`, `--leader`, `--log-dir`, `--rviz/--no-rviz`, `--delta on|off`, `--delta-dot on|off`, `--path-offset E|N,E`, `--tag`, `--speed`, `--amplitude`, `--wavelength`, `-h`.

---

## Configuration & Parameters

**Leader (`path_leader` ROS parameters)**

| Parameter | Default | Notes |
|---|---|---|
| `path` | `sinusoid` | `sinusoid` / `figure8` / `circle` / `straight_line` |
| `speed` (`v_gl`) | 0.5 m/s | Scaled for the Gazebo world (paper uses 15 m/s). |
| `path_offset`, `path_offset_north` | 0, 0 | Non-zero → VF mode. |
| `amplitude`, `wavelength` | 10 m, 130 m | Sinusoid `y = A(cos(2πx/λ) − 1)`. |
| `radius`, `direction` | 20 m, +1 | Circle. |
| `f8_length`, `f8_width` | 60 m, 20 m | Figure-8. |
| `vf_K`, `vf_chi_inf`, `vf_beta`, `vf_eps`, `vf_omega_max` | 0.3, π/2, 0.08, 0.3, 0.08 rad/s | VF gains, sized for 0.5 m/s and \|G\| = 4.24 m. The paper's β = π/4 saturates the followers. |
| `wait_for_formation`, `engage_delay`, `engage_timeout` | true, 3 s, 60 s | Start gate. |
| `ramp_time`, `publish_rate` | 6 s, 100 Hz | |

**Follower (`follower_controller`)**

| Symbol | Value | Notes |
|---|---|---|
| `k_y` | 1.2 | Raised from the paper's 0.02 for ~1 m errors. |
| `k_D` | 0.4 | Raised from 0.02 (≈2.5 s time constant). |
| `χ^∞` | π/2 | |
| `α` | 1.65 | Heading time constant (paper Eq. 4). |
| `η`, `n/m` | π/4, 3/5 | Finite-time term `(χ̃_i)^(n/m)`. |
| `ε_den` | 0.05 m | Eq. 21 regularisation. |
| speed clamp, `a_max` | [−0.3, 1.2] m/s, 1.5 m/s² | |
| `max_yaw_rate` | 0.8 rad/s | Slew on the course command. |
| `fwd_min_frac` | 0.25 | Guard on `v_gl − ω·g_y` in Δ. |
| `control_rate`, `yaw_rate_cmd` | 50 Hz, true | |
| `enable_delta`, `enable_delta_dot` | true, true | |
| `G_i` | (±3, ±3) m | One drone per quadrant. |

> **Trajectory geometry constraint:** the path's minimum turn radius must comfortably exceed the formation offset, or Eq. 21's denominator goes singular and the outer followers would need negative speed. The default sinusoid has R_min ≈ 42.8 m against |G| = 4.24 m.

---

## Diagnostics & Visualization

- **RViz:** formation shape, leader/follower paths, the reference path and goal markers (`--rviz`, default on).
- **CSV log:** `formation_<timestamp>_<tag>.csv` in the run's log dir, with `run_config.txt` recording every flag.
- **Launch logs:** `swarm_launch.log` contains the leader's start-up line (`d0`, `chi_tilde0`, `R_min`) and the `FORMATION ENGAGED` stamp used by `step8_evaluate.py --t0 engaged`.

---

## Offline Study Series

`study/` builds the guidance up one piece at a time in plain numpy, each script producing a figure in `study/figures/`:

| Step | Script | Topic |
|---|---|---|
| 1 | `step1_finite_time.py` | Finite-time heading convergence (Eq. 15) |
| 2 | `step2_steering.py` | Turn-only (unicycle) vector-field steering onto a line |
| 3 | `step3_curve.py` | Following a circle |
| 4 | `step4_formation.py` | Leader + follower on a circle; the carousel gap and Δ |
| 5 | `step5_varying_curve.py` | Time-varying curvature; Δ̇ |
| 6 | `step6_leader_convergence.py` | VF leader converging from off the path |
| 7 | `step7_initial_conditions.py` | Sensitivity to initial conditions |
| 8 | `step8_ros_harness.py`, `step8_evaluate.py` | Kinematic mirror of the ROS nodes; Gazebo evaluation |

```bash
python3 study/step8_ros_harness.py --plot --offset 5
python3 study/step8_evaluate.py none=<csv> pointwise=<csv> full=<csv> \
    --path sinusoid --offset 5 --t0 engaged --out-dir study/results_step8/v2
```

---

## Post-processing

**MATLAB** (`study/analyze_formation_csv.m`, tested in MATLAB Online): upload the CSV and its `run_config.txt` to the same folder, then

```matlab
analyze_formation_csv('formation_<...>.csv')
out = analyze_formation_csv(file, 'Path', 'sinusoid', 'Offset', [10 10])
```

It rebuilds the leader's cross-track error from the path in `run_config.txt` and plots leader convergence, follower formation convergence, follower motion, an overview and the convergence type. `'Export', true` saves `<csv>_analysis.png/.pdf`.

**Python:** `study/step8_evaluate.py` writes `metrics.md` and `step8_evaluation.png` (accuracy, effort, oscillation, Eq. 21 health, safety).

---

## Results

Full report: [`study/results_step8/STEP8_RESULTS.md`](study/results_step8/STEP8_RESULTS.md). Sinusoid path, leader starting 5 m off it (VF mode), 4 drones, start gate on, 50 Hz yaw-rate commands, ~380 s of flight per run:

| | none (paper laws) | pointwise Δ | full Δ + Δ̇ | harness (full) |
|---|---|---|---|---|
| Leader \|d\| < 0.1 m after | 39.1 s | 39.1 s | 39.2 s | 39.4 s |
| Steady RMS formation error D | 0.061 m | 0.032 m | 0.033 m | 0.003 m |
| Start-up peak D | 0.545 m | 0.201 m | 0.226 m | 0.031 m |
| RMS acceleration | 0.073 m/s² | 0.020 m/s² | 0.020 m/s² | |
| Min follower separation (6 m nominal) | 5.46 m | 5.89 m | 5.87 m | |
| Time in own quadrant | 100 % | 100 % | 100 % | |

- The VF leader converges onto the path in **39.1 s** (harness 39.4 s) and settles to 1.3e-5 m.
- **Δ halves the steady formation error** (0.061 → 0.032 m) and **cuts the start-up peak 2.4×**, with 3.7× less acceleration and no extra heading effort.
- Steady state is **PX4-limited at ~3 cm** (the kinematic harness reaches 3 mm). The remaining start-up peak is PX4's velocity lag during the speed ramp.
- The Δ̇ feedforward has **no effect that can be resolved** in Gazebo.
- Switching from a 10 Hz yaw-angle to a 50 Hz yaw-rate command cut follower yaw-rate RMS by **55 %** and the 0.78 Hz oscillation share from ~80 % to ~30 %, at unchanged accuracy.
- With a (10 N, 10 E) offset (`v2_n10e10/`), the leader converges in 51.1 s and steady RMS D is 0.060 / 0.033 / 0.031 m — the same picture.

One run per mode, so differences smaller than run-to-run noise are not resolved.

---

## Design Notes & Lessons Learned

- **PX4 multi-vehicle SITL odometry is spawn-relative, not world-relative.** Fix: add `spawn_ned` offsets in every consumer node.
- **ENU ↔ NED conversion:** `NED = (ENU_y, ENU_x, −ENU_z)`; display in RViz/Gazebo needs the reverse swap.
- **Analytic derivatives beat finite differences.** χ̇_l, χ̈_l and ė_y are all exact. A backward difference of χ̇_l injects a first-sample heading kick.
- **Velocity direction must use the current heading `χ_i`,** not the commanded heading — the latter flung drones outward.
- **No `max(0.0, …)` speed clamp.** It blocked reverse motion and froze drones away from their targets.
- **The paper's laws leave a steady error on curved paths** (`e_y_ss = −tan(Δ)/k_y`); the carousel Δ removes it.
- **Start-up races distort transient metrics.** Before the start gate, drones joining late caused peaks up to 5.7 m and one 0.42 m near-miss. The leader now waits for every drone.
- **Command yaw rate, not yaw angle.** A 10 Hz held yaw-angle command through PX4's ~0.32 s delay produced a 0.78 Hz limit cycle.
- **PX4 v1.17 renames topics** (`vehicle_local_position_v1`); readiness probes must accept the `_vN` suffix.
- **Launch-script pitfalls:** start PX4 with `-d` (the pxh shell otherwise writes hundreds of MB of logs), `set +u` around `source install/setup.bash`, and kill `gz sim` on cleanup.
- **Gain tuning must respect time constants** and leader VF gains must respect the formation: leader turn rate ω becomes follower sideways speed ω·g_x.

---

## Known Limitations

- Collision avoidance relies on quadrant separation, which assumes all followers engage together (enforced by the start gate). Densely populated formations impose restrictive leader turn-rate conditions (paper Sec. III-C).
- Simulation speeds and gains are scaled for the Gazebo world and differ from the paper's numerical study.
- Steady accuracy is bounded by PX4 velocity tracking (~3 cm), not the guidance law.
- One run per mode in Step 8; no repeat statistics yet.
- Validated for 4 drones; scaling to `N` followers requires partitioning the plane into `N` disjoint sectors.

---

## Progress Log

| Date | Milestone |
|---|---|
| 2026-07 | Velocity P-controller pipeline, then virtual-leader vector-field formation guidance (ROS 2 Humble). |
| 2026-08 | Migrated to ROS 2 Jazzy / Gazebo Harmonic / PX4 v1.17; rewrote `start_swarm.sh`; launch arguments. |
| 2026-10-02 | Sinusoid leader, carousel Δ / Δ̇, formation logger, study steps 1–7; first Step 8 Gazebo evaluation. |
| 2026-10-05 | Unified `path_leader` (all paths, exact + VF mode, N/E offsets); start gate; 50 Hz yaw-rate commands; Step 8 v2 re-evaluation and (10, 10) offset runs. Removed per-path leaders, `formation_manager` and `plotter`. |
| 2026-10 | MATLAB convergence dashboards (`analyze_formation_csv.m`). |

**Next:** repeat runs per mode for variance; figure-8 and circle evaluation with the start gate; scaling beyond 4 drones.

---

## Citation

If you use this implementation, please cite the original paper:

```bibtex
@inproceedings{basak2025virtual,
  title     = {Virtual Leader-based Multi-UAV Formation Path Following Guidance Using Vector Field},
  author    = {Basak, Subham and Ghosh, Satadal},
  booktitle = {2025 Eleventh Indian Control Conference (ICC)},
  year      = {2025},
  address   = {Bengaluru, India},
  doi       = {10.1109/ICC69100.2025.11372370}
}
```
