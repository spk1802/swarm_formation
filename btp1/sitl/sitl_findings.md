# SITL findings (thesis 7.2–7.3)

PX4 v1.17 SITL, Gazebo Harmonic, four x500 quadrotors, ROS 2 Jazzy, package `swarm_formation`
(branch jazzy-harmonic). Presets `start_swarm.sh --btp baseline|chosen|chosen_lead` match the Python
study: sine path y = 30 sin(2πx/300), leader 5 m/s from (0, −30) (≈25.5 m off), slots (±3, ±3) m,
each drone 5 m laterally outward from its slot, r_min = 20 m gains. Logs copied to `logs/`, metrics
in `sitl_table.md` (t = 20–88 s after the leader starts moving).

| Log | Source run |
| --- | --- |
| logs/baseline.csv | ~/sim_logs/sitl/runs/20261108_042313 |
| logs/chosen.csv | ~/sim_logs/sitl/runs/20261108_043534 |
| logs/chosen_repeat.csv | ~/sim_logs/sitl/runs/20261108_044742 (launched as chosen_lead, but the lead code had not reached the package: identical to chosen) |
| logs/chosen_course.csv | ~/sim_logs/sitl/runs/20261108_044148 (heading_source = course; stopped at 28 s) |
| logs/chosen_lead.csv | ~/sim_logs/sitl/runs/20261108_150959 |

- **The carousel effect is reproduced.** Lateral error splits into a part that flips sign with g_x
  (front − rear)/2 and a part common to all drones. In the baseline the first is 0.185 m RMS and matches
  the Python baseline e_y against leader along-track position (gain 0.99, correlation 0.996).
- **PX4 adds a common bias.** The drone's course trails its yaw by ω·0.385 s (PX4 velocity loop). With yaw
  feedback the field settles at e_y ≈ ωτ/(field slope): −2.82 m per rad/s with arctan (predicted
  0.385/0.130 = 2.96), −1.75 with arccos (predicted 0.385/0.167 = 2.31). 0.117 m RMS in the baseline.
- **Chosen combination:** carousel part 0.185 → 0.022 m; RMS e_y front drones 0.30 → 0.07–0.09 m; all
  drones settle (13–16 s); mean D 0.247 → 0.087 m; speed std 0.58 → 0.44 m/s. Repeat run within 0.01 m.
- **Course feedback fails:** feeding back the measured course puts the PX4 lag inside the heading loop;
  the formation oscillated at 0.22 Hz with ±4 m e_y and the run was stopped at 28 s.
- **Velocity lead works:** sending the velocity setpoint along χ + τ·χ̇_cmd (τ = 0.385 s) cancels the
  lag open-loop. Lag part 0.074 → 0.031 m (slope −1.75 → +0.50, slight over-compensation), RMS e_y
  0.034–0.047 m on all drones, settling 11–13 s, mean D 0.048 m, speed std 0.11 m/s.

Caveats: logged speed is measured, not commanded, so the guard law's chattering appears filtered by PX4
(0.2 Hz, ±0.9 m/s) rather than as 0.5–8 m/s switching. Leader ramps to 5 m/s over 6 s (Python starts at
speed), which delays leader settling by ~3 s (14.1–14.9 s vs 11.1–12.0 s). Python remains far lower
(0.29 mm) — the SITL floor (~0.04 m) is PX4 tracking, not the guidance law.
