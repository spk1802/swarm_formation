# swarm_formation

Four PX4 quadrotors flying in formation behind a virtual leader, in Gazebo, with ROS 2.

This is my implementation of the guidance law from Basak and Ghosh, *Virtual Leader-based Multi-UAV Formation Path Following Guidance Using Vector Field* (ICC 2025, [doi:10.1109/ICC69100.2025.11372370](https://doi.org/10.1109/ICC69100.2025.11372370)). A virtual leader flies along a path at constant speed, and each drone holds a fixed slot around it, one per quadrant.

I also added one thing the paper doesn't have, which I call the carousel correction (Δ). It's explained below, and it halves the formation error on curved paths.

![Step 8 evaluation](study/results_step8/v2/step8_evaluation.png)

## Quick start

Tested on Ubuntu 24.04 with ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.17.0 (`gz_x500`) and Micro-XRCE-DDS-Agent v2.4.3. You also need `px4_msgs` in the same workspace.

```bash
cd ~/ros2_ws/src
git clone -b jazzy-harmonic https://github.com/spk1802/swarm_formation.git
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
./src/swarm_formation/scripts/start_swarm.sh
```

The script starts the DDS agent, Gazebo and 4 PX4 instances, then launches the ROS nodes. It expects PX4 in `~/tools/PX4-Autopilot`. If yours is somewhere else, set `PX4_DIR`. Ctrl-C shuts everything down.

Some other ways to run it:

```bash
start_swarm.sh --leader circle --no-rviz       # other paths: sinusoid, figure8, straight_line
start_swarm.sh --path-offset 5                 # leader starts 5 m off the path and flies onto it
start_swarm.sh --path-offset 10,10             # offset North 10, East 10
start_swarm.sh --delta off                     # paper's laws only, no carousel correction
start_swarm.sh --delta on --delta-dot off      # Δ without its feedforward
start_swarm.sh --n 1 --leader straight_line    # one drone
```

Logs go to `~/swarm_logs/<timestamp>/`: one CSV, a `run_config.txt` with the flags used, and the process logs. If Gazebo hangs with `Network is unreachable`, run with `GZ_IP=127.0.0.1`.

## How it works

There are four nodes:

- `path_leader` is the virtual leader. It works with any of the four paths. With no offset it sits exactly on the path. With an offset it starts away from the path and converges onto it using the paper's vector field (Eq. 5 and 6). It publishes pose, turn rate and turn acceleration, all computed analytically.
- `follower_controller` runs all four drones at 50 Hz. It computes heading with Eq. 14 and 15 and speed with Eq. 21, then sends yaw-rate and velocity setpoints to PX4.
- `formation_logger` writes the CSV.
- `swarm_visualizer` draws the formation in RViz.

The leader doesn't move until every drone is in OFFBOARD mode. The follower publishes `/formation_engaged` when that happens, and the leader then waits 3 s and ramps up to speed over 6 s.

### The carousel correction

When the leader turns, each slot around it moves a little sideways, like a seat on a carousel. The paper's follower always points along the leader's heading, so on a curve it settles with a small steady sideways error, `e_y = -tan(Δ)/k_y`. The fix is to steer to the slot's own heading instead:

```
Δ = atan2(ω·g_x, v_leader − ω·g_y)
```

Here ω is the leader's turn rate and `(g_x, g_y)` is the slot offset. Its derivative Δ̇ is fed forward in Eq. 15.

## Results

These runs use the sinusoid path, with the leader starting 5 m off it. Each run is about 380 s of flight, one run per mode. The full write-up is in [study/results_step8/STEP8_RESULTS.md](study/results_step8/STEP8_RESULTS.md).

| | paper laws | + Δ | + Δ and Δ̇ |
|---|---|---|---|
| steady formation error (RMS) | 6.1 cm | 3.2 cm | 3.3 cm |
| peak error at start-up | 55 cm | 20 cm | 23 cm |
| RMS acceleration | 0.073 m/s² | 0.020 m/s² | 0.020 m/s² |
| closest two drones got (6 m nominal) | 5.46 m | 5.89 m | 5.87 m |

What the numbers show:

- The leader reaches the path (within 10 cm) in 39 s, which matches the offline simulation.
- Δ halves the steady error, and the drones also accelerate less with it on.
- About 3 cm is as good as it gets in Gazebo, because PX4 can't track the velocity commands any better. In a perfect-plant simulation the same controller reaches 3 mm.
- Δ̇ makes no measurable difference in Gazebo.
- Every drone stayed in its own quadrant for the whole run.

## Parameters

Leader defaults:

| Parameter | Default |
|---|---|
| speed | 0.5 m/s |
| sinusoid amplitude / wavelength | 10 m / 130 m |
| circle radius | 20 m |
| figure-8 length × width | 60 × 20 m |
| VF gains (K, β, ε, ω_max) | 0.3, 0.08, 0.3, 0.08 rad/s |

Follower values:

| Parameter | Value |
|---|---|
| k_y, k_D | 1.2, 0.4 |
| α, η, n/m | 1.65, π/4, 3/5 |
| speed limits | −0.3 to 1.2 m/s, max 1.5 m/s² |
| slots | (±3, ±3) m |

The paper's gains (k_y = k_D = 0.02, β = π/4) are tuned for 15 m/s and errors of tens of metres. At 0.5 m/s they're either far too slow or they saturate the followers.

Keep the path's tightest turn much wider than the formation, or Eq. 21 blows up. The default sinusoid's tightest turn is about 43 m, against a 4.2 m formation radius.

## Repo layout

```
scripts/start_swarm.sh        starts everything
launch/swarm.launch.py        the ROS nodes
swarm_formation/              path_leader, follower_controller, formation_logger, swarm_visualizer
study/step1..7_*.py           offline numpy notebooks, building the controller up step by step
study/step8_ros_harness.py    the ROS controller in pure Python, with a perfect plant
study/step8_evaluate.py       metrics and plots from Gazebo CSVs
study/analyze_formation_csv.m MATLAB plots for one run (works in MATLAB Online)
study/results_step8/          Step 8 report and figures
data/                         CSVs referenced in the reports
```

To analyse a run in MATLAB, upload the CSV and its `run_config.txt` to the same folder, then run `analyze_formation_csv('formation_xxx.csv')`.

## Things I learned the hard way

- PX4 multi-vehicle odometry is relative to each drone's spawn point, not the world. Every node has to add the spawn offset back.
- NED to ENU is `(y, x, -z)`. Get it wrong and everything looks almost right.
- Don't finite-difference the leader's turn rate. The first sample gives the followers a heading kick.
- Command yaw rate, not yaw angle. A 10 Hz yaw-angle setpoint through PX4's ~0.3 s lag made a 0.78 Hz wobble. Switching to 50 Hz yaw rate cut it by more than half.
- Without the start gate, drones that armed late sat in the way of the others. One pair came within 0.42 m.
- PX4 v1.17 renamed some topics (`vehicle_local_position_v1`), which made my launch script wait forever.
- Start PX4 with `-d`, or its shell writes hundreds of MB of logs a minute.

## Limitations and next steps

- Collision avoidance relies on each drone staying in its own quadrant. That only holds if they all start together.
- There's one run per mode so far, so small differences between modes are within the noise.
- Only tested with 4 drones. I still need repeat runs, and the figure-8 and circle paths with the start gate.

The original Humble / Gazebo Classic version is on `main`.

## Citation

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
