| metric | none (gazebo) | pointwise (gazebo) | full (gazebo) | none (harness) | pointwise (harness) | full (harness) |
|---|---|---|---|---|---|---|
| t0 -> leader start [s] | 3 | 3 | 3 |  |  |  |
| leader t(|d|<0.1) from leader start [s] | 51.08 | 51.06 | 51.08 | 51.3 | 51.3 | 51.3 |
| leader |d| end [m] | 1.267e-05 | 1.215e-05 | 1.165e-05 |  |  |  |
| max D transient [m] | 0.8742 | 0.2556 | 0.2375 | 0.5612 | 0.04301 | 0.03308 |
| RMS D steady [m] | 0.05985 | 0.033 | 0.03113 | 0.054 | 0.003227 | 0.002551 |
| RMS e_x steady [m] | 0.04315 | 0.02777 | 0.02554 |  |  |  |
| RMS e_y steady [m] | 0.04148 | 0.01783 | 0.0178 |  |  |  |
| all D<0.1 after [s] | 351.4 | 33.7 | 25.12 |  |  |  |
| RMS yaw rate [rad/s] | 0.05901 | 0.04961 | 0.04799 |  |  |  |
| max |yaw rate| [rad/s] | 0.457 | 0.5494 | 0.5225 |  |  |  |
| heading effort int|chidot| [rad] | 15.8 | 13.17 | 13.92 |  |  |  |
| RMS speed - v_gl [m/s] | 0.07295 | 0.05518 | 0.0527 |  |  |  |
| RMS accel [m/s^2] | 0.1104 | 0.02048 | 0.02018 |  |  |  |
| % time speed>=1.15 | 0 | 0 | 0 |  |  |  |
| 0.6-1.0 Hz yaw power [%] | 13.34 | 20.78 | 22.02 |  |  |  |
| % time |den/D|<0.2 | 6.975 | 7.261 | 7.667 |  |  |  |
| min follower separation [m] | 5.467 | 5.84 | 5.896 |  |  |  |
| min % time in own quadrant | 100 | 100 | 100 |  |  |  |

**none/full transient max-D ratio: gazebo 3.7x vs harness 17.0x; steady RMS-D ratio: gazebo 1.9x**
