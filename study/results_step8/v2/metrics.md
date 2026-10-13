| metric | none (gazebo) | pointwise (gazebo) | full (gazebo) | none (harness) | pointwise (harness) | full (harness) |
|---|---|---|---|---|---|---|
| t0 -> leader start [s] | 3 | 3 | 3.001 |  |  |  |
| leader t(|d|<0.1) from leader start [s] | 39.14 | 39.14 | 39.16 | 39.4 | 39.4 | 39.4 |
| leader |d| end [m] | 1.305e-05 | 1.289e-05 | 1.308e-05 |  |  |  |
| max D transient [m] | 0.545 | 0.2007 | 0.2258 | 0.3745 | 0.04028 | 0.03142 |
| RMS D steady [m] | 0.06138 | 0.03196 | 0.03308 | 0.05408 | 0.00343 | 0.003292 |
| RMS e_x steady [m] | 0.04402 | 0.0263 | 0.02788 |  |  |  |
| RMS e_y steady [m] | 0.04277 | 0.01817 | 0.01781 |  |  |  |
| all D<0.1 after [s] | 382.9 | 20.02 | 207.6 |  |  |  |
| RMS yaw rate [rad/s] | 0.04869 | 0.0473 | 0.04667 |  |  |  |
| max |yaw rate| [rad/s] | 0.4397 | 0.4973 | 0.4754 |  |  |  |
| heading effort int|chidot| [rad] | 13.83 | 13.5 | 13.19 |  |  |  |
| RMS speed - v_gl [m/s] | 0.05149 | 0.04095 | 0.04161 |  |  |  |
| RMS accel [m/s^2] | 0.07308 | 0.01997 | 0.01986 |  |  |  |
| % time speed>=1.15 | 0 | 0 | 0 |  |  |  |
| 0.6-1.0 Hz yaw power [%] | 17.99 | 21.35 | 22.48 |  |  |  |
| % time |den/D|<0.2 | 6.856 | 7.404 | 8.414 |  |  |  |
| min follower separation [m] | 5.458 | 5.894 | 5.872 |  |  |  |
| min % time in own quadrant | 100 | 100 | 100 |  |  |  |

**none/full transient max-D ratio: gazebo 2.4x vs harness 11.9x; steady RMS-D ratio: gazebo 1.9x**
