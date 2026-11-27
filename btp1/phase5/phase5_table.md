| Run | Law | Gain (η or μ) | Heading noise σ (deg) | Settling (s) | RMS e_y (m) | Effort | Max \|χ̃_i\| (deg) | t(\|χ̃_i\| < 1e-3) (s) | Turn-rate jitter (rad/s) | Clipped (s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| R1 | sign_pow | pi/8 | 0 | 3.16 | 3.05e-04 | 9.5 | 9.8 | 2.75 | – | 0.83 |
| R2 | sign_pow | pi/4 | 0 | 3.46 | 1.71e-04 | 9.3 | 9.8 | 1.30 | – | 0.84 |
| R3 | sign_pow | pi/2 | 0 | 3.50 | 2.78e-04 | 9.3 | 9.8 | 1.11 | – | 0.85 |
| R4 | sat | pi/8 | 0 | 2.42 | 2.22e-04 | 9.8 | 9.8 | 12.94 | – | 0.83 |
| R5 | sat | pi/4 | 0 | 2.79 | 2.87e-04 | 9.6 | 9.8 | 6.69 | – | 0.83 |
| R6 | sat | pi/2 | 0 | 3.29 | 2.61e-04 | 9.4 | 9.8 | 3.42 | – | 0.84 |
| R7 | sign_pow | pi/4 | 0.5 | 3.49 | 1.60e-03 | 12.7 | 10.1 | – | 0.0485 | 0.85 |
| R8 | sat | pi/4 | 0.5 | 2.81 | 1.46e-03 | 9.8 | 10.5 | – | 0.0141 | 0.84 |
| R9 | sign_pow | pi/4 | 1 | 3.49 | 3.12e-03 | 18.0 | 11.2 | – | 0.0770 | 0.85 |
| R10 | sat | pi/4 | 1 | 2.82 | 3.06e-03 | 10.7 | 11.7 | – | 0.0283 | 0.84 |
| R11 | sign_pow | pi/4 | 2 | 3.28 | 6.17e-03 | 33.2 | 13.6 | – | 0.1235 | 0.83 |
| R12 | sat | pi/4 | 2 | 2.63 | 6.43e-03 | 15.8 | 14.0 | – | 0.0565 | 0.82 |

Follower: arccos field, r_min = 20 m, full Δ, ε-fix speed law. Leader: arctan, unchanged. Sat band ε_s = 1 rad, p = 3/5. Noise: white Gaussian on the follower's measured course each 5 ms step, seed 0. RMS e_y uses t > 50 s. Jitter: RMS of χ̇_i(noisy) − χ̇_i(noise-free, same law and gain) for t > 20 s.
