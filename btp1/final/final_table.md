| Run | Controller | Case | Leader settling (s) | Follower settling (s) | RMS e_y (m) | Max \|e_y\| t>20 s (m) | RMS D (m) | Effort | Clipped (s) | Std v_i t>20 s (m/s) | Jitter (rad/s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| C1 | baseline | nominal | 11.98 | – | 2.80e-01 | 3.08e-01 | 2.80e-01 | 15.4 | 32.98 | 2.715 | – |
| C2 | chosen | nominal | 11.10 | 2.50 | 2.88e-04 | 7.42e-04 | 5.00e-03 | 10.3 | 0.83 | 0.133 | – |
| C3 | baseline | start15 | 11.98 | – | 2.80e-01 | 3.08e-01 | 2.80e-01 | 9.2 | 33.81 | 2.709 | – |
| C4 | chosen | start15 | 11.10 | 3.70 | 2.54e-04 | 5.80e-04 | 4.57e-03 | 8.8 | 3.13 | 0.133 | – |
| C5 | baseline | noise1 | 11.98 | – | 2.82e-01 | 3.12e-01 | 2.83e-01 | 24.7 | 30.28 | 2.632 | 0.0763 |
| C6 | chosen | noise1 | 11.10 | 2.52 | 3.06e-03 | 1.62e-02 | 3.54e-03 | 11.5 | 0.83 | 0.133 | 0.0283 |

Baseline: Basak follower (Δ = 0, arctan field, sign-pow η = π/4), guard speed law ε = 0.01, arctan leader. Chosen: full Δ, arccos follower field (r_min = 20 m), saturation μ = π/4 / ε_s = 1 rad, ε-fix speed law ε = 0.05, arccos leader. nominal: e_y0 = 5 m; start15: e_y0 = 15 m; noise1: nominal + 1° white heading noise on the follower. RMS values use t > 50 s. Jitter: RMS χ̇_i(noisy) − χ̇_i(nominal, same controller) for t > 20 s.
