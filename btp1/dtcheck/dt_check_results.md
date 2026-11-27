# Follower time-step check (chosen combination, nominal start)
Run from BTP1_runs root (script writes controllers/params.py; restore params_nominal afterwards).

| dt (ms) | RMS e_y t>50 s (mm) | RMS D t>50 s (mm) | Follower settling (s) | Leader RMS d (mm) |
| --- | --- | --- | --- | --- |
| 10   | 0.222 | 5.27 | 2.49 | 3.66 |
| 5    | 0.288 | 5.00 | 2.50 | 1.83 |
| 2.5  | 0.321 | 4.87 | 2.50 | 0.92 |
| 1.25 | 0.338 | 4.80 | 2.50 | 0.46 |

Differences halve with dt (first order); limits about 0.35 mm (e_y) and 4.7 mm (D). 5 ms values are within ~20%.
