# Phase 2 findings: speed law (factor 1)

Run IDs refer to `phase2_table.md`. Nominal: V_GL = 5 m/s, r_min = 20 m, arctan field, start offset e_y = 5 m.
Steady-state values use t > 50 s.

- **ε-fix wins, mainly by removing the guard's speed chatter.** Without Δ (basak.py) the guard at ε = 0.01
  switches between V_MIN and V_MAX almost every step and sits clipped for 32.98 s of 60 s (V3); ε-fix at
  ε = 0.05 is smooth and clipped for 0.78 s (V9). Tracking is unchanged by the speed law there
  (RMS e_y 0.280 vs 0.281 m): without Δ the follower keeps a ~0.28 m steady formation error on the curve
  whichever law is used, so the speed law cannot fix that part.
- **With Δ (run_ours.py), ε-fix also removes the along-track bias and the error floor.** The guard parks
  e_x at −ε (−0.0100 m at ε = 0.01, V4), because it holds the Eq. 21 denominator at −ε (fig (d)).
  ε-fix at ε = 0.05 gives mean e_x −0.0043 m, RMS e_y 4.6e-4 m vs 9.7e-4 m, and 0.85 s clipped vs
  1.57 s (V10 vs V4).
- **For the guard, ε is a hard floor; for ε-fix it is not.** Guard final D tracks ε almost exactly:
  0.099 / 0.0101 / 0.00105 m for ε = 0.1 / 0.01 / 0.001 (V2, V4, V6), flat after ~12 s. ε-fix final D is
  about 12–16× below its ε and still falling at 60 s: 0.0085 / 0.0043 / 0.00064 m for ε = 0.1 / 0.05 / 0.01
  (V8, V10, V12). A smaller ε lowers the floor but brings chatter back without Δ: the guard at ε = 0.001
  is clipped 37.08 s (V5), and ε-fix at ε = 0.01 is clipped 16.14 s (V11) against 0.78 s at ε = 0.05.
- **K_D speeds up the initial decay only up to about K_D = 1.** With Δ, the ln D decay rate is
  0.45 / 0.69 / 0.65 1/s for K_D = 0.5 / 1 / 2 with ε-fix (V18, V10, V20), and 0.50 / 0.69 / 0.66 1/s with
  the guard (V14, V4, V16). That is below the K_D the speed law alone would give, so above K_D ≈ 1 another
  part of the loop limits the decay. Doubling K_D to 2 doubles the clipped time (ε-fix 0.85 → 1.81 s, guard
  1.57 → 4.49 s; a V_MAX burst near t = 2 s) and lowers the ε-fix final D only from 0.0043 to 0.0029 m.
  K_D = 0.5 never clips with ε-fix (0.00 s) but ends with a larger D (0.0073 m) and positive mean e_x.
- **The regularisation is engaged throughout steady state.** With ε-fix, |den| < ε for the whole 10 s
  after t = 50 s in every case except basak at ε = 0.01 (7.42 s). This is by design: near the slot the
  law blends smoothly toward the slot speed instead of dividing by a tiny denominator. With the guard,
  |den| < ε for about 6 s of those 10 s (with Δ, ε = 0.01), as the denominator dithers around −ε.

Caveats: max |e_y| is 5.000 m in every run because it is the initial offset, so that column carries no
information. The decay rate is only meaningful for run_ours.py. For basak.py, D plateaus at ~0.28 m
inside the 0.05–1 m window, so the fitted slope mixes in the plateau.

## Recommendation (not a decision)

Use **ε-fix with ε = EPS_FIX = 0.05 and K_D = 1** for the remaining phases. It is the smallest ε tested
that keeps clipping under 1 s with and without Δ, it has no parked bias, and K_D = 1 gets the fastest
measured decay without the extra clipping of K_D = 2. ε = 0.01 is worth considering if only Δ-on runs
matter, since its floor is ~7× lower, but it chatters without Δ (16 s clipped). This would mean setting
`SPEED_LAW = "epsfix"` in params.py and params_nominal.py. That edit has not been made.
