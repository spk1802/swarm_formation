# Phase 3 findings: heading correction (factor 2)

Run IDs refer to `phase3_table.md`. Nominal: V_GL = 5 m/s, r_min = 20 m, arctan field, ε-fix speed law
(ε = 0.05, K_D = 1), sign-pow reaching law, start offset e_y = 5 m. Steady-state values use t > 50 s.

- **Uncorrected (H1) never settles.** e_y follows the path curvature, peaking near ±0.31 m at the bends and
  crossing zero at the inflection near t = 40 s (ω = 0). The parking estimate −tan Δ / k_y gives 0.32 m at
  the peak Δ = 2.35°, matching the measured 0.308 m. RMS e_y 0.281 m.
- **The uncorrected follower still reaches roughly the right relative course,** but only by sitting off its
  slot so the vector field turns it through Δ. It lags the predicted Δ by up to 5.0e-3 rad (0.29°).
- **Both corrected levels remove the offset.** χ_i − χ_l tracks the predicted Δ to within 1e-4 rad after
  t = 20 s, and RMS e_y drops by a factor of 490 (pointwise) and 610 (full).
- **Full beats pointwise on every metric:** settling 4.00 vs 6.59 s, RMS e_y 0.46 vs 0.57 mm, max |e_y|
  (t > 20 s) 0.78 vs 1.13 mm, effort 9.1 vs 10.3, time clipped 0.85 vs 1.79 s. The gain is largest in the
  transient, when ω changes fastest.

Caveats: max flown curvature (1.6–2.7 1/m) is not reported in the thesis table because the peaks occur
while the follower speed is clipped at V_MIN = 0.5 m/s in the first seconds. The remaining sub-mm e_y in
H2/H3 is dominated by the ε-fix speed-law floor (D still decaying at 60 s), not by the heading correction.

## Decision

Use **full** (Δ and Δ̇). Pointwise is the fallback if ω̇ is too noisy to estimate (e.g. SITL with a
measured leader); it keeps most of the improvement.
