# Phase 4 findings: follower field shape (factor 3)

Run IDs refer to `phase4_table.md`. Nominal settings plus the factor 1–2 choices: ε-fix speed law
(ε = 0.05, K_D = 1), full Δ, sign-pow reaching law. The leader is identical in every run (arctan field,
r_min = 20 m gain, leader settling 11.98 s); only the follower field and its gain change.

- **The arccos field settles the follower faster in every case:** 2.29 vs 2.44 s (r_min = 10 m),
  3.46 vs 4.00 s (20 m), 4.50 vs 5.21 s (30 m) and 5.51 vs 6.07 s (20 m, 15 m start), i.e. 6–14% faster.
  Effort is within 3% (F1–F8).
- **The difference is only in the final approach.** Above |e_y| ≈ 0.3 m the two curves overlap: the
  follower starts with the speed at V_MIN (clipped ~0.8 s for a 5 m start, 3.1 s for 15 m), and in that
  phase the turn is set by the reaching law and the speed law, not by the field shape. The arccos field's
  steeper slope near e_y = 0 (φ'(0) = −C against −(2/π)χ∞K) is what closes the last 0.3 m faster.
- **Smaller r_min (higher gain) settles faster for both fields:** about 2.4 / 4 / 5 s for r_min = 10 / 20 / 30 m.
  Steady RMS e_y stays sub-mm in all runs (0.17–0.64 mm) and is set mainly by the speed law.
- **Curvature:** once the speed leaves its limit, the flown curvature stays inside ±1/r_min and is almost
  the same for both fields (fig (d)). The large values in the first seconds (up to ~2.6 1/m) occur only
  while v_i = V_MIN, where curvature = turn rate / 0.5 m/s.

Caveat: the straight-line curvature bound behind the gain rule is only approximate for the follower,
whose field acts on e_y in the rotating leader frame.

## Decision

Use the **arccos follower field with r_min = 20 m**. r_min is the vehicle's turn limit, so it is not tuned
for speed; 10 m settles faster but asks for turns the vehicle cannot fly.
