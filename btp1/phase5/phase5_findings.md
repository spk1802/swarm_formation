# Phase 5 findings: follower reaching law (factor 4)

Run IDs refer to `phase5_table.md`. Follower uses the factor 1–3 choices: ε-fix speed law (ε = 0.05,
K_D = 1), full Δ, arccos field with r_min = 20 m (run_pal_ours.py). Leader unchanged (arctan).
Sign–power: p = 3/5, gain η. Saturation: band ε_s = 1 rad, gain μ. Noise: white Gaussian on the
follower's measured course every 5 ms step (seed 0), added via the new CHI_NOISE switch.

- **Without noise, sign–power is finite-time and saturation is not.** Time for |χ̃_i| to fall below 1e-3 rad:
  2.75 / 1.30 / 1.11 s (sign–power, η = π/8, π/4, π/2, R1–R3) against 12.94 / 6.69 / 3.42 s (saturation,
  R4–R6). The saturation tail is exponential with τ = ε_s/μ (1.27 s at μ = π/4). Sign–power then sits in
  a discrete-time chatter band of 1e-7 to 1e-6 rad (Euler, infinite slope at zero).
- **The reaching law barely changes the position result.** Follower settling 2.4–3.5 s and RMS e_y
  0.17–0.31 mm across R1–R6, effort 9.3–9.8. Position is limited by the field and speed law, not by how
  fast the heading error closes.
- **With heading noise, saturation is much smoother.** Turn-rate jitter (RMS deviation from the noise-free
  run) is 2.2–3.4× lower with saturation: 0.014 vs 0.049, 0.028 vs 0.077, 0.057 vs 0.124 rad/s for
  σ = 0.5, 1, 2° (R7–R12). Control effort grows to 12.7 / 18.0 / 33.2 with sign–power but only
  9.8 / 10.7 / 15.8 with saturation.
- **Noise sets the same position floor for both:** RMS e_y 1.5 / 3.1 / 6.2 mm for σ = 0.5 / 1 / 2°, within
  10% between laws. Saturation buys smoother commands at no cost in tracking.

Caveats: noise tested only at gain π/4; inside its band saturation is linear, so its jitter scales with μ/ε_s.
Noise is white per step; a real heading estimator is filtered and correlated, so absolute jitter values
are indicative only. The ranking matches the slope argument: sign–power has unbounded slope at χ̃ = 0.

## Decision

Use **saturation with μ = π/4, ε_s = 1 rad** for the follower (same as the leader's β, ε). It gives up
the finite-time heading guarantee of Proposition 1 (heading converges exponentially inside the band)
in exchange for 2–3× less turn-rate jitter and 23–52% less effort under noise, with the same tracking.
