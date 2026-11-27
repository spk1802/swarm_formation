# Final comparison: Basak baseline vs chosen combination

Run IDs refer to `final_table.md`.
Baseline: Basak follower (Δ = 0, arctan field, sign–power η = π/4), guard speed law (ε = 0.01), arctan leader.
Chosen: full Δ, arccos follower field (r_min = 20 m), saturation reaching law (μ = π/4, ε_s = 1 rad),
ε-fix speed law (ε = 0.05, K_D = 1), arccos leader. Cases: nominal (e_y0 = 5 m), start15 (e_y0 = 15 m),
noise1 (nominal + 1° white heading noise on the follower).

- **Nominal (C1 vs C2):** steady RMS e_y 280 mm → 0.29 mm (about 1000× lower); max |e_y| after 20 s
  308 → 0.74 mm. The baseline never settles inside 0.2 m; the chosen follower settles in 2.50 s. Leader
  settling 11.98 → 11.10 s (arccos leader). Follower effort 15.4 → 10.3 (−33%).
- **Speed command:** the baseline guard law sits at a speed limit for 33.0 of 60 s and switches between
  0.5 and 8 m/s (std of v_i after 20 s: 2.72 m/s). The chosen law is clipped 0.83 s, only at the start,
  and then tracks the carousel speed smoothly (std 0.13 m/s).
- **15 m start (C3 vs C4):** same picture: RMS e_y 280 → 0.25 mm, follower settling 3.70 s, clipped
  33.8 → 3.1 s.
- **With 1° heading noise (C5 vs C6):** RMS e_y 282 → 3.1 mm, turn-rate jitter 0.076 → 0.028 rad/s,
  effort 24.7 → 11.5.
- The chosen combination's remaining steady formation error (RMS D ≈ 5 mm) is mostly along-track and
  comes from the ε-fix speed law, as in phases 2–3.
