# BTP1 study

The offline Python study and SITL analysis for my BTP1 (B.Tech Project, part 1). See the
"BTP1" section of the [top-level README](../README.md) for the summary.

Run every command from this `btp1/` folder. The `.mat` result files are not in git (about 140 MB);
re-run the drivers below to regenerate them. Tables, findings, figures and the SITL logs are kept.

```
controllers/        the simulation code (params.py, basak.py, run_ours.py, run_pal.py, run_pal_ours.py)
plotting/           plot_compare.py (overlay one variable from several runs) and plot_style.py
sanity_checks.py    run before each phase; must print ALL PASSED
nominal/results/    .mat files from sanity_checks.py or a plain controller run
phase1/             leader study: arctan vs arccos field
  run_phase1.py       runs all 22 leader cases  -> phase1/results/, phase1/table_runs.md
  plot_phase1.py      makes all figures         -> phase1/figures/, phase1/table_paper.md
  results/            .mat files + phase1_rows.json
  figures/overview/   quick comparison plots (d, curvature, xy, settling bars)
  figures/paper/      3x2 (a)-(f) figures in the style of Pal & Hota
phase3/             factor 2: heading correction (uncorrected / pointwise / full)
  phase3_driver.py    runs H1-H3 -> phase3/results/, phase3_rows.json, phase3_table.md
  phase3_figs.py      -> phase3/figures/report/fig_p3_heading_correction.png
phase4/             factor 3: follower field shape (arctan / arccos, r_min sweep)
  phase4_driver.py    runs F1-F8 -> phase4/results/, phase4_rows.json, phase4_table.md
  phase4_figs.py      -> phase4/figures/report/fig_p4_follower_field.png
phase5/             factor 4: follower reaching law (sign-pow / sat, gain sweep, heading noise)
  phase5_driver.py    runs R1-R12 -> phase5/results/, phase5_rows.json, phase5_table.md
  phase5_figs.py      -> phase5/figures/report/fig_p5_reaching_law.png
final/              chapter 7: Basak baseline vs chosen combination (nominal, 15 m start, 1 deg noise)
  final_driver.py     runs C1-C6 -> final/results/, final_rows.json, final_table.md
  final_figs.py       -> final/figures/report/fig_final_comparison.png
sitl/               chapter 7 SITL: PX4/Gazebo logs (copied) + analysis
  logs/               baseline, chosen, chosen_repeat, chosen_course, chosen_lead (.csv)
  sitl_figs.py        -> sitl_table.md, figures/report/fig_sitl_overlay.png
carousel/           carousel_figs.py -> carousel/figures/report/carousel_vector_diagram.png + captions.txt
                      (concept sketch from params_nominal.py; no simulation needed)
old_2026-11-04/     backup of the earlier code and results
```

Figures are saved as PNG only.

## Controllers

| File | What it is |
| --- | --- |
| `controllers/params.py` | every value used by the runs, plus the factor switches |
| `controllers/params_nominal.py` | base config the phase drivers start from (keep in sync with `params.py`) |
| `controllers/basak.py` | arctan field, no Δ |
| `controllers/run_ours.py` | arctan field + Δ (`DELTA_MODE`) |
| `controllers/run_pal.py` | arccos field, no Δ |
| `controllers/run_pal_ours.py` | arccos field + Δ (`DELTA_MODE`) |

The four run files are identical except `NAME`, `HEAD` and two lines in `follower_law`.

### Factor switches in `controllers/params.py`

| Switch | Values |
| --- | --- |
| `LEADER_FIELD` | `"arctan"`, `"arccos"` |
| `SPEED_LAW` | `"guard"` (Eq. 21 + EPS_DEN floor), `"epsfix"` ((num·den + v_b·EPS_FIX²)/(den² + EPS_FIX²); v_b = carousel slot speed v_req with Δ, V_GL without; from swarm_formation follower_controller.py) |
| `DELTA_MODE` | `"pointwise"`, `"full"` (used by the two Δ files) |
| `SMC_LAW` | `"sign_pow"` (ETA, P), `"sat"` (MU, EPS_SAT) |
| `TAG` | extra text added to the result filename |
| `RESULTS_DIR` | where `.mat` files go (`"nominal/results"`; phase drivers override it) |

## Run

```bash
python3 sanity_checks.py
python3 controllers/basak.py
```

Result: `<RESULTS_DIR>/<name>_<head>_L-<leader field>_<speed law>_<smc law>_rmin<..>_chi<..>_off<..><TAG>.mat`
holding `t, xl, yl, chi_l, omega, d, chi_t_l, xi, yi, chi_i, chidot_i, u_i, v_i, ex, ey, chi_t_i, Delta, den`,
the metrics, and `params` (all values used).

## Phase 1

```bash
PYTHONDONTWRITEBYTECODE=1 python3 phase1/run_phase1.py
python3 phase1/plot_phase1.py
```

## Plot any runs

```bash
python3 plotting/plot_compare.py ey nominal/results/basak_*.mat nominal/results/ours_*.mat --labels Basak Ours --xlim 0 40 --out ey_nominal
python3 plotting/plot_compare.py d phase1/results/a.mat phase1/results/b.mat --abs --log --out phase1/figures/overview/leader_d
```

Variables: any saved array, plus `D`, `kappa_i`, `kappa_l`, `rel` (χ_i − χ_l), and `xy`.
Options: `--labels`, `--log`, `--abs`, `--xlim a b`, `--ylim a b`, `--hline v ...`, `--overlay var`, `--out name`.
`--out name` saves `figures/<name>.png`; `--out some/folder/name` saves `some/folder/name.png`.

## Carousel figure

```bash
python3 carousel/carousel_figs.py
```

## Phase 3 (factor 2: heading correction)

```bash
PYTHONDONTWRITEBYTECODE=1 python3 phase3/phase3_driver.py
python3 phase3/phase3_figs.py
```

## Phase 4 (factor 3: follower field shape)

```bash
PYTHONDONTWRITEBYTECODE=1 python3 phase4/phase4_driver.py
python3 phase4/phase4_figs.py
```

## Phase 5 (factor 4: follower reaching law)

```bash
PYTHONDONTWRITEBYTECODE=1 python3 phase5/phase5_driver.py
python3 phase5/phase5_figs.py
```

CHI_NOISE (rad, std of white noise on the follower's measured course each step) and NOISE_SEED were added to
params_nominal.py / params.py and to all four run files. Default 0.0 reproduces earlier results exactly.

## Final comparison (chapter 7)

```bash
PYTHONDONTWRITEBYTECODE=1 python3 final/final_driver.py
python3 final/final_figs.py
```

## SITL (chapter 7)

Runs come from `~/ros2_ws/src/swarm_formation/scripts/start_swarm.sh --btp baseline|chosen|chosen_lead`;
copy the CSVs into `sitl/logs/` (see `sitl/sitl_findings.md` for which run is which), then

```bash
python3 sitl/sitl_figs.py
```
