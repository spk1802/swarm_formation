#!/usr/bin/env bash
#
# index_sim_logs.sh -- keep a record of every simulation log under ~/sim_logs.
#
# Layout:
#   ~/sim_logs/sitl/runs/<timestamp>/   start_swarm.sh runs (CSV + run_config.txt + process logs)
#   ~/sim_logs/sitl/step8/<tag>/        Step 8 evaluation runs (+ <tag>.console / .validation)
#   ~/sim_logs/sitl/standalone/         formation_logger run without start_swarm.sh
#   ~/sim_logs/numpy/                   older numpy plots (current ones are in study/figures, in git)
#
# Usage:
#   ./index_sim_logs.sh            # rewrite INDEX.md, append new files to MANIFEST.sha256
#   ./index_sim_logs.sh --verify   # check every file in MANIFEST.sha256 is present and unchanged
#
# MANIFEST.sha256 is append-only: files already recorded keep their original
# checksum, so --verify catches anything that was later lost or overwritten.
#
set -euo pipefail

ROOT="${SIM_LOGS:-$HOME/sim_logs}"
cd "$ROOT"
MANIFEST="MANIFEST.sha256"
touch "$MANIFEST"

if [[ "${1:-}" == "--verify" ]]; then
  sha256sum -c --quiet "$MANIFEST" && echo "OK: $(wc -l < "$MANIFEST") files verified"
  exit
fi

# --- manifest: checksum files not yet recorded ---
export LC_ALL=C
new=0
while IFS= read -r f; do
  sha256sum "$f" >> "$MANIFEST"; new=$((new + 1))
done < <(comm -13 <(cut -c67- "$MANIFEST" | sort) <(find ./sitl ./numpy -type f | sort))

# --- index ---
cfg() { grep -m1 "^$2=" "$1/run_config.txt" 2>/dev/null | cut -d= -f2- || true; }
dur() {  # seconds covered by a formation CSV (first column is t)
  awk -F, 'NR==2{a=$1} END{if (NR>1) printf "%.0f", $1-a}' "$1" 2>/dev/null
}

{
  echo "# Simulation log index"
  echo
  echo "Generated $(date '+%Y-%m-%d %H:%M') by \`scripts/index_sim_logs.sh\`. Do not edit by hand."
  echo "Total: $(du -sh sitl | cut -f1) SITL data, $(wc -l < "$MANIFEST") files in MANIFEST.sha256."
  echo
  echo "## SITL runs (\`sitl/runs/\`)"
  echo
  echo "| run | leader | n | path_offset | delta | delta_dot | tag | git | CSV s | size |"
  echo "|---|---|---|---|---|---|---|---|---|---|"
  for d in sitl/runs/*/; do
    [[ -L "${d%/}" ]] && continue
    d="${d%/}"; csv=$(ls "$d"/formation_*.csv 2>/dev/null | head -1 || true)
    echo "| $(basename "$d") | $(cfg "$d" leader) | $(cfg "$d" n_drones) | $(cfg "$d" path_offset) |" \
         "$(cfg "$d" enable_delta) | $(cfg "$d" enable_delta_dot) | $(cfg "$d" tag) | $(cfg "$d" git_rev) |" \
         "${csv:+$(dur "$csv")} | $(du -sh "$d" | cut -f1) |"
  done
  echo
  echo "## Step 8 runs (\`sitl/step8/\`)"
  echo
  echo "| tag | leader | path_offset | delta | delta_dot | git | CSV s | validation |"
  echo "|---|---|---|---|---|---|---|---|"
  for d in sitl/step8/*/; do
    d="${d%/}"; t=$(basename "$d"); csv=$(ls "$d"/formation_*.csv 2>/dev/null | head -1 || true)
    v="-"; [[ -f "$d.validation" ]] && v=$(tail -1 "$d.validation")
    echo "| $t | $(cfg "$d" leader) | $(cfg "$d" path_offset) | $(cfg "$d" enable_delta) |" \
         "$(cfg "$d" enable_delta_dot) | $(cfg "$d" git_rev) | ${csv:+$(dur "$csv")} | $v |"
  done
  echo
  echo "## Standalone logger CSVs (\`sitl/standalone/\`)"
  echo
  echo "| file | CSV s | size |"
  echo "|---|---|---|"
  for f in sitl/standalone/*.csv; do
    [[ -e "$f" ]] || continue
    echo "| $(basename "$f") | $(dur "$f") | $(du -h "$f" | cut -f1) |"
  done
  echo
  echo "## Other"
  echo
  for d in sitl/*/; do
    case "$(basename "$d")" in runs|step8|standalone) continue ;; esac
    echo "- \`$d\`: $(ls "$d" | tr '\n' ',' | sed 's/,$//; s/,/, /g')"
  done
  for d in numpy/*/; do
    [[ -d "$d" ]] && echo "- \`$d\`: $(ls "$d" | tr '\n' ',' | sed 's/,$//; s/,/, /g')"
  done
  echo "- numpy study figures and Step 8 reports: \`study/figures/\`, \`study/results_step8/\` (in git)"
} > INDEX.md

echo "INDEX.md written; $new new file(s) added to $MANIFEST"
