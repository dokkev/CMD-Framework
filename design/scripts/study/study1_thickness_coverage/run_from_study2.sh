#!/usr/bin/env bash
# Study 1, last pass of the search protocol (docs/decisions.md, 2026-11-21): each CFL / CFL ideal condition
# at 5, 7 and 9 mm searched once more from its own best designs (independent runs and continuation) and the
# designs Study 2 found for that thickness (study2_redesign.py's fronts, written as warm-start files under
# <study2 output>/warm_start/). Same GA, seeds 0-2, stop after 150 generations without improvement; appended
# to output/study/<mechanism>/<file>_q<q>_h<h>_cont. From design/:
#
#   bash scripts/study/study1_thickness_coverage/run_from_study2.sh [python] [study2 output]
set -u
PYTHON="${1:-python}"
STUDY2="${2:-output/study/study2_redesign_2026-11-21}"
latest() { ls -d "$1"/portfolio_*/seeds.csv 2>/dev/null | sort | tail -1; }
for q in 120 90 150; do
  for f in cfl cfl_ideal; do
    mechanism=$(awk '/^mechanism:/ {print $2}' "scripts/study/$f.yaml")
    for h in 5 7 9; do
      own="output/study/${mechanism}/${f}_q${q}_h${h}"; [ "$h" = 5 ] && own="output/study/${mechanism}/${f}_q${q}"
      warm="${STUDY2}/warm_start/${f}_q${q}_h${h}.csv"
      for s in "$(latest "$own")" "$(latest "output/study/${mechanism}/${f}_q${q}_h${h}_cont")"; do
        [ -n "$s" ] && warm="${warm}, ${s}"
      done
      echo "=== ${f} q${q} h${h} $(date +%H:%M) from [${warm}]"
      "$PYTHON" scripts/optimize/run.py "scripts/study/${f}.yaml" --append \
        --set "goal.motion.required_flexion_deg=${q}" \
        --set "goal.pads.proximal.height_mm=${h}" --set "goal.pads.distal.height_mm=${h}" \
        --set "solver.optimizer.warm_start=[${warm}]" \
        --set "solver.optimizer.presets.search.algorithm=ga" \
        --set "solver.optimizer.presets.search.options={stall_generations: 150}" \
        --tag "q${q}_h${h}_cont" || echo "FAILED ${f} q${q} h${h}"
    done
  done
done
