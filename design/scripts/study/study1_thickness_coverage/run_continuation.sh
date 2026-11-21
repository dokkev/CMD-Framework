#!/usr/bin/env bash
# Study 1's search protocol for an architecture whose independent runs end on
# different designs (the CFL; docs/decisions.md, 2026-11-21). From design/:
#
#   bash scripts/study/study1_thickness_coverage/run_continuation.sh [python] [files] [flexions]
#
# After the independent runs (run_all.sh), each condition is searched again
# from the best designs of the same architecture and flexion for the other
# thicknesses (transfer), the thicknesses taken once upward and once downward
# so that every condition receives its neighbours' results. Each search: the
# GA, seeds 0-2, stopped once its best has not improved for 150 generations
# (at most the preset's generations), verified on the fine grid as every run.
#
# Writes output/study/<mechanism>/<file>_q<q>_h<h>_cont/ (the downward pass
# appended); the independent runs' folders are not touched. A condition's
# result is the best of its independent runs and these.
# A condition with no design to transfer from would start from feasible
# linkages sampled with their pads drawn given the linkage
# (exploratory/e14_conditional_pads.py); here every condition has neighbours.
set -u
PYTHON="${1:-python}"
FILES="${2:-cfl cfl_ideal}"
FLEXIONS="${3:-120 90 150}"
THICKNESSES="3 5 7 9"

folder() {  # the independent runs' folder of file $1, flexion $2, thickness $3
  local mechanism
  mechanism=$(awk '/^mechanism:/ {print $2}' "scripts/study/$1.yaml")
  if [ "$3" = 5 ]; then echo "output/study/${mechanism}/$1_q$2"; else echo "output/study/${mechanism}/$1_q$2_h$3"; fi
}

latest() {  # the newest portfolio's seeds of a folder ("" if none)
  ls -d "$1"/portfolio_*/seeds.csv 2>/dev/null | sort | tail -1
}

search() {  # file, flexion, thickness
  local f=$1 q=$2 h=$3 warm="" other seeds
  for other in $THICKNESSES; do
    [ "$other" = "$h" ] && continue
    seeds=$(latest "$(folder "$f" "$q" "$other")_cont")           # the continuation's, when there is one
    [ -z "$seeds" ] && seeds=$(latest "$(folder "$f" "$q" "$other")")
    [ -n "$seeds" ] && warm="${warm:+$warm, }$seeds"
  done
  echo "=== ${f} q${q} h${h} $(date +%H:%M) from [${warm}]"
  "$PYTHON" scripts/optimize/run.py "scripts/study/${f}.yaml" --append \
    --set "goal.motion.required_flexion_deg=${q}" \
    --set "goal.pads.proximal.height_mm=${h}" --set "goal.pads.distal.height_mm=${h}" \
    --set "solver.optimizer.warm_start=[${warm}]" \
    --set "solver.optimizer.presets.search.algorithm=ga" \
    --set "solver.optimizer.presets.search.options={stall_generations: 150}" \
    --tag "q${q}_h${h}_cont" || echo "FAILED ${f} q${q} h${h}"
}

for q in $FLEXIONS; do
  for f in $FILES; do
    for h in 3 5 7 9; do search "$f" "$q" "$h"; done      # upward
    for h in 7 5 3; do search "$f" "$q" "$h"; done        # downward
  done
done
