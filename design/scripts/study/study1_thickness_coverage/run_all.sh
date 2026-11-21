#!/usr/bin/env bash
# Study 1 (problem formulation A.6.2): each architecture's best design for each
# pad thickness and required flexion, the study's own objective (the
# cylinder-based pad coverage) and budget unchanged. From design/:
#
#   bash scripts/study/study1_thickness_coverage/run_all.sh [python]
#
# Writes output/study/<mechanism>/<file>_q<q>_h<h>/portfolio_NNN; a condition
# already run is skipped. Thickness 5 mm is the native designs' own
# (output/study/<mechanism>/<file>_q<q>), not run again.
set -u
PYTHON="${1:-python}"
for q in 120 90 150; do
  for h in 3 7 9; do
    for f in cfl 2r_dd 2r_remote rcj cfl_ideal rcj_ideal; do
      if compgen -G "output/study/*/${f}_q${q}_h${h}/portfolio_*" > /dev/null; then
        echo "skip ${f} q${q} h${h}"; continue
      fi
      echo "=== ${f} q${q} h${h} $(date +%H:%M)"
      "$PYTHON" scripts/optimize/run.py "scripts/study/${f}.yaml" \
        --set "goal.motion.required_flexion_deg=${q}" \
        --set "goal.pads.proximal.height_mm=${h}" --set "goal.pads.distal.height_mm=${h}" \
        --tag "q${q}_h${h}" || echo "FAILED ${f} q${q} h${h}"
    done
  done
done
