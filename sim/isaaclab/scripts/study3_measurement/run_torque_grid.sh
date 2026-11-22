#!/usr/bin/env bash
# Study 3, eighteenth experiment (separability.md): condition C on the hand-independent grid, O and the 27 hands of
# the thirteenth experiment. Usage (from sim/isaaclab): run_torque_grid.sh [hands...]  (default: all, O first)
set -euo pipefail
HANDS=("$@")
if [ ${#HANDS[@]} -eq 0 ]; then
  HANDS=(O $(uv run --quiet python -c "import sys; sys.path.insert(0,'scripts/study3_measurement'); import dose_analysis as da; print(' '.join(da.EXPAND))"))
fi
for hand in "${HANDS[@]}"; do
  for mu in 1.3 2.0; do
    for r in 15 20; do
      uv run --quiet python scripts/study3_measurement/two_pad_wrench.py --hand "$hand" --mu "$mu" --condition C \
        --radius_mm "$r" --impratio 1000 --grasps ../../design/output/sim/study3/grid_grasps.json --tag grid
    done
  done
done
