#!/usr/bin/env bash
# Study 4 confirmation, nineteenth experiment (report/study3/study3.md): soft 7 mm pads on the hand-independent grid.
# Usage (from sim/isaaclab): run_conclusion_grid.sh ref | run_conclusion_grid.sh [hands...] (default: all 27)
set -euo pipefail
KE=1071.43
GRID=../../design/output/sim/study3/grid_grasps.json
run() {  # hand tag
  for mu in 1.3 2.0; do
    for r in 15 20; do
      uv run --quiet python scripts/study3_measurement/two_pad_wrench.py --hand "$1" --mu "$mu" --condition A \
        --radius_mm "$r" --impratio 1000 --contact_ke "$KE" --grasps "$GRID" --tag "$2"
    done
  done
}
if [ "${1:-}" = "ref" ]; then run O grid-padk-ref7; exit 0; fi
HANDS=("$@")
if [ ${#HANDS[@]} -eq 0 ]; then
  HANDS=($(uv run --quiet python -c "import sys; sys.path.insert(0,'scripts/study3_measurement'); import dose_analysis as da; print(' '.join(da.EXPAND))"))
fi
for hand in "${HANDS[@]}"; do run "$hand" grid-padk; done
