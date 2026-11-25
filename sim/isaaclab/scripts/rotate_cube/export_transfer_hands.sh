#!/usr/bin/env bash
# Re-export R and U of the 5 and 9 mm Study 3 pairs with the current exporter (servo shapes and visuals, which the
# committed design/output exports of these pairs predate; the joints, loops, four-bar tables, pads and collision pairs
# are identical) into sim/isaaclab/logs/rotate_cube/transfer/hands, where grasp.HANDS (R5, U5, R9, U9) reads them.
# Needs the design environment (conda env cmd). The 7 mm pair's committed exports reproduce byte for byte (sans the
# source hash) with this exporter.
set -e
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
OUT=$ROOT/sim/isaaclab/logs/rotate_cube/transfer/hands
mkdir -p "$OUT"
cd "$ROOT/design"
for h in h3to5_hr h3to5_hu h3to9_hr h3to9_hu; do
  python scripts/export/code_hand_sim.py --params output/sim/study3/hands/cfl_q120_$h.yaml --out "$OUT/cfl_q120_$h.json"
done
