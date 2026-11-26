#!/usr/bin/env bash
# Re-export R and U of the 5 and 9 mm Study 3 pairs with the current exporter (servo shapes and visuals, which the
# committed design/output exports of these pairs predate; the joints, loops, four-bar tables, pads and collision pairs
# are identical) into sim/isaaclab/logs/rotate_cube/transfer/hands, where grasp.HANDS (R5, U5, R9, U9) reads them.
# Also RL, RL60, UL120 and UL, the long fingers (L_f 120, 120, 120 and 150 mm, 7 mm pads; design/scripts/study/study3_trajectory/long_fingers.py).
# Also O_thumbopt: O's fingers with the thumb-layout optimum (base offsets yaw -14 / pitch -16 / roll +14 deg, CMC servo
# at x -50.5, z -10.5 mm, IP servo turned toward the MCP, fingertip case 34 mm; docs/experimental/thumb_layout_opt.md),
# its parameters written by design/scripts/optimize/thumb_layout/write_hand.py.
# Needs the design environment (conda env cmd). The 7 mm pair's committed exports reproduce byte for byte (sans the
# source hash) with this exporter.
set -e
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
OUT=$ROOT/sim/isaaclab/logs/rotate_cube/transfer/hands
mkdir -p "$OUT"
cd "$ROOT/design"
for h in h3to5_hr h3to5_hu h3to9_hr h3to9_hu L120_h3to7_hr L150_h3to7_hu L120_h3to7_hu L120_h3to7_hr60; do
  python scripts/export/code_hand_sim.py --params output/sim/study3/hands/cfl_q120_$h.yaml --out "$OUT/cfl_q120_$h.json"
done
python scripts/optimize/thumb_layout/write_hand.py --hand h0 --x -50.5 --z -10.5 --extension 34 --orientation proximal \
  --yaw -14 --pitch -16 --roll 14 --note "O orientation optimum (functional Kapandji K8, pinch 8.8 deg)" \
  --out output/sim/study3/hands/cfl_q120_h3to7_h0_thumbopt.yaml
python scripts/export/code_hand_sim.py --params output/sim/study3/hands/cfl_q120_h3to7_h0_thumbopt.yaml \
  --out "$OUT/cfl_q120_h3to7_h0_thumbopt.json"
