#!/usr/bin/env bash
# Attempt 9 (docs/experimental/rl_pip_only.md): export the Study 3 hand grid for the finger-walk transfer into
# sim/isaaclab/logs/finger_walk/hands, where code_isaaclab.tasks.finger_walk.grid reads them.
# - every cfl_q120_h3to{5,7,9}_d*.yaml redesign (e18, design/scripts/study/exploratory/e18_rct_dose.py) with the
#   current exporter, the one O, R and U's exports come from (design/scripts/export/code_hand_sim.py); h3to7_d1p5_s1
#   is skipped (its parameters are h3to7_d1p5_s0's, byte for byte);
# - the fixed-axis family fa_h{3,5,7,9} rebuilt by sim/isaaclab/scripts/study3_measurement/fixed_axis_hands.py's
#   build() from O's current export (the committed fa_q120_*.json predate the palm housing; same kinematics).
# Needs the design environment (conda env cmd).
set -e
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
OUT=$ROOT/sim/isaaclab/logs/finger_walk/hands
mkdir -p "$OUT"
cd "$ROOT/design"
for y in output/sim/study3/hands/cfl_q120_h3to[579]_d*.yaml; do
  s=$(basename "$y" .yaml)
  [ "$s" = cfl_q120_h3to7_d1p5_s1 ] && continue
  [ -f "$OUT/$s.json" ] || conda run -n cmd python scripts/export/code_hand_sim.py --params "$y" --out "$OUT/$s.json" > /dev/null
done
python3 - "$ROOT" "$OUT" <<'EOF'
import json, sys
from pathlib import Path
root, out = Path(sys.argv[1]), Path(sys.argv[2])
sys.path.insert(0, str(root / "sim/isaaclab/scripts/study3_measurement"))
import fixed_axis_hands as fa
o = json.loads((root / "design/output/sim/study3/hands/cfl_q120_h3to7_h0.json").read_text())
for h in fa.THICKNESS_MM:
    d = fa.build(o, h)
    d["source"]["derived"] = f"fixed_axis_hands.py build() from cfl_q120_h3to7_h0.json ({o['source']['date']}), h {h:g} mm"
    (out / f"fa_q120_h{h}.json").write_text(json.dumps(d, indent=1))
EOF
ls "$OUT" | wc -l
