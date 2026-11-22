#!/usr/bin/env bash
# Study 3, fifteenth experiment (separability.md): pad contact stiffness that falls with thickness,
# ke = 2.5e3 * 3 / h N/m, condition A, tag padk. Run from sim/isaaclab.
set -euo pipefail
declare -A H=( [O]=3 [R]=7 [d1p5_minDs_s0]=7 [d2p25_minDs_s0]=7 [d2p25_maxDs_s0]=7 [d3_minDs_s0]=7 [U]=7 [d4p5_s0]=7
  [d5p5_s0]=7 [d1p5_maxDs_s0]=7 [d3_maxDs_s0]=7 [h3to5_hr]=5 [h3to5_hu]=5 [h3to5_d3_s0]=5 [h3to5_d3_minDs_s0]=5
  [h3to9_hr]=9 [h3to9_hu]=9 [h3to9_d3p5_maxDs_s0]=9 [h3to9_d5_minDs_s0]=9 )
for hand in O R U h3to5_hr h3to5_hu h3to9_hr h3to9_hu d1p5_minDs_s0 d2p25_minDs_s0 d2p25_maxDs_s0 d3_minDs_s0 \
    d4p5_s0 d5p5_s0 d1p5_maxDs_s0 d3_maxDs_s0 h3to5_d3_s0 h3to5_d3_minDs_s0 h3to9_d3p5_maxDs_s0 h3to9_d5_minDs_s0; do
  ke=$(python3 -c "print(2.5e3*3/${H[$hand]})")
  for mu in 1.3 2.0; do
    for r in 15 20; do
      uv run --quiet python scripts/study3_measurement/two_pad_wrench.py --hand "$hand" --mu "$mu" --condition A \
        --radius_mm "$r" --impratio 1000 --contact_ke "$ke" --tag padk
    done
  done
done
