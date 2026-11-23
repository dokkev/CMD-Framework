#!/bin/bash
# Experiment 25: hydroelastic pads with each hand's own thickness; references O_h (O's geometry, a pad h thick)
run() { uv run python two_pad_drake.py --hand "$1" --mu $2 --radius_mm $3 --contact hydro --pad_kh_mm "$4" --tag "$5" --workers 8 2>&1 | grep -E "Gate 0|Error"; }
for args in "$@"; do
  set -- $args   # "hand kh tag", e.g. "O 7 e25-ref7" or "R own e25"
  for mu in 1.3 2.0; do for r in 15 20; do run "$1" $mu $r "$2" "$3"; done; done
done
