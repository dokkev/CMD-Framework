#!/bin/bash
# usage: run_roll.sh <contact> <tag> <workers> <hand...>: rolling task, sweeps 8 and 15 deg, mu 1.3 / 2.0, rho 15 / 20
contact=$1; tag=$2; workers=$3; shift 3
for hand in "$@"; do for s in 8 15; do for mu in 1.3 2.0; do for r in 15 20; do
  uv run python two_pad_drake.py --task roll --sweep_deg $s --spin_damping 2.0 --hand "$hand" --mu $mu --radius_mm $r \
    --contact "$contact" --tag "$tag-s$s" --workers $workers 2>&1 | grep -E "Gate 0|Error"
done; done; done; done
