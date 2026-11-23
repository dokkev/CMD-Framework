#!/bin/bash
# usage: run_batch.sh <contact> <tag> <hand...>: every hand at mu 1.3 / 2.0 and rho 15 / 20 mm
contact=$1; tag=$2; shift 2
for hand in "$@"; do for mu in 1.3 2.0; do for r in 15 20; do
  uv run python two_pad_drake.py --hand "$hand" --mu $mu --radius_mm $r --contact "$contact" --tag "$tag" --workers 8 2>&1 | grep -E "Gate 0|Error"
done; done; done
