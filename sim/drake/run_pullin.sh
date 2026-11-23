#!/bin/bash
# usage: run_pullin.sh <contact> <tag> <workers> <hand...>: grasp acquisition, offsets 3 and 6 mm, mu 1.3 / 2.0, rho 15 / 20
contact=$1; tag=$2; workers=$3; shift 3
for hand in "$@"; do for a in 3 6; do for mu in 1.3 2.0; do for r in 15 20; do
  uv run python two_pad_drake.py --task pullin --offset_mm $a --tip_load_n 1.5 --face_load_n 1.5 --spin_damping 2.0 \
    --hand "$hand" --mu $mu --radius_mm $r --contact "$contact" --tag "$tag-a$a" --workers $workers 2>&1 | grep -E "Error"
done; done; done; done
