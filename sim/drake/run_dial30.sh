#!/bin/bash
# Experiment 26: the 30-degree dial. usage: run_dial30.sh <tag> <workers> [--delta_deg D] -- <hand...>
tag=$1; workers=$2; shift 2
extra=()
while [ "$1" != "--" ]; do extra+=("$1"); shift; done; shift
for hand in "$@"; do for mu in 1.3 2.0; do for r in 10 12; do
  uv run python two_pad_drake.py --task dial --dial_k 300 --brake_nm 0 --gap_mm 3 --sweep_deg 45 --hand "$hand" \
    --mu $mu --radius_mm $r --grasps logs/grasps_r$r.json --contact point --tag "$tag-s45" --workers $workers \
    "${extra[@]}" 2>&1 | grep -E "Error"
done; done; done
