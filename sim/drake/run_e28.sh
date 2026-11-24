#!/bin/bash
# Experiment 28: the fingertip dial (task tipdial), point contact, open loop from O's touch angle.
#   y: a dial in the finger's plane, rho 10 / 12 mm, axle sprung 60 N/m along O's distal-pad normal, torsional spring
#      0.02 N m/rad, 45 deg sweep (placements logs/tipdial_y.json)
#   z: a disk with its axis along z, rho 15 mm, 4 mm thick, 8 / 12 mm to the side, sprung 60 N/m in z, torsional
#      spring 0.01 N m/rad, 40 deg sweep (placements logs/tipdial_z.json)
# Placements: uv run python dial_placements.py --tip y --radius_mm 10 12 --q_touch 20 35 50 65 --back_mm 10 13
#                                                --out logs/tipdial_y.json
#             uv run python dial_placements.py --tip z --radius_mm 15 --q_touch 15 25 35 --x_back_mm 10 14 --y_mm 8 12
#                                                --out logs/tipdial_z.json
# usage: run_e28.sh <y|z> <tag> <workers> [--delta_deg 0.05] -- <hand...>
mode=$1; tag=$2; workers=$3; shift 3
extra=()
while [ "$1" != "--" ]; do extra+=("$1"); shift; done; shift
if [ "$mode" = y ]; then radii="10 12"; set_=(--tip_kr 0.02 --sweep_deg 45); else radii="15"; set_=(--tip_kr 0.01 --sweep_deg 40); fi
for hand in "$@"; do for mu in 1.3 2.0; do for r in $radii; do
  uv run python two_pad_drake.py --task tipdial --tip_mode "$mode" --hand "$hand" --mu $mu --radius_mm $r \
    --grasps logs/tipdial_$mode.json --delta_deg 0 --dial_k 60 --tip_dr 1e-4 --approach_s 1 --sweep_s 3 --hold_s 1 \
    "${set_[@]}" --contact point --tag "$tag" --workers $workers "${extra[@]}" 2>&1 | grep -E "Gate 0|Error"
done; done; done
