#!/bin/bash
# Experiment 29: experiment 28's fingertip dial (task tipdial) from 64 randomized start conditions per configuration,
# the same draws on every hand (common random numbers), each applied in the hand's own proximal-face frame; point
# contact, open loop from O's touch angle. Sweep, hold, axle spring (60 N/m) and damping as in experiment 28; each
# draw carries its own touch angle, radius, place and torsional spring (docs/experimental/drake_dial_random.md).
# Draws: uv run python dial_placements.py --tip y --random 64 --seed 29 --out logs/tipdial_y_e29.json
#        uv run python dial_placements.py --tip z --random 64 --seed 29 --out logs/tipdial_z_e29.json
# usage: run_e29.sh <y|z> <tag> <workers> [--delta_deg 0.05] -- <hand...>
mode=$1; tag=$2; workers=$3; shift 3
extra=()
while [ "$1" != "--" ]; do extra+=("$1"); shift; done; shift
if [ "$mode" = y ]; then sweep=45; else sweep=40; fi
for hand in "$@"; do for mu in 1.3 2.0; do
  uv run python two_pad_drake.py --task tipdial --tip_mode "$mode" --hand "$hand" --mu $mu --any_radius \
    --grasps logs/tipdial_${mode}_e29.json --delta_deg 0 --dial_k 60 --tip_dr 1e-4 --approach_s 1 --sweep_s 3 \
    --hold_s 1 --sweep_deg $sweep --contact point --tag "$tag" --workers $workers "${extra[@]}" 2>&1 | grep -E "Gate 0|Error"
done; done
