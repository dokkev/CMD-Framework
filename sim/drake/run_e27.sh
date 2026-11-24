#!/bin/bash
# Experiment 27: a contact-referenced joint setpoint (each hand's flexion that puts its distal-face point below the
# fingertip where O's lands at O's setpoint), point contact, otherwise experiment 21's settings.
# usage: run_e27.sh <setpoint: contact_ref | contact_mid> <tag> [--delta_deg 5.05] -- <hand...>
setpoint=$1; tag=$2; shift 2
extra=()
while [ "$1" != "--" ]; do extra+=("$1"); shift; done; shift
for hand in "$@"; do for mu in 1.3 2.0; do for r in 15 20; do
  uv run python two_pad_drake.py --hand "$hand" --mu $mu --radius_mm $r --contact point --setpoint "$setpoint" \
    --tag "$tag" --workers 6 "${extra[@]}" 2>&1 | grep -E "Gate 0|Error|AT BOUND"
done; done; done
