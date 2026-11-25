#!/bin/bash
# The point-vs-hydro rerun (docs/experimental/drake_hydro.md): one task's hydro runs for the given hands.
# usage: run_hydro.sh <task> <tag> <delta|-> <hand...>   (delta: the noise floor's command offset; - for none)
# Hydro: experiment 21's rule (every hand O's pad foundation stiffness, 0.1 MPa / 3 mm), 1 ms; randomized tasks on
# their first 32 draws; grid tasks on their full placement sets.
task=$1; tag=$2; delta=$3; shift 3
W=6
for hand in "$@"; do
  case $task in
    23) d=${delta/-/5}; for a in 3 6; do for mu in 1.3 2.0; do for r in 15 20; do
          uv run python two_pad_drake.py --task pullin --offset_mm $a --tip_load_n 1.5 --face_load_n 1.5 --spin_damping 2.0 \
            --hand "$hand" --mu $mu --radius_mm $r --contact hydro --delta_deg $d --tag "$tag-a$a" --workers $W 2>&1 | grep -E "Gate 0|Error"
        done; done; done ;;
    29z) d=${delta/-/0}; for mu in 1.3 2.0; do
          uv run python two_pad_drake.py --task tipdial --tip_mode z --hand "$hand" --mu $mu --any_radius \
            --grasps logs/tipdial_z_e29.json --delta_deg $d --dial_k 60 --tip_dr 1e-4 --approach_s 1 --sweep_s 3 \
            --hold_s 1 --sweep_deg 40 --contact hydro --only 32 --tag "$tag" --workers $W 2>&1 | grep -E "Gate 0|Error"
        done ;;
    38) d=${delta/-/5}; uv run python hydro_run.py hook_pull run --hands "$hand" --tag "$tag" --delta_deg $d --workers $W ;;
    39) d=${delta/-/0}; uv run python hydro_run.py twist_grasp run --hands "$hand" --tag "$tag" --delta_deg $d --workers $W ;;
    34) d=${delta/-/0}; uv run python hydro_run.py pad_shape run --hands "$hand" --tag "$tag" --delta_deg $d --workers $W ;;
    30b) d=${delta/-/0}; for mu in 1.3 2.0; do
          uv run python hydro_run.py coin_drag run --hand "$hand" --mu $mu --draws logs/coin_e30b.json --only $(seq 0 31) \
            --time_step 1e-3 --delta_deg $d --tag "$tag" --workers $W
        done ;;
    35) d=${delta/-/0}; uv run python hydro_run.py box_pivot run --hands "$hand" --draws logs/pivot_e35.json --only 32 \
          --config '{"time_step": 0.001}' --delta_deg $d --tag "$tag" --workers $W ;;
    40) d=${delta/-/0}; uv run python hydro_run.py slider_rail run --hands "$hand" --draws logs/slider_e40.json --only 32 \
          --config '{"time_step": 0.001}' --delta_deg $d --tag "$tag" --workers $W ;;
    41) d=${delta/-/0}; uv run python hydro_run.py ball_roll run --hands "$hand" --draws logs/ball_e41.json --only 32 \
          --config '{"time_step": 0.001}' --delta_deg $d --tag "$tag" --workers $W ;;
    42) d=${delta/-/0}; uv run python sphere_grasp.py run --hands "$hand" --draws logs/sphere_e42.json --only 32 \
          --contact hydro --delta_deg $d --tag "$tag" --workers $W ;;
  esac
done
