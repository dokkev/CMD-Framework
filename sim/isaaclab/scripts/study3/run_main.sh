#!/usr/bin/env bash
# Study 3A main experiment (docs/decisions.md 2026-11-21, pre-registered): seeds 0-9 per surface on the -Main
# tasks with the budget the pilot rule fixed, then every policy on both surfaces (MuJoCo-Warp, 256 episodes,
# eval seed 2000, fixed goal sequence). Usage: scripts/study3/run_main.sh <iterations> [first_seed] [last_seed]
set -euo pipefail
cd "$(dirname "$0")/../.."
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
ITER=${1:?iterations}
FIRST=${2:-0}
LAST=${3:-9}
LOG=logs/study3_main
mkdir -p $LOG
for s in $(seq $FIRST $LAST); do
  for v in flat crowned; do
    uv run --extra rsl-rl python scripts/study3/train.py --protocol main --variant $v --seed $s --iterations $ITER --label main \
      2>&1 | grep '^{' | cut -c1-160 >> $LOG/train.log || echo "{\"failed\": \"${v}_s${s}\"}" >> $LOG/train.log
  done
done
for s in $(seq $FIRST $LAST); do
  for run in logs/rsl_rl/study3_code_reorient/*_{flat,crowned}_s${s}_main; do
    for v in flat crowned; do
      uv run --extra rsl-rl python scripts/study3/eval_policy.py --run $run --test_variant $v --episodes 256 \
        --eval_seed 2000 --out_root $LOG/eval physics=newton_mjwarp 2>&1 | grep '^{"run"' | cut -c1-200 >> $LOG/eval.log
    done
  done
done
