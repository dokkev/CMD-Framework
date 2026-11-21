#!/usr/bin/env bash
# Study 3 pilot, end to end, from sim/isaaclab/ (report/study3/study3_pilot.md, "Reproduction").
#
#   bash scripts/study3/run_pilot.sh [stage]      stage: export | validate | train | eval | analyze | all
#
# export   : the CoDe hand description the variants are built from (design/, conda env "cmd")
# validate : Gate 0, scripted rollouts of both variants on both backends (implicit servo)
# train    : PPO, both variants x seeds 0 and 1, 488 iterations x 24 steps x 128 envs (1,499,136 transitions)
# eval     : every final checkpoint on both variants and both backends, 256 matched episodes, seed 1000
# analyze  : figures and tables into report/study3/
set -euo pipefail
STAGE="${1:-all}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
EPISODES=256
EVAL_SEED=1000
ITERATIONS=488

if [[ "$STAGE" == export || "$STAGE" == all ]]; then
  (cd ../../design && python scripts/export/code_hand_sim.py --params core/morphology/code_hand/pivot8_balanced.yaml \
     --out output/sim/study3/code_hand_pivot8.json)
fi

if [[ "$STAGE" == validate || "$STAGE" == all ]]; then
  for v in flat crowned; do
    for physics in newton_mjwarp newton_vbd; do
      for policy in hold zero random abduct; do
        uv run python scripts/study3/validate_env.py --variant "$v" --policy "$policy" --amplitude 0.5 \
          --num_envs 16 --steps 300 --tag gate0_implicit "physics=$physics"
      done
    done
  done
fi

if [[ "$STAGE" == train || "$STAGE" == all ]]; then
  for seed in 0 1; do
    for v in flat crowned; do
      uv run --extra rsl-rl python scripts/study3/train.py --variant "$v" --seed "$seed" --iterations "$ITERATIONS"
    done
  done
fi

if [[ "$STAGE" == eval || "$STAGE" == all ]]; then
  for run in logs/rsl_rl/study3_code_reorient/*_s[0-9]; do
    [[ -f "$run/study3_manifest.json" ]] || continue
    for v in flat crowned; do
      for physics in newton_mjwarp newton_vbd; do
        uv run --extra rsl-rl python scripts/study3/eval_policy.py --run "$run" --test_variant "$v" \
          --episodes "$EPISODES" --eval_seed "$EVAL_SEED" "physics=$physics"
      done
    done
  done
fi

if [[ "$STAGE" == analyze || "$STAGE" == all ]]; then
  uv run python scripts/study3/analyze.py
fi
