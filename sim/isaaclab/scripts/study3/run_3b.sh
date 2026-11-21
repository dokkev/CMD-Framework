#!/usr/bin/env bash
# Study 3B (docs/decisions.md 2026-11-21, pre-registered): every 3A-main policy on both surfaces under the two
# alternative contact models (pyramidal friction cone; impratio 1), same episodes as 3A. Usage: scripts/study3/run_3b.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
LOG=logs/study3_main
for model in "pyramidal env.sim.physics.solver_cfg.cone=pyramidal" "impratio1 env.sim.physics.solver_cfg.impratio=1.0"; do
  set -- $model
  for run in logs/rsl_rl/study3_code_reorient/*_s[0-9]_main; do
    for v in flat crowned; do
      uv run --extra rsl-rl python scripts/study3/eval_policy.py --run $run --test_variant $v --episodes 256 \
        --eval_seed 2000 --out_root $LOG/eval_cm_$1 physics=newton_mjwarp $2 2>&1 | grep '^{"run"' | cut -c1-200 | sed "s/^/$1 /" >> $LOG/eval_3b.log
    done
  done
done
