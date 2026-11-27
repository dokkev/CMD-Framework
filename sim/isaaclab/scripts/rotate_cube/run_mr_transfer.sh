#!/usr/bin/env bash
# Zero-shot transfer of O's policies to the MCP-referenced hands (docs/experimental/pr_mr.md, "Pre-registration: RL
# transfer to MR"). One eval process at a time; skips outputs that exist; stops when $R/STOP exists.
# - walk (attempt 9's protocol): scripts/finger_relay/eval_walk.py, both 7b' checkpoints in one process,
#   pad mu 1.3 / 2.0, eval seeds 94000 / 95000, 1024 envs -> $R/walk/<hand>_mu<mu>_seed<seed>.json
# - cube (attempt 1/1b's protocol): held-out grasps per hand (gen_grasps.py, seed 76000, dome, pad13_condim4_v1) in
#   logs/rotate_cube/heldout_transfer_v1 (O, R, U: the existing caches), then eval_policy.py with the v2c seg2
#   model_1497 and attempt 1's eval overrides -> $R/cube/eval_<hand>_mu<mu>_seed<seed>.json
# Usage: HANDS="O R U MR MRM" PARTS="walk cube" [TRANSFER_DIR=logs/...] [CUBE_CKPT=...] scripts/rotate_cube/run_mr_transfer.sh
cd "$(dirname "$0")/../.."
R=${TRANSFER_DIR:-logs/pr_mr_transfer}
LOG=$R/driver.log
mkdir -p $R/walk $R/cube
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
unset DISPLAY WAYLAND_DISPLAY
log() { echo "$(date '+%F %T') $*" >> $LOG; }
run() { local out=$1; shift; [ -f $R/STOP ] && { log "STOP"; exit 0; }; log "START $out"; "$@" > $out.out 2>&1; log "EXIT $out rc=$?"; }
C20=$PWD/logs/rsl_rl/code_finger_walk/2026-11-26_03-30-35_finger_walk_O_s20_walk_seg3/model_597.pt
C21=$PWD/logs/rsl_rl/code_finger_walk/2026-11-26_05-23-33_finger_walk_O_s21_walks21_seg3/model_597.pt
CKPT=${CUBE_CKPT:-$PWD/logs/rsl_rl/code_rotate_cube_omnireset_v2/2026-11-25_07-41-53_rotate_cube_O_OmniResetV2_s20_v2c_seg2/model_1497.pt}
HELD=$PWD/logs/rotate_cube/heldout_transfer_v1
EVAL_OVR="physics=newton_mjwarp env.observations.policy.enable_corruption=false env.terminations.object_off_grasp.params.max_distance=0.07 env.terminations.object_tilted.params.min_dot=0.0"
HANDS=${HANDS:-"O R U MR MRM MRbz MRMbz"}
PARTS=${PARTS:-"walk cube"}
log "DRIVER_START hands: $HANDS parts: $PARTS"
for H in $HANDS; do
  for P in $PARTS; do
    if [ $P = walk ]; then
      for MU in 1.3 2.0; do for S in 94000 95000; do
        OUT=$R/walk/${H}_mu${MU}_seed$S
        [ -f $OUT.json ] || CODE_CONTACT_PAD_MU=$MU run $OUT uv run --extra rsl-rl python scripts/finger_relay/eval_walk.py \
          --task CoDe-Finger-Walk-$H-Eval --checkpoint $C20 --checkpoint $C21 --num_envs 1024 --eval_seed $S \
          --output $OUT.json physics=newton_mjwarp
      done; done
    else
      [ -f $HELD/$H.pt ] || CODE_ROTATE_CUBE_CROWN_LAYOUT=dome CODE_ROTATE_CUBE_CONTACT=pad13_condim4_v1 run $R/cube/grasps_$H \
        uv run python scripts/rotate_cube/gen_grasps.py --hand $H --out_dir $HELD --target 1024 --num_envs 4096 \
        --seed_start 76000 physics=newton_mjwarp
      [ -f $HELD/$H.pt ] || { log "no grasps for $H"; continue; }
      for MU in 1.3 2.0; do for S in 94000 95000; do
        OUT=$R/cube/eval_${H}_mu${MU}_seed$S
        [ -f $OUT.json ] || CODE_CONTACT_PAD_MU=$MU CODE_ROTATE_CUBE_GRASPS=$HELD run $OUT \
          uv run --extra rsl-rl python scripts/rotate_cube/eval_policy.py --task CoDe-Rotate-Cube-$H-OmniResetV2-Eval \
          --checkpoint $CKPT --num_envs 1024 --eval_seed $S --contact_metrics --output $OUT.json $EVAL_OVR
      done; done
    fi
  done
done
log "DRIVER_DONE"
