#!/usr/bin/env bash
# Attempt 9 (docs/experimental/rl_pip_only.md): transfer the two O finger-walk policies (7b' closed loop, training
# seeds 20 and 21, the checkpoints the earlier transfers used) to every hand, one eval process at a time.
# Per hand: pad mu 1.3 and 2.0 x eval seeds 94000 and 95000, 1024 envs, both checkpoints in one process (seed 20 first).
# Writes logs/finger_walk/grid/<hand>_mu<mu>_seed<seed>.json; skips files that exist; stops when grid/STOP exists.
cd "$(dirname "$0")/../.."
G=logs/finger_walk/grid
LOG=$G/driver.log
mkdir -p $G
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
unset DISPLAY WAYLAND_DISPLAY
C20=$PWD/logs/rsl_rl/code_finger_walk/2026-11-26_03-30-35_finger_walk_O_s20_walk_seg3/model_597.pt
C21=$PWD/logs/rsl_rl/code_finger_walk/2026-11-26_05-23-33_finger_walk_O_s21_walks21_seg3/model_597.pt
log() { echo "$(date '+%F %T') $*" >> $LOG; }
HANDS=${HANDS:-"O R U R5 U5 R9 U9 $(uv run python -c 'from code_isaaclab.tasks.finger_walk.grid import GRID; print(" ".join(k for k in GRID if k.startswith("g")) + " " + " ".join(k for k in GRID if k.startswith("FA")))' 2>/dev/null | tail -1)"}
log "DRIVER_START hands: $HANDS"
for H in $HANDS; do for MU in 1.3 2.0; do for S in 94000 95000; do
  [ -f $G/STOP ] && { log "STOP"; exit 0; }
  OUT=$G/${H}_mu${MU}_seed$S.json
  [ -f $OUT ] && continue
  log "START $H mu $MU seed $S"
  CODE_CONTACT_PAD_MU=$MU uv run --extra rsl-rl python scripts/finger_relay/eval_walk.py --task CoDe-Finger-Walk-$H-Eval \
    --checkpoint $C20 --checkpoint $C21 --num_envs 1024 --eval_seed $S --output $OUT physics=newton_mjwarp \
    > $G/${H}_mu${MU}_seed$S.out 2>&1
  log "EXIT $H mu $MU seed $S rc=$?"
done; done; done
log "DRIVER_DONE"
