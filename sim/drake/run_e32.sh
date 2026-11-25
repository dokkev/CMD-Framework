#!/bin/bash
# Experiment 32: compensation-error sensitivity (comp_sensitivity.py). usage: [C2=1] run_e32.sh "<seeds>" <hand...>
# Families: pull F (exp 31), pull C (exp 27), disk F at +2 (exp 31) and +11 deg, disk C at +11 (and +2 with C2=1).
cd "$(dirname "$0")"
seeds=$1; shift
R="uv run python comp_sensitivity.py run --seeds $seeds --hands $* --workers 6"
SECONDS=0
$R --task pull --method F --conds force=0.05 force=0.1 force=0.2 joint=0.5 joint=1 joint=2 | grep -v "^exists"; echo "pullF ${SECONDS}s"; SECONDS=0
$R --task pull --method C --conds zero loc=0.5 loc=1 loc=2 joint=0.5 joint=1 joint=2 | grep -v "^exists"; echo "pullC ${SECONDS}s"; SECONDS=0
$R --task disk --method F --pre 2 --conds force=0.05 force=0.1 force=0.2 joint=0.5 joint=1 joint=2 | grep -v "^exists"; echo "diskF2 ${SECONDS}s"; SECONDS=0
$R --task disk --method F --pre 11 --conds zero | grep -v "^exists"
$R --task disk --method F --pre 11 --conds force=0.05 force=0.1 force=0.2 joint=0.5 joint=1 joint=2 | grep -v "^exists"; echo "diskF11 ${SECONDS}s"; SECONDS=0
$R --task disk --method C --pre 11 --conds zero loc=0.5 loc=1 loc=2 joint=0.5 joint=1 joint=2 | grep -v "^exists"; echo "diskC11 ${SECONDS}s"; SECONDS=0
if [ "${C2:-0}" = 1 ]; then
$R --task disk --method C --pre 2 --conds zero loc=0.5 loc=1 loc=2 joint=0.5 joint=1 joint=2 | grep -v "^exists"; echo "diskC2 ${SECONDS}s"; SECONDS=0
fi
date
