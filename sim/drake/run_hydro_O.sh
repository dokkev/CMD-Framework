#!/bin/bash
# O-only hydro calibration for every task (and its noise floor), before any redesigned hand runs under hydro
cd "$(dirname "$0")"
for t in 23 29z 38 39 34 30b 35 40 41 42; do
  tag=e${t}h
  case $t in 23) nd=5.05;; 38) nd=5.05;; *) nd=0.05;; esac
  echo "== $t $(date +%T)"
  ./run_hydro.sh $t $tag - O
  ./run_hydro.sh $t ${tag}n $nd O
done
echo "all done $(date +%T)"
