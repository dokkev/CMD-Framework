#!/bin/bash
# Run mechanism comparison configs (make_configs.py).
set -e
cd "$(dirname "$0")/../../.."
python scripts/optimize/run.py scripts/optimize/comparison/linkage_onlinks_h10_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/linkage_overhang0_h10_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_overhang0_h10_flex150.yaml
