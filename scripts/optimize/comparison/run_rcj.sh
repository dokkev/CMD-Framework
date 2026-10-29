#!/bin/bash
# Run mechanism comparison configs (make_configs.py).
set -e
cd "$(dirname "$0")/../../.."
python scripts/optimize/run.py scripts/optimize/comparison/rcj_h10_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_h10_flex120.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_h10_flex90.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_h5_flex120.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_h5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_h3_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_h3_flex120.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_h3_flex90.yaml
