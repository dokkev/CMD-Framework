#!/bin/bash
# Budget check: each mechanism at h5 and h10 / 150 deg, 3 seeds x 1500 generations.
set -e
cd "$(dirname "$0")/../../../.."
python scripts/optimize/run.py scripts/optimize/comparison/budget/revolute_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/budget/revolute_parallel_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/budget/revolute_idealized_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/budget/rcj_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/budget/linkage_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/budget/revolute_h10_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/budget/revolute_parallel_h10_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/budget/revolute_idealized_h10_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/budget/rcj_h10_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/budget/linkage_h10_flex150.yaml
