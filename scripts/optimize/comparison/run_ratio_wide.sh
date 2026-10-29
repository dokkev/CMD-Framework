#!/bin/bash
# Run mechanism comparison configs (make_configs.py).
set -e
cd "$(dirname "$0")/../../.."
python scripts/optimize/run.py scripts/optimize/comparison/revolute_ratio_wide_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/revolute_ratio_wide_h5_flex120.yaml
python scripts/optimize/run.py scripts/optimize/comparison/revolute_ratio_wide_h5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/comparison/revolute_parallel_ratio_wide_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/revolute_parallel_ratio_wide_h5_flex120.yaml
python scripts/optimize/run.py scripts/optimize/comparison/revolute_parallel_ratio_wide_h5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/comparison/revolute_idealized_ratio_wide_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/revolute_idealized_ratio_wide_h5_flex120.yaml
python scripts/optimize/run.py scripts/optimize/comparison/revolute_idealized_ratio_wide_h5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_ratio_wide_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_ratio_wide_h5_flex120.yaml
python scripts/optimize/run.py scripts/optimize/comparison/rcj_ratio_wide_h5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/comparison/linkage_ratio_wide_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/comparison/linkage_ratio_wide_h5_flex120.yaml
python scripts/optimize/run.py scripts/optimize/comparison/linkage_ratio_wide_h5_flex90.yaml
