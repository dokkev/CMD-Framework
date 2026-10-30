#!/bin/bash
# Run the accommodation study's configs (make_configs.py).
set -e
cd "$(dirname "$0")/../../.."
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D25x5_flex120.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D25x5_flex120.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D25x5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D25x5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D25x5_flex180.yaml
