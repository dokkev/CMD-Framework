#!/bin/bash
# Run the accommodation study's configs (make_configs.py).
set -e
cd "$(dirname "$0")/../../.."
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P15x3_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P15x5_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P15x10_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x3_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x10_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P35x3_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P35x5_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P35x10_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D15x3_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D15x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D15x10_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D25x3_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D25x10_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D35x3_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D35x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/revolute_P25x5_D35x10_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P15x3_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P15x5_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P15x10_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x3_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x10_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P35x3_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P35x5_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P35x10_D25x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D15x3_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D15x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D15x10_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D25x3_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D25x10_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D35x3_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D35x5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/linkage_P25x5_D35x10_flex90.yaml
