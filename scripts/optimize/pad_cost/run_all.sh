#!/bin/bash
# Run the pad-cost configs in order (make_configs.py); each warm-starts from the one before.
set -e
cd "$(dirname "$0")/../../.."
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p40_d45_flex180.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p40_d45_flex150.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p40_d45_flex120.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p40_d45_flex90.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p30_d35_flex180.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p30_d35_flex150.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p30_d35_flex120.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p30_d35_flex90.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p20_d25_flex180.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p20_d25_flex150.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p20_d25_flex120.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/linkage_p20_d25_flex90.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p40_d45_flex180.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p40_d45_flex150.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p40_d45_flex120.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p40_d45_flex90.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p30_d35_flex180.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p30_d35_flex150.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p30_d35_flex120.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p30_d35_flex90.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p20_d25_flex180.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p20_d25_flex150.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p20_d25_flex120.yaml
python scripts/optimize/run.py scripts/optimize/pad_cost/revolute_p20_d25_flex90.yaml
