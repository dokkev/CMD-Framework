#!/bin/bash
# Run pad trade-off configs, hardest first (make_configs.py).
set -e
cd "$(dirname "$0")/../../.."
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h8_flex180.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h8_flex165.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h8_flex150.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h8_flex135.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h8_flex120.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h8_flex105.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h8_flex90.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h5_flex180.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h5_flex165.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h5_flex150.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h5_flex135.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h5_flex120.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h5_flex105.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h5_flex90.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h3_flex180.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h3_flex165.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h3_flex150.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h3_flex135.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h3_flex120.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h3_flex105.yaml
python scripts/optimize/run.py scripts/optimize/pad_tradeoff/rcj_h3_flex90.yaml
