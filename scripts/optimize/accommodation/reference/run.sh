#!/bin/bash
# Search both reference layouts, then pick and record them (pick.py).
set -e
cd "$(dirname "$0")/../../../.."
python scripts/optimize/run.py scripts/optimize/accommodation/reference/revolute.yaml
python scripts/optimize/run.py scripts/optimize/accommodation/reference/linkage.yaml
python scripts/optimize/accommodation/reference/pick.py
