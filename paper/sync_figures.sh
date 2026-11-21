#!/usr/bin/env bash
# Build the paper's figures into the manuscript repo's figures/ (PNG only).
# Plots and the pole concept are drawn from design/output/study by figures/make_figures.py (conda env `cmd`);
# the designs strip is still cropped from the report figure by crop_figures.py.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "$here/.." && pwd)"
dest="${1:-$repo_root/../ContactCentricDesign-RSS2027}/figures"
mkdir -p "$dest"
python3 "$here/crop_figures.py" "$dest"
source "$(conda info --base)/etc/profile.d/conda.sh"
conda run -n cmd python "$here/figures/make_figures.py" "$dest"
