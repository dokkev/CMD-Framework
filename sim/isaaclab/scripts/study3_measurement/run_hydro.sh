#!/usr/bin/env bash
# Study 3, sixteenth experiment (separability.md): hydroelastic compliant pads with the reference's pad mechanics
# on every hand (E = 1 MPa, stiffness E / 3 mm), condition A, tag hydro. Run from sim/isaaclab.
set -euo pipefail
for hand in O R U h3to5_hr h3to5_hu h3to9_hr h3to9_hu d1p5_minDs_s0 d2p25_minDs_s0 d2p25_maxDs_s0 d3_minDs_s0 \
    d4p5_s0 d5p5_s0 d1p5_maxDs_s0 d3_maxDs_s0 h3to5_d3_s0 h3to5_d3_minDs_s0 h3to9_d3p5_maxDs_s0 h3to9_d5_minDs_s0; do
  for mu in 1.3 2.0; do
    for r in 15 20; do
      uv run --quiet python scripts/study3_measurement/two_pad_wrench.py --hand "$hand" --mu "$mu" --condition A \
        --radius_mm "$r" --impratio 1000 --pad_modulus 1e6 --pad_reference_mm 3 --gate_angle_deg 4 \
        --hydro_buffer 1.0 --tag hydro physics=newton_mjwarp_hydro
    done
  done
done
