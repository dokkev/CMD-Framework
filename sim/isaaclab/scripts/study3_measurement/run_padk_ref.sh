#!/usr/bin/env bash
# Study 3, seventeenth experiment (separability.md): O with the contact stiffness of a 5, 7 or 9 mm pad
# (ke = 2.5e3 * 3 / h), condition A, tags padk-ref5/7/9. Run from sim/isaaclab.
set -euo pipefail
for h in 5 7 9; do
  ke=$(python3 -c "print(2.5e3*3/$h)")
  for mu in 1.3 2.0; do
    for r in 15 20; do
      uv run --quiet python scripts/study3_measurement/two_pad_wrench.py --hand O --mu "$mu" --condition A \
        --radius_mm "$r" --impratio 1000 --contact_ke "$ke" --tag "padk-ref$h"
    done
  done
done
