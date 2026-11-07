#!/usr/bin/env python3
"""E10 follow-up: bootstrap intervals (2000 resamples of the 150 selected
designs) for the span-controlled partial correlations, plus feature ranges.
The intervals describe resampling of this archive cover, not a population."""
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from e10_design_drivers import partial  # noqa: E402

OUT = common.OUTPUT / "e10_design_drivers"
rows = list(csv.DictReader((OUT / "results.csv").open()))
num = lambda k: np.array([float(r[k]) if r.get(k) not in (None, "") else np.nan for r in rows])  # noqa: E731
y = num("carried")
spans = np.column_stack([num("e_proximal_span_mm"), num("e_distal_span_mm")])
rng = np.random.default_rng(20261107)
result = []
for f in ("ab_mm", "icr_travel_mm", "icr_drop_mm", "icr120_z_mm", "icr0_share", "ap_z_mm", "bc_mm"):
    a = num(f)
    boots = []
    for _ in range(2000):
        i = rng.integers(0, len(rows), len(rows))
        boots.append(partial(a[i], y[i], spans[i]))
    lo, hi = np.nanpercentile(boots, [2.5, 97.5])
    result.append({"feature": f, "partial_given_spans": partial(a, y, spans), "boot_2.5": lo, "boot_97.5": hi,
                   "min": np.nanmin(a), "max": np.nanmax(a)})
common.shared.write_csv(OUT / "bootstrap.csv", result)
for r in result:
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
