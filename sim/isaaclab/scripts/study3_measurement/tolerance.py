# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3 measurement (problem_formulation A.6.4): a design tolerance on Delta_contact.

    python3 scripts/study3_measurement/tolerance.py

For every Study 3 setting, the error E against Delta_contact over the redesigns that hold the cylinder, fitted by an
increasing (isotonic) regression. The tolerance at a level L is the largest Delta_contact whose fitted error stays at
or below L; its 95% interval resamples hands. Levels: 20% and 50%. Writes
``logs/study3/two_pad/tolerance.json``.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dose_analysis as da  # noqa: E402

SETTINGS = {"friction": ("dose_A_all.json", "E_rel_median"), "stiffness": ("comp.json", "E_median"),
            "grid": ("dose_A-grid_expanded.json", "E_rel_median"), "soft": ("padk_matched.json", "E_median"),
            "torque": ("dose_C_all.json", "E_rel_median")}
LEVELS = (0.2, 0.5)


def isotonic(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Pool-adjacent-violators fit of an increasing function; returns x sorted and the fitted y."""
    order = np.argsort(x)
    xs, ys = x[order], y[order].astype(float)
    blocks = [[v, 1.0] for v in ys]
    out = []
    for v, w in blocks:
        out.append([v, w])
        while len(out) > 1 and out[-2][0] > out[-1][0]:
            v2, w2 = out.pop()
            v1, w1 = out.pop()
            out.append([(v1 * w1 + v2 * w2) / (w1 + w2), w1 + w2])
    fit = np.concatenate([np.full(int(w), v) for v, w in out])
    return xs, fit


def tolerance(x: np.ndarray, y: np.ndarray, level: float) -> float:
    xs, fit = isotonic(x, y)
    ok = xs[fit <= level]
    return float(ok.max()) if ok.size else 0.0


def main() -> int:
    feats = json.loads((da.ROOT / "invariants.json").read_text())["features"]
    rng = np.random.default_rng(0)
    out = {}
    for name, (file, key) in SETTINGS.items():
        recs = json.loads((da.ROOT / file).read_text())["hands"]
        hs = [h for h in recs if h in feats and recs[h].get(key) is not None]
        x = np.array([feats[h]["contact"] for h in hs])
        y = np.array([recs[h][key] for h in hs])
        res = {"n": len(hs), "dc_range_mm": [float(x.min()), float(x.max())]}
        for level in LEVELS:
            boots = []
            for _ in range(5000):
                k = rng.integers(0, len(hs), len(hs))
                boots.append(tolerance(x[k], y[k], level))
            res[f"tolerance_{int(level * 100)}"] = {"mm": tolerance(x, y, level),
                                                    "ci95": [float(np.percentile(boots, 2.5)),
                                                             float(np.percentile(boots, 97.5))]}
        out[name] = res
        print(f"{name:9s} n {len(hs):2d} Dc {res['dc_range_mm'][0]:.2f}-{res['dc_range_mm'][1]:.2f} | " + " | ".join(
            f"E<={int(lv * 100)}%: Dc <= {res[f'tolerance_{int(lv * 100)}']['mm']:.2f} mm "
            f"[{res[f'tolerance_{int(lv * 100)}']['ci95'][0]:.2f}, {res[f'tolerance_{int(lv * 100)}']['ci95'][1]:.2f}]"
            for lv in LEVELS))
    (da.ROOT / "tolerance.json").write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
