# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Tables for the MR transfer (``docs/experimental/pr_mr.md``, "RL transfer to MR") from ``logs/pr_mr_transfer``.

Per hand H (both eval seeds averaged; walk per training seed and pooled):
- ratio = score(H, mu 1.3) / score(O, mu 1.3), O re-measured by the same driver;
- E = |Delta(H) - Delta(O)| / |Delta(O)|, Delta = score(mu 2.0) - score(mu 1.3);
- eval noise: the eval-seed difference of the ratio, and of Delta over |Delta(O)| (attempts 1b and 9).
Cube scores: rotation mean (attempt 1c's primary), full turns at upright <= 30 deg (attempt 1's primary), goals/episode.
Walk score: walked_mm_mean (attempt 9). Missing files are skipped.

    uv run python scripts/rotate_cube/mr_transfer_summary.py
"""

from __future__ import annotations

import os
import json
from pathlib import Path

R = Path(__file__).resolve().parents[2] / os.environ.get("TRANSFER_DIR", "logs/pr_mr_transfer")
HANDS = tuple(os.environ.get("SUMMARY_HANDS", "O R U MR MRM MRbz MRMbz").split())
SEEDS = (94000, 95000)


def _load(p: Path):
    return json.loads(p.read_text()) if p.exists() else None


def _metric(d: dict, *names):
    for n in names:
        if n in d:
            return float(d[n])
    raise KeyError(names)


def table(scores: dict, label: str, drops: dict | None = None) -> None:
    """scores[hand][mu] = [seed 94000 value, seed 95000 value]."""
    o = scores.get("O")
    if not o or len(o.get(1.3, [])) < 2:
        print(f"{label}: no O")
        return
    o13, o20 = sum(o[1.3]) / 2, sum(o.get(2.0, [float('nan')] * 2)) / 2
    d_o = o20 - o13
    d_o_s = [o[2.0][i] - o[1.3][i] for i in range(2)] if len(o.get(2.0, [])) == 2 else None
    print(f"\n### {label}  (O: mu1.3 {o[1.3][0]:.4g} / {o[1.3][1]:.4g}, mu2.0 {o.get(2.0)}, Delta(O) {d_o:.4g})")
    print("| hand | mu 1.3 (2 seeds) | mu 2.0 | Delta | ratio | ratio noise | E | E noise | drop |")
    print("|---|---|---|---|---|---|---|---|---|")
    for h in HANDS:
        s = scores.get(h)
        if not s or len(s.get(1.3, [])) < 2:
            continue
        m13 = sum(s[1.3]) / 2
        ratio = m13 / o13
        r_noise = abs(s[1.3][0] / o[1.3][0] - s[1.3][1] / o[1.3][1])
        row = f"| {h} | {s[1.3][0]:.4g} / {s[1.3][1]:.4g} | "
        if len(s.get(2.0, [])) == 2 and d_o_s:
            d = sum(s[2.0]) / 2 - m13
            ds = [s[2.0][i] - s[1.3][i] for i in range(2)]
            e = abs(d - d_o) / abs(d_o)
            e_noise = abs(ds[0] - ds[1]) / abs(d_o)
            row += f"{s[2.0][0]:.4g} / {s[2.0][1]:.4g} | {d:+.4g} | {ratio:.3f} | {r_noise:.3f} | {e:.3f} | {e_noise:.3f} |"
        else:
            row += f"– | – | {ratio:.3f} | {r_noise:.3f} | – | – |"
        dr = drops.get(h) if drops else None
        row += f" {dr * 100:.1f}% |" if dr is not None else " – |"
        print(row)
    if d_o_s:
        print(f"O's Delta seed difference {abs(d_o_s[0] - d_o_s[1]):.4g} (|Delta(O)| must exceed twice it)")


def main() -> None:
    # walk: one JSON per (hand, mu, seed) with a list of the two checkpoints (training seeds 20, 21)
    for ti, tname in enumerate(("training seed 20", "training seed 21")):
        sc, dr = {}, {}
        for h in HANDS:
            for mu in (1.3, 2.0):
                for s in SEEDS:
                    d = _load(R / "walk" / f"{h}_mu{mu}_seed{s}.json")
                    if d is None:
                        continue
                    sc.setdefault(h, {}).setdefault(mu, []).append(d[ti]["walked_mm_mean"])
                    if mu == 1.3:
                        dr.setdefault(h, []).append(d[ti]["dropped_rate"])
        table(sc, f"walk 7b', {tname}, walked mm/episode", {h: sum(v) / len(v) for h, v in dr.items()})
    sc = {}
    for h in HANDS:
        for mu in (1.3, 2.0):
            for s in SEEDS:
                d = _load(R / "walk" / f"{h}_mu{mu}_seed{s}.json")
                if d is not None:
                    sc.setdefault(h, {}).setdefault(mu, []).append((d[0]["walked_mm_mean"] + d[1]["walked_mm_mean"]) / 2)
    table(sc, "walk 7b', pooled (mean of the two training seeds)")
    # cube
    names = {
        "rotation mean [deg]": ("signed_rotation_deg_mean",),
        "full turn upright<=30": ("full_turn_within_40mm_and_upright30deg_rate",),
        "goals/episode": ("goals_per_episode_mean",),
        "full turn + clean time-out": ("full_turn_and_clean_timeout_rate",),
    }
    raw = {h: {mu: [_load(R / "cube" / f"eval_{h}_mu{mu}_seed{s}.json") for s in SEEDS] for mu in (1.3, 2.0)}
           for h in HANDS}
    drops = {h: sum(d["off_grasp_rate"] + d["tilted_rate"] for d in raw[h][1.3]) / 2
             for h in HANDS if all(d is not None for d in raw[h][1.3])}
    for label, keys in names.items():
        sc = {}
        for h in HANDS:
            for mu in (1.3, 2.0):
                v = [d for d in raw[h][mu] if d is not None]
                if len(v) == 2:
                    try:
                        sc.setdefault(h, {})[mu] = [_metric(d, *keys) for d in v]
                    except KeyError:
                        pass
        table(sc, f"cube, {label}", drops)


if __name__ == "__main__":
    main()
