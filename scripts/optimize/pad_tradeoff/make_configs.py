#!/usr/bin/env python3
"""Write the pad trade-off study's config files: at one finger length, how
much effective pad each mechanism keeps up to a required flexion, per pad
height, the linkage against the serial 2R (packaged and idealized) and the RCJ.

    python scripts/optimize/pad_tradeoff/make_configs.py [mechanism ...]
    bash scripts/optimize/pad_tradeoff/run_all.sh

Mechanisms named: only theirs written (configs and run_<mechanism>.sh).

Each config extends the linkage goal (scripts/optimize/linkage/config.yaml):
|OC| at FINGER_RANGE_MM and |OA| at PROXIMAL_RANGE_MM, the proximal pad free
from 0 and the distal pad from DISTAL_PAD_MIN_MM (a fingertip pad to pinch
with), k_max at most TORQUE_RATIO_MAX (the RCJ's), the least flexion required
and the pads' height, and the one objective
pad_coverage (each pad's face usable all over [0, flexion]). Within a
mechanism the configs run from the hardest (tallest pads, most flexion)
down, each warm-started from every harder one's front (a design that
flexes further, or carries taller pads, keeps as much pad at less: the
inheritance keeps the curves monotone). run_all.sh lists them in that
order; run_<mechanism>.sh one mechanism's.
"""

import sys
from pathlib import Path

HEIGHTS_MM = (8, 5, 3)
"""Pad heights, the tallest first."""
FLEXIONS_DEG = (180, 165, 150, 135, 120, 105, 90)
MECHANISMS = ("linkage", "revolute", "rcj", "revolute_idealized")
FINGER_RANGE_MM = (99.5, 100)
PROXIMAL_RANGE_MM = (40, 60)
"""|OA|: the proximal segment 40-60 % of the finger, as a human finger's
(no 70 + 30 mm finger that buys pad with odd proportions)."""
DISTAL_PAD_MIN_MM = 20
"""The fingertip must carry a pad to pinch with: without it the 2R keeps its
flexion by dropping the distal pad, which no pad total should reward."""
TORQUE_RATIO_MAX = 2.0
"""k_max at most the RCJ's (2; the 2R's is 1): no pad bought with servo torque."""
GENERATIONS = {"linkage": 500, "revolute": 300, "rcj": 300, "revolute_idealized": 300}
SEEDS = (0, 1, 2)

HERE = Path(__file__).resolve().parent


def name(mechanism: str, height: int, flexion: int) -> str:
    return f"{mechanism}_h{height}_flex{flexion}"


def harder(height: int, flexion: int) -> list[tuple[int, int]]:
    """The conditions at least as hard (pads at least as tall, flexion at least as much), but this one."""

    return [(h, f) for h in HEIGHTS_MM for f in FLEXIONS_DEG
            if h >= height and f >= flexion and (h, f) != (height, flexion)]


def config(mechanism: str, height: int, flexion: int, warm: list[str]) -> str:
    low, high = FINGER_RANGE_MM
    lines = [
        f"# Pad trade-off: {mechanism}, pads {height} mm tall, {flexion} deg required (make_configs.py).",
        f"mechanism: {mechanism}",
        "base: ../linkage/config.yaml",
        "goal:",
        "  motion:",
        f"    dip_flexion_deg: {flexion}",
        "  pads:",
        "    proximal:",
        "      length_mm: [0, null]",
        f"      height_mm: {height}",
        "    distal:",
        f"      length_mm: [{DISTAL_PAD_MIN_MM}, null]",
        f"      height_mm: {height}",
        "  hardware:",
        "    pip_servo:",
        f"      torque_ratio_max: {TORQUE_RATIO_MAX}",
        "  link:",
        f"    finger_length_mm: [{low}, {high}]",
        f"    proximal_length_mm: [{PROXIMAL_RANGE_MM[0]}, {PROXIMAL_RANGE_MM[1]}]",
        "solver:",
        "  objectives:",
        "    - pad_coverage",
        "  optimizer:",
        f"    generations: {GENERATIONS[mechanism]}",
        "    seeds:",
        *[f"      - {seed}" for seed in SEEDS],
    ]
    if warm:
        lines += ["    warm_start:", *[f"      - {path}" for path in warm]]
    return "\n".join(lines) + "\n"


def main(argv: list[str]) -> None:
    unknown = set(argv) - set(MECHANISMS)
    if unknown:
        raise SystemExit(f"unknown mechanisms {sorted(unknown)}; known: {MECHANISMS}")
    order: dict[str, list[str]] = {}
    for mechanism in argv or MECHANISMS:
        for height in HEIGHTS_MM:
            for flexion in FLEXIONS_DEG:
                stem = name(mechanism, height, flexion)
                # Every harder condition's fronts, the earlier runs' too.
                warm = [f"output/{mechanism}/{name(mechanism, h, f)}/portfolio_*/seeds.csv"
                        for h, f in harder(height, flexion)]
                (HERE / f"{stem}.yaml").write_text(config(mechanism, height, flexion, warm), encoding="utf-8")
                order.setdefault(mechanism, []).append(stem)
    header = ["#!/bin/bash", "# Run pad trade-off configs, hardest first (make_configs.py).", "set -e",
              'cd "$(dirname "$0")/../../.."']
    scripts = list(order.items()) if argv else [("all", [s for m in MECHANISMS for s in order[m]]), *order.items()]
    for label, stems in scripts:
        lines = header + [f"python scripts/optimize/run.py scripts/optimize/pad_tradeoff/{stem}.yaml"
                          for stem in stems]
        (HERE / f"run_{label}.sh").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {sum(map(len, order.values()))} configs and "
          + ", ".join(f"run_{label}.sh" for label, _ in scripts) + f" to {HERE}")


if __name__ == "__main__":
    main(sys.argv[1:])
