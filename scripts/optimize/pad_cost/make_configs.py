#!/usr/bin/env python3
"""Write the pad-cost study's config files: how long a finger must be to carry
given pads up to a given flexion, the linkage against the serial 2R.

    python scripts/optimize/pad_cost/make_configs.py
    bash scripts/optimize/pad_cost/run_all.sh

Each config extends the linkage goal (scripts/optimize/linkage/config.yaml):
the pads fixed at [L, L] (each must be usable that long, pad_usable_mm), the
least flexion required, the finger free over FINGER_RANGE_MM, and the one
objective the shortest finger |OC|. Within a mechanism the configs run from
the hardest (largest pads, most flexion) down, each warm-started from the one
before it. run_all.sh lists them in that order.
"""

from pathlib import Path

PADS_MM = ((40, 45), (30, 35), (20, 25))
"""(proximal, distal) pad lengths, the largest first."""
FLEXIONS_DEG = (180, 150, 120, 90)
MECHANISMS = ("linkage", "revolute")
FINGER_RANGE_MM = (60, 200)
GENERATIONS = 400
SEEDS = (0, 1)

HERE = Path(__file__).resolve().parent


def name(mechanism: str, pads: tuple[int, int], flexion: int) -> str:
    return f"{mechanism}_p{pads[0]}_d{pads[1]}_flex{flexion}"


def config(mechanism: str, pads: tuple[int, int], flexion: int, warm: str | None) -> str:
    low, high = FINGER_RANGE_MM
    lines = [
        f"# Pad cost: {mechanism}, pads {pads[0]} + {pads[1]} mm, {flexion} deg required (make_configs.py).",
        f"mechanism: {mechanism}",
        "base: ../linkage/config.yaml",
        "goal:",
        "  motion:",
        f"    dip_flexion_deg: {flexion}",
        "  pads:",
        "    proximal:",
        f"      length_mm: [{pads[0]}, {pads[0]}]",
        "    distal:",
        f"      length_mm: [{pads[1]}, {pads[1]}]",
        "  link:",
        f"    finger_length_mm: [{low}, {high}]",
        "solver:",
        "  objectives:",
        "    - finger_length",
        "  pareto_front:",
        f"    finger_length: {high}",
        "  optimizer:",
        f"    generations: {GENERATIONS}",
        "    seeds:",
        *[f"      - {seed}" for seed in SEEDS],
    ]
    if warm:
        lines += ["    warm_start:", f"      - {warm}"]
    return "\n".join(lines) + "\n"


def main() -> None:
    order = []
    for mechanism in MECHANISMS:
        previous = None
        for pads in PADS_MM:
            for flexion in FLEXIONS_DEG:
                stem = name(mechanism, pads, flexion)
                warm = f"output/{mechanism}/{previous}/portfolio_001/seeds.csv" if previous else None
                (HERE / f"{stem}.yaml").write_text(config(mechanism, pads, flexion, warm), encoding="utf-8")
                order.append(stem)
                previous = stem
    runner = ["#!/bin/bash", "# Run the pad-cost configs in order (make_configs.py); each warm-starts from the one before.",
              "set -e", 'cd "$(dirname "$0")/../../.."']
    runner += [f"python scripts/optimize/run.py scripts/optimize/pad_cost/{stem}.yaml" for stem in order]
    (HERE / "run_all.sh").write_text("\n".join(runner) + "\n", encoding="utf-8")
    print(f"wrote {len(order)} configs and run_all.sh to {HERE}")


if __name__ == "__main__":
    main()
