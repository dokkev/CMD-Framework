#!/usr/bin/env python3
"""Write the pad-cost study's config files: how long a finger must be to carry
given pads up to a given flexion, the linkage against the serial 2R.

    python scripts/optimize/pad_cost/make_configs.py
    bash scripts/optimize/pad_cost/run_all.sh

Each config extends the linkage goal (scripts/optimize/linkage/config.yaml):
the pads fixed at [L, L] (each must be usable that long, pad_usable_mm), the
least flexion required, the finger free over FINGER_RANGE_MM, and the one
objective the shortest finger |OC|. Within a mechanism the configs run from
the hardest (largest pads, most flexion) down, each warm-started from every
harder one's front (a design that carries larger pads, or flexes further,
carries smaller ones and flexes less: the inheritance keeps the table
monotone). run_all.sh lists them in that order; run_<mechanism>.sh one
mechanism's.
"""

from pathlib import Path

PADS_MM = ((40, 45), (30, 35), (20, 25))
"""(proximal, distal) pad lengths, the largest first."""
FLEXIONS_DEG = (180, 150, 120, 90)
MECHANISMS = ("linkage", "revolute")
FINGER_RANGE_MM = {"linkage": (60, 200), "revolute": (60, 300)}
GENERATIONS = {"linkage": 400, "revolute": 800}
SEEDS = {"linkage": (0, 1), "revolute": (0, 1, 2)}

HERE = Path(__file__).resolve().parent


def name(mechanism: str, pads: tuple[int, int], flexion: int) -> str:
    return f"{mechanism}_p{pads[0]}_d{pads[1]}_flex{flexion}"


def harder(pads: tuple[int, int], flexion: int) -> list[tuple[tuple[int, int], int]]:
    """The conditions at least as hard (pads at least as large, flexion at least as much), but this one."""

    return [(p, f) for p in PADS_MM for f in FLEXIONS_DEG
            if p[0] >= pads[0] and p[1] >= pads[1] and f >= flexion and (p, f) != (pads, flexion)]


def config(mechanism: str, pads: tuple[int, int], flexion: int, warm: list[str]) -> str:
    low, high = FINGER_RANGE_MM[mechanism]
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
        f"    generations: {GENERATIONS[mechanism]}",
        "    seeds:",
        *[f"      - {seed}" for seed in SEEDS[mechanism]],
    ]
    if warm:
        lines += ["    warm_start:", *[f"      - {path}" for path in warm]]
    return "\n".join(lines) + "\n"


def main() -> None:
    order: dict[str, list[str]] = {}
    for mechanism in MECHANISMS:
        for pads in PADS_MM:
            for flexion in FLEXIONS_DEG:
                stem = name(mechanism, pads, flexion)
                # Every harder condition's fronts, the earlier runs' too.
                warm = [f"output/{mechanism}/{name(mechanism, p, f)}/portfolio_*/seeds.csv"
                        for p, f in harder(pads, flexion)]
                (HERE / f"{stem}.yaml").write_text(config(mechanism, pads, flexion, warm), encoding="utf-8")
                order.setdefault(mechanism, []).append(stem)
    header = ["#!/bin/bash", "# Run pad-cost configs, hardest first (make_configs.py).", "set -e",
              'cd "$(dirname "$0")/../../.."']
    for label, stems in (("all", [s for m in MECHANISMS for s in order[m]]), *order.items()):
        lines = header + [f"python scripts/optimize/run.py scripts/optimize/pad_cost/{stem}.yaml" for stem in stems]
        (HERE / f"run_{label}.sh").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {sum(map(len, order.values()))} configs and run_all.sh, "
          + ", ".join(f"run_{m}.sh" for m in order) + f" to {HERE}")


if __name__ == "__main__":
    main()
