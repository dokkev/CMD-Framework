#!/usr/bin/env python3
"""Write the interface coverage study's config files: at one extended finger
length, what share of it each mechanism can give to contact interfaces
usable all through a required flexion, per interface thickness.

    python scripts/optimize/interface_coverage/make_configs.py [mechanism ...]
    bash scripts/optimize/interface_coverage/run_all.sh

Mechanisms named: only theirs written (configs and run_<mechanism>.sh).

Each config extends the linkage goal (scripts/optimize/linkage/config.yaml):
|OC| at FINGER_RANGE_MM and |OA| at PROXIMAL_RANGE_MM, each pad at least
PAD_MIN_MM (no segment gives its interface up), the least flexion required
and the pads' thickness, the 2R's servo tilt searched over TILT_RANGE_DEG,
and the one
objective pad_coverage ((P + D) / |OC|, each pad's face usable all over
[0, flexion]). k and the finger's thickness are each mechanism's own price,
reported and not limited. Within a mechanism the configs run from the
hardest (thickest pads, most flexion) down, each warm-started from every
harder one's front (the inheritance keeps the results monotone).
"""

import sys
from pathlib import Path

THICKNESSES_MM = (10, 5, 3)
"""Interface thicknesses, the thickest first."""
FLEXIONS_DEG = (150, 120, 90)
MECHANISMS = ("linkage", "revolute", "rcj", "revolute_idealized", "revolute_parallel")
FINGER_RANGE_MM = (99.5, 100)
PROXIMAL_RANGE_MM = (40, 60)
"""|OA|: the proximal segment 40-60 % of the finger, as a human finger's."""
PAD_MIN_MM = 20
"""Each segment must carry an interface: without it a fixed pivot keeps its
flexion by dropping one pad (the distal for a pinch, the proximal for a grasp)."""
TILT_RANGE_DEG = (0, 90)
"""The packaged 2R's distal servo, from laid back along -X to standing."""
GENERATIONS = {"linkage": 500, "revolute": 400, "rcj": 300, "revolute_idealized": 300, "revolute_parallel": 500}
SEEDS = (0, 1, 2)

HERE = Path(__file__).resolve().parent
STUDY = "interface_coverage"


def name(mechanism: str, thickness: int, flexion: int) -> str:
    return f"{mechanism}_cov_h{thickness}_flex{flexion}"


def harder(thickness: int, flexion: int) -> list[tuple[int, int]]:
    """The conditions at least as hard (as thick, as much flexion), but this one."""

    return [(h, f) for h in THICKNESSES_MM for f in FLEXIONS_DEG
            if h >= thickness and f >= flexion and (h, f) != (thickness, flexion)]


def config(mechanism: str, thickness: int, flexion: int, warm: list[str]) -> str:
    low, high = FINGER_RANGE_MM
    lines = [
        f"# Interface coverage: {mechanism}, interfaces {thickness} mm thick, {flexion} deg required (make_configs.py).",
        f"mechanism: {mechanism}",
        "base: ../linkage/config.yaml",
        "goal:",
        "  motion:",
        f"    dip_flexion_deg: {flexion}",
        "  pads:",
        "    proximal:",
        f"      length_mm: [{PAD_MIN_MM}, null]",
        f"      height_mm: {thickness}",
        "    distal:",
        f"      length_mm: [{PAD_MIN_MM}, null]",
        f"      height_mm: {thickness}",
        "  link:",
        f"    finger_length_mm: [{low}, {high}]",
        f"    proximal_length_mm: [{PROXIMAL_RANGE_MM[0]}, {PROXIMAL_RANGE_MM[1]}]",
        *(["  hardware:", "    pip_servo:", f"      tilt_angle_deg: [{TILT_RANGE_DEG[0]}, {TILT_RANGE_DEG[1]}]"]
          if mechanism == "revolute" else []),
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
        for thickness in THICKNESSES_MM:
            for flexion in FLEXIONS_DEG:
                stem = name(mechanism, thickness, flexion)
                warm = [f"output/{mechanism}/{name(mechanism, h, f)}/portfolio_*/seeds.csv"
                        for h, f in harder(thickness, flexion)]
                (HERE / f"{stem}.yaml").write_text(config(mechanism, thickness, flexion, warm), encoding="utf-8")
                order.setdefault(mechanism, []).append(stem)
    header = ["#!/bin/bash", "# Run interface coverage configs, hardest first (make_configs.py).", "set -e",
              'cd "$(dirname "$0")/../../.."']
    scripts = list(order.items()) if argv else [("all", [s for m in MECHANISMS for s in order[m]]), *order.items()]
    for label, stems in scripts:
        lines = header + [f"python scripts/optimize/run.py scripts/optimize/{STUDY}/{stem}.yaml" for stem in stems]
        (HERE / f"run_{label}.sh").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {sum(map(len, order.values()))} configs and "
          + ", ".join(f"run_{label}.sh" for label, _ in scripts) + f" to {HERE}")


if __name__ == "__main__":
    main(sys.argv[1:])
