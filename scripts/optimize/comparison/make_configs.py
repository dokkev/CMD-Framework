#!/usr/bin/env python3
"""Write the mechanism comparison study's config files: at one extended
finger length (|OC| 99.5-100 mm), the most of it each mechanism can give to
contact interfaces usable all through a required flexion, per interface
thickness.

    python scripts/optimize/comparison/make_configs.py [mechanism ...]
    bash scripts/optimize/comparison/run_all.sh

Every config extends base.yaml (the frozen goal, the optimizer, no warm
start; runs in output/comparison/<mechanism>/<config>/) and sets the
interface thickness, the required flexion and the mechanism's generations.
The one objective is pad_coverage ((P + D) / |OC|, each pad's face usable
all over [0, flexion]); k and the finger's thickness are reported, not
limited. Mechanisms named: only theirs written (and run_<mechanism>.sh).

Besides the grid, VARIANTS: conditions run once to check a choice (the
linkage with its pads held to its links, at the base condition); CHECKS:
the bounds' checks (Task 2) at CHECK_CONDITIONS: the link shares widened
(the mechanisms' order kept?), and the extended centre of rotation held where
the 2R's lies (the moving centres' gain kept at the 2R's bending point?).
"""

import sys
from pathlib import Path

THICKNESSES_MM = (10, 5, 3)
"""Interface thicknesses, the thickest first."""
FLEXIONS_DEG = (150, 120, 90)
MECHANISMS = ("revolute", "revolute_parallel", "revolute_idealized", "rcj", "linkage")
"""In the figures' order: the 2R with its servo on the joint, on A' (the
actuator moved), without one (the fixed pivot alone), then the two whose
centre of rotation moves."""
POPULATION = 200
SEEDS = (0, 1, 2)
GENERATIONS = {"revolute": 600, "revolute_parallel": 600, "revolute_idealized": 600, "rcj": 600, "linkage": 600}
"""Every baseline at least the linkage's budget (population x generations x seeds)."""
BASE_CONDITION = (10, 150)
NO_OVERHANG = ["  pads:", "    proximal:", "      overhang_max_mm: 0", "    distal:", "      overhang_max_mm: 0"]
VARIANTS = {"linkage_onlinks": ("linkage", ["  pads:", "    on_links: true"]),
            "linkage_overhang0": ("linkage", NO_OVERHANG),
            "rcj_overhang0": ("rcj", NO_OVERHANG)}
"""Extra configs at BASE_CONDITION, each to check one choice: name:
(mechanism, lines under goal). The pads held to the linkage's links; no
overhang past a pad's body (the 2R's pads cannot pass their joint either way)."""

CHECK_CONDITIONS = ((5, 150), (5, 120), (5, 90))
CHECKS = {
    "ratio_wide": (MECHANISMS, ["  link:", "    proximal_length_ratio: [0.25, 0.65]",
                                "    distal_length_ratio: [0.25, 0.65]"]),
    # The 2R's pin A at 0.45-0.55 of |OC|: its |OA| and BC shares leave it nowhere else.
    "icr_control": (("rcj", "linkage"), ["  link:", "    icr0_share: [0.45, 0.55]"]),
}
"""name: (mechanisms, lines under goal), each run at CHECK_CONDITIONS."""

HERE = Path(__file__).resolve().parent
STUDY = "comparison"


def name(mechanism: str, thickness: int, flexion: int) -> str:
    return f"{mechanism}_h{thickness}_flex{flexion}"


def config(mechanism: str, thickness: int, flexion: int, label: str, goal_extra: list[str] = ()) -> str:
    sides = {"proximal": [f"      height_mm: {thickness}"], "distal": [f"      height_mm: {thickness}"]}
    top, extra, side = [], list(goal_extra), None
    if extra and extra[0] == "  pads:":  # merged into the pads table
        for line in extra[1:]:
            if line.strip().rstrip(":") in sides and line.startswith("    ") and not line.startswith("      "):
                side = line.strip().rstrip(":")
            elif side is not None:
                sides[side].append(line)
            else:
                top.append(line)
        extra = []
    pads = ["  pads:", *top, "    proximal:", *sides["proximal"], "    distal:", *sides["distal"]]
    lines = [
        f"# Mechanism comparison: {label}, interfaces {thickness} mm thick, {flexion} deg required (make_configs.py).",
        f"mechanism: {mechanism}",
        "base: base.yaml",
        "goal:",
        "  motion:",
        f"    dip_flexion_deg: {flexion}",
        *pads,
        *extra,
        "solver:",
        "  optimizer:",
        f"    population: {POPULATION}",
        f"    generations: {GENERATIONS[mechanism]}",
        f"    seeds: [{', '.join(map(str, SEEDS))}]",
    ]
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
                (HERE / f"{stem}.yaml").write_text(config(mechanism, thickness, flexion, mechanism), encoding="utf-8")
                order.setdefault(mechanism, []).append(stem)
    for label, (mechanism, extra) in VARIANTS.items():
        if argv and mechanism not in argv:
            continue
        stem = name(label, *BASE_CONDITION)
        (HERE / f"{stem}.yaml").write_text(config(mechanism, *BASE_CONDITION, label, extra), encoding="utf-8")
        order.setdefault("variants", []).append(stem)
    for label, (mechanisms, extra) in CHECKS.items():
        for mechanism in mechanisms:
            if argv and mechanism not in argv:
                continue
            for condition in CHECK_CONDITIONS:
                stem = name(f"{mechanism}_{label}", *condition)
                (HERE / f"{stem}.yaml").write_text(config(mechanism, *condition, f"{mechanism}, {label}", extra),
                                                   encoding="utf-8")
                order.setdefault(label, []).append(stem)
    header = ["#!/bin/bash", "# Run mechanism comparison configs (make_configs.py).", "set -e",
              'cd "$(dirname "$0")/../../.."']
    scripts = list(order.items()) if argv else [("all", [s for stems in order.values() for s in stems]),
                                                *order.items()]
    for label, stems in scripts:
        lines = header + [f"python scripts/optimize/run.py scripts/optimize/{STUDY}/{stem}.yaml" for stem in stems]
        (HERE / f"run_{label}.sh").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {sum(map(len, order.values()))} configs and "
          + ", ".join(f"run_{label}.sh" for label, _ in scripts) + f" to {HERE}")


if __name__ == "__main__":
    main(sys.argv[1:])
