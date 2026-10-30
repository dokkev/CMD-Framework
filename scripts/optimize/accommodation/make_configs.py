#!/usr/bin/env python3
"""Write the accommodation study's configs: per mechanism and module pair,
the shortest finger that carries both modules through the required flexion.

    python scripts/optimize/accommodation/make_configs.py
    bash scripts/optimize/accommodation/run_all.sh

Experiment A varies the proximal module over LENGTHS_MM x HEIGHTS_MM with the
distal one at REFERENCE; experiment B the distal one, the proximal at
REFERENCE (the pair at REFERENCE, in both, is written once); each at every
required flexion of FLEXIONS_DEG, for every mechanism (one that finds no
design reports none found within the bounds, not "unreachable"). Every config extends base.yaml (the comparison model, runs in
output/accommodation/<mechanism>/<config>/). Run scripts: run_all.sh, one
per mechanism, one per flexion, and run_reference.sh (the reference pair at
every flexion, the first to run).
"""

from pathlib import Path

MECHANISMS = ("revolute", "linkage")
"""The 2R with its servo on the joint, and CoDe."""
LENGTHS_MM = (15, 25, 35)
HEIGHTS_MM = (3, 5, 10)
REFERENCE = (25, 5)
"""The fixed module (length, thickness) of each experiment."""
FLEXIONS_DEG = (90, 120, 150, 180)
"""Required flexions: the result is the shortest finger at each."""

HERE = Path(__file__).resolve().parent


def name(mechanism: str, proximal: tuple[int, int], distal: tuple[int, int], flexion: int) -> str:
    return f"{mechanism}_P{proximal[0]}x{proximal[1]}_D{distal[0]}x{distal[1]}_flex{flexion}"


def pairs() -> list[tuple[tuple[int, int], tuple[int, int]]]:
    """The module pairs: experiment A's, then B's not already in A."""

    grid = [(length, height) for length in LENGTHS_MM for height in HEIGHTS_MM]
    found = [(module, REFERENCE) for module in grid]
    return found + [(REFERENCE, module) for module in grid if (REFERENCE, module) not in found]


def config(mechanism: str, proximal: tuple[int, int], distal: tuple[int, int], flexion: int) -> str:
    lines = [
        f"# Accommodation: {mechanism}, proximal module {proximal[0]} x {proximal[1]} mm, "
        f"distal {distal[0]} x {distal[1]} mm, {flexion} deg required (make_configs.py).",
        f"mechanism: {mechanism}",
        "base: base.yaml",
        "goal:",
        "  motion:",
        f"    dip_flexion_deg: {flexion}",
        "  pads:",
        *(line for side, (length, height) in (("proximal", proximal), ("distal", distal))
          for line in (f"    {side}:", f"      length_mm: [{length}, {length}]", f"      height_mm: {height}")),
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    for stale in HERE.glob("*_P*_D*.yaml"):  # written by an earlier grid
        stale.unlink()
    stems = []
    for flexion in FLEXIONS_DEG:
        for mechanism in MECHANISMS:
            for proximal, distal in pairs():
                stem = name(mechanism, proximal, distal, flexion)
                (HERE / f"{stem}.yaml").write_text(config(mechanism, proximal, distal, flexion), encoding="utf-8")
                stems.append(stem)
    header = ["#!/bin/bash", "# Run the accommodation study's configs (make_configs.py).", "set -e",
              'cd "$(dirname "$0")/../../.."']
    reference = f"_P{REFERENCE[0]}x{REFERENCE[1]}_D{REFERENCE[0]}x{REFERENCE[1]}_"
    scripts = [("all", stems), ("reference", [s for s in stems if reference in s]),
               *((m, [s for s in stems if s.startswith(f"{m}_P")]) for m in MECHANISMS),
               *((f"flex{f}", [s for s in stems if s.endswith(f"_flex{f}")]) for f in FLEXIONS_DEG)]
    for label, chosen in scripts:
        lines = header + [f"python scripts/optimize/run.py scripts/optimize/accommodation/{stem}.yaml"
                          for stem in chosen]
        (HERE / f"run_{label}.sh").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {len(stems)} configs and " + ", ".join(f"run_{label}.sh" for label, _ in scripts))


if __name__ == "__main__":
    main()
