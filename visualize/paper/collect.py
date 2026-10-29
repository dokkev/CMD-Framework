"""Gather the comparison's best design per mechanism and condition into one
table the paper's figures read (they read nothing else of the runs).

    python -m visualize.paper.collect [source]

``source``: the comparison study's output folder (default
``output/comparison_quick``, the quick runs: drafts). Per mechanism,
interface thickness and required flexion (``scripts/optimize/comparison``'s
grid): the verified front's most pad coverage, its pads, ``k_max``, the
extended centre of rotation's share of ``|OC|``, the most flexion, the
extended finger's dorsal-palmar thickness (its own solids and pads), the
servo's tilt and the bound hits, each
run's own best (the seeds' spread), and the share of the two faces an
object ``goal.loss.access_depth_mm`` deep (ACCESS_DEPTH_MM without one)
cannot reach at the required flexion. The loss rule is the run's
(``goal.loss.integrity_depth_mm``; without one, v4's fold), stated in the
table (``loss_rule``) with the source. Writes output/paper/data/<source name>.csv.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np

from core.optimization.analysis.icr import icr0_share
from core.optimization.analysis.loss import face_loss, scene, thickness_mm
from core.optimization.study import load_run
from visualize.fronts import latest_portfolio, read_rows
from visualize.studies import grid
from visualize.style import MECHANISMS, OUTPUT, REPOSITORY_ROOT

STUDY = grid("comparison")
ACCESS_DEPTH_MM = 20.0
"""The accessibility depth of runs whose goal sets none."""
DATA = OUTPUT / "paper" / "data"


def best_of(config_dir: Path):
    """The verified front's most-pad row and the runs' own bests (``None``: no front)."""

    front = latest_portfolio(config_dir)
    if front is None:
        return None, []
    rows = read_rows(front)
    seeds = []
    for run in sorted(config_dir.glob("run_*")):
        pareto = read_rows(run / "pareto.csv") if (run / "pareto.csv").exists() else []
        if pareto:
            seeds.append(max(float(r["pad_coverage"]) for r in pareto))
    return max(rows, key=lambda r: float(r["pad_coverage"]), default=None), seeds


def loss_rule(goal) -> str:
    depth = goal.integrity_depth_mm
    return "segment (v4)" if depth is None else f"integrity {depth:g} mm"


def collect(source: Path) -> list[dict]:
    table = []
    for mechanism in MECHANISMS:
        for thickness in STUDY.THICKNESSES_MM:
            for flexion in STUDY.FLEXIONS_DEG:
                config_dir = source / mechanism / STUDY.name(mechanism, thickness, flexion)
                row, seeds = best_of(config_dir)
                entry = {"mechanism": mechanism, "thickness_mm": thickness, "flexion_deg": flexion,
                         "source": str(source.relative_to(REPOSITORY_ROOT)),
                         "has_front": latest_portfolio(config_dir) is not None, "runs": len(seeds),
                         "seed_min_pct": 100 * min(seeds) if seeds else None,
                         "seed_max_pct": 100 * max(seeds) if seeds else None}
                if row is not None:
                    run = load_run(config_dir / row["run"].split("/")[-1])
                    x = run.mechanism.read_design(row)
                    length = float(row["finger_length_mm"])
                    built = scene(mechanism, run.config, x, [float(flexion)])
                    dorsal, palmar = thickness_mm(scene(mechanism, run.config, x, [0.0]))
                    goal = getattr(run.config, "goal", run.config)
                    access = face_loss(built, "probe", goal.access_depth_mm)
                    lost = sum(float(access[side].lost[0]) for side in access)
                    faces = sum(float(access[side].length[0]) for side in access)
                    entry.update({
                        "coverage_pct": 100 * float(row["pad_coverage"]),
                        "proximal_mm": float(row["proximal_effective_mm"]),
                        "distal_mm": float(row["distal_effective_mm"]),
                        "k_max": float(row["torque_ratio_max"]), "finger_length_mm": length,
                        "icr0_share": icr0_share(mechanism, x, run.mechanism.variable_names, length),
                        "phi_max_deg": float(row["phi_max_deg"]),
                        "dorsal_z_mm": dorsal, "palmar_z_mm": palmar, "thickness_mm": dorsal - palmar,
                        "servo_tilt_deg": float(row["servo_tilt_deg"]) if row.get("servo_tilt_deg") else None,
                        "bound_hits": row.get("bound_hits", ""),
                        "loss_rule": loss_rule(goal), "access_depth_mm": goal.access_depth_mm,
                        "access_lost_pct_at_flexion": 100 * lost / faces if faces else math.nan,
                        "run": f"{config_dir.relative_to(REPOSITORY_ROOT)}/{row['run'].split('/')[-1]}",
                        "candidate": row["candidate"],
                        "x": " ".join(f"{value:g}" for value in x),
                    })
                table.append(entry)
    return table


def main(argv: list[str]) -> int:
    source = (REPOSITORY_ROOT / (argv[0] if argv else "output/comparison_quick")).resolve()
    table = collect(source)
    DATA.mkdir(parents=True, exist_ok=True)
    path = DATA / f"{source.name}.csv"
    columns = list(dict.fromkeys(key for row in table for key in row))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(table)
    found = sum(1 for row in table if row.get("coverage_pct") is not None)
    print(f"{found} of {len(table)} cells with a design -> {path.relative_to(REPOSITORY_ROOT)}")
    return 0


def read(source: str = "comparison_quick") -> list[dict]:
    """The collected table (numbers as floats, empty as ``None``)."""

    rows = []
    for row in csv.DictReader((DATA / f"{source}.csv").open(encoding="utf-8", newline="")):
        parsed = {}
        for key, value in row.items():
            if value in ("", "None"):
                parsed[key] = None
            elif key in ("mechanism", "source", "loss_rule", "run", "candidate", "x", "bound_hits"):
                parsed[key] = value
            elif value in ("True", "False"):
                parsed[key] = value == "True"
            else:
                parsed[key] = float(value)
        rows.append(parsed)
    return rows


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
