"""Verify the Pareto candidates of several runs on a fine grid, merge them and
mark the representative seeds.

Runs of one study complement each other: started from different populations
(seeds) or searched by different optimizers, they fill different parts of the
front, so a few of them merged come close to the front of many.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from core.optimization.study import Run, write_rows


def study_settings(run: Run) -> dict:
    """The run's study settings as loaded (so a setting added since, at its
    default, matches). The optimizer and its settings are not among them."""

    config = run.config
    return {f.name: getattr(config, f.name) for f in dataclasses.fields(config)}


def check_mergeable(runs: Sequence[Run]) -> None:
    """``ValueError`` unless the runs are of one mechanism, its current
    formulation, the same objectives and the same study settings."""

    first = runs[0]
    found, names = first.mechanism, first.objectives
    for run in runs:
        if run.mechanism is not found:
            raise ValueError(f"{run.directory} is a {run.mechanism.name} run, not {found.name}")
        if not run.current:
            raise ValueError(f"{run.directory} used the {run.formulation!r} formulation, not "
                             f"{found.formulation!r}; warm-start a new run from it instead")
        if run.objectives != names:
            raise ValueError(f"{run.directory} optimized {run.objectives}, not {names}")
    reference = study_settings(first)
    for run in runs[1:]:
        settings = study_settings(run)
        differing = sorted(key for key in reference.keys() | settings.keys()
                           if reference.get(key) != settings.get(key))
        if differing:
            raise ValueError(f"{run.directory} differs from {first.directory} in {', '.join(differing)}")


@dataclass(frozen=True)
class Selection:
    rows: list[dict[str, Any]]
    """The verified non-dominated candidates, representative seeds first, each
    with its ``seed_role`` (``pareto`` for the rest)."""
    counts: list[tuple[str, int, int]]
    """Per run: its name, Pareto candidates and those verified."""


def run_label(run: Run) -> str:
    return f"{run.directory.parent.name}/{run.directory.name}"


def select_seeds(runs: Sequence[Run], step_rad: float = math.radians(0.05)) -> Selection:
    """Re-evaluate every run's candidates with the motion sampled every
    ``step_rad``, keep the verified ones no other verified one dominates and
    mark the seeds (:meth:`~core.optimization.mechanism.Mechanism.seed_roles`).
    A candidate that holds several roles lists them all, joined by ``;``."""

    check_mergeable(runs)
    found, names = runs[0].mechanism, runs[0].objectives
    rows, counts = [], []
    for run in runs:
        config = found.refine(run.config, step_rad)
        candidates = run.candidates()
        verified = 0
        for row in candidates:
            x = found.read_design(row)
            e = found.evaluate(x, config)
            if e is None or not e.feasible:
                continue
            verified += 1
            rows.append({
                "run": run_label(run), "candidate": row["candidate"],
                "algorithm": run.settings.algorithm, "seed": run.settings.seed,
                **dict(zip(found.variable_names, x)),
                "objectives": found.objective_values(e, names),
                **found.report(e),
            })
        counts.append((run_label(run), len(candidates), verified))

    def dominates(first, second):
        return all(a <= b for a, b in zip(first, second)) and first != second

    rows = [row for row in rows
            if not any(dominates(other["objectives"], row["objectives"]) for other in rows)]
    order = found.role_order(names)
    for row in rows:
        row["roles"] = []
    for role, index in found.seed_roles([row["objectives"] for row in rows], rows, names).items():
        rows[index]["roles"].append(role)
    for row in rows:
        row["roles"].sort(key=order.index)
    rows.sort(key=lambda row: (order.index(row["roles"][0]) if row["roles"] else len(order),
                               row["objectives"][0]))
    for row in rows:
        row.pop("objectives")
        row["seed_role"] = ";".join(row.pop("roles")) or "pareto"
    return Selection(rows, counts)


def write_seeds(selection: Selection, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    write_rows(path, selection.rows, ("seed_role", "run", "candidate", "algorithm", "seed"))


def print_selection(selection: Selection, mechanism, output: Path) -> None:
    print(f"{'run':19s} pareto  verified")
    for name, candidates, verified in selection.counts:
        print(f"{name:19s} {candidates:6d}  {verified:8d}")
    print(f"\n{len(selection.rows)} verified non-dominated seeds -> {output}")
    print(f"  {'role':28s} {'run':14s} {'#':>4s} {'optimizer':>12s} {mechanism.summary_header()}")
    for row in selection.rows:
        if row["seed_role"] == "pareto":
            break
        optimizer = f"{row['algorithm']}/{row['seed']}"
        print(f"  {row['seed_role']:28s} {row['run']:14s} {row['candidate']:>4s} {optimizer:>12s} "
              f"{mechanism.summary_line(row)}")


__all__ = ["Selection", "check_mergeable", "print_selection", "select_seeds", "study_settings", "write_seeds"]
