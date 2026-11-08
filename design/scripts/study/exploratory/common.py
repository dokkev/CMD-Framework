"""Shared helpers for the exploratory studies E01-E10.

Thin glue over the Study 1/2 infrastructure (`../shared.py`, Study 1's
`assembly`), so every E-study judges designs with the mechanisms' own
`evaluate()` and draws solids from the existing scenes. Nothing here
reimplements mechanism mathematics.
"""
from __future__ import annotations

import dataclasses
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
for path in (str(ROOT), str(HERE.parent)):
    if path not in sys.path:
        sys.path.insert(0, path)

import shared  # noqa: E402  (scripts/study/shared.py)

study1 = shared.load_module("study1_run", HERE.parent / "study1_fixed_morphology" / "run.py")

from core.optimization.analysis.tradeoff import goal_of, with_goal, with_modules  # noqa: E402
from core.optimization.mechanism import mechanism  # noqa: E402

OUTPUT = ROOT / "output" / "study" / "exploratory"
FIGURES = ROOT.parent / "report" / "figures" / "exploratory"  # report/ is at the monorepo root
LABEL = {
    "code_matched": "CoDe (tilt 10°)",
    "2r_dd_tilt0_matched": "2R-DD, servo 0°",
    "2r_dd_tilt90_matched": "2R-DD, servo 90°",
    "belt_2r_matched": "Belt 2R (tilt 10°)",
    "rcj_tilt0_matched": "RCJ, servo 0°",
    "rcj_tilt90_matched": "RCJ, servo 90°",
    "rcj_ideal_matched": "Ideal RCJ (upper bound)",
}
PHYSICAL = tuple(k for k in LABEL if k != "rcj_ideal_matched")


def references() -> dict:
    found = shared.load_references()
    shared.check_comparable(found)
    return {r.reference_id: r for r in found}


def catalog() -> dict:
    return shared.catalog()


def carried(reference, modules):
    """The frozen reference carrying ``modules`` at policy depths: ``(x, config)``; ``x`` None if it does not mount."""
    return study1.frozen_design(reference, modules)


def with_packaging(config, **changes):
    """``config`` with packaging fields replaced, rebuilt through the goal (bounds follow)."""
    goal = goal_of(config)
    return with_goal(config, dataclasses.replace(goal, packaging=dataclasses.replace(goal.packaging, **changes)))


def output(name: str, fresh: bool = True) -> Path:
    out = OUTPUT / name
    if fresh and out.exists():
        import shutil
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)
    return out


def finish(out: Path, rows: list[dict], config: dict, table: str = "results.csv", **extra) -> None:
    """Write the result table and a manifest with the source record."""
    if rows:
        shared.write_csv(out / table, rows)
    manifest = {"config": config, "source": shared.git_commit(out), **extra}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str) + "\n", encoding="utf-8")


__all__ = ["FIGURES", "LABEL", "OUTPUT", "PHYSICAL", "ROOT", "carried", "catalog", "finish", "goal_of", "mechanism",
           "output", "references", "shared", "study1", "with_goal", "with_modules", "with_packaging"]
