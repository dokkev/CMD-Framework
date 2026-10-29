"""The comparison studies' fronts, by their configs' names: each study's grid
is its ``scripts/optimize/<study>/make_configs.py``, its fronts
``output/<mechanism>/<config>/portfolio_NNN/seeds.csv``."""

from __future__ import annotations

import functools
import importlib.util
import json
from pathlib import Path
from types import ModuleType

from visualize.finger import Finger, finger_from_row
from visualize.fronts import latest_portfolio, read_rows
from visualize.style import OUTPUT, REPOSITORY_ROOT


@functools.cache
def grid(study: str) -> ModuleType:
    """The study's ``make_configs`` module: its grid's constants and config names."""

    path = REPOSITORY_ROOT / "scripts" / "optimize" / study / "make_configs.py"
    spec = importlib.util.spec_from_file_location(f"{study}_configs", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def config_front(mechanism: str, config: str) -> list[dict[str, str]]:
    """A config's latest verified front (none run yet: empty)."""

    front = latest_portfolio(OUTPUT / mechanism / config)
    return read_rows(front) if front else []


# The interface coverage study (scripts/optimize/interface_coverage).

def stem(mechanism: str, height: int, flexion: int) -> str:
    return f"{mechanism}_cov_h{height}_flex{flexion}"


def latest_front(mechanism: str, height: int, flexion: int) -> Path | None:
    return latest_portfolio(OUTPUT / mechanism / stem(mechanism, height, flexion))


def front_rows(mechanism: str, height: int, flexion: int) -> list[dict[str, str]]:
    return config_front(mechanism, stem(mechanism, height, flexion))


def best_row(mechanism: str, height: int, flexion: int) -> dict[str, str] | None:
    """The verified front's most pad (the study's one objective)."""

    return max(front_rows(mechanism, height, flexion), key=lambda row: float(row["pad_coverage"]), default=None)


def seed_bests(mechanism: str, height: int, flexion: int) -> list[float]:
    """Each run's best ``P + D`` (its own front, on the optimization grid), for the
    latest portfolio's runs: the spread between seeds."""

    front = latest_front(mechanism, height, flexion)
    if front is None:
        return []
    portfolio = json.loads((front.parent / "portfolio.json").read_text(encoding="utf-8"))
    values = []
    for run in portfolio["runs"]:
        path = front.parent.parent / run / "pareto.csv"
        if not path.exists():
            continue
        rows = read_rows(path)
        if rows:
            values.append(max(float(r["proximal_effective_mm"]) + float(r["distal_effective_mm"]) for r in rows))
    return values


def pad_total(row: dict[str, str] | None) -> float | None:
    return None if row is None else float(row["proximal_effective_mm"]) + float(row["distal_effective_mm"])


def best(mechanism: str, height: int, flexion: int) -> Finger | None:
    """The most-pad design (none: ``None``)."""

    row = best_row(mechanism, height, flexion)
    if row is None:
        return None
    return finger_from_row(mechanism, row, OUTPUT / mechanism / stem(mechanism, height, flexion))
