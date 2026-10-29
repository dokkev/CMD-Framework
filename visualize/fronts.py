"""Reading a study's verified fronts: a portfolio's ``seeds.csv`` (merged over
its runs, see ``scripts/optimize/run.py``), its rows and their runs."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from core.optimization.study import load_run


def read_rows(path: Path) -> list[dict[str, str]]:
    return list(csv.DictReader(Path(path).open(encoding="utf-8", newline="")))


def column(rows, name: str) -> np.ndarray:
    return np.array([float(row[name]) for row in rows])


def has_role(row: dict[str, str], role: str) -> bool:
    return role in row["seed_role"].split(";")


def with_role(rows, role: str) -> dict[str, str] | None:
    """The representative seed of ``role`` (``balanced``, ``most_pad``, ...)."""

    return next((row for row in rows if has_role(row, role)), None)


def latest_portfolio(study: Path) -> Path | None:
    """A study's (or preset's) latest merged front: its ``portfolio_NNN/seeds.csv``."""

    fronts = sorted(Path(study).glob("portfolio_*/seeds.csv"))
    return fronts[-1] if fronts else None


def run_of(seeds: Path, row: dict[str, str]):
    """The run a row of ``seeds`` (a portfolio's CSV) came from."""

    return load_run(Path(seeds).parent.parent / row["run"].split("/")[-1])


def load_front(portfolio: Path) -> dict:
    """A merged front: its rows, mechanism and config (from its first run)."""

    rows = read_rows(portfolio / "seeds.csv")
    names = json.loads((portfolio / "portfolio.json").read_text(encoding="utf-8"))["runs"]
    run = load_run(portfolio.parent / names[0])
    return {"rows": rows, "mechanism": run.mechanism, "config": run.config}
