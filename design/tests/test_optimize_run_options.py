"""scripts/optimize/run.py's --set and --tag: one entry of a config file
changed for a run, and the run sent to its own folder."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _run_module():
    spec = importlib.util.spec_from_file_location("optimize_run", ROOT / "scripts/optimize/run.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_set_changes_an_entry_and_tag_names_the_folder(tmp_path):
    from core.optimization.analysis.tradeoff import goal_of
    from core.optimization.study import load_run

    run = _run_module()
    code = run.main(["scripts/study/2r_dd.yaml",
                     "--set", "solver.optimizer.preset=quick",
                     "--set", "solver.optimizer.presets.quick.population=12",
                     "--set", "solver.optimizer.presets.quick.generations=2",
                     "--set", f"output={tmp_path}",
                     "--set", "goal.motion.required_flexion_deg=150",
                     "--tag", "q150"])
    assert code == 0
    folder = tmp_path / "revolute" / "2r_dd_q150"
    portfolio = json.loads((folder / "portfolio_001" / "portfolio.json").read_text())
    assert portfolio["tag"] == "q150"
    assert "goal.motion.required_flexion_deg=150" in portfolio["overrides"]
    saved = load_run(folder / "run_001")
    assert math.isclose(math.degrees(goal_of(saved.config).required_rom_rad), 150.0)
    assert run.main(["scripts/study/2r_dd.yaml", "--tag"]) == 2  # a flag without its value
