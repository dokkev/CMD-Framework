# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Attempt 9's hands (``docs/experimental/rl_pip_only.md``): the Study 3 hand grid for the finger-walk transfer.

- ``g<t>_<suffix>``: the e18 redesigns ``cfl_q120_h3to<t>_<suffix>`` (3 -> t mm pads, t in 5, 7, 9), exported with
  the current exporter by ``scripts/finger_relay/export_walk_grid.sh``;
- ``FA3``, ``FA5``, ``FA7``, ``FA9``: the fixed-axis family (one revolute PIP at the centre of O's joint gap, O's
  links; ``scripts/study3_measurement/fixed_axis_hands.py``) rebuilt from O's current export by the same script. Their
  distal pad rides on ``<finger>_servo_link`` (:data:`TOUCH_BODY`).

- ``MR``, ``MRM``, ``MRbz``, ``MRMbz``: the MCP-referenced 7 mm redesigns (``rotate_cube.grasp.MR_HANDS``).

Importing this module adds them to ``rotate_cube.grasp.HANDS`` (new keys only)."""

from __future__ import annotations

import os
from pathlib import Path

from ..rotate_cube.grasp import HANDS, MR_HANDS

GRID_HAND_DIR = Path(os.environ.get("CODE_WALK_GRID_HAND_DIR",
                                    Path(__file__).resolve().parents[4] / "logs" / "finger_walk" / "hands"))
GRID: dict[str, Path] = {}
for _p in sorted(GRID_HAND_DIR.glob("cfl_q120_h3to*_d*.json")):
    _stem = _p.stem.removeprefix("cfl_q120_h3to")  # e.g. "7_d1p5_minDs_s0"
    GRID[f"g{_stem}"] = _p
for _h in (3, 5, 7, 9):
    if (GRID_HAND_DIR / f"fa_q120_h{_h}.json").exists():
        GRID[f"FA{_h}"] = GRID_HAND_DIR / f"fa_q120_h{_h}.json"
for _k, _p in GRID.items():
    HANDS.setdefault(_k, _p)
for _k in MR_HANDS:  # the MCP-referenced redesigns (``docs/experimental/pr_mr.md``), already in HANDS
    GRID[_k] = HANDS[_k]

TOUCH_BODY = {k: ("servo_link" if k.startswith("FA") else "distal") for k in GRID}
"""The body that carries each finger's distal pad (the touch sensor's filter)."""

__all__ = ["GRID", "GRID_HAND_DIR", "TOUCH_BODY"]
