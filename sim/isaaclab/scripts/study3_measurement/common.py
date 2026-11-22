# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Shared pieces of the Study 3 measurement (design/SSoT/problem_formulation.md A.6.4).

Run records (a unique run id, the git state and versions, immutable result files), the hand's body order, and where
the hand and its middle finger sit in the scene. The archived learning programme keeps its own helpers in
``scripts/study3/common.py``; nothing here depends on it.
"""

from __future__ import annotations

import datetime as _dt
import json
import os
import platform
import subprocess
from pathlib import Path
from typing import Any

SIM_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = SIM_ROOT.parents[1]
LOG_ROOT = SIM_ROOT / "logs" / "experimental" / "two_pad"
"""Where every Study 3 run and analysis is written (kept where the first runs were written)."""
HAND_DIR = REPO_ROOT / "design" / "output" / "sim" / "study3" / "hands"

HANDS = {k: HAND_DIR / f"cfl_q120_h3to7_{v}.json" for k, v in (("O", "h0"), ("R", "hr"), ("U", "hu"))}
"""The reference O and the 7 mm redesigns R (keeps the contact-surface motion) and U (coverage alone)."""
MCP_X_M = 0.0375
"""The middle finger's MCP flexion axis, hand x [m]."""
FINGER_Y_M = -0.025
"""The middle finger, hand y [m]."""
HAND_Z = 0.5
"""Height of the hand root above the ground [m]."""


def proximal_face_z(desc: dict) -> float:
    """Height of the middle finger's proximal contact face in the hand description at q = 0 [m]."""
    box = next(b for body in desc["bodies"] for b in (body.get("boxes") or []) if b["name"] == "middle_proximal_pad")
    return float(box["center"][2]) - float(box["half_extents"][1])


def run_id(prefix: str) -> str:
    """A unique, sortable identifier (results are never overwritten)."""
    return f"{prefix}_{_dt.datetime.now().strftime('%Y%m%d-%H%M%S')}_{os.getpid()}"


def _git(*args: str) -> str:
    try:
        return subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def versions() -> dict[str, str]:
    from importlib.metadata import PackageNotFoundError, version

    found = {"python": platform.python_version()}
    for name in ("isaaclab", "newton", "warp", "torch", "mujoco_warp"):
        try:
            found[name] = version(name)
        except PackageNotFoundError:
            found[name] = "n/a"
    return found


def gpu() -> dict[str, Any]:
    try:
        import torch

        return {
            "device": torch.cuda.get_device_name(0),
            "max_memory_allocated_gb": round(torch.cuda.max_memory_allocated() / 1e9, 3),
            "max_memory_reserved_gb": round(torch.cuda.max_memory_reserved() / 1e9, 3),
        }
    except Exception:  # noqa: BLE001 (no CUDA device or no torch)
        return {"device": "n/a"}


def manifest(**extra: Any) -> dict[str, Any]:
    """Git state, versions and GPU, plus ``extra``."""
    status = _git("status", "--porcelain")
    return {
        "created": _dt.datetime.now().isoformat(timespec="seconds"),
        "git_commit": _git("rev-parse", "HEAD"),
        "git_dirty": bool(status) and status != "unknown",
        "git_dirty_files": status.splitlines()[:50] if status != "unknown" else [],
        "versions": versions(),
        "gpu": gpu(),
        **extra,
    }


def write_json(path: Path, data: Any) -> Path:
    """Write a result; an existing file is never overwritten."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(f"{path} exists: results are immutable, use a new run id")
    path.write_text(json.dumps(data, indent=2, default=str) + "\n", encoding="utf-8")
    return path


def hand_bodies() -> list[str]:
    """The hand's bodies in the description's order."""
    from code_isaaclab.assets.description import Description
    from code_isaaclab.hands.code_variants import STUDY3_DESCRIPTION

    return [b["name"] for b in Description.load(STUDY3_DESCRIPTION).bodies]


def torch_of(value):
    """A torch view of a torch tensor, a Newton array or a warp array."""
    import torch

    if isinstance(value, torch.Tensor):
        return value
    if hasattr(value, "torch"):
        return value.torch
    import warp as wp

    return wp.to_torch(value)
