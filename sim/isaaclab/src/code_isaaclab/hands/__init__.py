# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Hand specifications: everything a task needs to know about one hand.

A task is written once against :class:`HandSpec`. Adding a hand, including the CoDe hand built from
CMD-Framework's exported geometry (:mod:`code_isaaclab.hands.code_hand`), means writing or calling a spec
factory and registering it here. No task MDP code changes.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.markers import VisualizationMarkersCfg


@dataclass(frozen=True)
class HandSpec:
    """One hand and the in-hand object it is evaluated with."""

    name: str
    """Registry key and the suffix of the task ids (``CoDe-Reorient-Cube-<Name>``)."""
    robot_cfg: ArticulationCfg
    """The hand articulation, with ``prim_path="{ENV_REGEX_NS}/Robot"``."""
    actuated_joint_names: list[str]
    """Joints the action term drives."""
    fingertip_body_names: list[str]
    """Bodies whose poses and velocities the policy observes."""
    object_cfg: RigidObjectCfg
    """The in-hand object, with ``prim_path="{ENV_REGEX_NS}/object"``."""
    goal_marker_cfg: VisualizationMarkersCfg
    """Marker shown at the goal orientation."""
    goal_marker_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Goal marker display offset from the in-hand position [m]."""
    goal_orientation_threshold: float = 0.2
    """Orientation error below which a goal counts as reached [rad]."""
    decimation: int = 4
    """Physics steps per policy step."""
    mjwarp_buffers: dict[str, int] = field(default_factory=lambda: {"njmax": 80, "nconmax": 70})
    """MuJoCo-Warp constraint and contact buffer sizes for this hand (they scale with joints and contacts)."""
    stiffen_mjwarp_loops: bool = False
    """Tighten MuJoCo-Warp equality constraints on startup (required for closed four-bar hands)."""
    four_bar_tables: dict[str, dict[str, list[float]]] | None = None
    """Per-digit four-bar closure tables used to synchronize passive joints after environment resets."""


HANDS: dict[str, HandSpec] = {}
"""Registered hands by name."""


def register_hand(spec: HandSpec) -> HandSpec:
    if spec.name in HANDS:
        raise ValueError(f"hand {spec.name!r} is already registered")
    HANDS[spec.name] = spec
    return spec


def get_hand(name: str) -> HandSpec:
    try:
        return HANDS[name]
    except KeyError:
        raise KeyError(f"unknown hand {name!r}; registered: {sorted(HANDS)}") from None


from . import allegro, code_hand  # noqa: E402,F401  (registers the reference hand; exposes CoDe builders)

__all__ = ["HANDS", "HandSpec", "code_hand", "get_hand", "register_hand"]
