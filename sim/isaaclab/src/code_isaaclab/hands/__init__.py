# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Hand specifications: everything a task needs to know about one hand.

A task is written once against :class:`HandSpec`. Adding a hand, including the CoDe hand built from
CMD-Framework's exported geometry, means writing one spec module and registering it here. No task code
changes.
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


from . import allegro  # noqa: E402,F401  (registers the reference hand)

__all__ = ["HANDS", "HandSpec", "get_hand", "register_hand"]
