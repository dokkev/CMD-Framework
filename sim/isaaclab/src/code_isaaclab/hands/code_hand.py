# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Articulation and :class:`~code_isaaclab.hands.HandSpec` builders for ``code_hand_sim/1`` exports.

Because ``design/`` owns the parametric morphology and exports versioned JSON descriptions, nothing here
hardcodes link lengths, joint limits, or servo constants. Any ``code_hand_sim/1`` export—whether the CoDe
closed four-bar hand or an open-chain baseline (``loops`` and ``four_bar`` empty)—is turned into an Isaac Lab
articulation and :class:`HandSpec` through the same functions.
"""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorBaseCfg, ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.markers import VisualizationMarkersCfg

from code_isaaclab.actuators import ServoMode, xm335_actuator_cfg
from code_isaaclab.assets.description import Description

from . import HandSpec

PASSIVE_ARMATURE_DEFAULT = 1.0e-5
"""Minimal armature on passive tree joints so soft loop constraints do not snap open in transients (E2)."""


def code_hand_articulation_cfg(
    desc: Description,
    usd_path: str | Path,
    *,
    prim_path: str = "{ENV_REGEX_NS}/Robot",
    servo_mode: ServoMode = "dc_motor",
    servo_actuator: ActuatorBaseCfg | None = None,
    servo_armature: float | None = None,
    passive_armature: float = PASSIVE_ARMATURE_DEFAULT,
    init_pos: tuple[float, float, float] = (0.0, 0.0, 0.3),
    init_rot: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    init_joint_pos: dict[str, float] | None = None,
    activate_contact_sensors: bool = False,
) -> ArticulationCfg:
    """Build an :class:`ArticulationCfg` from an exported :class:`Description` and its authored USD stage."""
    actuated = desc.actuated()
    passive = desc.passive()
    servos_cfg = (
        servo_actuator
        if servo_actuator is not None
        else xm335_actuator_cfg(desc.servo, actuated, mode=servo_mode, armature=servo_armature)
    )
    actuators: dict[str, ActuatorBaseCfg] = {"servos": servos_cfg}
    if passive:
        actuators["passive"] = ImplicitActuatorCfg(
            joint_names_expr=passive,
            stiffness=0.0,
            damping=0.0,
            armature=passive_armature,
        )
    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path),
            activate_contact_sensors=activate_contact_sensors,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=init_pos,
            rot=init_rot,
            joint_pos={".*": 0.0} if init_joint_pos is None else dict(init_joint_pos),
        ),
        actuators=actuators,
    )


def make_code_hand_spec(
    desc: Description,
    usd_path: str | Path,
    *,
    name: str = "code_hand",
    servo_mode: ServoMode = "dc_motor",
    servo_armature: float | None = None,
    passive_armature: float = PASSIVE_ARMATURE_DEFAULT,
    init_pos: tuple[float, float, float] = (0.0, 0.0, 0.5),
    init_rot: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    init_joint_pos: dict[str, float] | None = None,
    activate_contact_sensors: bool = False,
    object_cfg: RigidObjectCfg | None = None,
    goal_marker_cfg: VisualizationMarkersCfg | None = None,
    goal_marker_offset: tuple[float, float, float] | None = None,
    goal_orientation_threshold: float = 0.2,
    decimation: int = 4,
    mjwarp_buffers: dict[str, int] | None = None,
) -> HandSpec:
    """Create a :class:`HandSpec` from ``desc`` and ``usd_path`` with closed-loop hooks enabled automatically."""
    if object_cfg is None or goal_marker_cfg is None or goal_marker_offset is None:
        from isaaclab_tasks.core.reorient.config.allegro_hand.allegro_hand_common import (
            CUBE_CFG,
            GOAL_MARKER_OFFSET,
            GOAL_OBJECT_CFG,
        )

        object_cfg = CUBE_CFG if object_cfg is None else object_cfg
        goal_marker_cfg = GOAL_OBJECT_CFG if goal_marker_cfg is None else goal_marker_cfg
        goal_marker_offset = GOAL_MARKER_OFFSET if goal_marker_offset is None else goal_marker_offset

    has_loops = bool(desc.loops)
    default_buffers = {"njmax": 300, "nconmax": 120} if has_loops else {"njmax": 120, "nconmax": 80}
    robot_cfg = code_hand_articulation_cfg(
        desc,
        usd_path,
        servo_mode=servo_mode,
        servo_armature=servo_armature,
        passive_armature=passive_armature,
        init_pos=init_pos,
        init_rot=init_rot,
        init_joint_pos=init_joint_pos,
        activate_contact_sensors=activate_contact_sensors,
    )
    return HandSpec(
        name=name,
        robot_cfg=robot_cfg,
        actuated_joint_names=desc.actuated(),
        fingertip_body_names=desc.fingertip_bodies(),
        object_cfg=object_cfg,
        goal_marker_cfg=goal_marker_cfg,
        goal_marker_offset=goal_marker_offset,
        goal_orientation_threshold=goal_orientation_threshold,
        decimation=decimation,
        mjwarp_buffers=default_buffers if mjwarp_buffers is None else dict(mjwarp_buffers),
        stiffen_mjwarp_loops=has_loops,
        four_bar_tables=dict(desc.four_bar) if desc.four_bar else None,
    )


__all__ = ["PASSIVE_ARMATURE_DEFAULT", "code_hand_articulation_cfg", "make_code_hand_spec"]
