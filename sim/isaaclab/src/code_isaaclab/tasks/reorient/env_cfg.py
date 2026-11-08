# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Hand-agnostic in-hand cube reorientation, on top of Isaac Lab's manager-based reorientation task.

The MDP is upstream's: observations, actions, goal command, rewards, terminations and reset events
(``isaaclab_tasks.core.reorient``). Two things come from elsewhere:
- the hand, from a :class:`~code_isaaclab.hands.HandSpec` named by :attr:`HandReorientEnvCfg.hand`;
- the physics backend, from :class:`~code_isaaclab.physics.HandPhysicsCfg`, chosen with ``physics=<name>``.
"""

from dataclasses import MISSING

from isaaclab.utils import configclass

from isaaclab_tasks.core.reorient.config.allegro_hand.allegro_hand_manager_env_cfg import (
    AllegroHandEventPresetCfg as ReorientEventPresetCfg,  # hand-agnostic: "robot" and "object" entities only
)
from isaaclab_tasks.core.reorient.reorient_manager_env_cfg import ReorientManagerEnvBaseCfg, ReorientSceneBaseCfg

from code_isaaclab.hands import get_hand
from code_isaaclab.physics import HandPhysicsCfg, apply_hand_buffers


@configclass
class HandReorientSceneCfg(ReorientSceneBaseCfg):
    """The shared scene. The hand and object are filled in from the hand spec."""


@configclass
class HandReorientEnvCfg(ReorientManagerEnvBaseCfg):
    """In-hand cube reorientation for any registered hand."""

    hand: str = MISSING
    """Name of the registered :class:`~code_isaaclab.hands.HandSpec`."""

    scene: HandReorientSceneCfg = HandReorientSceneCfg()
    # ``presets=randomized`` adds upstream's domain-randomization terms; ``reset_only`` is the default
    events: ReorientEventPresetCfg = ReorientEventPresetCfg()

    def __post_init__(self):
        spec = get_hand(self.hand)
        # what the upstream base reads in its __post_init__
        self.fingertip_body_names = list(spec.fingertip_body_names)
        self.actuated_joint_names = list(spec.actuated_joint_names)
        self.goal_orientation_threshold = spec.goal_orientation_threshold
        self.goal_marker_cfg = spec.goal_marker_cfg
        self.decimation = spec.decimation
        self.scene.robot = spec.robot_cfg
        self.scene.object = spec.object_cfg
        super().__post_init__()
        self.commands.object_pose.marker_pos_offset = spec.goal_marker_offset
        self.sim.physics = HandPhysicsCfg()
        apply_hand_buffers(self.sim.physics, spec.mjwarp_buffers)


@configclass
class AllegroReorientEnvCfg(HandReorientEnvCfg):
    """The reference hand; reproduces ``Isaac-Reorient-Cube-Allegro`` on the project's physics presets."""

    hand = "allegro"
