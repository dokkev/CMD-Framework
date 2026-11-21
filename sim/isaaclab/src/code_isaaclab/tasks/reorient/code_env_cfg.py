# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""In-hand cube rotation with the CoDe hand, one task per contact interface (Study 3 pilot).

The MDP is :class:`HandReorientEnvCfg`'s (upstream's reorientation terms). What the CoDe hand changes, the
same for every contact interface:

- the hand palm up, the cube (50 mm, 50 g) resting on the extended fingers' pads (:mod:`code_variants`);
- the goal: rotations of the cube about the vertical, up to 30 deg (:class:`AxisReorientCommandCfg`), since
  three parallel fingers cannot reach most of SO(3) without regrasping;
- the reset: the cube at rest on the pads with a few millimetres of noise and its reset orientation the
  identity (upstream drops the Allegro's cube in a random orientation); the hand's joints with less noise
  (upstream's 0.2 of the way to the limits would knock the cube off a flat hand);
- the fall condition: the cube more than 8 cm from its goal position (upstream's 24 cm is for a larger hand).

Only the hand (``code_flat`` / ``code_crowned``) differs between the tasks.
"""

from __future__ import annotations

import math

from isaaclab.envs import mdp as base_mdp
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.utils import configclass

from isaaclab_tasks.core.reorient import mdp

from code_isaaclab.hands.code_variants import CUBE_DROP_M, register_code_variants

from .code_mdp import AxisReorientCommandCfg
from .env_cfg import HandReorientEnvCfg, HandReorientSceneCfg

GOAL_AXIS = (0.0, 0.0, 1.0)
"""World axis of the goal rotations (vertical: the cube turned while it rests on the pads).

Chosen from scripted probes of the flat variant before any training (report/study3/study3_pilot.md): the three
fingers abducted together turn the cube about the vertical by about 11 deg with five or six pads in contact,
while flexion tips the cube over onto the palm (a 90 deg flip about the finger axes, with non-pad contact)."""
GOAL_MAX_ANGLE = math.radians(30.0)
GOAL_MIN_STEP = math.radians(20.0)
"""Consecutive goals at least 20 deg apart: a new goal is never within the 0.2 rad (11.5 deg) success band."""
OBJECT_POSE_NOISE_M = 0.003
HAND_JOINT_NOISE = 0.02
"""Fraction of the way from the default joint position toward a random limit (upstream: 0.2)."""
FALL_DISTANCE_M = 0.08
EPISODE_S = 10.0


@configclass
class CoDeReorientEventCfg:
    """Per-episode reset: the cube near its resting pose, the hand near extension, its four-bars closed."""

    reset_object = EventTermCfg(
        func=base_mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-OBJECT_POSE_NOISE_M, OBJECT_POSE_NOISE_M),
                "y": (-OBJECT_POSE_NOISE_M, OBJECT_POSE_NOISE_M),
            },
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object"),
        },
    )
    reset_hand = EventTermCfg(
        func=mdp.reset_reorient_hand,
        mode="reset",
        params={"joint_position_noise": HAND_JOINT_NOISE, "joint_velocity_noise": 0.0},
    )
    stiffen_loops: EventTermCfg | None = None
    close_four_bar: EventTermCfg | None = None


@configclass
class CoDeReorientEnvCfg(HandReorientEnvCfg):
    """In-hand cube rotation for a CoDe contact-interface variant (``hand``)."""

    scene: HandReorientSceneCfg = HandReorientSceneCfg(num_envs=128, env_spacing=0.5)
    events: CoDeReorientEventCfg = CoDeReorientEventCfg()

    def __post_init__(self):
        register_code_variants()
        self.commands.object_pose = AxisReorientCommandCfg(
            asset_name="object",
            init_pos_offset=(0.0, 0.0, -CUBE_DROP_M),
            update_goal_on_success=True,
            orientation_success_threshold=0.2,
            make_quat_unique=False,
            debug_vis=False,
            axis=GOAL_AXIS,
            max_angle=GOAL_MAX_ANGLE,
            min_step=GOAL_MIN_STEP,
        )
        super().__post_init__()
        self.terminations.object_out_of_reach.params["threshold"] = FALL_DISTANCE_M
        self.episode_length_s = EPISODE_S


@configclass
class CoDeFlatReorientEnvCfg(CoDeReorientEnvCfg):
    hand = "code_flat"


@configclass
class CoDeCrownedReorientEnvCfg(CoDeReorientEnvCfg):
    hand = "code_crowned"


__all__ = ["CoDeCrownedReorientEnvCfg", "CoDeFlatReorientEnvCfg", "CoDeReorientEnvCfg"]
