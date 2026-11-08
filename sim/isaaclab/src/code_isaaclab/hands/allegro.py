# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The Allegro hand: the reference hand, taken unchanged from Isaac Lab's reorientation task.

Every value is imported from upstream (``isaaclab_assets`` and the Allegro reorientation config), so this
spec reproduces ``Isaac-Reorient-Cube-Allegro`` and serves as the template for a custom hand.
"""

from isaaclab_tasks.core.reorient.config.allegro_hand.allegro_hand_common import (
    ALLEGRO_HAND_ROBOT_CFG,
    CUBE_CFG,
    GOAL_MARKER_OFFSET,
    GOAL_OBJECT_CFG,
)
from isaaclab_tasks.core.reorient.config.allegro_hand.allegro_hand_common import (
    PhysicsCfg as _UpstreamPhysicsCfg,
)

from isaaclab_assets.robots.allegro import ALLEGRO_ACTUATED_JOINT_NAMES, ALLEGRO_FINGERTIP_BODY_NAMES

from . import HandSpec, register_hand

_mjwarp = _UpstreamPhysicsCfg().newton_mjwarp.solver_cfg

ALLEGRO = register_hand(
    HandSpec(
        name="allegro",
        robot_cfg=ALLEGRO_HAND_ROBOT_CFG,
        actuated_joint_names=ALLEGRO_ACTUATED_JOINT_NAMES,
        fingertip_body_names=ALLEGRO_FINGERTIP_BODY_NAMES,
        object_cfg=CUBE_CFG,
        goal_marker_cfg=GOAL_OBJECT_CFG,
        goal_marker_offset=GOAL_MARKER_OFFSET,
        goal_orientation_threshold=0.2,
        decimation=4,
        mjwarp_buffers={"njmax": _mjwarp.njmax, "nconmax": _mjwarp.nconmax},
    )
)
