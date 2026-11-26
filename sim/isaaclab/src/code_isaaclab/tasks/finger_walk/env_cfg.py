# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Finger walk (attempt 7b', ``docs/experimental/rl_pip_only.md``): the palm-up hand with the MCPs, abductions and the
thumb held; the index, middle and ring PIPs (flexion 0-120 deg) roll a cylinder lying across the proximal pads toward
the MCPs and back out to alternating targets. A distal pad pressing on the cylinder's top rolls it toward the MCP when
the finger flexes and toward the tips when it extends; one stroke moves it a limited distance, so the fingers lift,
return and press again (gaiting). With the MCPs locked the fingertips cannot reach the palm itself: "the palm" here is
the proximal pad face.

Face frame (``grasp.face_frame`` at MCP 0, each hand's own pad face): x along the face toward the tips, y across, z
the outward normal. The cylinder starts at rest on the pads at x uniform in :data:`START_X_M`; targets alternate between
:data:`TARGETS_M` (the first one drawn per episode)."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs import mdp as base_mdp
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialBaseCfg
from isaaclab.utils import configclass

from code_isaaclab import contact_model
from code_isaaclab.assets.description import Description
from code_isaaclab.contact_interface import PadProfile
from code_isaaclab.flexion_control import FlexionEMAActionCfg, flexion_joint_pos_normalized
from code_isaaclab.hands import get_hand
from code_isaaclab.hands.description_hands import register_description_hand
from code_isaaclab.loops import stiffen_env_mujoco_loops
from code_isaaclab.physics import HandPhysicsCfg, apply_hand_buffers

from ..finger_relay.env_cfg import TOUCH, default_joint_pos
from ..rotate_cube.camera import recording_visualizer_cfg
from ..rotate_cube.grasp import FLEXION_DAMPING, FLEXION_STIFFNESS, HANDS, _matrix_to_quat_xyzw, face_frame
from . import mdp

FINGERS = ("index", "middle", "ring")
ACTING = tuple(f"{f}_pip_servo" for f in FINGERS)
RADIUS_M, LENGTH_M, MASS_KG = 0.015, 0.070, 0.030
START_X_M = (0.028, 0.042)
TARGETS_M = (0.020, 0.045)
SUCCESS_M, HOLD_STEPS = 0.004, 3
X_MAX_M, Z_MIN_M, Y_MAX_M = 0.090, -0.010, 0.040
EPISODE_S = 10.0
START_MAX_DEG = 25


@configclass
class SceneCfg(InteractiveSceneCfg):
    num_envs: int = 4096
    env_spacing: float = 0.5
    robot: ArticulationCfg = None
    object: RigidObjectCfg = None
    tip_touch: ContactSensorCfg = None


@configclass
class ActionsCfg:
    joint_pos = FlexionEMAActionCfg(asset_name="robot", joint_names=list(ACTING), four_bar_tables={}, alpha=0.5,
                                    flexion_limits_deg=(0.0, 120.0), flexion_stiffness=FLEXION_STIFFNESS,
                                    flexion_damping=FLEXION_DAMPING)  # fmt: skip


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        prev_actions = ObsTerm(func=base_mdp.last_action)
        pip = ObsTerm(func=flexion_joint_pos_normalized, params={})
        cyl = ObsTerm(func=mdp.walk_obs)
        touch = ObsTerm(func=mdp.tip_touch, params=dict(TOUCH))

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 5

    @configclass
    class CriticCfg(ObsGroup):
        prev_actions = ObsTerm(func=base_mdp.last_action)
        pip = ObsTerm(func=flexion_joint_pos_normalized, params={})
        cyl = ObsTerm(func=mdp.walk_obs)
        priv = ObsTerm(func=mdp.walk_priv)
        touch = ObsTerm(func=mdp.tip_touch, params={**TOUCH, "force_noise": 0.0, "flip_prob": 0.0})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 1

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class RewardsCfg:
    walk_goal = RewTerm(func=mdp.WalkGoal, weight=1.0, params={})
    progress = RewTerm(func=mdp.walk_progress, weight=3000.0, params={"clip_m": 0.01})
    """x step_dt: 100 per metre walked toward the target (1 per cm)."""
    bonus = RewTerm(func=mdp.goal_bonus, weight=2.0)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    off = RewTerm(func=base_mdp.is_terminated, weight=-60.0)
    """-2 per episode that ends early (x step_dt): the cylinder off the fingers."""


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    cylinder_off = DoneTerm(func=mdp.cylinder_off, params={"x_max_m": X_MAX_M, "z_min_m": Z_MIN_M, "y_max_m": Y_MAX_M})
    non_finite = DoneTerm(func=mdp.NonFinite)


@configclass
class EventCfg:
    reset = EventTermCfg(func=mdp.reset_hand_and_cylinder, mode="reset", params={})


@configclass
class FingerWalkEnvCfg(ManagerBasedRLEnvCfg):
    hand_key: str = "O"
    object_observed: bool = True
    """False: the actor sees no cylinder state (touch, joints and actions only; the critic keeps it)."""
    evaluation: bool = False
    contact_model: dict | None = None

    scene: SceneCfg = SceneCfg()
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    commands = None
    curriculum = None
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120, physics_material=RigidBodyMaterialBaseCfg(static_friction=1.0, dynamic_friction=1.0)
    )

    def __post_init__(self):
        desc_path = HANDS[self.hand_key]
        desc = Description.load(desc_path)
        profile = PadProfile("dome1x1", crown_mm=1.0, crown_wid_mm=1.0)
        o, r = face_frame(desc, 0.0)
        rot = np.asarray(r).reshape(3, 3)
        z0 = RADIUS_M + 0.0015  # dropped onto the domed pads
        start = np.asarray(o) + rot @ np.array([sum(START_X_M) / 2, 0.0, z0])
        obj = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/object",
            spawn=sim_utils.CylinderCfg(
                radius=RADIUS_M, height=LENGTH_M, axis="Y", rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
                mass_props=sim_utils.MassCfg(mass=MASS_KG), collision_props=sim_utils.UsdPhysicsCollisionCfg(),
                activate_contact_sensors=True, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.6, 0.8)),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(float(v) for v in start), rot=_matrix_to_quat_xyzw(rot)),
        )  # fmt: skip
        name = register_description_hand(f"finger_walk_{self.hand_key}_{profile.key()}", desc_path, obj,
                                         servo_saturation=(True, True), profile=profile, thumb_pad=True,
                                         visuals=False, servo_colliders=("pip",))  # fmt: skip
        spec = get_hand(name)
        self.scene.robot = spec.robot_cfg.replace(
            init_state=spec.robot_cfg.init_state.replace(joint_pos=default_joint_pos(desc))
        )
        self.scene.object = obj
        self.scene.tip_touch = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/object",
            filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{f}_distal" for f in FINGERS],
        )
        face = {"face_origin": tuple(float(v) for v in o), "face_rot": tuple(float(v) for v in rot.reshape(-1))}
        self.rewards.walk_goal.params = {**face, "targets_m": TARGETS_M, "success_m": SUCCESS_M,
                                         "hold_steps": HOLD_STEPS}  # fmt: skip
        self.events.reset.params = {
            "robot_cfg": SceneEntityCfg("robot"), **face, "x_range_m": START_X_M, "z_m": z0,
            "start_poses": {
                f: {"joints": [f"{f}_pip_servo", f"{f}_a", f"{f}_b"],
                    "rad": [[math.radians(desc.closed_pose(f, float(q))[j]) for j in (f"{f}_pip_servo", f"{f}_a", f"{f}_b")]
                            for q in range(0, START_MAX_DEG + 1)]}
                for f in FINGERS
            },
        }  # fmt: skip
        tables = dict(spec.four_bar_tables)
        self.actions.joint_pos.four_bar_tables = tables
        joints = SceneEntityCfg("robot", joint_names=list(ACTING), preserve_order=True)
        for group in (self.observations.policy, self.observations.critic):
            group.pip.params = {"asset_cfg": joints, "four_bar_tables": tables}
        if not self.object_observed:
            self.observations.policy.cyl = None
        self.decimation = spec.decimation
        self.sim.render_interval = self.decimation
        self.episode_length_s = EPISODE_S
        self.sim.physics = HandPhysicsCfg()
        apply_hand_buffers(self.sim.physics, spec.mjwarp_buffers)
        contact_model.apply(self.sim.physics)
        self.contact_model = contact_model.describe()
        if spec.stiffen_mjwarp_loops:
            self.events.stiffen_loops = EventTermCfg(func=stiffen_env_mujoco_loops, mode="startup")
        self.sim.default_visualizer_cfg = recording_visualizer_cfg(self.scene.num_envs, self.scene.env_spacing)


@configclass
class FingerWalkOEnvCfg(FingerWalkEnvCfg):
    hand_key = "O"


@configclass
class FingerWalkOTouchEnvCfg(FingerWalkEnvCfg):
    hand_key = "O"
    object_observed = False


for _key in ("O", "R", "U", "R5", "U5", "R9", "U9"):
    for _v, _obs in (("", True), ("Touch", False)):
        _n = f"FingerWalk{_key}{_v}EvalEnvCfg"
        globals()[_n] = configclass(type(_n, (FingerWalkEnvCfg,), {"hand_key": _key, "object_observed": _obs,
                                                                    "evaluation": True}))
