# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""In-hand rotation of a cube about world +z by the CoDe hand (O; R and U as config switches), after Hora (Qi et al.
2022, ``haozhiqi/hora``) and LEAP Hand's Isaac Lab task (``leap-hand/LEAP_Hand_Isaac_Lab``), with terms from
RotateIt (Qi et al. 2023) and PenSpin (Wang et al. 2024).

The defaults are the settings of the current best policy, the s15 run
(``logs/rsl_rl/code_rotate_cube/2026-11-24_03-21-59_rotate_cube_O_s15_crown1_pose0_late3_ft25e6_preload20``,
``report/study4/in-hand.md``), so that ``CoDe-Rotate-Cube-O`` trains it with no overrides:

- hand and grasp (:mod:`.grasp`): palm up, the thumb pad pressing a 40 mm cube onto the tilted proximal pads against the
  distal pads at 90 deg; pads with a 1 mm crown along their length; the visual asset (physics as the plain one);
- actions: relative joint targets, ``target = clamp(prev + (1/24) clip(a), q_lo, q_hi)`` (Hora), PIP in flexion;
- reset: a grasp drawn from a cache (``scripts/rotate_cube/gen_grasps.py``; s15's mixes 20 % post-turn states into the
  stable grasps, ``scripts/rotate_cube/mix_grasps.py``);
- observations: 3 frames of [joint q normalised (U(+-0.02) noise), current targets], the cube's position from the grasp
  (pad-face frame), its quaternion and angular velocity (privileged), a 4.5 s gait clock and the error to the next
  30 deg milestone;
- rewards: :class:`RewardsCfg` (each term's docstring names its source);
- terminations: the cube 55 mm off its grasp position, tilted (reset up axis z < 0.35), or 600 steps (20 s).

Environment switches, read when the config is built (before the hand asset is made, so Hydra cannot set them):

- ``CODE_ROTATE_CUBE_GRASPS``: grasp-cache directory holding ``<hand>.pt`` (default :data:`DEFAULT_GRASP_DIR`);
- ``CODE_ROTATE_CUBE_CROWN_MM`` (default 1; 0 is flat) and ``CODE_ROTATE_CUBE_CROWN_LAYOUT`` (``length_all``,
  ``distal_width``: across the width of the distal pads only, or ``dome``: that crown along the length and across the
  width of every pad, the OmniReset task's pads);
- ``CODE_ROTATE_CUBE_CONTACT``: ``default`` (unset), or ``pad13_condim4_v1`` (``code_isaaclab.contact_model``, the
  OmniReset task's contact model; for grasp caches and evaluations on its asset);
- ``CODE_ROTATE_CUBE_PHASE_PERIOD_S``: gait clock period (default 4.5; 0 drops the observation);
- ``CODE_ROTATE_CUBE_GOAL_OBS``: next-milestone observation (default 1);
- ``CODE_ROTATE_CUBE_HORA=1``: Hora's stage-1 package (:func:`apply_hora`);
- ``CODE_ROTATE_CUBE_GEN_GRASPS=1``: the grasp-generation variant (set by ``gen_grasps.py``);
- ``CODE_ROTATE_CUBE_RESET_POOL``: a reset-pool directory holding ``<hand>.pt`` (``harvest_reset_pool.py``); half the
  resets then start from its mid-rotation states (:class:`~.rotate_cube_mdp.GraspCacheReset`; unset: none);
- ``CODE_ROTATE_CUBE_SERVO_COLLIDERS``: ``1`` (default) gives the PIP servo bodies convex colliders
  (``assets.builders.write_usd``); ``0`` reproduces the asset of the runs before them (s15, model_5595);
- ``CODE_ROTATE_CUBE_POSE_DIFF``: weight of a joint-target penalty toward the grasp cache's mean pose
  (:class:`~.rotate_cube_mdp.TargetPoseDiff`, e.g. -0.1; unset or 0: none, as in s15).

Changing the crown, the clock or the goal observation changes the asset or the observation size; a checkpoint and its
grasp cache must match them.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs import mdp as base_mdp
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialBaseCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import UniformNoiseCfg as Unoise

from code_isaaclab.assets.description import Description
from code_isaaclab.contact_interface import FLAT, PadProfile
from code_isaaclab.flexion_control import flexion_joint_pos_normalized
from code_isaaclab.hands import get_hand
from code_isaaclab.hands.description_hands import register_description_hand
from code_isaaclab.loops import stiffen_env_mujoco_loops
from code_isaaclab.physics import HandPhysicsCfg, apply_hand_buffers

from . import rotate_cube_mdp as mdp
from .camera import recording_visualizer_cfg
from .grasp import CUBE_MASS_KG, CUBE_SIZE_M, FLEXION_DAMPING, FLEXION_STIFFNESS, HANDS, grasp_joint_pos, pinch_grasp

SIM_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_GRASP_DIR = SIM_ROOT / "logs" / "rotate_cube" / "goal2720_curriculum20_preload_train"
"""s15's training cache (O only): 1,024 stable grasps (``gen_grasps.py``, crown 1 mm) and 256 rows of 98 post-turn
states exported by ``eval_policy.py --export_grasps`` (``mix_grasps.py --fraction 0.2``)."""
GRASP_DIR = Path(os.environ.get("CODE_ROTATE_CUBE_GRASPS", DEFAULT_GRASP_DIR))
"""Grasp caches, ``<hand>.pt``."""
CROWN_MM = float(os.environ.get("CODE_ROTATE_CUBE_CROWN_MM", "1"))
CROWN_LAYOUT = os.environ.get("CODE_ROTATE_CUBE_CROWN_LAYOUT", "length_all")
PHASE_PERIOD_S = float(os.environ.get("CODE_ROTATE_CUBE_PHASE_PERIOD_S", "4.5"))
GOAL_OBS = os.environ.get("CODE_ROTATE_CUBE_GOAL_OBS", "1") == "1"
GEN_GRASPS = os.environ.get("CODE_ROTATE_CUBE_GEN_GRASPS", "0") == "1"
HORA = os.environ.get("CODE_ROTATE_CUBE_HORA", "0") == "1"
RESET_POOL_DIR = os.environ.get("CODE_ROTATE_CUBE_RESET_POOL")
RESET_POOL_FRACTION = 0.5
POSE_DIFF_WEIGHT = float(os.environ.get("CODE_ROTATE_CUBE_POSE_DIFF", "0"))
SERVO_COLLIDERS = os.environ.get("CODE_ROTATE_CUBE_SERVO_COLLIDERS", "1") == "1"
CONTACT = os.environ.get("CODE_ROTATE_CUBE_CONTACT", "default")

EPISODE_STEPS = 600
TAU_MAX_NM = 1.03
"""XM335 stall torque at 11.1 V (the servo effort limit)."""
DROP_DISTANCE_M = 0.055
TILT_MIN_DOT = 0.35
MILESTONE_STEP_RAD = math.pi / 6
ENVELOPE = {"max_distance": 0.04, "min_dot": 0.5}
"""The retained-grasp envelope of the milestone, timeout and regrasp terms (and of the evaluation's strict metrics)."""
GRASP_JOINT_NOISE = 0.25
GRASP_LIFT_M = 0.003
"""Grasp generation: U(+-0.25) rad on every actuated joint, the cube 3 mm above its rest."""
HORA_DR = {"mass_kg": (0.01, 0.25), "friction": (0.3, 3.0), "com_m": 0.01, "gain_scale": 0.03}
HORA_FORCE = {"scale": 2.0, "prob": 0.25, "decay": 0.9, "decay_interval": 0.08}
HORA_WORK_WEIGHT = -2.0


@configclass
class SceneCfg(InteractiveSceneCfg):
    num_envs = 128
    env_spacing = 0.5
    robot = None
    object = None
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    )


@configclass
class ActionsCfg:
    joint_pos = mdp.RelativeFlexionActionCfg(
        asset_name="robot",
        joint_names=[],
        four_bar_tables={},
        flexion_limits_deg=(0.0, 120.0),
        flexion_stiffness=FLEXION_STIFFNESS,
        flexion_damping=FLEXION_DAMPING,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """Actor and critic input, in this order (the checkpoints' layout)."""

        joint_pos = ObsTerm(
            func=flexion_joint_pos_normalized,
            noise=Unoise(n_min=-0.02, n_max=0.02),
            history_length=3,
            params={"asset_cfg": SceneEntityCfg("robot"), "four_bar_tables": {}},
        )
        joint_targets = ObsTerm(func=mdp.actuated_targets_normalized, history_length=3)
        object_pos = ObsTerm(func=mdp.object_pos_from_grasp_face, scale=10.0)
        object_quat = ObsTerm(
            func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("object"), "make_quat_unique": True}
        )
        object_ang_vel = ObsTerm(func=mdp.object_ang_vel_w, scale=0.2)
        gait_phase: ObsTerm | None = None
        """``(sin, cos)`` of the episode clock, set from ``CODE_ROTATE_CUBE_PHASE_PERIOD_S``."""
        next_rotation_goal: ObsTerm | None = None
        """The error to the next milestone, set by ``CODE_ROTATE_CUBE_GOAL_OBS``."""

        def __post_init__(self):
            self.concatenate_terms = True
            self.enable_corruption = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsCfg:
    """The order matters: ``rotation`` computes the finite-difference motion and ``milestones`` the progress that later
    terms read."""

    rotation = RewTerm(
        func=mdp.ObjectMotion,
        weight=2.5,
        params={"clip_min": -0.6, "clip_max": 0.6, "full_credit_m": 0.02, "zero_credit_m": 0.05},
    )
    """Hora's rotation reward, ``clip(omega_z)`` (Hora clips at +-0.25 rad/s with weight 1.25); its positive part fades
    linearly from full credit at 20 mm to none at 50 mm of displacement from the grasp."""
    object_lin_vel = RewTerm(func=mdp.object_lin_vel_fd_l1, weight=-0.3)
    """Hora's object linear-velocity penalty, ``|v|_1`` (x -0.3)."""
    torque = RewTerm(func=mdp.servo_torque_l2, weight=-0.1, params={"tau_max": TAU_MAX_NM, "asset_cfg": None})
    """Hora's torque penalty (x -0.1), normalised by the XM335 stall torque."""
    off_axis = RewTerm(func=mdp.off_axis_ang_vel_l1, weight=-0.15)
    """RotateIt's rotation penalty, ``|omega_x| + |omega_y|``."""
    object_distance = RewTerm(func=mdp.object_grasp_distance, weight=-4.0)
    """LEAP Isaac Lab's position term, ``|p - p_grasp|`` (LEAP uses x -10)."""
    rotate_penalty = RewTerm(func=mdp.rotate_over_speed, weight=-0.3, params={"omega_max": 0.8})
    """PenSpin's over-speed penalty, ``max(omega_z - omega_max, 0)`` (x -0.3; PenSpin's threshold 1.0 is for a pen)."""
    failure = RewTerm(func=base_mdp.is_terminated, weight=-90.0)
    """One penalty on an off-grasp or tilt termination (Isaac Lab scales the weight by the control step: -3 once)."""
    tilt_deviation = RewTerm(func=mdp.tilt_deviation, weight=-1.0, params={"free_tilt_dot": 0.94})
    """Tilt beyond about 20 deg from the reset pose, as a cosine deficit (project term)."""
    milestones = RewTerm(
        func=mdp.RotationMilestones, weight=18.0, params={"angle_step_rad": MILESTONE_STEP_RAD, **ENVELOPE}
    )
    """Each new 30 deg of progress, once, while the cube stays in the envelope (project term; 0.6 per milestone)."""
    timeout_progress = RewTerm(func=mdp.retained_turns_at_timeout, weight=150.0, params={"second_half_weight": 3.0})
    """Retained turns at a clean timeout, rotation in the episode's second half weighted x (1 + 3) (project term)."""
    joint_target_limit_regrasp = RewTerm(
        func=mdp.joint_target_limit_cost,
        weight=-0.18,
        params={
            "lower_joints": ["index_aa", "middle_aa", "thumb_ro"],
            "upper_joints": ["middle_pip_servo"],
            "free_fraction": 0.85,
            "min_progress_rad": math.pi / 2,
            "ramp_progress_rad": math.pi / 6,
            "max_distance_m": ENVELOPE["max_distance"],
            "min_up_dot": ENVELOPE["min_dot"],
        },
    )
    """Targets saturating in the directions that stall the rotation, after a quarter turn in the envelope: keeps
    command travel for the next regrasp (project term)."""


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    object_off_grasp = DoneTerm(func=mdp.object_off_grasp, params={"max_distance": DROP_DISTANCE_M})
    object_tilted = DoneTerm(func=mdp.ObjectTilted, params={"min_dot": TILT_MIN_DOT})


@configclass
class EventCfg:
    reset_grasp = EventTermCfg(func=mdp.GraspCacheReset, mode="reset", params={})
    stiffen_loops: EventTermCfg | None = None


@configclass
class CoDeRotateCubeEnvCfg(ManagerBasedRLEnvCfg):
    """See the module docstring. ``hand_key``: O, R or U; ``gen_grasps``: the grasp-generation variant (cradle + noise
    resets, no terminations but the time-out)."""

    hand_key: str = "O"
    gen_grasps: bool = GEN_GRASPS
    visuals: bool = True
    """The hand asset with drawing-only meshes (servo bodies, colours); colliders, masses and joints are the plain
    asset's."""
    pad_crown_mm: float = CROWN_MM
    pad_crown_layout: str = CROWN_LAYOUT
    phase_period_s: float = PHASE_PERIOD_S
    observe_rotation_goal: bool = GOAL_OBS
    face_origin: tuple | None = None
    face_rot: tuple | None = None
    rest_face: tuple | None = None
    grasp: dict | None = None

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
        if not math.isfinite(self.pad_crown_mm) or self.pad_crown_mm < 0.0:
            raise ValueError("CODE_ROTATE_CUBE_CROWN_MM must be finite and nonnegative")
        if self.pad_crown_layout not in ("length_all", "distal_width", "dome"):
            raise ValueError("CODE_ROTATE_CUBE_CROWN_LAYOUT must be length_all, distal_width or dome")
        if CONTACT not in ("default", "pad13_condim4_v1"):
            raise ValueError("CODE_ROTATE_CUBE_CONTACT must be default or pad13_condim4_v1")
        if not math.isfinite(self.phase_period_s) or self.phase_period_s < 0.0:
            raise ValueError("CODE_ROTATE_CUBE_PHASE_PERIOD_S must be finite and nonnegative")
        desc_path = HANDS[self.hand_key]
        desc = Description.load(desc_path)
        crown, layout = self.pad_crown_mm, self.pad_crown_layout
        distal_width = layout == "distal_width"
        profile = FLAT
        if crown and layout == "dome":  # the same profile as the OmniReset task's (same asset)
            profile = PadProfile(f"dome{crown:g}x{crown:g}", crown_mm=crown, crown_wid_mm=crown)
        elif crown:
            distal_pads = ("index_distal_pad", "middle_distal_pad", "ring_distal_pad")
            profile = PadProfile(
                f"crown{crown:g}_{layout}", crown_mm=crown, across_width=distal_width,
                pads=distal_pads if distal_width else (),
            )  # fmt: skip
        else:
            self.pad_crown_layout = "flat"
        g = pinch_grasp(desc)
        self.grasp = {k: v for k, v in g.items() if k in ("thumb_mcp_deg", "thumb_ip_deg", "rest_face", "rest_env")}
        name = register_description_hand(
            f"rotate_cube_{self.hand_key}" + (f"_{profile.key()}" if crown else "") + ("_vis" if self.visuals else "")
            + ("" if SERVO_COLLIDERS else "_noservocol"),
            desc_path,
            RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/object",
                spawn=sim_utils.CuboidCfg(
                    size=(CUBE_SIZE_M,) * 3,
                    rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
                    mass_props=sim_utils.MassCfg(mass=CUBE_MASS_KG),
                    collision_props=sim_utils.UsdPhysicsCollisionCfg(),
                    activate_contact_sensors=True,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.5, 0.2)),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=g["rest_env"], rot=g["rest_quat_xyzw"]),
            ),
            servo_saturation=(True, True),
            profile=profile,
            thumb_pad=True,
            visuals=self.visuals,
            servo_colliders=("pip",) if SERVO_COLLIDERS else (),
        )
        spec = get_hand(name)
        self.scene.robot = spec.robot_cfg.replace(
            init_state=spec.robot_cfg.init_state.replace(
                joint_pos=grasp_joint_pos(desc, g["thumb_mcp_deg"], g["thumb_ip_deg"])
            )
        )
        self.scene.object = spec.object_cfg
        self.decimation = spec.decimation
        self.sim.render_interval = self.decimation
        self.episode_length_s = EPISODE_STEPS * self.sim.dt * self.decimation
        self.face_origin, self.face_rot, self.rest_face = g["face_origin"], g["face_rot"], g["rest_face"]
        actuated, tables = list(spec.actuated_joint_names), dict(spec.four_bar_tables)
        joints = SceneEntityCfg("robot", joint_names=actuated, preserve_order=True)
        self.actions.joint_pos.joint_names = actuated
        self.actions.joint_pos.four_bar_tables = tables
        policy = self.observations.policy
        policy.joint_pos.params = {"asset_cfg": joints, "four_bar_tables": tables}
        if self.phase_period_s > 0.0:
            policy.gait_phase = ObsTerm(func=mdp.gait_phase, params={"period_s": self.phase_period_s})
        if self.observe_rotation_goal:
            policy.next_rotation_goal = ObsTerm(func=mdp.next_rotation_goal)
        self.rewards.torque.params["asset_cfg"] = joints
        self.events.reset_grasp.params = {
            "actuated": actuated,
            "four_bar_tables": tables,
            "joint_noise": GRASP_JOINT_NOISE,
            "lift_m": GRASP_LIFT_M,
            "cache": None if self.gen_grasps else str(GRASP_DIR / f"{self.hand_key}.pt"),
        }
        if POSE_DIFF_WEIGHT and not self.gen_grasps:
            self.rewards.pose_diff = RewTerm(func=mdp.TargetPoseDiff, weight=POSE_DIFF_WEIGHT)
        if RESET_POOL_DIR and not self.gen_grasps:
            self.events.reset_grasp.params["pool"] = str(Path(RESET_POOL_DIR) / f"{self.hand_key}.pt")
            self.events.reset_grasp.params["pool_fraction"] = RESET_POOL_FRACTION
        if self.gen_grasps:  # zero actions held: no terminations, nor the terms that read them or the milestones
            self.terminations.object_off_grasp = None
            self.terminations.object_tilted = None
            for term in ("failure", "tilt_deviation", "milestones", "timeout_progress", "joint_target_limit_regrasp"):
                setattr(self.rewards, term, None)
            policy.next_rotation_goal = None
        if HORA:
            apply_hora(self, actuated, joints)
        self.sim.physics = HandPhysicsCfg()
        apply_hand_buffers(self.sim.physics, spec.mjwarp_buffers)
        if CONTACT != "default":
            from code_isaaclab import contact_model

            contact_model.apply(self.sim.physics)
        if spec.stiffen_mjwarp_loops:
            self.events.stiffen_loops = EventTermCfg(func=stiffen_env_mujoco_loops, mode="startup")
        self.sim.default_visualizer_cfg = recording_visualizer_cfg(self.scene.num_envs, self.scene.env_spacing)


def apply_hora(cfg: CoDeRotateCubeEnvCfg, actuated: list[str], joints: SceneEntityCfg) -> None:
    """Hora's stage 1 (``haozhiqi/hora`` ``configs/task/AllegroHandHora.yaml`` and ``allegro_hand_hora.py``): work
    penalty -2.0; at reset, cube mass U(0.01, 0.25) kg, friction U(0.3, 3.0) on the cube's and every hand shape, cube
    centre of mass +-1 cm per axis, servo PD gains x U(0.97, 1.03) (PIP: the scheduled flexion gains, in the action
    term); a random force on the cube every step (scale 2 x mass, probability 0.25, decay 0.9 per 0.08 s); the cube's
    mass, friction, centre-of-mass offset and scale as privileged observations (6 more inputs); and, in ``agents.py``,
    horizon 8."""
    cfg.rewards.work = RewTerm(
        func=mdp.work_l2, weight=HORA_WORK_WEIGHT, params={"tau_max": TAU_MAX_NM, "asset_cfg": joints}
    )
    obj, hand = SceneEntityCfg("object"), SceneEntityCfg("robot", body_names=".*")
    cfg.events.cube_mass = EventTermCfg(
        func=base_mdp.randomize_rigid_body_mass,
        mode="reset",
        params={"asset_cfg": obj, "mass_distribution_params": HORA_DR["mass_kg"], "operation": "abs"},
    )
    for name, asset in (("cube_friction", obj), ("hand_friction", hand)):
        setattr(
            cfg.events,
            name,
            EventTermCfg(
                func=base_mdp.randomize_rigid_body_material,
                mode="reset",
                params={
                    "asset_cfg": asset,
                    "static_friction_range": HORA_DR["friction"],
                    "dynamic_friction_range": HORA_DR["friction"],
                    "restitution_range": (0.0, 0.0),
                    "num_buckets": 250,
                },
            ),
        )
    c = HORA_DR["com_m"]
    cfg.events.cube_com = EventTermCfg(
        func=base_mdp.randomize_rigid_body_com,
        mode="reset",
        params={"asset_cfg": obj, "com_range": {"x": (-c, c), "y": (-c, c), "z": (-c, c)}},
    )
    g = HORA_DR["gain_scale"]
    cfg.events.servo_gains = EventTermCfg(
        func=base_mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=[j for j in actuated if not j.endswith("_pip_servo")], preserve_order=True
            ),
            "stiffness_distribution_params": (1.0 - g, 1.0 + g),
            "damping_distribution_params": (1.0 - g, 1.0 + g),
            "operation": "scale",
        },
    )
    cfg.actions.joint_pos.gain_noise = g
    dt = cfg.sim.dt * cfg.decimation
    cfg.events.random_force = EventTermCfg(
        func=mdp.ObjectRandomForce, mode="interval", interval_range_s=(dt, dt), params=dict(HORA_FORCE)
    )
    cfg.observations.policy.object_priv = ObsTerm(func=mdp.object_physics_priv)


@configclass
class CoDeRotateCubeOEnvCfg(CoDeRotateCubeEnvCfg):
    hand_key = "O"


@configclass
class CoDeRotateCubeREnvCfg(CoDeRotateCubeEnvCfg):
    hand_key = "R"


@configclass
class CoDeRotateCubeUEnvCfg(CoDeRotateCubeEnvCfg):
    hand_key = "U"


for _key in ("R5", "U5", "R9", "U9", "RL", "RL60", "UL120", "UL", "O_thumbopt"):  # the 5 and 9 mm pairs (grasp.HANDS), for grasp caches of the transfer study
    globals()[f"CoDeRotateCube{_key}EnvCfg"] = configclass(
        type(f"CoDeRotateCube{_key}EnvCfg", (CoDeRotateCubeEnvCfg,), {"hand_key": _key})
    )


__all__ = [
    "DEFAULT_GRASP_DIR",
    "GRASP_DIR",
    "CoDeRotateCubeEnvCfg",
    "CoDeRotateCubeOEnvCfg",
    "CoDeRotateCubeREnvCfg",
    "CoDeRotateCubeUEnvCfg",
]
