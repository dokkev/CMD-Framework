# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""In-hand rotation of a cube about world z by the CoDe hand (O; R and U as config switches), after the working
recipes of Hora (Qi et al. 2022, ``haozhiqi/hora``) and LEAP Hand's Isaac Lab task
(``leap-hand/LEAP_Hand_Isaac_Lab``):

- hand and grasp as :mod:`code_isaaclab.tasks.reorient.o_env_cfg` (palm up, the thumb pad pressing a 40 mm cube onto
  the tilted proximal pads against the distal pads at 90 deg); the visual asset has physics identical to the plain one;
- actions: relative joint targets, ``target = clamp(prev + (1/24) clip(a), q_lo, q_hi)``, PIP in flexion (K_q schedule);
- reset: a grasp drawn from a cache made by ``scripts/rotate_cube/gen_grasps.py`` (the cradle grasp with joint noise,
  the cube dropped a few mm with a random yaw, kept if it held for 3 s);
- observations: 3 frames of [joint q normalised (U(+-0.02) noise), current targets], plus the cube's position from the
  grasp (pad-face frame), quaternion and angular velocity (privileged), PIP in flexion, no fingertip or skeleton terms;
- rewards (Hora / LEAP Sim): clip(omega_z, -0.25, 0.25) x 1.25, object |v|_1 x -0.3, pose difference x -0.1, normalised
  torque^2 x -0.1; no work, fall or goal terms;
- terminations: the cube 2.5 cm off its grasp position, tilted (up axis z < 0.5), or 600 steps (20 s).
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
from code_isaaclab.tasks.reorient.o_env_cfg import (
    CUBE_MASS_KG,
    CUBE_SIZE_M,
    FLEXION_DAMPING,
    FLEXION_STIFFNESS,
    grasp_joint_pos,
    pad_points,
    pinch_grasp,
)
from code_isaaclab.tasks.rotate.env_cfg import HANDS

from . import rotate_cube_mdp as mdp

SIM_ROOT = Path(__file__).resolve().parents[4]
GRASP_DIR = Path(os.environ.get("CODE_ROTATE_CUBE_GRASPS", SIM_ROOT / "logs" / "rotate_cube" / "grasps"))
"""Grasp caches, ``<hand>.pt`` (``scripts/rotate_cube/gen_grasps.py``)."""
EPISODE_STEPS = 600
OMEGA_CLIP = (-0.25, 0.25)
REWARD_WEIGHTS = {"rotation": 1.25, "object_lin_vel": -0.3, "pose_diff": -0.1, "torque": -0.1}
TAU_MAX_NM = 1.03
"""XM335 stall torque at 11.1 V (the servo effort limit)."""
DROP_DISTANCE_M = 0.025
TILT_MIN_DOT = 0.5
GRASP_JOINT_NOISE = 0.25
GRASP_LIFT_M = 0.003
HORA = os.environ.get("CODE_ROTATE_CUBE_HORA", "0") == "1"
"""Hora's stage-1 package (``CODE_ROTATE_CUBE_HORA=1``, :meth:`CoDeRotateCubeEnvCfg.apply_hora`): the work penalty,
domain randomisation at reset, random object forces, privileged physical properties in the observation (and, in
``agents.py``, horizon 8)."""
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
        object_ang_vel = ObsTerm(func=mdp.object_omega_fd, scale=0.2)
        gait_phase: ObsTerm | None = None
        next_rotation_goal: ObsTerm | None = None

        def __post_init__(self):
            self.concatenate_terms = True
            self.enable_corruption = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsCfg:
    rotation = RewTerm(
        func=mdp.ObjectMotion,
        weight=REWARD_WEIGHTS["rotation"],
        params={"clip_min": OMEGA_CLIP[0], "clip_max": OMEGA_CLIP[1], "full_credit_m": 0.0, "zero_credit_m": 0.0},
    )
    """Positive rotation credit can fade linearly with displacement. Zero for both distances disables the fade."""
    object_lin_vel = RewTerm(func=mdp.object_lin_vel_fd_l1, weight=REWARD_WEIGHTS["object_lin_vel"])
    pose_diff = RewTerm(
        func=mdp.pose_diff_l2, weight=REWARD_WEIGHTS["pose_diff"], params={"asset_cfg": SceneEntityCfg("robot")}
    )
    torque = RewTerm(
        func=mdp.servo_torque_l2,
        weight=REWARD_WEIGHTS["torque"],
        params={"tau_max": TAU_MAX_NM, "asset_cfg": SceneEntityCfg("robot")},
    )
    off_axis = RewTerm(func=mdp.off_axis_ang_vel_l1, weight=-1.0e-9)
    """RotateIt's rotation penalty, weight set by the ``off_axis_ramp`` curriculum (a nonzero placeholder keeps the term
    registered: the manager drops zero-weight terms)."""
    object_distance = RewTerm(func=mdp.object_grasp_distance, weight=0.0)
    """LEAP Isaac Lab's position term, off by default; ``env.rewards.object_distance.weight=-10``."""
    rotate_penalty = RewTerm(func=mdp.rotate_over_speed, weight=0.0, params={"omega_max": 0.5})
    """PenSpin's over-speed penalty, off by default (weight 0); switch on with
    ``env.rewards.rotate_penalty.weight=-0.3`` (omega_max 0.5 rad/s: PenSpin's 1.0 is for a pen)."""
    failure = RewTerm(func=base_mdp.is_terminated, weight=0.0)
    """One penalty on off-grasp or tilt termination. Isaac Lab scales the weight by the control timestep."""
    tilt_deviation = RewTerm(func=mdp.tilt_deviation, weight=0.0, params={"free_tilt_dot": 0.94})
    """Absolute tilt penalty beyond about 20 degrees from the reset pose."""
    milestones = RewTerm(
        func=mdp.RotationMilestones,
        weight=0.0,
        params={"angle_step_rad": 0.5235987756, "max_distance": 0.04, "min_dot": 0.5},
    )
    """Credit each 30-degree progress milestone once, while the cube is within the grasp envelope."""
    timeout_progress = RewTerm(func=mdp.retained_turns_at_timeout, weight=0.0, params={"second_half_weight": 0.0})
    """Credit net turns at a clean timeout if all steps stayed within the milestone bounds."""
    joint_target_limit = RewTerm(
        func=mdp.joint_target_limit_cost,
        weight=0.0,
        params={"lower_joint": "middle_aa", "upper_joint": "middle_pip_servo", "free_fraction": 0.85},
    )
    """Preserve command travel in the two directions most often saturated during stalls."""
    joint_target_limit_regrasp = RewTerm(
        func=mdp.joint_target_limit_cost,
        weight=0.0,
        params={
            "lower_joints": ["index_aa", "middle_aa", "thumb_ro"],
            "upper_joints": ["middle_pip_servo"],
            "free_fraction": 0.85,
            "min_progress_rad": 0.0,
            "ramp_progress_rad": 0.0,
            "max_distance_m": 0.0,
            "min_up_dot": -1.0,
        },
    )
    """After initial rotation progress, encourage saturated fingers to recover command travel for the next regrasp."""


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    object_off_grasp = DoneTerm(func=mdp.object_off_grasp, params={"max_distance": DROP_DISTANCE_M})
    object_tilted = DoneTerm(func=mdp.ObjectTilted, params={"min_dot": TILT_MIN_DOT})


@configclass
class CurriculumCfg:
    off_axis_ramp = None
    """``CurrTerm(func=mdp.ramp_reward_weight, params={"term_name": "off_axis", "weight": -0.1, "num_steps": 4800})``
    when enabled (``CODE_ROTATE_CUBE_OFF_AXIS_RAMP=<weight>,<steps>``)."""


@configclass
class EventCfg:
    reset_grasp = EventTermCfg(func=mdp.GraspCacheReset, mode="reset", params={})
    stiffen_loops: EventTermCfg | None = None


@configclass
class CoDeRotateCubeEnvCfg(ManagerBasedRLEnvCfg):
    """See the module docstring. ``gen_grasps``: build the grasp-generation variant (cradle + noise resets, no
    terminations but the time-out); ``hand_key``: O, R or U."""

    hand_key: str = "O"
    gen_grasps: bool = os.environ.get("CODE_ROTATE_CUBE_GEN_GRASPS", "0") == "1"
    visuals: bool = os.environ.get("CODE_REORIENT_O_VISUALS", "1") == "1"
    pad_crown_mm: float = 0.0
    """Authored pad sagitta [mm], read from `CODE_ROTATE_CUBE_CROWN_MM` before USD asset creation."""
    pad_crown_layout: str = "flat"
    """Authored crown axis and pad selection."""
    phase_period_s: float = 0.0
    """Period of the optional two-component gait phase observation, in seconds."""
    observe_rotation_goal: bool = False
    face_origin: tuple | None = None
    face_rot: tuple | None = None
    rest_face: tuple | None = None
    distal_face_points: tuple | None = None
    grasp: dict | None = None

    scene: SceneCfg = SceneCfg()
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()
    commands = None
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120, physics_material=RigidBodyMaterialBaseCfg(static_friction=1.0, dynamic_friction=1.0)
    )

    def __post_init__(self):
        desc_path = HANDS[self.hand_key]
        desc = Description.load(desc_path)
        crown_mm = float(os.environ.get("CODE_ROTATE_CUBE_CROWN_MM", "0"))
        if not math.isfinite(crown_mm) or crown_mm < 0.0:
            raise ValueError("CODE_ROTATE_CUBE_CROWN_MM must be finite and nonnegative")
        layout = os.environ.get("CODE_ROTATE_CUBE_CROWN_LAYOUT", "length_all")
        if layout not in ("length_all", "distal_width"):
            raise ValueError("CODE_ROTATE_CUBE_CROWN_LAYOUT must be length_all or distal_width")
        self.pad_crown_mm = crown_mm
        self.pad_crown_layout = layout if crown_mm else "flat"
        distal_width = layout == "distal_width"
        distal_pads = ("index_distal_pad", "middle_distal_pad", "ring_distal_pad")
        profile = (
            FLAT
            if crown_mm == 0.0
            else PadProfile(
                f"crown{crown_mm:g}_{layout}",
                crown_mm=crown_mm,
                across_width=distal_width,
                pads=distal_pads if distal_width else (),
            )
        )
        g = pinch_grasp(desc)
        self.grasp = {k: v for k, v in g.items() if k in ("thumb_mcp_deg", "thumb_ip_deg", "rest_face", "rest_env")}
        name = register_description_hand(
            f"rotate_cube_{self.hand_key}"
            + (f"_{profile.key()}" if crown_mm else "")
            + ("_vis" if self.visuals else ""),
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
        self.distal_face_points = pad_points(desc)
        actuated, tables = list(spec.actuated_joint_names), dict(spec.four_bar_tables)
        self.actions.joint_pos.joint_names = actuated
        self.actions.joint_pos.four_bar_tables = tables
        joints = SceneEntityCfg("robot", joint_names=actuated, preserve_order=True)
        self.observations.policy.joint_pos.params = {"asset_cfg": joints, "four_bar_tables": tables}
        self.phase_period_s = float(os.environ.get("CODE_ROTATE_CUBE_PHASE_PERIOD_S", "0"))
        if not math.isfinite(self.phase_period_s) or self.phase_period_s < 0.0:
            raise ValueError("CODE_ROTATE_CUBE_PHASE_PERIOD_S must be finite and nonnegative")
        if self.phase_period_s > 0.0:
            self.observations.policy.gait_phase = ObsTerm(
                func=mdp.gait_phase,
                params={"period_s": self.phase_period_s},
            )
        self.observe_rotation_goal = os.environ.get("CODE_ROTATE_CUBE_GOAL_OBS", "0") == "1"
        if self.observe_rotation_goal:
            self.rewards.milestones.weight = 18.0
            self.observations.policy.next_rotation_goal = ObsTerm(func=mdp.next_rotation_goal)
        self.rewards.pose_diff.params["asset_cfg"] = joints
        self.rewards.torque.params["asset_cfg"] = joints
        cache = GRASP_DIR / f"{self.hand_key}.pt"
        self.events.reset_grasp.params = {
            "actuated": actuated,
            "four_bar_tables": tables,
            "joint_noise": GRASP_JOINT_NOISE,
            "lift_m": GRASP_LIFT_M,
            "cache": None if self.gen_grasps else str(cache),
        }
        if self.gen_grasps:
            self.terminations.object_off_grasp = None
            self.terminations.object_tilted = None
        if HORA:
            self.apply_hora(actuated, joints)
        ramp = os.environ.get("CODE_ROTATE_CUBE_OFF_AXIS_RAMP")
        if ramp:
            from isaaclab.managers import CurriculumTermCfg

            w, steps = ramp.split(",")
            self.curriculum.off_axis_ramp = CurriculumTermCfg(
                func=mdp.ramp_reward_weight,
                params={"term_name": "off_axis", "weight": float(w), "num_steps": int(steps)},
            )
        self.sim.physics = HandPhysicsCfg()
        apply_hand_buffers(self.sim.physics, spec.mjwarp_buffers)
        if spec.stiffen_mjwarp_loops:
            self.events.stiffen_loops = EventTermCfg(func=stiffen_env_mujoco_loops, mode="startup")
        from code_isaaclab.tasks.reorient.o_env_cfg import recording_visualizer_cfg

        self.sim.default_visualizer_cfg = recording_visualizer_cfg(self.scene.num_envs, self.scene.env_spacing)


def _apply_hora(self, actuated: list[str], joints: SceneEntityCfg) -> None:
    """Hora's stage 1 (``haozhiqi/hora`` ``configs/task/AllegroHandHora.yaml`` and ``allegro_hand_hora.py``):
    work penalty -2.0; at reset, cube mass U(0.01, 0.25) kg, friction U(0.3, 3.0) on the cube's and every hand shape,
    cube centre of mass +-1 cm per axis, servo PD gains x U(0.97, 1.03) (PIP: the scheduled flexion gains, in the action
    term); a random force on the cube every step (scale 2 x mass, probability 0.25, decay 0.9 per 0.08 s); the cube's
    mass, friction, centre-of-mass offset and scale as privileged observations."""
    from isaaclab.managers import EventTermCfg as Ev

    self.rewards.work = RewTerm(
        func=mdp.work_l2, weight=HORA_WORK_WEIGHT, params={"tau_max": TAU_MAX_NM, "asset_cfg": joints}
    )
    obj, hand = SceneEntityCfg("object"), SceneEntityCfg("robot", body_names=".*")
    self.events.cube_mass = Ev(
        func=base_mdp.randomize_rigid_body_mass,
        mode="reset",
        params={"asset_cfg": obj, "mass_distribution_params": HORA_DR["mass_kg"], "operation": "abs"},
    )
    for name, asset in (("cube_friction", obj), ("hand_friction", hand)):
        setattr(
            self.events,
            name,
            Ev(
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
    self.events.cube_com = Ev(
        func=base_mdp.randomize_rigid_body_com,
        mode="reset",
        params={"asset_cfg": obj, "com_range": {"x": (-c, c), "y": (-c, c), "z": (-c, c)}},
    )
    g = HORA_DR["gain_scale"]
    self.events.servo_gains = Ev(
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
    self.actions.joint_pos.gain_noise = g
    dt = self.sim.dt * self.decimation
    self.events.random_force = Ev(
        func=mdp.ObjectRandomForce, mode="interval", interval_range_s=(dt, dt), params=dict(HORA_FORCE)
    )
    self.observations.policy.object_priv = ObsTerm(func=mdp.object_physics_priv)


CoDeRotateCubeEnvCfg.apply_hora = _apply_hora


@configclass
class CoDeRotateCubeOEnvCfg(CoDeRotateCubeEnvCfg):
    hand_key = "O"


@configclass
class CoDeRotateCubeREnvCfg(CoDeRotateCubeEnvCfg):
    hand_key = "R"


@configclass
class CoDeRotateCubeUEnvCfg(CoDeRotateCubeEnvCfg):
    hand_key = "U"


__all__ = [
    "CoDeRotateCubeEnvCfg",
    "CoDeRotateCubeOEnvCfg",
    "CoDeRotateCubeREnvCfg",
    "CoDeRotateCubeUEnvCfg",
    "GRASP_DIR",
]
