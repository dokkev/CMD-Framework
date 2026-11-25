# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""In-hand rotation of a 40 mm cube about world +z by the CoDe hand O, trained with OmniReset's recipe only (Yin et
al., "Emergent Dexterity via Diverse Resets and Large-Scale Reinforcement Learning", ICLR 2026; ``UW-Lab/UWLab``).
The recipe, the file references and the mapping are in ``docs/experimental/omnireset_recipe.md``; nothing of the
rotate_cube task's tuning is used (its hand, grasp geometry, joint impedance and relative action are).

- goal: a moving yaw goal: the cube on its rest pose on the pads, turned ``k * 90 deg`` from the start; it advances
  after the cube has been at the goal for 0.5 s (:class:`~.mdp.GoalContext`);
- reset: OmniReset's four reset types, from cached datasets (``scripts/rotate_cube/gen_omnireset_resets.py``),
  0.25 each (:class:`~.mdp.OmniResetStates`);
- rewards: OmniReset's (:class:`RewardsCfg`), weights as theirs;
- observations: OmniReset's actor (last action, joint positions, the object's pose and its pose relative to the goal;
  no history, no noise) and, with ``CODE_OMNIRESET_ASYM_CRITIC=1``, its privileged critic (defined but switched off in
  their configuration);
- terminations: time-out (16 s), abnormal joint speed, a non-finite (diverged) state, and the cube off the hand;
- contact model: ``code_isaaclab.contact_model`` (pad mu 1.3, condim 4, impratio 1000, stiffer solimp/solref);
- domain randomisation: OmniReset's stage-1 object mass and actuator gains (its friction ranges would override the
  contact model's pads);
- PPO: ``agents.py``, OmniReset's ``Base_PPORunnerCfg``.

The pads are domed (``CODE_OMNIRESET_CROWN_LEN_MM``, ``CODE_OMNIRESET_CROWN_WID_MM``, default 1 mm each; thumb pad
too); the PIP servo bodies collide (the builder's default). ``CoDe-Rotate-Cube-O-OmniReset-Eval`` is the same policy
interface on held-out grasps (``CODE_ROTATE_CUBE_GRASPS``, default ``heldout_omnireset_v1``: ``gen_grasps.py``
seed 76000 on this asset and contact model), 20 s, no randomisation, the rotate_cube terminations, for
``scripts/rotate_cube/eval_policy.py``.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np

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
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialBaseCfg
from isaaclab.utils import configclass

from code_isaaclab import contact_model
from code_isaaclab.assets.description import Description
from code_isaaclab.contact_interface import PadProfile
from code_isaaclab.flexion_control import flexion_joint_pos_normalized, flexion_joint_vel
from code_isaaclab.hands import get_hand
from code_isaaclab.hands.description_hands import register_description_hand
from code_isaaclab.loops import stiffen_env_mujoco_loops
from code_isaaclab.physics import HandPhysicsCfg, apply_hand_buffers

from ..rotate_cube import rotate_cube_mdp as rc_mdp
from ..rotate_cube.camera import recording_visualizer_cfg
from ..rotate_cube.grasp import (
    CUBE_MASS_KG,
    CUBE_SIZE_M,
    FLEXION_DAMPING,
    FLEXION_STIFFNESS,
    HANDS,
    grasp_joint_pos,
    pinch_grasp,
)
from . import mdp

SIM_ROOT = Path(__file__).resolve().parents[4]
CROWN_LEN_MM = float(os.environ.get("CODE_OMNIRESET_CROWN_LEN_MM", "1"))
CROWN_WID_MM = float(os.environ.get("CODE_OMNIRESET_CROWN_WID_MM", "1"))
ASYM_CRITIC = os.environ.get("CODE_OMNIRESET_ASYM_CRITIC", "0") == "1"
ABNORMAL_FACTOR = float(os.environ.get("CODE_OMNIRESET_ABNORMAL_FACTOR", "2"))
GOAL_BONUS = float(os.environ.get("CODE_OMNIRESET_GOAL_BONUS", "0"))
TILT_PENALTY = float(os.environ.get("CODE_OMNIRESET_TILT_PENALTY", "0"))
"""Weight (negative) of ``mdp.upright_tilt_excess`` beyond a 15 deg free band (0: none). v2d: -0.1."""
"""A one-time reward per goal reached (``mdp.goal_advance_bonus``; 0: none, as OmniReset). v2c: 5.0."""
"""The abnormal-speed termination's multiple of the servo velocity limit (5.55 rad/s): OmniReset's 2 by default. The
implicit servo model's velocity limit is not enforced by MuJoCo-Warp, so 2x (11 rad/s) ends episodes on ordinary fast
moves; ``CODE_OMNIRESET_ABNORMAL_FACTOR=10`` (55 rad/s) keeps it to divergence (the from-scratch run, segment 3 on,
``docs/experimental/omnireset_recipe.md``)."""
HELD_OUT = Path(os.environ.get("CODE_ROTATE_CUBE_GRASPS", SIM_ROOT / "logs" / "rotate_cube" / "heldout_omnireset_v1"))

GOAL_STEP_RAD = math.pi / 2
SUCCESS = {"success_distance_m": 0.02, "success_orientation_rad": 0.2, "hold_steps": 15}
"""Success: within 20 mm of the rest position and 0.2 rad (yaw error + tilt) of the goal, held 15 control steps
(0.5 s) before the goal advances. OmniReset reads its thresholds from each receptive object's metadata (not in the
repository); these are this task's."""
REACH_STD_M = 0.1
"""OmniReset's reach term uses std 1.0 m for an arm whose gripper starts 0.3-0.7 m from the object; the hand's
fingertips start within about 0.1 m, so std scales by 0.1 (with 1.0 the term would be constant)."""
OUT_OF_REACH_M = 0.08
TRAIN_EPISODE_S = 16.0
EVAL_EPISODE_STEPS = 600
GEN = {
    "drop_xy_m": 0.015, "drop_lift_m": (0.005, 0.025),
    "grasp_xy_m": 0.008, "grasp_lift_m": (0.0, 0.002), "grasp_tilt_rad": math.radians(10.0),
    "window_rad": {"aa": math.radians(15.0), "fe": math.radians(25.0), "pip_servo": math.radians(30.0),
                   "ro": math.radians(40.0), "raa": math.radians(15.0), "mcp": math.radians(15.0),
                   "ip": math.radians(15.0)},
    "lift_prob": 0.25, "close_deg": 10.0, "midturn_lift_deg": 20.0, "midturn_goal_rad": (math.radians(10.0),
                                                                                       math.radians(60.0)),
    "near_open_deg": 20.0, "goal_jitter_deg": 2.0,
    "goal_inside_rad": 0.3, "goal_outside_rad": 0.8,
}  # fmt: skip
"""Generation ranges (``gen_omnireset_resets.py`` documents their sources); ``open_sign`` is set per joint."""
OPEN_SIGN = {"fe": -1.0, "pip_servo": -1.0, "thumb_mcp": -1.0, "thumb_ip": -1.0}
"""Opening directions: finger flexion decreases; the thumb's MCP and IP lower its pad onto the cube when they increase
(``grasp.thumb_pad_pose``)."""


def dataset_dir(hand_key: str, profile: PadProfile) -> Path:
    """Where the reset datasets of a hand and pad profile live (``CODE_OMNIRESET_DATASETS`` overrides)."""
    default = SIM_ROOT / "logs" / "rotate_cube" / "omnireset" / f"{hand_key}_{profile.key()}"
    return Path(os.environ.get("CODE_OMNIRESET_DATASETS", default))


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
    joint_pos = mdp.OmniResetFlexionActionCfg(
        asset_name="robot", joint_names=[], four_bar_tables={}, flexion_limits_deg=(0.0, 120.0),
        flexion_stiffness=FLEXION_STIFFNESS, flexion_damping=FLEXION_DAMPING, gain_log_range=(0.5, 2.0),
    )  # fmt: skip


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """OmniReset's actor input (``rl_state_cfg.py`` ``PolicyCfg``): no history, no noise terms."""

        prev_actions = ObsTerm(func=base_mdp.last_action)
        joint_pos = ObsTerm(func=flexion_joint_pos_normalized, params={})
        cube_pose = ObsTerm(func=mdp.cube_pose, params={})
        cube_goal_error = ObsTerm(func=mdp.cube_goal_error, params={})
        joint_targets: ObsTerm | None = None
        """The commanded joint targets (v2)."""

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
            self.history_length = 0

    @configclass
    class CriticCfg(PolicyCfg):
        """OmniReset's privileged critic (``CriticCfg``): the actor's input plus time left, joint and object
        velocities and the randomised physics (cube mass and friction; the gains are not observable here)."""

        time_left = ObsTerm(func=mdp.time_left)
        joint_vel = ObsTerm(func=flexion_joint_vel, params={})
        cube_lin_vel = ObsTerm(func=base_mdp.root_lin_vel_w, params={"asset_cfg": SceneEntityCfg("object")})
        cube_ang_vel = ObsTerm(func=base_mdp.root_ang_vel_w, params={"asset_cfg": SceneEntityCfg("object")})
        cube_physics = ObsTerm(func=rc_mdp.object_physics_priv)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 1

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg | None = None


@configclass
class RewardsCfg:
    """OmniReset's ``RewardsCfg`` with its weights; ``goal_context`` first (it computes what the others read)."""

    goal_context = RewTerm(func=mdp.GoalContext, weight=0.1, params={})
    action_magnitude = RewTerm(func=mdp.action_l2_clamped, weight=-1e-4)
    action_rate = RewTerm(func=mdp.action_rate_l2_clamped, weight=-1e-3)
    joint_vel = RewTerm(func=mdp.joint_vel_l2_clamped, weight=-1e-2, params={})
    abnormal_robot = RewTerm(func=mdp.abnormal_robot_state, weight=-100.0)
    reach = RewTerm(func=mdp.fingertip_cube_distance_tanh, weight=0.1, params={"std": REACH_STD_M})
    dense_success_reward = RewTerm(func=mdp.dense_success_reward, weight=0.1, params={"std": 1.0})
    success_reward = RewTerm(func=mdp.success_reward, weight=1.0)


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    abnormal_robot = DoneTerm(func=mdp.joint_speed_abnormal, params={"factor": ABNORMAL_FACTOR})
    non_finite_state = DoneTerm(func=mdp.NonFiniteState)
    cube_out_of_reach = DoneTerm(func=mdp.cube_out_of_reach, params={"max_distance": OUT_OF_REACH_M})


@configclass
class EventCfg:
    """OmniReset's ``TrainEventCfg``: the four-way reset and its stage-1 randomisation (``BaseEventCfg``)."""

    reset_grasp = EventTermCfg(func=mdp.OmniResetStates, mode="reset", params={})
    # OmniReset's friction ranges (robot 0.3-1.2, object 1.0-2.0) are left out: under MuJoCo's larger-coefficient rule
    # they would replace the contact model's pad 1.3 / cube and links 0.5 (``code_isaaclab.contact_model``).
    cube_mass = EventTermCfg(
        func=base_mdp.randomize_rigid_body_mass, mode="startup",
        params={"asset_cfg": SceneEntityCfg("object"), "mass_distribution_params": (0.02, 0.2), "operation": "abs"},
    )  # fmt: skip
    servo_gains = EventTermCfg(
        func=base_mdp.randomize_actuator_gains, mode="reset",
        params={"asset_cfg": None, "stiffness_distribution_params": (0.5, 2.0),
                "damping_distribution_params": (0.5, 2.0), "operation": "scale", "distribution": "log_uniform"},
    )  # fmt: skip
    stiffen_loops: EventTermCfg | None = None


@configclass
class CoDeRotateCubeOmniResetEnvCfg(ManagerBasedRLEnvCfg):
    """See the module docstring."""

    hand_key: str = "O"
    evaluation: bool = False
    """The held-out evaluation variant (one grasp cache, 20 s, no randomisation, rotate_cube's terminations)."""
    visuals: bool = True
    pad_crown_mm: float = CROWN_LEN_MM
    pad_crown_wid_mm: float = CROWN_WID_MM
    pad_crown_layout: str = "dome"
    phase_period_s: float = 0.0
    observe_rotation_goal: bool = True
    face_origin: tuple | None = None
    face_rot: tuple | None = None
    rest_face: tuple | None = None
    grasp: dict | None = None
    reset_datasets: str = ""
    contact_model: dict | None = None
    """The hand-cube contact model (``code_isaaclab.contact_model.describe()``), recorded in the run's params."""
    version: str = "v1"
    goal_step_rad: float = GOAL_STEP_RAD
    success: dict | None = None
    """``GoalContext``'s success thresholds and hold (None: :data:`SUCCESS`)."""
    asym_critic: bool = ASYM_CRITIC
    actor_history: int = 0
    observe_targets: bool = False
    """Add the commanded joint targets (PIP in flexion, normalised) to the actor input (v2)."""
    abnormal_factor: float = ABNORMAL_FACTOR
    goal_bonus: float = GOAL_BONUS
    tilt_penalty: float = TILT_PENALTY
    joint_map: bool = False
    object_kind: str = "cube"
    """``cube`` (40 mm, turned about world z) or ``cylinder`` (lying across the fingers, turned about its own axis)."""
    object_diameter_m: float = 0.035
    object_length_m: float = 0.055
    held_out_dir: str = ""
    """The evaluation's grasp caches (``<hand>.pt``); empty: ``CODE_ROTATE_CUBE_GRASPS`` or heldout_omnireset_v1."""
    """Run an O-trained policy through the contact-referenced joint maps (``joint_transfer.joint_maps``)."""
    reset_probs: dict | None = None
    """``{reset type: probability}`` (None: OmniReset's four types at 0.25)."""
    goal_offsets: dict | None = None
    """``OmniResetStates.goal_offsets`` (None: each dataset's stored offsets, v1)."""

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

    def pad_profile(self) -> PadProfile:
        return PadProfile(
            f"dome{self.pad_crown_mm:g}x{self.pad_crown_wid_mm:g}", crown_mm=self.pad_crown_mm,
            crown_wid_mm=self.pad_crown_wid_mm,
        )  # fmt: skip

    def __post_init__(self):
        desc_path = HANDS[self.hand_key]
        desc = Description.load(desc_path)
        profile = self.pad_profile()
        cylinder = self.object_kind == "cylinder"
        g = pinch_grasp(desc, size_m=self.object_diameter_m if cylinder else CUBE_SIZE_M)
        self.grasp = {k: v for k, v in g.items() if k in ("thumb_mcp_deg", "thumb_ip_deg", "rest_face", "rest_env")}
        common = dict(rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
                      collision_props=sim_utils.UsdPhysicsCollisionCfg(), activate_contact_sensors=True,
                      visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.5, 0.2)))  # fmt: skip
        if cylinder:  # the cube's density (400 kg/m^3); its axis along body y, across the fingers at rest
            r, length = self.object_diameter_m / 2, self.object_length_m
            spawn = sim_utils.CylinderCfg(radius=r, height=length, axis="Y",
                                          mass_props=sim_utils.MassCfg(mass=400.0 * math.pi * r * r * length),
                                          **common)  # fmt: skip
        else:
            spawn = sim_utils.CuboidCfg(size=(CUBE_SIZE_M,) * 3, mass_props=sim_utils.MassCfg(mass=CUBE_MASS_KG),
                                        **common)  # fmt: skip
        name = register_description_hand(
            f"rotate_{self.object_kind}_omnireset_{self.hand_key}_{profile.key()}" + ("_vis" if self.visuals else ""),
            desc_path,
            RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/object",
                spawn=spawn,
                init_state=RigidObjectCfg.InitialStateCfg(pos=g["rest_env"], rot=g["rest_quat_xyzw"]),
            ),
            servo_saturation=(True, True),
            profile=profile,
            thumb_pad=True,
            visuals=self.visuals,
            servo_colliders=("pip",),
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
        self.face_origin, self.face_rot, self.rest_face = g["face_origin"], g["face_rot"], g["rest_face"]
        rest = tuple(float(v) for v in g["rest_env"])
        actuated, tables = list(spec.actuated_joint_names), dict(spec.four_bar_tables)
        joints = SceneEntityCfg("robot", joint_names=actuated, preserve_order=True)
        tips = SceneEntityCfg("robot", body_names=["index_distal", "middle_distal", "ring_distal", "thumb_distal"],
                              preserve_order=True)  # fmt: skip
        if self.joint_map:
            from .joint_transfer import joint_maps

            maps = joint_maps(str(HANDS["O"]), str(desc_path))
            self.actions.joint_pos = mdp.MappedFlexionActionCfg(
                **{k: v for k, v in self.actions.joint_pos.to_dict().items() if k != "class_type"},
                joint_maps={j: [[math.radians(x) for x in m["q_o_deg"]], [math.radians(x) for x in m["q_h_deg"]]]
                            for j, m in maps.items() if j != "identity"},
            )  # fmt: skip
            self.observations.policy.joint_pos = ObsTerm(func=mdp.mapped_joint_pos_normalized, params={})
        self.actions.joint_pos.joint_names = actuated
        self.actions.joint_pos.four_bar_tables = tables
        obs = self.observations
        obs.policy.joint_pos.params = {"asset_cfg": joints, "four_bar_tables": tables}
        obs.policy.cube_pose.params = {"rest_pos": rest}
        obs.policy.cube_goal_error.params = {"rest_pos": rest}
        if self.observe_targets:
            obs.policy.joint_targets = ObsTerm(func=rc_mdp.actuated_targets_normalized)
        obs.policy.history_length = self.actor_history
        if self.asym_critic:
            obs.critic = ObservationsCfg.CriticCfg()
            for term in ("joint_pos", "cube_pose", "cube_goal_error"):
                getattr(obs.critic, term).params = dict(getattr(obs.policy, term).params)  # managers add defaults
            if self.observe_targets:
                obs.critic.joint_targets = ObsTerm(func=rc_mdp.actuated_targets_normalized)
            obs.critic.joint_vel.params = {"asset_cfg": joints, "four_bar_tables": tables}
        rest_axis = tuple(float(v) for v in np.asarray(g["face_rot"]).reshape(3, 3)[:, 1])  # across the fingers
        self.rewards.goal_context.params = {"rest_pos": rest, "goal_step": self.goal_step_rad,
                                            **(self.success or SUCCESS)}  # fmt: skip
        if cylinder:
            self.rewards.goal_context.params.update(mode="body_y", rest_axis=rest_axis)
            # the only contact rule: the cylinder must not touch the palm (the episode ends; nothing else is penalised)
            self.scene.palm_contact = ContactSensorCfg(
                prim_path="{ENV_REGEX_NS}/object", filter_prim_paths_expr=["{ENV_REGEX_NS}/Robot/palm"]
            )
            self.terminations.object_palm_contact = DoneTerm(func=mdp.object_palm_contact,
                                                             params={"threshold_n": 0.05})  # fmt: skip
        self.terminations.abnormal_robot.params["factor"] = self.abnormal_factor
        if self.goal_bonus > 0.0:
            self.rewards.goal_advance_bonus = RewTerm(func=mdp.goal_advance_bonus, weight=self.goal_bonus)
        if self.tilt_penalty != 0.0:
            self.rewards.upright_tilt = RewTerm(func=mdp.upright_tilt_excess, weight=self.tilt_penalty,
                                                params={"free_rad": math.radians(15.0)})  # fmt: skip
        self.rewards.joint_vel.params = {"asset_cfg": joints, "four_bar_tables": tables}
        self.rewards.reach.params["asset_cfg"] = tips
        self.terminations.cube_out_of_reach.params["rest_pos"] = rest
        non_pip = [j for j in actuated if not j.endswith("_pip_servo")]
        self.events.servo_gains.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=non_pip, preserve_order=True)
        open_sign = [OPEN_SIGN.get(j, OPEN_SIGN.get(j.split("_", 1)[1], 0.0)) for j in actuated]
        reset = {
            "actuated": actuated, "four_bar_tables": tables, "goal_step": self.goal_step_rad, "rest_pos": rest,
            "rest_quat": tuple(float(v) for v in g["rest_quat_xyzw"]),
            "gen": {**GEN, "open_sign": open_sign, "spin_axis": rest_axis if cylinder else None},
        }  # fmt: skip
        if self.evaluation:
            self.episode_length_s = EVAL_EPISODE_STEPS * self.sim.dt * self.decimation
            reset["cache"] = str(Path(self.held_out_dir or HELD_OUT) / f"{self.hand_key}.pt")
            for term in ("cube_mass", "servo_gains"):
                setattr(self.events, term, None)
            self.actions.joint_pos.gain_log_range = (1.0, 1.0)
            self.terminations.cube_out_of_reach = None
            self.terminations.object_off_grasp = DoneTerm(func=rc_mdp.object_off_grasp, params={"max_distance": 0.055})
            self.terminations.object_tilted = DoneTerm(func=rc_mdp.ObjectTilted, params={"min_dot": 0.35})
            if cylinder:  # a cylinder turned about its own axis tilts its reset up axis by design
                self.terminations.object_tilted.params["min_dot"] = -2.0
        else:
            self.episode_length_s = TRAIN_EPISODE_S
            directory = dataset_dir(self.hand_key if not cylinder else f"cylinder_{self.hand_key}", profile)
            self.reset_datasets = str(directory)
            probs = self.reset_probs or {t: 0.25 for t in mdp.OMNIRESET_TYPES}
            reset["datasets"] = {t: str(directory / f"resets_{t}.pt") for t in probs}
            reset["probs"] = list(probs.values())
            if self.goal_offsets:
                reset["goal_offsets"] = self.goal_offsets
        self.events.reset_grasp.params = reset
        self.sim.physics = HandPhysicsCfg()
        apply_hand_buffers(self.sim.physics, spec.mjwarp_buffers)
        contact_model.apply(self.sim.physics)
        self.contact_model = contact_model.describe()
        if spec.stiffen_mjwarp_loops:
            self.events.stiffen_loops = EventTermCfg(func=stiffen_env_mujoco_loops, mode="startup")
        self.sim.default_visualizer_cfg = recording_visualizer_cfg(self.scene.num_envs, self.scene.env_spacing)


@configclass
class CoDeRotateCubeOOmniResetEnvCfg(CoDeRotateCubeOmniResetEnvCfg):
    hand_key = "O"


@configclass
class CoDeRotateCubeOOmniResetEvalEnvCfg(CoDeRotateCubeOmniResetEnvCfg):
    hand_key = "O"
    evaluation = True


V2_GOAL_STEP_RAD = math.radians(45.0)
V2_SUCCESS = {"success_distance_m": 0.02, "success_orientation_rad": 0.3, "hold_steps": 6}


@configclass
class CoDeRotateCubeOOmniResetV2EnvCfg(CoDeRotateCubeOmniResetEnvCfg):
    """v2 (``docs/experimental/omnireset_recipe.md`` section 6): the OmniReset framework with

    - a mid-turn reset type (Khandate et al., RSS 2023: reset states from inside a gait), half of the near-goal share;
    - the commanded joint targets in the actor input, a 5-step actor history and the privileged critic (OmniReset paper
      section 3.3 and UWLab v1.2.0 ``rl_state_cfg.py``: ``history_length = 5``, ``critic = CriticCfg()``);
    - 45 deg chained goals (AnyRotate's small increments), success at 0.3 rad held 0.2 s (6 steps; references count
      success at once, at 0.1-0.4 rad);
    - the abnormal-speed test at 10x the servo limit (``CODE_OMNIRESET_ABNORMAL_FACTOR`` overrides).

    Goal offsets at reset: 45 deg ahead for the reaching, near-object and grasp types; near-goal half within +-0.45 rad
    (1.5x the success threshold) and half 0.45-0.785 rad ahead; mid-turn as stored (10-60 deg)."""

    hand_key = "O"
    version = "v2"
    goal_step_rad = V2_GOAL_STEP_RAD
    success = V2_SUCCESS
    asym_critic = True
    actor_history = 5
    observe_targets = True
    abnormal_factor = float(os.environ.get("CODE_OMNIRESET_ABNORMAL_FACTOR", "10"))
    reset_probs = {"ObjectAnywhereHandAnywhere": 0.25, "ObjectRestingHandNear": 0.25, "ObjectAnywhereHandGrasped": 0.25,
                   "ObjectNearGoalHandGrasped": 0.125, "ObjectMidTurnHandWound": 0.125}  # fmt: skip
    goal_offsets = {
        "ObjectAnywhereHandAnywhere": ["fixed", V2_GOAL_STEP_RAD],
        "ObjectRestingHandNear": ["fixed", V2_GOAL_STEP_RAD],
        "ObjectAnywhereHandGrasped": ["fixed", V2_GOAL_STEP_RAD],
        "ObjectNearGoalHandGrasped": ["near", 1.5 * V2_SUCCESS["success_orientation_rad"], V2_GOAL_STEP_RAD],
    }


@configclass
class CoDeRotateCubeOOmniResetV2EvalEnvCfg(CoDeRotateCubeOOmniResetV2EnvCfg):
    evaluation = True


TRANSFER_HANDS = ("R", "U", "R5", "U5", "R9", "U9")
"""Hands the O-trained v2 policy is evaluated on zero-shot (``docs/experimental/rl_transfer.md``)."""
for _key in TRANSFER_HANDS:
    globals()[f"CoDeRotateCube{_key}OmniResetV2EvalEnvCfg"] = configclass(
        type(f"CoDeRotateCube{_key}OmniResetV2EvalEnvCfg", (CoDeRotateCubeOOmniResetV2EvalEnvCfg,), {"hand_key": _key})
    )
for _key in ("O", *TRANSFER_HANDS):  # the "mapped" condition (O: the maps are identity, a check)
    globals()[f"CoDeRotateCube{_key}OmniResetV2EvalMappedEnvCfg"] = configclass(
        type(f"CoDeRotateCube{_key}OmniResetV2EvalMappedEnvCfg", (CoDeRotateCubeOOmniResetV2EvalEnvCfg,),
             {"hand_key": _key, "joint_map": True})
    )

CYLINDER_HELD_OUT = SIM_ROOT / "logs" / "rotate_cube" / "heldout_cylinder_v1"


@configclass
class CoDeRotateCylinderOFingertipEnvCfg(CoDeRotateCubeOOmniResetV2EnvCfg):
    """A 35 mm x 55 mm cylinder lying across the fingers of the palm-up hand, turned about its own axis by chained 45
    deg goals (``docs/experimental/rl_cylinder.md``): v2's framework and v2c's recipe (goal bonus 5; entropy 0.001 and
    std cap 1.0 in the agent config), OmniReset's four reset types (a mid-turn type once a policy exists), rounded pads
    (2 mm crown along the length, 0.5 mm across), and one contact rule: the cylinder must not touch the palm (the
    episode ends)."""

    object_kind = "cylinder"
    pad_crown_mm = 2.0
    pad_crown_wid_mm = 0.5
    goal_bonus = 5.0
    reset_probs = None
    goal_offsets = {
        "ObjectAnywhereHandAnywhere": ["fixed", V2_GOAL_STEP_RAD],
        "ObjectRestingHandNear": ["fixed", V2_GOAL_STEP_RAD],
        "ObjectAnywhereHandGrasped": ["fixed", V2_GOAL_STEP_RAD],
        "ObjectNearGoalHandGrasped": ["near", 1.5 * V2_SUCCESS["success_orientation_rad"], V2_GOAL_STEP_RAD],
    }
    held_out_dir = str(CYLINDER_HELD_OUT)


@configclass
class CoDeRotateCylinderOFingertipEvalEnvCfg(CoDeRotateCylinderOFingertipEnvCfg):
    evaluation = True


for _key in ("O", *TRANSFER_HANDS):
    for _suffix, _extra in (("", {}), ("Mapped", {"joint_map": True})):
        _n = f"CoDeRotateCylinder{_key}FingertipEval{_suffix}EnvCfg"
        globals()[_n] = configclass(type(_n, (CoDeRotateCylinderOFingertipEvalEnvCfg,), {"hand_key": _key, **_extra}))

__all__ = ["CoDeRotateCubeOOmniResetEnvCfg", "CoDeRotateCubeOOmniResetEvalEnvCfg", "CoDeRotateCubeOOmniResetV2EnvCfg",
           "CoDeRotateCubeOOmniResetV2EvalEnvCfg", "dataset_dir"]
