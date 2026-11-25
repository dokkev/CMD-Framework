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
- terminations: time-out (16 s), abnormal joint speed, and the cube off the hand;
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
    "lift_prob": 0.25, "close_deg": 10.0,
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
    abnormal_robot = DoneTerm(func=mdp.abnormal_robot_state)
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
        g = pinch_grasp(desc)
        self.grasp = {k: v for k, v in g.items() if k in ("thumb_mcp_deg", "thumb_ip_deg", "rest_face", "rest_env")}
        name = register_description_hand(
            f"rotate_cube_omnireset_{self.hand_key}_{profile.key()}" + ("_vis" if self.visuals else ""),
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
        self.actions.joint_pos.joint_names = actuated
        self.actions.joint_pos.four_bar_tables = tables
        obs = self.observations
        obs.policy.joint_pos.params = {"asset_cfg": joints, "four_bar_tables": tables}
        obs.policy.cube_pose.params = {"rest_pos": rest}
        obs.policy.cube_goal_error.params = {"rest_pos": rest}
        if ASYM_CRITIC:
            obs.critic = ObservationsCfg.CriticCfg()
            for term in ("joint_pos", "cube_pose", "cube_goal_error"):
                getattr(obs.critic, term).params = getattr(obs.policy, term).params
            obs.critic.joint_vel.params = {"asset_cfg": joints, "four_bar_tables": tables}
        self.rewards.goal_context.params = {"rest_pos": rest, "goal_step": GOAL_STEP_RAD, **SUCCESS}
        self.rewards.joint_vel.params = {"asset_cfg": joints, "four_bar_tables": tables}
        self.rewards.reach.params["asset_cfg"] = tips
        self.terminations.cube_out_of_reach.params["rest_pos"] = rest
        non_pip = [j for j in actuated if not j.endswith("_pip_servo")]
        self.events.servo_gains.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=non_pip, preserve_order=True)
        open_sign = [OPEN_SIGN.get(j, OPEN_SIGN.get(j.split("_", 1)[1], 0.0)) for j in actuated]
        reset = {
            "actuated": actuated, "four_bar_tables": tables, "goal_step": GOAL_STEP_RAD, "rest_pos": rest,
            "rest_quat": tuple(float(v) for v in g["rest_quat_xyzw"]), "gen": {**GEN, "open_sign": open_sign},
        }  # fmt: skip
        if self.evaluation:
            self.episode_length_s = EVAL_EPISODE_STEPS * self.sim.dt * self.decimation
            reset["cache"] = str(HELD_OUT / f"{self.hand_key}.pt")
            for term in ("cube_mass", "servo_gains"):
                setattr(self.events, term, None)
            self.actions.joint_pos.gain_log_range = (1.0, 1.0)
            self.terminations.cube_out_of_reach = None
            self.terminations.object_off_grasp = DoneTerm(func=rc_mdp.object_off_grasp, params={"max_distance": 0.055})
            self.terminations.object_tilted = DoneTerm(func=rc_mdp.ObjectTilted, params={"min_dot": 0.35})
        else:
            self.episode_length_s = TRAIN_EPISODE_S
            directory = dataset_dir(self.hand_key, profile)
            self.reset_datasets = str(directory)
            reset["datasets"] = {t: str(directory / f"resets_{t}.pt") for t in mdp.RESET_TYPES}
            reset["probs"] = [0.25] * len(mdp.RESET_TYPES)
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


__all__ = ["CoDeRotateCubeOOmniResetEnvCfg", "CoDeRotateCubeOOmniResetEvalEnvCfg", "dataset_dir"]
