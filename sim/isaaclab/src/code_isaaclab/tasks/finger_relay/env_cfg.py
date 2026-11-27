# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Finger-relay dial (attempt 7a', ``docs/experimental/rl_pip_only.md``): the palm-up hand with every MCP, the
abductions, the ring PIP and the thumb held; the index and middle PIPs (flexion 0-120 deg) take turns dragging the
hand-facing face of a disk on an axle normal to the proximal pad face, against a torsional spring, to turn it on
through chained goals. One finger's stroke turns it a limited amount, and a finger dragging back while extending turns
it back, so continued turning needs lift-off, return and re-engagement while the other finger holds (finger gaiting).

Geometry (proximal pad-face frame of the middle finger at MCP 0, ``grasp.face_frame``: x along the face toward the
tips, y across, z the face's outward normal; the same face-frame coordinates on every hand, so each hand's own pad
face carries the dial as in Drake's tip dial, experiment 28/29z): the disk's hand-facing face at O's index fingertip
height at the touch flexion :data:`Q_TOUCH_DEG` (pad tip end), its centre :data:`X_BACK_M` proximal of that tip
and :data:`Y_OUT_M` beyond the index on the side away from the middle finger, radius :data:`RADIUS_M` (covers both
fingertips, not the ring's). The axle slides along its axis on a spring (Drake's 60 N/m) so a flexing fingertip can
press on and drag, and the disk turns against :data:`K_SPIN` N m/rad with :data:`D_SPIN` damping."""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
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
from code_isaaclab.hands.code_variants import HAND_ROOT_POS
from code_isaaclab.hands.description_hands import register_description_hand
from code_isaaclab.loops import stiffen_env_mujoco_loops
from code_isaaclab.physics import HandPhysicsCfg, apply_hand_buffers

from ..rotate_cube.camera import recording_visualizer_cfg
from ..rotate_cube.grasp import FLEXION_DAMPING, FLEXION_STIFFNESS, HANDS, _joint_tf, face_frame
from . import mdp

SIM_ROOT = Path(__file__).resolve().parents[4]
ASSET = SIM_ROOT / "logs" / "finger_relay" / "assets"
Q_TOUCH_DEG = 30.0
X_BACK_M, Y_OUT_M, RADIUS_M, THICK_M = 0.010, 0.012, 0.045, 0.004
DIAL_MASS_KG = 0.03
SLIDE_M, K_SLIDE, D_SLIDE = 0.025, 60.0, 1.0
K_SPIN, D_SPIN = 0.002, 0.001
ACTING = ("index_pip_servo", "middle_pip_servo")
GOAL_STEP_RAD = math.radians(30.0)
SUCCESS_RAD, HOLD_STEPS = math.radians(8.0), 3
EPISODE_S = 10.0
START_MAX_DEG = 25
"""Each acting finger starts at a flexion uniform on 0..25 deg (1 deg grid), below the 30 deg touch."""
THUMB_DEG = {"thumb_ro": 0.0, "thumb_raa": 0.0, "thumb_mcp": 0.0, "thumb_ip": 0.0}
"""The thumb lies out flat on the index side, below the dial."""
TOUCH = {"max_force_n": 5.0, "threshold_n": 0.05, "force_noise": 0.10, "flip_prob": 0.05}


def tip_point_face(desc: Description, finger: str, q_deg: float) -> np.ndarray:
    """The distal pad face's tip-end centre of ``finger`` at PIP flexion ``q_deg`` and MCP 0, face frame [m]."""
    pose = desc.closed_pose(finger, q_deg)
    m = (_joint_tf(desc, f"{finger}_fe", 0.0) @ _joint_tf(desc, f"{finger}_pip_servo", pose[f"{finger}_pip_servo"])
         @ _joint_tf(desc, f"{finger}_b", pose[f"{finger}_b"]))  # fmt: skip
    box = next(b for b in desc.body(f"{finger}_distal")["boxes"] if b["name"] == f"{finger}_distal_pad")
    c, h = np.asarray(box["center"]), box["half_extents"]
    o, r = face_frame(desc, 0.0)
    p = (m @ np.array([c[0] + h[0], c[1], c[2] - h[1], 1.0]))[:3]
    return np.asarray(r).reshape(3, 3).T @ (np.asarray(HAND_ROOT_POS) + np.diag([1.0, -1.0, -1.0]) @ p - np.asarray(o))


def dial_place_face() -> np.ndarray:
    """The dial frame's origin in the face frame, fixed on O (the same coordinates on every hand)."""
    o = Description.load(HANDS["O"])
    tip = tip_point_face(o, "index", Q_TOUCH_DEG)
    side = np.sign(tip_point_face(o, "index", 0.0)[1] - tip_point_face(o, "middle", 0.0)[1])
    return np.array([tip[0] - X_BACK_M, tip[1] + side * Y_OUT_M, tip[2]])


def default_joint_pos(desc: Description) -> dict[str, float]:
    pose = {j["name"]: 0.0 for j in desc.joints}
    for f in ("index", "middle", "ring"):
        for joint, deg in desc.closed_pose(f, 0.0).items():
            pose[joint] = math.radians(deg)
    pose.update({k: math.radians(v) for k, v in THUMB_DEG.items()})
    return pose


@configclass
class SceneCfg(InteractiveSceneCfg):
    num_envs: int = 4096
    env_spacing: float = 0.5
    robot: ArticulationCfg = None
    dial: ArticulationCfg = None
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
        dial = ObsTerm(func=mdp.dial_obs)
        touch = ObsTerm(func=mdp.tip_touch, params=dict(TOUCH))

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 5

    @configclass
    class CriticCfg(ObsGroup):
        prev_actions = ObsTerm(func=base_mdp.last_action)
        pip = ObsTerm(func=flexion_joint_pos_normalized, params={})
        dial = ObsTerm(func=mdp.dial_obs)
        dial_priv = ObsTerm(func=mdp.dial_priv)
        touch = ObsTerm(func=mdp.tip_touch, params={**TOUCH, "force_noise": 0.0, "flip_prob": 0.0})

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
            self.history_length = 1

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class RewardsCfg:
    dial_goal = RewTerm(func=mdp.DialGoal, weight=1.0,
                        params={"goal_step": GOAL_STEP_RAD, "success_rad": SUCCESS_RAD, "hold_steps": HOLD_STEPS})
    progress = RewTerm(func=mdp.dial_progress, weight=300.0, params={"clip_rad": 0.2})
    """x step_dt: 10 per rad turned."""
    bonus = RewTerm(func=mdp.goal_bonus, weight=2.0)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    non_finite = DoneTerm(func=mdp.NonFinite)


@configclass
class EventCfg:
    reset = EventTermCfg(func=mdp.reset_hand_and_dial, mode="reset",
                         params={"robot_cfg": SceneEntityCfg("robot"), "dial_cfg": SceneEntityCfg("dial"),
                                 "start_poses": {}})


@configclass
class FingerRelayEnvCfg(ManagerBasedRLEnvCfg):
    hand_key: str = "O"
    dial_observed: bool = True
    dial_noise: dict | None = None
    """Attempt 8: the actor's dial angle delayed and noisy (``mdp.dial_obs_noisy`` parameters); None: exact."""
    """False: the actor sees no dial state (touch, joints and actions only; the critic keeps it)."""
    evaluation: bool = False
    placement_face: tuple | None = None
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
        dummy = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/unused", spawn=None)
        name = register_description_hand(f"finger_relay_{self.hand_key}_{profile.key()}", desc_path, dummy,
                                         servo_saturation=(True, True), profile=profile, thumb_pad=True,
                                         visuals=False, servo_colliders=("pip",))  # fmt: skip
        spec = get_hand(name)
        self.scene.robot = spec.robot_cfg.replace(
            init_state=spec.robot_cfg.init_state.replace(joint_pos=default_joint_pos(desc))
        )
        # the dial: fixed at the same face-frame coordinates on every hand (this hand's own pad face); with
        # CODE_RELAY_ANCHOR=f1 (docs/experimental/pr_mr.md) at O's face frame on every hand instead, i.e. fixed to the
        # palm and MCP axis (F_1) at O's world pose
        anchor = os.environ.get("CODE_RELAY_ANCHOR", "face")
        if anchor not in ("face", "f1"):
            raise ValueError(f"CODE_RELAY_ANCHOR must be face or f1, not {anchor!r}")
        o, r = face_frame(Description.load(HANDS["O"]) if anchor == "f1" else desc, 0.0)
        r = np.asarray(r).reshape(3, 3)
        place = dial_place_face()
        self.placement_face = tuple(float(v) for v in place)
        pos = np.asarray(o) + r @ place
        from ..rotate_cube.grasp import _matrix_to_quat_xyzw

        q = _matrix_to_quat_xyzw(r)
        usd = mdp_usd_path()
        self.scene.dial = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/object",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(usd), activate_contact_sensors=True,
            ),
            init_state=ArticulationCfg.InitialStateCfg(pos=tuple(float(v) for v in pos), rot=q,
                                                       joint_pos={"slide_joint": 0.0, "spin_joint": 0.0}),
            actuators={
                "slide": ImplicitActuatorCfg(joint_names_expr=["slide_joint"], stiffness=K_SLIDE, damping=D_SLIDE),
                "spin": ImplicitActuatorCfg(joint_names_expr=["spin_joint"], stiffness=K_SPIN, damping=D_SPIN),
            },
        )  # fmt: skip
        self.scene.tip_touch = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/object/disk",
            filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{f}_distal" for f in ("index", "middle")],
        )
        # random start: index and middle each at a flexion drawn from 0..START_MAX_DEG (1 deg grid), the four-bar
        # closed from the export (servo, a, b per finger), below the touch flexion
        self.events.reset.params["start_poses"] = {
            f: {"joints": [f"{f}_pip_servo", f"{f}_a", f"{f}_b"],
                "rad": [[math.radians(desc.closed_pose(f, float(q))[j]) for j in (f"{f}_pip_servo", f"{f}_a", f"{f}_b")]
                        for q in range(0, START_MAX_DEG + 1)]}
            for f in ("index", "middle")
        }  # fmt: skip
        tables = dict(spec.four_bar_tables)
        self.actions.joint_pos.four_bar_tables = tables
        joints = SceneEntityCfg("robot", joint_names=list(ACTING), preserve_order=True)
        for group in (self.observations.policy, self.observations.critic):
            group.pip.params = {"asset_cfg": joints, "four_bar_tables": tables}
        if not self.dial_observed:
            self.observations.policy.dial = None
        elif self.dial_noise:
            self.observations.policy.dial = ObsTerm(func=mdp.dial_obs_noisy, params=dict(self.dial_noise))
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


def mdp_usd_path() -> Path:
    from .dial_usd import write_dial_usd

    tag = f"r{RADIUS_M * 1e3:g}_t{THICK_M * 1e3:g}_m{DIAL_MASS_KG * 1e3:g}_s{SLIDE_M * 1e3:g}"
    return write_dial_usd(ASSET / f"relay_dial_{tag}.usda", RADIUS_M, THICK_M, DIAL_MASS_KG, SLIDE_M)


@configclass
class FingerRelayOEnvCfg(FingerRelayEnvCfg):
    hand_key = "O"


@configclass
class FingerRelayOTouchEnvCfg(FingerRelayEnvCfg):
    hand_key = "O"
    dial_observed = False


OBS_NOISE = {"ObsS": {"angle_amp_rad": math.radians(3.0), "delay_steps": (3, 6)},
             "ObsL": {"angle_amp_rad": math.radians(15.0), "delay_steps": (3, 6)}}
"""Attempt 8's observation levels: the dial angle +-3 deg (a vision tracker) or +-15 deg, 100-200 ms late."""
for _v, _noise in OBS_NOISE.items():
    _n = f"FingerRelayO{_v}EnvCfg"
    globals()[_n] = configclass(type(_n, (FingerRelayEnvCfg,), {"hand_key": "O", "dial_noise": _noise}))

KEYS = ("O", "R", "U", "R5", "U5", "R9", "U9", "MR", "MRM", "MRS", "MS")
for _key in KEYS:
    for _v, _extra in (("", {}), ("Touch", {"dial_observed": False}),
                       *((k, {"dial_noise": v}) for k, v in OBS_NOISE.items())):
        _n = f"FingerRelay{_key}{_v}EvalEnvCfg"
        globals()[_n] = configclass(type(_n, (FingerRelayEnvCfg,), {"hand_key": _key, "evaluation": True, **_extra}))
