# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""In-hand cylinder rotation on the CoDe hand built with hand O's fingers (Hora-style, continuous).

The hand is O (``design/output/sim/study3/hands/cfl_q120_h3to7_h0.json``: CFL four-bar PIP, 120 deg, 3 mm pads),
built the way the Study 3 measurement builds it (:func:`~code_isaaclab.hands.description_hands.register_description_hand`:
flat convex-mesh pads, the pilot's shared servo, root pose and solver buffers), with the XM335 effort limit on.

- Hand: palm up (the Study 3 pose). The three fingers are parallel and flex in one plane, so the cylinder lies across
  them (axis along world ``y``) in a cradle: each finger's MCP extended by 5 deg, so the proximal pads slope down
  toward the PIP, and its PIP flexed to 50 deg, so the distal pad closes the cradle. Gravity seats the cylinder
  against the distal pads. The thumb starts at its zero pose and is actuated like every other joint.
- Cylinder: radius 20 mm, length 70 mm (spans the three fingers' 69 mm of pad width), 50 g. Its resting place is O's
  two-pad grasp at q0 = 50 deg (``design/output/sim/study3/two_pad_grasps.json``: the distal pad touches a 20 mm
  cylinder lying on the proximal pad at x_c = 36.1 mm from the MCP axis), turned with the proximal phalanx by the
  MCP's -5 deg.
- Goal: spin about the cylinder's own axis in a fixed direction: ``ROTATION_DIRECTION`` = -1 about world ``+y``, i.e.
  the top of the cylinder moving toward the palm, the way a curling finger's pad drags it.
- Actions: joint targets around the cradle pose (zero action holds it), PIP in flexion (:mod:`.rotate_mdp`), EMA.
- Rewards (weights; the manager multiplies each by the step dt): spin clipped to [-3, 1] rad/s (+1.0), cylinder
  speed L1 (-1.0), distance from the nominal resting place (-10), axis tilt (-1.0), action rate (-0.005), actuated
  joint torque L2 (-0.1), drop (-60, i.e. -2 once).
- Termination: the cylinder more than 40 mm from its resting place (a drop), or the 10 s time out.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs import mdp as base_mdp
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialBaseCfg
from isaaclab.utils import configclass

from isaaclab_tasks.core.reorient import mdp as reorient_mdp

from code_isaaclab.assets.description import Description
from code_isaaclab.flexion_control import flexion_joint_pos_normalized, flexion_joint_vel
from code_isaaclab.hands import get_hand
from code_isaaclab.hands.code_variants import HAND_ROOT_POS, REPO_ROOT
from code_isaaclab.hands.description_hands import cylinder_cfg, register_description_hand
from code_isaaclab.loops import stiffen_env_mujoco_loops, sync_four_bar_joints_on_reset
from code_isaaclab.physics import HandPhysicsCfg, apply_hand_buffers

from . import rotate_mdp

HAND_DIR = Path(os.environ.get("CODE_ROTATE_HAND_DIR", REPO_ROOT / "design" / "output" / "sim" / "study3" / "hands"))
HANDS = {k: HAND_DIR / f"cfl_q120_h3to7_{v}.json" for k, v in (("O", "h0"), ("R", "hr"), ("U", "hu"))}
"""Exports of the reference O and the 7 mm redesigns R (keeps the contact-surface motion) and U (coverage alone),
as in ``scripts/study3_measurement/common.py`` (``design/scripts/export/code_hand_sim.py --params <hand>.yaml``)."""
HAND_O = HANDS["O"]
GRASPS_JSON = REPO_ROOT / "design" / "output" / "sim" / "study3" / "two_pad_grasps.json"

CYLINDER_RADIUS_M = 0.020
CYLINDER_LENGTH_M = 0.070
CYLINDER_MASS_KG = 0.050
CRADLE_PIP_DEG = 50.0
CRADLE_MCP_DEG = -5.0
CYLINDER_DROP_M = 0.001
"""Spawned this far above its resting place (it settles into the cradle)."""
FINGERS = ("index", "middle", "ring")

ROTATION_DIRECTION = -1.0
"""Spin sign about world +y that is rewarded (-1: the cylinder's top moving toward the palm, world -x)."""
SPIN_CLIP = (-3.0, 1.0)
"""Rewarded spin range [rad/s]. The negative side is wider so that a back-and-forth roll (fast back, slow forward)
cannot earn reward without net rotation."""
DROP_DISTANCE_M = 0.04
OBJECT_POSITION_NOISE_M = 0.002
HAND_JOINT_NOISE = 0.02
"""Fraction of the way from the default joint position toward a random limit (as the CoDe reorientation task)."""
ACTION_EMA_ALPHA = 0.2
EPISODE_S = 10.0


def cradle_x_c_m(radius_m: float = CYLINDER_RADIUS_M, q0_deg: float = CRADLE_PIP_DEG, hand: str = "O") -> float:
    """The two-pad grasp rule: the distance of the cylinder's centre from the foot of the MCP axis, along the proximal
    pad face, at which ``hand``'s distal pad touches it while the PIP is at ``q0_deg`` [m].

    From Study 3's two-pad placements (``two_pad_grasps.json``, e17), which list for each placement x_c the flexion at
    which each hand's distal pad touches; ``q0_deg`` is interpolated in that column (exact for O, whose flexions are the
    grid)."""
    import numpy as np

    rows = sorted((g[hand], g["x_c_mm"]) for g in json.loads(GRASPS_JSON.read_text())["grasps"]
                  if g["rho_mm"] == radius_m * 1e3 and g.get(hand) is not None)
    q, x = np.array(rows).T
    if not q[0] <= q0_deg <= q[-1]:
        raise ValueError(f"q0 {q0_deg:g} deg outside {hand}'s two-pad placements at rho {radius_m * 1e3:g} mm")
    return float(np.interp(q0_deg, q, x)) * 1e-3


def face_frame(desc: Description, mcp_deg: float = CRADLE_MCP_DEG) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """The proximal pad-face frame at the cradle, fixed to the palm: the middle finger's proximal pad face with the MCP
    at ``mcp_deg`` (default the cradle's :data:`CRADLE_MCP_DEG`); origin at the foot of the MCP axis on that face, x along the face toward the tip, z its
    outward normal (toward the object), y = z x x. Returns ``(origin, rotation)`` in the environment frame, the rotation
    as 9 numbers, row-major, whose columns are the frame's axes. A hand's face depth below the link line (O 6 mm,
    R 10 mm) moves the origin, not what is observed in the frame."""
    import numpy as np

    fe = np.array(desc.joint("middle_fe")["anchor"], dtype=float)
    pad = next(b for body in desc.bodies for b in (body.get("boxes") or []) if b["name"] == "middle_proximal_pad")
    face_z = float(pad["center"][2]) - float(pad["half_extents"][1])  # palmar face (hand -z side) at q = 0
    t = math.radians(mcp_deg)  # MCP flexion: a rotation about hand +y (positive toward the palmar side, -z)
    rot_y = np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])
    root = np.diag([1.0, -1.0, -1.0])  # palm up: half a turn about x (HAND_ROOT_ROT_XYZW)
    foot = fe + rot_y @ (np.array([fe[0], fe[1], face_z]) - fe)
    origin = np.array(HAND_ROOT_POS) + root @ foot
    ex = root @ rot_y @ np.array([1.0, 0.0, 0.0])
    ez = root @ rot_y @ np.array([0.0, 0.0, -1.0])
    rot = np.stack((ex, np.cross(ez, ex), ez), axis=1)
    return tuple(float(v) for v in origin), tuple(float(v) for v in rot.reshape(-1))


def rest_in_face(hand: str = "O") -> tuple[float, float, float]:
    """The cylinder's rest place in the face frame: ``(x_c, 0, radius)`` by the two-pad grasp rule (:func:`cradle_x_c_m`)."""
    return (cradle_x_c_m(hand=hand), 0.0, CYLINDER_RADIUS_M)


def distal_face_points(desc: Description) -> tuple[tuple[float, float, float], ...]:
    """Per finger (index, middle, ring): the point of the distal pad's contact face below the fingertip (the face's
    distal end, on the pad's centre line), in the distal body's frame (body axes = hand axes at the geometric zero)."""
    out = []
    for f in FINGERS:
        origin = desc.body(f"{f}_distal")["origin"]
        pad = next(b for b in desc.body(f"{f}_distal")["boxes"] if b["name"] == f"{f}_distal_pad")
        point = (pad["center"][0] + pad["half_extents"][0], pad["center"][1], pad["center"][2] - pad["half_extents"][1])
        out.append(tuple(float(point[k] - origin[k]) for k in range(3)))
    return tuple(out)


def cradle_joint_pos(desc: Description) -> dict[str, float]:
    """The cradle pose: every finger's MCP at :data:`CRADLE_MCP_DEG`, its four-bar closed at :data:`CRADLE_PIP_DEG`
    (servo and passive joints from the export's tables); everything else (abduction, thumb) at zero [rad]."""
    pose = {j["name"]: 0.0 for j in desc.joints}  # explicit names: overlapping regex keys are an error
    for f in FINGERS:
        pose[f"{f}_fe"] = math.radians(CRADLE_MCP_DEG)
        for joint, deg in desc.closed_pose(f, CRADLE_PIP_DEG).items():
            pose[joint] = math.radians(deg)
    return pose


def cradle_cylinder_pos_w(desc: Description, hand: str = "O") -> tuple[float, float, float]:
    """The cylinder's spawn position, environment frame: its rest place (:func:`rest_in_face`) over the middle finger,
    :data:`CYLINDER_DROP_M` above it."""
    origin, rot = face_frame(desc)
    rest = rest_in_face(hand)
    pos = [origin[i] + sum(rot[3 * i + j] * rest[j] for j in range(3)) for i in range(3)]
    return (pos[0], pos[1], pos[2] + CYLINDER_DROP_M)


##
# MDP
##


@configclass
class RotateSceneCfg(InteractiveSceneCfg):
    num_envs = 128
    env_spacing = 0.5
    robot = None  # from the hand spec
    object = None
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/Light",
                         spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75)))


@configclass
class ActionsCfg:
    joint_pos = rotate_mdp.DefaultOffsetFlexionActionCfg(asset_name="robot", joint_names=[], alpha=ACTION_EMA_ALPHA,
                                                         four_bar_tables={})


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=flexion_joint_pos_normalized,
                            params={"asset_cfg": SceneEntityCfg("robot"), "four_bar_tables": {}})
        joint_vel = ObsTerm(func=flexion_joint_vel, scale=0.2,
                            params={"asset_cfg": SceneEntityCfg("robot"), "four_bar_tables": {}})
        fingertip_pos = ObsTerm(func=reorient_mdp.fingertip_pos, params={"asset_cfg": SceneEntityCfg("robot")})
        fingertip_vel = ObsTerm(func=reorient_mdp.fingertip_vel, scale=0.2, params={"asset_cfg": SceneEntityCfg("robot")})
        object_pos = ObsTerm(func=rotate_mdp.object_pos_from_nominal, scale=10.0)
        object_quat = ObsTerm(func=base_mdp.root_quat_w, params={"asset_cfg": SceneEntityCfg("object"),
                                                                 "make_quat_unique": True})
        object_axis = ObsTerm(func=rotate_mdp.cylinder_axis_w)
        object_lin_vel = ObsTerm(func=base_mdp.root_lin_vel_w, scale=10.0, params={"asset_cfg": SceneEntityCfg("object")})
        object_ang_vel = ObsTerm(func=base_mdp.root_ang_vel_w, scale=0.2, params={"asset_cfg": SceneEntityCfg("object")})
        object_spin = ObsTerm(func=rotate_mdp.axis_spin, scale=0.2)
        last_action = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class RewardsCfg:
    rotation = RewTerm(func=rotate_mdp.rotation_reward, weight=1.0,
                       params={"direction": ROTATION_DIRECTION, "clip_min": SPIN_CLIP[0], "clip_max": SPIN_CLIP[1]})
    object_lin_vel = RewTerm(func=rotate_mdp.object_lin_vel_l1, weight=-1.0)
    object_pos = RewTerm(func=rotate_mdp.object_pos_deviation, weight=-10.0)
    axis_tilt = RewTerm(func=rotate_mdp.axis_tilt, weight=-1.0)
    action_rate = RewTerm(func=base_mdp.action_rate_l2, weight=-0.005)
    joint_torque = RewTerm(func=base_mdp.joint_torques_l2, weight=-0.1, params={"asset_cfg": SceneEntityCfg("robot")})
    drop = RewTerm(func=base_mdp.is_terminated_term, weight=-60.0, params={"term_keys": "object_dropped"})


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    object_dropped = DoneTerm(func=rotate_mdp.object_dropped, params={"max_distance": DROP_DISTANCE_M})


@configclass
class EventCfg:
    reset_object = EventTermCfg(func=rotate_mdp.reset_object_pose, mode="reset",
                                params={"position_noise_m": OBJECT_POSITION_NOISE_M})
    reset_hand = EventTermCfg(func=reorient_mdp.reset_reorient_hand, mode="reset",
                              params={"joint_position_noise": HAND_JOINT_NOISE, "joint_velocity_noise": 0.0})
    stiffen_loops: EventTermCfg | None = None
    close_four_bar: EventTermCfg | None = None


@configclass
class CoDeRotateCylinderEnvCfg(ManagerBasedRLEnvCfg):
    """In-hand cylinder rotation on a CoDe hand from a ``code_hand_sim/1`` description (:attr:`description`)."""

    description: str = str(HAND_O)
    hand_name: str = "rotate_O"
    hand_key: str = "O"
    """Which column of the two-pad placements sets the cylinder's rest place (O, R, U)."""

    scene: RotateSceneCfg = RotateSceneCfg()
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    commands = None

    episode_length_s = EPISODE_S
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120, physics_material=RigidBodyMaterialBaseCfg(static_friction=1.0, dynamic_friction=1.0))

    def __post_init__(self):
        desc = Description.load(self.description)
        name = register_description_hand(
            self.hand_name, Path(self.description),
            cylinder_cfg(CYLINDER_RADIUS_M, CYLINDER_LENGTH_M, CYLINDER_MASS_KG, cradle_cylinder_pos_w(desc, self.hand_key)),
            servo_saturation=(True, True))
        spec = get_hand(name)
        self.scene.robot = spec.robot_cfg.replace(
            init_state=spec.robot_cfg.init_state.replace(joint_pos=cradle_joint_pos(desc)))
        self.scene.object = spec.object_cfg
        self.decimation = spec.decimation
        self.sim.render_interval = self.decimation
        tables = dict(spec.four_bar_tables)
        actuated = list(spec.actuated_joint_names)
        self.actions.joint_pos.joint_names = actuated
        self.actions.joint_pos.four_bar_tables = tables
        policy = self.observations.policy
        for term in (policy.joint_pos, policy.joint_vel):
            term.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=actuated, preserve_order=True)
            term.params["four_bar_tables"] = tables
        for term in (policy.fingertip_pos, policy.fingertip_vel):
            term.params["asset_cfg"] = SceneEntityCfg("robot", body_names=list(spec.fingertip_body_names),
                                                      preserve_order=True)
        self.rewards.joint_torque.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=actuated,
                                                                       preserve_order=True)
        self.sim.physics = HandPhysicsCfg()
        apply_hand_buffers(self.sim.physics, spec.mjwarp_buffers)
        if spec.stiffen_mjwarp_loops:
            self.events.stiffen_loops = EventTermCfg(func=stiffen_env_mujoco_loops, mode="startup")
        if spec.four_bar_tables is not None:
            self.events.close_four_bar = EventTermCfg(func=sync_four_bar_joints_on_reset, mode="reset",
                                                      params={"tables": spec.four_bar_tables, "asset_name": "robot"})


@configclass
class CoDeRotateCylinderOEnvCfg(CoDeRotateCylinderEnvCfg):
    """Hand O (the reference CFL finger, 3 mm pads)."""


SMOOTH_ACTION_EMA_ALPHA = 0.1
"""As the Study 3 main reorientation task (``MAIN_ACTION_EMA_ALPHA``)."""
FLEXION_STIFFNESS = 5.0
FLEXION_DAMPING = 0.08
"""PIP impedance in flexion, K_q [N m/rad] and D_q [N m s/rad], the same for every hand: each hand's servo gains are
rescheduled to K_q / (dtheta/dq)^2 and D_q / (dtheta/dq)^2 every step (the Study 3 two-pad measurement's values)."""


@configclass
class CoDeRotateCylinderSmoothEnvCfg(CoDeRotateCylinderEnvCfg):
    """The rotation task, smoothed against the first policy's shaking gait and made transferable across hands (O, R, U)
    without changing the policy's inputs or outputs. Relative to :class:`CoDeRotateCylinderEnvCfg`:

    Smoothing (the first O policy rattled the index and ring abduction at ~5 Hz: the cylinder at 123 mm/s and 1.7 rad/s
    off its axis on average):

    - action EMA alpha 0.2 -> 0.1; action rate L2 weight -0.005 -> -0.02; cylinder linear velocity L1 -1.0 -> -3.0;
    - new: actuated joint velocity L2 (isaaclab ``joint_vel_l2``), -0.01; the cylinder's angular velocity off its axis
      (norm), -0.2. At the first policy's motion these cost about 1.1 per second, more than its rotation reward (0.95).
      The spin cap stays at 1 rad/s.

    Transfer (hand-independent observations, actions and rest place):

    - actions: the PIP impedance set in flexion (K_q, D_q), the same for every hand; targets as before (cradle pose and
      ranges in q: MCP, abduction and thumb joints are direct drive, the PIP in flexion through each hand's own table);
    - observations: no fingertip pose or velocity (skeleton quantities); instead each finger's distal pad-face point
      below the fingertip; the cylinder's position in the proximal pad-face frame (:func:`face_frame`);
    - rest place: the two-pad grasp rule at the cradle q0 for this hand (:func:`rest_in_face`); the position penalty
      and the drop termination measure the distance from it in the face frame.
    """

    hand_key: str = "O"
    face_origin: tuple | None = None
    face_rot: tuple | None = None
    rest_face: tuple | None = None
    distal_face_points: tuple | None = None

    def __post_init__(self):
        self.description = str(HANDS[self.hand_key])
        self.hand_name = f"rotate_{self.hand_key}"
        super().__post_init__()
        desc = Description.load(self.description)
        self.face_origin, self.face_rot = face_frame(desc)
        self.rest_face = rest_in_face(self.hand_key)
        self.distal_face_points = distal_face_points(desc)
        act = self.actions.joint_pos
        act.alpha = SMOOTH_ACTION_EMA_ALPHA
        act.flexion_stiffness = FLEXION_STIFFNESS
        act.flexion_damping = FLEXION_DAMPING
        policy = self.observations.policy
        policy.fingertip_pos = None
        policy.fingertip_vel = None
        policy.object_pos = ObsTerm(func=rotate_mdp.object_pos_face, scale=10.0)
        policy.distal_pad_points = ObsTerm(func=rotate_mdp.distal_pad_points_face, scale=10.0, params={
            "asset_cfg": SceneEntityCfg("robot", body_names=[f"{f}_distal" for f in FINGERS], preserve_order=True)})
        self.rewards.object_pos = RewTerm(func=rotate_mdp.object_rest_distance, weight=-10.0)
        self.terminations.object_dropped = DoneTerm(func=rotate_mdp.object_dropped_face,
                                                    params={"max_distance": DROP_DISTANCE_M})
        self.rewards.action_rate.weight = -0.02
        self.rewards.object_lin_vel.weight = -3.0
        self.rewards.joint_vel = RewTerm(func=base_mdp.joint_vel_l2, weight=-0.01, params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=list(act.joint_names), preserve_order=True)})
        self.rewards.off_axis_ang_vel = RewTerm(func=rotate_mdp.off_axis_ang_vel, weight=-0.2)


@configclass
class CoDeRotateCylinderOSmoothEnvCfg(CoDeRotateCylinderSmoothEnvCfg):
    hand_key = "O"


@configclass
class CoDeRotateCylinderRSmoothEnvCfg(CoDeRotateCylinderSmoothEnvCfg):
    hand_key = "R"


@configclass
class CoDeRotateCylinderUSmoothEnvCfg(CoDeRotateCylinderSmoothEnvCfg):
    hand_key = "U"


__all__ = ["CoDeRotateCylinderEnvCfg", "CoDeRotateCylinderOEnvCfg", "CoDeRotateCylinderOSmoothEnvCfg",
           "CoDeRotateCylinderRSmoothEnvCfg", "CoDeRotateCylinderSmoothEnvCfg", "CoDeRotateCylinderUSmoothEnvCfg",
           "cradle_cylinder_pos_w", "cradle_joint_pos", "distal_face_points", "face_frame", "rest_in_face"]
