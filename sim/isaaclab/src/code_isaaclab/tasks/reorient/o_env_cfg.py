# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""In-hand cube reorientation on the CoDe hand with hand O's fingers and a thumb pad: upstream's manager-based task
(``Isaac-Reorient-Cube-Allegro``: goal-orientation command resampled on success, its rewards and weights, its
termination, its PPO settings) with the CoDe adaptation of :mod:`.code_env_cfg` (palm up, a cube (40 mm here),
goals about one world axis, smoothed actions, small reset noise), and hand-transferable conventions:

- the hand: an export (O; R and U as config switches, ``hand_key``) built with :func:`register_description_hand`, plus
  a flat 3 mm contact pad on the thumb's distal segment (:func:`~code_isaaclab.hands.description_hands.thumb_pad_box`);
- the grasp: every finger's MCP at :data:`GRASP_MCP_DEG` and PIP at :data:`GRASP_PIP_DEG` (the same in q on every hand);
  the cube lies with a face on the three tilted proximal pads, its far face 2 mm from the distal pads standing at 90 deg,
  and the thumb pad presses its top face (:func:`pinch_grasp`: the thumb's MCP and IP put the pad
  :data:`PINCH_SQUEEZE_M` into the cube);
- actions: upstream's joint-position-to-limits EMA action with the PIP in flexion (0..120 deg, each hand's own
  theta(q)) and the PIP impedance set in flexion, K_q / (dtheta/dq)^2 (:class:`FlexionEMAActionCfg`);
- observations: upstream's, with the actuated joints in flexion space, no fingertip (skeleton) pose or velocity but a
  point on each pad face (the three distal pads below the tip, the thumb pad's centre), the cube's position in the
  proximal pad-face frame (:func:`~code_isaaclab.tasks.rotate.env_cfg.face_frame` at the grasp's MCP angle), and the
  goal as its orientation only (its position is a constant of the hand).
"""

from __future__ import annotations

import math
import os

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.envs import mdp as base_mdp
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from isaaclab_tasks.core.reorient import mdp as reorient_mdp

from code_isaaclab.assets.description import Description
from code_isaaclab.flexion_control import FlexionEMAActionCfg, flexion_joint_pos_normalized, flexion_joint_vel
from code_isaaclab.hands import get_hand
from code_isaaclab.hands.description_hands import register_description_hand, thumb_pad_box
from code_isaaclab.tasks.rotate import rotate_mdp
from code_isaaclab.tasks.rotate.env_cfg import HANDS, distal_face_points, face_frame

from .camera import env0_camera
from .code_env_cfg import CoDeReorientMainEnvCfg

FINGERS = ("index", "middle", "ring")
THUMB = ("thumb_ro", "thumb_raa", "thumb_mcp", "thumb_ip")
CUBE_SIZE_M = float(os.environ.get("CODE_REORIENT_O_CUBE_MM", 40.0)) * 1e-3
CUBE_MASS_KG = 0.05 * (CUBE_SIZE_M / 0.05) ** 3
"""The density of the CoDe task's 50 mm / 50 g cube. 40 mm (25.6 g) by default: a 50 mm cube does not fit between the
MCP knuckles and the distal pads standing at 90 deg (the proximal pad is 44 mm long), and touched both (probes)."""
GRASP_MCP_DEG = float(os.environ.get("CODE_REORIENT_O_MCP_DEG", 35.0))
GRASP_PIP_DEG = float(os.environ.get("CODE_REORIENT_O_PIP_DEG", 90.0))
"""The distal pads stand at right angles to the proximal pads, a wall behind the cube's far face (2 mm away), so that
PIP flexion pushes the cube (at 20 deg, the first probe, the distal pads never reached it)."""
WALL_GAP_M = 0.002
"""The cube's far face this far short of the distal pads' wall (:func:`distal_wall_along`) at rest."""
GRASP_THUMB_DEG = {"thumb_ro": -90.0, "thumb_raa": 20.0}
"""The thumb's fixed joints in the grasp; its MCP and IP are solved per hand by :func:`pinch_grasp` (a kinematic search
over the thumb's joints for its pad pressing down on a top face, ``scratchpad`` probes: ro -90, raa 20 reach farthest)."""
PINCH_SQUEEZE_M = 0.0005
"""The pinch is closed this much below the cube's width (a light preload through the thumb's drives)."""
GOAL_AXIS = tuple(float(v) for v in os.environ.get("CODE_REORIENT_O_AXIS", "0,0,1").split(","))
GOAL_MAX_DEG = float(os.environ.get("CODE_REORIENT_O_MAX_DEG", 30.0))
GOAL_MIN_STEP_DEG = 25.0
"""As the CoDe main task: more than twice the 0.2 rad success threshold, so no goal is reached for free."""
ACTION_EMA_ALPHA = 0.1
FLEXION_STIFFNESS, FLEXION_DAMPING = 5.0, 0.08
OBJECT_POSITION_NOISE_M = 0.001
def recording_visualizer_cfg(num_envs: int = 128, spacing: float = 0.5):
    """The Newton GL visualizer settings for video clips, as cheap as possible: offscreen (``headless``: it renders only
    when the video recorder grabs a frame, never on a plain step), the first 4 worlds only, 960 x 540, no live plots or
    picking; markers on (the goal-orientation marker). A hint (``sim.default_visualizer_cfg``): it adds no visualizer
    unless one is selected (``--viz newton_gl``)."""
    from isaaclab_visualizers.newton.newton_visualizer_cfg import NewtonGLVisualizerCfg

    eye, lookat = env0_camera(num_envs, spacing)
    return NewtonGLVisualizerCfg(headless=True, max_visible_envs=4, randomly_sample_visible_envs=False,
                                 window_width=960, window_height=540, enable_live_plots=False, enable_picking=False,
                                 enable_markers=True, eye=eye, lookat=lookat)
FALL_DISTANCE_M = 0.08


# --- kinematics of the export (hand frame at the geometric zero; joints rotate about their anchors) -------------------


def _rot(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def _joint_tf(desc: Description, name: str, deg: float) -> np.ndarray:
    j = desc.joint(name)
    r, p = _rot(j["axis"], math.radians(deg)), np.asarray(j["anchor"], float)
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, p - r @ p
    return m


def thumb_pad_pose(desc: Description, deg: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:  # noqa: D417
    """The thumb pad's face centre and outward normal in the hand frame at thumb joint angles ``deg``."""
    m = np.eye(4)
    for name in THUMB:
        m = m @ _joint_tf(desc, name, deg[name])
    box = thumb_pad_box(desc)
    rot = np.array(_quat_wxyz_to_matrix(box["quat_wxyz"]))
    face = np.asarray(box["center"]) + rot[:, 1] * box["half_extents"][1]
    return (m @ np.append(face, 1.0))[:3], m[:3, :3] @ rot[:, 1]


def _quat_wxyz_to_matrix(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _matrix_to_quat_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    w = 0.5 * math.sqrt(max(1e-12, 1.0 + np.trace(r)))
    return tuple(float(v) for v in ((r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w),
                                     (r[1, 0] - r[0, 1]) / (4 * w), w))


def distal_wall_along(desc: Description, mcp_deg: float = GRASP_MCP_DEG, pip_deg: float = GRASP_PIP_DEG) -> float:
    """Where the middle finger's distal pad face meets the proximal pad face plane at the grasp, along the face from the
    MCP axis's foot [m] (O: 44 mm, the distal pads standing at 90 deg at the end of the proximal pad)."""
    origin, rot9 = face_frame(desc, mcp_deg)
    origin, rot = np.asarray(origin), np.asarray(rot9).reshape(3, 3)
    from code_isaaclab.hands.code_variants import HAND_ROOT_POS

    pose = desc.closed_pose("middle", pip_deg)
    m = (_joint_tf(desc, "middle_fe", mcp_deg) @ _joint_tf(desc, "middle_pip_servo", pose["middle_pip_servo"])
         @ _joint_tf(desc, "middle_b", pose["middle_b"]))
    box = next(b for b in desc.body("middle_distal")["boxes"] if b["name"] == "middle_distal_pad")
    c, h = np.asarray(box["center"]), box["half_extents"]
    ends = [rot.T @ (np.asarray(HAND_ROOT_POS) + np.diag([1.0, -1.0, -1.0]) @ (m @ np.array([c[0] + sx * h[0], c[1],
            c[2] - h[1], 1.0]))[:3] - origin) for sx in (-1.0, 1.0)]
    (x0, _, z0), (x1, _, z1) = ends
    return float(x0 + (x1 - x0) * (0.0 - z0) / (z1 - z0))  # the face line at height 0


def pinch_grasp(desc: Description, mcp_deg: float = GRASP_MCP_DEG) -> dict:
    """The grasp: the face frame (env frame); the cube's rest centre (face frame ``(wall - WALL_GAP_M - size/2, 0,
    size/2)``, a face on the proximal pads, its far face short of the distal pads' wall, :func:`distal_wall_along`) and its rest pose in the env frame; the thumb's MCP and IP angles (0.25 deg grid) that put the
    thumb pad's face centre on the cube's top face, ``CUBE_SIZE_M - PINCH_SQUEEZE_M`` above the proximal pad face, as
    close above the cube's centre as the thumb reaches, its normal within 20 deg of straight down."""
    origin, rot9 = face_frame(desc, mcp_deg)
    origin, rot = np.asarray(origin), np.asarray(rot9).reshape(3, 3)  # columns: face axes in the env frame
    root = np.diag([1.0, -1.0, -1.0])  # palm up (hand -> env frame); the hand root at code_variants.HAND_ROOT_POS
    from code_isaaclab.hands.code_variants import HAND_ROOT_POS

    height = CUBE_SIZE_M - PINCH_SQUEEZE_M
    rest_along = distal_wall_along(desc, mcp_deg) - WALL_GAP_M - CUBE_SIZE_M / 2
    best = None
    for tm in np.arange(0.0, 100.0, 0.25):
        for ti in np.arange(-10.0, 100.0, 0.25):
            c, n = thumb_pad_pose(desc, {**GRASP_THUMB_DEG, "thumb_mcp": tm, "thumb_ip": ti})
            cf = rot.T @ (np.asarray(HAND_ROOT_POS) + root @ c - origin)
            nf = rot.T @ (root @ n)
            if abs(cf[2] - height) > 2.5e-4 or nf[2] > -math.cos(math.radians(20.0)) or abs(cf[1]) > 0.008:
                continue
            score = abs(cf[0] - rest_along)
            if best is None or score < best[0]:
                best = (score, float(tm), float(ti), cf, nf)
    if best is None or best[0] > CUBE_SIZE_M / 2 - 0.004:
        raise ValueError("the thumb pad cannot press the cube's top face in this grasp")
    _, tm, ti, cf, nf = best
    rest_face = (rest_along, 0.0, CUBE_SIZE_M / 2)
    rest_env = origin + rot @ np.asarray(rest_face)
    return {"thumb_mcp_deg": tm, "thumb_ip_deg": ti, "thumb_pad_face": [float(v) for v in cf],
            "thumb_normal_face": [float(v) for v in nf], "face_origin": tuple(float(v) for v in origin),
            "face_rot": tuple(float(v) for v in rot9), "rest_face": rest_face,
            "rest_env": tuple(float(v) for v in rest_env), "rest_quat_xyzw": _matrix_to_quat_xyzw(rot)}


def grasp_joint_pos(desc: Description, thumb_mcp_deg: float, thumb_ip_deg: float) -> dict[str, float]:
    """Every joint at the grasp [rad]: MCP, PIP (four-bar closed from the export's tables), abduction 0, the thumb."""
    pose = {j["name"]: 0.0 for j in desc.joints}
    for f in FINGERS:
        pose[f"{f}_fe"] = math.radians(GRASP_MCP_DEG)
        for joint, deg in desc.closed_pose(f, GRASP_PIP_DEG).items():
            pose[joint] = math.radians(deg)
    for name, deg in {**GRASP_THUMB_DEG, "thumb_mcp": thumb_mcp_deg, "thumb_ip": thumb_ip_deg}.items():
        pose[name] = math.radians(deg)
    return pose


def pad_points(desc: Description) -> tuple[tuple[float, float, float], ...]:
    """Observed pad-face points in their bodies' frames: the three distal pads' face below the fingertip
    (:func:`distal_face_points`) and the thumb pad's face centre (bodies: :data:`PAD_POINT_BODIES`)."""
    box = thumb_pad_box(desc)
    rot = _quat_wxyz_to_matrix(box["quat_wxyz"])
    face = np.asarray(box["center"]) + rot[:, 1] * box["half_extents"][1] - np.asarray(desc.body("thumb_distal")["origin"])
    return (*distal_face_points(desc), tuple(float(v) for v in face))


PAD_POINT_BODIES = [f"{f}_distal" for f in FINGERS] + ["thumb_distal"]


def goal_quat(env, command_name: str) -> torch.Tensor:
    """The goal orientation (x, y, z, w): the command without its position, a constant of the hand."""
    return env.command_manager.get_command(command_name)[:, 3:7]


@configclass
class CoDeReorientCubeEnvCfg(CoDeReorientMainEnvCfg):
    """Cube reorientation by hand ``hand_key`` (O, R, U) with a thumb pad; see the module docstring."""

    hand: str = "reorient_O_thumbpad"
    hand_key: str = "O"
    visuals: bool = os.environ.get("CODE_REORIENT_O_VISUALS", "0") == "1"
    """Build the hand with drawing-only meshes (servo bodies, colours) for play and eval renders: set
    ``CODE_REORIENT_O_VISUALS=1`` (read when the config is built; a Hydra override would come after the hand is
    registered). The colliders, masses and joints are the same; training uses the plain asset."""
    face_origin: tuple | None = None
    face_rot: tuple | None = None
    rest_face: tuple | None = None
    distal_face_points: tuple | None = None
    grasp: dict | None = None

    def __post_init__(self):
        desc_path = HANDS[self.hand_key]
        desc = Description.load(desc_path)
        g = pinch_grasp(desc)
        self.grasp = {k: v for k, v in g.items() if k in ("thumb_mcp_deg", "thumb_ip_deg", "thumb_pad_face", "rest_face", "rest_env")}
        self.hand = register_description_hand(
            f"reorient_{self.hand_key}_thumbpad" + ("_vis" if self.visuals else ""), desc_path,
            RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/object",
                spawn=sim_utils.CuboidCfg(
                    size=(CUBE_SIZE_M,) * 3, rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
                    mass_props=sim_utils.MassCfg(mass=CUBE_MASS_KG), collision_props=sim_utils.UsdPhysicsCollisionCfg(),
                    activate_contact_sensors=True, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.5, 0.2))),
                init_state=RigidObjectCfg.InitialStateCfg(pos=g["rest_env"], rot=g["rest_quat_xyzw"])),
            servo_saturation=(True, True), thumb_pad=True, visuals=self.visuals)
        super().__post_init__()
        spec = get_hand(self.hand)
        self.scene.robot = spec.robot_cfg.replace(
            init_state=spec.robot_cfg.init_state.replace(joint_pos=grasp_joint_pos(desc, g["thumb_mcp_deg"], g["thumb_ip_deg"])))
        self.face_origin, self.face_rot, self.rest_face = g["face_origin"], g["face_rot"], g["rest_face"]
        self.distal_face_points = pad_points(desc)
        # goals: turns of the reset orientation about a world axis
        cmd = self.commands.object_pose
        cmd.axis = GOAL_AXIS
        cmd.max_angle = math.radians(GOAL_MAX_DEG)
        cmd.min_step = math.radians(GOAL_MIN_STEP_DEG)
        cmd.base_quat = g["rest_quat_xyzw"]
        cmd.init_pos_offset = (0.0, 0.0, 0.0)
        # actions: upstream's EMA to-limits action, PIP in flexion with the impedance set in flexion
        actuated = list(spec.actuated_joint_names)
        tables = dict(spec.four_bar_tables)
        self.actions.joint_pos = FlexionEMAActionCfg(
            asset_name="robot", joint_names=actuated, alpha=ACTION_EMA_ALPHA, rescale_to_limits=True,
            four_bar_tables=tables, flexion_limits_deg=(0.0, 120.0), flexion_stiffness=FLEXION_STIFFNESS,
            flexion_damping=FLEXION_DAMPING)
        # observations: upstream's order, transfer-ready terms
        policy = self.observations.policy
        joints = SceneEntityCfg("robot", joint_names=actuated, preserve_order=True)
        policy.joint_pos = ObsTerm(func=flexion_joint_pos_normalized,
                                   params={"asset_cfg": joints, "four_bar_tables": tables})
        policy.joint_vel = ObsTerm(func=flexion_joint_vel, scale=0.2, params={"asset_cfg": joints, "four_bar_tables": tables})
        policy.fingertip_pose = None
        policy.fingertip_vel = None
        policy.object_pos = ObsTerm(func=rotate_mdp.object_pos_face, scale=10.0)
        policy.goal_pose = ObsTerm(func=goal_quat, params={"command_name": "object_pose"})
        policy.pad_points = ObsTerm(func=rotate_mdp.distal_pad_points_face, scale=10.0, params={
            "asset_cfg": SceneEntityCfg("robot", body_names=PAD_POINT_BODIES, preserve_order=True)})
        # reset: the cube at its rest pose with little noise (it is pinched), the hand at the grasp
        self.events.reset_object.params["pose_range"] = {
            "x": (-OBJECT_POSITION_NOISE_M, OBJECT_POSITION_NOISE_M), "y": (-OBJECT_POSITION_NOISE_M, OBJECT_POSITION_NOISE_M)}
        self.terminations.object_out_of_reach.params["threshold"] = FALL_DISTANCE_M
        # video: the camera follows the scene's environment count only if set again (scripts/reorient_o/train.py does)
        self.sim.default_visualizer_cfg = recording_visualizer_cfg(self.scene.num_envs, self.scene.env_spacing)

    def play_mode(self) -> None:
        """Upstream's play mode, keeping the recording visualizer settings (upstream replaces them with its own)."""
        super().play_mode()
        self.sim.default_visualizer_cfg = recording_visualizer_cfg(self.scene.num_envs, self.scene.env_spacing)


SPIN_STEP_DEG = 30.0
"""Continuous rotation: each goal is the previous one turned this far about +z. +z is the direction the probes turned
furthest (ring PIP +60: +16.7 / +19.6 deg; abduction -15: +15 deg; against at most -10.3 deg)."""
SPIN_SUCCESS_THRESHOLD = 0.1
"""Success threshold [rad] (upstream's tighter value): each goal then needs at least 30 - 2 x 5.7 = 18.5 deg of turning."""
FALL_PENALTY = -250.0
"""One success bonus, once, when the cube drops (``object_out_of_reach``; not on time-out). Upstream Allegro's fall penalty
is 0; with it 0 the first Spin run learned to turn the cube twice and flick it out (episodes 0.4 s, all drops)."""
SMOOTHNESS_WEIGHTS = {"joint_vel_l2": -0.2, "joint_acc_l2": -5.0e-4, "action_rate_l2": -0.25}
"""Isaac Lab terms on the actuated joints (the manager scales each by the step dt, so a weight w costs w x value per
second). Calibrated on the first policy's ~6 Hz shaking (per-step means: joint_vel_l2 51.4 (rad/s)^2, joint_acc_l2
6.83e4 (rad/s^2)^2, action_rate_l2 39.5): about 10 + 34 + 10 = 54 per second there, a fifth of a success bonus every
second (540 per episode, about half that policy's return). A smooth gait at XM335-like speeds (1 Hz, 20 deg on 8 joints:
about 19, 760 and 0.04) costs about 4 + 0.4 + 0.01 per second. The action-rate share is kept small: it also prices the
exploration noise of an untrained policy."""


@configclass
class CoDeReorientCubeSpinEnvCfg(CoDeReorientCubeEnvCfg):
    """Continuous rotation about +z: on success the goal turns on by :data:`SPIN_STEP_DEG`, with no range limit; success
    within :data:`SPIN_SUCCESS_THRESHOLD`; upstream's rewards plus the smoothness terms (:data:`SMOOTHNESS_WEIGHTS`) and a fall penalty (:data:`FALL_PENALTY`)."""

    visuals: bool = os.environ.get("CODE_REORIENT_O_VISUALS", "1") == "1"
    """The hand asset with visual-only meshes by default (side plates, links, servos, coloured pads; colliders, masses
    and joints the same as the plain asset); ``CODE_REORIENT_O_VISUALS=0`` builds the plain one."""

    def __post_init__(self):
        super().__post_init__()
        cmd = self.commands.object_pose
        cmd.continuous_step = math.radians(SPIN_STEP_DEG)
        cmd.orientation_success_threshold = SPIN_SUCCESS_THRESHOLD
        joints = SceneEntityCfg("robot", joint_names=list(self.actions.joint_pos.joint_names), preserve_order=True)
        self.rewards.joint_vel_l2 = RewTerm(func=base_mdp.joint_vel_l2, weight=SMOOTHNESS_WEIGHTS["joint_vel_l2"],
                                            params={"asset_cfg": joints})
        self.rewards.joint_acc_l2 = RewTerm(func=base_mdp.joint_acc_l2, weight=SMOOTHNESS_WEIGHTS["joint_acc_l2"],
                                            params={"asset_cfg": joints})
        self.rewards.action_rate_l2 = RewTerm(func=base_mdp.action_rate_l2, weight=SMOOTHNESS_WEIGHTS["action_rate_l2"])
        # a drop costs one success bonus, once (the manager multiplies by the step dt); time-outs cost nothing
        self.rewards.fall_penalty = RewTerm(func=base_mdp.is_terminated_term, weight=FALL_PENALTY / self.step_dt_s(),
                                            params={"term_keys": "object_out_of_reach"})

    def step_dt_s(self) -> float:
        return self.sim.dt * self.decimation


REACH_MIN_STEP_DEG = 40.0
"""Reachable-set goals: consecutive goals at least this far apart within +-GOAL_MAX_DEG, so they alternate between the
range's halves and each needs at least 40 - 2 x 5.7 = 28.5 deg of turning (from the reset orientation, 0, no goal lies
40 deg away: the sampler then takes the far end, +-30 deg, as the first goal)."""


@configclass
class CoDeReorientCubeReachEnvCfg(CoDeReorientCubeSpinEnvCfg):
    """Upstream-style goals within the hand's reachable set: random yaw about z, uniform in +-:data:`GOAL_MAX_DEG` (the
    probes' range), consecutive goals at least :data:`REACH_MIN_STEP_DEG` apart; otherwise the Spin task (success at
    0.1 rad, smoothness and fall penalties)."""

    def __post_init__(self):
        super().__post_init__()
        cmd = self.commands.object_pose
        cmd.continuous_step = None
        cmd.max_angle = math.radians(GOAL_MAX_DEG)
        cmd.min_step = math.radians(REACH_MIN_STEP_DEG)


@configclass
class CoDeReorientCubeOReachEnvCfg(CoDeReorientCubeReachEnvCfg):
    hand_key = "O"


@configclass
class CoDeReorientCubeRReachEnvCfg(CoDeReorientCubeReachEnvCfg):
    hand: str = "reorient_R_thumbpad"
    hand_key = "R"


@configclass
class CoDeReorientCubeUReachEnvCfg(CoDeReorientCubeReachEnvCfg):
    hand: str = "reorient_U_thumbpad"
    hand_key = "U"


@configclass
class CoDeReorientCubeOSpinEnvCfg(CoDeReorientCubeSpinEnvCfg):
    hand_key = "O"


@configclass
class CoDeReorientCubeRSpinEnvCfg(CoDeReorientCubeSpinEnvCfg):
    hand: str = "reorient_R_thumbpad"
    hand_key = "R"


@configclass
class CoDeReorientCubeUSpinEnvCfg(CoDeReorientCubeSpinEnvCfg):
    hand: str = "reorient_U_thumbpad"
    hand_key = "U"


@configclass
class CoDeReorientCubeOEnvCfg(CoDeReorientCubeEnvCfg):
    hand_key = "O"


@configclass
class CoDeReorientCubeREnvCfg(CoDeReorientCubeEnvCfg):
    hand: str = "reorient_R_thumbpad"
    hand_key = "R"


@configclass
class CoDeReorientCubeUEnvCfg(CoDeReorientCubeEnvCfg):
    hand: str = "reorient_U_thumbpad"
    hand_key = "U"


__all__ = ["CoDeReorientCubeOReachEnvCfg", "CoDeReorientCubeReachEnvCfg", "CoDeReorientCubeRReachEnvCfg",
           "CoDeReorientCubeUReachEnvCfg", "CoDeReorientCubeOSpinEnvCfg", "CoDeReorientCubeRSpinEnvCfg", "CoDeReorientCubeSpinEnvCfg",
           "CoDeReorientCubeUSpinEnvCfg", "env0_camera", "recording_visualizer_cfg", "CoDeReorientCubeEnvCfg", "CoDeReorientCubeOEnvCfg", "CoDeReorientCubeREnvCfg", "CoDeReorientCubeUEnvCfg",
           "grasp_joint_pos", "pad_points", "pinch_grasp", "thumb_pad_pose"]
