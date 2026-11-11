"""Bind the CoDe parametric model to the Onshape CAD.

- ``CadMap``: which CAD occurrence and mate is which model servo and axis.
- ``layout_from_snapshot`` / ``configuration_from_snapshot``: read the layout and the joint angles off a CAD
  snapshot (``core.cad.snapshot``). The morphology parameters are the model's; CAD only checks them.
- ``compare``: the model's frames against the snapshot, as errors per servo and per axis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from core.cad.layout_mates import frame_error, frame_from_axes, zyx_angles
from core.cad.snapshot import length_mm
from core.morphology.code_hand.model import CoDeHand, pip_from_pins
from core.morphology.code_hand.parameters import (
    DIGITS,
    FINGERS,
    FingerConfiguration,
    FingerParameters,
    HandConfiguration,
    HandLayout,
    HandParameters,
    Pose,
    ThumbConfiguration,
    ry,
    rz,
    trans,
)

# X330 part frame -> servo (actuator centre) frame: the body spans z -16.5..5.5 mm in its part studio.
SERVO_CENTER = trans(z=-5.5)

FINGER_SERVOS = {"aa": "X335-with-bearing <1>/X330 <1>", "fe": "MCP-FE <1>/X330 <3>",
                 "pip": "proximal-phalanx <1>/X330 <1>"}
FINGER_MATES = {"aa": "Revolute 5", "Ap": "Revolute 1", "A": "Revolute 2", "B": "Revolute 3", "Bp": "Revolute 4"}
THUMB_SERVOS = {"ro": "X335-with-output-horn <3>/X330 <1>", "raa": "X335-with-output-horn <1>/X330 <1>",
                "mcp": "X335-with-output-horn <2>/X330 <1>", "ip": "X335-with-bearing <1>/X330 <1>"}
# Servo mate connectors (at the actuator centre, on the axis), under the digit's top-level instance
SERVO_CONNECTORS = {"aa": ":MCP-AA", "fe": "/MCP-FE <1>:O", "ro": ":thumb-A-horn-3", "raa": ":thumb-A-horn-1",
                    "mcp": ":thumb-A-horn-2", "ip": ":thumb-A-bearing-1"}
THUMB_MATES = {"ro": "(root):Revolute 1", "raa": "{thumb}:Revolute 1", "mcp": "{thumb}:Revolute 2"}

# Onshape "Linkage Dimensions" variable -> FingerParameters field
LINKAGE_VARIABLES = {"A_x": "a_x_mm", "A_z": "a_z_mm", "Ap_x": "ap_x_mm", "Ap_z": "ap_z_mm", "ApB": "apb_mm",
                     "ABp": "abp_mm", "Bp_x": "bp_x_mm", "Bp_z": "bp_z_mm", "C_x": "c_x_mm", "C_z": "c_z_mm"}


@dataclass(frozen=True)
class CadMap:
    """Top-level instance name of each digit in the hand-layout assembly."""

    instances: dict[str, str]
    end_effectors: dict[str, str] = field(default_factory=dict)  # model key -> CAD mate connector name

    def servo(self, key: str) -> str:
        digit, joint = key.split("/")
        table = THUMB_SERVOS if digit == "thumb" else FINGER_SERVOS
        return f"{self.instances[digit]}/{table[joint]}"

    def mate(self, key: str) -> str | None:
        digit, joint = key.split("/")
        if digit == "thumb":
            name = THUMB_MATES.get(joint)
            return name.format(thumb=self.instances["thumb"]) if name else None
        name = FINGER_MATES.get(joint)
        return f"{self.instances[digit]}:{name}" if name else None


def frame(entry: dict) -> np.ndarray:
    return frame_from_axes(entry["origin_mm"], entry["x_axis"], entry["z_axis"])


def pose_from_matrix(m: np.ndarray, digits: int = 4) -> Pose:
    """A Pose from a 4x4 frame, rounded to ``digits`` decimals (drops float noise and -0.0)."""
    angles = zyx_angles(m[:3, :3])
    return Pose(*(round(float(v), digits) + 0.0 for v in (*m[:3, 3], *angles)))


def servo_frame(snapshot: dict, cad: CadMap, key: str) -> np.ndarray:
    """A model servo frame (actuator centre) from the snapshot's X330 part frame."""
    return frame(snapshot["servos"][cad.servo(key)]) @ SERVO_CENTER


def layout_from_snapshot(snapshot: dict, cad: CadMap) -> HandLayout:
    first = {d: ("thumb/ro" if d == "thumb" else f"{d}/aa") for d in DIGITS}
    return HandLayout(**{d: pose_from_matrix(servo_frame(snapshot, cad, k)) for d, k in first.items()})


def linkage_from_variables(snapshot: dict, studio: str = "Linkage Dimensions") -> dict:
    """The FingerParameters fields that the Onshape Variable Studio sets (blank expressions read as 0)."""
    variables = snapshot["variables"][studio]
    return {field: length_mm(variables.get(name, "")) or 0.0 for name, field in LINKAGE_VARIABLES.items()}


def _angle_about(axis: np.ndarray, ref: np.ndarray, cur: np.ndarray) -> float:
    return math.degrees(math.atan2(axis @ np.cross(ref, cur), ref @ cur))


def configuration_from_snapshot(snapshot: dict, cad: CadMap, parameters: HandParameters) -> HandConfiguration:
    s = {k: servo_frame(snapshot, cad, k) for d in FINGERS for k in (f"{d}/aa", f"{d}/fe", f"{d}/pip")}
    s.update({f"thumb/{j}": servo_frame(snapshot, cad, f"thumb/{j}") for j in ("ro", "raa", "mcp")})
    fingers = {}
    for d in FINGERS:
        base, fe, pip = s[f"{d}/aa"], s[f"{d}/fe"], s[f"{d}/pip"]
        # aa: the FE axis (fe +Z) turned about the base axis from its zero direction (base frame -X, i.e. H +Y)
        aa = _angle_about(base[:3, 2], (base @ rz(90.0))[:3, 1], fe[:3, 2])
        # fe: the PIP servo's x turned about the FE axis from its zero-pose direction
        zero = (fe @ trans(parameters.finger.ap_x_mm, -parameters.finger.ap_z_mm)
                @ rz(parameters.finger.pip_servo_tilt_deg - 90.0))
        fe_deg = _angle_about(fe[:3, 2], zero[:3, 0], pip[:3, 0])
        # pip: from the B and B' pins, in the linkage plane of the posed proximal frame
        proximal = fe @ rz(fe_deg)
        inv = np.linalg.inv(proximal)

        def plane(key: str, inv=inv):
            point = inv @ np.r_[snapshot["revolutes"][cad.mate(key)]["origin_mm"], 1.0]
            return np.array([point[0], -point[1]])

        pip_deg = pip_from_pins(parameters.finger, plane(f"{d}/B"), plane(f"{d}/Bp"))
        fingers[d] = FingerConfiguration(*(round(v, 6) + 0.0 for v in (aa, fe_deg, pip_deg)))

    p = parameters.thumb
    ro, raa, mcp = s["thumb/ro"], s["thumb/raa"], s["thumb/mcp"]
    z_raa = np.linalg.inv(ro)[:3, :3] @ raa[:3, 2]  # = Rz(ro) (-1, 0, 0)
    ro_raw = math.degrees(math.atan2(-z_raa[1], -z_raa[0]))
    z_mcp = np.linalg.inv(raa)[:3, :3] @ mcp[:3, 2]
    raa_raw = math.degrees(math.atan2(-z_mcp[1], -z_mcp[0]))
    partial = raa @ rz(raa_raw) @ trans(z=p.raa_to_mcp_mm) @ ry(-90.0)
    mcp_raw = _angle_about(partial[:3, 2], partial[:3, 0], mcp[:3, 0])
    wrap = lambda deg: math.remainder(deg, 360.0)
    ro_deg = wrap(ro_raw)
    raa_deg = wrap(raa_raw - p.raa_to_mcp_clocking_deg)
    mcp_deg = wrap(mcp_raw - 90.0)  # geometric zero has RAA->MCP->IP straight
    thumb = ThumbConfiguration(*(round(v, 6) + 0.0 for v in (ro_deg, raa_deg, mcp_deg)), 0.0)
    return HandConfiguration(**fingers, thumb=thumb)


def _line_error(model: tuple, cad_origin, cad_axis) -> tuple[float, float]:
    """Distance [mm] of the CAD axis point from the model axis line, and the angle [deg] between them."""
    point, direction = model
    cad_origin, cad_axis = np.asarray(cad_origin), np.asarray(cad_axis)
    offset = cad_origin - point
    distance = float(np.linalg.norm(offset - (offset @ direction) * direction))
    angle = math.degrees(math.acos(min(1.0, abs(float(direction @ cad_axis)))))
    return distance, angle


def compare(parameters: HandParameters, configuration: HandConfiguration, snapshot: dict,
            cad: CadMap) -> dict[str, tuple[float, float]]:
    """``{key: (mm, deg)}``: servo and end-effector frames (``servo:``, ``ee:``) by position and rotation;
    servo mate connectors (``connector:``) by origin distance and axis angle; revolute axes (``axis:``) by line
    distance and angle."""
    frames = CoDeHand(parameters).frames(configuration)
    errors = {f"servo:{k}": frame_error(m, servo_frame(snapshot, cad, k)) for k, m in frames.servos.items()}
    for key, m in frames.servos.items():
        digit, joint = key.split("/")
        connector = f"{cad.instances[digit]}{SERVO_CONNECTORS[joint]}" if joint in SERVO_CONNECTORS else None
        if connector in snapshot.get("connectors", {}):
            entry = snapshot["connectors"][connector]
            angle = _line_error((m[:3, 3], m[:3, 2]), entry["origin_mm"], entry["z_axis"])[1]
            errors[f"connector:{key}"] = (float(np.linalg.norm(m[:3, 3] - entry["origin_mm"])), angle)
    for key, m in frames.ends.items():
        name = cad.end_effectors.get(key)
        if name and f"(root):{name}" in snapshot.get("connectors", {}):
            errors[f"ee:{key}"] = frame_error(m, frame(snapshot["connectors"][f"(root):{name}"]))
    for key, line in frames.axes.items():
        mate = cad.mate(key)
        if mate and mate in snapshot["revolutes"]:
            entry = snapshot["revolutes"][mate]
            errors[f"axis:{key}"] = _line_error(line, entry["origin_mm"], entry["axis"])
    return errors


def finger_parameters_with_cad_linkage(snapshot: dict, finger: FingerParameters) -> FingerParameters:
    """``finger`` with its four-bar replaced by the CAD Variable Studio values."""
    return FingerParameters(**{**finger.__dict__, **linkage_from_variables(snapshot)})
