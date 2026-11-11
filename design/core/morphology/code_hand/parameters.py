"""Independent design variables of the CoDe hand (the parametric model's L1).

Frames and conventions:
- **Hand frame H**: the Onshape ``hand-layout`` assembly origin. ``+X`` distal, ``+Y`` toward the thumb (the
  ordinary FE axes), ``+Z`` dorsal. Millimetres and degrees.
- **Servo frame**: the actuator's centre frame. ``+Z`` is the output axis, the origin is on that axis at the
  body's mid-plane (the X330 body is symmetric about it, +-11 mm along the axis), axes as the X330 part frame.
  It is the X330 part frame moved 5.5 mm along ``-Z``; the CAD's servo mate connectors sit there.
- **Finger frame**: at the finger's base (MCP-AA) servo centre, ``+X`` distal along the finger's heading,
  ``+Z`` dorsal. The base servo frame is the finger frame turned -90° about ``Z``.
- **Linkage plane**: the finger's ``X-Z`` plane through the servo centres, ``O`` on the MCP-FE axis, ``+X``
  along ``O->A``.

A digit's base pose is the pose of its first servo frame in H: the MCP-AA servo for a finger, the CMC-RO
servo for the thumb. Joint configurations are kept apart (``HandConfiguration``).
All joint coordinates are CCW positive about local +Z (viewed from +Z toward the origin).
Zero is the geometric reference configuration, never a pose calibrated by hand in CAD.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path

import numpy as np

FINGERS = ("index", "middle", "ring")
DIGITS = (*FINGERS, "thumb")


def _rot(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    m = np.eye(4)
    i, j = {"x": (1, 2), "y": (2, 0), "z": (0, 1)}[axis]
    m[i, i], m[i, j], m[j, i], m[j, j] = c, -s, s, c
    return m


def rx(deg: float) -> np.ndarray:
    return _rot("x", deg)


def ry(deg: float) -> np.ndarray:
    return _rot("y", deg)


def rz(deg: float) -> np.ndarray:
    return _rot("z", deg)


def trans(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def _finite(obj) -> None:
    for f in fields(obj):
        value = getattr(obj, f.name)
        if isinstance(value, float | int) and not math.isfinite(value):
            raise ValueError(f"{type(obj).__name__}.{f.name} must be finite")


@dataclass(frozen=True)
class Pose:
    """A pose in H [mm, deg]; orientation is intrinsic Z-Y-X: ``R = Rz(yaw) Ry(pitch) Rx(roll)``."""

    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0

    def __post_init__(self) -> None:
        _finite(self)

    def matrix(self) -> np.ndarray:
        return trans(self.x_mm, self.y_mm, self.z_mm) @ rz(self.yaw_deg) @ ry(self.pitch_deg) @ rx(self.roll_deg)


@dataclass(frozen=True)
class FingerParameters:
    """One ordinary finger; the three fingers share it.

    - MCP-FE axis: ``fe_axis_*`` from the base servo centre, in the finger frame.
    - Four-bar, in the linkage plane: ``A`` and ``A'`` on the proximal segment, the driver ``A'-B`` (the PIP
      servo shaft is on ``A'``), the coupler ``A-B'``. ``B'`` and ``C`` are given in the distal frame (origin
      ``B``, ``+X`` along ``B->C``). These are the Onshape ``Linkage Dimensions`` variables.
    - Servos: the MCP-FE servo centre is ``O``, the PIP servo centre is ``A'``, both in the linkage plane;
      ``pip_servo_tilt_deg`` is the dorsal tilt of the rear body from -X, matching the study convention.
    """

    fe_axis_x_mm: float = 23.5
    fe_axis_z_mm: float = -16.75
    a_x_mm: float = 35.7
    a_z_mm: float = 0.0
    ap_x_mm: float = 37.5
    ap_z_mm: float = 12.4
    apb_mm: float = 32.2
    abp_mm: float = 25.6
    bp_x_mm: float = -6.7
    bp_z_mm: float = 6.5
    c_x_mm: float = 22.8
    c_z_mm: float = 0.0
    pip_servo_tilt_deg: float = 15.0

    def __post_init__(self) -> None:
        _finite(self)
        for name in ("apb_mm", "abp_mm", "c_x_mm"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"FingerParameters.{name} must be positive")


@dataclass(frozen=True)
class ThumbParameters:
    """The thumb's four serial joints: CMC-RO, CMC-RAA, MCP-FE and IP-FE, each the axis of its servo.

    - The RAA axis is perpendicular to the RO axis and meets it at the RAA servo centre, ``ro_to_raa_mm``
      along RO from the RO servo centre.
    - The MCP axis is perpendicular to RAA and meets it at the MCP servo centre, ``raa_to_mcp_mm`` along RAA
      from the RAA servo centre.
    - The IP axis is parallel to MCP, ``mcp_to_ip_mm`` away along the MCP servo's ``-Y``. At geometric
      zero, MCP -Y aligns with RAA +Z, so the RAA->MCP and MCP->IP vectors point along the same line.
    - The end-effector frame ``EE`` has the IP servo frame's axes and sits at ``(ee_x, ee_y, ee_z)`` in it,
      turned by the IP angle. The prototype puts it on the thumb's centre line, straight out of the MCP-IP
      line, at the fingertip case's end (``ee_y`` from the CAD ``Thumb EE`` connector, which sits 4.5 mm
      toward the pad: ``ee_x`` = -4.5 there).
    - ``raa_to_mcp_clocking_deg``: fixed mounting rotation about RAA +Z before the perpendicular MCP
      axis. The nominal 90 degrees sets the reference chain geometry; it is not a fitted joint offset.
      A further fixed MCP Rz(90) aligns the MCP-IP line with the RAA-MCP line. At MCP = IP = 0,
      the RAA, MCP, IP and on-centre tip are collinear, independently of the CAD's posed angles.
    """

    ro_to_raa_mm: float = 26.0
    raa_to_mcp_mm: float = 26.5
    mcp_to_ip_mm: float = 35.5
    ee_x_mm: float = 0.0
    ee_y_mm: float = -53.5
    ee_z_mm: float = 0.0
    raa_to_mcp_clocking_deg: float = 90.0

    def __post_init__(self) -> None:
        _finite(self)


@dataclass(frozen=True)
class HandLayout:
    """Base pose of each digit's first servo in H."""

    index: Pose
    middle: Pose
    ring: Pose
    thumb: Pose

    def base(self, digit: str) -> Pose:
        return getattr(self, digit)


@dataclass(frozen=True)
class HandParameters:
    layout: HandLayout
    finger: FingerParameters = field(default_factory=FingerParameters)
    thumb: ThumbParameters = field(default_factory=ThumbParameters)

    @classmethod
    def from_dict(cls, data: dict) -> HandParameters:
        layout = HandLayout(**{d: Pose(**data["layout"][d]) for d in DIGITS})
        return cls(layout, FingerParameters(**data.get("finger", {})), ThumbParameters(**data.get("thumb", {})))

    def to_dict(self) -> dict:
        return {"layout": {d: asdict(self.layout.base(d)) for d in DIGITS},
                "finger": asdict(self.finger), "thumb": asdict(self.thumb)}


@dataclass(frozen=True)
class FingerConfiguration:
    """Joint angles [deg], all CCW positive about local +Z.

    AA rotates about the base servo axis; FE and PIP about the proximal plane's +Z (the FE axis).
    Zero AA/FE aligns with the nominal finger frame; zero PIP is the extended four-bar with B->C
    along proximal +X. PIP is the distal segment angle, not the four-bar actuator shaft angle.
    """

    aa_deg: float = 0.0
    fe_deg: float = 0.0
    pip_deg: float = 0.0


@dataclass(frozen=True)
class ThumbConfiguration:
    """Joint angles [deg], all CCW positive about their servo's local +Z.

    All zeros select the nominal chain defined by ThumbParameters and the fixed perpendicular-axis
    transforms, with RAA->MCP->IP straight. No CAD-fitted joint offsets are applied.
    """

    ro_deg: float = 0.0
    raa_deg: float = 0.0
    mcp_deg: float = 0.0
    ip_deg: float = 0.0


@dataclass(frozen=True)
class HandConfiguration:
    index: FingerConfiguration = field(default_factory=FingerConfiguration)
    middle: FingerConfiguration = field(default_factory=FingerConfiguration)
    ring: FingerConfiguration = field(default_factory=FingerConfiguration)
    thumb: ThumbConfiguration = field(default_factory=ThumbConfiguration)

    @classmethod
    def from_dict(cls, data: dict) -> HandConfiguration:
        return cls(**{d: FingerConfiguration(**data.get(d, {})) for d in FINGERS},
                   thumb=ThumbConfiguration(**data.get("thumb", {})))

    def to_dict(self) -> dict:
        return {d: asdict(getattr(self, d)) for d in DIGITS}


PROTOTYPE = Path(__file__).with_name("prototype.yaml")


def load(path=PROTOTYPE) -> tuple[HandParameters, HandConfiguration]:
    """Return model parameters and the separately recorded CAD observation.

    The second return value is not the model's zero/default configuration. Use HandConfiguration()
    or CoDeHand(parameters).frames() for the geometric zero.
    """
    import yaml

    data = yaml.safe_load(Path(path).read_text())
    return HandParameters.from_dict(data), HandConfiguration.from_dict(data.get("cad_configuration", {}))
