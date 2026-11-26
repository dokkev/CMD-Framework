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
All joint coordinates are CCW positive about local +Z (viewed from +Z toward the origin), and flexion is
positive on every digit. Zero is the geometric reference configuration, never a pose calibrated by hand in CAD.

Actuator coordinates are separate: each servo's output angle CCW about its +Z, zero where the CAD's revolute
mate reads 0 (``*_actuator_zero_deg``, set by ``scripts/cad/code_hand.py fit``). A joint without a CAD mate
keeps its geometric zero as the actuator zero.
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
    - Actuator zeros: ``aa``/``fe_actuator_zero_deg`` are the joint angles at the servo's zero; the PIP servo
      turns the link ``A'-B``, so ``pip_actuator_zero_deg`` is that link's direction in the PIP servo frame
      at the servo's zero. The prototype CAD's PIP mate reads 0 with ``A'->B`` along the servo's +Y (90 deg);
      the CAD has no MCP-FE mate, so FE keeps its geometric zero.
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
    aa_actuator_zero_deg: float = 0.0
    fe_actuator_zero_deg: float = 0.0
    pip_actuator_zero_deg: float = 90.0

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
    - The MCP and IP servos are mounted so that +Z rotation flexes the thumb toward its pad, which is the
      IP servo frame's +X side (``model.THUMB_MCP_MOUNT``). The prototype CAD still has them the other way
      round (``core.cad.code_hand.CAD_TURNED_SERVOS``).
    - The end-effector frame ``EE`` has the IP servo frame's axes and sits at ``(ee_x, ee_y, ee_z)`` in it,
      turned by the IP angle. The prototype puts it on the thumb's centre line, straight out of the MCP-IP
      line, at the fingertip case's end (``ee_y`` from the CAD ``Thumb EE`` connector, which sits 4.5 mm
      toward the pad: ``ee_x`` = +4.5 there).
    - ``raa_to_mcp_clocking_deg``: fixed mounting rotation about RAA +Z before the perpendicular MCP
      axis. The nominal 90 degrees sets the reference chain geometry; it is not a fitted joint offset.
      A further fixed MCP mount aligns the MCP-IP line with the RAA-MCP line. At MCP = IP = 0,
      the RAA, MCP, IP and on-centre tip are collinear, independently of the CAD's posed angles.
    - ``*_actuator_zero_deg``: each joint's angle at its servo's zero. The prototype CAD's mates read 0 at
      RO = -90, RAA = +90 and MCP = +90 deg; it has no IP mate, so IP keeps its geometric zero.
    """

    ro_to_raa_mm: float = 26.0
    raa_to_mcp_mm: float = 26.5
    mcp_to_ip_mm: float = 35.5
    ee_x_mm: float = 0.0
    ee_y_mm: float = -53.5
    ee_z_mm: float = 0.0
    raa_to_mcp_clocking_deg: float = 90.0
    ro_actuator_zero_deg: float = -90.0
    raa_actuator_zero_deg: float = 90.0
    mcp_actuator_zero_deg: float = 90.0
    ip_actuator_zero_deg: float = 0.0
    ip_servo_flipped: bool = False
    """The IP servo turned 180 deg about its output axis, its body toward the MCP on the proximal segment (the
    prototype has it beyond the IP axis, in the distal segment). The joint axes and the end effector do not change;
    ``mcp_to_ip_mm`` and ``ee_y_mm`` must leave room for it (``core.optimization.thumb_layout.design``)."""

    def __post_init__(self) -> None:
        _finite(self)


@dataclass(frozen=True)
class PadParameters:
    """The contact pads of an ordinary finger: two plain blocks on the palmar side, as the linkage study lays them.

    Each pad's mounting face lies below its link line (palmar, ``-Z`` in the linkage plane): the proximal pad's
    ``mount_offset_mm`` below, the distal pad's ``distal_mount_offset_mm`` (None: the same as the proximal). The
    study sets each depth to the least that clears that pad's own hardware (``mount_policy:
    shallowest_supported``), so a proximal pad under a low-mounted PIP servo can sit deeper than the distal one.
    The pad fills ``thickness_mm`` below its face, ``width_mm`` across the finger (along the FE axis, centred on
    the linkage plane).
    - Proximal pad: from ``proximal_start_mm`` to ``proximal_end_mm`` along the proximal link line (linkage
      ``x``, from ``O``); the end is the study's ``A''``.
    - Distal pad: from ``distal_start_mm`` to ``distal_end_mm`` in the distal frame (origin ``B``, ``+X`` along
      ``B->C`` in the extended pose); the start is the study's ``B''``, the end is at ``C``.
    """

    proximal_start_mm: float
    proximal_end_mm: float
    distal_start_mm: float
    distal_end_mm: float
    mount_offset_mm: float
    thickness_mm: float
    width_mm: float
    distal_mount_offset_mm: float | None = None

    @property
    def mount_offsets_mm(self) -> tuple[float, float]:
        """(proximal, distal) mounting depths."""
        distal = self.mount_offset_mm if self.distal_mount_offset_mm is None else self.distal_mount_offset_mm
        return self.mount_offset_mm, distal

    def __post_init__(self) -> None:
        _finite(self)
        if self.proximal_end_mm <= self.proximal_start_mm or self.distal_end_mm <= self.distal_start_mm:
            raise ValueError("PadParameters: each pad must end past its start")
        for name in ("thickness_mm", "width_mm"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"PadParameters.{name} must be positive")
        if min(self.mount_offsets_mm) < 0.0:
            raise ValueError("PadParameters: a mount offset must not be negative")


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
    pads: PadParameters | None = None
    """The fingers' contact pads (``None``: not modelled, as in the prototype CAD)."""

    @classmethod
    def from_dict(cls, data: dict) -> HandParameters:
        layout = HandLayout(**{d: Pose(**data["layout"][d]) for d in DIGITS})
        pads = PadParameters(**data["pads"]) if data.get("pads") else None
        return cls(layout, FingerParameters(**data.get("finger", {})), ThumbParameters(**data.get("thumb", {})),
                   pads)

    def to_dict(self) -> dict:
        data = {"layout": {d: asdict(self.layout.base(d)) for d in DIGITS},
                "finger": asdict(self.finger), "thumb": asdict(self.thumb)}
        if self.pads is not None:
            data["pads"] = asdict(self.pads)
        return data


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
    """Joint angles [deg], all CCW positive about their servo's local +Z; MCP and IP positive in flexion.

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

    A file may extend another (``base: <path relative to it>``): its sections update the base's, key by key.
    A variant inherits the base's parameters but not its CAD observation (``cad_configuration``, ``source``),
    which describes the base's CAD only.
    """
    data = _read(Path(path))
    return HandParameters.from_dict(data), HandConfiguration.from_dict(data.get("cad_configuration", {}))


def _read(path: Path) -> dict:
    import yaml

    data = yaml.safe_load(path.read_text())
    if "base" not in data:
        return data
    merged = {k: v for k, v in _read(path.parent / data.pop("base")).items()
              if k not in ("cad_configuration", "source")}
    for key, value in data.items():
        merged[key] = {**merged[key], **value} if isinstance(value, dict) and isinstance(merged.get(key), dict) else value
    return merged
