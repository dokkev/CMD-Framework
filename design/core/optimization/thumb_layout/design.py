"""The thumb-layout design variables and what they fix in the hand model.

- ``cmc_x_mm``, ``cmc_z_mm``: the CMC (RO) servo frame's position in H (the thumb's base ``Pose``); ``y`` and the
  orientation stay the prototype's.
- ``ip_orientation``: where the IP servo body sits.
  ``"distal"`` (a, the prototype): the body lies beyond the IP axis, in the distal segment; the proximal segment holds
  the MCP servo body and the IP servo's rounded cap: ``mcp_to_ip = rear + r + gap`` (25.5 + 9.5 + 0.5 = 35.5 mm, the
  prototype), and the fingertip case starts at the IP servo's rear face, ``rear`` (25.5 mm) from the IP axis.
  ``"proximal"`` (b, turned 180 deg about the IP axis): the IP body lies back toward the MCP, rear face to rear face
  with the MCP servo body: ``mcp_to_ip = 2 rear + gap`` (51.5 mm); the distal segment is the horn bracket round the
  cap and the case, which starts ``r + gap`` (10 mm) from the IP axis.
  ``r`` (9.5), ``rear`` (25.5) are the XM335 packaging profile (``core.hardware.xm335``); ``gap`` (0.5 mm) is the
  prototype's (35.5 - 25.5 - 9.5). Turning the servo about its own output axis keeps the flexion sense.
- ``base_*_deg`` (exploratory, beyond the original list): offsets of the CMC servo's orientation.
- ``extension_mm``: the fingertip case's length past the servo (a) or past the cap bracket (b); the end effector sits
  at ``case_start + extension`` from the IP axis. The prototype's is 53.5 - 25.5 = 28.0 mm.

The thumb pad (``sim/isaaclab``'s ``THUMB_PAD_MM``: 3 mm thick, mounted 10 mm off the centre line, 25 mm long, on the
case only) covers the case's last ``min(25, extension)`` mm; its contact face is 13 mm off the centre line on the
IP servo frame's ``+X`` side (the side IP flexion moves toward).
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from core.hardware.xm335 import XM335Actuator
from core.morphology.code_hand import HandParameters, Pose

SERVO = XM335Actuator().geometry
SERVO_R_MM = SERVO.radius_mm
SERVO_REAR_MM = SERVO.rear_extent_mm
SERVO_GAP_MM = 0.5
"""Between the MCP servo body and the IP servo on the prototype (35.5 - 25.5 - 9.5)."""
PAD_THICKNESS_MM, PAD_MOUNT_MM, PAD_LENGTH_MM = 3.0, 10.0, 25.0
PAD_FACE_MM = PAD_MOUNT_MM + PAD_THICKNESS_MM
ORIENTATIONS = ("distal", "proximal")
BEND_FROM_MCP_MM = SERVO_REAR_MM + SERVO_GAP_MM / 2.0
"""Where a bent proximal phalanx bends: between the MCP servo body (its rear face, 25.5 mm from the MCP axis) and the
next body."""


@dataclass(frozen=True)
class ThumbDesign:
    cmc_x_mm: float
    cmc_z_mm: float
    extension_mm: float
    ip_orientation: str = "distal"
    base_yaw_deg: float = 0.0
    base_pitch_deg: float = 0.0
    base_roll_deg: float = 0.0
    """Exploratory (beyond the original variable list): offsets added to the prototype base orientation's
    intrinsic Z-Y-X angles (yaw -90, pitch 0, roll -90)."""
    proximal_bend_deg: float = 0.0
    """Exploratory: a fixed bend of the proximal phalanx where the MCP and IP servo bodies meet
    (:data:`BEND_FROM_MCP_MM` from the MCP axis), about an axis parallel to the flexion axes; positive in the flexion
    sense, negative bends the IP end dorsally (a built-in hyperextension)."""

    def __post_init__(self) -> None:
        if self.ip_orientation not in ORIENTATIONS:
            raise ValueError(f"ip_orientation must be one of {ORIENTATIONS}")
        if self.extension_mm <= 0.0:
            raise ValueError("extension_mm must be positive")

    @property
    def mcp_to_ip_mm(self) -> float:
        if self.ip_orientation == "distal":
            return SERVO_REAR_MM + SERVO_R_MM + SERVO_GAP_MM
        return 2.0 * SERVO_REAR_MM + SERVO_GAP_MM

    @property
    def case_start_mm(self) -> float:
        return SERVO_REAR_MM if self.ip_orientation == "distal" else SERVO_R_MM + SERVO_GAP_MM

    @property
    def ee_mm(self) -> float:
        """IP axis to the end effector (the case's end), along the segment."""
        return self.case_start_mm + self.extension_mm

    @property
    def pad_length_mm(self) -> float:
        return min(PAD_LENGTH_MM, self.extension_mm)

    @property
    def pad_along_mm(self) -> float:
        """IP axis to the pad centre, along the segment."""
        return self.ee_mm - self.pad_length_mm / 2.0

    @classmethod
    def prototype(cls, parameters: HandParameters) -> ThumbDesign:
        base = parameters.layout.thumb
        return cls(base.x_mm, base.z_mm, -parameters.thumb.ee_y_mm - SERVO_REAR_MM, "distal")


def hand_parameters(parameters: HandParameters, design: ThumbDesign) -> HandParameters:
    """``parameters`` with the thumb base, MCP-IP distance and end effector of ``design``."""
    base: Pose = parameters.layout.thumb
    layout = replace(parameters.layout, thumb=replace(
        base, x_mm=float(design.cmc_x_mm), z_mm=float(design.cmc_z_mm), yaw_deg=base.yaw_deg + design.base_yaw_deg,
        pitch_deg=base.pitch_deg + design.base_pitch_deg, roll_deg=base.roll_deg + design.base_roll_deg))
    thumb = replace(parameters.thumb, mcp_to_ip_mm=design.mcp_to_ip_mm, ee_x_mm=0.0, ee_y_mm=-design.ee_mm, ee_z_mm=0.0)
    return replace(parameters, layout=layout, thumb=thumb)


def palm_arch(parameters: HandParameters, arch_deg: float) -> HandParameters:
    """Exploratory: a curved (arched) finger layout. The middle and ring bases turn about H's x axis (toward the
    fingers) through the index base, by ``arch_deg`` and ``2 arch_deg``; positive turns them palmar, toward the thumb.
    The index stays. The palm housing (``palm_housing``) is rebuilt from the turned servos, so its flat palmar face
    drops with the ring's abduction servo (a conservative stand-in for a curved palm)."""
    if not arch_deg:
        return parameters
    from scipy.spatial.transform import Rotation

    lay = parameters.layout
    pivot = np.array([0.0, lay.index.y_mm, lay.index.z_mm])
    new = {}
    for k, digit in ((1, "middle"), (2, "ring")):
        m = lay.base(digit).matrix()
        turn = Rotation.from_euler("x", k * arch_deg, degrees=True)
        pos = turn.apply(m[:3, 3] - pivot) + pivot
        yaw, pitch, roll = (turn * Rotation.from_matrix(m[:3, :3])).as_euler("ZYX", degrees=True)
        new[digit] = Pose(*(float(v) for v in (*pos, yaw, pitch, roll)))
    return replace(parameters, layout=replace(lay, **new))


def lever_arms(design: ThumbDesign) -> dict[str, float]:
    """The IP and MCP moment arms [mm] of a force along the thumb pad's normal at its centre, thumb straight: the
    pad's distance along the segment from each axis (the pad's 13 mm offset is along the force, so it adds no arm).
    A direct-drive joint's pad force is its torque over this arm."""
    return {"ip": design.pad_along_mm, "mcp": design.mcp_to_ip_mm + design.pad_along_mm}
