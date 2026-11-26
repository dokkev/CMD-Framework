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
- ``extension_mm``: the fingertip case's length past the servo (a) or past the cap bracket (b); the end effector sits
  at ``case_start + extension`` from the IP axis. The prototype's is 53.5 - 25.5 = 28.0 mm.

The thumb pad (``sim/isaaclab``'s ``THUMB_PAD_MM``: 3 mm thick, mounted 10 mm off the centre line, 25 mm long, on the
case only) covers the case's last ``min(25, extension)`` mm; its contact face is 13 mm off the centre line on the
IP servo frame's ``+X`` side (the side IP flexion moves toward).
"""

from __future__ import annotations

from dataclasses import dataclass, replace

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


@dataclass(frozen=True)
class ThumbDesign:
    cmc_x_mm: float
    cmc_z_mm: float
    extension_mm: float
    ip_orientation: str = "distal"

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
    layout = replace(parameters.layout, thumb=replace(base, x_mm=float(design.cmc_x_mm), z_mm=float(design.cmc_z_mm)))
    thumb = replace(parameters.thumb, mcp_to_ip_mm=design.mcp_to_ip_mm, ee_x_mm=0.0, ee_y_mm=-design.ee_mm, ee_z_mm=0.0)
    return replace(parameters, layout=layout, thumb=thumb)


def lever_arms(design: ThumbDesign) -> dict[str, float]:
    """The IP and MCP moment arms [mm] of a force along the thumb pad's normal at its centre, thumb straight: the
    pad's distance along the segment from each axis (the pad's 13 mm offset is along the force, so it adds no arm).
    A direct-drive joint's pad force is its torque over this arm."""
    return {"ip": design.pad_along_mm, "mcp": design.mcp_to_ip_mm + design.pad_along_mm}
