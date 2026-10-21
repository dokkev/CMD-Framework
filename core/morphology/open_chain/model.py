"""Forward kinematics of the open-chain (serial 2R) finger.

Two independently actuated revolute joints about ``+Y``: the MCP at ``O``
turns the proximal segment by ``q_mcp``, and the distal FE joint at ``A``
turns the distal segment by ``q_distal`` relative to it. Positive flexion
turns ``+X`` toward ``-Z``. The MCP servo's output axis is on ``O``; the
distal servo's is on ``A``, its housing fixed to the proximal segment.
Actuator packaging is not part of this model.
"""

from __future__ import annotations

from dataclasses import dataclass

from core._validation import require_finite
from core.geometry import RigidTransform
from core.morphology.linkage.parameters import PointXZ
from core.morphology.open_chain.parameters import OpenChainParameters


FLEXION_AXIS = (0.0, 1.0, 0.0)
"""Both joints turn about ``+Y``: a positive angle turns ``+X`` toward ``-Z``."""


@dataclass(frozen=True)
class OpenChainPose:
    """One solved pose of the open-chain finger."""

    q_mcp_rad: float
    q_distal_rad: float
    """Distal flexion relative to the proximal segment."""
    proximal: RigidTransform
    """Proximal segment frame: origin ``O``, ``+X`` along ``OA``. The MCP servo's
    output turns it."""
    distal_joint: RigidTransform
    """Distal servo housing frame, fixed to the proximal segment: origin ``A``,
    ``+X`` along ``OA``."""
    distal: RigidTransform
    """Distal segment frame: origin ``A``, ``+X`` along ``A-EE``."""
    o: PointXZ
    a: PointXZ
    b: PointXZ
    ee: PointXZ

    @property
    def q_distal_absolute_rad(self) -> float:
        """The distal segment's flexion from extension, ``q_mcp + q_distal``."""

        return self.q_mcp_rad + self.q_distal_rad


@dataclass(frozen=True)
class OpenChainFinger:
    """Pose solver for one open-chain finger."""

    parameters: OpenChainParameters

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, OpenChainParameters):
            raise TypeError("parameters must be a OpenChainParameters instance")

    def solve(self, q_mcp_rad: float, q_distal_rad: float) -> OpenChainPose:
        """Return the pose for the MCP and distal FE flexions."""

        q_mcp = require_finite("q_mcp_rad", q_mcp_rad)
        q_distal = require_finite("q_distal_rad", q_distal_rad)
        p = self.parameters
        proximal = RigidTransform.rotation_about_axis(FLEXION_AXIS, q_mcp)
        distal_joint = proximal @ RigidTransform.from_translation((p.proximal_length_mm, 0.0, 0.0))
        distal = distal_joint @ RigidTransform.rotation_about_axis(FLEXION_AXIS, q_distal)

        def point(frame: RigidTransform, x_mm: float) -> PointXZ:
            x, _, z = frame.transform_point((x_mm, 0.0, 0.0))
            return PointXZ(x, z)

        return OpenChainPose(
            q_mcp_rad=q_mcp,
            q_distal_rad=q_distal,
            proximal=proximal,
            distal_joint=distal_joint,
            distal=distal,
            o=point(proximal, 0.0),
            a=point(distal, 0.0),
            b=point(distal, p.b_offset_mm),
            ee=point(distal, p.distal_length_mm),
        )


__all__ = ["FLEXION_AXIS", "OpenChainFinger", "OpenChainPose"]
