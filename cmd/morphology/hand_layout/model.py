"""Reference-frame graph for the CLEAR Hand morphology.

The hand/global frame ``H`` uses +X distal, +Y along ordinary finger FE axes,
and +Z dorsal. Transforms use ``T_parent_child`` notation and millimetres.
The index MCP-AA is the identity root. Runtime joint configurations are outside
this reference morphology model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ...geometry import RigidTransform, Vector3
from ..linkage import FingerLinkage, LinkagePose, ReferencePositions
from ..pad import Fingertip, FingertipParameters
from . import HandLayoutParameters, HandMorphologyParameters


HAND_X_LONGITUDINAL: Vector3 = (1.0, 0.0, 0.0)
HAND_Y_FE_AXIS: Vector3 = (0.0, 1.0, 0.0)
HAND_Z_DORSAL: Vector3 = (0.0, 0.0, 1.0)

ORDINARY_MCP_AA_TO_MCP_FE = RigidTransform.from_translation((23.5, 0.0, 16.75))
ORDINARY_MCP_FE_TO_LINKAGE = RigidTransform.identity()

# Mounts are expressed joint-to-actuator. The FE-style quarter-turn maps
# actuator +Z onto joint +Y while retaining actuator +X along joint +X.
MCP_AA_ACTUATOR_MOUNT = RigidTransform.identity()
FE_ACTUATOR_MOUNT = RigidTransform.rotation_about_axis(
    (1.0, 0.0, 0.0),
    -math.pi / 2.0,
)
THUMB_CMC_RO_ACTUATOR_MOUNT = FE_ACTUATOR_MOUNT
THUMB_CMC_RAA_ACTUATOR_MOUNT = MCP_AA_ACTUATOR_MOUNT
THUMB_MCP_FE_ACTUATOR_MOUNT = FE_ACTUATOR_MOUNT
# IP-FE reuses the provisional FE-axis mount; CAD housing clocking remains
# unconfirmed, like the other actuator mounts.
THUMB_IP_FE_ACTUATOR_MOUNT = FE_ACTUATOR_MOUNT

# Each fingertip's rotation axis lies on its distal joint: B for ordinary
# fingers, the IP-FE axis for the thumb. A lateral offset shifts the fingertip
# along that axis; both digits use the fingertip centreline for now.
ORDINARY_FINGERTIP_LATERAL_OFFSET_MM = 0.0
THUMB_FINGERTIP_LATERAL_OFFSET_MM = 0.0

THUMB_CMC_RAA_TO_MCP_FE_DISTANCE_MM = 39.5
THUMB_MCP_FE_TO_IP_FE_DISTANCE_MM = 35.5

_JOINT_AXES_LOCAL: dict[str, Vector3] = {
    "index_mcp_aa": (0.0, 0.0, 1.0),
    "index_mcp_fe": (0.0, 1.0, 0.0),
    "middle_mcp_aa": (0.0, 0.0, 1.0),
    "middle_mcp_fe": (0.0, 1.0, 0.0),
    "ring_mcp_aa": (0.0, 0.0, 1.0),
    "ring_mcp_fe": (0.0, 1.0, 0.0),
    "little_mcp_aa": (0.0, 0.0, 1.0),
    "little_mcp_fe": (0.0, 1.0, 0.0),
    "thumb_cmc_ro": (0.0, 1.0, 0.0),
    "thumb_cmc_raa": (0.0, 0.0, 1.0),
    "thumb_mcp_fe": (0.0, 1.0, 0.0),
    "thumb_ip_fe": (0.0, 1.0, 0.0),
}

_ACTUATOR_MOUNTS: dict[str, RigidTransform] = {
    "index_mcp_aa": MCP_AA_ACTUATOR_MOUNT,
    "index_mcp_fe": FE_ACTUATOR_MOUNT,
    "middle_mcp_aa": MCP_AA_ACTUATOR_MOUNT,
    "middle_mcp_fe": FE_ACTUATOR_MOUNT,
    "ring_mcp_aa": MCP_AA_ACTUATOR_MOUNT,
    "ring_mcp_fe": FE_ACTUATOR_MOUNT,
    "little_mcp_aa": MCP_AA_ACTUATOR_MOUNT,
    "little_mcp_fe": FE_ACTUATOR_MOUNT,
    "thumb_cmc_ro": THUMB_CMC_RO_ACTUATOR_MOUNT,
    "thumb_cmc_raa": THUMB_CMC_RAA_ACTUATOR_MOUNT,
    "thumb_mcp_fe": THUMB_MCP_FE_ACTUATOR_MOUNT,
    "thumb_ip_fe": THUMB_IP_FE_ACTUATOR_MOUNT,
}


def _translation(x_mm: float, y_mm: float, z_mm: float) -> RigidTransform:
    return RigidTransform.from_translation((x_mm, y_mm, z_mm))


def distal_link_transform(pose: LinkagePose | ReferencePositions) -> RigidTransform:
    """Return ``T_L_BC``: origin at the DIP axis ``B``, ``+X`` along ``B -> C``.

    Flexion turns ``B -> C`` toward ``-Z``, which is a positive rotation about
    the ``+Y`` flexion axis.
    """

    angle_rad = math.atan2(-(pose.c.z_mm - pose.b.z_mm), pose.c.x_mm - pose.b.x_mm)
    return _translation(pose.b.x_mm, 0.0, pose.b.z_mm) @ RigidTransform.rotation_about_axis(
        HAND_Y_FE_AXIS, angle_rad
    )


def fingertip_attachment(
    fingertip: Fingertip,
    rotation_axis_to_tip_mm: float,
    lateral_offset_mm: float = 0.0,
) -> RigidTransform:
    """Return ``T_RA_tip``, which places the fingertip's rotation axis on the joint.

    The joint frame has its origin on the rotation axis, ``+X`` distal along the
    link, ``+Y`` along the axis and ``+Z`` dorsal, so only a translation is needed.
    ``rotation_axis_to_tip_mm`` is the linkage ``bc_length_mm``.
    """

    x_mm, _, z_mm = fingertip.rotation_axis_mm(rotation_axis_to_tip_mm)
    return _translation(-x_mm, lateral_offset_mm, -z_mm)


@dataclass(frozen=True)
class HandLayout:
    """Derive the reference joint and XM335 poses from layout parameters.

    ``joint_transforms[name]`` is ``T_H_J``. ``actuator_transforms[name]`` is
    ``T_H_A = T_H_J @ T_J_A``. Joint-axis directions are separate from joint
    frame orientation, so ordinary MCP-AA and MCP-FE frames remain parallel to
    the hand frame even though their physical axes differ.
    """

    parameters: HandLayoutParameters

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, HandLayoutParameters):
            raise TypeError("parameters must be HandLayoutParameters")

    @property
    def joint_transforms(self) -> dict[str, RigidTransform]:
        p = self.parameters
        identity = RigidTransform.identity()

        thumb_cmc_ro = (
            _translation(p.thumb_cmc_ro.x_mm, p.thumb_cmc_ro.y_mm, 0.0)
            @ RigidTransform.rotation_about_axis(HAND_Z_DORSAL, p.theta_rad)
        )
        thumb_cmc_raa = thumb_cmc_ro @ _translation(p.ro_raa_distance_mm, 0.0, 0.0)
        thumb_mcp_fe = (
            thumb_cmc_raa
            @ _translation(THUMB_CMC_RAA_TO_MCP_FE_DISTANCE_MM, 0.0, 0.0)
            @ RigidTransform.rotation_about_axis(HAND_Z_DORSAL, p.phi_rad)
        )
        thumb_ip_fe = thumb_mcp_fe @ _translation(
            THUMB_MCP_FE_TO_IP_FE_DISTANCE_MM,
            0.0,
            0.0,
        )

        transforms = {
            "index_mcp_aa": identity,
            "index_mcp_fe": identity @ ORDINARY_MCP_AA_TO_MCP_FE,
        }
        for finger, point in (
            ("middle", p.middle_mcp_aa),
            ("ring", p.ring_mcp_aa),
            ("little", p.little_mcp_aa),
        ):
            mcp_aa = _translation(point.x_mm, point.y_mm, 0.0)
            transforms[f"{finger}_mcp_aa"] = mcp_aa
            transforms[f"{finger}_mcp_fe"] = mcp_aa @ ORDINARY_MCP_AA_TO_MCP_FE

        transforms.update(
            {
                "thumb_cmc_ro": thumb_cmc_ro,
                "thumb_cmc_raa": thumb_cmc_raa,
                "thumb_mcp_fe": thumb_mcp_fe,
                "thumb_ip_fe": thumb_ip_fe,
            }
        )
        return transforms

    @property
    def joint_axes_local(self) -> dict[str, Vector3]:
        """Physical rotation axes expressed in each named joint frame."""

        return dict(_JOINT_AXES_LOCAL)

    @property
    def joint_axes_hand(self) -> dict[str, Vector3]:
        """Physical joint axes transformed into the hand/global frame."""

        transforms = self.joint_transforms
        return {
            name: transforms[name].transform_vector(axis)
            for name, axis in _JOINT_AXES_LOCAL.items()
        }

    @property
    def actuator_mount_transforms(self) -> dict[str, RigidTransform]:
        """Fixed ``T_J_A`` mounts, keyed by the associated joint frame name."""

        return dict(_ACTUATOR_MOUNTS)

    @property
    def actuator_transforms(self) -> dict[str, RigidTransform]:
        """Derived ``T_H_A`` poses for all actuated reference joints."""

        joints = self.joint_transforms
        return {
            name: joints[name] @ mount
            for name, mount in _ACTUATOR_MOUNTS.items()
        }

    @property
    def linkage_transforms(self) -> dict[str, RigidTransform]:
        """Derived ``T_H_L`` poses for each ordinary finger's linkage frame."""

        joints = self.joint_transforms
        return {
            f"{finger}_linkage": (
                joints[f"{finger}_mcp_fe"] @ ORDINARY_MCP_FE_TO_LINKAGE
            )
            for finger in ("index", "middle", "ring", "little")
        }


@dataclass(frozen=True)
class HandMorphology:
    """Materialize a whole hand from shared and per-digit parameters."""

    parameters: HandMorphologyParameters

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, HandMorphologyParameters):
            raise TypeError("parameters must be HandMorphologyParameters")

    @property
    def layout(self) -> HandLayout:
        return HandLayout(self.parameters.layout)

    @property
    def linkage(self) -> FingerLinkage:
        return FingerLinkage(self.parameters.linkage)

    @property
    def fingertip(self) -> Fingertip:
        """The fingertip shared by all five digits, as long as the linkage BC."""

        return Fingertip(
            FingertipParameters(
                family=self.parameters.fingertip,
                total_length_mm=self.parameters.linkage.bc_length_mm,
            )
        )

    @property
    def fingertip_transforms(self) -> dict[str, RigidTransform]:
        """Reference-pose ``T_H_tip`` for every digit.

        Ordinary fingers compose ``T_H_L @ T_L_BC @ T_BC_tip`` from the fully
        extended linkage; the thumb reuses the fingertip and its ``BC`` length
        at its IP-FE joint frame.
        """

        layout = self.layout
        bc_mm = self.parameters.linkage.bc_length_mm
        distal = distal_link_transform(self.linkage.reference_positions)
        fingertip = self.fingertip
        transforms = {
            finger: (
                layout.linkage_transforms[f"{finger}_linkage"]
                @ distal
                @ fingertip_attachment(
                    fingertip, bc_mm, ORDINARY_FINGERTIP_LATERAL_OFFSET_MM
                )
            )
            for finger in ("index", "middle", "ring", "little")
        }
        transforms["thumb"] = layout.joint_transforms["thumb_ip_fe"] @ fingertip_attachment(
            fingertip, bc_mm, THUMB_FINGERTIP_LATERAL_OFFSET_MM
        )
        return transforms


__all__ = [
    "FE_ACTUATOR_MOUNT",
    "HAND_X_LONGITUDINAL",
    "HAND_Y_FE_AXIS",
    "HAND_Z_DORSAL",
    "HandLayout",
    "HandMorphology",
    "MCP_AA_ACTUATOR_MOUNT",
    "ORDINARY_FINGERTIP_LATERAL_OFFSET_MM",
    "ORDINARY_MCP_AA_TO_MCP_FE",
    "ORDINARY_MCP_FE_TO_LINKAGE",
    "THUMB_CMC_RAA_ACTUATOR_MOUNT",
    "THUMB_CMC_RO_ACTUATOR_MOUNT",
    "THUMB_CMC_RAA_TO_MCP_FE_DISTANCE_MM",
    "THUMB_FINGERTIP_LATERAL_OFFSET_MM",
    "THUMB_IP_FE_ACTUATOR_MOUNT",
    "THUMB_MCP_FE_ACTUATOR_MOUNT",
    "THUMB_MCP_FE_TO_IP_FE_DISTANCE_MM",
    "distal_link_transform",
    "fingertip_attachment",
]
