"""Reference-frame graph for the CLEAR Hand morphology.

The hand/global frame ``H`` uses +X distal, +Y along ordinary finger FE axes,
and +Z dorsal. Transforms use ``T_parent_child`` notation and millimetres.
The index MCP-AA is the identity root. Runtime joint configurations are outside
this reference morphology model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .hand_morphology_parameter import HandMorphologyParameters
from .rigid_transform import RigidTransform, Vector3


HAND_X_LONGITUDINAL: Vector3 = (1.0, 0.0, 0.0)
HAND_Y_FE_AXIS: Vector3 = (0.0, 1.0, 0.0)
HAND_Z_DORSAL: Vector3 = (0.0, 0.0, 1.0)

ORDINARY_MCP_AA_TO_MCP_FE = RigidTransform.from_translation((23.5, 0.0, 16.75))

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
}


def _translation(x_mm: float, y_mm: float, z_mm: float) -> RigidTransform:
    return RigidTransform.from_translation((x_mm, y_mm, z_mm))


@dataclass(frozen=True)
class HandMorphology:
    """Derive all reference joint and XM335 poses from independent parameters.

    ``joint_transforms[name]`` is ``T_H_J``. ``actuator_transforms[name]`` is
    ``T_H_A = T_H_J @ T_J_A``. Joint-axis directions are separate from joint
    frame orientation, so ordinary MCP-AA and MCP-FE frames remain parallel to
    the hand frame even though their physical axes differ.
    """

    parameters: HandMorphologyParameters

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, HandMorphologyParameters):
            raise TypeError("parameters must be HandMorphologyParameters")

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
            @ _translation(p.raa_mcp_fe_distance_mm, 0.0, 0.0)
            @ RigidTransform.rotation_about_axis(HAND_Z_DORSAL, p.phi_rad)
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


__all__ = [
    "FE_ACTUATOR_MOUNT",
    "HAND_X_LONGITUDINAL",
    "HAND_Y_FE_AXIS",
    "HAND_Z_DORSAL",
    "HandMorphology",
    "MCP_AA_ACTUATOR_MOUNT",
    "ORDINARY_MCP_AA_TO_MCP_FE",
    "THUMB_CMC_RAA_ACTUATOR_MOUNT",
    "THUMB_CMC_RO_ACTUATOR_MOUNT",
    "THUMB_MCP_FE_ACTUATOR_MOUNT",
]
