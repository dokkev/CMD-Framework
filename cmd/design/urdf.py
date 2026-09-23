"""Compile CLEAR Hand design parameters into a standard opened-tree URDF."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree

from ..hardware import ACTUATOR_SHAFT_AXIS_XYZ, Actuator, XM335Actuator
from .hand import HandDesignParameters, PlanarPlacement


MM_TO_M = 1.0e-3
PALM_LINK_NAME = "palm_link"
MCP_AA_AXIS_XYZ = ACTUATOR_SHAFT_AXIS_XYZ
MCP_AA_ACTUATOR_RPY_IN_HAND_RAD = (math.pi / 2.0, 0.0, 0.0)
ACTUATOR_CYLINDER_RPY_RAD = (-math.pi / 2.0, 0.0, 0.0)

# Existing finger/linkage morphology uses +X as its flexion axis and +Y as its
# distal direction. The hand-level frame uses +X distal and +Y radial, so the
# two conventions need an explicit fixed transform. The actuator frame is
# mounted by Rx(+pi/2), mapping actuator +Y onto hand +Z while keeping +X distal.
# The combined actuator-to-linkage rotation below preserves the prior physical
# linkage orientation in the hand frame:
#
#     linkage +X -> hand -Y
#     linkage +Y -> hand +X
#     linkage +Z -> hand +Z
LINKAGE_FRAME_RPY_IN_ACTUATOR_RAD = (0.0, -math.pi / 2.0, -math.pi / 2.0)


def _format_number(value: float) -> str:
    if value == 0.0:
        return "0"
    return f"{value:.12g}"


def _format_vector(values: Iterable[float]) -> str:
    return " ".join(_format_number(float(value)) for value in values)


def _format_xyz_m(values_mm: Iterable[float]) -> str:
    """Serialize millimetre values as URDF metres at the XML boundary."""

    return _format_vector(float(value_mm) * MM_TO_M for value_mm in values_mm)


def _format_rpy(rpy_rad: Iterable[float]) -> str:
    return _format_vector(rpy_rad)


def _add_link(robot: ElementTree.Element, name: str) -> ElementTree.Element:
    return ElementTree.SubElement(robot, "link", {"name": name})


def _add_joint(
    robot: ElementTree.Element,
    *,
    joint_name: str,
    joint_type: str,
    parent_link: str,
    child_link: str,
    xyz_mm: tuple[float, float, float],
    rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0),
    axis_xyz: tuple[float, float, float] | None = None,
) -> ElementTree.Element:
    joint = ElementTree.SubElement(
        robot,
        "joint",
        {"name": joint_name, "type": joint_type},
    )
    ElementTree.SubElement(joint, "parent", {"link": parent_link})
    ElementTree.SubElement(joint, "child", {"link": child_link})
    ElementTree.SubElement(
        joint,
        "origin",
        {
            "xyz": _format_xyz_m(xyz_mm),
            "rpy": _format_rpy(rpy_rad),
        },
    )
    if axis_xyz is not None:
        ElementTree.SubElement(joint, "axis", {"xyz": _format_vector(axis_xyz)})
    return joint


def _add_actuator_geometry(link: ElementTree.Element, actuator: Actuator) -> None:
    """Add the exact box-plus-cylinder D-shaped body to visual and collision."""

    geometry = actuator.geometry
    box_size_mm = (
        geometry.rear_extent_mm,
        geometry.depth_mm,
        geometry.width_mm,
    )
    box_origin_mm = (-geometry.rear_extent_mm / 2.0, 0.0, 0.0)
    for element_name in ("visual", "collision"):
        box_element = ElementTree.SubElement(link, element_name)
        ElementTree.SubElement(
            box_element,
            "origin",
            {
                "xyz": _format_xyz_m(box_origin_mm),
                "rpy": "0 0 0",
            },
        )
        box_geometry = ElementTree.SubElement(box_element, "geometry")
        ElementTree.SubElement(
            box_geometry,
            "box",
            {"size": _format_xyz_m(box_size_mm)},
        )

        cylinder_element = ElementTree.SubElement(link, element_name)
        ElementTree.SubElement(
            cylinder_element,
            "origin",
            {
                "xyz": "0 0 0",
                "rpy": _format_rpy(ACTUATOR_CYLINDER_RPY_RAD),
            },
        )
        cylinder_geometry = ElementTree.SubElement(cylinder_element, "geometry")
        ElementTree.SubElement(
            cylinder_geometry,
            "cylinder",
            {
                "radius": _format_number(geometry.radius_mm * MM_TO_M),
                "length": _format_number(geometry.depth_mm * MM_TO_M),
            },
        )


def _add_mcp_aa_branch(
    robot: ElementTree.Element,
    *,
    finger_name: str,
    placement: PlanarPlacement,
    actuator: Actuator,
) -> None:
    """Add shaft-centred XM335 actuator, MCP-AA output, and linkage frame."""

    housing_link = f"{finger_name}_mcp_aa_housing"
    output_link = f"{finger_name}_mcp_aa_output"
    linkage_link = f"{finger_name}_linkage_frame"
    housing = _add_link(robot, housing_link)
    _add_actuator_geometry(housing, actuator)

    # PlanarPlacement is the MCP-AA shaft/joint-frame position. The actuator
    # link frame has the same origin and is rotated to map its +Y shaft axis
    # onto the established +Z hand-frame MCP-AA axis.
    _add_joint(
        robot,
        joint_name=f"{finger_name}_mcp_aa_housing_mount_joint",
        joint_type="fixed",
        parent_link=PALM_LINK_NAME,
        child_link=housing_link,
        xyz_mm=(placement.x_mm, placement.y_mm, 0.0),
        rpy_rad=MCP_AA_ACTUATOR_RPY_IN_HAND_RAD,
    )

    _add_link(robot, output_link)
    _add_joint(
        robot,
        joint_name=f"{finger_name}_mcp_aa_joint",
        joint_type="continuous",
        parent_link=housing_link,
        child_link=output_link,
        xyz_mm=(0.0, 0.0, 0.0),
        axis_xyz=MCP_AA_AXIS_XYZ,
    )

    # This relative transform composes with the actuator mount to preserve the
    # prior linkage orientation expressed in the hand frame.
    _add_link(robot, linkage_link)
    _add_joint(
        robot,
        joint_name=f"{finger_name}_mcp_fe_to_linkage_frame_joint",
        joint_type="fixed",
        parent_link=output_link,
        child_link=linkage_link,
        xyz_mm=(0.0, 0.0, 0.0),
        rpy_rad=LINKAGE_FRAME_RPY_IN_ACTUATOR_RAD,
    )


def _validate_robot_name(robot_name: str) -> None:
    if not isinstance(robot_name, str) or not robot_name or any(
        character.isspace() for character in robot_name
    ):
        raise ValueError("robot_name must be a non-empty string without whitespace")


def compile_hand_urdf(
    parameters: HandDesignParameters,
    *,
    robot_name: str = "clear_hand",
    actuator: Actuator | None = None,
) -> str:
    """Compile hand placement and XM335 packaging into an opened-tree URDF.

    Python design positions are millimetres in the right-hand palm frame. The
    generated URDF uses metres, as required by URDF consumers. Each non-thumb
    branch is ``palm -> fixed housing -> continuous MCP-AA output -> fixed
    linkage-frame transform``. Thumb CMC remains a placement-only mount because
    its future rotation axis and limits are not defined in this layer.

    No custom robot-tree classes, MCP-FE joint, closed-loop constraint, or
    kinematic solve is created here.
    """

    if not isinstance(parameters, HandDesignParameters):
        raise TypeError("parameters must be a HandDesignParameters instance")
    _validate_robot_name(robot_name)
    actuator = XM335Actuator() if actuator is None else actuator
    if not isinstance(actuator, Actuator):
        raise TypeError("actuator must be an Actuator instance")

    robot = ElementTree.Element("robot", {"name": robot_name})
    _add_link(robot, PALM_LINK_NAME)

    non_thumb_placements = (
        ("index", parameters.index_mcp_aa),
        ("middle", parameters.middle_mcp_aa),
        ("ring", parameters.ring_mcp_aa),
        ("little", parameters.little_mcp_aa),
    )
    for finger_name, placement in non_thumb_placements:
        _add_mcp_aa_branch(
            robot,
            finger_name=finger_name,
            placement=placement,
            actuator=actuator,
        )

    _add_link(robot, "thumb_cmc_link")
    _add_joint(
        robot,
        joint_name="thumb_cmc_mount_joint",
        joint_type="fixed",
        parent_link=PALM_LINK_NAME,
        child_link="thumb_cmc_link",
        xyz_mm=(parameters.thumb_cmc.x_mm, parameters.thumb_cmc.y_mm, 0.0),
    )

    ElementTree.indent(robot, space="  ")
    return ElementTree.tostring(robot, encoding="unicode") + "\n"


def write_hand_urdf(
    parameters: HandDesignParameters,
    output_path: str | Path,
    *,
    robot_name: str = "clear_hand",
    actuator: Actuator | None = None,
) -> Path:
    """Compile and write a hand URDF, returning the output path."""

    output_path = Path(output_path)
    output_path.write_text(
        compile_hand_urdf(
            parameters,
            robot_name=robot_name,
            actuator=actuator,
        ),
        encoding="utf-8",
    )
    return output_path


__all__ = [
    "ACTUATOR_CYLINDER_RPY_RAD",
    "LINKAGE_FRAME_RPY_IN_ACTUATOR_RAD",
    "MCP_AA_ACTUATOR_RPY_IN_HAND_RAD",
    "MCP_AA_AXIS_XYZ",
    "MM_TO_M",
    "PALM_LINK_NAME",
    "compile_hand_urdf",
    "write_hand_urdf",
]
