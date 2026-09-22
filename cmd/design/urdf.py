"""Compile CLEAR Hand design parameters into a standard opened-tree URDF."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree

from ..hardware import Actuator, XM335Actuator
from .hand import HandDesignParameters, PlanarPlacement


MM_TO_M = 1.0e-3
PALM_LINK_NAME = "palm_link"
MCP_AA_AXIS_XYZ = (0.0, 0.0, 1.0)
MCP_AA_HOUSING_RPY_RAD = (0.0, 0.0, 0.0)

# Existing finger/linkage morphology uses +X as its flexion axis and +Y as its
# distal direction. The hand-level frame uses +X distal and +Y radial, so the
# two conventions need an explicit fixed transform. For the current MCP-AA
# package, the actuator housing frame is aligned with the hand frame. The
# fixed Rz(-pi/2) transform below maps the child linkage-local frame into the
# parent hand/MCP-FE shaft frame:
#
#     linkage +X -> hand -Y
#     linkage +Y -> hand +X
#     linkage +Z -> hand +Z
LINKAGE_FRAME_RPY_IN_HAND_RAD = (0.0, 0.0, -math.pi / 2.0)


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


def _add_box_geometry(link: ElementTree.Element, actuator: Actuator) -> None:
    """Add XM335-sized visual and collision boxes in metres.

    The housing link frame is at the actuator body center. The actuator-local
    dimensions are X=width, Y=height, and Z=depth.
    """

    box = actuator.box
    size_m = (box.width_mm * MM_TO_M, box.height_mm * MM_TO_M, box.depth_mm * MM_TO_M)
    for element_name in ("visual", "collision"):
        element = ElementTree.SubElement(link, element_name)
        ElementTree.SubElement(
            element,
            "origin",
            {"xyz": "0 0 0", "rpy": "0 0 0"},
        )
        geometry = ElementTree.SubElement(element, "geometry")
        ElementTree.SubElement(geometry, "box", {"size": _format_vector(size_m)})


def _shaft_offset_from_body_center_mm(actuator: Actuator) -> float:
    """Return the shaft center offset along actuator-local +Y."""

    return actuator.shaft.center_from_bottom_mm - actuator.box.height_mm / 2.0


def _validate_actuator_frame(actuator: Actuator) -> None:
    if not isinstance(actuator, Actuator):
        raise TypeError("actuator must be an Actuator instance")
    if not all(
        math.isclose(component, expected, rel_tol=0.0, abs_tol=1.0e-12)
        for component, expected in zip(actuator.shaft.axis_xyz, (0.0, 0.0, 1.0))
    ):
        raise ValueError("hand URDF actuator compilation requires a +Z shaft axis")


def _add_mcp_aa_branch(
    robot: ElementTree.Element,
    *,
    finger_name: str,
    placement: PlanarPlacement,
    actuator: Actuator,
) -> None:
    """Add fixed XM335 housing, MCP-AA output, and linkage frame boundary."""

    housing_link = f"{finger_name}_mcp_aa_housing"
    output_link = f"{finger_name}_mcp_aa_output"
    linkage_link = f"{finger_name}_linkage_frame"
    shaft_offset_mm = _shaft_offset_from_body_center_mm(actuator)

    housing = _add_link(robot, housing_link)
    _add_box_geometry(housing, actuator)

    # The design placement is the fixed XM335 housing-frame position in the
    # hand frame. The MCP-AA shaft/output position is derived by the following
    # revolute origin inside that housing.
    _add_joint(
        robot,
        joint_name=f"{finger_name}_mcp_aa_housing_mount_joint",
        joint_type="fixed",
        parent_link=PALM_LINK_NAME,
        child_link=housing_link,
        xyz_mm=(placement.x_mm, placement.y_mm, 0.0),
        rpy_rad=MCP_AA_HOUSING_RPY_RAD,
    )

    _add_link(robot, output_link)
    _add_joint(
        robot,
        joint_name=f"{finger_name}_mcp_aa_joint",
        joint_type="continuous",
        parent_link=housing_link,
        child_link=output_link,
        xyz_mm=(0.0, shaft_offset_mm, 0.0),
        axis_xyz=MCP_AA_AXIS_XYZ,
    )

    # The output link is the hand/MCP-FE shaft frame for this opened tree. The
    # fixed rotation makes the existing linkage-local frame explicit instead
    # of silently assuming that the hand and linkage conventions are identical.
    _add_link(robot, linkage_link)
    _add_joint(
        robot,
        joint_name=f"{finger_name}_mcp_fe_to_linkage_frame_joint",
        joint_type="fixed",
        parent_link=output_link,
        child_link=linkage_link,
        xyz_mm=(0.0, 0.0, 0.0),
        rpy_rad=LINKAGE_FRAME_RPY_IN_HAND_RAD,
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
    _validate_actuator_frame(actuator)

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
    "LINKAGE_FRAME_RPY_IN_HAND_RAD",
    "MCP_AA_HOUSING_RPY_RAD",
    "MCP_AA_AXIS_XYZ",
    "MM_TO_M",
    "PALM_LINK_NAME",
    "compile_hand_urdf",
    "write_hand_urdf",
]
