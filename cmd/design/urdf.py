"""Compile CLEAR Hand layout parameters into a standard opened-tree URDF."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree

from .hand import HandDesignParameters, PlanarPlacement


PALM_LINK_NAME = "palm_link"
MCP_AA_AXIS_XYZ = (0.0, 0.0, 1.0)


def _format_number(value: float) -> str:
    if value == 0.0:
        return "0"
    return f"{value:.12g}"


def _format_xyz(xyz: Iterable[float]) -> str:
    return " ".join(_format_number(float(value)) for value in xyz)


def _add_link(robot: ElementTree.Element, name: str) -> None:
    ElementTree.SubElement(robot, "link", {"name": name})


def _add_placement_joint(
    robot: ElementTree.Element,
    *,
    joint_name: str,
    child_link: str,
    placement: PlanarPlacement,
    joint_type: str,
    axis_xyz: tuple[float, float, float] | None,
) -> None:
    joint = ElementTree.SubElement(
        robot,
        "joint",
        {"name": joint_name, "type": joint_type},
    )
    ElementTree.SubElement(joint, "parent", {"link": PALM_LINK_NAME})
    ElementTree.SubElement(joint, "child", {"link": child_link})
    ElementTree.SubElement(
        joint,
        "origin",
        {
            "xyz": _format_xyz((placement.x_mm, placement.y_mm, 0.0)),
            "rpy": "0 0 0",
        },
    )
    if axis_xyz is not None:
        ElementTree.SubElement(joint, "axis", {"xyz": _format_xyz(axis_xyz)})


def _validate_robot_name(robot_name: str) -> None:
    if not isinstance(robot_name, str) or not robot_name or any(
        character.isspace() for character in robot_name
    ):
        raise ValueError("robot_name must be a non-empty string without whitespace")


def compile_hand_urdf(
    parameters: HandDesignParameters,
    *,
    robot_name: str = "clear_hand",
) -> str:
    """Compile hand placement parameters into an opened-tree URDF string.

    The generated tree contains a palm root, one continuous MCP-AA joint and
    link for each non-thumb finger, and one fixed placement joint and link for
    the thumb CMC. MCP-FE, distal links, actuator geometry, and closed-loop
    constraints are intentionally outside this first compilation layer.
    """

    if not isinstance(parameters, HandDesignParameters):
        raise TypeError("parameters must be a HandDesignParameters instance")
    _validate_robot_name(robot_name)

    robot = ElementTree.Element("robot", {"name": robot_name})
    _add_link(robot, PALM_LINK_NAME)

    non_thumb_placements = (
        ("index", parameters.index_mcp_aa),
        ("middle", parameters.middle_mcp_aa),
        ("ring", parameters.ring_mcp_aa),
        ("little", parameters.little_mcp_aa),
    )
    for finger_name, placement in non_thumb_placements:
        child_link = f"{finger_name}_mcp_aa_link"
        _add_link(robot, child_link)
        _add_placement_joint(
            robot,
            joint_name=f"{finger_name}_mcp_aa_joint",
            child_link=child_link,
            placement=placement,
            joint_type="continuous",
            axis_xyz=MCP_AA_AXIS_XYZ,
        )

    _add_link(robot, "thumb_cmc_link")
    _add_placement_joint(
        robot,
        joint_name="thumb_cmc_joint",
        child_link="thumb_cmc_link",
        placement=parameters.thumb_cmc,
        joint_type="fixed",
        axis_xyz=None,
    )

    ElementTree.indent(robot, space="  ")
    return ElementTree.tostring(robot, encoding="unicode") + "\n"


def write_hand_urdf(
    parameters: HandDesignParameters,
    output_path: str | Path,
    *,
    robot_name: str = "clear_hand",
) -> Path:
    """Compile and write a hand URDF, returning the output path."""

    output_path = Path(output_path)
    output_path.write_text(
        compile_hand_urdf(parameters, robot_name=robot_name),
        encoding="utf-8",
    )
    return output_path


__all__ = [
    "MCP_AA_AXIS_XYZ",
    "PALM_LINK_NAME",
    "compile_hand_urdf",
    "write_hand_urdf",
]
