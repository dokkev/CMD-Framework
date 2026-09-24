"""Serialize reference CLEAR Hand morphology transforms as an opened URDF tree."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree

from ..geometry import RigidTransform
from ..hardware import Actuator, XM335Actuator
from ..morphology.hand import HAND_Z_DORSAL, HandLayout


MM_TO_M = 1.0e-3
PALM_LINK_NAME = "palm_link"
LINKAGE_FRAME_IN_MCP_FE = RigidTransform.rotation_about_axis(
    HAND_Z_DORSAL,
    -math.pi / 2.0,
)
# This preserves the pre-existing linkage orientation in the hand frame:
# linkage +X -> hand -Y, +Y -> hand +X, and +Z -> hand +Z.


def _format_number(value: float) -> str:
    if value == 0.0:
        return "0"
    return f"{value:.12g}"


def _format_vector(values: Iterable[float]) -> str:
    return " ".join(_format_number(float(value)) for value in values)


def _format_xyz_m(values_mm: Iterable[float]) -> str:
    """Convert morphology millimetres at the URDF serialization boundary."""

    return _format_vector(float(value_mm) * MM_TO_M for value_mm in values_mm)


def _rpy_from_rotation(rotation: tuple[tuple[float, ...], ...]) -> tuple[float, float, float]:
    """Convert a rotation matrix to URDF fixed-axis roll, pitch, yaw."""

    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if abs(math.cos(pitch)) > 1.0e-9:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def _add_link(robot: ElementTree.Element, name: str) -> ElementTree.Element:
    return ElementTree.SubElement(robot, "link", {"name": name})


def _add_joint(
    robot: ElementTree.Element,
    *,
    joint_name: str,
    joint_type: str,
    parent_link: str,
    child_link: str,
    origin: RigidTransform,
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
            "xyz": _format_xyz_m(origin.translation_mm),
            "rpy": _format_vector(_rpy_from_rotation(origin.rotation_matrix)),
        },
    )
    if axis_xyz is not None:
        ElementTree.SubElement(joint, "axis", {"xyz": _format_vector(axis_xyz)})
    return joint


def _add_actuator_geometry(link: ElementTree.Element, actuator: Actuator) -> None:
    """Add the exact local box-plus-cylinder XM335 D body to visual/collision."""

    geometry = actuator.geometry
    box_size_mm = (geometry.width_mm, geometry.rear_extent_mm, geometry.depth_mm)
    box_origin = RigidTransform.from_translation(
        (0.0, -geometry.rear_extent_mm / 2.0, 0.0)
    )
    for element_name in ("visual", "collision"):
        box_element = ElementTree.SubElement(link, element_name)
        ElementTree.SubElement(
            box_element,
            "origin",
            {
                "xyz": _format_xyz_m(box_origin.translation_mm),
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
            {"xyz": "0 0 0", "rpy": "0 0 0"},
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


def _add_actuated_joint(
    robot: ElementTree.Element,
    layout: HandLayout,
    actuator: Actuator,
    *,
    joint_frame: str,
    parent_joint_frame: str | None,
    parent_link: str,
) -> str:
    """Serialize a fixed actuator mount followed by its revolute joint."""

    actuator_link = f"{joint_frame}_actuator"
    output_link = f"{joint_frame}_output"
    _add_actuator_geometry(_add_link(robot, actuator_link), actuator)

    joint_poses = layout.joint_transforms
    actuator_poses = layout.actuator_transforms
    parent_pose = (
        RigidTransform.identity()
        if parent_joint_frame is None
        else joint_poses[parent_joint_frame]
    )
    parent_to_actuator = parent_pose.inverse() @ actuator_poses[joint_frame]
    _add_joint(
        robot,
        joint_name=f"{joint_frame}_actuator_mount",
        joint_type="fixed",
        parent_link=parent_link,
        child_link=actuator_link,
        origin=parent_to_actuator,
    )

    actuator_to_joint = actuator_poses[joint_frame].inverse() @ joint_poses[joint_frame]
    _add_link(robot, output_link)
    _add_joint(
        robot,
        joint_name=f"{joint_frame}_joint",
        joint_type="continuous",
        parent_link=actuator_link,
        child_link=output_link,
        origin=actuator_to_joint,
        axis_xyz=layout.joint_axes_local[joint_frame],
    )
    return output_link


def _add_ordinary_finger(
    robot: ElementTree.Element,
    layout: HandLayout,
    actuator: Actuator,
    finger_name: str,
) -> None:
    aa_frame = f"{finger_name}_mcp_aa"
    fe_frame = f"{finger_name}_mcp_fe"
    aa_output = _add_actuated_joint(
        robot,
        layout,
        actuator,
        joint_frame=aa_frame,
        parent_joint_frame=None,
        parent_link=PALM_LINK_NAME,
    )
    fe_output = _add_actuated_joint(
        robot,
        layout,
        actuator,
        joint_frame=fe_frame,
        parent_joint_frame=aa_frame,
        parent_link=aa_output,
    )
    linkage_frame = f"{finger_name}_linkage_frame"
    _add_link(robot, linkage_frame)
    _add_joint(
        robot,
        joint_name=f"{fe_frame}_to_linkage_frame",
        joint_type="fixed",
        parent_link=fe_output,
        child_link=linkage_frame,
        origin=LINKAGE_FRAME_IN_MCP_FE,
    )


def _add_thumb_chain(
    robot: ElementTree.Element,
    layout: HandLayout,
    actuator: Actuator,
) -> None:
    parent_link = PALM_LINK_NAME
    parent_joint_frame: str | None = None
    for joint_frame in (
        "thumb_cmc_ro",
        "thumb_cmc_raa",
        "thumb_mcp_fe",
        "thumb_ip_fe",
    ):
        parent_link = _add_actuated_joint(
            robot,
            layout,
            actuator,
            joint_frame=joint_frame,
            parent_joint_frame=parent_joint_frame,
            parent_link=parent_link,
        )
        parent_joint_frame = joint_frame


def _validate_robot_name(robot_name: str) -> None:
    if not isinstance(robot_name, str) or not robot_name or any(
        character.isspace() for character in robot_name
    ):
        raise ValueError("robot_name must be a non-empty string without whitespace")


def compile_hand_urdf(
    layout: HandLayout,
    *,
    robot_name: str = "clear_hand",
    actuator: Actuator | None = None,
) -> str:
    """Serialize hand morphology frames and XM335 packaging as an opened tree.

    Each actuator mount and revolute-joint origin is composed from the
    layout's ``T_H_J`` and ``T_H_A`` poses. Lengths remain millimetres
    until the URDF XML boundary. Closed-loop linkage constraints and runtime
    joint states are not represented here.
    """

    if not isinstance(layout, HandLayout):
        raise TypeError("layout must be a HandLayout instance")
    _validate_robot_name(robot_name)
    actuator = XM335Actuator() if actuator is None else actuator
    if not isinstance(actuator, Actuator):
        raise TypeError("actuator must be an Actuator instance")

    robot = ElementTree.Element("robot", {"name": robot_name})
    _add_link(robot, PALM_LINK_NAME)
    for finger_name in ("index", "middle", "ring", "little"):
        _add_ordinary_finger(robot, layout, actuator, finger_name)
    _add_thumb_chain(robot, layout, actuator)

    ElementTree.indent(robot, space="  ")
    return ElementTree.tostring(robot, encoding="unicode") + "\n"


def write_hand_urdf(
    layout: HandLayout,
    output_path: str | Path,
    *,
    robot_name: str = "clear_hand",
    actuator: Actuator | None = None,
) -> Path:
    """Compile and write a hand URDF, returning its output path."""

    output_path = Path(output_path)
    output_path.write_text(
        compile_hand_urdf(
            layout,
            robot_name=robot_name,
            actuator=actuator,
        ),
        encoding="utf-8",
    )
    return output_path


__all__ = [
    "LINKAGE_FRAME_IN_MCP_FE",
    "MM_TO_M",
    "PALM_LINK_NAME",
    "compile_hand_urdf",
    "write_hand_urdf",
]
