from dataclasses import FrozenInstanceError
from xml.etree import ElementTree

import pytest

from cmd.design import HandDesignParameters, PlanarPlacement, compile_hand_urdf


def make_hand_design() -> HandDesignParameters:
    return HandDesignParameters(
        index_mcp_aa=PlanarPlacement(30.0, 10.0),
        middle_mcp_aa=PlanarPlacement(32.0, 3.0),
        ring_mcp_aa=PlanarPlacement(31.0, -4.0),
        little_mcp_aa=PlanarPlacement(28.0, -11.0),
        thumb_cmc=PlanarPlacement(12.0, 18.0),
    )


def test_planar_hand_parameters_are_immutable_and_finite() -> None:
    parameters = make_hand_design()

    with pytest.raises(FrozenInstanceError):
        parameters.thumb_cmc = PlanarPlacement(1.0, 1.0)

    with pytest.raises(FrozenInstanceError):
        parameters.index_mcp_aa.x_mm = 1.0

    with pytest.raises(ValueError):
        PlanarPlacement(float("nan"), 0.0)


def _joint_map(root: ElementTree.Element) -> dict[str, ElementTree.Element]:
    return {joint.attrib["name"]: joint for joint in root.findall("joint")}


def test_compile_emits_hardware_aware_opened_tree_in_urdf_metres() -> None:
    root = ElementTree.fromstring(compile_hand_urdf(make_hand_design()))

    assert root.tag == "robot"
    assert root.attrib == {"name": "clear_hand"}

    links = {link.attrib["name"] for link in root.findall("link")}
    expected_links = {"palm_link", "thumb_cmc_link"}
    for finger_name in ("index", "middle", "ring", "little"):
        expected_links.update(
            {
                f"{finger_name}_mcp_aa_housing",
                f"{finger_name}_mcp_aa_output",
                f"{finger_name}_linkage_frame",
            }
        )
    assert links == expected_links

    joints = _joint_map(root)
    expected_joints = {"thumb_cmc_mount_joint"}
    for finger_name in ("index", "middle", "ring", "little"):
        expected_joints.update(
            {
                f"{finger_name}_mcp_aa_housing_mount_joint",
                f"{finger_name}_mcp_aa_joint",
                f"{finger_name}_mcp_fe_to_linkage_frame_joint",
            }
        )
    assert set(joints) == expected_joints
    assert not any("mcp_fe_joint" in name for name in joints)

    housing_mount = joints["index_mcp_aa_housing_mount_joint"]
    assert housing_mount.attrib["type"] == "fixed"
    assert housing_mount.find("parent").attrib["link"] == "palm_link"
    assert housing_mount.find("child").attrib["link"] == "index_mcp_aa_housing"
    assert [float(value) for value in housing_mount.find("origin").attrib["xyz"].split()] == pytest.approx(
        [0.03, 0.01, 0.0]
    )

    mcp_aa = joints["index_mcp_aa_joint"]
    assert mcp_aa.attrib["type"] == "continuous"
    assert mcp_aa.find("parent").attrib["link"] == "index_mcp_aa_housing"
    assert mcp_aa.find("child").attrib["link"] == "index_mcp_aa_output"
    assert [float(value) for value in mcp_aa.find("origin").attrib["xyz"].split()] == pytest.approx(
        [0.0, 0.008, 0.0]
    )
    assert mcp_aa.find("axis").attrib == {"xyz": "0 0 1"}
    housing_xyz = [
        float(value) for value in housing_mount.find("origin").attrib["xyz"].split()
    ]
    shaft_offset_xyz = [
        float(value) for value in mcp_aa.find("origin").attrib["xyz"].split()
    ]
    assert [housing_xyz[index] + shaft_offset_xyz[index] for index in range(3)] == pytest.approx(
        [0.03, 0.018, 0.0]
    )

    linkage_frame = joints["index_mcp_fe_to_linkage_frame_joint"]
    assert linkage_frame.attrib["type"] == "fixed"
    assert linkage_frame.find("parent").attrib["link"] == "index_mcp_aa_output"
    assert linkage_frame.find("child").attrib["link"] == "index_linkage_frame"
    assert linkage_frame.find("origin").attrib["rpy"] == "0 0 -1.57079632679"

    housing_link = next(
        link for link in root.findall("link") if link.attrib["name"] == "index_mcp_aa_housing"
    )
    for geometry_owner in ("visual", "collision"):
        box = housing_link.find(f"{geometry_owner}/geometry/box")
        assert [float(value) for value in box.attrib["size"].split()] == pytest.approx(
            [0.019, 0.035, 0.022]
        )

    thumb_mount = joints["thumb_cmc_mount_joint"]
    assert thumb_mount.attrib["type"] == "fixed"
    assert thumb_mount.find("origin").attrib["xyz"] == "0.012 0.018 0"


def test_compile_rejects_wrong_input_invalid_robot_names_and_wrong_actuator_frame() -> None:
    with pytest.raises(TypeError):
        compile_hand_urdf(object())

    with pytest.raises(ValueError):
        compile_hand_urdf(make_hand_design(), robot_name="clear hand")

    from cmd.hardware import Actuator, MechanicalParameters, RectangularBox, ShaftLocation

    wrong_frame_actuator = Actuator(
        box=RectangularBox(19.0, 35.0, 22.0),
        shaft=ShaftLocation(25.5, axis_xyz=(1.0, 0.0, 0.0)),
        mechanics=MechanicalParameters(323.04, 1.0, 27.0),
    )
    with pytest.raises(ValueError, match=r"\+Z shaft axis"):
        compile_hand_urdf(make_hand_design(), actuator=wrong_frame_actuator)
