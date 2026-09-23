import math
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


def _matmul(first, second):
    return tuple(
        tuple(sum(first[row][k] * second[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def _rotation_from_rpy(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    return _matmul(_matmul(rz, ry), rx)


def _rotate(rotation, vector):
    return tuple(sum(rotation[row][column] * vector[column] for column in range(3)) for row in range(3))


def _parse_vector(text: str) -> tuple[float, ...]:
    return tuple(float(value) for value in text.split())


def test_compile_emits_shaft_centered_d_body_and_preserves_hand_axes() -> None:
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

    actuator_mount = joints["index_mcp_aa_housing_mount_joint"]
    assert actuator_mount.attrib["type"] == "fixed"
    assert actuator_mount.find("parent").attrib["link"] == "palm_link"
    assert actuator_mount.find("child").attrib["link"] == "index_mcp_aa_housing"
    assert _parse_vector(actuator_mount.find("origin").attrib["xyz"]) == pytest.approx(
        (0.03, 0.01, 0.0)
    )
    actuator_rpy = _parse_vector(actuator_mount.find("origin").attrib["rpy"])
    assert actuator_rpy == pytest.approx((math.pi / 2.0, 0.0, 0.0))

    mcp_aa = joints["index_mcp_aa_joint"]
    assert mcp_aa.attrib["type"] == "continuous"
    assert mcp_aa.find("parent").attrib["link"] == "index_mcp_aa_housing"
    assert mcp_aa.find("child").attrib["link"] == "index_mcp_aa_output"
    assert _parse_vector(mcp_aa.find("origin").attrib["xyz"]) == pytest.approx(
        (0.0, 0.0, 0.0)
    )
    assert _parse_vector(mcp_aa.find("axis").attrib["xyz"]) == pytest.approx(
        (0.0, 1.0, 0.0)
    )

    # The new actuator +Y axis remains the established physical hand +Z axis.
    actuator_to_hand = _rotation_from_rpy(actuator_rpy)
    assert _rotate(actuator_to_hand, (0.0, 1.0, 0.0)) == pytest.approx(
        (0.0, 0.0, 1.0), abs=1.0e-10
    )

    linkage_joint = joints["index_mcp_fe_to_linkage_frame_joint"]
    assert linkage_joint.attrib["type"] == "fixed"
    assert linkage_joint.find("parent").attrib["link"] == "index_mcp_aa_output"
    assert linkage_joint.find("child").attrib["link"] == "index_linkage_frame"
    linkage_rpy = _parse_vector(linkage_joint.find("origin").attrib["rpy"])
    actuator_to_linkage = _rotation_from_rpy(linkage_rpy)
    hand_to_linkage = _matmul(actuator_to_hand, actuator_to_linkage)
    assert _rotate(hand_to_linkage, (1.0, 0.0, 0.0)) == pytest.approx(
        (0.0, -1.0, 0.0), abs=1.0e-10
    )
    assert _rotate(hand_to_linkage, (0.0, 1.0, 0.0)) == pytest.approx(
        (1.0, 0.0, 0.0), abs=1.0e-10
    )
    assert _rotate(hand_to_linkage, (0.0, 0.0, 1.0)) == pytest.approx(
        (0.0, 0.0, 1.0), abs=1.0e-10
    )

    housing_link = next(
        link for link in root.findall("link") if link.attrib["name"] == "index_mcp_aa_housing"
    )
    for geometry_owner in ("visual", "collision"):
        components = housing_link.findall(geometry_owner)
        assert len(components) == 2
        box = next(component for component in components if component.find("geometry/box") is not None)
        cylinder = next(
            component for component in components
            if component.find("geometry/cylinder") is not None
        )

        assert _parse_vector(box.find("origin").attrib["xyz"]) == pytest.approx(
            (-0.01275, 0.0, 0.0)
        )
        assert _parse_vector(box.find("geometry/box").attrib["size"]) == pytest.approx(
            (0.0255, 0.022, 0.019)
        )
        assert _parse_vector(cylinder.find("origin").attrib["xyz"]) == pytest.approx(
            (0.0, 0.0, 0.0)
        )
        cylinder_rpy = _parse_vector(cylinder.find("origin").attrib["rpy"])
        assert cylinder_rpy == pytest.approx((-math.pi / 2.0, 0.0, 0.0))
        cylinder_axis = _rotate(
            _rotation_from_rpy(cylinder_rpy),
            (0.0, 0.0, 1.0),
        )
        assert cylinder_axis == pytest.approx((0.0, 1.0, 0.0), abs=1.0e-10)
        assert float(cylinder.find("geometry/cylinder").attrib["radius"]) == pytest.approx(
            0.0095
        )
        assert float(cylinder.find("geometry/cylinder").attrib["length"]) == pytest.approx(
            0.022
        )

    thumb_mount = joints["thumb_cmc_mount_joint"]
    assert thumb_mount.attrib["type"] == "fixed"
    assert thumb_mount.find("origin").attrib["xyz"] == "0.012 0.018 0"


def test_compile_rejects_wrong_input_invalid_names_and_non_actuator() -> None:
    with pytest.raises(TypeError):
        compile_hand_urdf(object())

    with pytest.raises(ValueError):
        compile_hand_urdf(make_hand_design(), robot_name="clear hand")

    with pytest.raises(TypeError, match="Actuator"):
        compile_hand_urdf(make_hand_design(), actuator=object())
