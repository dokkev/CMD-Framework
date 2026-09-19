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


def test_compile_emits_only_the_opened_hand_placement_tree() -> None:
    root = ElementTree.fromstring(compile_hand_urdf(make_hand_design()))

    assert root.tag == "robot"
    assert root.attrib == {"name": "clear_hand"}

    links = {link.attrib["name"] for link in root.findall("link")}
    assert links == {
        "palm_link",
        "index_mcp_aa_link",
        "middle_mcp_aa_link",
        "ring_mcp_aa_link",
        "little_mcp_aa_link",
        "thumb_cmc_link",
    }

    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    assert set(joints) == {
        "index_mcp_aa_joint",
        "middle_mcp_aa_joint",
        "ring_mcp_aa_joint",
        "little_mcp_aa_joint",
        "thumb_cmc_joint",
    }
    assert not any("mcp_fe" in name for name in (*links, *joints))

    index_joint = joints["index_mcp_aa_joint"]
    assert index_joint.attrib["type"] == "continuous"
    assert index_joint.find("parent").attrib["link"] == "palm_link"
    assert index_joint.find("child").attrib["link"] == "index_mcp_aa_link"
    assert index_joint.find("origin").attrib == {
        "xyz": "30 10 0",
        "rpy": "0 0 0",
    }
    assert index_joint.find("axis").attrib == {"xyz": "0 0 1"}

    thumb_joint = joints["thumb_cmc_joint"]
    assert thumb_joint.attrib["type"] == "fixed"
    assert thumb_joint.find("origin").attrib == {
        "xyz": "12 18 0",
        "rpy": "0 0 0",
    }
    assert thumb_joint.find("axis") is None


def test_compile_rejects_wrong_input_and_invalid_robot_names() -> None:
    with pytest.raises(TypeError):
        compile_hand_urdf(object())

    with pytest.raises(ValueError):
        compile_hand_urdf(make_hand_design(), robot_name="clear hand")
