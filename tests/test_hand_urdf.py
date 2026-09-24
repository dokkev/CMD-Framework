import math
from xml.etree import ElementTree

import pytest

from cmd.geometry import RigidTransform
from cmd.design import compile_hand_urdf
from cmd.morphology import HandMorphology, HandMorphologyParameters, HandPlanePoint


def make_morphology() -> HandMorphology:
    return HandMorphology(
        HandMorphologyParameters(
            middle_mcp_aa=HandPlanePoint(40.0, 5.0),
            ring_mcp_aa=HandPlanePoint(35.0, -5.0),
            little_mcp_aa=HandPlanePoint(25.0, -15.0),
            thumb_cmc_ro=HandPlanePoint(5.0, 30.0),
            theta_rad=0.25,
            ro_raa_distance_mm=35.0,
            raa_mcp_fe_distance_mm=28.0,
            phi_rad=-0.2,
        )
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


def _matmul(first, second):
    return tuple(
        tuple(sum(first[row][k] * second[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def _rotate(rotation, vector):
    return tuple(
        sum(rotation[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _parse_vector(value: str) -> tuple[float, ...]:
    return tuple(float(component) for component in value.split())


def _parse_origin(origin: ElementTree.Element) -> RigidTransform:
    xyz_m = _parse_vector(origin.attrib["xyz"])
    rpy = _parse_vector(origin.attrib["rpy"])
    return RigidTransform(
        rotation_matrix=_rotation_from_rpy(rpy),
        translation_mm=tuple(value * 1000.0 for value in xyz_m),
    )


def _assert_transform_equal(actual: RigidTransform, expected: RigidTransform) -> None:
    assert actual.translation_mm == pytest.approx(expected.translation_mm, abs=1.0e-8)
    actual_rotation = [value for row in actual.rotation_matrix for value in row]
    expected_rotation = [value for row in expected.rotation_matrix for value in row]
    assert actual_rotation == pytest.approx(expected_rotation, abs=1.0e-10)


def test_urdf_serializes_all_morphology_frames_and_actuator_mounts() -> None:
    morphology = make_morphology()
    root = ElementTree.fromstring(compile_hand_urdf(morphology))
    assert root.tag == "robot"
    assert root.attrib == {"name": "clear_hand"}

    joint_map = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    link_names = {link.attrib["name"] for link in root.findall("link")}
    assert "palm_link" in link_names

    frame_parents = []
    for finger in ("index", "middle", "ring", "little"):
        frame_parents.extend(
            (
                (f"{finger}_mcp_aa", None, "palm_link"),
                (
                    f"{finger}_mcp_fe",
                    f"{finger}_mcp_aa",
                    f"{finger}_mcp_aa_output",
                ),
            )
        )
    frame_parents.extend(
        (
            ("thumb_cmc_ro", None, "palm_link"),
            ("thumb_cmc_raa", "thumb_cmc_ro", "thumb_cmc_ro_output"),
            ("thumb_mcp_fe", "thumb_cmc_raa", "thumb_cmc_raa_output"),
        )
    )

    expected_links = {"palm_link"}
    expected_joint_names = set()
    for frame_name, parent_frame, parent_link in frame_parents:
        actuator_link = f"{frame_name}_actuator"
        output_link = f"{frame_name}_output"
        mount_name = f"{frame_name}_actuator_mount"
        joint_name = f"{frame_name}_joint"
        expected_links.update((actuator_link, output_link))
        expected_joint_names.update((mount_name, joint_name))

        mount_joint = joint_map[mount_name]
        assert mount_joint.attrib["type"] == "fixed"
        assert mount_joint.find("parent").attrib["link"] == parent_link
        assert mount_joint.find("child").attrib["link"] == actuator_link
        mount_transform = _parse_origin(mount_joint.find("origin"))
        parent_pose = (
            RigidTransform.identity()
            if parent_frame is None
            else morphology.joint_transforms[parent_frame]
        )
        _assert_transform_equal(
            parent_pose @ mount_transform,
            morphology.actuator_transforms[frame_name],
        )

        revolute = joint_map[joint_name]
        assert revolute.attrib["type"] == "continuous"
        assert revolute.find("parent").attrib["link"] == actuator_link
        assert revolute.find("child").attrib["link"] == output_link
        actuator_to_joint = _parse_origin(revolute.find("origin"))
        _assert_transform_equal(
            morphology.actuator_transforms[frame_name] @ actuator_to_joint,
            morphology.joint_transforms[frame_name],
        )
        assert actuator_to_joint.translation_mm == pytest.approx((0.0, 0.0, 0.0), abs=1.0e-8)
        assert _parse_vector(revolute.find("axis").attrib["xyz"]) == pytest.approx(
            morphology.joint_axes_local[frame_name]
        )
        assert _rotate(
            morphology.joint_transforms[frame_name].rotation_matrix,
            _parse_vector(revolute.find("axis").attrib["xyz"]),
        ) == pytest.approx(morphology.joint_axes_hand[frame_name], abs=1.0e-10)

        actuator_xml = next(link for link in root.findall("link") if link.attrib["name"] == actuator_link)
        for element_name in ("visual", "collision"):
            elements = actuator_xml.findall(element_name)
            assert len(elements) == 2
            box = next(element for element in elements if element.find("geometry/box") is not None)
            cylinder = next(
                element
                for element in elements
                if element.find("geometry/cylinder") is not None
            )
            assert _parse_vector(box.find("origin").attrib["xyz"]) == pytest.approx(
                (0.0, -0.01275, 0.0)
            )
            assert _parse_vector(box.find("geometry/box").attrib["size"]) == pytest.approx(
                (0.019, 0.0255, 0.022)
            )
            assert _parse_vector(cylinder.find("origin").attrib["rpy"]) == pytest.approx(
                (0.0, 0.0, 0.0)
            )
            cylinder_geometry = cylinder.find("geometry/cylinder")
            assert float(cylinder_geometry.attrib["radius"]) == pytest.approx(0.0095)
            assert float(cylinder_geometry.attrib["length"]) == pytest.approx(0.022)

    for finger in ("index", "middle", "ring", "little"):
        linkage_frame = f"{finger}_linkage_frame"
        expected_links.add(linkage_frame)
        linkage_joint = f"{finger}_mcp_fe_to_linkage_frame"
        expected_joint_names.add(linkage_joint)
        assert joint_map[linkage_joint].find("parent").attrib["link"] == f"{finger}_mcp_fe_output"
        linkage_transform = _parse_origin(joint_map[linkage_joint].find("origin"))
        expected_rotation = RigidTransform.rotation_about_axis((0.0, 0.0, 1.0), -math.pi / 2.0)
        _assert_transform_equal(linkage_transform, expected_rotation)
        assert linkage_transform.transform_vector((1.0, 0.0, 0.0)) == pytest.approx(
            (0.0, -1.0, 0.0),
            abs=1.0e-10,
        )
        assert linkage_transform.transform_vector((0.0, 1.0, 0.0)) == pytest.approx(
            (1.0, 0.0, 0.0),
            abs=1.0e-10,
        )
        assert linkage_transform.transform_vector((0.0, 0.0, 1.0)) == pytest.approx(
            (0.0, 0.0, 1.0),
            abs=1.0e-10,
        )

    assert link_names == expected_links
    assert set(joint_map) == expected_joint_names


def test_urdf_rejects_wrong_morphology_and_invalid_robot_name() -> None:
    with pytest.raises(TypeError, match="HandMorphology"):
        compile_hand_urdf(object())
    with pytest.raises(ValueError, match="robot_name"):
        compile_hand_urdf(make_morphology(), robot_name="clear hand")
