from dataclasses import FrozenInstanceError
import math

import pytest

from cmd.morphology import (
    FE_ACTUATOR_MOUNT,
    HAND_X_LONGITUDINAL,
    HAND_Y_FE_AXIS,
    HAND_Z_DORSAL,
    MCP_AA_ACTUATOR_MOUNT,
    ORDINARY_MCP_AA_TO_MCP_FE,
    HandMorphology,
    HandMorphologyParameters,
    HandPlanePoint,
    RigidTransform,
    THUMB_CMC_RAA_ACTUATOR_MOUNT,
    THUMB_CMC_RO_ACTUATOR_MOUNT,
    THUMB_MCP_FE_ACTUATOR_MOUNT,
)


def make_parameters(
    *,
    middle=(40.0, 5.0),
    ring=(35.0, -5.0),
    little=(25.0, -15.0),
    thumb=(5.0, 30.0),
    theta=0.2,
    ro_raa=35.0,
    raa_mcp=28.0,
    phi=-0.3,
):
    return HandMorphologyParameters(
        middle_mcp_aa=HandPlanePoint(*middle),
        ring_mcp_aa=HandPlanePoint(*ring),
        little_mcp_aa=HandPlanePoint(*little),
        thumb_cmc_ro=HandPlanePoint(*thumb),
        theta_rad=theta,
        ro_raa_distance_mm=ro_raa,
        raa_mcp_fe_distance_mm=raa_mcp,
        phi_rad=phi,
    )


def test_rigid_transform_identity_composition_rotation_and_inverse() -> None:
    identity = RigidTransform.identity()
    translation = RigidTransform.from_translation((10.0, -2.0, 3.0))
    rotation = RigidTransform.rotation_about_axis(HAND_Z_DORSAL, math.pi / 2.0)
    parent_to_child = translation @ rotation

    assert identity @ parent_to_child == parent_to_child
    assert parent_to_child @ identity == parent_to_child
    assert parent_to_child.transform_point((1.0, 0.0, 0.0)) == pytest.approx(
        (10.0, -1.0, 3.0)
    )
    assert rotation.transform_vector(HAND_X_LONGITUDINAL) == pytest.approx(
        HAND_Y_FE_AXIS
    )
    recovered = parent_to_child.inverse() @ parent_to_child
    assert recovered.translation_mm == pytest.approx((0.0, 0.0, 0.0), abs=1.0e-12)
    assert [value for row in recovered.rotation_matrix for value in row] == pytest.approx(
        [value for row in identity.rotation_matrix for value in row],
        abs=1.0e-12,
    )


def test_rigid_transform_composes_parent_child_transforms() -> None:
    hand_from_parent = RigidTransform.from_translation((10.0, 0.0, 0.0))
    parent_from_child = RigidTransform(
        rotation_matrix=RigidTransform.rotation_about_axis(
            HAND_Z_DORSAL,
            math.pi / 2.0,
        ).rotation_matrix,
        translation_mm=(2.0, 0.0, 0.0),
    )

    hand_from_child = hand_from_parent @ parent_from_child
    assert hand_from_child.transform_point((1.0, 0.0, 0.0)) == pytest.approx(
        (12.0, 1.0, 0.0)
    )


def test_hand_root_and_ordinary_finger_frames_are_transform_derived() -> None:
    parameters = make_parameters()
    morphology = HandMorphology(parameters)
    joints = morphology.joint_transforms

    assert joints["index_mcp_aa"] == RigidTransform.identity()
    assert HAND_X_LONGITUDINAL == (1.0, 0.0, 0.0)
    assert HAND_Y_FE_AXIS == (0.0, 1.0, 0.0)
    assert HAND_Z_DORSAL == (0.0, 0.0, 1.0)
    assert ORDINARY_MCP_AA_TO_MCP_FE.translation_mm == pytest.approx(
        (23.5, 0.0, 16.75)
    )
    assert ORDINARY_MCP_AA_TO_MCP_FE.rotation_matrix == RigidTransform.identity().rotation_matrix

    for finger in ("index", "middle", "ring", "little"):
        relative = joints[f"{finger}_mcp_aa"].inverse() @ joints[f"{finger}_mcp_fe"]
        assert relative == ORDINARY_MCP_AA_TO_MCP_FE
        assert joints[f"{finger}_mcp_aa"].rotation_matrix == RigidTransform.identity().rotation_matrix
        assert joints[f"{finger}_mcp_fe"].rotation_matrix == RigidTransform.identity().rotation_matrix

    assert joints["middle_mcp_aa"].translation_mm == pytest.approx((40.0, 5.0, 0.0))
    assert joints["ring_mcp_aa"].translation_mm == pytest.approx((35.0, -5.0, 0.0))
    assert joints["little_mcp_aa"].translation_mm == pytest.approx((25.0, -15.0, 0.0))
    assert not {"index_mcp_aa", "index_mcp_fe", "middle_mcp_fe"} & {
        field.name for field in parameters.__dataclass_fields__.values()
    }


def test_ordinary_placement_changes_propagate_without_fe_design_variables() -> None:
    first = HandMorphology(make_parameters())
    moved = HandMorphology(make_parameters(middle=(43.0, 9.0)))

    assert moved.joint_transforms["middle_mcp_aa"].translation_mm == pytest.approx(
        (43.0, 9.0, 0.0)
    )
    assert moved.joint_transforms["middle_mcp_fe"].translation_mm == pytest.approx(
        (66.5, 9.0, 16.75)
    )
    assert first.joint_transforms["ring_mcp_aa"] == moved.joint_transforms["ring_mcp_aa"]
    assert first.joint_transforms["little_mcp_fe"] == moved.joint_transforms["little_mcp_fe"]
    assert "middle_mcp_fe" not in HandMorphologyParameters.__dataclass_fields__


def test_thumb_positions_and_orientations_follow_the_low_dimensional_chain() -> None:
    parameters = make_parameters(thumb=(5.0, 30.0), theta=0.4, ro_raa=20.0, raa_mcp=12.0, phi=-0.25)
    morphology = HandMorphology(parameters)
    joints = morphology.joint_transforms

    assert joints["thumb_cmc_ro"].translation_mm == pytest.approx((5.0, 30.0, 0.0))
    assert joints["thumb_cmc_raa"].translation_mm == pytest.approx(
        (5.0 + 20.0 * math.cos(0.4), 30.0 + 20.0 * math.sin(0.4), 0.0)
    )
    assert joints["thumb_mcp_fe"].translation_mm == pytest.approx(
        (5.0 + 32.0 * math.cos(0.4), 30.0 + 32.0 * math.sin(0.4), 0.0)
    )
    assert joints["thumb_cmc_ro"].transform_vector(HAND_X_LONGITUDINAL) == pytest.approx(
        (math.cos(0.4), math.sin(0.4), 0.0)
    )
    assert joints["thumb_mcp_fe"].transform_vector(HAND_X_LONGITUDINAL) == pytest.approx(
        (math.cos(0.15), math.sin(0.15), 0.0)
    )

    phi_changed = HandMorphology(
        make_parameters(
            thumb=(5.0, 30.0),
            theta=0.4,
            ro_raa=20.0,
            raa_mcp=12.0,
            phi=0.2,
        )
    )
    assert phi_changed.joint_transforms["thumb_mcp_fe"].translation_mm == pytest.approx(
        joints["thumb_mcp_fe"].translation_mm
    )
    phi_rotation = phi_changed.joint_transforms["thumb_mcp_fe"].rotation_matrix
    original_rotation = joints["thumb_mcp_fe"].rotation_matrix
    assert [value for row in phi_rotation for value in row] != pytest.approx(
        [value for row in original_rotation for value in row]
    )

    theta_changed = HandMorphology(make_parameters(thumb=(5.0, 30.0), theta=0.7, phi=-0.25))
    assert theta_changed.joint_transforms["thumb_cmc_raa"].translation_mm != pytest.approx(
        joints["thumb_cmc_raa"].translation_mm
    )
    assert theta_changed.joint_transforms["thumb_mcp_fe"].translation_mm != pytest.approx(
        joints["thumb_mcp_fe"].translation_mm
    )


def test_thumb_distances_change_only_their_downstream_chain() -> None:
    base = HandMorphology(make_parameters(theta=0.0, phi=0.0))
    ro_distance_changed = HandMorphology(make_parameters(theta=0.0, phi=0.0, ro_raa=40.0))
    mcp_distance_changed = HandMorphology(make_parameters(theta=0.0, phi=0.0, raa_mcp=40.0))

    assert ro_distance_changed.joint_transforms["thumb_cmc_raa"].translation_mm != pytest.approx(
        base.joint_transforms["thumb_cmc_raa"].translation_mm
    )
    assert ro_distance_changed.joint_transforms["thumb_mcp_fe"].translation_mm != pytest.approx(
        base.joint_transforms["thumb_mcp_fe"].translation_mm
    )
    assert mcp_distance_changed.joint_transforms["thumb_cmc_raa"] == base.joint_transforms["thumb_cmc_raa"]
    assert mcp_distance_changed.joint_transforms["thumb_mcp_fe"].translation_mm != pytest.approx(
        base.joint_transforms["thumb_mcp_fe"].translation_mm
    )


def test_joint_axes_and_actuator_mounts_are_derived_from_joint_frames() -> None:
    morphology = HandMorphology(make_parameters(theta=0.35, phi=-0.2))
    axes_hand = morphology.joint_axes_hand
    actuators = morphology.actuator_transforms

    assert axes_hand["index_mcp_aa"] == pytest.approx((0.0, 0.0, 1.0))
    assert axes_hand["index_mcp_fe"] == pytest.approx((0.0, 1.0, 0.0))
    assert axes_hand["thumb_cmc_ro"] == pytest.approx(
        (-math.sin(0.35), math.cos(0.35), 0.0)
    )
    assert axes_hand["thumb_cmc_raa"] == pytest.approx((0.0, 0.0, 1.0))
    assert axes_hand["thumb_mcp_fe"] == pytest.approx(
        (-math.sin(0.15), math.cos(0.15), 0.0)
    )

    for joint_name, actuator_pose in actuators.items():
        assert actuator_pose.transform_vector((0.0, 0.0, 1.0)) == pytest.approx(
            axes_hand[joint_name]
        )

    assert MCP_AA_ACTUATOR_MOUNT.transform_vector((0.0, 0.0, 1.0)) == pytest.approx(
        (0.0, 0.0, 1.0)
    )
    for mount in (
        FE_ACTUATOR_MOUNT,
        THUMB_CMC_RO_ACTUATOR_MOUNT,
        THUMB_MCP_FE_ACTUATOR_MOUNT,
    ):
        assert mount.transform_vector((0.0, 0.0, 1.0)) == pytest.approx((0.0, 1.0, 0.0))
    assert THUMB_CMC_RAA_ACTUATOR_MOUNT.transform_vector((0.0, 0.0, 1.0)) == pytest.approx(
        (0.0, 0.0, 1.0)
    )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"theta": float("nan")},
        {"phi": float("inf")},
        {"ro_raa": 0.0},
        {"raa_mcp": -1.0},
    ],
)
def test_hand_morphology_parameters_reject_invalid_independent_values(kwargs) -> None:
    with pytest.raises(ValueError):
        make_parameters(**kwargs)


def test_hand_morphology_parameters_are_immutable() -> None:
    parameters = make_parameters()
    with pytest.raises(FrozenInstanceError):
        parameters.theta_rad = 0.0
    with pytest.raises(FrozenInstanceError):
        parameters.middle_mcp_aa.x_mm = 0.0
