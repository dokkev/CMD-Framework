import math
from dataclasses import replace

import pytest

from cmd.geometry import RigidTransform
from cmd.morphology import (
    BASELINE_LINKAGE_V0,
    SELECTED_LINKAGE_V1,
    FingertipFamilyParameters,
    FingertipLengths,
    HandLayout,
    HandLayoutParameters,
    HandMorphology,
    HandMorphologyParameters,
    HandPlanePoint,
)
from cmd.morphology.hand_layout import (
    HAND_X_LONGITUDINAL,
    HAND_Y_FE_AXIS,
    HAND_Z_DORSAL,
    ORDINARY_MCP_AA_TO_MCP_FE,
    THUMB_CMC_RAA_TO_MCP_FE_DISTANCE_MM,
    THUMB_MCP_FE_TO_IP_FE_DISTANCE_MM,
    distal_link_transform,
)
from cmd.morphology.linkage import LinkageKinematics


def make_layout_parameters(
    *,
    middle=(40.0, 5.0),
    ring=(35.0, -5.0),
    little=(25.0, -15.0),
    thumb=(5.0, 30.0),
    theta=0.2,
    ro_raa=35.0,
    phi=-0.3,
) -> HandLayoutParameters:
    return HandLayoutParameters(
        middle_mcp_aa=HandPlanePoint(*middle),
        ring_mcp_aa=HandPlanePoint(*ring),
        little_mcp_aa=HandPlanePoint(*little),
        thumb_cmc_ro=HandPlanePoint(*thumb),
        theta_rad=theta,
        ro_raa_distance_mm=ro_raa,
        phi_rad=phi,
    )


def make_parameters(**layout_kwargs) -> HandMorphologyParameters:
    return HandMorphologyParameters(
        fingertip=FingertipFamilyParameters(),
        fingertip_lengths=FingertipLengths(
            index_mm=40.0,
            middle_mm=41.0,
            ring_mm=42.0,
            little_mm=43.0,
            thumb_mm=44.0,
        ),
        linkage=BASELINE_LINKAGE_V0.parameters,
        layout=make_layout_parameters(**layout_kwargs),
    )


def test_hand_root_and_ordinary_finger_frames_are_transform_derived() -> None:
    parameters = make_parameters()
    hand = HandMorphology(parameters)
    joints = hand.layout.joint_transforms

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
    assert "middle_mcp_fe" not in HandLayoutParameters.__dataclass_fields__


def test_ordinary_placement_changes_propagate_without_fe_design_variables() -> None:
    first = HandLayout(make_parameters().layout)
    moved = HandLayout(make_parameters(middle=(43.0, 9.0)).layout)

    assert moved.joint_transforms["middle_mcp_aa"].translation_mm == pytest.approx(
        (43.0, 9.0, 0.0)
    )
    assert moved.joint_transforms["middle_mcp_fe"].translation_mm == pytest.approx(
        (66.5, 9.0, 16.75)
    )
    assert first.joint_transforms["ring_mcp_aa"] == moved.joint_transforms["ring_mcp_aa"]
    assert first.joint_transforms["little_mcp_fe"] == moved.joint_transforms["little_mcp_fe"]


def test_thumb_fixed_chain_is_derived_from_the_low_dimensional_layout() -> None:
    layout = HandLayout(
        make_layout_parameters(thumb=(5.0, 30.0), theta=0.4, ro_raa=20.0, phi=-0.25)
    )
    joints = layout.joint_transforms

    assert joints["thumb_cmc_ro"].translation_mm == pytest.approx((5.0, 30.0, 0.0))
    assert joints["thumb_cmc_raa"].translation_mm == pytest.approx(
        (5.0 + 20.0 * math.cos(0.4), 30.0 + 20.0 * math.sin(0.4), 0.0)
    )
    mcp_fe_length = 20.0 + THUMB_CMC_RAA_TO_MCP_FE_DISTANCE_MM
    assert joints["thumb_mcp_fe"].translation_mm == pytest.approx(
        (5.0 + mcp_fe_length * math.cos(0.4), 30.0 + mcp_fe_length * math.sin(0.4), 0.0)
    )
    assert joints["thumb_cmc_ro"].transform_vector(HAND_X_LONGITUDINAL) == pytest.approx(
        (math.cos(0.4), math.sin(0.4), 0.0)
    )
    assert joints["thumb_mcp_fe"].transform_vector(HAND_X_LONGITUDINAL) == pytest.approx(
        (math.cos(0.15), math.sin(0.15), 0.0)
    )
    mcp_to_ip = joints["thumb_mcp_fe"].inverse() @ joints["thumb_ip_fe"]
    assert mcp_to_ip.translation_mm == pytest.approx(
        (THUMB_MCP_FE_TO_IP_FE_DISTANCE_MM, 0.0, 0.0)
    )
    assert [value for row in mcp_to_ip.rotation_matrix for value in row] == pytest.approx(
        [value for row in RigidTransform.identity().rotation_matrix for value in row]
    )
    assert joints["thumb_ip_fe"].rotation_matrix == joints["thumb_mcp_fe"].rotation_matrix
    assert "raa_mcp_fe_distance_mm" not in HandLayoutParameters.__dataclass_fields__


def test_whole_hand_materializes_one_shared_linkage_and_five_tips() -> None:
    parameters = make_parameters()
    hand = HandMorphology(parameters)

    assert hand.linkage.parameters is parameters.linkage
    assert tuple(hand.fingertips) == ("index", "middle", "ring", "little", "thumb")
    assert tuple(tip.total_length_mm for tip in hand.fingertips.values()) == pytest.approx(
        (40.0, 41.0, 42.0, 43.0, 44.0)
    )
    assert all(tip.parameters.family is parameters.fingertip for tip in hand.fingertips.values())
    assert "layout" in HandMorphologyParameters.__dataclass_fields__
    assert "fingertip_lengths" in HandMorphologyParameters.__dataclass_fields__


def test_fingertip_rotation_axis_lands_on_the_dip_and_ip_joints() -> None:
    parameters = make_parameters()
    hand = HandMorphology(parameters)
    tips = hand.fingertip_transforms
    reference = hand.linkage.reference_positions

    for finger in ("index", "middle", "ring", "little"):
        b_in_hand = hand.layout.linkage_transforms[f"{finger}_linkage"].transform_point(
            (reference.b.x_mm, 0.0, reference.b.z_mm)
        )
        axis = hand.fingertips[finger].rotation_axis_mm(parameters.linkage.bc_length_mm)
        assert tips[finger].transform_point(axis) == pytest.approx(b_in_hand)
        assert tips[finger].rotation_matrix == hand.layout.linkage_transforms[
            f"{finger}_linkage"
        ].rotation_matrix
    thumb_ip = hand.layout.joint_transforms["thumb_ip_fe"]
    thumb_axis = hand.fingertips["thumb"].rotation_axis_mm(parameters.linkage.bc_length_mm)
    assert tips["thumb"].transform_point(thumb_axis) == pytest.approx(thumb_ip.translation_mm)

    # With l_ep = BC, the linkage C point lands on the fingertip's pad tip.
    c_in_hand = hand.layout.linkage_transforms["index_linkage"].transform_point(
        (reference.c.x_mm, 0.0, reference.c.z_mm)
    )
    tip_end = (hand.fingertips["index"].total_length_mm, 0.0, hand.fingertips["index"].rotation_axis_z_mm)
    assert tips["index"].transform_point(tip_end) == pytest.approx(c_in_hand)


def test_rotation_axis_must_fall_within_every_carrier() -> None:
    with pytest.raises(ValueError, match="little_mm must exceed linkage.bc_length_mm"):
        replace(
            make_parameters(),
            fingertip_lengths=FingertipLengths(40.0, 41.0, 42.0, 20.0, 44.0),
        )


def test_distal_link_transform_follows_flexion_about_the_dip_axis() -> None:
    linkage = SELECTED_LINKAGE_V1
    pose = LinkageKinematics(linkage).solve(0.6)
    frame = distal_link_transform(pose)

    assert frame.transform_point((0.0, 0.0, 0.0)) == pytest.approx((pose.b.x_mm, 0.0, pose.b.z_mm))
    assert frame.transform_point((linkage.parameters.bc_length_mm, 0.0, 0.0)) == pytest.approx(
        (pose.c.x_mm, 0.0, pose.c.z_mm)
    )
    assert frame.transform_vector((0.0, 1.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))
    assert frame.transform_vector((0.0, 0.0, 1.0))[2] > 0.0
