from dataclasses import FrozenInstanceError
import math

import pytest

from cmd.geometry import RigidTransform
from cmd.morphology import (
    BASELINE_LINKAGE_V0,
    FingertipFamilyParameters,
    FingertipLengths,
    HandLayout,
    HandLayoutParameters,
    HandMorphology,
    HandMorphologyParameters,
    HandPlanePoint,
)
from cmd.morphology.hand_layout import (
    FE_ACTUATOR_MOUNT,
    HAND_X_LONGITUDINAL,
    HAND_Y_FE_AXIS,
    HAND_Z_DORSAL,
    MCP_AA_ACTUATOR_MOUNT,
    ORDINARY_MCP_AA_TO_MCP_FE,
    ORDINARY_MCP_FE_TO_LINKAGE,
    THUMB_CMC_RAA_ACTUATOR_MOUNT,
    THUMB_CMC_RAA_TO_MCP_FE_DISTANCE_MM,
    THUMB_CMC_RO_ACTUATOR_MOUNT,
    THUMB_IP_FE_ACTUATOR_MOUNT,
    THUMB_MCP_FE_ACTUATOR_MOUNT,
    THUMB_MCP_FE_TO_IP_FE_DISTANCE_MM,
)


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


def test_linkage_base_transforms_are_derived_from_each_mcp_fe_frame() -> None:
    layout = HandLayout(make_layout_parameters())
    joints = layout.joint_transforms
    linkages = layout.linkage_transforms

    assert set(linkages) == {
        "index_linkage",
        "middle_linkage",
        "ring_linkage",
        "little_linkage",
    }
    for finger in ("index", "middle", "ring", "little"):
        relative = joints[f"{finger}_mcp_fe"].inverse() @ linkages[f"{finger}_linkage"]
        assert relative.translation_mm == pytest.approx(
            ORDINARY_MCP_FE_TO_LINKAGE.translation_mm
        )
        assert [value for row in relative.rotation_matrix for value in row] == pytest.approx(
            [
                value
                for row in ORDINARY_MCP_FE_TO_LINKAGE.rotation_matrix
                for value in row
            ]
        )
        assert relative.transform_vector((1.0, 0.0, 0.0)) == pytest.approx(
            (1.0, 0.0, 0.0)
        )
        assert relative.transform_vector((0.0, 1.0, 0.0)) == pytest.approx(
            (0.0, 1.0, 0.0)
        )
        assert relative.transform_vector((0.0, 0.0, 1.0)) == pytest.approx(
            (0.0, 0.0, 1.0)
        )


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


def test_thumb_orientation_and_ro_distance_propagate_downstream() -> None:
    base = HandLayout(make_layout_parameters(theta=0.0, phi=0.0))
    ro_distance_changed = HandLayout(
        make_layout_parameters(theta=0.0, phi=0.0, ro_raa=40.0)
    )
    phi_changed = HandLayout(make_layout_parameters(theta=0.0, phi=0.2))

    assert ro_distance_changed.joint_transforms["thumb_cmc_raa"].translation_mm != pytest.approx(
        base.joint_transforms["thumb_cmc_raa"].translation_mm
    )
    assert ro_distance_changed.joint_transforms["thumb_ip_fe"].translation_mm != pytest.approx(
        base.joint_transforms["thumb_ip_fe"].translation_mm
    )
    assert phi_changed.joint_transforms["thumb_mcp_fe"].translation_mm == pytest.approx(
        base.joint_transforms["thumb_mcp_fe"].translation_mm
    )
    assert phi_changed.joint_transforms["thumb_ip_fe"].translation_mm != pytest.approx(
        base.joint_transforms["thumb_ip_fe"].translation_mm
    )


def test_joint_axes_and_actuator_mounts_are_derived_from_joint_frames() -> None:
    layout = HandLayout(make_layout_parameters(theta=0.35, phi=-0.2))
    axes_hand = layout.joint_axes_hand
    actuators = layout.actuator_transforms

    assert axes_hand["index_mcp_aa"] == pytest.approx((0.0, 0.0, 1.0))
    assert axes_hand["index_mcp_fe"] == pytest.approx((0.0, 1.0, 0.0))
    assert axes_hand["thumb_cmc_ro"] == pytest.approx(
        (-math.sin(0.35), math.cos(0.35), 0.0)
    )
    assert axes_hand["thumb_cmc_raa"] == pytest.approx((0.0, 0.0, 1.0))
    assert axes_hand["thumb_mcp_fe"] == pytest.approx(
        (-math.sin(0.15), math.cos(0.15), 0.0)
    )
    assert axes_hand["thumb_ip_fe"] == axes_hand["thumb_mcp_fe"]

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
        THUMB_IP_FE_ACTUATOR_MOUNT,
    ):
        assert mount.transform_vector((0.0, 0.0, 1.0)) == pytest.approx((0.0, 1.0, 0.0))
    assert THUMB_CMC_RAA_ACTUATOR_MOUNT.transform_vector((0.0, 0.0, 1.0)) == pytest.approx(
        (0.0, 0.0, 1.0)
    )


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


@pytest.mark.parametrize(
    "kwargs",
    [
        {"theta": float("nan")},
        {"phi": float("inf")},
        {"ro_raa": 0.0},
    ],
)
def test_hand_layout_parameters_reject_invalid_independent_values(kwargs) -> None:
    with pytest.raises(ValueError):
        make_layout_parameters(**kwargs)


def test_hand_parameters_are_immutable() -> None:
    parameters = make_parameters()
    with pytest.raises(FrozenInstanceError):
        parameters.layout.theta_rad = 0.0
    with pytest.raises(FrozenInstanceError):
        parameters.layout.middle_mcp_aa.x_mm = 0.0


def test_hand_morphology_rejects_total_fingertip_length_not_long_enough() -> None:
    with pytest.raises(ValueError, match="greater than"):
        HandMorphologyParameters(
            fingertip=FingertipFamilyParameters(),
            fingertip_lengths=FingertipLengths(
                index_mm=10.0,
                middle_mm=40.0,
                ring_mm=40.0,
                little_mm=40.0,
                thumb_mm=40.0,
            ),
            linkage=BASELINE_LINKAGE_V0.parameters,
            layout=make_layout_parameters(),
        )
