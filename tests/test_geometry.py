"""Tests for shared SE(3) geometry primitives."""

import math

import pytest

from cmd.geometry import RigidTransform


def test_rigid_transform_identity_composition_rotation_and_inverse() -> None:
    identity = RigidTransform.identity()
    translation = RigidTransform.from_translation((10.0, -2.0, 3.0))
    rotation = RigidTransform.rotation_about_axis((0.0, 0.0, 1.0), math.pi / 2.0)
    parent_from_child = translation @ rotation

    assert identity @ parent_from_child == parent_from_child
    assert parent_from_child @ identity == parent_from_child
    assert parent_from_child.transform_point((1.0, 0.0, 0.0)) == pytest.approx(
        (10.0, -1.0, 3.0)
    )
    assert rotation.transform_vector((1.0, 0.0, 0.0)) == pytest.approx(
        (0.0, 1.0, 0.0)
    )
    recovered = parent_from_child.inverse() @ parent_from_child
    assert recovered.translation_mm == pytest.approx((0.0, 0.0, 0.0), abs=1.0e-12)
    assert [value for row in recovered.rotation_matrix for value in row] == pytest.approx(
        [value for row in identity.rotation_matrix for value in row],
        abs=1.0e-12,
    )


def test_rigid_transform_composes_parent_child_transforms() -> None:
    hand_from_parent = RigidTransform.from_translation((10.0, 0.0, 0.0))
    parent_from_child = RigidTransform(
        rotation_matrix=RigidTransform.rotation_about_axis(
            (0.0, 0.0, 1.0),
            math.pi / 2.0,
        ).rotation_matrix,
        translation_mm=(2.0, 0.0, 0.0),
    )

    hand_from_child = hand_from_parent @ parent_from_child
    assert hand_from_child.transform_point((1.0, 0.0, 0.0)) == pytest.approx(
        (12.0, 1.0, 0.0)
    )
