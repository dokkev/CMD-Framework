# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3's contact-interface variants: their geometry, and that only the pads' surface differs between them."""

from __future__ import annotations

import math

import numpy as np
import pytest

from code_isaaclab.contact_interface import (
    CROWNED,
    FLAT,
    PadProfile,
    contact_face_height,
    pad_mesh,
    pad_profile_polygon,
)

HALF = (0.0178, 0.0015, 0.0095)


def test_flat_profile_is_the_box():
    points, counts, _ = pad_mesh(HALF, FLAT)
    assert len(points) == 8 and counts[:2] == [4, 4]
    assert np.allclose(np.abs(points).max(axis=0), HALF)


def test_crowned_profile_keeps_the_envelope_and_mounting_face():
    outline = pad_profile_polygon(HALF, CROWNED)
    flat = pad_profile_polygon(HALF, FLAT)
    assert outline[:, 0].min() == pytest.approx(-HALF[0]) and outline[:, 0].max() == pytest.approx(HALF[0])
    assert outline[:, 1].max() == pytest.approx(HALF[1])  # the middle of the contact face is the flat face
    assert outline[:, 1].min() == pytest.approx(flat[:, 1].min())  # the same mounting face
    ends = contact_face_height(np.array([-HALF[0], HALF[0]]), HALF[0], HALF[1], 2e-3)
    assert np.allclose(ends, HALF[1] - 2e-3)  # 2 mm lower at the ends
    # the face is a circular arc: every sample is on one circle
    xs = np.linspace(-HALF[0], HALF[0], 11)
    radius = HALF[0] ** 2 / (2 * 2e-3) + 1e-3
    centre = HALF[1] - radius
    assert np.allclose(np.hypot(xs, contact_face_height(xs, HALF[0], HALF[1], 2e-3) - centre), radius)


@pytest.mark.parametrize("profile", [CROWNED, PadProfile("crosswise", crown_mm=0.5, across_width=True)])
def test_crowned_mesh_is_convex_and_closed(profile):
    points, counts, indices = pad_mesh(HALF, profile)
    # every face's outward normal sees all other points behind it (convex polyhedron)
    start = 0
    centroid = points.mean(axis=0)
    for count in counts:
        face = points[indices[start : start + count]]
        start += count
        normal = np.cross(face[1] - face[0], face[2] - face[0])
        normal /= np.linalg.norm(normal)
        assert normal @ (face[0] - centroid) > 0  # outward
        assert np.all((points - face[0]) @ normal <= 1e-9)
    # closed: every edge is shared by exactly two faces
    edges = {}
    start = 0
    for count in counts:
        ring = indices[start : start + count]
        start += count
        for a, b in zip(ring, ring[1:] + ring[:1], strict=True):
            edges[frozenset((a, b))] = edges.get(frozenset((a, b)), 0) + 1
    assert set(edges.values()) == {2}


def test_a_crown_deeper_than_the_pad_is_refused():
    with pytest.raises(ValueError):
        pad_profile_polygon(HALF, PadProfile("too_deep", crown_mm=3.0))
    with pytest.raises(ValueError):
        PadProfile("negative", crown_mm=-1.0)


def _usd_attributes(path):
    from pxr import Usd

    stage = Usd.Stage.Open(str(path))
    found = {}
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            found[(str(prim.GetPath()), attr.GetName())] = attr.Get()
        for rel in prim.GetRelationships():
            found[(str(prim.GetPath()), rel.GetName())] = tuple(str(t) for t in rel.GetTargets())
        found[(str(prim.GetPath()), "@type")] = prim.GetTypeName()
        found[(str(prim.GetPath()), "@apis")] = tuple(prim.GetAppliedSchemas())
    return found


def test_variant_assets_differ_only_in_the_pad_surfaces():
    from code_isaaclab.hands.code_variants import STUDY3_DESCRIPTION, variant_asset

    if not STUDY3_DESCRIPTION.exists():
        pytest.skip(f"{STUDY3_DESCRIPTION} not exported")
    flat, crowned = (_usd_attributes(variant_asset(v)) for v in ("flat", "crowned"))
    assert flat.keys() == crowned.keys()  # same prims, same attributes, same schemas
    differing = {key for key in flat if flat[key] != crowned[key]}
    assert differing, "the variants must differ"
    for prim, name in differing:
        assert prim.endswith("_pad"), (prim, name)
        assert name in ("points", "faceVertexCounts", "faceVertexIndices", "extent"), (prim, name)
    pads = {prim for prim, _ in differing}
    assert len(pads) == 6  # three fingers x (proximal, distal)


def test_distal_width_crown_changes_only_the_selected_colliders(tmp_path):
    from code_isaaclab.hands.description_hands import description_asset
    from code_isaaclab.tasks.rotate_cube.grasp import HANDS

    distal = ("index_distal_pad", "middle_distal_pad", "ring_distal_pad")
    profile = PadProfile("distal_width", crown_mm=0.5, across_width=True, pads=distal)
    flat, curved = (
        _usd_attributes(description_asset(HANDS["O"], profile=p, out_dir=tmp_path, thumb_pad=True))
        for p in (FLAT, profile)
    )
    assert flat.keys() == curved.keys()
    differing = {(prim, attr) for prim, attr in flat if flat[(prim, attr)] != curved[(prim, attr)]}
    assert {prim.rsplit("/", 1)[-1] for prim, _ in differing} == set(distal)
    assert {attr for _, attr in differing} <= {"points", "faceVertexCounts", "faceVertexIndices", "extent"}
    for prim, attr in differing:
        if attr == "points":
            box, crown = np.asarray(flat[(prim, attr)]), np.asarray(curved[(prim, attr)])
            np.testing.assert_allclose(box.min(axis=0), crown.min(axis=0))
            np.testing.assert_allclose(box.max(axis=0), crown.max(axis=0))
            edge = np.isclose(np.abs(crown[:, 2]), box[:, 2].max())
            assert crown[edge, 1].max() == pytest.approx(box[:, 1].max() - 0.0005)


def test_code_tasks_share_everything_but_the_hand_asset():
    import gymnasium as gym

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.hands.code_variants import STUDY3_DESCRIPTION

    if not STUDY3_DESCRIPTION.exists():
        pytest.skip(f"{STUDY3_DESCRIPTION} not exported")
    from code_isaaclab.tasks.reorient.code_env_cfg import CoDeCrownedReorientEnvCfg, CoDeFlatReorientEnvCfg

    for task in ("CoDe-Reorient-Cube-CoDeFlat", "CoDe-Reorient-Cube-CoDeCrowned"):
        spec = gym.registry[task]
        assert spec.kwargs["rsl_rl_cfg_entry_point"].endswith("agents:CoDeReorientPPORunnerCfg")
    flat, crowned = CoDeFlatReorientEnvCfg(), CoDeCrownedReorientEnvCfg()
    assert flat.actuated_joint_names == crowned.actuated_joint_names and len(flat.actuated_joint_names) == 13
    assert flat.fingertip_body_names == crowned.fingertip_body_names
    assert flat.decimation == crowned.decimation and flat.episode_length_s == crowned.episode_length_s
    assert flat.commands.object_pose.to_dict() == crowned.commands.object_pose.to_dict()
    assert flat.rewards.to_dict() == crowned.rewards.to_dict()
    assert flat.terminations.to_dict() == crowned.terminations.to_dict()
    assert flat.observations.to_dict() == crowned.observations.to_dict()
    assert flat.actions.to_dict() == crowned.actions.to_dict()
    assert flat.scene.object.to_dict() == crowned.scene.object.to_dict()
    r_flat, r_crowned = flat.scene.robot.to_dict(), crowned.scene.robot.to_dict()
    assert r_flat["spawn"].pop("usd_path") != r_crowned["spawn"].pop("usd_path")
    assert r_flat == r_crowned  # actuators, gains, armature, initial state: identical
    assert flat.events.to_dict() == crowned.events.to_dict()
    assert math.isclose(flat.commands.object_pose.max_angle, math.radians(30.0))


DOME = PadProfile("dome", crown_mm=1.0, crown_wid_mm=1.0)


def test_dome_keeps_the_middle_and_lowers_sides_and_corners():
    from code_isaaclab.contact_interface import dome_face_height, surface_report

    hx, hy, hz = HALF
    assert dome_face_height(0.0, 0.0, HALF, DOME) == pytest.approx(hy)  # the middle is the flat face
    assert dome_face_height(hx, 0.0, HALF, DOME) == pytest.approx(hy - 1e-3)
    assert dome_face_height(0.0, hz, HALF, DOME) == pytest.approx(hy - 1e-3)
    assert dome_face_height(hx, hz, HALF, DOME) == pytest.approx(hy - 2e-3)
    points, _, _ = pad_mesh(HALF, DOME)
    assert len(points) <= 64  # Newton's convex-hull vertex limit
    assert points[:, 1].max() == pytest.approx(hy) and points[:, 1].min() == pytest.approx(-hy)
    assert np.allclose(np.abs(points[:, [0, 2]]).max(axis=0), (hx, hz))
    body = {"boxes": [{"name": "p_pad", "half_extents": list(HALF)}]}
    row = surface_report([body], DOME)[0]
    assert row["thickness_middle_mm"] == pytest.approx(3.0)
    assert row["thickness_ends_mm"] == pytest.approx(2.0) and row["thickness_sides_mm"] == pytest.approx(2.0)
    assert row["thickness_corners_mm"] == pytest.approx(1.0)


def test_dome_mesh_is_convex_and_closed():
    test_crowned_mesh_is_convex_and_closed(DOME)


def test_dome_refuses_bad_settings():
    with pytest.raises(ValueError):
        pad_mesh(HALF, PadProfile("too_deep", crown_mm=1.5, crown_wid_mm=1.5))
    with pytest.raises(ValueError):
        PadProfile("both", crown_mm=1.0, crown_wid_mm=1.0, across_width=True)
    with pytest.raises(ValueError):
        PadProfile("even", crown_mm=1.0, crown_wid_mm=1.0, dome_grid=(10, 5))
