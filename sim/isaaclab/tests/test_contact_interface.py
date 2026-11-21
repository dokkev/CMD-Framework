# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3's contact-interface variants: their geometry, and that only the pads' surface differs between them."""

from __future__ import annotations

import math

import numpy as np
import pytest

from code_isaaclab.contact_interface import CROWNED, FLAT, PadProfile, contact_face_height, pad_mesh, pad_profile_polygon

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


def test_crowned_mesh_is_convex_and_closed():
    points, counts, indices = pad_mesh(HALF, CROWNED)
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
        for a, b in zip(ring, ring[1:] + ring[:1]):
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
