# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Tests for the drawing-only hand geometry (``code_isaaclab.assets.visuals``) and its URDF / USD authoring.

They use the Study 3 hand O description (``design/output/sim/study3/hands``, written by the design export) and are
skipped when it is missing."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from code_isaaclab.assets import Description, write_urdf, write_usd
from code_isaaclab.assets.visuals import (
    PALETTE,
    body_meshes,
    body_poses,
    finger_poses,
    hand_visuals,
    servo_clearances,
    srgb_to_linear,
)
from code_isaaclab.contact_interface import FLAT

HANDS = Path(__file__).resolve().parents[3] / "design" / "output" / "sim" / "study3" / "hands"
HAND_O = HANDS / "cfl_q120_h3to7_h0.json"
pytestmark = pytest.mark.skipif(not HAND_O.exists(), reason="Study 3 hand O description not exported")


@pytest.fixture(scope="module")
def desc() -> Description:
    from code_isaaclab.hands.description_hands import with_thumb_pad

    return with_thumb_pad(Description.load(HAND_O))


def test_every_body_but_markers_gets_visuals_and_every_servo_is_drawn(desc):
    pieces = hand_visuals(desc)
    bodies = {p.body for p in pieces}
    assert bodies == {b["name"] for b in desc.bodies}
    assert {p.servo for p in pieces if p.servo} == {v["servo"] for v in desc.data["visuals"]}
    assert sum(p.material == "pad" for p in pieces) == 7  # six finger pads and the thumb pad
    case = next(p for p in pieces if p.name == "thumb_tip_case")
    assert case.body == "thumb_distal"


def test_body_poses_close_the_four_bars_from_the_tables(desc):
    for angles in finger_poses(desc, [0.0, 60.0, 120.0]):
        poses = body_poses(desc, angles)
        for cut in desc.loops:
            pin = np.asarray(cut["anchor"]) * 1e3
            (ra, ta), (rb, tb) = poses[cut["parent"]], poses[cut["child"]]
            assert np.linalg.norm((ra @ pin + ta) - (rb @ pin + tb)) < 1e-3


def test_plates_clear_their_own_and_the_neighbouring_servos_over_the_flexion_range(desc):
    result = servo_clearances(desc, finger_poses(desc, [0.0, 30.0, 60.0, 90.0, 120.0]))
    fingers = ("index", "middle", "ring", "palm")
    checked = {k: v for k, v in result.items()  # pads are the description's (the proximal pad meets the FE servo)
               if k[0].split("_")[0] in fingers and k[1].split("/")[0] in fingers and not k[0].endswith("_pad_visual")}
    assert any(k[0].split("_")[0] != k[1].split("/")[0] for k in checked)  # neighbours come within range
    assert checked and min(checked.values()) > 0.15  # the link plates sit 0.2 mm off the servo faces


def test_meshes_are_closed_and_wound_outward(desc):
    for (body, colour), (verts, normals, faces) in body_meshes(desc).items():
        assert colour in PALETTE
        assert len(verts) == len(normals) and faces.max() < len(verts)
        tri = verts[faces]
        face_n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        assert np.all((face_n * normals[faces].sum(axis=1)).sum(axis=1) >= -1e-15), body


def test_urdf_visuals_leave_collision_and_inertia_unchanged(desc, tmp_path):
    plain = ET.parse(write_urdf(desc, tmp_path / "a.urdf", geometry=True)).getroot()
    vis = ET.parse(write_urdf(desc, tmp_path / "b.urdf", geometry=True, visuals=True)).getroot()
    for tag in ("inertial", "collision"):
        a = [ET.tostring(e).strip() for link in plain.iter("link") for e in link.iter(tag)]
        b = [ET.tostring(e).strip() for link in vis.iter("link") for e in link.iter(tag)]
        assert a == b and a
    assert [ET.tostring(j).strip() for j in plain.iter("joint")] == [ET.tostring(j).strip() for j in vis.iter("joint")]
    meshes = [m.get("filename") for m in vis.iter("mesh")]
    assert meshes and all(m.endswith(".obj") and (tmp_path / m).exists() for m in meshes)
    assert not any(v.find("geometry/cylinder") is not None for v in vis.iter("visual"))


def test_usd_visuals_add_only_drawing_prims(desc, tmp_path):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    def physics(path):
        stage = Usd.Stage.Open(str(path))
        out = {}
        for prim in stage.Traverse():
            if prim.GetName().startswith("visual_") or str(prim.GetPath()).startswith("/code_hand/Looks"):
                continue
            attrs = {a.GetName(): str(a.Get()) for a in prim.GetAttributes()
                     if a.GetName() not in ("purpose", "primvars:displayColor")}
            out[str(prim.GetPath())] = (str(prim.GetTypeName()), tuple(prim.GetAppliedSchemas()), attrs)
        return stage, out

    _, a = physics(write_usd(desc, tmp_path / "a.usda", colliders=True, pad_profile=FLAT))
    stage, b = physics(write_usd(desc, tmp_path / "b.usda", colliders=True, pad_profile=FLAT, visuals=True))
    assert a == b
    visual = [p for p in stage.Traverse() if p.GetName().startswith("visual_")]
    assert visual
    for prim in visual:
        assert not prim.HasAPI(UsdPhysics.CollisionAPI) and not prim.HasAPI(UsdPhysics.MassAPI)
        colour = prim.GetName().removeprefix("visual_")
        assert np.allclose(UsdGeom.Mesh(prim).GetDisplayColorAttr().Get()[0], srgb_to_linear(PALETTE[colour]))
        assert UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert colliders and all(UsdGeom.Imageable(p).ComputePurpose() == UsdGeom.Tokens.guide for p in colliders)


def test_newton_imports_the_same_colliders_and_the_display_colours(desc, tmp_path):
    newton = pytest.importorskip("newton")
    from newton import ShapeFlags

    def model(path):
        builder = newton.ModelBuilder()
        builder.add_usd(str(path))
        return builder.finalize(device="cpu")

    a = model(write_usd(desc, tmp_path / "a.usda", colliders=True, pad_profile=FLAT))
    b = model(write_usd(desc, tmp_path / "b.usda", colliders=True, pad_profile=FLAT, visuals=True))

    def colliders(m):
        flags = m.shape_flags.numpy()
        keep = (flags & int(ShapeFlags.COLLIDE_SHAPES)) != 0
        return (np.array(m.shape_label)[keep], m.shape_type.numpy()[keep], m.shape_transform.numpy()[keep],
                m.shape_scale.numpy()[keep], m.shape_body.numpy()[keep])

    for x, y in zip(colliders(a), colliders(b), strict=True):
        assert np.array_equal(x, y)
    for name in ("body_mass", "body_com", "body_inertia", "joint_X_p", "joint_X_c", "joint_axis", "joint_limit_lower",
                 "joint_limit_upper"):
        assert np.array_equal(getattr(a, name).numpy(), getattr(b, name).numpy()), name
    colours = {label.rsplit("/", 1)[-1]: c for label, c in zip(b.shape_label, b.shape_color.numpy(), strict=True)
               if "/visual_" in label}
    for name, rgb in PALETTE.items():
        assert np.allclose(colours[f"visual_{name}"], rgb, atol=2e-3)
