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
    assert {p.servo for p in pieces if getattr(p, "servo", None)} == {v["servo"] for v in desc.data["visuals"]}
    assert sum(p.material == "pad" for p in pieces) == 7  # six finger pads and the thumb pad
    case = next(p for p in pieces if p.name == "thumb_tip_case")
    assert case.body == "thumb_distal"
    palm = [p for p in pieces if p.body == "palm" and getattr(p, "servo", None) is None]
    assert {p.name for p in palm} >= {"palm_hood", "palm_block", "palm_bridge", "palm_heel"}  # the housing
    assert not any(p.name.startswith("palm_plate") for p in palm)  # in place of the plain palm plates


def test_profiles_are_simple_and_fully_triangulated(desc):
    from shapely.geometry import Polygon

    from code_isaaclab.assets.visuals import ear_clip

    for piece in (p for p in hand_visuals(desc) if hasattr(p, "profile")):
        poly = np.asarray(piece.profile)
        if not Polygon(poly).exterior.is_ccw:
            poly = poly[::-1]
        assert Polygon(poly).is_valid, piece.name
        assert len(ear_clip(poly)) == len(poly) - 2, piece.name


def test_link_plates_end_round_about_their_pins(desc):
    from shapely.geometry import Point, Polygon

    from code_isaaclab.assets.visuals import COUPLER_HALF_WIDTH_MM, DRIVER_HALF_WIDTH_MM, finger_plane

    pieces = {p.name: p for p in hand_visuals(desc)}
    pl = finger_plane(desc, "index")
    pin = {name: pl.xz(np.asarray(desc.joint(f"index_{name}")["anchor"]) * 1e3) for name in ("a", "pip_servo", "b")}
    pin["bp"] = pl.xz(np.asarray(desc.loops[0]["anchor"]) * 1e3)
    for link, ends, half in (("index_driver", ("pip_servo", "b"), DRIVER_HALF_WIDTH_MM),
                             ("index_coupler", ("a", "bp"), COUPLER_HALF_WIDTH_MM)):
        ring = Polygon(pieces[link].profile).exterior
        for end in ends:  # a semicircle of the half width about the pin, and nothing nearer to it
            arc = np.array([c for c in np.array(ring.coords) if abs(np.linalg.norm(c - pin[end]) - half) < 0.02])
            angles = np.unwrap(np.sort(np.arctan2(*(arc - pin[end]).T[::-1])))
            gaps = np.diff(np.concatenate([angles, angles[:1] + 2 * np.pi]))
            assert len(arc) >= 16 and 2 * np.pi - gaps.max() > np.radians(175), (link, end)
            assert abs(ring.distance(Point(pin[end])) - half) < 0.02


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
    plain = ET.parse(write_urdf(desc, tmp_path / "a.urdf", geometry=True, mesh_dir=tmp_path / "m")).getroot()
    vis = ET.parse(write_urdf(desc, tmp_path / "b.urdf", geometry=True, visuals=True,
                              mesh_dir=tmp_path / "m")).getroot()
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


def test_housing_replaces_the_palm_capsule_as_convex_colliders(desc, tmp_path):
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(write_usd(desc, tmp_path / "h.usda", colliders=True, pad_profile=FLAT)))
    palm = stage.GetPrimAtPath("/code_hand/palm")
    children = {c.GetName(): c for c in palm.GetChildren()}
    assert "capsule" not in children
    boxes = [b["name"] for b in desc.body("palm")["housing"]]
    for name in boxes:
        prim = children[name]
        assert prim.HasAPI(UsdPhysics.CollisionAPI)
        assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == UsdPhysics.Tokens.convexHull
        assert len(UsdGeom.Mesh(prim).GetPointsAttr().Get()) <= 64  # within convex-hull vertex limits
    urdf = ET.parse(write_urdf(desc, tmp_path / "h.urdf", geometry=True)).getroot()
    link = next(link for link in urdf.iter("link") if link.get("name") == "palm")
    meshes = [c.find("geometry/mesh").get("filename") for c in link.iter("collision")]
    assert len(meshes) == len(boxes) and all((tmp_path / m).exists() for m in meshes)
    assert not any(c.find("geometry/sphere") is not None for c in link.iter("collision"))


def test_housing_geometry_follows_its_parameters(desc):
    from code_isaaclab.assets.visuals import housing_boxes

    params = desc.data["palm_housing"]
    boxes = {b.name: b for b in housing_boxes(desc)}
    lo = {n: b.center - b.half for n, b in boxes.items()}
    hi = {n: b.center + b.half for n, b in boxes.items()}
    fe_x = min(np.asarray(desc.joint(f"{d}_fe")["anchor"])[0] * 1e3 for d in ("index", "middle", "ring"))
    assert max(h[0] for h in hi.values()) == pytest.approx(fe_x - params["front_setback_mm"], abs=1e-6)
    aa = [v for v in desc.data["visuals"] if v["servo"].endswith("/aa")]
    aa_z = min(v["position"][2] for v in aa) * 1e3 - desc.servo["shape"]["depth"] / 2 * 1e3
    assert lo["palm_block"][2] == pytest.approx(aa_z - params["palmar_depth_mm"], abs=0.2)  # the flat palmar face
    assert all(b.fillet == pytest.approx(params["fillet_mm"]) for b in boxes.values())


def test_housing_clears_the_fingers_over_their_range(desc):
    from code_isaaclab.assets.visuals import housing_clearances

    poses = []
    for aa, fe, q in ((-30.0, -10.0, 0.0), (30.0, 100.0, 120.0), (30.0, -10.0, 60.0), (-30.0, 100.0, 0.0)):
        pose = {}
        for d in desc.four_bar_digits():
            pose.update(desc.closed_pose(d, q))
            pose[f"{d}_aa"], pose[f"{d}_fe"] = aa, fe
        poses.append(pose)
    result = housing_clearances(desc, poses)
    visual = {k: v for k, v in result.items() if k[1] == "visual" and not k[0].startswith("thumb")}
    assert visual and min(v[0] for v in visual.values()) > 0.3
    # the placeholder MCP capsules start on the abduction axes, inside the housed servos (the pairs are filtered)
    colliders = {k: v for k, v in result.items() if k[1] == "collider" and not k[0].startswith("thumb")}
    assert all(v[0] > 0.3 for k, v in colliders.items() if not k[0].endswith("_mcp"))
