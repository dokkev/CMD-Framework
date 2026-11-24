# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Asset builders turning a :class:`~code_isaaclab.assets.Description` into URDF, USD, or a Newton ModelBuilder.

- **URDF** (:func:`write_urdf`): spanning tree plus massless loop-marker links (``<cut>_loop_parent`` and
  ``<cut>_loop_child``) for standalone URDF loaders that close cuts after import (Drake: ``sim/drake/load_hand.py``);
  with ``geometry=True`` also collision, coloured visuals (servo bodies as OBJ) and transmissions; with
  ``visuals=True`` the visuals are instead :func:`~code_isaaclab.assets.visuals.hand_visuals` as OBJ meshes.
- **USD** (:func:`write_usd`): ``UsdPhysics`` articulation for the spanning tree, plus loop joints marked with
  ``physics:excludeFromArticulation = true`` (converted by Newton into loop constraints / MuJoCo ``CONNECT``).
  Optional ``colliders=True`` authors capsule and pad-box colliders together with ``physics:filteredPairs``.
  Optional ``visuals=True`` adds drawing-only meshes (:func:`~code_isaaclab.assets.visuals.hand_visuals`: plates,
  four-bar links, pins, pads and the servo bodies, on the bodies they move with) and hides the colliders from
  viewports (``purpose = guide``); bodies, colliders, masses and joints are unchanged.
- **Newton** (:func:`build_newton`): builds a ``newton.ModelBuilder`` directly in memory without intermediate files.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Literal

import numpy as np

from ..contact_interface import is_pad, pad_mesh
from .description import Description, rod_inertia
from .visuals import PALETTE, body_meshes, rounded_box_mesh, srgb_to_linear

LoopKind = Literal["ball", "revolute"]

SERVO_RGB = (0.22, 0.23, 0.25)
PAD_RGB = (0.30, 0.52, 0.93)
LINK_RGB = (0.78, 0.78, 0.80)
"""Display colours (sRGB) of the ``geometry`` URDF's shape visuals and the USD colliders' ``displayColor``."""
CAP_SEGMENTS = 16
HOUSING_SEGMENTS = 2
"""Steps per quarter circle of a palm housing box's collider hull (56 vertices: within convex-hull vertex limits)."""


def housing_collider(box: dict, origin: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """A palm housing box (``bodies[palm].housing``) as its convex collider mesh in its body's frame [m]: vertices
    and outward triangles."""
    verts, _, faces = rounded_box_mesh(np.asarray(box["center"], float) - origin, box["half_extents"],
                                       float(box["fillet"]), _quat_wxyz_matrix(box["quat_wxyz"]), HOUSING_SEGMENTS)
    return verts, faces


def servo_mesh(shape: dict) -> tuple[np.ndarray, np.ndarray]:
    """A servo body's mesh from its packaging profile (the description's ``servo.shape``), in its actuator frame
    (shaft at the origin along +Z, the body along -Y, centred along Z) [m]: vertices ``(n, 3)`` and triangles.

    ``d_extrusion``: x +-radius, y from -rear_extent to 0, plus a cap of that radius in y >= 0 (``CAP_SEGMENTS``
    chords); ``box_extrusion``: x +-width/2, y from -(length - shaft_from_front) to +shaft_from_front. Both extruded
    over z +-depth/2. The profile is convex, so each end is a triangle fan."""
    if shape["type"] == "d_extrusion":
        r, rear = shape["radius"], shape["rear_extent"]
        cap = [(r * math.cos(t), r * math.sin(t)) for t in np.linspace(0.0, math.pi, CAP_SEGMENTS + 1)]
        profile = [(-r, -rear), (r, -rear)] + cap  # counter-clockwise seen from +Z
    elif shape["type"] == "box_extrusion":
        hw, front = shape["width"] / 2, shape["shaft_from_front"]
        rear = shape["length"] - front
        profile = [(-hw, -rear), (hw, -rear), (hw, front), (-hw, front)]
    else:
        raise ValueError(f"unknown servo shape {shape['type']!r}")
    m, hz = len(profile), shape["depth"] / 2
    verts = np.array([(x, y, z) for z in (-hz, hz) for x, y in profile])
    faces = [(0, k + 1, k) for k in range(1, m - 1)]  # bottom (normal -Z)
    faces += [(m, m + k, m + k + 1) for k in range(1, m - 1)]  # top (+Z)
    for k in range(m):
        a, b = k, (k + 1) % m
        faces += [(a, b, m + b), (a, m + b, m + a)]
    return verts, np.array(faces, dtype=np.int32)


def _xyz(values: Any) -> str:
    return " ".join(f"{float(x):.9g}" for x in values)


def _joint_limits_deg(joint: dict[str, Any], passive_limit_margin_deg: float) -> tuple[float, float]:
    lo, hi = (float(v) for v in joint["limits_deg"])
    if joint.get("actuator") is None and passive_limit_margin_deg != 0.0:
        return lo - passive_limit_margin_deg, hi + passive_limit_margin_deg
    return lo, hi


def _rpy(r: np.ndarray) -> tuple[float, float, float]:
    """URDF roll-pitch-yaw (fixed X, Y, Z: ``R = Rz(yaw) Ry(pitch) Rx(roll)``) of a rotation matrix."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        return math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])
    return math.atan2(-r[1, 2], r[1, 1]), pitch, 0.0


def _quat_wxyz_matrix(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def write_obj(verts: np.ndarray, faces: np.ndarray, path: str | Path, normals: np.ndarray | None = None) -> Path:
    """A triangle mesh as a Wavefront OBJ (vertices as given, metres), with the given per-vertex ``normals`` or else
    a flat normal per face (renderers such as Drake's VTK require normals)."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"v {v[0]:.9g} {v[1]:.9g} {v[2]:.9g}" for v in verts]
    if normals is not None:
        lines += [f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}" for n in normals]
        lines += [f"f {a + 1}//{a + 1} {b + 1}//{b + 1} {c + 1}//{c + 1}" for a, b, c in faces]
        out.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return out
    for a, b, c in faces:
        n = np.cross(verts[b] - verts[a], verts[c] - verts[a])
        n = n / max(np.linalg.norm(n), 1e-12)
        lines.append(f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}")
    lines += [f"f {a + 1}//{k + 1} {b + 1}//{k + 1} {c + 1}//{k + 1}" for k, (a, b, c) in enumerate(faces)]
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return out


def write_urdf(desc: Description, path: str | Path, *, passive_limit_margin_deg: float = 0.0,
               geometry: bool = False, mesh_dir: str | Path | None = None, visuals: bool = False) -> Path:
    """Write the spanning tree and loop-cut marker frames as a URDF file (plain URDF: the loops are not closed in it;
    a loader closes them from the description's ``loops``, the cut pins marked by fixed-joint frames).

    ``geometry`` also writes, per link, its collision geometry as in the USD (each capsule as a cylinder with a sphere
    at each end, URDF having no capsule; each pad box as a box), its visuals in the USD's display colours (the same
    shapes, and every ``visuals`` entry: the servo bodies extruded from ``servo.shape``, as OBJ meshes in
    ``mesh_dir``, default ``<urdf stem>_meshes`` beside the URDF), and a ``SimpleTransmission`` for every actuated
    joint (``EffortJointInterface``, reduction 1). Pads are boxes (the flat profile).

    ``visuals`` replaces those visuals (or adds visuals to a plain URDF) with
    :func:`~code_isaaclab.assets.visuals.hand_visuals`, merged per body and colour into OBJ meshes in ``mesh_dir``
    (with smoothed normals); collision, inertia and joints are as without it."""
    out = Path(path)
    frame: dict[str, np.ndarray] = {"palm": np.zeros(3)}
    for joint in desc.joints:
        frame[joint["child"]] = np.asarray(joint["anchor"], dtype=float)
    commit = desc.source.get("commit", "unknown")[:12]
    lines = [
        '<?xml version="1.0"?>',
        '<robot name="code_hand">',
        f"  <!-- generated from {commit}; loops closed by the loader -->",
    ]
    servo_obj = None
    mesh_root = Path(mesh_dir) if mesh_dir is not None else out.with_name(f"{out.stem}_meshes")
    visual_objs: dict[str, list[tuple[str, Path]]] = {}
    if visuals:
        lines += [f'  <material name="vis_{name}"><color rgba="{r} {g} {b} 1"/></material>'
                  for name, (r, g, b) in PALETTE.items()]
        for (body_name, material), (verts, normals, faces) in body_meshes(desc).items():
            path_obj = write_obj(verts, faces, mesh_root / f"{body_name}_{material}.obj", normals)
            visual_objs.setdefault(body_name, []).append((material, path_obj))
    if geometry:
        lines += [f'  <material name="{name}"><color rgba="{r} {g} {b} 1"/></material>'
                  for name, (r, g, b) in (("link", LINK_RGB), ("pad", PAD_RGB), ("servo", SERVO_RGB))]
        if desc.data.get("visuals") and not visuals:
            servo_obj = write_obj(*servo_mesh(desc.servo["shape"]),
                                  mesh_root / f"servo_{desc.servo['shape']['type']}.obj")

    def shapes(body: dict) -> list[tuple[str, str, str]]:  # (geometry xml, origin xml, material) in the link frame
        found = []
        for box in body.get("housing", []):  # the palm housing replaces the placeholder capsule
            obj = write_obj(*housing_collider(box, frame[body["name"]]), mesh_root / f"{box['name']}.obj")
            rel = obj.relative_to(out.parent) if obj.is_relative_to(out.parent) else obj
            found.append((f'<mesh filename="{rel}"/>', 'xyz="0 0 0" rpy="0 0 0"', "link"))
        start = np.asarray(body["capsule"]["start"], dtype=float) - frame[body["name"]]
        end = np.asarray(body["capsule"]["end"], dtype=float) - frame[body["name"]]
        radius, length = float(body["capsule"]["radius"]), float(np.linalg.norm(end - start))
        if body.get("housing"):
            length, start, end = -1.0, None, None
        if length > 1e-6:
            direction = (end - start) / length
            axis = np.cross([0.0, 0.0, 1.0], direction)
            angle = math.acos(max(-1.0, min(1.0, direction[2])))
            rot = np.eye(3) if np.linalg.norm(axis) < 1e-9 and direction[2] > 0 else (
                np.diag([1.0, -1.0, -1.0]) if np.linalg.norm(axis) < 1e-9 else _axis_angle_matrix(axis, angle))
            found.append((f'<cylinder radius="{radius:.9g}" length="{length:.9g}"/>',
                          f'xyz="{_xyz((start + end) / 2)}" rpy="{_xyz(_rpy(rot))}"', "link"))
        for point in (start, end) if length > 1e-6 else (start,) if start is not None else ():
            found.append((f'<sphere radius="{radius:.9g}"/>', f'xyz="{_xyz(point)}" rpy="0 0 0"', "link"))
        for box in body.get("boxes", []):
            rot = _quat_wxyz_matrix(box["quat_wxyz"])
            found.append((f'<box size="{_xyz(2.0 * np.asarray(box["half_extents"], dtype=float))}"/>',
                          f'xyz="{_xyz(np.asarray(box["center"], dtype=float) - frame[body["name"]])}" '
                          f'rpy="{_xyz(_rpy(rot))}"', "pad" if is_pad(box) else "link"))
        return found

    for body in desc.bodies:
        com = np.asarray(body["com"], dtype=float) - frame[body["name"]]
        inertia = rod_inertia(body)
        lines += [
            f'  <link name="{body["name"]}">',
            "    <inertial>",
            f'      <origin xyz="{_xyz(com)}"/>',
            f'      <mass value="{body["mass"]:.9g}"/>',
            f'      <inertia ixx="{inertia[0, 0]:.9g}" iyy="{inertia[1, 1]:.9g}" izz="{inertia[2, 2]:.9g}"'
            ' ixy="0" ixz="0" iyz="0"/>',
            "    </inertial>",
        ]
        for material, path_obj in visual_objs.get(body["name"], []):
            rel = path_obj.relative_to(out.parent) if path_obj.is_relative_to(out.parent) else path_obj
            lines.append(f'    <visual name="{body["name"]}_{material}"><origin xyz="0 0 0" rpy="0 0 0"/><geometry>'
                         f'<mesh filename="{rel}"/></geometry><material name="vis_{material}"/></visual>')
        if geometry:
            for k, (geom, origin, material) in enumerate(shapes(body)):
                if not visuals:
                    lines.append(f'    <visual name="{body["name"]}_v{k}"><origin {origin}/><geometry>{geom}'
                                 f'</geometry><material name="{material}"/></visual>')
                lines.append(f'    <collision name="{body["name"]}_c{k}"><origin {origin}/><geometry>{geom}'
                             "</geometry></collision>")
            for item in (desc.data.get("visuals") or []) if servo_obj is not None else []:
                if item["body"] != body["name"]:
                    continue
                rot = _quat_wxyz_matrix(item["quat_wxyz"])
                rel = (Path(servo_obj).relative_to(out.parent) if Path(servo_obj).is_relative_to(out.parent)
                       else servo_obj)
                lines.append(f'    <visual name="{item["name"]}"><origin xyz="'
                             f'{_xyz(np.asarray(item["position"], dtype=float) - frame[body["name"]])}" '
                             f'rpy="{_xyz(_rpy(rot))}"/><geometry><mesh filename="{rel}"/></geometry>'
                             '<material name="servo"/></visual>')
        lines.append("  </link>")
    for joint in desc.joints:
        origin = np.asarray(joint["anchor"], dtype=float) - frame[joint["parent"]]
        lo_deg, hi_deg = _joint_limits_deg(joint, passive_limit_margin_deg)
        lo, hi = math.radians(lo_deg), math.radians(hi_deg)
        lines += [
            f'  <joint name="{joint["name"]}" type="revolute">',
            f'    <parent link="{joint["parent"]}"/>',
            f'    <child link="{joint["child"]}"/>',
            f'      <origin xyz="{_xyz(origin)}" rpy="0 0 0"/>'.replace("      <origin", "    <origin"),
            f'    <axis xyz="{_xyz(joint["axis"])}"/>',
            f'    <limit lower="{lo:.9g}" upper="{hi:.9g}" effort="1.0" velocity="10.0"/>',
            "  </joint>",
        ]
        if geometry and joint.get("actuator") is not None:
            lines += [
                f'  <transmission name="{joint["name"]}_transmission">',
                "    <type>transmission_interface/SimpleTransmission</type>",
                f'    <joint name="{joint["name"]}"><hardwareInterface>hardware_interface/EffortJointInterface'
                "</hardwareInterface></joint>",
                f'    <actuator name="{joint["name"]}_motor"><mechanicalReduction>1</mechanicalReduction></actuator>',
                "  </transmission>",
            ]
    for cut in desc.loops:
        for side in ("parent", "child"):
            marker = f"{cut['name']}_loop_{side}"
            origin = np.asarray(cut["anchor"], dtype=float) - frame[cut[side]]
            lines += [
                f'  <link name="{marker}"/>',
                f'  <joint name="{marker}_anchor" type="fixed">',
                f'    <parent link="{cut[side]}"/>',
                f'    <child link="{marker}"/>',
                f'    <origin xyz="{_xyz(origin)}" rpy="0 0 0"/>',
                "  </joint>",
            ]
    lines.append("</robot>")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return out


def _axis_angle_matrix(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def write_usd(
    desc: Description,
    path: str | Path,
    *,
    loop: LoopKind = "ball",
    ke: float = 5.0,
    kd: float = 0.05,
    colliders: bool = False,
    passive_limit_margin_deg: float = 0.0,
    pad_profile=None,
    visuals: bool = False,
) -> Path:
    """Write the hand as a ``UsdPhysics`` stage (``ke``/``kd`` in N m/rad, authored in per-degree USD units).

    ``pad_profile`` (a :class:`~code_isaaclab.contact_interface.PadProfile`) authors every pad (a box named
    ``*_pad``) as a convex mesh of that contact-face profile instead of a box primitive, with the same mounting
    face, length and width (convex-hull collision approximation). Other boxes stay boxes.

    ``passive_limit_margin_deg`` widens passive tree-joint limits so loop closure and passive joint stops do not
    become active simultaneously at the end of the PIP range (important on hard bilateral solvers such as Kamino).

    ``visuals`` adds, for rendering only, :func:`~code_isaaclab.assets.visuals.body_meshes` as one ``Mesh`` per body
    and colour (``<body>/visual_<colour>``: no collision, mass or physics schema; ``displayColor`` and a bound
    ``UsdPreviewSurface``, both linear, which Newton's importer turns back into the display colour), and gives the
    colliders ``purpose = guide`` so viewports draw the visuals instead (Newton keeps them as colliders and hides
    them unless ``show_collision``). Without it the stage is exactly as before.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    if loop not in ("ball", "revolute"):
        raise ValueError(f"loop must be 'ball' or 'revolute', got {loop!r}")

    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/code_hand")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    origin = {b["name"]: np.asarray(b["origin"], dtype=float) for b in desc.bodies}

    for body in desc.bodies:
        xf = UsdGeom.Xform.Define(stage, f"/code_hand/{body['name']}")
        xf.AddTranslateOp().Set(Gf.Vec3d(*origin[body["name"]]))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr(float(body["mass"]))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*(np.asarray(body["com"], dtype=float) - origin[body["name"]])))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*np.diag(rod_inertia(body))))

        for box in body.get("housing", []):  # the palm housing replaces the placeholder capsule: convex hulls
            verts, faces = housing_collider(box, origin[body["name"]])
            hull = UsdGeom.Mesh.Define(stage, f"/code_hand/{body['name']}/{box['name']}")
            hull.CreatePointsAttr([Gf.Vec3f(*v) for v in verts])
            hull.CreateFaceVertexCountsAttr([3] * len(faces))
            hull.CreateFaceVertexIndicesAttr(faces.reshape(-1).tolist())
            hull.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            if colliders:
                UsdPhysics.CollisionAPI.Apply(hull.GetPrim())
                UsdPhysics.MeshCollisionAPI.Apply(hull.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
            if visuals:
                hull.CreateDisplayColorAttr([Gf.Vec3f(*srgb_to_linear(LINK_RGB))])
                hull.CreatePurposeAttr(UsdGeom.Tokens.guide)

        start = np.asarray(body["capsule"]["start"], dtype=float) - origin[body["name"]]
        end = np.asarray(body["capsule"]["end"], dtype=float) - origin[body["name"]]
        length = -1.0 if body.get("housing") else float(np.linalg.norm(end - start))
        if length > 1e-6:
            cap = UsdGeom.Capsule.Define(stage, f"/code_hand/{body['name']}/capsule")
            cap.CreateRadiusAttr(float(body["capsule"]["radius"]))
            cap.CreateHeightAttr(length)
            cap.CreateAxisAttr("Z")
            direction = (end - start) / length
            rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*direction)).GetQuat()
            cap.AddTranslateOp().Set(Gf.Vec3d(*((start + end) / 2)))
            cap.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(rot)
            if colliders:
                UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
            if visuals:
                cap.CreateDisplayColorAttr([Gf.Vec3f(*srgb_to_linear(LINK_RGB))])
                cap.CreatePurposeAttr(UsdGeom.Tokens.guide)

        for box in body.get("boxes", []):
            if pad_profile is not None and is_pad(box):
                surface = pad_profile.for_pad(box["name"])
                points, counts, indices = pad_mesh(box["half_extents"], surface)
                mesh = UsdGeom.Mesh.Define(stage, f"/code_hand/{body['name']}/{box['name']}")
                mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
                mesh.CreateFaceVertexCountsAttr(counts)
                mesh.CreateFaceVertexIndicesAttr(indices)
                mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
                w, x, y, z = box["quat_wxyz"]
                mesh.AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(box["center"], dtype=float) - origin[body["name"]])))
                mesh.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
                mesh.GetPrim().SetCustomDataByKey("code:pad_profile", surface.name)
                mesh.GetPrim().SetCustomDataByKey("code:crown_mm", float(surface.crown_mm))
                if visuals:
                    mesh.CreateDisplayColorAttr([Gf.Vec3f(*srgb_to_linear(PAD_RGB))])
                    mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)
                if colliders:
                    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
                    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
                        UsdPhysics.Tokens.convexHull
                    )
                continue
            cube = UsdGeom.Cube.Define(stage, f"/code_hand/{body['name']}/{box['name']}")
            cube.CreateSizeAttr(1.0)
            w, x, y, z = box["quat_wxyz"]
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(box["center"], dtype=float) - origin[body["name"]])))
            cube.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
            cube.AddScaleOp().Set(Gf.Vec3d(*(2.0 * np.asarray(box["half_extents"], dtype=float))))
            if colliders:
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            if visuals:
                cube.CreateDisplayColorAttr([Gf.Vec3f(*srgb_to_linear(PAD_RGB if is_pad(box) else LINK_RGB))])
                cube.CreatePurposeAttr(UsdGeom.Tokens.guide)

    if visuals:  # drawing only: plain Mesh prims, no CollisionAPI, no MassAPI
        from pxr import UsdShade

        looks = {}
        for name, rgb in PALETTE.items():
            material = UsdShade.Material.Define(stage, f"/code_hand/Looks/{name}")
            shader = UsdShade.Shader.Define(stage, f"/code_hand/Looks/{name}/surface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*srgb_to_linear(rgb)))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            looks[name] = material
        for (body_name, colour), (verts, normals, faces) in body_meshes(desc).items():
            mesh = UsdGeom.Mesh.Define(stage, f"/code_hand/{body_name}/visual_{colour}")
            mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in verts])
            mesh.CreateNormalsAttr([Gf.Vec3f(*n) for n in normals])
            mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
            mesh.CreateFaceVertexCountsAttr([3] * len(faces))
            mesh.CreateFaceVertexIndicesAttr(faces.reshape(-1).tolist())
            mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            mesh.CreateDisplayColorAttr([Gf.Vec3f(*srgb_to_linear(PALETTE[colour]))])
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(looks[colour])
            mesh.GetPrim().SetCustomDataByKey("code:visual", colour)

    if colliders:
        for a, b in desc.filtered_pairs:
            rel = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"/code_hand/{a}")).CreateFilteredPairsRel()
            rel.AddTarget(Sdf.Path(f"/code_hand/{b}"))

    fixed = UsdPhysics.FixedJoint.Define(stage, "/code_hand/joints/palm_fixed")
    fixed.CreateBody1Rel().SetTargets([Sdf.Path("/code_hand/palm")])

    def revolute(name: str, parent: str, child: str, anchor: list[float], axis: list[float], exclude: bool = False):
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/code_hand/joints/{name}")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(f"/code_hand/{parent}")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(f"/code_hand/{child}")])
        anchor_vec = np.asarray(anchor, dtype=float)
        # USD revolute joints rotate about a principal axis of the joint frame: rotate X onto the joint axis.
        rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.Vec3d(*axis)).GetQuat()
        quat = Gf.Quatf(rot.GetReal(), Gf.Vec3f(*rot.GetImaginary()))
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(anchor_vec - origin[parent])))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*(anchor_vec - origin[child])))
        joint.CreateLocalRot0Attr(quat)
        joint.CreateLocalRot1Attr(quat)
        joint.CreateAxisAttr("X")
        if exclude:
            joint.CreateExcludeFromArticulationAttr(True)
        return joint

    for j in desc.joints:
        joint = revolute(j["name"], j["parent"], j["child"], j["anchor"], j["axis"])
        lo_deg, hi_deg = _joint_limits_deg(j, passive_limit_margin_deg)
        joint.CreateLowerLimitAttr(lo_deg)
        joint.CreateUpperLimitAttr(hi_deg)
        if j.get("actuator") is not None:
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(float(ke * np.pi / 180.0))
            drive.CreateDampingAttr(float(kd * np.pi / 180.0))
            drive.CreateTargetPositionAttr(0.0)

    for cut in desc.loops:
        if loop == "revolute":
            revolute(cut["name"], cut["parent"], cut["child"], cut["anchor"], cut["axis"], exclude=True)
        else:
            joint = UsdPhysics.SphericalJoint.Define(stage, f"/code_hand/joints/{cut['name']}")
            joint.CreateBody0Rel().SetTargets([Sdf.Path(f"/code_hand/{cut['parent']}")])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(f"/code_hand/{cut['child']}")])
            anchor_vec = np.asarray(cut["anchor"], dtype=float)
            joint.CreateLocalPos0Attr(Gf.Vec3f(*(anchor_vec - origin[cut["parent"]])))
            joint.CreateLocalPos1Attr(Gf.Vec3f(*(anchor_vec - origin[cut["child"]])))
            joint.CreateExcludeFromArticulationAttr(True)

    stage.GetRootLayer().Save()
    return out


def build_newton(
    desc: Description,
    *,
    loop: Literal["ball", "revolute", "none"] = "ball",
    ke: float = 5.0,
    kd: float = 0.05,
    armature: float = 1e-5,
    digits: tuple[str, ...] | None = None,
    passive_limit_margin_deg: float = 0.0,
):
    """Build a standalone ``newton.ModelBuilder`` from ``desc`` (tree articulation + non-tree loop cuts)."""
    import newton
    import warp as wp

    if loop not in ("ball", "revolute", "none"):
        raise ValueError(f"loop must be 'ball', 'revolute', or 'none', got {loop!r}")

    keep = (lambda name: True) if digits is None else (lambda name: name == "palm" or name.split("_")[0] in digits)
    builder = newton.ModelBuilder()
    index: dict[str, int] = {}
    origin: dict[str, np.ndarray] = {}
    for body in desc.bodies:
        if not keep(body["name"]):
            continue
        origin[body["name"]] = np.asarray(body["origin"], dtype=float)
        com = np.asarray(body["com"], dtype=float) - origin[body["name"]]
        index[body["name"]] = builder.add_link(
            xform=wp.transform(wp.vec3(*origin[body["name"]]), wp.quat_identity()),
            com=wp.vec3(*com),
            inertia=wp.mat33(rod_inertia(body)),
            mass=float(body["mass"]),
            label=body["name"],
        )

    def xforms(joint: dict[str, Any]):
        anchor = np.asarray(joint["anchor"], dtype=float)
        return (
            wp.transform(wp.vec3(*(anchor - origin[joint["parent"]])), wp.quat_identity()),
            wp.transform(wp.vec3(*(anchor - origin[joint["child"]])), wp.quat_identity()),
        )

    tree = [builder.add_joint_fixed(-1, index["palm"], label="palm_fixed")]
    for joint in desc.joints:
        if not keep(joint["child"]):
            continue
        parent_xform, child_xform = xforms(joint)
        lo_deg, hi_deg = _joint_limits_deg(joint, passive_limit_margin_deg)
        lower, upper = math.radians(lo_deg), math.radians(hi_deg)
        driven = joint.get("actuator") is not None
        tree.append(
            builder.add_joint_revolute(
                index[joint["parent"]],
                index[joint["child"]],
                parent_xform=parent_xform,
                child_xform=child_xform,
                axis=wp.vec3(*joint["axis"]),
                target_ke=ke if driven else 0.0,
                target_kd=kd if driven else 0.0,
                limit_lower=lower,
                limit_upper=upper,
                armature=armature,
                label=joint["name"],
            )
        )
    builder.add_articulation(tree, label="code_hand")

    loop_indices: list[int] = []
    for cut in desc.loops:
        if loop == "none" or not keep(cut["child"]):
            continue
        parent_xform, child_xform = xforms(cut)
        if loop == "ball":
            j = builder.add_joint_ball(
                index[cut["parent"]],
                index[cut["child"]],
                parent_xform=parent_xform,
                child_xform=child_xform,
                label=cut["name"],
            )
        else:
            j = builder.add_joint_revolute(
                index[cut["parent"]],
                index[cut["child"]],
                parent_xform=parent_xform,
                child_xform=child_xform,
                axis=wp.vec3(*cut["axis"]),
                target_ke=0.0,
                target_kd=0.0,
                label=cut["name"],
            )
        builder.joint_articulation[j] = -1
        loop_indices.append(j)
    return builder, index, loop_indices


__all__ = ["LoopKind", "build_newton", "servo_mesh", "write_obj", "write_urdf", "write_usd"]
