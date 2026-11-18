# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Asset builders turning a :class:`~code_isaaclab.assets.Description` into URDF, USD, or a Newton ModelBuilder.

- **URDF** (:func:`write_urdf`): spanning tree plus massless loop-marker links (``<cut>_loop_parent`` and
  ``<cut>_loop_child``) for standalone URDF loaders that close cuts after import.
- **USD** (:func:`write_usd`): ``UsdPhysics`` articulation for the spanning tree, plus loop joints marked with
  ``physics:excludeFromArticulation = true`` (converted by Newton into loop constraints / MuJoCo ``CONNECT``).
  Optional ``colliders=True`` authors capsule and pad-box colliders together with ``physics:filteredPairs``.
- **Newton** (:func:`build_newton`): builds a ``newton.ModelBuilder`` directly in memory without intermediate files.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Literal

import numpy as np

from .description import Description, rod_inertia

LoopKind = Literal["ball", "revolute"]


def _xyz(values: Any) -> str:
    return " ".join(f"{float(x):.9g}" for x in values)


def _joint_limits_deg(joint: dict[str, Any], passive_limit_margin_deg: float) -> tuple[float, float]:
    lo, hi = (float(v) for v in joint["limits_deg"])
    if joint.get("actuator") is None and passive_limit_margin_deg != 0.0:
        return lo - passive_limit_margin_deg, hi + passive_limit_margin_deg
    return lo, hi


def write_urdf(desc: Description, path: str | Path, *, passive_limit_margin_deg: float = 0.0) -> Path:
    """Write the spanning tree and loop-cut marker frames as a URDF file."""
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
            "  </link>",
        ]
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


def write_usd(
    desc: Description,
    path: str | Path,
    *,
    loop: LoopKind = "ball",
    ke: float = 5.0,
    kd: float = 0.05,
    colliders: bool = False,
    passive_limit_margin_deg: float = 0.0,
) -> Path:
    """Write the hand as a ``UsdPhysics`` stage (``ke``/``kd`` in N m/rad, authored in per-degree USD units).

    ``passive_limit_margin_deg`` widens passive tree-joint limits so loop closure and passive joint stops do not
    become active simultaneously at the end of the PIP range (important on hard bilateral solvers such as Kamino).
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

        start = np.asarray(body["capsule"]["start"], dtype=float) - origin[body["name"]]
        end = np.asarray(body["capsule"]["end"], dtype=float) - origin[body["name"]]
        length = float(np.linalg.norm(end - start))
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

        for box in body.get("boxes", []):
            cube = UsdGeom.Cube.Define(stage, f"/code_hand/{body['name']}/{box['name']}")
            cube.CreateSizeAttr(1.0)
            w, x, y, z = box["quat_wxyz"]
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(box["center"], dtype=float) - origin[body["name"]])))
            cube.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
            cube.AddScaleOp().Set(Gf.Vec3d(*(2.0 * np.asarray(box["half_extents"], dtype=float))))
            if colliders:
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

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


__all__ = ["LoopKind", "build_newton", "write_urdf", "write_usd"]
