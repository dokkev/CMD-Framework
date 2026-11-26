# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The relay dial as a small articulation USD (no Isaac Lab imports): a base held to the world by a fixed joint, a slider on a prismatic joint
along the dial's axis (local z; its spring comes from the actuator config), and a disk on a revolute joint about the
same axis (continuous; its torsional spring and damping from the actuator config). The disk's hand-facing face lies in
the root frame's z = 0 plane when the slider is at 0; the disk extends toward +z."""

from __future__ import annotations

from pathlib import Path


def write_dial_usd(path: Path, radius_m: float, thick_m: float, mass_kg: float, slide_m: float) -> Path:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    path = Path(path)
    if path.exists():
        return path
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp.usda")
    stage = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/dial")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())

    def body(name: str, mass: float):
        x = UsdGeom.Xform.Define(stage, f"/dial/{name}")
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(mass)
        return x

    body("base", 0.01)
    body("slider", 0.005)
    disk = body("disk", mass_kg)
    geom = UsdGeom.Cylinder.Define(stage, "/dial/disk/collider")
    geom.CreateRadiusAttr(radius_m)
    geom.CreateHeightAttr(thick_m)
    geom.CreateAxisAttr("Z")
    geom.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, thick_m / 2))
    UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
    mark = UsdGeom.Cube.Define(stage, "/dial/disk/marker")  # a visual radius mark (no collision)
    mark.CreateSizeAttr(1.0)
    mark.AddTranslateOp().Set(Gf.Vec3d(radius_m * 0.6, 0.0, thick_m + 0.0005))
    mark.AddScaleOp().Set(Gf.Vec3f(radius_m * 0.8, 0.003, 0.001))
    del disk

    def joint(kind, name, b0, b1):
        j = kind.Define(stage, f"/dial/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"/dial/{b0}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"/dial/{b1}")])
        j.CreateAxisAttr("Z")
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0))
        return j

    fix = UsdPhysics.FixedJoint.Define(stage, "/dial/fix_joint")  # the base held to the world
    fix.CreateBody1Rel().SetTargets([Sdf.Path("/dial/base")])
    fix.CreateLocalPos0Attr(Gf.Vec3f(0.0))
    fix.CreateLocalPos1Attr(Gf.Vec3f(0.0))
    slide = joint(UsdPhysics.PrismaticJoint, "slide_joint", "base", "slider")
    slide.CreateLowerLimitAttr(0.0)
    slide.CreateUpperLimitAttr(slide_m)
    UsdPhysics.DriveAPI.Apply(slide.GetPrim(), "linear")
    spin = joint(UsdPhysics.RevoluteJoint, "spin_joint", "slider", "disk")
    UsdPhysics.DriveAPI.Apply(spin.GetPrim(), "angular")
    stage.GetRootLayer().Save()
    tmp.rename(path)
    return path
