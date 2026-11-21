# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""A cylinder on an axle, as a small fixed-base articulation (experimental friction tests, Study 3 playground).

``/Roller``: ``base`` (welded to the world) -> ``slide_x`` (prismatic along the world ``x``, held by a stiff drive: it
places the axle along the fingers per environment) -> ``carriage`` -> ``spin`` (revolute about the world ``y``, free:
a torque is applied to it) -> ``roller`` (the cylinder, axis along ``y``, the only collider). The roller's friction
coefficient is set low so that a touching pad's material decides the contact friction (MuJoCo-Warp takes the larger
of the two coefficients).
"""

from __future__ import annotations

from pathlib import Path


def write_roller_usd(path: str | Path, radius_m: float, length_m: float, mass_kg: float, friction: float = 0.01) -> Path:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Roller")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())

    def body(name: str, mass: float, inertia: tuple[float, float, float]):
        xf = UsdGeom.Xform.Define(stage, f"/Roller/{name}")
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
        m.CreateMassAttr(float(mass))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        return xf

    body("base", 0.1, (1e-5, 1e-5, 1e-5))
    body("carriage", 0.05, (1e-5, 1e-5, 1e-5))
    r, h, m = radius_m, length_m, mass_kg
    roller = body("roller", m, (m * (3 * r * r + h * h) / 12.0, 0.5 * m * r * r, m * (3 * r * r + h * h) / 12.0))
    cyl = UsdGeom.Cylinder.Define(stage, "/Roller/roller/shape")
    cyl.CreateRadiusAttr(float(r))
    cyl.CreateHeightAttr(float(h))
    cyl.CreateAxisAttr("Y")
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    material = UsdShade.Material.Define(stage, "/Roller/roller_material")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(float(friction))
    api.CreateDynamicFrictionAttr(float(friction))
    UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")

    fix = UsdPhysics.FixedJoint.Define(stage, "/Roller/fix")
    fix.CreateBody1Rel().SetTargets(["/Roller/base"])
    slide = UsdPhysics.PrismaticJoint.Define(stage, "/Roller/slide_x")
    slide.CreateBody0Rel().SetTargets(["/Roller/base"])
    slide.CreateBody1Rel().SetTargets(["/Roller/carriage"])
    slide.CreateAxisAttr("X")
    slide.CreateLowerLimitAttr(-0.1)
    slide.CreateUpperLimitAttr(0.1)
    spin = UsdPhysics.RevoluteJoint.Define(stage, "/Roller/spin")
    spin.CreateBody0Rel().SetTargets(["/Roller/carriage"])
    spin.CreateBody1Rel().SetTargets(["/Roller/roller"])
    spin.CreateAxisAttr("Y")
    stage.GetRootLayer().Save()
    return out


__all__ = ["write_roller_usd"]
