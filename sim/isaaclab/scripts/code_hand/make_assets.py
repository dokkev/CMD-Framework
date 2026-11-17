"""Write the CoDe hand description as URDF (tree + loop markers) and as USD (tree + loop joints).

    uv run python scripts/code_hand/make_assets.py [--description ...] [--out-dir ...] [--loop ball|revolute]

- URDF, lumohand style: the spanning tree only. Each loop cut gets two massless marker links fixed to its two
  bodies at the cut (``<cut>_loop_parent``, ``<cut>_loop_child``); a loader closes the loop between them.
- USD (UsdPhysics): the same tree as one articulation, and each cut as a joint with
  ``physics:excludeFromArticulation = true``, which Newton turns into a loop joint (MuJoCo CONNECT).
  ``--colliders`` makes every capsule and pad box a collider and authors the description's
  ``collision.filtered_pairs`` (``physics:filteredPairs`` on the body prims, which Newton imports).

Link frames sit at each link's parent joint anchor (URDF) or at the description's body origin (USD), with the
hand frame's orientation at the geometric zero.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from description import DEFAULT, Description, rod_inertia  # noqa: E402

OUT = DEFAULT.parent


def _xyz(v) -> str:
    return " ".join(f"{float(x):.9g}" for x in v)


def write_urdf(desc: Description, path: Path) -> Path:
    frame = {"palm": np.zeros(3)}
    for joint in desc.joints:
        frame[joint["child"]] = np.array(joint["anchor"])
    lines = ['<?xml version="1.0"?>', '<robot name="code_hand">',
             f'  <!-- generated from {desc.data["source"]["commit"][:12]}; loops closed by the loader -->']
    for body in desc.bodies:
        com = np.array(body["com"]) - frame[body["name"]]
        i = rod_inertia(body)
        lines += [f'  <link name="{body["name"]}">', "    <inertial>", f'      <origin xyz="{_xyz(com)}"/>',
                  f'      <mass value="{body["mass"]:.9g}"/>',
                  f'      <inertia ixx="{i[0, 0]:.9g}" iyy="{i[1, 1]:.9g}" izz="{i[2, 2]:.9g}"'
                  ' ixy="0" ixz="0" iyz="0"/>',
                  "    </inertial>", "  </link>"]
    for joint in desc.joints:
        origin = np.array(joint["anchor"]) - frame[joint["parent"]]
        lo, hi = (np.radians(v) for v in joint["limits_deg"])
        lines += [f'  <joint name="{joint["name"]}" type="revolute">', f'    <parent link="{joint["parent"]}"/>',
                  f'    <child link="{joint["child"]}"/>', f'    <origin xyz="{_xyz(origin)}" rpy="0 0 0"/>',
                  f'    <axis xyz="{_xyz(joint["axis"])}"/>',
                  f'    <limit lower="{lo:.9g}" upper="{hi:.9g}" effort="1.0" velocity="10.0"/>', "  </joint>"]
    for cut in desc.loops:
        for side in ("parent", "child"):
            marker = f'{cut["name"]}_loop_{side}'
            origin = np.array(cut["anchor"]) - frame[cut[side]]
            lines += [f'  <link name="{marker}"/>', f'  <joint name="{marker}_anchor" type="fixed">',
                      f'    <parent link="{cut[side]}"/>', f'    <child link="{marker}"/>',
                      f'    <origin xyz="{_xyz(origin)}" rpy="0 0 0"/>', "  </joint>"]
    lines.append("</robot>")
    path.write_text("\n".join(lines) + "\n")
    return path


def write_usd(desc: Description, path: Path, loop: str = "ball", ke: float = 5.0, kd: float = 0.05,
              colliders: bool = False) -> Path:
    """``ke``/``kd`` per radian, written in UsdPhysics' per-degree units."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/code_hand")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    origin = {b["name"]: np.array(b["origin"]) for b in desc.bodies}
    for body in desc.bodies:
        xf = UsdGeom.Xform.Define(stage, f"/code_hand/{body['name']}")
        xf.AddTranslateOp().Set(Gf.Vec3d(*origin[body["name"]]))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr(float(body["mass"]))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*(np.array(body["com"]) - origin[body["name"]])))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*np.diag(rod_inertia(body))))
        start = np.array(body["capsule"]["start"]) - origin[body["name"]]
        end = np.array(body["capsule"]["end"]) - origin[body["name"]]
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
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(box["center"]) - origin[body["name"]])))
            cube.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
            cube.AddScaleOp().Set(Gf.Vec3d(*(2.0 * np.array(box["half_extents"]))))
            if colliders:
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if colliders:
        for a, b in desc.data.get("collision", {}).get("filtered_pairs", []):
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"/code_hand/{a}")).CreateFilteredPairsRel(
            ).AddTarget(Sdf.Path(f"/code_hand/{b}"))
    fixed = UsdPhysics.FixedJoint.Define(stage, "/code_hand/joints/palm_fixed")
    fixed.CreateBody1Rel().SetTargets([Sdf.Path("/code_hand/palm")])

    def revolute(name, parent, child, anchor, axis, exclude=False):
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/code_hand/joints/{name}")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(f"/code_hand/{parent}")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(f"/code_hand/{child}")])
        anchor = np.array(anchor)
        # USD revolute joints turn about a principal axis of the joint frame: rotate X onto the axis.
        rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.Vec3d(*axis)).GetQuat()
        q = Gf.Quatf(rot.GetReal(), Gf.Vec3f(*rot.GetImaginary()))
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(anchor - origin[parent])))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*(anchor - origin[child])))
        joint.CreateLocalRot0Attr(q)
        joint.CreateLocalRot1Attr(q)
        joint.CreateAxisAttr("X")
        if exclude:
            joint.CreateExcludeFromArticulationAttr(True)
        return joint

    for j in desc.joints:
        joint = revolute(j["name"], j["parent"], j["child"], j["anchor"], j["axis"])
        joint.CreateLowerLimitAttr(float(j["limits_deg"][0]))
        joint.CreateUpperLimitAttr(float(j["limits_deg"][1]))
        if j["actuator"] is not None:
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
            anchor = np.array(cut["anchor"])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*(anchor - origin[cut["parent"]])))
            joint.CreateLocalPos1Attr(Gf.Vec3f(*(anchor - origin[cut["child"]])))
            joint.CreateExcludeFromArticulationAttr(True)
    stage.GetRootLayer().Save()
    return path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--description", type=Path, default=DEFAULT)
    parser.add_argument("--out-dir", type=Path, default=OUT)
    parser.add_argument("--loop", choices=("ball", "revolute"), default="ball")
    parser.add_argument("--colliders", action="store_true")
    args = parser.parse_args(argv)
    desc = Description.load(args.description)
    print(write_urdf(desc, args.out_dir / f"{args.description.stem}.urdf"))
    stem = args.description.stem
    suffix = "_colliders" if args.colliders else ""
    print(write_usd(desc, args.out_dir / f"{stem}_{args.loop}{suffix}.usda", loop=args.loop, colliders=args.colliders))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
