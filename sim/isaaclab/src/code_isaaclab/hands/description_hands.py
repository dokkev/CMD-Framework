# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Register a hand from any ``code_hand_sim/1`` description file, with a chosen object (Study 3).

:mod:`code_isaaclab.hands.code_variants` builds the pilot's two hands from one fixed export. Study 3 compares
other fingers (redesigns, perturbations), so this module takes the description path: the USD is written next to
the pilot's assets under a name carrying a hash of the description and the builder sources (an existing asset is
reused, never overwritten), with flat pads as convex meshes, and registered with the pilot's shared settings. The
object is passed in; :func:`cylinder_cfg` makes the Study 3 cylinder lying across the fingers.
"""

from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg

from code_isaaclab.assets.builders import write_usd
from code_isaaclab.assets.description import Description
from code_isaaclab.contact_interface import FLAT, PadProfile

from . import HANDS, register_hand
from .code_hand import make_code_hand_spec
from .code_variants import _BUILDER_SOURCES, ASSET_DIR, SHARED, _goal_marker_cfg


def bind_pad_friction(usd_path: Path, friction: float) -> int:
    """Give every pad collider (``*_pad``) of the stage a physics material with static and dynamic friction
    ``friction``; returns the number of pads bound. MuJoCo-Warp takes the larger of two touching shapes'
    coefficients and Newton's default is 1.0, so the pads' material sets the contact friction only against objects
    whose own coefficient is lower."""
    from pxr import Usd, UsdPhysics, UsdShade

    stage = Usd.Stage.Open(str(usd_path))
    material = UsdShade.Material.Define(stage, "/code_hand/pad_material")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(float(friction))
    api.CreateDynamicFrictionAttr(float(friction))
    api.CreateRestitutionAttr(0.0)
    count = 0
    for prim in stage.Traverse():
        if prim.GetName().endswith("_pad") and prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
            count += 1
    stage.GetRootLayer().Save()
    return count


def bind_pad_hydroelastic(usd_path: Path, stiffness: float, sdf_max_resolution: int = 64) -> int:
    """Make every pad collider (``*_pad``) a hydroelastic triangle mesh with stiffness ``stiffness`` [N/m^3] for
    Newton's SDF hydroelastic contact (``physics=newton_mjwarp_hydro``); returns the number of pads.

    A convex-hull approximation is replaced by non-mesh geometry on import and cannot keep an SDF, so the pads are
    authored as plain meshes (approximation ``none``) with ``NewtonSDFCollisionAPI``, ``newton:hydroelasticEnabled``,
    ``newton:hydroelasticStiffness`` and ``newton:sdfMaxResolution`` (read by Newton's USD importer). Other colliders
    keep the preset's default hydroelastic shape settings."""
    from pxr import Sdf, Usd, UsdPhysics

    stage = Usd.Stage.Open(str(usd_path))
    count = 0
    for prim in stage.Traverse():
        if prim.GetName().endswith("_pad") and prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.none)
            prim.AddAppliedSchema("NewtonSDFCollisionAPI")
            prim.CreateAttribute("newton:hydroelasticEnabled", Sdf.ValueTypeNames.Bool).Set(True)
            prim.CreateAttribute("newton:hydroelasticStiffness", Sdf.ValueTypeNames.Float).Set(float(stiffness))
            prim.CreateAttribute("newton:sdfMaxResolution", Sdf.ValueTypeNames.Int).Set(int(sdf_max_resolution))
            count += 1
    stage.GetRootLayer().Save()
    return count


THUMB_PAD_MM = {"thickness_mm": 3.0, "mount_offset_mm": 10.0, "width_mm": 18.0, "length_mm": 25.0}
"""The thumb pad: on the fingertip case only (the CAD's thumb distal segment is the IP servo's X330 body for its first
~26 mm from the IP axis, then a rounded fingertip case out to the end effector at 53.5 mm, ``ee_y_mm``; the CAD has no
thumb pad), its last ``length_mm`` before the end effector. Flat, 3 mm thick like the finger pads (no material of its
own: the same contact as theirs), 18 mm wide (the case is about the X330's 20 mm). Its mounting face sits ``mount_offset_mm``
from the thumb's centre line, the X330's half width (the export's thumb capsule is a 6 mm placeholder, thinner than the
case). Fixed for every hand (R, U change the fingers only)."""


def thumb_pad_box(desc: Description) -> dict:
    """The thumb pad (:data:`THUMB_PAD_MM`) as a ``bodies[].boxes`` entry of ``thumb_distal`` (hand frame at the geometric
    zero; local ``x`` along the segment, local ``+y`` the contact face, as :mod:`code_isaaclab.contact_interface`
    expects). The segment runs from the IP axis to the end effector (the distal capsule's ends); the pad side is the
    IP servo frame's +X (``design/core/morphology/code_hand/parameters.py``: the thumb flexes toward its pad), the side
    the IP flexion moves the segment toward, ``axis x segment``."""
    pads = THUMB_PAD_MM
    cap = desc.body("thumb_distal")["capsule"]
    start, end = np.asarray(cap["start"], float), np.asarray(cap["end"], float)
    seg = (end - start) / np.linalg.norm(end - start)
    face = np.cross(np.asarray(desc.joint("thumb_ip")["axis"], float), seg)
    face /= np.linalg.norm(face)
    across = np.cross(seg, face)
    t, mount, width, length = (pads[k] * 1e-3 for k in ("thickness_mm", "mount_offset_mm", "width_mm", "length_mm"))
    centre = end - seg * (length / 2) + face * (mount + 0.5 * t)
    rot = np.stack((seg, face, across), axis=1)  # columns: local x, y, z in the hand frame
    w = 0.5 * math.sqrt(max(0.0, 1.0 + np.trace(rot)))
    if w > 1e-6:
        quat = (w, (rot[2, 1] - rot[1, 2]) / (4 * w), (rot[0, 2] - rot[2, 0]) / (4 * w), (rot[1, 0] - rot[0, 1]) / (4 * w))
    else:  # a half turn: axis from the diagonal
        axis = np.sqrt(np.clip((np.diag(rot) + 1.0) / 2.0, 0.0, None))
        axis[1] *= 1.0 if rot[0, 1] + rot[1, 0] >= 0 else -1.0
        axis[2] *= 1.0 if rot[0, 2] + rot[2, 0] >= 0 else -1.0
        quat = (0.0, *axis)
    return {"name": "thumb_distal_pad", "center": [float(v) for v in centre], "quat_wxyz": [float(v) for v in quat],
            "half_extents": [length / 2, t / 2, width / 2]}


def with_thumb_pad(desc: Description) -> Description:
    """A copy of ``desc`` with :func:`thumb_pad_box` on ``thumb_distal`` (the export itself is unchanged)."""
    import copy

    data = copy.deepcopy(desc.data)
    body = next(b for b in data["bodies"] if b["name"] == "thumb_distal")
    body.setdefault("boxes", []).append(thumb_pad_box(desc))
    return Description(data)


def description_asset(description: Path, out_dir: Path = ASSET_DIR, profile: PadProfile = FLAT,
                      pad_friction: float | None = None, pad_hydroelastic: float | None = None,
                      thumb_pad: bool = False, visuals: bool = False) -> Path:
    """The USD of ``description`` with ``profile`` convex-mesh pads (written if missing); ``pad_friction``: the pads'
    friction coefficient (None: the backend default); ``pad_hydroelastic``: the pads' hydroelastic stiffness
    [N/m^3] (None: point-contact pads, :func:`bind_pad_hydroelastic`); ``thumb_pad``: add :func:`thumb_pad_box`;
    ``visuals``: add the description's drawing-only meshes and display colours (rendering only: the same bodies,
    colliders, masses and joints; ``write_usd``)."""
    digest = hashlib.sha256(Path(description).read_bytes())
    if thumb_pad:
        digest.update(repr(("thumb_pad", thumb_pad_box(Description.load(description)))).encode())
        digest.update(Path(__file__).read_bytes())
    digest.update(repr(sorted(profile.__dict__.items())).encode())
    digest.update(repr(pad_friction).encode())
    if pad_hydroelastic is not None:
        digest.update(repr(("hydroelastic", pad_hydroelastic)).encode())
    package = Path(__file__).resolve().parents[1]
    for name in _BUILDER_SOURCES:
        digest.update((package / name).read_bytes())
    tag = "" if pad_friction is None else f"_mu{pad_friction:g}"
    if pad_hydroelastic is not None:
        tag += f"_kh{pad_hydroelastic:.3g}"
    if thumb_pad:
        tag += "_thumbpad"
    if visuals:  # the geometry comes from the description (hashed above) and assets/visuals.py
        tag += "_vis"
        digest.update(b"visuals")
        digest.update((package / "assets" / "visuals.py").read_bytes())
    path = Path(out_dir) / f"{Path(description).stem}_{profile.name}{tag}_{digest.hexdigest()[:12]}.usda"
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(f".{os.getpid()}.tmp.usda")
        desc = Description.load(description)
        write_usd(with_thumb_pad(desc) if thumb_pad else desc, tmp, loop="ball", colliders=True, pad_profile=profile,
                  visuals=visuals)
        pads = 6 + int(thumb_pad)
        if pad_friction is not None and bind_pad_friction(tmp, pad_friction) != pads:
            raise RuntimeError(f"expected {pads} pad colliders in {tmp}")
        if pad_hydroelastic is not None and bind_pad_hydroelastic(tmp, pad_hydroelastic) != pads:
            raise RuntimeError(f"expected {pads} pad colliders in {tmp}")
        os.replace(tmp, path)
    return path


def cylinder_cfg(radius_m: float, length_m: float, mass_kg: float, pos_w: tuple[float, float, float],
                 kinematic: bool = False) -> RigidObjectCfg:
    """A cylinder with its axis along the world ``y`` (across the fingers), at ``pos_w`` (env frame); ``kinematic``:
    held in place (moved only by writing its pose)."""
    return RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/object",
        spawn=sim_utils.CylinderCfg(
            radius=radius_m, height=length_m, axis="Y",
            rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=kinematic or None),
            mass_props=sim_utils.MassCfg(mass=mass_kg),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            activate_contact_sensors=True,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.6, 0.8)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=pos_w, rot=(0.0, 0.0, 0.0, 1.0)),
    )


def bar_cfg(size_m: tuple[float, float, float], mass_kg: float, pos_w: tuple[float, float, float],
            friction: float | None = None) -> RigidObjectCfg:
    """A rectangular bar (world ``x``, ``y``, ``z`` extents), at ``pos_w`` (env frame); a bar does not roll on flat
    pads, a cylinder does (neutrally stable on a horizontal face). ``friction``: its static and dynamic friction
    coefficient (None: the backend's default)."""
    return RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/object",
        spawn=sim_utils.CuboidCfg(
            size=size_m, rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(), mass_props=sim_utils.MassCfg(mass=mass_kg),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(), activate_contact_sensors=True,
            physics_material=None if friction is None else sim_utils.RigidBodyMaterialCfg(
                static_friction=friction, dynamic_friction=friction),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.6, 0.8)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=pos_w, rot=(0.0, 0.0, 0.0, 1.0)),
    )


def register_description_hand(name: str, description: Path, object_cfg: RigidObjectCfg,
                              servo_saturation: tuple[bool, bool] = (False, True), profile: PadProfile = FLAT,
                              pad_friction: float | None = None, pad_hydroelastic: float | None = None,
                              thumb_pad: bool = False, visuals: bool = False) -> str:
    """Register ``name`` (idempotent) from ``description`` with ``object_cfg``; the servo effort limit is off by
    default (simulation comparisons, ``code_isaaclab.actuators``)."""
    if name in HANDS:
        return name
    desc = Description.load(description)
    register_hand(make_code_hand_spec(
        desc, description_asset(description, profile=profile, pad_friction=pad_friction,
                                                    pad_hydroelastic=pad_hydroelastic, thumb_pad=thumb_pad, visuals=visuals), name=name,
        object_cfg=object_cfg,
        goal_marker_cfg=_goal_marker_cfg(), goal_marker_offset=(0.0, -0.12, 0.0), activate_contact_sensors=True,
        servo_saturation=servo_saturation, **SHARED))
    return name


__all__ = ["THUMB_PAD_MM", "thumb_pad_box", "with_thumb_pad", "bar_cfg", "bind_pad_friction", "bind_pad_hydroelastic", "cylinder_cfg", "description_asset",
           "register_description_hand"]
