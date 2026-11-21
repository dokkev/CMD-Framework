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
import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg

from code_isaaclab.assets.builders import write_usd
from code_isaaclab.assets.description import Description
from code_isaaclab.contact_interface import FLAT

from . import HANDS, register_hand
from .code_hand import make_code_hand_spec
from .code_variants import _BUILDER_SOURCES, ASSET_DIR, SHARED, _goal_marker_cfg


def description_asset(description: Path, out_dir: Path = ASSET_DIR) -> Path:
    """The USD of ``description`` with flat convex-mesh pads (written if missing)."""
    digest = hashlib.sha256(Path(description).read_bytes())
    package = Path(__file__).resolve().parents[1]
    for name in _BUILDER_SOURCES:
        digest.update((package / name).read_bytes())
    path = Path(out_dir) / f"{Path(description).stem}_{digest.hexdigest()[:12]}.usda"
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(f".{os.getpid()}.tmp.usda")
        write_usd(Description.load(description), tmp, loop="ball", colliders=True, pad_profile=FLAT)
        os.replace(tmp, path)
    return path


def cylinder_cfg(radius_m: float, length_m: float, mass_kg: float, pos_w: tuple[float, float, float]) -> RigidObjectCfg:
    """A cylinder with its axis along the world ``y`` (across the fingers), at ``pos_w`` (env frame)."""
    return RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/object",
        spawn=sim_utils.CylinderCfg(
            radius=radius_m, height=length_m, axis="Y",
            rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
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
                              servo_saturation: tuple[bool, bool] = (False, True)) -> str:
    """Register ``name`` (idempotent) from ``description`` with ``object_cfg``; the servo effort limit is off by
    default (simulation comparisons, ``code_isaaclab.actuators``)."""
    if name in HANDS:
        return name
    desc = Description.load(description)
    register_hand(make_code_hand_spec(
        desc, description_asset(description), name=name, object_cfg=object_cfg,
        goal_marker_cfg=_goal_marker_cfg(), goal_marker_offset=(0.0, -0.12, 0.0), activate_contact_sensors=True,
        servo_saturation=servo_saturation, **SHARED))
    return name


__all__ = ["bar_cfg", "cylinder_cfg", "description_asset", "register_description_hand"]
