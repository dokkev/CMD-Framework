# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The CoDe hand with interchangeable contact interfaces (Study 3): one articulation, several pad surfaces.

Every variant is built from the same ``code_hand_sim/1`` export and differs only in the pads' contact face
(:mod:`code_isaaclab.contact_interface`). The USD of a variant is written once per content (export bytes,
profile parameters, builder source) under :data:`ASSET_DIR` and reused afterwards; its name carries the content
key, so a changed input never overwrites an earlier asset. :func:`variant_manifest` records what each asset
was made from.

The hands registered here (``code_flat``, ``code_crowned``) share every other setting: the servo model, the
root pose (palm up), the initial joint pose, the in-hand object and the goal tolerance (:data:`SHARED`).
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.markers import VisualizationMarkersCfg

from code_isaaclab.assets.builders import write_usd
from code_isaaclab.assets.description import Description
from code_isaaclab.contact_interface import VARIANTS, PadProfile, surface_report

from . import HANDS, HandSpec, register_hand
from .code_hand import make_code_hand_spec

SIM_ROOT = Path(__file__).resolve().parents[3]
"""``sim/isaaclab``."""
REPO_ROOT = SIM_ROOT.parents[1]
STUDY3_DESCRIPTION = Path(
    os.environ.get("CODE_STUDY3_DESCRIPTION", REPO_ROOT / "design" / "output" / "sim" / "study3" / "code_hand_pivot8.json")
)
"""The export every Study 3 variant is built from (``design/scripts/export/code_hand_sim.py --params
core/morphology/code_hand/pivot8_balanced.yaml --out output/sim/study3/code_hand_pivot8.json``)."""
ASSET_DIR = Path(os.environ.get("CODE_STUDY3_ASSETS", SIM_ROOT / "logs" / "study3_pilot" / "assets"))
_BUILDER_SOURCES = ("assets/builders.py", "contact_interface.py")

CUBE_SIZE_M = 0.05
CUBE_MASS_KG = 0.05
HAND_ROOT_POS = (0.0, 0.0, 0.5)
HAND_ROOT_ROT_XYZW = (1.0, 0.0, 0.0, 0.0)
"""Half a turn about the hand's ``x`` (fingers): the palmar side (hand ``-z``) faces up."""
PAD_FACE_DEPTH_M = 0.0128
"""Palmar pad faces below the hand frame's origin in the export (pad centre 11.3 mm + half thickness 1.5 mm)."""
CUBE_REST_POS = (0.088, 0.025, HAND_ROOT_POS[2] + PAD_FACE_DEPTH_M + CUBE_SIZE_M / 2)
"""The cube's centre resting on the extended fingers' pads: midway along the pads, over the middle finger."""
CUBE_DROP_M = 0.004
"""Spawn this far above the resting height (it settles onto the pads)."""


def content_key(description: Path, profile: PadProfile) -> str:
    """Hash of everything an asset is made from: the export's bytes, the profile and the builder sources."""
    digest = hashlib.sha256(Path(description).read_bytes())
    digest.update(json.dumps(profile.__dict__, sort_keys=True).encode())
    package = Path(__file__).resolve().parents[1]
    for name in _BUILDER_SOURCES:
        digest.update((package / name).read_bytes())
    return digest.hexdigest()[:12]


def variant_asset(variant: str, description: Path = STUDY3_DESCRIPTION, out_dir: Path = ASSET_DIR) -> Path:
    """The USD of the hand with the ``variant`` pads (written if it does not exist yet)."""
    profile = VARIANTS[variant]
    if not Path(description).exists():
        raise FileNotFoundError(
            f"{description} is missing: export it with design/scripts/export/code_hand_sim.py "
            "--params core/morphology/code_hand/pivot8_balanced.yaml --out output/sim/study3/code_hand_pivot8.json"
        )
    key = content_key(description, profile)
    path = Path(out_dir) / f"code_hand_pivot8_{variant}_{key}.usda"
    if not path.exists():
        tmp = path.with_suffix(f".{os.getpid()}.tmp.usda")
        write_usd(Description.load(description), tmp, loop="ball", colliders=True, pad_profile=profile)
        os.replace(tmp, path)
    return path


PAD_COLLISION = ("convex_hull", "mesh")
"""How the pad colliders are authored: ``convex_hull`` (the pilot: ``physics:approximation = convexHull``) or
``mesh`` (Study 3A main: the same triangle mesh with no approximation). The pads are convex prisms, so the hull
is the mesh itself and MuJoCo-Warp's point contact sees the same geometry either way; only ``mesh`` lets
Newton's hydroelastic contact build an SDF of the pad (Study 3B), which it refuses on an approximated shape."""


def mesh_collision_asset(variant: str, description: Path = STUDY3_DESCRIPTION, out_dir: Path = ASSET_DIR) -> Path:
    """The ``variant`` asset with every pad collider's mesh approximation removed (written if missing).

    Derived from the convex-hull asset by a stage edit, nothing else: its name carries a hash of that asset and of
    this module, so it changes when either does and never overwrites an earlier one."""
    from pxr import Usd, UsdPhysics

    base = variant_asset(variant, description, out_dir)
    digest = hashlib.sha256(base.read_bytes())
    digest.update(Path(__file__).read_bytes())
    path = Path(out_dir) / f"{base.stem}_meshcol_{digest.hexdigest()[:12]}.usda"
    if not path.exists():
        stage = Usd.Stage.Open(str(base))
        edited = 0
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                prim.RemoveProperty("physics:approximation")
                prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
                edited += 1
        if edited != 6:
            raise RuntimeError(f"expected 6 pad colliders with a mesh approximation in {base}, found {edited}")
        tmp = path.with_suffix(f".{os.getpid()}.tmp.usda")
        stage.GetRootLayer().Export(str(tmp))
        os.replace(tmp, path)
    return path


def variant_manifest(
    variant: str, description: Path = STUDY3_DESCRIPTION, out_dir: Path = ASSET_DIR, pad_collision: str = "convex_hull"
) -> dict:
    """What the variant's asset was made from, and the asset's own hash."""
    path = (mesh_collision_asset if pad_collision == "mesh" else variant_asset)(variant, description, out_dir)
    desc = Description.load(description)
    profile = VARIANTS[variant]
    return {
        "variant": variant,
        "pad_collision": pad_collision,
        "profile": dict(profile.__dict__),
        "usd": str(path),
        "usd_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "description": str(description),
        "description_sha256": hashlib.sha256(Path(description).read_bytes()).hexdigest(),
        "description_source": desc.source,
        "pads": surface_report(desc.bodies, profile),
    }


def _cube_cfg() -> RigidObjectCfg:
    return RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/object",
        spawn=sim_utils.CuboidCfg(
            size=(CUBE_SIZE_M,) * 3,
            rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
            mass_props=sim_utils.MassCfg(mass=CUBE_MASS_KG),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            activate_contact_sensors=True,  # read by the Study 3 scripts' contact sensor; no effect on dynamics
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.5, 0.2)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(CUBE_REST_POS[0], CUBE_REST_POS[1], CUBE_REST_POS[2] + CUBE_DROP_M), rot=(0.0, 0.0, 0.0, 1.0)
        ),
    )


def _goal_marker_cfg() -> VisualizationMarkersCfg:
    return VisualizationMarkersCfg(
        prim_path="/Visuals/goal_marker",
        markers={"goal": sim_utils.CuboidCfg(size=(CUBE_SIZE_M,) * 3, visual_material=sim_utils.PreviewSurfaceCfg())},
    )


SERVO_MODE = os.environ.get("CODE_STUDY3_SERVO", "implicit")
"""The XM335 model every variant uses: ``implicit`` (the PD law's linearisation, integrated by the solver) for the
whole pilot, because the explicit ``dc_motor`` law blows the closed loops apart on VBD (loop residual of metres,
Gate 0) and Study 3B must replay a policy under the same actuator on both solvers. Its limitation (E6): large
moves run faster than the motor's no-load speed. ``CODE_STUDY3_SERVO=dc_motor`` selects the explicit law."""

MJWARP_BUFFERS = {"njmax": 1024, "nconmax": 256}
"""MuJoCo-Warp capacity per environment. The general CoDe default (300 / 120) overflowed during training
("broadphase overflow ... increase nconmax to 131"): a curled hand with inter-finger contact exceeds 120
contacts, and an overflow drops contacts. Same for every variant."""

SHARED = {
    "servo_mode": SERVO_MODE,
    "mjwarp_buffers": MJWARP_BUFFERS,
    "init_pos": HAND_ROOT_POS,
    "init_rot": HAND_ROOT_ROT_XYZW,
    "goal_orientation_threshold": 0.2,
    "decimation": 4,
}
"""Settings every variant shares (the export sets the rest: joints, limits, servo constants, masses)."""


def register_code_variants(
    description: Path = STUDY3_DESCRIPTION, out_dir: Path = ASSET_DIR, servo_mode: str | None = None
) -> list[str]:
    """Register ``code_<variant>`` (convex-hull pads, the pilot) and ``code_<variant>_main`` (mesh pads, Study 3A
    main) for every variant (idempotent); return their names."""
    desc = Description.load(description)
    names = []
    for variant, pad_collision in ((v, c) for v in VARIANTS for c in PAD_COLLISION):
        name = f"code_{variant}" + ("_main" if pad_collision == "mesh" else "")
        names.append(name)
        if name in HANDS:
            continue
        settings = dict(SHARED)
        if servo_mode is not None:
            settings["servo_mode"] = servo_mode
        spec: HandSpec = make_code_hand_spec(
            desc,
            (mesh_collision_asset if pad_collision == "mesh" else variant_asset)(variant, description, out_dir),
            name=name,
            object_cfg=_cube_cfg(),
            goal_marker_cfg=_goal_marker_cfg(),
            goal_marker_offset=(0.0, -0.12, 0.0),
            activate_contact_sensors=True,
            **settings,
        )
        register_hand(spec)
    return names


__all__ = [
    "ASSET_DIR",
    "CUBE_MASS_KG",
    "CUBE_REST_POS",
    "CUBE_SIZE_M",
    "SHARED",
    "STUDY3_DESCRIPTION",
    "PAD_COLLISION",
    "content_key",
    "mesh_collision_asset",
    "register_code_variants",
    "variant_asset",
    "variant_manifest",
]
