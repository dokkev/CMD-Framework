"""What of each contact face is lost at a pose, by rule and by what hides it.

Two rules (``rule``), each with a depth:

- ``"segment"``: the fold. A point of a face is lost behind the other pad's
  face, opposite it (``packaging.fold_free_lengths``, the v4 objective's
  rule); the depth is not used.
- ``"probe"``: a point is lost when any solid of the finger but its own pad
  lies within ``depth_mm`` in front of it, along its outward normal
  (``packaging.probe_blocked``), whatever the solid's Y layer.

A :class:`Scene` holds a design's two faces and every other solid at a set
of poses (distal flexions ``phi``, MCP at 0, in the proximal frame), the
solids sorted into :data:`CATEGORIES`, so the loss can be told by what
causes it. :func:`scene` builds one for any mechanism of the comparison.
"""

from __future__ import annotations

import importlib
import math
from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np

from core.morphology.linkage import FingerLinkage
from core.morphology.linkage.packaging import (
    contact_faces as linkage_faces,
    distal_carrier_polygon,
    distal_pad_polygon,
    fold_free_lengths,
    probe_blocked,
    proximal_pad_polygon,
)

CATEGORIES = ("other_pad", "carrier", "link", "pin", "servo", "pulley", "roller")
"""What may stand in front of a face: the other pad, the distal pad's carrier,
links and bars (the linkage's bars, a 2R's distal truss, the RCJ's carrier,
a belt's strands count as ``pulley``), pin housings, servos (the joint's and
the MCP's), a belt's pulleys, an RCJ's rollers."""
RULES = ("segment", "probe")
SIDES = ("proximal_pad", "distal_pad")


@dataclass
class Shapes:
    """Solids at every pose: convex polygons ``(n, m, 2)`` (or one ``(m, 2)``
    fixed), bars ``(n, 2, 2)`` and disks ``((n, 2) or (2,) centres, radius)``."""

    polygons: list = field(default_factory=list)
    segments: list = field(default_factory=list)
    disks: list = field(default_factory=list)


@dataclass
class Scene:
    """One design at ``count`` poses: each face ``(n, 2, 2)`` (outer end
    first) with its outward normal ``(n, 2)``, and what stands near it, by
    category, for each face (its own pad left out)."""

    phi_deg: np.ndarray
    faces: dict[str, tuple[np.ndarray, np.ndarray]]
    solids: dict[str, dict[str, Shapes]]


@dataclass
class FaceLoss:
    """A face's loss at each pose: its length, the length lost, and the
    length each category alone would hide (``probe``; ``segment``: the other pad)."""

    length: np.ndarray
    lost: np.ndarray
    by_category: dict[str, np.ndarray]

    @property
    def free(self) -> np.ndarray:
        return self.length - self.lost


def face_loss(scene: Scene, rule: str, depth_mm: float = 0.0) -> dict[str, FaceLoss]:
    """Each face's loss at every pose of ``scene`` under ``rule``."""

    if rule not in RULES:
        raise ValueError(f"rule must be one of {RULES}, not {rule!r}")
    result = {}
    if rule == "segment":
        (pf, pn), (df, dn) = scene.faces["proximal_pad"], scene.faces["distal_pad"]
        free = fold_free_lengths(pf, pn, df, dn)
        for side in SIDES:
            face = scene.faces[side][0]
            length = np.linalg.norm(face[:, 1] - face[:, 0], axis=1)
            lost = length - free[side]
            result[side] = FaceLoss(length, lost, {name: (lost if name == "other_pad" else np.zeros_like(lost))
                                                  for name in CATEGORIES})
        return result
    for side in SIDES:
        face, normal = scene.faces[side]
        masks = {}
        length = np.linalg.norm(face[:, 1] - face[:, 0], axis=1)
        for name in CATEGORIES:
            shapes = scene.solids[side][name]
            length, masks[name] = probe_blocked(face, normal, depth_mm, shapes.polygons, shapes.segments,
                                                shapes.disks)
        union = np.logical_or.reduce(list(masks.values()))
        result[side] = FaceLoss(length, length * union.mean(axis=1),
                                {name: length * mask.mean(axis=1) for name, mask in masks.items()})
    return result


def access_points(scene: Scene, side: str, depth_mm: float, step_mm: float = 0.5):
    """The probed points of ``side``'s face ``(n, k, 2)`` and which of them an
    object ``depth_mm`` deep cannot reach ``(n, k)`` (every category)."""

    face, normal = scene.faces[side]
    shapes = Shapes()
    for shape in scene.solids[side].values():
        shapes.polygons += shape.polygons
        shapes.segments += shape.segments
        shapes.disks += shape.disks
    _, blocked = probe_blocked(face, normal, depth_mm, shapes.polygons, shapes.segments, shapes.disks, step_mm)
    share = (np.arange(blocked.shape[1]) + 0.5) / blocked.shape[1]
    points = face[:, :1] + share[None, :, None] * (face[:, 1:] - face[:, :1])
    return points, blocked


def intrusion_mm(scene: Scene, side: str, categories: Sequence[str], depth_max_mm: float = 30.0,
                 step_mm: float = 0.25) -> np.ndarray:
    """How close (mm, along the normal) solids of ``categories`` come in front
    of ``side``'s face at each pose: the least probe depth that meets one,
    NaN when none within ``depth_max_mm``."""

    face, normal = scene.faces[side]
    shapes = Shapes()
    for name in categories:
        shape = scene.solids[side][name]
        shapes.polygons += shape.polygons
        shapes.segments += shape.segments
        shapes.disks += shape.disks
    found = np.full(len(face), np.nan)
    for depth in np.arange(step_mm, depth_max_mm + 1e-9, step_mm):
        _, blocked = probe_blocked(face, normal, depth, shapes.polygons, shapes.segments, shapes.disks)
        newly = np.isnan(found) & blocked.any(axis=1)
        found[newly] = depth
        if not np.isnan(found).any():
            break
    return found


def _bars(*pairs) -> list[np.ndarray]:
    return [np.stack(np.broadcast_arrays(*(np.atleast_2d(np.asarray(point, dtype=float)) for point in pair)), axis=1)
            for pair in pairs]


def _scene(phi_deg, faces, own: dict[str, list], others: dict[str, Shapes]) -> Scene:
    """The scene with each face's other pad from ``own`` (the pads by side) and the shared solids."""

    solids = {}
    for side, other in (("proximal_pad", "distal_pad"), ("distal_pad", "proximal_pad")):
        solids[side] = {name: Shapes(list(shape.polygons), list(shape.segments), list(shape.disks))
                        for name, shape in others.items()}
        solids[side]["other_pad"] = Shapes([own[other]])
    return Scene(np.asarray(phi_deg, dtype=float), faces, solids)


def scene(mechanism: str, config, x: Sequence[float], phi_deg: Sequence[float]) -> Scene:
    """The scene of a design ``x`` of ``mechanism`` (the comparison's) at
    distal flexions ``phi_deg`` (the linkage: the first sample of its servo
    sweep reaching each)."""

    phi_deg = np.atleast_1d(np.asarray(phi_deg, dtype=float))
    if mechanism == "linkage":
        return _linkage_scene(config, x, phi_deg)
    return _joint_scene(mechanism, config, x, phi_deg)


def _linkage_scene(config, x, phi_deg) -> Scene:
    from core.optimization.linkage.design import decode

    packaging = config.packaging
    linkage = FingerLinkage(decode(x, config.link_length_resolution_mm))
    step = math.radians(0.05)
    sense = 1.0 if linkage.sweep_servo([0.0, step]).q_pip_rad[-1] >= 0.0 else -1.0
    sweep = linkage.sweep_servo(sense * np.arange(0.0, math.pi, step))
    reached = np.maximum.accumulate(sweep.total_flexion_rad)
    index = np.minimum(np.searchsorted(reached, np.radians(phi_deg) - 1e-9), len(reached) - 1)
    b, c, bp = sweep.b[index], sweep.c[index], sweep.bp[index]
    p = linkage.parameters
    o, a, ap = (0.0, 0.0), (p.a_x_mm, p.a_z_mm), (p.ap.x_mm, p.ap.z_mm)
    proximal_face, proximal_normal, distal_face, distal_normal = linkage_faces(linkage, packaging, b, c)
    count = len(index)
    faces = {"proximal_pad": (np.broadcast_to(proximal_face, (count, 2, 2)),
                              np.broadcast_to(proximal_normal, (count, 2))),
             "distal_pad": (distal_face, distal_normal)}
    radius = packaging.pivot_radius_mm
    others = {
        "carrier": Shapes([distal_carrier_polygon(linkage, packaging, b, c)] if packaging.distal_carrier_height_mm > 0
                          else []),
        "link": Shapes(segments=_bars((o, a), (a, ap), (ap, o), (a, bp), (ap, b), (bp, b), (b, c))),
        "pin": Shapes(disks=[(a, radius("a")), (ap, radius("ap")), (b, radius("b")), (bp, radius("bp"))]),
        "servo": Shapes([packaging.pip_footprint(linkage).outline, packaging.mcp_footprint().outline]),
        "pulley": Shapes(),
        "roller": Shapes(),
    }
    own = {"proximal_pad": proximal_pad_polygon(linkage, packaging), "distal_pad": distal_pad_polygon(linkage, packaging, b, c)}
    return _scene(np.degrees(sweep.total_flexion_rad[index]), faces, own, others)


def _joint_scene(mechanism: str, config, x, phi_deg) -> Scene:
    from core.optimization.revolute.evaluate import contact_faces

    module = importlib.import_module(
        f"core.optimization.{'revolute' if mechanism == 'revolute_idealized' else mechanism}.evaluate")
    geo = module.geometry(x, config)
    joint = getattr(geo, "joint", geo)  # the belt 2R's 2R
    packaging = config.goal.packaging
    phi = np.radians(phi_deg)
    count = len(phi)
    proximal_face, proximal_normal, distal_face, distal_normal = contact_faces(joint, phi)
    faces = {"proximal_pad": (np.broadcast_to(proximal_face, (count, 2, 2)),
                              np.broadcast_to(proximal_normal, (count, 2))),
             "distal_pad": (distal_face, distal_normal)}
    near, far = np.moveaxis(joint.posed(joint.distal_mount, phi), 1, 0)
    carrier = [] if joint.distal_carrier_local is None else [joint.posed(joint.distal_carrier_local, phi)]
    mcp = packaging.mcp_footprint().outline
    if mechanism == "rcj":
        a, b = geo.a, geo.b(phi)
        others = {"link": Shapes(segments=_bars(((0.0, 0.0), a), (a, b), (b, near), (b, far), (near, far))),
                  "pin": Shapes(), "servo": Shapes([geo.servo.outline, mcp]), "pulley": Shapes(),
                  "roller": Shapes(disks=[(a, geo.roller_radius_mm), (b, geo.roller_radius_mm)])}
    else:
        a = np.array((joint.a_x_mm, 0.0))
        others = {"link": Shapes(segments=_bars(((0.0, 0.0), a), (a, near), (a, far), (near, far))),
                  "pin": Shapes(disks=[(a, packaging.pivot_radius_mm("a"))]),
                  "servo": Shapes([mcp] + ([geo.distal_servo.outline] if mechanism == "revolute" else [])
                                  + ([geo.servo.outline] if mechanism == "revolute_parallel" else [])),
                  "pulley": Shapes(), "roller": Shapes()}
        if mechanism == "revolute_parallel":  # the belt and its pulleys, in the outer layer
            radius = geo.pulley_radius_mm
            along = (a - geo.ap) / np.linalg.norm(a - geo.ap)
            side = radius * np.array((-along[1], along[0]))
            others["pulley"] = Shapes(segments=_bars((geo.ap + side, a + side), (geo.ap - side, a - side)),
                                      disks=[(a, radius), (geo.ap, radius)])
    others["carrier"] = Shapes(carrier)
    own = {"proximal_pad": joint.proximal_pad, "distal_pad": joint.distal_pad(phi)}
    return _scene(phi_deg, faces, own, others)


__all__ = ["CATEGORIES", "FaceLoss", "RULES", "Scene", "Shapes", "access_points", "face_loss", "intrusion_mm", "scene"]
