"""Pad Coverage Formulation in Material Coordinates (SSoT Section 7 / INSTRUCTION Section 5).

Distinguishes five physical quantities that must never be conflated:

1. **Geometrically supported room** ``S_i(M)``: available mounting span on segment ``i``.
2. **Mounted contact length** ``l_i``: physical axial length of the mounted pad block.
3. **Accessible contact-face set** ``U_i(M, phi; eta_L) <= [0, l_i]`` in material coordinate
   ``xi_i in [0, l_i]`` (from outer end ``xi_i = 0`` to joint-facing inner end ``xi_i = l_i``),
   supporting disconnected intervals, with effective usable length
   ``L_i^eff(M, phi; eta_L) = mu(U_i(M, phi; eta_L))``.
4. **Endpoint Coverage** ``C_endpoint(M; eta_L)`` evaluated at ``phi = phi_min^des``.
5. **Motion-Robust Coverage** ``C_robust(M; eta_L)`` from material-coordinate set intersection
   ``U_i^rob(M; eta_L) = intersection_{phi in [0, phi_min^des]} U_i(M, phi; eta_L)``.

Linkage occlusion (``include_linkage_occlusion``, indicator ``eta_L in {0, 1}``) is an
independently controlled option of the coverage evaluator. Context-specific defaults
are resolved explicitly through configuration profiles:

- ``"study"`` profile default: ``include_linkage_occlusion = False``
- ``"prototype"`` profile default: ``include_linkage_occlusion = True``
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Literal

import numpy as np

from core._validation import require_finite, require_nonnegative
from core.morphology.canonical import (
    CoDeMechanismParams,
    FingerMorphology,
    MorphologyError,
    RevoluteMechanismParams,
)
from core.morphology.intervals import (
    IntervalSet,
    base_accessible_sets,
    disk_occluded_interval,
    face_occluded_set,
    segment_occluded_interval,
)
from core.morphology.mechanism.linkage.model import KinematicClosureError

CoverageProfile = Literal["study", "prototype"]
PROFILE_DEFAULT_LINKAGE_OCCLUSION: Mapping[str, bool] = MappingProxyType(
    {
        "study": False,
        "prototype": True,
    }
)


def resolve_linkage_occlusion(
    include_linkage_occlusion: bool | None,
    *,
    profile: CoverageProfile,
) -> bool:
    """Resolve ``include_linkage_occlusion`` explicitly from ``profile`` when omitted."""

    if profile not in PROFILE_DEFAULT_LINKAGE_OCCLUSION:
        raise ValueError(
            f"profile must be one of {tuple(PROFILE_DEFAULT_LINKAGE_OCCLUSION)}, got {profile!r}"
        )
    if include_linkage_occlusion is None:
        return PROFILE_DEFAULT_LINKAGE_OCCLUSION[profile]
    if not isinstance(include_linkage_occlusion, bool):
        raise ValueError("include_linkage_occlusion must be a boolean")
    return include_linkage_occlusion


def _linkage_occluders_for_morphology(
    morphology: FingerMorphology, phi_rad: float
) -> dict[str, tuple[list[tuple[np.ndarray, np.ndarray]], list[tuple[np.ndarray, float]]]]:
    """Transmission members capable of occluding each pad face at ``phi_rad``."""

    empty: tuple[list[tuple[np.ndarray, np.ndarray]], list[tuple[np.ndarray, float]]] = ([], [])
    if morphology.architecture == "CoDe":
        assert isinstance(morphology.mech, CoDeMechanismParams)
        pose = morphology.mech.four_bar.solve(phi_rad)
        a = np.array(morphology.mech.t_a.pos, dtype=float)
        ap = np.array(morphology.mech.t_ap.pos, dtype=float)
        b = pose.b.array
        bp = pose.bp.array
        bars = [(a, bp), (ap, b)]
        return {"proximal_pad": (bars, []), "distal_pad": (bars, [])}
    if morphology.architecture == "2R-remote":
        assert isinstance(morphology.mech, RevoluteMechanismParams)
        r_pulley = morphology.act.pulley_radius_mm
        if r_pulley is None or r_pulley <= 0.0:
            return {"proximal_pad": empty, "distal_pad": empty}
        a = np.array(morphology.mech.t_a.pos, dtype=float)
        ap = np.array(morphology.act.pip.pose.pos, dtype=float)
        diff = a - ap
        dist = float(np.linalg.norm(diff))
        strands: list[tuple[np.ndarray, np.ndarray]] = []
        if dist > 1e-9:
            along = diff / dist
            side = r_pulley * np.array((-along[1], along[0]), dtype=float)
            strands = [(ap + side, a + side), (ap - side, a - side)]
        p_disks = [(a, r_pulley), (ap, r_pulley)]
        d_disks = [(a, r_pulley)]
        return {"proximal_pad": (strands, p_disks), "distal_pad": ([], d_disks)}
    return {"proximal_pad": empty, "distal_pad": empty}


def accessible_face_sets(
    morphology: FingerMorphology,
    phi_rad: float,
    *,
    include_linkage_occlusion: bool,
    include_fold: bool = True,
    fold_mode: Literal["crossing", "segment"] = "crossing",
) -> dict[str, IntervalSet]:
    """Compute ``U_i(M, phi; eta_L) <= [0, l_i]`` for ``i in {proximal_pad, distal_pad}``."""

    phi = require_finite("phi_rad", phi_rad)
    try:
        faces = morphology.posed_contact_faces(phi, 0.0)
    except (KinematicClosureError, MorphologyError, ValueError):
        return {"proximal_pad": IntervalSet.empty(), "distal_pad": IntervalSet.empty()}

    p_face, p_norm = faces["proximal_pad"]
    d_face, d_norm = faces["distal_pad"]
    mode: Literal["crossing", "segment", "none"] = fold_mode if include_fold else "none"
    base = base_accessible_sets(p_face, p_norm, d_face, d_norm, fold_mode=mode)
    if not include_linkage_occlusion:
        return base

    occluders = _linkage_occluders_for_morphology(morphology, phi)
    result: dict[str, IntervalSet] = {}
    for side, (face, norm) in (("proximal_pad", (p_face, p_norm)), ("distal_pad", (d_face, d_norm))):
        segs, disks = occluders[side]
        if not segs and not disks:
            result[side] = base[side]
        else:
            occ = face_occluded_set(face, norm, segments=segs, disks=disks)
            result[side] = base[side].difference(occ)
    return result


@dataclass(frozen=True)
class PadCoverageResult:
    """Complete SSoT pad coverage evaluation distinguishing mounted, endpoint, and motion-robust metrics."""

    finger_length_mm: float
    proximal_room_mm: float
    distal_room_mm: float
    proximal_mounted_mm: float
    distal_mounted_mm: float
    include_linkage_occlusion: bool
    include_fold: bool
    endpoint_sets: Mapping[str, IntervalSet]
    robust_sets: Mapping[str, IntervalSet]

    @property
    def mounted_coverage(self) -> float:
        return (self.proximal_mounted_mm + self.distal_mounted_mm) / self.finger_length_mm

    @property
    def proximal_endpoint_effective_mm(self) -> float:
        return self.endpoint_sets["proximal_pad"].measure

    @property
    def distal_endpoint_effective_mm(self) -> float:
        return self.endpoint_sets["distal_pad"].measure

    @property
    def endpoint_coverage(self) -> float:
        """Primary optimization metric ``C_endpoint(M; eta_L)`` at ``phi = phi_min^des``."""

        return (
            self.proximal_endpoint_effective_mm + self.distal_endpoint_effective_mm
        ) / self.finger_length_mm

    @property
    def proximal_robust_effective_mm(self) -> float:
        return self.robust_sets["proximal_pad"].measure

    @property
    def distal_robust_effective_mm(self) -> float:
        return self.robust_sets["distal_pad"].measure

    @property
    def motion_robust_coverage(self) -> float:
        """Supplementary diagnostic ``C_robust(M; eta_L)`` over ``[0, phi_min^des]``."""

        return (
            self.proximal_robust_effective_mm + self.distal_robust_effective_mm
        ) / self.finger_length_mm


def evaluate_pad_coverage(
    morphology: FingerMorphology,
    phi_min_des_rad: float,
    *,
    include_linkage_occlusion: bool,
    include_fold: bool = True,
    fold_mode: Literal["crossing", "segment"] = "crossing",
    motion_samples_rad: Sequence[float] | None = None,
    step_rad: float = math.radians(1.0),
    distal_tip_offset_mm: float = 0.0,
) -> PadCoverageResult:
    """Evaluate Endpoint Coverage and Motion-Robust Coverage on ``morphology``."""

    target = require_nonnegative("phi_min_des_rad", phi_min_des_rad)
    if not isinstance(include_linkage_occlusion, bool):
        raise ValueError("include_linkage_occlusion must be an explicit bool")

    endpoint = accessible_face_sets(
        morphology,
        target,
        include_linkage_occlusion=include_linkage_occlusion,
        include_fold=include_fold,
        fold_mode=fold_mode,
    )

    if motion_samples_rad is None:
        if target == 0.0:
            samples = (0.0,)
        else:
            n = max(int(math.ceil(target / step_rad)), 1)
            samples = tuple(float(v) for v in np.linspace(0.0, target, n + 1))
    else:
        samples = tuple(float(v) for v in motion_samples_rad)
        if not samples or abs(samples[-1] - target) > 1e-9:
            samples = (*samples, target)

    robust_p: IntervalSet | None = None
    robust_d: IntervalSet | None = None
    for phi in samples:
        at_phi = (
            endpoint
            if abs(phi - target) <= 1e-12
            else accessible_face_sets(
                morphology,
                phi,
                include_linkage_occlusion=include_linkage_occlusion,
                include_fold=include_fold,
                fold_mode=fold_mode,
            )
        )
        robust_p = at_phi["proximal_pad"] if robust_p is None else robust_p.intersection(at_phi["proximal_pad"])
        robust_d = at_phi["distal_pad"] if robust_d is None else robust_d.intersection(at_phi["distal_pad"])

    s_p, s_d = morphology.support_rooms_mm(distal_tip_offset_mm)
    cg = morphology.common_geometry
    return PadCoverageResult(
        finger_length_mm=cg.finger_length_mm,
        proximal_room_mm=s_p,
        distal_room_mm=s_d,
        proximal_mounted_mm=morphology.contact.proximal.length_mm,
        distal_mounted_mm=morphology.contact.distal.length_mm,
        include_linkage_occlusion=include_linkage_occlusion,
        include_fold=include_fold,
        endpoint_sets=MappingProxyType(endpoint),
        robust_sets=MappingProxyType(
            {
                "proximal_pad": robust_p if robust_p is not None else IntervalSet.empty(),
                "distal_pad": robust_d if robust_d is not None else IntervalSet.empty(),
            }
        ),
    )


__all__ = [
    "PROFILE_DEFAULT_LINKAGE_OCCLUSION",
    "CoverageProfile",
    "IntervalSet",
    "PadCoverageResult",
    "accessible_face_sets",
    "base_accessible_sets",
    "disk_occluded_interval",
    "evaluate_pad_coverage",
    "face_occluded_set",
    "resolve_linkage_occlusion",
    "segment_occluded_interval",
]
