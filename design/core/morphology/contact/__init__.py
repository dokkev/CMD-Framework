"""The contact faces of any finger mechanism: what of them is lost, and to
what (``loss``)."""

from core.morphology.contact.loss import (
    CATEGORIES,
    FaceLoss,
    Scene,
    Shapes,
    dorsal_space_mm,
    face_loss,
    integrity_free,
    joint_scene,
    linkage_scene,
    thickness_mm,
)
from core.morphology.coverage import (
    PROFILE_DEFAULT_LINKAGE_OCCLUSION,
    CoverageProfile,
    IntervalSet,
    PadCoverageResult,
    accessible_face_sets,
    evaluate_pad_coverage,
    resolve_linkage_occlusion,
)

__all__ = [
    "CATEGORIES",
    "PROFILE_DEFAULT_LINKAGE_OCCLUSION",
    "CoverageProfile",
    "FaceLoss",
    "IntervalSet",
    "PadCoverageResult",
    "Scene",
    "Shapes",
    "accessible_face_sets",
    "dorsal_space_mm",
    "evaluate_pad_coverage",
    "face_loss",
    "integrity_free",
    "joint_scene",
    "linkage_scene",
    "resolve_linkage_occlusion",
    "thickness_mm",
]
