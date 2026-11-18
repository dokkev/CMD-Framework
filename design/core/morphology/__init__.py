"""Concrete analytic morphology components: what each finger mechanism is
and does (design vector, kinematics, solids, their gaps along the motion),
apart from the requirements a study holds it to (``core.optimization``).

- ``finger``: a finger as its segments, the joint between them and its servo's mount
- ``code_hand``: the five-digit 3D CoDe Hand assembly, frames and forward kinematics
- ``mechanism``: the finger mechanisms (``linkage``, the CoDe; ``revolute``,
  ``revolute_parallel``, ``rcj``)
- ``packaging``: the common hardware, its rigid bodies in Y layers and their gaps
- ``contact``: what of a contact face is lost (``loss``)
- ``icr``: where each mechanism bends when it starts to flex
"""

from core.morphology.canonical import (
    ARCHITECTURES,
    ARCHITECTURE_TO_MECHANISM,
    DEFAULT_PIVOT_RADII_MM,
    MECHANISM_TO_ARCHITECTURE,
    ActuationParams,
    ActuatorSpec,
    Architecture,
    CoDeMechanismParams,
    CommonGeometry,
    ContactModuleSpec,
    ContactParams,
    FingerMorphology,
    MechanismParams,
    MorphologyError,
    RcjMechanismParams,
    RevoluteMechanismParams,
    StructureParams,
)
from core.morphology.collision import (
    DynamicCollisionResult,
    StaticCollisionResult,
    dynamic_checkpoints_deg,
    dynamic_checkpoints_rad,
    evaluate_dynamic_collision,
    evaluate_static_collision,
    pad_pad_clearance_mm,
    rect_disk_clearance_mm,
    verify_dynamic_collision_refined,
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
from core.morphology.finger import (
    Finger,
    Joint,
    Kinematics,
    PointXZ,
    Segment,
    ServoMount,
)

__all__ = [
    "ARCHITECTURES",
    "ARCHITECTURE_TO_MECHANISM",
    "DEFAULT_PIVOT_RADII_MM",
    "MECHANISM_TO_ARCHITECTURE",
    "PROFILE_DEFAULT_LINKAGE_OCCLUSION",
    "ActuationParams",
    "ActuatorSpec",
    "Architecture",
    "CoDeMechanismParams",
    "CommonGeometry",
    "ContactModuleSpec",
    "ContactParams",
    "CoverageProfile",
    "DynamicCollisionResult",
    "Finger",
    "FingerMorphology",
    "IntervalSet",
    "Joint",
    "Kinematics",
    "MechanismParams",
    "MorphologyError",
    "PadCoverageResult",
    "PointXZ",
    "RcjMechanismParams",
    "RevoluteMechanismParams",
    "Segment",
    "ServoMount",
    "StaticCollisionResult",
    "StructureParams",
    "accessible_face_sets",
    "dynamic_checkpoints_deg",
    "dynamic_checkpoints_rad",
    "evaluate_dynamic_collision",
    "evaluate_pad_coverage",
    "evaluate_static_collision",
    "pad_pad_clearance_mm",
    "rect_disk_clearance_mm",
    "resolve_linkage_occlusion",
    "verify_dynamic_collision_refined",
]
