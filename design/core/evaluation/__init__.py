"""The problem formulation's evaluation quantities, shared by every mechanism
(``design/SSoT/problem_formulation.md`` A.4-A.5, ``code_formulation.md`` §4).

- ``coverage``: cylinder-based common pad length, posed finger length, contact pad coverage
- ``motion``: reach of the valid motion, ``F_motion``, ``q_max``, transmission angle range
- ``transmission``: mechanical advantage from the actuator angle along the motion

A mechanism supplies the geometry (its faces, its phalanx ends, its sampled
motion); these functions turn it into the formulation's quantities.
"""

from core.evaluation.coverage import (
    PoseCoverage,
    common_length_mm,
    mean_coverage,
    evaluation_poses_rad,
    pad_coverage,
    posed_finger_length_mm,
)
from core.evaluation.motion import MotionSamples, gamma_violation_deg, reach_rad, uniform_samples_rad
from core.evaluation.transmission import mechanical_advantage, worst_mechanical_advantage

__all__ = [
    "MotionSamples",
    "PoseCoverage",
    "mean_coverage",
    "common_length_mm",
    "evaluation_poses_rad",
    "gamma_violation_deg",
    "mechanical_advantage",
    "pad_coverage",
    "posed_finger_length_mm",
    "reach_rad",
    "uniform_samples_rad",
    "worst_mechanical_advantage",
]
