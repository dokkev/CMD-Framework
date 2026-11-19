"""Mechanical advantage (problem formulation A.5.2).

``MA = |d theta_pip / d q_2|``, the PIP actuator's angle per unit flexion
(virtual work: the output torque per unit actuator torque). It is taken by
central differences on the uniform ``q_2`` samples of the motion.
"""

from __future__ import annotations

import math

import numpy as np

from core.evaluation.motion import MotionSamples


def mechanical_advantage(motion: MotionSamples) -> np.ndarray:
    """``MA`` at each sample (NaN with fewer than two samples)."""

    if len(motion.q_rad) < 2:
        return np.full(len(motion.q_rad), math.nan)
    return np.abs(np.gradient(motion.theta_rad, motion.q_rad))


def worst_mechanical_advantage(motion: MotionSamples, q_req_rad: float) -> float:
    """``MA_wc``: the least ``MA`` on ``[0, q_req]`` (0 when the motion does not reach ``q_req``)."""

    if motion.valid_reach_rad() < q_req_rad - 1e-9 or len(motion.q_rad) < 2:
        return 0.0
    return float(np.min(mechanical_advantage(motion)[motion.window(q_req_rad)]))


__all__ = ["mechanical_advantage", "worst_mechanical_advantage"]
