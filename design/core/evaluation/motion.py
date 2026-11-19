"""The valid motion and its reach (problem formulation A.4, A.5.2, A.5.3).

A mechanism samples its PIP flexion ``q_2`` on a uniform grid from extension
and reports, per sample, whether the pose is valid (its geometry exists and
has no self-collision), its transmission angle (CFL only) and its actuator
angle. Everything below reads those samples.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TOLERANCE_RAD = 1e-9


def uniform_samples_rad(end_rad: float, step_rad: float) -> np.ndarray:
    """``0, step, 2 step, ...`` up to and including ``end_rad``."""

    count = int(math.floor(end_rad / step_rad + 1e-9))
    samples = np.arange(count + 1) * step_rad
    if end_rad - samples[-1] > 1e-12:
        samples = np.append(samples, end_rad)
    return samples


def reach_rad(q_rad: np.ndarray, ok: np.ndarray) -> float:
    """The largest ``q`` such that every sample up to it is ``ok``: the sample
    before the first one that is not (0 when the first is not)."""

    bad = np.flatnonzero(~np.asarray(ok, dtype=bool))
    if not len(q_rad):
        return 0.0
    if not bad.size:
        return float(q_rad[-1])
    return 0.0 if bad[0] == 0 else float(q_rad[bad[0] - 1])


def gamma_violation_deg(gamma_rad: np.ndarray, gamma_range_rad: tuple[float, float]) -> float:
    """The most the transmission angle leaves ``[gamma_min, gamma_max]`` over the samples (deg; 0 within)."""

    if not len(gamma_rad):
        return 0.0
    low, high = gamma_range_rad
    outside = np.maximum(low - gamma_rad, gamma_rad - high)
    return max(0.0, math.degrees(float(outside.max())))


@dataclass(frozen=True)
class MotionSamples:
    """A mechanism's motion sampled from extension on a uniform ``q_2`` grid.

    The arrays cover the samples where the geometry exists (the mechanism
    closes); past the last one nothing is valid.
    """

    q_rad: np.ndarray
    valid: np.ndarray
    """The pose is valid: no self-collision (A.4)."""
    theta_rad: np.ndarray
    """The PIP actuator's input angle."""
    gamma_rad: np.ndarray | None = None
    """The transmission angle (CFL only)."""
    gamma_range_rad: tuple[float, float] | None = None

    @property
    def gamma_ok(self) -> np.ndarray:
        if self.gamma_rad is None or self.gamma_range_rad is None:
            return np.ones(len(self.q_rad), dtype=bool)
        low, high = self.gamma_range_rad
        return (self.gamma_rad >= low - TOLERANCE_RAD) & (self.gamma_rad <= high + TOLERANCE_RAD)

    def window(self, q_req_rad: float) -> np.ndarray:
        return self.q_rad <= q_req_rad + TOLERANCE_RAD

    def valid_reach_rad(self) -> float:
        return reach_rad(self.q_rad, self.valid)

    def q_max_rad(self) -> float:
        """``q_max``: the most flexion with every pose valid and the transmission angle in range, at most pi."""

        return min(reach_rad(self.q_rad, self.valid & self.gamma_ok), math.pi)

    def f_motion_violation_deg(self, q_req_rad: float) -> float:
        """How far (deg) the valid motion falls short of ``q_req`` (0: ``F_motion`` holds)."""

        return max(0.0, math.degrees(q_req_rad - self.valid_reach_rad()))

    def gamma_violation_deg(self, q_req_rad: float) -> float:
        if self.gamma_rad is None or self.gamma_range_rad is None:
            return 0.0
        return gamma_violation_deg(self.gamma_rad[self.window(q_req_rad)], self.gamma_range_rad)


__all__ = ["MotionSamples", "gamma_violation_deg", "reach_rad", "uniform_samples_rad"]
