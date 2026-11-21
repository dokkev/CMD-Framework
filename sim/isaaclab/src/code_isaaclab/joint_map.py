# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The map between a finger's flexion q and its PIP servo joint theta, from the exported four-bar table.

A policy that commands and observes flexion q is independent of the hand's transmission: each hand turns
q into its own servo target theta_H(q) and reads q back from the servo through theta_H^-1 (Study 3,
``design/SSoT/research-objective.md`` Section 8). The transmission ratio dtheta/dq (the mechanical advantage
tau_pip / tau_servo by virtual work) sets how stiff the finger is in q under a servo position gain:
K_q = K_theta (dtheta/dq)^2, so a gain scheduled as K_theta(q) = K_q* / (dtheta/dq)^2 gives every hand the same
flexion stiffness K_q*.

The table is the description's ``four_bar[digit]``: ``pip_deg`` (q) and ``ap_deg`` (the PIP servo's tree joint at
A'), sampled every degree. theta(q) is interpolated linearly; dtheta/dq is the table's central difference
(one-sided at the ends), interpolated linearly. The servo's actuator angle is ``theta - zero_deg`` (the
description's convention), which only a hardware command needs; a simulator targets the joint angle.

Nothing here imports Isaac Lab or torch.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FlexionServoMap:
    """One digit's flexion-to-servo map (angles in rad)."""

    digit: str
    q: np.ndarray
    """Flexion samples, ascending."""
    theta: np.ndarray
    """PIP servo joint angle at each flexion sample."""
    zero: float
    """The servo's actuator zero (joint angle at the servo's zero) [rad]."""

    def __post_init__(self) -> None:
        if self.q.ndim != 1 or self.q.shape != self.theta.shape or len(self.q) < 3:
            raise ValueError(f"{self.digit}: need matching 1-D tables of at least 3 samples")
        if not np.all(np.diff(self.q) > 0):
            raise ValueError(f"{self.digit}: flexion samples must be strictly ascending")
        steps = np.diff(self.theta)
        if not (np.all(steps > 0) or np.all(steps < 0)):
            raise ValueError(f"{self.digit}: theta(q) is not monotonic over the table; it has no inverse")

    @classmethod
    def from_table(cls, digit: str, table: Mapping[str, Sequence[float]], zero_deg: float = 0.0) -> FlexionServoMap:
        q = np.radians(np.asarray(table["pip_deg"], dtype=float))
        theta = np.radians(np.asarray(table["ap_deg"], dtype=float))
        order = np.argsort(q)
        return cls(digit, q[order], theta[order], math.radians(zero_deg))

    @property
    def q_range(self) -> tuple[float, float]:
        return float(self.q[0]), float(self.q[-1])

    def servo(self, q) -> np.ndarray:
        """theta_H(q): the servo joint angle that gives flexion q (clamped to the table)."""
        return np.interp(np.asarray(q, dtype=float), self.q, self.theta)

    def flexion(self, theta) -> np.ndarray:
        """theta_H^-1: the flexion a servo joint angle gives (clamped to the table)."""
        theta = np.asarray(theta, dtype=float)
        if self.theta[0] < self.theta[-1]:
            return np.interp(theta, self.theta, self.q)
        return np.interp(theta, self.theta[::-1], self.q[::-1])

    def ratio(self, q) -> np.ndarray:
        """dtheta/dq at flexion q (the transmission ratio; the mechanical advantage tau_pip / tau_servo)."""
        return np.interp(np.asarray(q, dtype=float), self.q, np.gradient(self.theta, self.q))

    def servo_gain(self, q, flexion_stiffness: float) -> np.ndarray:
        """The servo position gain that gives flexion stiffness ``flexion_stiffness`` at q: K_q* / (dtheta/dq)^2."""
        return flexion_stiffness / self.ratio(q) ** 2

    def actuator(self, q) -> np.ndarray:
        """The servo's actuator angle for flexion q (hardware command): theta_H(q) - zero."""
        return self.servo(q) - self.zero


def flexion_maps(description: Mapping) -> dict[str, FlexionServoMap]:
    """Every four-bar digit's map from a ``code_hand_sim/1`` description (the parsed JSON)."""
    zeros = {j["name"]: (j.get("actuator") or {}).get("zero_deg", 0.0) for j in description["joints"]}
    return {digit: FlexionServoMap.from_table(digit, table, zeros.get(f"{digit}_pip_servo", 0.0))
            for digit, table in description.get("four_bar", {}).items()}


__all__ = ["FlexionServoMap", "flexion_maps"]
