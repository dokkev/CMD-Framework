"""The rolling contact joint's view: two rollers rolling on each other, the
servo on ``A`` turning their carrier (``core.morphology.mechanism.rcj``). It
has no ``Finger`` of its own yet, so it poses its solids itself."""

from __future__ import annotations

import importlib
import math

import numpy as np

from core.morphology.mechanism.rcj import TORQUE_RATIO
from visualize import sketch

# The module, not the package's evaluate() of the same name.
evaluate = importlib.import_module("core.optimization.rcj.evaluate")


class RollerView:
    """The RCJ's solids and its study config."""

    def __init__(self, geometry, config) -> None:
        self.geometry, self.config = geometry, config

    @classmethod
    def load(cls, config, x) -> "RollerView":
        return cls(evaluate.geometry(x, config), config)

    @property
    def state(self):
        return self.geometry

    @property
    def packaging(self):
        return self.config.goal.packaging

    def reachable_deg(self) -> float:
        return 180.0

    def margins(self, phi_deg) -> dict[tuple[str, str], np.ndarray]:
        return evaluate._margins(self.geometry, np.radians(phi_deg), self.config)

    def icr(self, phi_deg) -> np.ndarray:
        """The rollers' contact point, halfway along the carrier."""

        return (self.geometry.a + self.geometry.b(np.radians(phi_deg))) / 2.0

    def k(self, phi_deg) -> np.ndarray:
        return np.full(len(phi_deg), TORQUE_RATIO)

    def tip(self, phi_deg: float) -> np.ndarray:
        return self.geometry.tip(np.radians([phi_deg]))[0]

    def distal_pad(self, phi_deg: float) -> np.ndarray:
        return self.geometry.distal_pad(np.radians([phi_deg]))[0]

    def proximal_pad(self) -> np.ndarray:
        return self.geometry.proximal_pad

    def draw(self, axis, phi_deg: float, hit: set[str]) -> None:
        geo, phi = self.geometry, math.radians(phi_deg)
        sketch.servo(axis, geo.servo.outline)
        b = geo.b([phi])[0]
        for center, name in ((geo.a, "proximal_roller"), (b, "distal_roller")):
            sketch.roller(axis, center, geo.roller_radius_mm, name in hit)
        near, far = geo.posed(geo.distal_mount, [phi])[0]
        sketch.links(axis, ((0.0, 0.0), geo.a), (geo.a, b), (b, near, far, b))
        sketch.nodes(axis, b, near, far)
        sketch.active(axis, (0.0, 0.0), geo.a)


__all__ = ["RollerView"]
