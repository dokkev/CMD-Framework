"""The rolling contact joint's view: two rollers rolling on each other, the
servo on ``A`` turning their carrier (``core.morphology.mechanism.rcj``). It
has no ``Finger`` of its own yet, so it poses its solids itself."""

from __future__ import annotations

import importlib
import math

import numpy as np

from core.morphology.mechanism.rcj import TORQUE_RATIO
from visualize import sketch
from visualize.finger import solid

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

    def draw_sketch(self, axis, phi_deg: float, hit: set[str]) -> None:
        geo, phi = self.geometry, math.radians(phi_deg)
        sketch.servo(axis, geo.servo.outline)
        b = geo.b([phi])[0]
        for center, name in ((geo.a, "proximal_roller"), (b, "distal_roller")):
            sketch.roller(axis, center, geo.roller_radius_mm, name in hit)
        near, far = geo.posed(geo.distal_mount, [phi])[0]
        sketch.links(axis, ((0.0, 0.0), geo.a), (geo.a, b), (b, near, far, b))
        sketch.nodes(axis, b, near, far)
        sketch.active(axis, (0.0, 0.0), geo.a)

    def status(self, phi_deg: float) -> list[str]:
        return []

    def draw_solid(self, axis, phi_deg: float, hit: set[str]) -> None:
        """The servo on ``A``, the proximal roller (proximal) and the distal
        one (distal), the carrier ``A-B`` grey, ``O-A`` red and the distal truss blue."""

        geo, phi = self.geometry, math.radians(phi_deg)
        b = geo.b([phi])[0]
        near, far = geo.posed(geo.distal_mount, [phi])[0]
        solid.servo(axis, geo.servo.outline, "proximal", "pip_servo" in hit)
        solid.roller(axis, geo.a, geo.roller_radius_mm, "proximal", "proximal_roller" in hit)
        solid.roller(axis, b, geo.roller_radius_mm, "distal", "distal_roller" in hit)
        solid.link(axis, ((0.0, 0.0), geo.a), solid.RED)
        solid.link(axis, (geo.a, b), solid.GRAY, zorder=3)
        solid.link(axis, (b, near, far, b), solid.BLUE)
        solid.point(axis, (0.0, 0.0), solid.YELLOW, solid.RED, "$O$", offset=(-12, 12))
        solid.point(axis, geo.a, solid.YELLOW, solid.GRAY, "$A$", solid.RED)
        solid.point(axis, b, solid.BLUE, solid.GRAY, "$B$", solid.BLUE)
        solid.point(axis, self.tip(phi_deg), solid.BLUE, solid.BLUE, "$C$", offset=(12, -12))


__all__ = ["RollerView"]
