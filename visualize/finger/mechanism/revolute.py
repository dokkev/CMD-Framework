"""The serial 2R's joint view: the pin at ``A`` with the servo on it (the
idealized 2R: the pin's housing alone), the distal phalanx a truss to its
pad (``core.morphology.mechanism.revolute``)."""

from __future__ import annotations

import importlib
import math

import numpy as np
from matplotlib.patches import Circle

from core.morphology.mechanism.revolute.model import decode
from visualize import sketch
from visualize.finger import solid
from visualize.finger.view import KinematicJoint

REVOLUTE_VARIABLES = 7
"""The 2R's design vector (``core.morphology.mechanism.revolute.VARIABLE_NAMES``)."""

# The module, not the package's evaluate() of the same name.
evaluate = importlib.import_module("core.optimization.revolute.evaluate")


class PinView(KinematicJoint):
    """The 2R's ``Finger`` (its joint a ``Pin``), its solids and its study config."""

    evaluate = evaluate

    def __init__(self, finger, geometry, config) -> None:
        super().__init__(finger)
        self.geometry, self.config = geometry, config

    @classmethod
    def load(cls, config, x) -> "PinView":
        return cls(decode(x[:REVOLUTE_VARIABLES]), cls.evaluate.geometry(x, config), config)

    @property
    def state(self):
        return self.geometry

    @property
    def packaging(self):
        return self.config.goal.packaging

    def margins(self, phi_deg) -> dict[tuple[str, str], np.ndarray]:
        return self.evaluate._margins(self.geometry, np.radians(phi_deg), self.config)

    def distal_pad(self, phi_deg: float) -> np.ndarray:
        return self.geometry.distal_pad(np.radians([phi_deg]))[0]

    def proximal_pad(self) -> np.ndarray:
        return self.geometry.proximal_pad

    def draw_sketch(self, axis, phi_deg: float, hit: set[str]) -> None:
        geo = self.geometry
        a = np.array((geo.a_x_mm, 0.0))
        near, far = geo.posed(geo.distal_mount, [math.radians(phi_deg)])[0]
        sketch.links(axis, ((0.0, 0.0), a), (a, near, far, a))  # the distal phalanx: a truss to the pad's ends
        sketch.nodes(axis, near, far)
        if self.config.joint_servo:
            sketch.servo(axis, geo.distal_servo.outline)
            sketch.active(axis, (0.0, 0.0), a)
            return
        axis.add_patch(Circle(a, self.config.joint_radius_mm, facecolor="white", edgecolor=sketch.INK,
                              linewidth=1.0, zorder=4))  # the idealized 2R: the pin, no servo there
        sketch.nodes(axis, a)
        sketch.active(axis, (0.0, 0.0))

    def truss(self, phi_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The distal phalanx's nodes at the flexion: ``B''`` and the fingertip end of its pad's mount, and ``C``."""

        geo = self.geometry
        near, far = geo.posed(geo.distal_mount, [math.radians(phi_deg)])[0]
        return near, far, geo.tip(np.radians([phi_deg]))[0]

    def draw_solid(self, axis, phi_deg: float, hit: set[str]) -> None:
        """The pin at ``A`` (its servo's or, idealized, its housing's
        circle), ``O-A`` red and the distal truss blue."""

        geo, config = self.geometry, self.config
        a = np.array((geo.a_x_mm, 0.0))
        near, far, c = self.truss(phi_deg)
        if config.joint_servo:
            solid.servo(axis, geo.distal_servo.outline, "proximal", "pip_servo" in hit)
        solid.pin(axis, a, config.joint_radius_mm, "proximal", "a_pin" in hit)
        solid.link(axis, ((0.0, 0.0), a), solid.RED)
        solid.link(axis, (a, near, far, a), solid.BLUE)
        solid.point(axis, (0.0, 0.0), solid.YELLOW, solid.RED, "$O$", offset=(-12, 12))
        solid.point(axis, a, solid.YELLOW if config.joint_servo else solid.RED, solid.BLACK, "$A$", solid.RED)
        solid.point(axis, c, solid.BLUE, solid.BLUE, "$C$", offset=(12, -12))


__all__ = ["PinView"]
