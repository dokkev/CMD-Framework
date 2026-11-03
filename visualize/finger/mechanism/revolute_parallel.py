"""The belt 2R's joint view: the 2R's pin at ``A`` turned through a belt by
the servo on ``A'`` (``core.morphology.mechanism.revolute_parallel``)."""

from __future__ import annotations

import importlib
import math

import numpy as np
from matplotlib.patches import Circle

from visualize import sketch
from visualize.finger import solid
from visualize.finger.mechanism.revolute import PinView


class BeltPinView(PinView):
    """The 2R's view with the servo and the drive pulley on ``A'``."""

    evaluate = importlib.import_module("core.optimization.revolute_parallel.evaluate")

    def draw_sketch(self, axis, phi_deg: float, hit: set[str]) -> None:
        geo = self.geometry
        sketch.servo(axis, geo.servo.outline)
        a, ap, radius = geo.a, geo.ap, geo.pulley_radius_mm
        for center in (a, ap):
            axis.add_patch(Circle(center, radius, fill=False, edgecolor=sketch.INK, linewidth=1.0, zorder=3.5))
        along = (a - ap) / np.linalg.norm(a - ap)
        side = radius * np.array((-along[1], along[0]))
        for sign in (1.0, -1.0):  # the belt's two strands
            axis.plot(*np.array((ap + sign * side, a + sign * side)).T, color=sketch.INK, linewidth=1.0, zorder=3.5)
        near, far = geo.joint.posed(geo.joint.distal_mount, [math.radians(phi_deg)])[0]
        sketch.links(axis, ((0.0, 0.0), a, ap), (a, near, far, a))
        sketch.nodes(axis, a, near, far)
        sketch.active(axis, (0.0, 0.0), ap)

    def draw_solid(self, axis, phi_deg: float, hit: set[str]) -> None:
        """The servo on ``A'``, the belt over its pulleys to the pin at ``A``,
        ``O-A-A'`` red and the distal truss blue."""

        geo, config = self.geometry, self.config
        a, ap, radius = geo.a, geo.ap, geo.pulley_radius_mm
        near, far, c = self.truss(phi_deg)
        solid.servo(axis, geo.servo.outline, "proximal", "pip_servo" in hit)
        solid.pin(axis, a, config.joint_radius_mm, "proximal", "a_pin" in hit)
        along = (a - ap) / np.linalg.norm(a - ap)
        side = radius * np.array((-along[1], along[0]))
        for center in (a, ap):
            axis.add_patch(Circle(center, radius, fill=False, edgecolor=solid.GRAY, linewidth=1.0, zorder=3.5))
        for sign in (1.0, -1.0):  # the belt's two strands
            solid.link(axis, (ap + sign * side, a + sign * side), solid.GRAY, width=1.2, zorder=3.5)
        solid.link(axis, ((0.0, 0.0), a, ap), solid.RED)
        solid.link(axis, (a, near, far, a), solid.BLUE)
        solid.point(axis, (0.0, 0.0), solid.YELLOW, solid.RED, "$O$", offset=(-12, 12))
        solid.point(axis, a, solid.RED, solid.BLACK, "$A$", solid.RED)
        solid.point(axis, ap, solid.YELLOW, solid.GRAY, "$A'$", solid.RED, (0, 15))
        solid.point(axis, c, solid.BLUE, solid.BLUE, "$C$", offset=(12, -12))


__all__ = ["BeltPinView"]
