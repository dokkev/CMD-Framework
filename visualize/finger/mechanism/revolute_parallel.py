"""The belt 2R's joint view: the 2R's pin at ``A`` turned through a belt by
the servo on ``A'`` (``core.morphology.mechanism.revolute_parallel``)."""

from __future__ import annotations

import importlib
import math

import numpy as np
from matplotlib.patches import Circle

from visualize import sketch
from visualize.finger.mechanism.revolute import PinView


class BeltPinView(PinView):
    """The 2R's view with the servo and the drive pulley on ``A'``."""

    evaluate = importlib.import_module("core.optimization.revolute_parallel.evaluate")

    def draw(self, axis, phi_deg: float, hit: set[str]) -> None:
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


__all__ = ["BeltPinView"]
