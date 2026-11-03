"""The CoDe linkage's joint view: the crossed four-bar posed by the distal
flexion, the PIP servo on ``A'``, the pads on their links
(``core.morphology.mechanism.linkage``)."""

from __future__ import annotations

import numpy as np

from core.morphology.mechanism.linkage.design import decode
from core.morphology.mechanism.linkage.packaging import (
    distal_pad_polygon,
    pad_margins,
    pip_footprint,
    proximal_pad_polygon,
)
from visualize import sketch
from visualize.finger.view import KinematicJoint


class FourBarView(KinematicJoint):
    """The linkage's ``Finger`` (its joint a ``FourBar``) and its packaging."""

    def __init__(self, finger, packaging) -> None:
        super().__init__(finger)
        self.packaging = packaging

    @classmethod
    def load(cls, config, x) -> "FourBarView":
        return cls(decode(x, config.link_length_resolution_mm), config.packaging)

    @property
    def state(self):
        return self.finger

    def margins(self, phi_deg) -> dict[tuple[str, str], np.ndarray]:
        s = self.sweep(phi_deg)
        return pad_margins(self.finger, self.packaging, s.b, s.tip, s.bp, (0.0,))

    def distal_pad(self, phi_deg: float) -> np.ndarray:
        s = self.sweep([phi_deg])
        return distal_pad_polygon(self.finger, self.packaging, s.b, s.tip)[0]

    def proximal_pad(self) -> np.ndarray:
        return proximal_pad_polygon(self.finger, self.packaging)

    def draw(self, axis, phi_deg: float, hit: set[str]) -> None:
        s = self.sweep([phi_deg])
        b, c, bp = s.b[0], s.tip[0], s.bp[0]
        joint = self.finger.joint
        o, a, ap = (0.0, 0.0), joint.a.array, joint.ap.array
        sketch.servo(axis, pip_footprint(self.packaging, self.finger).outline)
        sketch.links(axis, (o, a), (o, ap), (ap, b), (a, bp), (bp, b, c, bp))
        sketch.nodes(axis, a, b, bp, c)
        sketch.active(axis, o, ap)


__all__ = ["FourBarView"]
