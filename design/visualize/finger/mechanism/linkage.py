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
from visualize.finger import solid
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

    def draw_sketch(self, axis, phi_deg: float, hit: set[str]) -> None:
        s = self.sweep([phi_deg])
        b, c, bp = s.b[0], s.tip[0], s.bp[0]
        joint = self.finger.joint
        o, a, ap = (0.0, 0.0), joint.a.array, joint.ap.array
        sketch.servo(axis, pip_footprint(self.packaging, self.finger).outline)
        sketch.links(axis, (o, a), (o, ap), (ap, b), (a, bp), (bp, b, c, bp))
        sketch.nodes(axis, a, b, bp, c)
        sketch.active(axis, o, ap)

    def status(self, phi_deg: float) -> list[str]:
        s = self.sweep([phi_deg])
        # The transmission angle is at the joint the driven link does not touch.
        return [f"transmission at B' {np.degrees(s.margin_at_bp_rad[0]):3.0f}°  "
                f"limit position at B {np.degrees(s.margin_at_b_rad[0]):3.0f}°"]

    def draw_solid(self, axis, phi_deg: float, hit: set[str]) -> None:
        """As ``visualize.animate`` draws it: the bars ``O-A-A'`` red, ``A-B'``
        black, ``A'-B`` grey, ``B'-B-C`` blue; each pin's clearance circle,
        dashed ``A``'s, ``B'``'s and ``B``'s grown by the body clearance;
        ``A''`` on ``O-A`` and ``B''`` on ``B-C``, dashed to the joints of
        their rigid bodies (``A``, ``A'``; ``B``, ``B'``)."""

        s = self.sweep([phi_deg])
        b, c, bp = s.b[0], s.tip[0], s.bp[0]
        joint, packaging = self.finger.joint, self.packaging
        o, a, ap = np.zeros(2), joint.a.array, joint.ap.array
        solid.servo(axis, pip_footprint(packaging, self.finger).outline, "proximal", "pip_servo" in hit)
        grown = packaging.body_clearance_mm
        for name, center, group in (("a", a, "proximal"), ("ap", ap, "proximal"), ("bp", bp, "distal"),
                                    ("b", b, "distal")):
            radius = packaging.pivot_radius_mm(name)
            solid.pin(axis, center, radius, group, f"{name}_pin" in hit, None if name == "ap" else radius + grown)
        solid.link(axis, (o, a, ap, o), solid.RED)
        solid.link(axis, (a, bp), solid.BLACK, zorder=3)
        solid.link(axis, (bp, b, c, bp), solid.BLUE)
        solid.link(axis, (ap, b), solid.GRAY, zorder=3)
        # A'' and B'' on their link lines (O-A, B-C), dashed to the joints of the
        # same rigid body: A'' to A and A', B'' to B and B'.
        app, bpp = self.finger.proximal_end.array, self.distal_start(phi_deg)
        for end, joints, color in ((app, (a, ap), solid.RED), (bpp, (b, bp), solid.BLUE)):
            for joint_point in joints:
                solid.link(axis, (joint_point, end), color, width=1.4, zorder=1.8, dashed=True)
        red, blue, gray, yellow = solid.RED, solid.BLUE, solid.GRAY, solid.YELLOW
        solid.point(axis, o, yellow, red, "$O$", offset=(-12, 12))
        solid.point(axis, a, red, solid.BLACK, "$A$", red)
        solid.point(axis, ap, yellow, gray, "$A'$", red, (0, 15))
        solid.point(axis, bp, blue, solid.BLACK, "$B'$", blue, (0, 15))
        solid.point(axis, b, blue, gray, "$B$", blue)
        solid.point(axis, c, blue, blue, "$C$", offset=(12, -12))
        solid.point(axis, app, red, red, "$A''$", size=6)
        solid.point(axis, bpp, blue, blue, "$B''$", offset=(-14, -14), size=6)


__all__ = ["FourBarView"]
