"""Analytic geometry for the concrete contact-morphology fingertip.

The canonical frame is ``X`` lateral, ``Y`` longitudinal, and ``Z`` dorsal;
the finger extends toward positive ``Y``.  The straight body occupies
``0 <= y <= L`` and the distal nose occupies ``L <= y <= L + b``.  All public
dimensions and query coordinates are in millimetres.

The lower contact surface is a transverse semiellipse in the straight body,

``z = -h - c * sqrt(1 - (x/a)**2)``,

and a true distal quarter-ellipsoid,

``z = -h - c * sqrt(1 - (x/a)**2 - ((y-L)/b)**2)``.

The compliant outer envelope is represented separately from the rigid carrier:
at a footprint point it spans from the analytic lower contact surface to the
carrier shoulder plane ``z = 0``.  The carrier is a derived XZ polygon.  Its
fundamental component is a T-shaped phalanx; the exposed polygon additionally
contains the fixed stepped bonding interface.  The distal rigid core extends
into the ellipsoid only by ``carrier_extension_mm``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .fingertip_parameter import FingertipParameters


PointXZ = tuple[float, float]
_QUERY_TOLERANCE_MM = 1e-12


def _require_finite_query(name: str, value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite coordinate in mm; got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite coordinate in mm; got {value!r}")
    return value


@dataclass(frozen=True)
class Fingertip:
    """Constructed analytic geometry for one parameterized fingertip."""

    parameters: FingertipParameters = field(default_factory=FingertipParameters)

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, FingertipParameters):
            raise TypeError("parameters must be a FingertipParameters instance")

    @property
    def half_width_mm(self) -> float:
        """Maximum half-width of the straight contact body."""

        return self.parameters.pad.width_mm / 2.0

    @property
    def total_length_mm(self) -> float:
        """Longitudinal length from the body root to the distal tip."""

        pad = self.parameters.pad
        return pad.body_length_mm + pad.tip_length_mm

    @property
    def carrier_length_mm(self) -> float:
        """Length of the rigid core through the body and into the nose."""

        pad = self.parameters.pad
        return pad.body_length_mm + self.parameters.phalanx.carrier_extension_mm

    @property
    def dorsal_plate_length_mm(self) -> float:
        """Length of the dorsal plate interface, through the complete nose."""

        return self.total_length_mm

    @property
    def a(self) -> float:
        """Transverse semi-axis ``a`` of the pad, in mm."""

        return self.half_width_mm

    @property
    def b(self) -> float:
        """Longitudinal semi-axis ``b`` of the distal quarter-ellipsoid, in mm."""

        return self.parameters.pad.tip_length_mm

    @property
    def c(self) -> float:
        """Dorsal semi-axis ``c`` of the distal quarter-ellipsoid, in mm."""

        return self.parameters.pad.ellipse_height_mm

    def half_width_at_y_mm(self, y_mm: float) -> float:
        """Return the available pad half-width at longitudinal coordinate ``y``.

        The footprint is constant through the straight body and follows the
        ellipsoidal ``a * sqrt(1 - v**2)`` law in the distal nose.
        """

        y_mm = self._validate_y(y_mm)
        body_length_mm = self.parameters.pad.body_length_mm
        if y_mm <= body_length_mm:
            return self.a

        v = (y_mm - body_length_mm) / self.b
        return self.a * math.sqrt(max(0.0, 1.0 - v * v))

    def lower_surface_z_mm(self, x_mm: float, y_mm: float) -> float:
        """Return the analytic lower contact surface at ``(x, y)``.

        ``ValueError`` is raised when the query is outside the longitudinal
        domain or outside the available cross-sectional footprint.
        """

        x_mm = _require_finite_query("x_mm", x_mm)
        y_mm = self._validate_y(y_mm)
        half_width_mm = self.half_width_at_y_mm(y_mm)
        self._validate_x_in_footprint(x_mm, half_width_mm)

        pad = self.parameters.pad
        h_mm = pad.pad_straight_height_mm
        if y_mm <= pad.body_length_mm:
            normalized_x = x_mm / self.a
            radicand = max(0.0, 1.0 - normalized_x * normalized_x)
        else:
            v = (y_mm - pad.body_length_mm) / self.b
            normalized_x = x_mm / self.a
            radicand = max(0.0, 1.0 - normalized_x * normalized_x - v * v)
        return -h_mm - self.c * math.sqrt(radicand)

    def is_inside_pad_footprint(self, x_mm: float, y_mm: float) -> bool:
        """Return whether ``(x, y)`` belongs to the closed pad footprint.

        Longitudinal coordinates outside the fingertip domain are invalid
        queries and raise ``ValueError``.  A lateral coordinate outside the
        footprint simply produces ``False``.
        """

        x_mm = _require_finite_query("x_mm", x_mm)
        y_mm = self._validate_y(y_mm)
        return abs(x_mm) <= self.half_width_at_y_mm(y_mm) + _QUERY_TOLERANCE_MM

    def outer_envelope_z_bounds_mm(self, x_mm: float, y_mm: float) -> tuple[float, float]:
        """Return ``(lower_z, upper_z)`` for the compliant outer envelope.

        The supplied morphology defines the upper shoulder plane at ``z=0``;
        the dorsal carrier above that plane is returned separately through its
        carrier cross-section rather than folded into the contact envelope.
        """

        lower_z_mm = self.lower_surface_z_mm(x_mm, y_mm)
        return lower_z_mm, 0.0

    @property
    def t_phalanx_cross_section(self) -> tuple[PointXZ, ...]:
        """Return the fundamental T-shaped phalanx XZ polygon.

        The polygon is derived from the phalanx and pad dimensions.  It does
        not include the fixed stepped bonding interface.
        """

        phalanx = self.parameters.phalanx
        stem_half_width_mm = phalanx.stem_width_mm / 2.0
        return (
            (-self.a, 0.0),
            (-stem_half_width_mm, 0.0),
            (-stem_half_width_mm, -phalanx.stem_height_mm),
            (stem_half_width_mm, -phalanx.stem_height_mm),
            (stem_half_width_mm, 0.0),
            (self.a, 0.0),
            (self.a, phalanx.carrier_height_mm),
            (-self.a, phalanx.carrier_height_mm),
        )

    @property
    def carrier_cross_section(self) -> tuple[PointXZ, ...]:
        """Return the physical stepped carrier XZ polygon.

        The twelve vertices are derived from the semantic T-phalanx and
        mounting-interface dimensions; they are not independent parameters.
        """

        phalanx = self.parameters.phalanx
        mounting = self.parameters.mounting_interface
        bond_width_mm = mounting.bond_width_mm
        bond_height_mm = mounting.bond_height_mm
        stem_half_width_mm = phalanx.stem_width_mm / 2.0
        a_mm = self.a
        return (
            (-a_mm, bond_height_mm),
            (-a_mm + bond_width_mm, bond_height_mm),
            (-a_mm + bond_width_mm, 0.0),
            (-stem_half_width_mm, 0.0),
            (-stem_half_width_mm, -phalanx.stem_height_mm),
            (stem_half_width_mm, -phalanx.stem_height_mm),
            (stem_half_width_mm, 0.0),
            (a_mm - bond_width_mm, 0.0),
            (a_mm - bond_width_mm, bond_height_mm),
            (a_mm, bond_height_mm),
            (a_mm, phalanx.carrier_height_mm),
            (-a_mm, phalanx.carrier_height_mm),
        )

    def _validate_y(self, y_mm: float) -> float:
        y_mm = _require_finite_query("y_mm", y_mm)
        if y_mm < -_QUERY_TOLERANCE_MM or y_mm > self.total_length_mm + _QUERY_TOLERANCE_MM:
            raise ValueError(
                f"y_mm={y_mm} is outside the fingertip domain "
                f"[0, {self.total_length_mm}] mm"
            )
        if abs(y_mm) <= _QUERY_TOLERANCE_MM:
            return 0.0
        if abs(y_mm - self.total_length_mm) <= _QUERY_TOLERANCE_MM:
            return self.total_length_mm
        return y_mm

    @staticmethod
    def _validate_x_in_footprint(x_mm: float, half_width_mm: float) -> None:
        if abs(x_mm) > half_width_mm + _QUERY_TOLERANCE_MM:
            raise ValueError(
                f"x_mm={x_mm} is outside the pad footprint with half-width "
                f"{half_width_mm} mm"
            )


__all__ = ["Fingertip"]
