"""Analytic geometry for the concrete contact-morphology fingertip.

The fingertip frame is right-handed and shares the hand and linkage axis
convention: ``+X`` distal (longitudinal), ``+Y`` lateral (parallel to the
flexion axis), and ``+Z`` dorsal, so the contact pad faces ``-Z``. The origin
is the proximal end of the carrier on its centreline, at the carrier shoulder
plane ``z = 0``. The straight body occupies ``0 <= x <= L`` and the distal
nose occupies ``L <= x <= L + b``. The shape is symmetric in ``y``. All public
dimensions and query coordinates are in millimetres.

The compliant pad has a constant ``vertical_pad_width_mm`` through the
straight body. Its lower half-ellipse uses ``elliptical_pad_width_mm`` and
``elliptical_pad_height_mm``,

``z = -h - c * sqrt(1 - (y/a)**2)``.

The distal section is the lower half of a longitudinal ellipsoid joined to
the straight body. Its transverse semi-axis is the derived tip half-width
from ``carrier_width_mm``. Its lower surface is,

``z = -h - c * sqrt(1 - (y/a)**2 - ((x-L)/b)**2)``.

The compliant pad's upper surface remains at the carrier shoulder plane
``z = 0``. This produces the rectangular pad wall above the curved half-
ellipsoid shown in the CAD front/back views.

The compliant outer envelope is represented separately from the rigid carrier:
at a footprint point it spans between analytic lower and upper surfaces. The
carrier has a plain T-shaped reference YZ section without bonding steps.
The rigid carrier length is derived from the concrete fingertip total length
and the family-wide distal inset. The dorsal plate continues to the full
fingertip length with an elliptical nose, forming the fingernail.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import FingertipParameters
from ..._validation import require_finite


PointYZ = tuple[float, float]
_QUERY_TOLERANCE_MM = 1e-12


@dataclass(frozen=True)
class Fingertip:
    """Constructed analytic geometry for one parameterized fingertip."""

    parameters: FingertipParameters = field(default_factory=FingertipParameters)

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, FingertipParameters):
            raise TypeError("parameters must be a FingertipParameters instance")

    @property
    def half_width_mm(self) -> float:
        """Half-width of the straight vertical pad."""

        return self.parameters.family.pad.vertical_pad_width_mm / 2.0

    @property
    def elliptical_pad_half_width_mm(self) -> float:
        """Half-width of the compliant lower ellipse, in mm."""

        return self.parameters.family.pad.elliptical_pad_width_mm / 2.0

    @property
    def total_length_mm(self) -> float:
        """Per-digit total fingertip length, in mm."""

        return self.parameters.total_length_mm

    @property
    def straight_length_mm(self) -> float:
        """Length before the distal tip begins, in mm."""

        return self.total_length_mm - self.tip_length_mm

    @property
    def carrier_length_mm(self) -> float:
        """Total longitudinal length of the rigid carrier stem."""

        return (
            self.total_length_mm
            - self.parameters.family.carrier_distal_inset_mm
        )

    @property
    def dorsal_plate_length_mm(self) -> float:
        """Length of the dorsal plate interface, through the complete nose."""

        return self.total_length_mm

    @property
    def tip_length_mm(self) -> float:
        """Longitudinal semi-axis of the derived distal tip, in mm."""

        return self.parameters.family.pad.tip_length_mm

    @property
    def tip_half_width_mm(self) -> float:
        """Derived transverse semi-axis from the carrier width, in mm."""

        return self.parameters.family.phalanx.carrier_width_mm / 2.0

    @property
    def tip_height_mm(self) -> float:
        """Derived vertical semi-axis from the elliptical pad height, in mm."""

        return self.parameters.family.pad.elliptical_pad_height_mm

    def dorsal_plate_half_width_at_x_mm(self, x_mm: float) -> float:
        """Return the dorsal plate half-width at longitudinal coordinate ``x``."""

        x_mm = self._validate_x(x_mm)
        if x_mm <= self.straight_length_mm:
            return self.tip_half_width_mm

        normalized_x = (x_mm - self.straight_length_mm) / self.tip_length_mm
        return self.tip_half_width_mm * math.sqrt(
            max(0.0, 1.0 - normalized_x * normalized_x)
        )

    @property
    def a(self) -> float:
        """Transverse semi-axis ``a`` of the derived tip, in mm."""

        return self.tip_half_width_mm

    @property
    def b(self) -> float:
        """Longitudinal semi-axis ``b`` of the distal ellipsoid, in mm."""

        return self.tip_length_mm

    @property
    def c(self) -> float:
        """Vertical semi-axis ``c`` of the derived tip, in mm."""

        return self.tip_height_mm

    def half_width_at_x_mm(self, x_mm: float) -> float:
        """Return the pad half-width at longitudinal coordinate ``x``.

        The straight body has the vertical pad width. The distal tip uses
        the carrier-derived tip half-width as its planform semi-axis.
        """

        x_mm = self._validate_x(x_mm)
        if x_mm <= self.straight_length_mm:
            return self.half_width_mm

        normalized_x = (x_mm - self.straight_length_mm) / self.tip_length_mm
        return self.tip_half_width_mm * math.sqrt(
            max(0.0, 1.0 - normalized_x * normalized_x)
        )

    def lower_surface_z_mm(self, x_mm: float, y_mm: float) -> float:
        """Return the analytic lower contact surface at ``(x, y)``.

        ``ValueError`` is raised when the query is outside the longitudinal
        domain or outside the available cross-sectional footprint.
        """

        x_mm = self._validate_x(x_mm)
        y_mm = require_finite("y_mm", y_mm)
        self._validate_y_in_footprint(y_mm, self.half_width_at_x_mm(x_mm))

        pad = self.parameters.family.pad
        if x_mm <= self.straight_length_mm:
            return self._body_lower_surface_z_mm(y_mm)
        normalized_x = (x_mm - self.straight_length_mm) / self.tip_length_mm
        normalized_y = y_mm / self.tip_half_width_mm
        ellipsoid_radicand = max(
            0.0,
            1.0 - normalized_y * normalized_y - normalized_x * normalized_x,
        )
        return -pad.vertical_pad_height_mm - self.tip_height_mm * math.sqrt(
            ellipsoid_radicand
        )

    def upper_surface_z_mm(self, x_mm: float, y_mm: float) -> float:
        """Return the analytic upper surface of the compliant pad in mm.

        The straight body and the half-ellipsoid meet the carrier shoulder at
        ``z = 0``. The curved distal surface only changes the lower boundary.
        """

        x_mm = self._validate_x(x_mm)
        y_mm = require_finite("y_mm", y_mm)
        self._validate_y_in_footprint(y_mm, self.half_width_at_x_mm(x_mm))
        return 0.0

    def is_inside_pad_footprint(self, x_mm: float, y_mm: float) -> bool:
        """Return whether ``(x, y)`` belongs to the closed pad footprint.

        Longitudinal coordinates outside the fingertip domain are invalid
        queries and raise ``ValueError``.  A lateral coordinate outside the
        footprint simply produces ``False``.
        """

        x_mm = self._validate_x(x_mm)
        y_mm = require_finite("y_mm", y_mm)
        return abs(y_mm) <= self.half_width_at_x_mm(x_mm) + _QUERY_TOLERANCE_MM

    def outer_envelope_z_bounds_mm(self, x_mm: float, y_mm: float) -> tuple[float, float]:
        """Return ``(lower_z, upper_z)`` for the compliant outer envelope.

        The dorsal carrier is returned separately through its carrier
        cross-section rather than folded into the contact envelope.
        """

        return self.lower_surface_z_mm(x_mm, y_mm), self.upper_surface_z_mm(x_mm, y_mm)

    @property
    def carrier_cross_section(self) -> tuple[PointYZ, ...]:
        """Return the plain T-shaped ``(y, z)`` carrier section in the straight body.

        Distally the nail narrows and continues beyond the end of the stem;
        this reference section must not be extruded through the entire nail.
        """

        phalanx = self.parameters.family.phalanx
        stem_half_width_mm = phalanx.stem_width_mm / 2.0
        return (
            (-self.tip_half_width_mm, 0.0),
            (-stem_half_width_mm, 0.0),
            (-stem_half_width_mm, -phalanx.stem_height_mm),
            (stem_half_width_mm, -phalanx.stem_height_mm),
            (stem_half_width_mm, 0.0),
            (self.tip_half_width_mm, 0.0),
            (self.tip_half_width_mm, phalanx.carrier_height_mm),
            (-self.tip_half_width_mm, phalanx.carrier_height_mm),
        )

    def _validate_x(self, x_mm: float) -> float:
        x_mm = require_finite("x_mm", x_mm)
        if x_mm < -_QUERY_TOLERANCE_MM or x_mm > self.total_length_mm + _QUERY_TOLERANCE_MM:
            raise ValueError(
                f"x_mm={x_mm} is outside the fingertip domain "
                f"[0, {self.total_length_mm}] mm"
            )
        if abs(x_mm) <= _QUERY_TOLERANCE_MM:
            return 0.0
        if abs(x_mm - self.total_length_mm) <= _QUERY_TOLERANCE_MM:
            return self.total_length_mm
        return x_mm

    def _body_lower_surface_z_mm(self, y_mm: float) -> float:
        pad = self.parameters.family.pad
        if abs(y_mm) > self.elliptical_pad_half_width_mm:
            return -pad.vertical_pad_height_mm

        normalized_y = y_mm / self.elliptical_pad_half_width_mm
        return -pad.vertical_pad_height_mm - self.tip_height_mm * math.sqrt(
            max(0.0, 1.0 - normalized_y * normalized_y)
        )

    @staticmethod
    def _validate_y_in_footprint(y_mm: float, half_width_mm: float) -> None:
        if abs(y_mm) > half_width_mm + _QUERY_TOLERANCE_MM:
            raise ValueError(
                f"y_mm={y_mm} is outside the pad footprint with half-width "
                f"{half_width_mm} mm"
            )


__all__ = ["Fingertip"]
