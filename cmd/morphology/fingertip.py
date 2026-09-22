"""Analytic geometry for the concrete contact-morphology fingertip.

The canonical frame is ``X`` lateral, ``Y`` longitudinal, and ``Z`` dorsal;
the finger extends toward positive ``Y``.  The straight body occupies
``0 <= y <= L`` and the distal nose occupies ``L <= y <= L + b``.  All public
dimensions and query coordinates are in millimetres.

The compliant pad keeps a constant lateral width along the finger. Its
straight body has a transverse semielliptic lower surface,

``z = -h - c * sqrt(1 - (x/a)**2)``.

The distal section is a rounded longitudinal cap. For each lateral section,
the lower and upper surfaces converge at the distal end instead of shrinking
the pad width to zero. This matches the physical U-shaped pad and rounded
distal nose used by the prototype.

The compliant outer envelope is represented separately from the rigid carrier:
at a footprint point it spans between analytic lower and upper surfaces. The
carrier has a plain T-shaped reference XZ section without bonding steps.
The stem extends by ``carrier_extension_mm`` beyond the straight body. The
dorsal plate continues to the full fingertip length with an elliptical nose,
forming the fingernail.
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
        """Half-width of the constant-width compliant pad."""

        return self.parameters.pad.width_mm / 2.0

    @property
    def total_length_mm(self) -> float:
        """Longitudinal length from the body root to the distal tip."""

        pad = self.parameters.pad
        return pad.body_length_mm + pad.tip_length_mm

    @property
    def carrier_length_mm(self) -> float:
        """Length of the carrier stem, excluding the longer dorsal nail."""

        pad = self.parameters.pad
        return pad.body_length_mm + self.parameters.phalanx.carrier_extension_mm

    @property
    def dorsal_plate_length_mm(self) -> float:
        """Length of the dorsal plate interface, through the complete nose."""

        return self.total_length_mm

    def dorsal_plate_half_width_at_y_mm(self, y_mm: float) -> float:
        """Return the nail half-width: straight body followed by an ellipse."""

        y_mm = self._validate_y(y_mm)
        pad = self.parameters.pad
        if y_mm <= pad.body_length_mm:
            return self.a
        v = (y_mm - pad.body_length_mm) / pad.tip_length_mm
        return self.a * math.sqrt(max(0.0, 1.0 - v * v))

    @property
    def a(self) -> float:
        """Transverse semi-axis ``a`` of the pad, in mm."""

        return self.half_width_mm

    @property
    def b(self) -> float:
        """Longitudinal length ``b`` of the rounded distal cap, in mm."""

        return self.parameters.pad.tip_length_mm

    @property
    def c(self) -> float:
        """Maximum lower-surface depth ``c`` below the pad shoulder, in mm."""

        return self.parameters.pad.ellipse_height_mm

    def half_width_at_y_mm(self, y_mm: float) -> float:
        """Return the constant pad half-width at longitudinal coordinate ``y``."""

        self._validate_y(y_mm)
        return self.a

    def lower_surface_z_mm(self, x_mm: float, y_mm: float) -> float:
        """Return the analytic lower contact surface at ``(x, y)``.

        ``ValueError`` is raised when the query is outside the longitudinal
        domain or outside the available cross-sectional footprint.
        """

        x_mm = _require_finite_query("x_mm", x_mm)
        y_mm = self._validate_y(y_mm)
        self._validate_x_in_footprint(x_mm, self.a)

        pad = self.parameters.pad
        if y_mm <= pad.body_length_mm:
            return self._body_lower_surface_z_mm(x_mm)
        return self._distal_cap_bounds_z_mm(x_mm, y_mm)[0]

    def upper_surface_z_mm(self, x_mm: float, y_mm: float) -> float:
        """Return the analytic upper surface of the compliant pad in mm.

        The straight body meets the carrier shoulder at ``z = 0``. The
        rounded distal cap lowers this upper surface until it meets the lower
        surface at the end of the pad.
        """

        x_mm = _require_finite_query("x_mm", x_mm)
        y_mm = self._validate_y(y_mm)
        self._validate_x_in_footprint(x_mm, self.a)

        if y_mm <= self.parameters.pad.body_length_mm:
            return 0.0
        return self._distal_cap_bounds_z_mm(x_mm, y_mm)[1]

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

        The dorsal carrier is returned separately through its carrier
        cross-section rather than folded into the contact envelope.
        """

        return self.lower_surface_z_mm(x_mm, y_mm), self.upper_surface_z_mm(x_mm, y_mm)

    @property
    def carrier_cross_section(self) -> tuple[PointXZ, ...]:
        """Return the plain T-shaped carrier section in the straight body.

        Distally the nail narrows and continues beyond the end of the stem;
        this reference section must not be extruded through the entire nail.
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

    def _body_lower_surface_z_mm(self, x_mm: float) -> float:
        pad = self.parameters.pad
        normalized_x = x_mm / self.a
        return -pad.pad_straight_height_mm - self.c * math.sqrt(
            max(0.0, 1.0 - normalized_x * normalized_x)
        )

    def _distal_cap_bounds_z_mm(
        self,
        x_mm: float,
        y_mm: float,
    ) -> tuple[float, float]:
        """Return lower/upper z bounds for one rounded-cap section."""

        pad = self.parameters.pad
        lower_body_z_mm = self._body_lower_surface_z_mm(x_mm)
        center_z_mm = lower_body_z_mm / 2.0
        half_thickness_mm = -lower_body_z_mm / 2.0
        normalized_y = (y_mm - pad.body_length_mm) / pad.tip_length_mm
        cap_scale = math.sqrt(max(0.0, 1.0 - normalized_y * normalized_y))
        return (
            center_z_mm - half_thickness_mm * cap_scale,
            center_z_mm + half_thickness_mm * cap_scale,
        )

    @staticmethod
    def _validate_x_in_footprint(x_mm: float, half_width_mm: float) -> None:
        if abs(x_mm) > half_width_mm + _QUERY_TOLERANCE_MM:
            raise ValueError(
                f"x_mm={x_mm} is outside the pad footprint with half-width "
                f"{half_width_mm} mm"
            )


__all__ = ["Fingertip"]
