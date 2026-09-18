"""Lightweight ROBOTIS XM335-T323-T actuator geometry.

This module is a layout/collision abstraction, not a replacement for the vendor
CAD.  The dimensions are taken from the ROBOTIS XM335 drawing/specification:
19 x 35 x 22 mm overall envelope and a 16 mm diameter output disk.

Canonical XM335 frame
---------------------
The frame origin lies on the output rotation axis at the mid-depth plane of the
simplified overall envelope.  +Z is the output rotation axis pointing toward
the horn/front side, +X spans the 19 mm actuator width, and +Y points upward in
the front view.

The drawing places the 16 mm diameter output disk tangent to the 35 mm overall
top envelope.  Therefore the output axis is 8 mm below the top and 27 mm above
the bottom.  The simplified keep-out box is intentionally conservative around
the rounded upper housing.
"""

from __future__ import annotations

from dataclasses import dataclass

from .actuator import BoxEnvelope, RevoluteActuatorGeometry, Vector3, _finite_real


@dataclass(frozen=True)
class XM335Actuator:
    """ROBOTIS XM335-T323-T hardware abstraction for hand layout."""

    clearance_mm: float = 0.0

    MODEL_NAME = "XM335-T323-T"
    WIDTH_MM = 19.0
    HEIGHT_MM = 35.0
    DEPTH_MM = 22.0
    OUTPUT_DISK_DIAMETER_MM = 16.0
    MASS_G = 27.0

    # The output disk is tangent to the top of the stated overall height.
    AXIS_TO_TOP_MM = OUTPUT_DISK_DIAMETER_MM / 2.0
    AXIS_TO_BOTTOM_MM = HEIGHT_MM - AXIS_TO_TOP_MM

    # +Z is the revolute/output axis.
    JOINT_AXIS: Vector3 = (0.0, 0.0, 1.0)

    def __post_init__(self) -> None:
        clearance = _finite_real("clearance_mm", self.clearance_mm)
        if clearance < 0.0:
            raise ValueError("clearance_mm must be non-negative")
        object.__setattr__(self, "clearance_mm", clearance)

    @property
    def housing_center_mm(self) -> Vector3:
        """Center of the simplified overall box in the XM335 joint frame."""

        center_y = (self.AXIS_TO_TOP_MM - self.AXIS_TO_BOTTOM_MM) / 2.0
        return (0.0, center_y, 0.0)

    @property
    def nominal_housing_box(self) -> BoxEnvelope:
        """Conservative box for the vendor-specified 19 x 35 x 22 mm envelope."""

        return BoxEnvelope(
            size_mm=(self.WIDTH_MM, self.HEIGHT_MM, self.DEPTH_MM),
            center_mm=self.housing_center_mm,
        )

    @property
    def housing_box(self) -> BoxEnvelope:
        """Collision/clearance box including the requested uniform margin."""

        return self.nominal_housing_box.expanded(self.clearance_mm)

    @property
    def geometry(self) -> RevoluteActuatorGeometry:
        """Return the generic solver-neutral actuator representation."""

        return RevoluteActuatorGeometry(
            model_name=self.MODEL_NAME,
            joint_axis=self.JOINT_AXIS,
            keepout_boxes=(self.housing_box,),
            mass_g=self.MASS_G,
        )


__all__ = ["XM335Actuator"]
