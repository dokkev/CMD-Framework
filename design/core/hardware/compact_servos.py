"""Compact smart servos of the XL330/XC330 frame (about 20 x 34 x 26 mm), as
packaging bodies beside the XM335 (``core.hardware.xm335``).

Each body is a box (``BoxExtrusion``), not the XM335's D shape: the 20 x 34
mm face carries the horn, its shaft centred across the width and 9.5 mm
behind the near end face (ROBOTIS's XL/XC330 drawing, X330, 28-May-20:
https://www.dropbox.com/s/gxgye7wt5sbt4i4/XL,XC-330.pdf, linked from the
e-manual); ``depth`` is the case thickness along the shaft, the horn left
out. The Feetech and Unitree bodies share the frame (sold as XL330 drop-in
replacements); their shaft position is the X330 drawing's, assumed, not
read from their own drawings. Values read 2026-11-19:

- ROBOTIS DYNAMIXEL XC330-T288-T: 20 x 34 x 26 mm (23 mm case, 3 mm horn), 23 g, gear 288.35:1, stall
  0.92 N m at 11.1 V (1.00 N m at 12 V). ROBOTIS e-manual X-series table
  (https://emanual.robotis.com/docs/en/dxl/x/xc330-m181), robotis.us.
- Feetech HD-1910-C001 (coreless, dual shaft): 34 x 20 x 23 mm, about 21 g,
  12 kg cm (1.18 N m) at 6 V; gear ratio not published. Feetech product list
  (https://www.feetechrc.com/products/serial-port-series-steering-gear-page-7),
  armdojo.com. A retail figure, to be checked against Feetech's datasheet.
- Unitree YS-342026-J288 brushless digital servo: 20 x 34 x 26 mm, 35 g,
  gear 288.35:1, stall 0.5 N m driving (1.5 N m held against an external
  load). Unitree's digital_servo repository
  (https://github.com/unitreerobotics/digital_servo), servodatabase.com and
  retail listings; Unitree's own datasheet not seen.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from core.hardware.actuator import Actuator, BoxExtrusion, MechanicalParameters

SHAFT_FROM_FRONT_MM = 9.5
"""The X330 drawing's shaft centre behind the near end face."""


def _body(depth_mm: float) -> BoxExtrusion:
    return BoxExtrusion(width_mm=20.0, length_mm=34.0, shaft_from_front_mm=SHAFT_FROM_FRONT_MM, depth_mm=depth_mm)


@dataclass(frozen=True)
class XC330Actuator(Actuator):
    """ROBOTIS DYNAMIXEL XC330-T288-T."""

    geometry: BoxExtrusion = field(default_factory=lambda: _body(23.0))
    mechanics: MechanicalParameters = field(
        default_factory=lambda: MechanicalParameters(gear_ratio=288.35, max_output_torque_nm=0.92, mass_g=23.0))


@dataclass(frozen=True)
class FeetechHD1910Actuator(Actuator):
    """Feetech HD-1910-C001 (coreless serial bus servo)."""

    geometry: BoxExtrusion = field(default_factory=lambda: _body(23.0))
    mechanics: MechanicalParameters = field(
        default_factory=lambda: MechanicalParameters(gear_ratio=None, max_output_torque_nm=1.18, mass_g=21.0))


@dataclass(frozen=True)
class UnitreeJ288Actuator(Actuator):
    """Unitree YS-342026-J288 brushless digital servo (the driving stall torque)."""

    geometry: BoxExtrusion = field(default_factory=lambda: _body(26.0))
    mechanics: MechanicalParameters = field(
        default_factory=lambda: MechanicalParameters(gear_ratio=288.35, max_output_torque_nm=0.5, mass_g=35.0))


__all__ = ["SHAFT_FROM_FRONT_MM", "FeetechHD1910Actuator", "UnitreeJ288Actuator", "XC330Actuator"]
