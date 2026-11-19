"""Compact smart servos of the XL330/XC330 frame (about 20 x 34 x 26 mm), as
packaging bodies beside the XM335 (``core.hardware.xm335``).

Each body is the D-shaped extrusion every actuator uses: the output shaft at
the centre of the rounded end, so ``radius = width / 2`` and
``rear_extent = length - width / 2``; ``depth`` is the case thickness along
the shaft. The shaft position is assumed, as for the XM335 model, not taken
from a vendor drawing. Values read 2026-11-19:

- ROBOTIS DYNAMIXEL XC330-T288-T: 20 x 34 x 26 mm, 23 g, gear 288.35:1, stall
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

from core.hardware.actuator import Actuator, DShapedExtrusion, MechanicalParameters


def _body(width_mm: float, length_mm: float, depth_mm: float) -> DShapedExtrusion:
    return DShapedExtrusion(radius_mm=width_mm / 2.0, rear_extent_mm=length_mm - width_mm / 2.0, depth_mm=depth_mm)


@dataclass(frozen=True)
class XC330Actuator(Actuator):
    """ROBOTIS DYNAMIXEL XC330-T288-T."""

    geometry: DShapedExtrusion = field(default_factory=lambda: _body(20.0, 34.0, 26.0))
    mechanics: MechanicalParameters = field(
        default_factory=lambda: MechanicalParameters(gear_ratio=288.35, max_output_torque_nm=0.92, mass_g=23.0))


@dataclass(frozen=True)
class FeetechHD1910Actuator(Actuator):
    """Feetech HD-1910-C001 (coreless serial bus servo)."""

    geometry: DShapedExtrusion = field(default_factory=lambda: _body(20.0, 34.0, 23.0))
    mechanics: MechanicalParameters = field(
        default_factory=lambda: MechanicalParameters(gear_ratio=None, max_output_torque_nm=1.18, mass_g=21.0))


@dataclass(frozen=True)
class UnitreeJ288Actuator(Actuator):
    """Unitree YS-342026-J288 brushless digital servo (the driving stall torque)."""

    geometry: DShapedExtrusion = field(default_factory=lambda: _body(20.0, 34.0, 26.0))
    mechanics: MechanicalParameters = field(
        default_factory=lambda: MechanicalParameters(gear_ratio=288.35, max_output_torque_nm=0.5, mass_g=35.0))


__all__ = ["FeetechHD1910Actuator", "UnitreeJ288Actuator", "XC330Actuator"]
