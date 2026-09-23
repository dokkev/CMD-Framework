"""Minimal ROBOTIS XM335 actuator parameters."""

from __future__ import annotations

from dataclasses import dataclass, field

from .actuator import (
    Actuator,
    DShapedExtrusion,
    MechanicalParameters,
)


@dataclass(frozen=True)
class XM335Actuator(Actuator):
    """XM335 physical representation for CLEAR Hand layout.

    The dimensions are project parameters, not a collision or simulator
    geometry model.
    """

    geometry: DShapedExtrusion = field(
        default_factory=lambda: DShapedExtrusion(
            radius_mm=9.5,
            rear_extent_mm=25.5,
            depth_mm=22.0,
        )
    )
    mechanics: MechanicalParameters = field(
        default_factory=lambda: MechanicalParameters(
            gear_ratio=323.04,
            max_output_torque_nm=1.0,
            mass_g=27.0,
        )
    )


__all__ = ["XM335Actuator"]
