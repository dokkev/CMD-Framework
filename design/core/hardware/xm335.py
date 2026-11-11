"""ROBOTIS XM335: the body used for packaging, and the servo's datasheet and drive model.

``XM335Actuator`` is the body the planar packaging checks use (project parameters, not a vendor drawing).

``XM335Datasheet`` holds the vendor values of the XM335-T323-T, from the ROBOTIS e-manual
(https://emanual.robotis.com/docs/en/dxl/x/xm335-t323/; source ``ROBOTIS-GIT/emanual``: the spec table is
``_data/dxl_x_info.yml`` at 4077642189c6, the control table ``docs/en/dxl/x/xm335-t323.md`` at 577b9250ea60,
read 2026-11-11). ``XM335DriveModel`` turns them into a joint-side drive law for simulation:

    u   = clip(K_PP * e_pulse, -PWM_limit, PWM_limit) / 885       duty in [-1, 1]
    tau = tau_stall(V) * (u - qd / w_noload(V))                    coreless DC motor behind the gearbox

The second line is the linear torque-speed (N-T) curve of a DC motor driven at duty ``u``: back-EMF damping acts
at every speed, not only at saturation. The vendor's three voltage points are linear in voltage to within 2 %
(``tau_stall/V`` 0.094, 0.093, 0.093 N m/V; ``w_noload/V`` 4.78, 4.77, 4.75 rpm/V), which supports that model.

Assumptions, each to be identified on the hardware before dynamics are interpreted:
- The position PID's units: error in encoder pulses, output in Goal PWM units (885 = 100 %), with
  ``K_PP = KPP(TBL) / 128`` as the manual states. The manual gives the conversion but not the units, so the
  resulting stiffness (about 12 N m/rad at the default gain, saturating at a 5 deg error) is model-derived.
- No reflected rotor inertia is published. ``armature_kg_m2`` is a placeholder.
- No friction or backlash is published (backlash "N/A"). Both are left out.
- The listed N m/A ratios use the current measured at the supply input, not the motor phase current (the
  manual's Current Limit note), so they are not a motor torque constant.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from itertools import pairwise

from core.hardware.actuator import (
    Actuator,
    DShapedExtrusion,
    MechanicalParameters,
)

RPM = 2.0 * math.pi / 60.0


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


@dataclass(frozen=True)
class XM335Datasheet:
    """Vendor values of the XM335-T323-T (see the module docstring for the source)."""

    model: str = "XM335-T323-T"
    gear_ratio: float = 323.04
    mass_g: float = 27.0
    dimensions_mm: tuple[float, float, float] = (19.0, 35.0, 22.0)
    voltage_range_v: tuple[float, float] = (6.5, 12.0)
    recommended_voltage_v: float = 11.1
    rated_voltages_v: tuple[float, ...] = (9.0, 11.1, 12.0)
    stall_torque_nm: tuple[float, ...] = (0.85, 1.03, 1.12)
    stall_current_a: tuple[float, ...] = (0.61, 0.80, 0.88)
    no_load_speed_rpm: tuple[float, ...] = (43.0, 53.0, 57.0)
    standby_current_a: float = 0.017
    encoder_pulses_per_rev: int = 4096
    # Control table (defaults and units)
    pwm_limit_lsb: int = 885
    pwm_lsb_percent: float = 0.113
    velocity_limit_lsb: int = 280
    velocity_lsb_rpm: float = 0.229
    position_p_gain_tbl: int = 2000
    position_i_gain_tbl: int = 0
    position_d_gain_tbl: int = 0
    feedforward_1st_gain_tbl: int = 0
    feedforward_2nd_gain_tbl: int = 0
    min_position_limit_pulse: int = 1024
    max_position_limit_pulse: int = 3072

    def _at(self, values: tuple[float, ...], voltage_v: float) -> float:
        low, high = self.voltage_range_v
        if not low <= voltage_v <= high:
            raise ValueError(f"supply voltage {voltage_v} V is outside the rated {low}-{high} V")
        # Piecewise linear through the rated points (exact there); below the lowest, the first segment's line.
        points = list(zip(self.rated_voltages_v, values))
        for (v0, y0), (v1, y1) in pairwise(points):
            if voltage_v <= v1 or (v1, y1) == points[-1]:
                return y0 + (y1 - y0) * (voltage_v - v0) / (v1 - v0)
        raise AssertionError("unreachable")

    def stall_torque_at(self, voltage_v: float) -> float:
        """Stall torque [N m] at the output shaft."""
        return self._at(self.stall_torque_nm, voltage_v)

    def no_load_speed_at(self, voltage_v: float) -> float:
        """No-load speed [rad/s] at the output shaft."""
        return self._at(self.no_load_speed_rpm, voltage_v) * RPM

    @property
    def rad_per_pulse(self) -> float:
        return 2.0 * math.pi / self.encoder_pulses_per_rev


@dataclass(frozen=True)
class XM335DriveModel:
    """The joint-side drive law of one XM335 in Position Control Mode (see the module docstring).

    ``position_p_gain_tbl`` is the control-table value (default 2000); I, D and feedforward stay at their
    defaults (0), and Profile Velocity/Acceleration at 0 (a step goal).
    """

    datasheet: XM335Datasheet = field(default_factory=XM335Datasheet)
    supply_voltage_v: float = 11.1
    position_p_gain_tbl: int = 2000
    pwm_limit_lsb: int = 885
    armature_kg_m2: float = 5e-3
    """Reflected rotor inertia at the output [kg m^2]: a placeholder, not published (identify from a no-load step:
    the speed time constant is ``armature * no_load_speed / stall_torque``, about 27 ms at this value)."""

    @property
    def stall_torque_nm(self) -> float:
        return self.datasheet.stall_torque_at(self.supply_voltage_v)

    @property
    def no_load_speed_rad_s(self) -> float:
        return self.datasheet.no_load_speed_at(self.supply_voltage_v)

    @property
    def duty_per_rad(self) -> float:
        """Duty per radian of position error, below saturation (assumed units: pulse in, PWM LSB out)."""
        return self.position_p_gain_tbl / 128.0 / self.datasheet.rad_per_pulse / self.datasheet.pwm_limit_lsb

    @property
    def duty_limit(self) -> float:
        return self.pwm_limit_lsb / self.datasheet.pwm_limit_lsb

    @property
    def stiffness_nm_per_rad(self) -> float:
        """Small-error position stiffness: ``tau_stall * duty_per_rad``."""
        return self.stall_torque_nm * self.duty_per_rad

    @property
    def back_emf_damping_nms_per_rad(self) -> float:
        """Velocity damping of the N-T curve: ``tau_stall / w_noload``."""
        return self.stall_torque_nm / self.no_load_speed_rad_s

    @property
    def saturation_error_rad(self) -> float:
        """The position error at which the duty reaches its limit."""
        return self.duty_limit / self.duty_per_rad

    def torque(self, error_rad: float, velocity_rad_s: float) -> float:
        """Output torque [N m] at a position error (goal - present) and output velocity."""
        duty = max(-self.duty_limit, min(self.duty_limit, self.duty_per_rad * error_rad))
        return self.stall_torque_nm * (duty - velocity_rad_s / self.no_load_speed_rad_s)

    def as_dict(self) -> dict:
        """The values a simulator needs (SI units), with the assumptions they rest on."""
        d = self.datasheet
        return {
            "model": d.model,
            "source": "ROBOTIS e-manual xm335-t323 (emanual@4077642189c6 _data/dxl_x_info.yml, "
                      "@577b9250ea60 docs/en/dxl/x/xm335-t323.md), read 2026-11-11",
            "supply_voltage_v": self.supply_voltage_v,
            "stall_torque_nm": self.stall_torque_nm,
            "no_load_speed_rad_s": self.no_load_speed_rad_s,
            "duty_per_rad": self.duty_per_rad,
            "duty_limit": self.duty_limit,
            "armature_kg_m2": self.armature_kg_m2,
            "encoder_rad_per_pulse": d.rad_per_pulse,
            "position_p_gain_tbl": self.position_p_gain_tbl,
            "derived": {"stiffness_nm_per_rad": self.stiffness_nm_per_rad,
                        "back_emf_damping_nms_per_rad": self.back_emf_damping_nms_per_rad,
                        "saturation_error_rad": self.saturation_error_rad},
            "assumptions": ["PID units: pulse in, PWM LSB out", "armature is a placeholder",
                            "no friction or backlash", "position I, D and feedforward gains 0, step goals"],
        }


__all__ = ["XM335Actuator", "XM335Datasheet", "XM335DriveModel"]
