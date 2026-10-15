"""Configuration of the 2-DoF finger linkage study."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType

from cmd._validation import require_finite, require_nonnegative, require_positive

from cmd.morphology.linkage.packaging import Packaging

from .design import VARIABLE_NAMES


@dataclass(frozen=True)
class FingerStudyConfig:
    """Search space, requirements and NSGA-II settings.

    The workspace is the PIP range from extension to the curl pose, where the
    distal link has turned 180 deg relative to the proximal link (``BC``
    antiparallel to ``OA``). The evaluated range extends it by
    ``range_margin_rad`` at both ends, into hyperextension and past the curl,
    where the mechanical stoppers go.

    ``packaging`` holds the servo, bushing and clearance dimensions; swap it to
    study other hardware. ``bounds`` are the user's search box;
    :attr:`search_bounds` tightens them where a constraint has a closed form.
    """

    bounds: Mapping[str, tuple[float, float]]
    pip_step_rad: float = math.radians(1.0)
    pip_horizon_rad: float = math.pi
    range_margin_rad: float = math.radians(10.0)
    minimum_transmission_margin_rad: float = math.radians(30.0)
    dip_backoff_tolerance_rad: float = math.radians(0.1)
    packaging: Packaging = field(default_factory=Packaging)
    reference_point: str = "C"
    """Point of the distal link the length and curl-tip constraints use: ``"C"``
    (the tip; they then depend on ``BC``) or ``"B"`` (independent of ``BC``)."""
    extended_length_bounds_mm: tuple[float, float] = (90.0, 100.0)
    """Bounds on ``|O ref|`` (``|OC|`` or ``|OB|``) in the fully extended pose."""
    curl_gap_mm: float = 20.0
    """Least distance between the ``OA`` and ``BC`` lines at the curl pose."""
    curl_tip_min_x_mm: float = 0.0
    """Least ``x`` of the reference point at the curl pose with the MCP at 0: it
    stays ahead of ``O``."""
    mcp_flexion_max_rad: float = math.radians(90.0)
    """MCP flexion up to which the fingertip must clear the MCP servo."""
    link_clearance: bool = False
    """Require the straight middle link ``A-B'`` to clear the PIP servo at ``A'``
    in the plane. Off: its plates straddle the servo in other layers; the
    planar clearance is still reported."""
    objective: str = "torque_ratio"
    """First objective: ``"transmission"`` (max worst transmission quality),
    ``"lever"`` (min worst PIP servo torque per unit pad force) or
    ``"torque_ratio"`` (min worst servo torque per unit distal-link moment)."""
    minimize_ab: bool = False
    """Add ``|AB|`` as a second objective (a Pareto study); otherwise single-objective."""
    keep_candidates: int = 50
    """Single-objective runs keep this many best feasible designs of the final population."""
    servo_travel_max_rad: float | None = None
    """Most PIP servo rotation from extension to the curl pose; ``None``: unbounded."""
    oa_interface_clearance_mm: float = 0.0
    """Least distance of the PIP servo dorsal of the ``OA`` line, keeping the
    proximal contact interface free (a contact pad thickness, later)."""
    resolution_mm: float = 0.1
    population_size: int = 200
    generations: int = 200
    seed: int = 0

    def __post_init__(self) -> None:
        if set(self.bounds) != set(VARIABLE_NAMES):
            raise ValueError(f"bounds must cover exactly {VARIABLE_NAMES}")
        bounds = {}
        for name in VARIABLE_NAMES:
            lower, upper = (require_finite(name, value) for value in self.bounds[name])
            if lower > upper:
                raise ValueError(f"{name} lower bound exceeds its upper bound")
            bounds[name] = (lower, upper)
        object.__setattr__(self, "bounds", MappingProxyType(bounds))
        for name in ("pip_step_rad", "pip_horizon_rad",
                     "minimum_transmission_margin_rad", "resolution_mm"):
            require_positive(name, getattr(self, name))
        require_nonnegative("range_margin_rad", self.range_margin_rad)
        require_nonnegative("dip_backoff_tolerance_rad", self.dip_backoff_tolerance_rad)
        require_finite("curl_gap_mm", self.curl_gap_mm)
        require_finite("curl_tip_min_x_mm", self.curl_tip_min_x_mm)
        if self.reference_point not in ("C", "B"):
            raise ValueError("reference_point must be 'C' or 'B'")
        require_finite("oa_interface_clearance_mm", self.oa_interface_clearance_mm)
        if self.objective not in ("transmission", "lever", "torque_ratio"):
            raise ValueError("objective must be 'transmission', 'lever' or 'torque_ratio'")
        if self.servo_travel_max_rad is not None:
            require_positive("servo_travel_max_rad", self.servo_travel_max_rad)
        require_nonnegative("mcp_flexion_max_rad", self.mcp_flexion_max_rad)
        lower, upper = self.extended_length_bounds_mm
        if lower > upper:
            raise ValueError("extended_length_bounds_mm lower bound exceeds its upper bound")

    @property
    def search_bounds(self) -> Mapping[str, tuple[float, float]]:
        """``bounds`` tightened by constraints with a closed form.

        With the PIP servo tilted dorsally by 0-90 deg its most palmar point is
        the bottom of the shaft disk, ``A'_z - radius``, so the OA interface
        constraint is exactly ``A'_z >= radius + oa_interface_clearance_mm``.
        """

        bounds = dict(self.bounds)
        tilt_low, tilt_high = bounds["servo_tilt_deg"]
        if 0.0 <= tilt_low and tilt_high <= 90.0:
            low, high = bounds["ap_z_mm"]
            least = self.packaging.pip_servo.radius_mm + self.oa_interface_clearance_mm
            bounds["ap_z_mm"] = (min(max(low, least), high), high)
        return MappingProxyType(bounds)

    @property
    def lower(self) -> tuple[float, ...]:
        return tuple(self.search_bounds[name][0] for name in VARIABLE_NAMES)

    @property
    def upper(self) -> tuple[float, ...]:
        return tuple(self.search_bounds[name][1] for name in VARIABLE_NAMES)

    @property
    def pip_samples_rad(self) -> tuple[float, ...]:
        count = int(math.floor(self.pip_horizon_rad / self.pip_step_rad + 1e-9)) + 1
        return tuple(index * self.pip_step_rad for index in range(count))


__all__ = ["FingerStudyConfig"]
