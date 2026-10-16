"""Configuration of the 2-DoF finger linkage study."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field, fields
from pathlib import Path
from types import MappingProxyType

import yaml

from core._validation import require_finite, require_nonnegative, require_positive

from core.hardware import DShapedExtrusion
from core.morphology.linkage.packaging import Packaging

from core.optimization.stage0.design import VARIABLE_NAMES


@dataclass(frozen=True)
class FingerStudyConfig:
    """Search space, requirements and NSGA-II settings.

    The motion is swept by the PIP servo. The usable PIP range runs from
    ``-hyperextension_rad`` (the hyperextension stopper) to
    ``flexion_stopper_margin_rad`` before the first unsafe flexion pose, or to
    the curl pose (``BC`` antiparallel to ``OA``) if that comes first.

    ``packaging`` holds the servo, bearing and clearance dimensions; swap it to
    study other hardware. ``bounds`` are the user's search box;
    :attr:`search_bounds` tightens them where a constraint has a closed form.
    """

    bounds: Mapping[str, tuple[float, float]]
    servo_step_rad: float = math.radians(1.0)
    """PIP servo step of the motion sweep."""
    servo_horizon_rad: float = math.pi
    """Most PIP servo rotation swept from extension."""
    hyperextension_rad: float = math.radians(10.0)
    """PIP (and MCP) hyperextension the finger must reach safely: where the
    hyperextension stoppers go. The servos are checked to this MCP angle too."""
    flexion_stopper_margin_rad: float = math.radians(10.0)
    """PIP the usable range stops short of the first unsafe flexion pose: room
    past the flexion stopper."""
    minimum_transmission_margin_rad: float = math.radians(30.0)
    dip_backoff_tolerance_rad: float = math.radians(0.1)
    packaging: Packaging = field(default_factory=Packaging)
    reference_point: str = "C"
    """Point of the distal link the length and curl-tip constraints use: ``"C"``
    (the tip; they then depend on ``BC``) or ``"B"`` (independent of ``BC``)."""
    extended_length_bounds_mm: tuple[float, float] = (90.0, 100.0)
    """Bounds on ``|O ref|`` (``|OC|`` or ``|OB|``) in the fully extended pose."""
    enforce_curl_tip: bool = False
    enforce_fingertip_mcp: bool = False
    """Held Stage-0 constraints, reported but enforced only when enabled (Stage-1
    defines them with the contact interfaces)."""
    curl_tip_min_x_mm: float = 0.0
    """Least ``x`` of the reference point at the curl pose with the MCP at 0: it
    stays ahead of ``O``."""
    mcp_flexion_max_rad: float = math.radians(90.0)
    """MCP flexion up to which the fingertip must clear the MCP servo."""
    require_middle_link_servo_clearance: bool = False
    """Require the straight middle link ``A-B'`` to clear the PIP servo at ``A'``
    in the plane. Off: its plates straddle the servo in other layers; the
    planar clearance is still reported."""
    minimum_flexion_rad: float = math.radians(120.0)
    """Least ``phi_safe_max``: the distal link must turn at least this far, safely."""
    servo_travel_max_rad: float | None = None
    """Most PIP servo rotation from extension to the curl pose; ``None``: unbounded."""
    oa_interface_clearance_mm: float = 0.0
    """Least distance of the PIP servo dorsal of the ``OA`` line, keeping the
    proximal contact interface free (a contact pad thickness, later)."""
    resolution_mm: float = 0.1
    link_length_resolution_mm: float | None = None
    """Machining resolution: the bars ``|A'B|``, ``|AB'|`` and ``B'`` from ``B`` are
    rounded to it and the extended pose is re-assembled (see ``design``);
    ``None``: the design vector is used as it is."""
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
        for name in ("servo_step_rad", "servo_horizon_rad",
                     "minimum_transmission_margin_rad", "resolution_mm"):
            require_positive(name, getattr(self, name))
        require_nonnegative("hyperextension_rad", self.hyperextension_rad)
        require_nonnegative("flexion_stopper_margin_rad", self.flexion_stopper_margin_rad)
        require_nonnegative("dip_backoff_tolerance_rad", self.dip_backoff_tolerance_rad)
        require_finite("curl_tip_min_x_mm", self.curl_tip_min_x_mm)
        if self.reference_point not in ("C", "B"):
            raise ValueError("reference_point must be 'C' or 'B'")
        require_finite("oa_interface_clearance_mm", self.oa_interface_clearance_mm)
        if self.servo_travel_max_rad is not None:
            require_positive("servo_travel_max_rad", self.servo_travel_max_rad)
        require_nonnegative("mcp_flexion_max_rad", self.mcp_flexion_max_rad)
        require_nonnegative("minimum_flexion_rad", self.minimum_flexion_rad)
        if self.link_length_resolution_mm is not None:
            require_positive("link_length_resolution_mm", self.link_length_resolution_mm)
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
    def servo_samples_rad(self) -> tuple[float, ...]:
        count = int(math.floor(self.servo_horizon_rad / self.servo_step_rad + 1e-9)) + 1
        return tuple(index * self.servo_step_rad for index in range(count))


def config_from_mapping(values: Mapping[str, object]) -> FingerStudyConfig:
    """Build a config from plain values, as written in a config file.

    Angles may be given in degrees: ``name_deg`` sets the field ``name_rad``.
    ``bounds`` and ``packaging`` are tables of their fields (``packaging``'s
    servos tables of :class:`DShapedExtrusion` fields); any other table only
    groups fields for reading, so ``motion: {hyperextension_deg: 10}`` is
    ``hyperextension_deg: 10``. A field left out keeps its default.
    """

    known = {f.name for f in fields(FingerStudyConfig)}
    flat: list[tuple[str, object]] = []
    for key, value in values.items():
        if isinstance(value, Mapping) and key not in known:
            flat.extend(value.items())
        else:
            flat.append((key, value))
    kwargs: dict[str, object] = {}
    for key, value in flat:
        name = key
        if key.endswith("_deg") and key[:-4] + "_rad" in known:
            name = key[:-4] + "_rad"
            value = None if value is None else math.radians(float(value))
        if name not in known:
            raise ValueError(f"unknown config entry {key!r}")
        if name in kwargs:
            raise ValueError(f"config entry {name!r} given twice")
        kwargs[name] = value
    kwargs["bounds"] = {name: tuple(pair) for name, pair in dict(kwargs.get("bounds", {})).items()}
    if "extended_length_bounds_mm" in kwargs:
        kwargs["extended_length_bounds_mm"] = tuple(kwargs["extended_length_bounds_mm"])
    if "packaging" in kwargs:
        packaging = dict(kwargs["packaging"])
        for servo in ("pip_servo", "mcp_servo"):
            if servo in packaging:
                packaging[servo] = DShapedExtrusion(**packaging[servo])
        if "mcp_servo_rear" in packaging:
            packaging["mcp_servo_rear"] = tuple(packaging["mcp_servo_rear"])
        kwargs["packaging"] = Packaging(**packaging)
    return FingerStudyConfig(**kwargs)


def load_yaml(path: str | Path) -> FingerStudyConfig:
    """Read a study config from a YAML file (see :func:`config_from_mapping`)."""

    with Path(path).open(encoding="utf-8") as stream:
        return config_from_mapping(yaml.safe_load(stream) or {})


__all__ = ["FingerStudyConfig", "config_from_mapping", "load_yaml"]
