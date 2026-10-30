"""Configuration of a contact-retention study, shared by the mechanisms it compares."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass, field, fields
from types import MappingProxyType

from core._validation import require_finite, require_nonnegative, require_positive
from core.morphology.packaging.hardware import ContactPad, Packaging

from core.optimization.linkage.config import packaging_from_mapping


def _nominal_packaging() -> Packaging:
    pad = ContactPad(height_mm=5.0, offset_mm=0.0, chamfer_mm=0.0, length_mm=45.0)
    return Packaging(proximal_pad=pad, distal_pad=pad)


@dataclass(frozen=True)
class RetentionConfig:
    """Search space and requirements of one mechanism's retention study.

    The comparison fixes everything but ``bounds`` across mechanisms: the
    nominal modules (``packaging``'s pads: ``height_mm`` the thickness ``h``,
    ``length_mm`` the nominal length, ``offset_mm`` the outer edge in from
    its end, no chamfer), the hardware and clearances, and the required ROM.
    The proximal module's outer edge is at
    :attr:`~core.morphology.linkage.packaging.Packaging.proximal_pad_start_mm`
    (past the MCP servo's shaft disk unless ``pad_mcp_check`` is off), the
    distal one's its offset short of the distal segment's end, whether or not
    the mechanism models the MCP servo.
    """

    bounds: Mapping[str, tuple[float, float]]
    """The mechanism's geometry variables; the module variables run over
    ``[0, nominal]`` on their own."""
    packaging: Packaging = field(default_factory=_nominal_packaging)
    """``pip_servo``: the distal joint's servo (2R: at ``A``; RCJ: at the
    proximal roller), ``mcp_servo``: at ``O``."""
    required_rom_rad: float = math.radians(120.0)
    """Distal flexion from extension (relative to the proximal segment) every
    design must reach continuously, without a collision."""
    rom_step_rad: float = math.radians(1.0)
    mcp_flexion_max_rad: float = math.radians(90.0)
    """MCP flexion over which the palm's MCP servo is checked."""
    mcp_step_rad: float = math.radians(1.0)
    extended_length_bounds_mm: tuple[float, float] = (80.0, 110.0)
    """Bounds on ``|O tip|`` in the extended pose."""
    joint_servo_rear: tuple[float, float] = (0.0, 1.0)
    """Unit direction of the distal joint servo's rear body in the proximal
    frame (``+Z``: standing dorsally), fixed to the proximal body."""
    cable_allowance_mm: float = 0.0
    """RCJ: roller radius kept beyond the servo radius for the cables."""
    resolution_mm: float = 0.1
    objectives: tuple[str, ...] | None = None

    def __post_init__(self) -> None:
        bounds = {}
        for name, pair in self.bounds.items():
            lower, upper = (require_finite(name, value) for value in pair)
            if lower > upper:
                raise ValueError(f"{name} lower bound exceeds its upper bound")
            bounds[name] = (lower, upper)
        object.__setattr__(self, "bounds", MappingProxyType(bounds))
        for name in ("proximal_pad", "distal_pad"):
            pad = getattr(self.packaging, name)
            if pad.length_mm is None:
                raise ValueError(f"{name} needs its nominal length_mm")
            if pad.chamfer_mm != 0.0:
                raise ValueError(f"{name}: the module is the 4-vertex block, chamfer_mm must be 0")
        for name in ("required_rom_rad", "rom_step_rad", "mcp_step_rad", "resolution_mm"):
            require_positive(name, getattr(self, name))
        require_nonnegative("mcp_flexion_max_rad", self.mcp_flexion_max_rad)
        require_nonnegative("cable_allowance_mm", self.cable_allowance_mm)
        lower, upper = self.extended_length_bounds_mm
        if lower > upper:
            raise ValueError("extended_length_bounds_mm lower bound exceeds its upper bound")
        length = math.hypot(*self.joint_servo_rear)
        if length <= 0.0:
            raise ValueError("joint_servo_rear must be a non-zero direction")
        object.__setattr__(self, "joint_servo_rear",
                           (self.joint_servo_rear[0] / length, self.joint_servo_rear[1] / length))

    @property
    def rom_samples_rad(self) -> tuple[float, ...]:
        """Distal flexion from extension to the required ROM, every ``rom_step_rad``, both ends included."""

        count = int(math.ceil(self.required_rom_rad / self.rom_step_rad - 1e-9))
        return tuple(min(index * self.rom_step_rad, self.required_rom_rad) for index in range(count + 1))

    @property
    def mcp_samples_rad(self) -> tuple[float, ...]:
        count = int(math.floor(self.mcp_flexion_max_rad / self.mcp_step_rad + 1e-9)) + 1
        return tuple(index * self.mcp_step_rad for index in range(count))


def config_from_mapping(values: Mapping[str, object]) -> RetentionConfig:
    """Build a config from plain values, as written in a config file.

    As the linkage's former (v1) config file:
    ``name_deg`` sets ``name_rad``, ``optimization_parameter_bounds`` the
    bounds, ``pad`` the modules, other tables only group fields.
    """

    values = merge_pad_group(values)
    if "optimization_parameter_bounds" in values:
        values["bounds"] = values.pop("optimization_parameter_bounds")
    known = {f.name for f in fields(RetentionConfig)}
    kwargs: dict[str, object] = {}
    flat = [item for key, value in values.items()
            for item in (value.items() if isinstance(value, Mapping) and key not in known else ((key, value),))]
    for key, value in flat:
        name = key
        if key.endswith("_deg") and key[:-4] + "_rad" in known:
            name, value = key[:-4] + "_rad", math.radians(float(value))
        if name not in known:
            raise ValueError(f"unknown config entry {key!r}")
        if name in kwargs:
            raise ValueError(f"config entry {name!r} given twice")
        kwargs[name] = value
    return _build(kwargs)


def _build(kwargs: dict) -> RetentionConfig:
    kwargs["bounds"] = {name: tuple(pair) for name, pair in dict(kwargs.get("bounds", {})).items()}
    for name in ("objectives", "extended_length_bounds_mm", "joint_servo_rear"):
        if kwargs.get(name) is not None:
            kwargs[name] = tuple(kwargs[name])
    if "packaging" in kwargs:
        kwargs["packaging"] = packaging_from_mapping(kwargs["packaging"])
    return RetentionConfig(**kwargs)


def config_to_json(config: RetentionConfig) -> dict:
    values = {f.name: getattr(config, f.name) for f in fields(config)}
    values["bounds"] = dict(config.bounds)
    values["packaging"] = asdict(config.packaging)
    return values


def config_from_json(values: dict) -> RetentionConfig:
    known = {f.name for f in fields(RetentionConfig)}
    return _build({name: value for name, value in values.items() if name in known})



# The linkage's former (v1) pad group, kept here for the retention config.
def merge_pad_group(values: Mapping[str, object]) -> dict:
    """Move a config's ``pad`` group into its ``packaging`` table (see
    :func:`config_from_mapping`)."""

    values = dict(values)
    pads = values.pop("pad", None)
    if pads is None:
        return values
    names = {  # entry: (packaging field, its value)
        "proximal": ("proximal_pad", lambda value: value),
        "distal": ("distal_pad", lambda value: value),
        "allow_collision_with_mcp_servo": ("pad_mcp_check", lambda allow: not allow),
        "mcp_servo_clearance_mm": ("pad_mcp_clearance_mm", lambda value: value),
        "allow_longer_pad_than_link": ("pad_link_ratio_max", lambda allow: None if allow else 1.0),
    }
    unknown = set(pads) - set(names)
    if unknown:
        raise ValueError(f"unknown pad entries {sorted(unknown)}; known: {sorted(names)}")
    packaging = dict(values.get("packaging") or {})
    for key, value in pads.items():
        name, convert = names[key]
        if name in packaging:
            raise ValueError(f"pad entry {key!r} also given in packaging")
        packaging[name] = convert(value)
        if key in ("proximal", "distal") and isinstance(value, Mapping):
            packaging[name] = _length_range(values, key, value)
    values["packaging"] = packaging
    return values


def _length_range(values: dict, side: str, pad: Mapping[str, object]) -> dict:
    """A pad entry as written: ``chamfer`` true makes the chamfer a design
    variable (0 to the height), false gives none (``chamfer_mm`` still sets a
    fixed one). A ``length_mm`` of ``[min, max]`` gives the least mounting
    length (``minimum_<side>_support_mm``, set in ``values``) and the most
    (``max_length_mm``: a free pad is cut to it; null: none). A number or null
    ``length_mm`` is left as it is (fixed, or free)."""

    pad = dict(pad)
    if pad.get("chamfer", False) is None:  # null: not given (a fixed chamfer_mm may follow)
        pad.pop("chamfer")
    if "chamfer" in pad:  # true: the chamfer is a design variable up to the height; false: none
        if "chamfer_mm" in pad:
            raise ValueError(f"pad {side}: give chamfer (true/false) or chamfer_mm, not both")
        pad["chamfer_free"], pad["chamfer_mm"] = bool(pad.pop("chamfer")), 0.0
    length = pad.get("length_mm")
    if not isinstance(length, (list, tuple)):
        return pad
    if len(length) != 2:
        raise ValueError(f"pad {side} length_mm must be a number, null or [min, max]")
    low, high = length
    if low is not None and high is not None and low > high:
        raise ValueError(f"pad {side} length_mm: min exceeds max")
    if f"minimum_{side}_support_mm" in values:
        raise ValueError(f"give the {side} pad's least length in pad.{side}.length_mm or as "
                         f"minimum_{side}_support_mm, not both")
    values[f"minimum_{side}_support_mm"] = 0.0 if low is None else low
    pad["length_mm"], pad["max_length_mm"] = None, high
    return pad


__all__ = ["RetentionConfig", "config_from_json", "config_from_mapping", "config_to_json"]
