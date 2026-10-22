"""Configuration of the 2-DoF finger linkage study."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from types import MappingProxyType

import yaml

from core._validation import require_finite, require_nonnegative, require_positive

from core.hardware import DShapedExtrusion, actuator
from core.morphology.linkage.packaging import ContactPad, Packaging

from core.optimization.linkage.design import DEFAULT_VALUES, VARIABLE_NAMES


@dataclass(frozen=True)
class FingerStudyConfig:
    """Search space and requirements (the optimizer's settings are
    ``core.optimization.optimizer.OptimizerSettings``).

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
    """Held constraints, reported but enforced only when enabled (the pad
    defines them with the contact interfaces)."""
    curl_tip_min_x_mm: float = 0.0
    """Least ``x`` of the reference point at the most curl (the end of the usable
    range) with the MCP at 0: with ``enforce_curl_tip``, the curled tip stays
    ahead of ``O``."""
    mcp_flexion_max_rad: float = math.radians(90.0)
    """MCP flexion up to which the distal pad (and, when enabled,
    the fingertip) must clear the MCP servo."""
    mcp_step_rad: float = math.radians(1.0)
    """MCP flexion step of the distal pad's MCP servo check."""
    require_middle_link_servo_clearance: bool = False
    """Require the straight middle link ``A-B'`` to clear the PIP servo at ``A'``
    in the plane. Off: its plates straddle the servo in other layers; the
    planar clearance is still reported."""
    minimum_flexion_rad: float = math.radians(120.0)
    """Least ``phi_safe_max``: the distal link must turn at least this far, safely."""
    torque_ratio_max: float | None = None
    """Most ``k_max`` allowed (the PIP servo's torque over the distal moment it
    must hold); ``None``: unbounded."""
    servo_travel_max_rad: float | None = None
    """Most PIP servo rotation from extension to the curl pose; ``None``: unbounded."""
    minimum_proximal_support_mm: float = 0.0
    minimum_distal_support_mm: float = 0.0
    """Least pad lengths ``|OA''|`` and ``|B''C|`` (0: none; the pad topology
    already keeps them positive). Contact-module sizes are the pad requirements; set
    these only for a known mounting requirement."""
    resolution_mm: float = 0.1
    link_length_resolution_mm: float | None = None
    """Machining resolution: the bars ``|A'B|``, ``|AB'|`` and ``B'`` from ``B`` are
    rounded to it and the extended pose is re-assembled (see ``design``);
    ``None``: the design vector is used as it is."""
    pip_servo_tilt_rad: float | None = None
    """The PIP servo body's dorsal tilt as an input, not optimized: the design
    variable ``servo_tilt_deg`` is held at it (its bounds may be left out).
    ``None``: optimized within its bounds."""
    objectives: tuple[str, ...] | None = None
    """Objectives to optimize, by name (``objectives.OBJECTIVES``); ``None``: the
    default ones, ``transmission`` and ``curl``."""

    def __post_init__(self) -> None:
        given = dict(self.bounds)
        for name, value in DEFAULT_VALUES.items():  # a variable added later, left out: fixed
            given.setdefault(name, (value, value))
        if self.pip_servo_tilt_rad is not None:  # an input: the variable is held at it
            tilt = round(math.degrees(require_finite("pip_servo_tilt_rad", self.pip_servo_tilt_rad)), 9)
            if "servo_tilt_deg" in given and tuple(given["servo_tilt_deg"]) != (tilt, tilt):
                raise ValueError("give the PIP servo tilt as pip_servo_tilt_deg or as bounds, not both")
            given["servo_tilt_deg"] = (tilt, tilt)
        if set(given) != set(VARIABLE_NAMES):
            raise ValueError(f"bounds must cover exactly {VARIABLE_NAMES}")
        object.__setattr__(self, "bounds", given)
        bounds = {}
        for name in VARIABLE_NAMES:
            lower, upper = (require_finite(name, value) for value in self.bounds[name])
            if lower > upper:
                raise ValueError(f"{name} lower bound exceeds its upper bound")
            bounds[name] = (lower, upper)
        object.__setattr__(self, "bounds", MappingProxyType(bounds))
        for name in ("servo_step_rad", "servo_horizon_rad", "mcp_step_rad",
                     "minimum_transmission_margin_rad", "resolution_mm"):
            require_positive(name, getattr(self, name))
        require_nonnegative("hyperextension_rad", self.hyperextension_rad)
        require_nonnegative("flexion_stopper_margin_rad", self.flexion_stopper_margin_rad)
        require_nonnegative("dip_backoff_tolerance_rad", self.dip_backoff_tolerance_rad)
        require_finite("curl_tip_min_x_mm", self.curl_tip_min_x_mm)
        if self.reference_point not in ("C", "B"):
            raise ValueError("reference_point must be 'C' or 'B'")
        if self.torque_ratio_max is not None:
            require_positive("torque_ratio_max", self.torque_ratio_max)
        if self.servo_travel_max_rad is not None:
            require_positive("servo_travel_max_rad", self.servo_travel_max_rad)
        require_nonnegative("mcp_flexion_max_rad", self.mcp_flexion_max_rad)
        require_nonnegative("minimum_flexion_rad", self.minimum_flexion_rad)
        require_nonnegative("minimum_proximal_support_mm", self.minimum_proximal_support_mm)
        require_nonnegative("minimum_distal_support_mm", self.minimum_distal_support_mm)
        if self.link_length_resolution_mm is not None:
            require_positive("link_length_resolution_mm", self.link_length_resolution_mm)
        lower, upper = self.extended_length_bounds_mm
        if lower > upper:
            raise ValueError("extended_length_bounds_mm lower bound exceeds its upper bound")

    @property
    def search_bounds(self) -> Mapping[str, tuple[float, float]]:
        """``bounds`` tightened by constraints with a closed form; no feasible
        design is lost.

        With the PIP servo tilted dorsally by 0-90 deg its most palmar point is
        the bottom of the shaft disk, ``A'_z - radius``, so the OA interface
        constraint (the servo dorsal of the ``OA`` line) is exactly ``A'_z >= radius``.
        The linear constraints between the ``x`` coordinates then narrow each
        other's bounds (:meth:`_linear_bounds`).
        """

        bounds = dict(self.bounds)
        tilt_low, tilt_high = bounds["servo_tilt_deg"]
        if 0.0 <= tilt_low and tilt_high <= 90.0 and not self.packaging.pip_servo_on_a:
            low, high = bounds["ap_z_mm"]
            least = self.packaging.pip_servo.radius_mm
            bounds["ap_z_mm"] = (min(max(low, least), high), high)
        self._linear_bounds(bounds)
        return MappingProxyType(bounds)

    def _linear_bounds(self, bounds: dict[str, tuple[float, float]]) -> None:
        """Narrow the ``x`` bounds by the linear constraints, to a fixed point.

        ``B_x + BC`` (or ``B_x``) lies in the extended length bounds, as far as
        ``B_z`` allows; ``A_x < B_x``; ``A'_x >= A_x`` and ``B'_x <= B_x`` (the
        pad topology); each pad's mounting line has room for its fixed or
        minimum length; and, with everything on the ``x`` axis, the extended
        pose's pad checks (:meth:`_extended_pad_bounds`). A bound is only ever moved inward, and never past the
        other one (an empty box is left for the constraints to reject).
        """

        def narrow(name: str, low: float = -math.inf, high: float = math.inf) -> None:
            old_low, old_high = bounds[name]
            new_low, new_high = max(old_low, low), min(old_high, high)
            bounds[name] = (min(new_low, old_high), max(new_high, old_low))

        packaging = self.packaging
        length_low, length_high = self.extended_length_bounds_mm
        z_most = max(abs(value) for value in bounds["b_z_mm"])
        reach_low = math.sqrt(max(length_low ** 2 - z_most ** 2, 0.0))
        proximal_need = max(self.minimum_proximal_support_mm, packaging.proximal_pad.length_mm or 0.0)
        distal_need = max(self.minimum_distal_support_mm, packaging.distal_pad.length_mm or 0.0)
        start = packaging.proximal_pad_start_mm
        on_axis = self._pads_on_axis
        for _ in range(4):
            (a_lo, a_hi), (b_lo, b_hi) = bounds["a_x_mm"], bounds["b_x_mm"]
            (c_lo, c_hi) = bounds["bc_length_mm"]
            if self.reference_point == "C":  # reach_low <= B_x + BC <= length_high
                narrow("b_x_mm", reach_low - c_hi, length_high - c_lo)
                narrow("bc_length_mm", reach_low - b_hi, length_high - b_lo)
            else:
                narrow("b_x_mm", reach_low, length_high)
            (b_lo, b_hi), (c_lo, c_hi) = bounds["b_x_mm"], bounds["bc_length_mm"]
            narrow("a_x_mm", high=b_hi)                                # A_x < B_x
            narrow("b_x_mm", low=a_lo)
            if not packaging.pip_servo_on_a:  # the proximal pad runs to A'' = A'_x + offset
                offset_lo, offset_hi = bounds["app_offset_mm"]
                narrow("ap_x_mm", low=max(a_lo, start + proximal_need) - offset_hi)  # A''_x >= A_x, room
                narrow("a_x_mm", high=bounds["ap_x_mm"][1] + offset_hi)
            distal_outer = b_hi + c_hi - packaging.distal_pad.offset_mm
            narrow("bp_x_mm", high=min(b_hi, distal_outer - distal_need))  # B'_x <= B_x, room
            narrow("b_x_mm", low=bounds["bp_x_mm"][0])
            if on_axis:
                self._extended_pad_bounds(bounds, narrow)

    @property
    def _pads_on_axis(self) -> bool:
        """Whether ``A``, ``B`` and both free-length pads lie on the ``x`` axis when extended."""

        packaging = self.packaging
        return (not packaging.pip_servo_on_a and packaging.pad_link_ratio_max is None
                and self.bounds["a_z_mm"] == (0.0, 0.0) and self.bounds["b_z_mm"] == (0.0, 0.0)
                and packaging.proximal_pad.length_mm is None and packaging.distal_pad.length_mm is None)

    def _extended_pad_bounds(self, bounds, narrow) -> None:
        """The extended pose's pad checks, linear when everything lies on the ``x`` axis.

        The proximal pad ends at ``A''_x = A'_x + offset``, the distal pad starts at ``B'_x``, both
        with their mounting edge on ``z = 0``, as are the ``A`` and ``B`` pins:
        the pads keep the body clearance from each other, the pins a pin
        envelope more, and (with ``pad_mcp_check``) the distal pad keeps clear
        of the MCP servo's shaft disk.
        """

        packaging = self.packaging
        clearance = packaging.body_clearance_mm
        pin = packaging.pin_radius_mm + clearance
        (a_lo, _), (b_lo, b_hi) = bounds["a_x_mm"], bounds["b_x_mm"]
        (ap_lo, ap_hi), (bp_lo, bp_hi) = bounds["ap_x_mm"], bounds["bp_x_mm"]
        offset_lo, offset_hi = bounds["app_offset_mm"]
        app_lo = ap_lo + offset_lo  # least A''_x
        mcp = (packaging.mcp_servo.radius_mm + packaging.mcp_pad_clearance_mm if packaging.pad_mcp_check
               else -math.inf)
        narrow("bp_x_mm", low=max(app_lo + clearance, a_lo + pin, mcp))  # pads, A pin, MCP
        # A''_x <= B'_x - clearance and B_x - pin, as far as the offset allows.
        narrow("ap_x_mm", high=min(bp_hi - clearance, b_hi - pin) - offset_lo)
        narrow("a_x_mm", high=bp_hi - pin)
        narrow("b_x_mm", low=app_lo + pin)

    @property
    def lower(self) -> tuple[float, ...]:
        return tuple(self.search_bounds[name][0] for name in VARIABLE_NAMES)

    @property
    def upper(self) -> tuple[float, ...]:
        return tuple(self.search_bounds[name][1] for name in VARIABLE_NAMES)

    @property
    def mcp_samples_rad(self) -> tuple[float, ...]:
        """MCP flexion of the pad to MCP servo check: from extension (the pads may
        touch in hyperextension) to ``mcp_flexion_max_rad``, every ``mcp_step_rad``."""

        count = int(math.floor(self.mcp_flexion_max_rad / self.mcp_step_rad + 1e-9)) + 1
        return tuple(index * self.mcp_step_rad for index in range(count))

    @property
    def servo_samples_rad(self) -> tuple[float, ...]:
        count = int(math.floor(self.servo_horizon_rad / self.servo_step_rad + 1e-9)) + 1
        return tuple(index * self.servo_step_rad for index in range(count))


_ALIASES = {"minimum_desired_flexion_angle_deg": "minimum_flexion_deg"}
"""Config file names of fields named otherwise in the code."""


def config_from_mapping(values: Mapping[str, object]) -> FingerStudyConfig:
    """Build a config from plain values, as written in a config file.

    Angles may be given in degrees: ``name_deg`` sets the field ``name_rad``.
    ``optimization_parameter_bounds`` (or ``bounds``) and ``packaging`` are
    tables of their fields (``packaging``'s
    servos an actuator's name, e.g. ``xm335`` (``core.hardware.ACTUATORS``), or a
    table of :class:`DShapedExtrusion` fields). ``pad`` holds the
    contact pads, part of the packaging: ``proximal`` and ``distal`` tables of
    :class:`ContactPad` fields; ``allow_collision_with_mcp_servo`` (no pad to
    MCP servo check, the proximal pad from ``O``), ``mcp_servo_clearance_mm`` (the
    gap kept from it otherwise) and
    ``allow_longer_pad_than_link`` (false: a pad no longer than its link,
    ``|OA|`` or ``BC``). Any other
    table only groups fields for reading, so ``motion: {hyperextension_deg:
    10}`` is ``hyperextension_deg: 10``. A field left out keeps its default.
    """

    values = dict(values)
    if "optimization_parameter_bounds" in values:  # the design variables' search box
        if "bounds" in values:
            raise ValueError("give optimization_parameter_bounds or bounds, not both")
        values["bounds"] = values.pop("optimization_parameter_bounds")
    values = merge_pad_group(values)

    known = {f.name for f in fields(FingerStudyConfig)}
    flat: list[tuple[str, object]] = []
    for key, value in values.items():
        if isinstance(value, Mapping) and key not in known:
            flat.extend(value.items())
        else:
            flat.append((key, value))
    kwargs: dict[str, object] = {}
    for key, value in flat:
        key = _ALIASES.get(key, key)
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
    if kwargs.get("objectives") is not None:
        kwargs["objectives"] = tuple(kwargs["objectives"])
    if "extended_length_bounds_mm" in kwargs:
        kwargs["extended_length_bounds_mm"] = tuple(kwargs["extended_length_bounds_mm"])
    if "packaging" in kwargs:
        kwargs["packaging"] = packaging_from_mapping(kwargs["packaging"])
    return FingerStudyConfig(**kwargs)


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
    values["packaging"] = packaging
    return values


def packaging_from_mapping(values: Mapping[str, object]) -> Packaging:
    """A :class:`Packaging` from plain values: servos by actuator name (e.g.
    ``xm335``, ``core.hardware.ACTUATORS``) or as tables, pads as tables."""

    packaging = dict(values)
    if "pin_envelope_diameter_mm" in packaging:  # its earlier name (runs before 2026-10-21)
        packaging["pivot_clearance_diameter_mm"] = packaging.pop("pin_envelope_diameter_mm")
    for joint in ("A", "Bp"):  # as written in the config
        if f"allow_collision_pip_servo_to_joint_{joint}" in packaging:
            packaging[f"allow_collision_pip_servo_to_joint_{joint.lower()}"] = packaging.pop(
                f"allow_collision_pip_servo_to_joint_{joint}")
    for servo in ("pip_servo", "mcp_servo"):
        if isinstance(packaging.get(servo), str):  # an actuator by name, e.g. xm335
            packaging[servo] = actuator(packaging[servo]).geometry
        elif servo in packaging and not isinstance(packaging[servo], DShapedExtrusion):
            packaging[servo] = DShapedExtrusion(**packaging[servo])
    for pad in ("proximal_pad", "distal_pad"):
        if pad in packaging and not isinstance(packaging[pad], ContactPad):
            packaging[pad] = ContactPad(**packaging[pad])
    if "mcp_servo_rear" in packaging:
        packaging["mcp_servo_rear"] = tuple(packaging["mcp_servo_rear"])
    return Packaging(**packaging)


def load_yaml(path: str | Path) -> FingerStudyConfig:
    """Read a study config from a YAML file (see :func:`config_from_mapping`),
    less its optimizer settings (``core.optimization.study.load_yaml`` reads both)."""

    from core.optimization.optimizer import split_settings

    with Path(path).open(encoding="utf-8") as stream:
        return config_from_mapping(split_settings(yaml.safe_load(stream) or {})[0])


def config_to_json(config: FingerStudyConfig) -> dict:
    """The config as JSON values, for ``run.json`` (with the search box it used)."""

    values = {f.name: getattr(config, f.name) for f in fields(config)}
    values["bounds"] = dict(config.bounds)
    values["search_bounds"] = dict(config.search_bounds)
    values["packaging"] = asdict(config.packaging)
    return values


def config_from_json(values: dict) -> FingerStudyConfig:
    """Rebuild a config recorded by :func:`config_to_json`, migrating earlier runs."""

    values["bounds"] = {name: tuple(bounds) for name, bounds in values["bounds"].items()}
    for old, new in (("pip_step_rad", "servo_step_rad"), ("pip_horizon_rad", "servo_horizon_rad")):
        if old in values:  # runs swept by the PIP; the step and horizon carry over
            values[new] = values.pop(old)
    for name in ("range_margin_rad", "stopper_margin_rad"):  # one margin for both ends
        if name in values:
            margin = values.pop(name)
            values["hyperextension_rad"] = values["flexion_stopper_margin_rad"] = margin
    for name, value in DEFAULT_VALUES.items():  # runs from before the variable existed
        values["bounds"].setdefault(name, (value, value))
    values["extended_length_bounds_mm"] = tuple(values["extended_length_bounds_mm"])
    if values.get("objectives") is not None:
        values["objectives"] = tuple(values["objectives"])
    if "packaging" in values:
        packaging = values["packaging"]
        # A clearance of the PIP servo over the OA line that was always 0.
        values.pop("oa_interface_clearance_mm", None)
        packaging.pop("oa_interface_clearance_mm", None)
        # The dorsal support deck and web of support-structure-v1; pads replaced them.
        packaging.pop("support_thickness_mm", None)
        packaging.pop("support_web_width_mm", None)
        for kind in ("bushing", "bearing"):  # runs that recorded the bearing itself
            if f"{kind}_outer_diameter_mm" in packaging:
                housing = (packaging.pop(f"{kind}_outer_diameter_mm")
                           + 2.0 * packaging.pop("housing_wall_mm"))
                # Before the flange was modelled, the housing alone was the envelope.
                flange = packaging.pop(f"{kind}_flange_diameter_mm", 0.0)
                packaging["pivot_clearance_diameter_mm"] = max(housing, flange)
        values["packaging"] = packaging_from_mapping(packaging)
    known = {f.name for f in fields(FingerStudyConfig)}
    return FingerStudyConfig(**{name: value for name, value in values.items() if name in known})


__all__ = [
    "FingerStudyConfig",
    "config_from_json",
    "config_from_mapping",
    "config_to_json",
    "load_yaml",
    "merge_pad_group",
    "packaging_from_mapping",
]
