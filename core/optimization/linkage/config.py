"""Configuration of the linkage study, read from a ``goal`` / ``solver`` file.

The file (``scripts/optimize/linkage/config.yaml``) has two levels, by who
sets them: ``goal``, what the finger must do, carry and keep clear (the user),
and ``solver``, how the design is searched. :func:`config_from_goal` maps it
onto :class:`LinkageConfig`. Fixed in the code, not options:

- no stopper margin past ``goal.motion.dip_flexion_deg``;
- the fingertip ``C`` stays ahead of the MCP (``x >= 0``, MCP at 0) while flexing;
- the PIP servo's shaft is on ``A'``; the servos are at their model's nominal
  size; no clearance anywhere (touching is allowed, overlapping is not:
  ``Packaging.overlap_depth``);
- each pad's mounting length is searched in ``[min, max]`` (its ``A''`` /
  ``B''``), whatever the objectives.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass, field, fields
from types import MappingProxyType
from typing import Any

import numpy as np

from core._validation import require_finite, require_nonnegative, require_positive
from core.hardware import DShapedExtrusion, actuator
from core.morphology.linkage.packaging import ContactPad, Packaging

from core.optimization.linkage.design import DEFAULT_VALUES, VARIABLE_NAMES


OBJECTIVE_NAMES = ("transmission", "pad_coverage", "curl", "finger_length")
SEARCH_BOUNDS = ("a_z_mm", "b_z_mm", "ap_from_a_x_mm", "ap_from_a_z_mm", "bp_from_b_x_mm", "bp_from_b_z_mm",
                 "app_offset_mm", "bpp_offset_mm")
"""The design variables ``solver.search_bounds`` gives; the rest come from the goal."""
RELATIVE_BOUNDS = ("ap_from_a_x_mm", "ap_from_a_z_mm", "bp_from_b_x_mm", "bp_from_b_z_mm")
"""``A'`` from ``A`` and ``B'`` from ``B``, as searched (:mod:`~core.optimization.linkage.search`)."""
COLLISION_CHECKS = ("a_pin_and_pip_servo", "a_pin_and_a_prime_pin", "b_prime_pin_and_pip_servo",
                    "a_pin_and_proximal_pad", "b_pin_and_distal_pad", "proximal_pad_and_distal_pad",
                    "pip_servo_and_proximal_pad", "mcp_servo_and_proximal_pad", "distal_carrier_and_pip_servo")
PIVOT_KEYS = MappingProxyType({"a": "a", "a_prime": "ap", "b": "b", "b_prime": "bp"})
"""``goal.collision.param.pivot_diameter_mm`` keys and the packaging's pivot names."""


@dataclass(frozen=True)
class LinkageConfig:
    """One study: the search box, the requirements and the packaging.

    ``bounds`` are the search box as given; :attr:`search_bounds` narrows it
    where a constraint has a closed form. Angles in radians, lengths in mm.
    """

    bounds: Mapping[str, tuple[float, float]]
    packaging: Packaging = field(default_factory=Packaging)
    required_rom_rad: float = math.radians(150.0)
    """``goal.motion.dip_flexion_deg``: the rotation of ``BC`` relative to
    ``OA`` (``phi = q_pip + q_dip``) every design must reach safely; ``k_max``
    and the effective pads are taken on ``[0, required_rom_rad]``."""
    hyperextension_rad: float = math.radians(10.0)
    """PIP hyperextension the linkage must reach (the stopper)."""
    mcp_flexion_max_rad: float = math.radians(90.0)
    """MCP flexion up to which the pads must clear the MCP servo (with
    :attr:`mcp_rotation_checked`)."""
    mcp_rotation_checked: bool = True
    """Check the pads against the MCP servo over the MCP flexion, and the two
    servos over MCP hyperextension. Off (the goal file, since 2026-10-26):
    everything at MCP 0. On: runs recorded before then."""
    transmission_angle_range_rad: tuple[float, float] = (math.radians(30.0), math.radians(150.0))
    """The transmission angle at ``B'`` stays within it over the whole motion."""
    torque_ratio_max: float | None = None
    """``goal.hardware.pip_servo.torque_ratio_max``: the most ``k_max`` (PIP
    servo torque per distal moment held, on ``[0, required_rom_rad]``)
    allowed; ``None``: no limit."""
    extended_length_bounds_mm: tuple[float, float] = (90.0, 110.0)
    """``|OC|`` in the extended pose."""
    proximal_length_bounds_mm: tuple[float | None, float | None] = (None, None)
    """``|OA|`` (``None``: no limit)."""
    proximal_length_ratio_bounds: tuple[float | None, float | None] = (None, None)
    distal_length_ratio_bounds: tuple[float | None, float | None] = (None, None)
    joint_span_ratio_bounds: tuple[float | None, float | None] = (None, None)
    """``|OA|``, ``BC`` and ``|AB|`` as shares of the extended ``|OC|``
    (``goal.link.proximal_length_ratio`` / ``distal_length_ratio`` /
    ``joint_span_ratio``; ``None``: no limit), with the ``_mm`` bounds besides."""
    joint_span_bounds_mm: tuple[float | None, float | None] = (None, None)
    """``|AB|`` in the extended pose, the four-bar's span along the finger
    (``goal.link.joint_span_mm``; ``None``: no limit)."""
    minimum_proximal_support_mm: float = 0.0
    minimum_distal_support_mm: float = 0.0
    """Each pad's least mounting length ``a-b``."""
    pip_servo_tilt_rad: float | None = None
    """The PIP servo's dorsal tilt as an input (the variable is held at it);
    ``None``: optimized within its bounds."""
    servo_step_rad: float = math.radians(1.0)
    servo_horizon_rad: float = math.pi
    """PIP servo step and the most rotation swept from extension."""
    verify_step_rad: float = math.radians(0.05)
    """Servo step the front is verified on."""
    mcp_step_rad: float = math.radians(1.0)
    dip_backoff_tolerance_rad: float = math.radians(0.05)
    resolution_mm: float = 0.1
    link_length_resolution_mm: float | None = 0.1
    """The prototyping resolution: the design grid, and the bars ``|A'B|``,
    ``|AB'|`` and ``B'`` from ``B`` rounded to it (``design.decode``)."""
    objectives: tuple[str, ...] | None = None
    """Objectives to optimize, by name; ``None``: the mechanism's defaults."""
    hypervolume_reference: Mapping[str, float] | None = None
    """Per objective, in its own terms (``pad_coverage`` as a share,
    ``curl`` as ``180 - phi_max`` in degrees); missing entries take the
    defaults: ``k_max`` 2.3, coverage 0, ``180 - phi_req``, the finger's max."""
    seeds: tuple[int, ...] = (0,)
    """Optimizer seeds, one run each, merged into one front."""
    tilt_step_deg: float = 1.0
    """Grid step of the PIP servo's tilt when searched, from the least of its range."""
    proximal_pad_past_icr_max_mm: float | None = None
    """How far the proximal pad's end ``A''`` may lie past the extended pose's
    centre of rotation (``A-B'`` crossing ``A'-B``), along the finger
    (``goal.pads.proximal.past_icr_max_mm``): the fold, which a fixed pivot's
    pads stop at too (its pin); past it the pad hangs unsupported under the
    four-bar. ``None``: no limit. The distal pad may run back as far as it fits."""
    pads_on_links: bool = False
    """Searched with each pad on its own link only (``goal.pads.on_links``,
    true unless the file says otherwise; ``False`` here for runs before it):
    the proximal pad to ``A``, the distal one from ``B`` to ``C``, so more pad
    means a shorter four-bar. ``scripts/optimize/extend_pads.py`` then grows
    the pads of the designs found as far as they keep usable."""
    symmetric_four_bar: bool = False
    """The four-bar an antiparallelogram: ``|B'B| = |A'A|`` and ``|AB'| = |A'B|``,
    ``B'`` then set by ``A``, ``A'`` and ``B`` (``goal.link.symmetric_four_bar``)."""
    warm_start: tuple[str, ...] = ()
    """``seeds.csv`` / ``pareto.csv`` files (from the repository root) whose
    designs seed up to half of each run's first population
    (``solver.optimizer.warm_start``; re-evaluated, so any earlier study may serve)."""
    relative_bounds: Mapping[str, tuple[float, float]] | None = None
    """Search box of ``A'`` from ``A`` and ``B'`` from ``B`` (:data:`RELATIVE_BOUNDS`);
    ``bounds`` then holds the absolute box they span. ``None``: ``A'`` and
    ``B'`` searched in ``bounds`` (runs recorded before 2026-10-26)."""
    proximal_overhang_max_mm: float | None = None
    distal_overhang_max_mm: float | None = None
    """How far each pad may run past its own body (``goal.pads.*.overhang_max_mm``):
    the proximal pad past the proximal body's front (its joint's housing, the
    servo), the distal pad behind the distal body's back, in the extended
    pose; ``None``: no limit."""
    pulley_radius_mm: float | None = None
    """``goal.hardware.pulley_radius_mm``: the envelope of a belt's or a
    tendon's pulley on a joint (the mechanisms that drive one through them);
    ``None``: none given."""
    icr0_share_bounds: tuple[float | None, float | None] = (None, None)
    """``goal.link.icr0_share``: where the extended pose's centre of rotation
    may lie, as a share of ``|OC|`` from ``O`` (``core.optimization.analysis.icr``;
    the bending-point control runs); ``(None, None)``: anywhere."""
    dorsal_space_mm: float | None = None
    """``goal.link.dorsal_space_mm``: how far the extended finger's own solids
    may rise above the common datum, the line through ``O`` along the finger
    (the pads, free in their drops, below it; the MCP servo, in the palm, left
    out; ``core.optimization.analysis.loss.dorsal_space_mm``); ``None``: any."""
    link_radius_mm: float | None = None
    """``goal.collision.param.link_radius_mm``: the links and bars as capsules
    of this radius, each checked against the pads it does not carry, in the
    side view (the comparison model); ``None``: the links pass beside the
    pads (the outer layer), unchecked."""
    count_fold: bool = True
    """``goal.loss.fold``: each face less what the other hides in the fold
    (the v4 measure); false: a module mounted and clear through the motion
    counts whole (the comparison model's accommodation)."""
    integrity_depth_mm: float | None = None
    """``goal.loss.integrity_depth_mm``: the objective's loss rule
    (``core.optimization.analysis.loss.integrity_free``): a face's point is
    lost when, at any pose to the required flexion, it is hidden in the fold
    or any solid but its own pad lies within this depth in front of it.
    ``None``: the fold alone, as measured before linkage-v5."""
    access_depth_mm: float = 20.0
    """``goal.loss.access_depth_mm``: the depth of the accessibility profile
    (reports only, not the objective)."""

    def __post_init__(self) -> None:
        if self.packaging.pip_servo_on_a:
            raise ValueError("the linkage study has the PIP servo on A'")
        given = dict(self.bounds)
        # A free chamfer runs 0 to the pad's height (unless bounded); a fixed
        # one leaves its variable unused, held at 0 whatever bounds say.
        for side in ("proximal", "distal"):
            pad, name = getattr(self.packaging, f"{side}_pad"), f"{side}_chamfer_mm"
            if pad.chamfer_free:
                given.setdefault(name, (0.0, pad.height_mm))
            else:
                given[name] = (0.0, 0.0)
        # The pads' drops, unless searched: the packaging's (the variables held at it).
        given.setdefault("proximal_drop_mm", (self.packaging.proximal_pad_drop_mm,) * 2)
        given.setdefault("distal_drop_mm", (self.packaging.distal_pad_drop_mm,) * 2)
        for name, value in DEFAULT_VALUES.items():  # a variable left out: fixed
            given.setdefault(name, (value, value))
        if self.pip_servo_tilt_rad is not None:  # an input: the variable is held at it
            tilt = round(math.degrees(require_finite("pip_servo_tilt_rad", self.pip_servo_tilt_rad)), 9)
            if "servo_tilt_deg" in given and tuple(given["servo_tilt_deg"]) != (tilt, tilt):
                raise ValueError("give the PIP servo tilt as an input or as bounds, not both")
            given["servo_tilt_deg"] = (tilt, tilt)
        if set(given) != set(VARIABLE_NAMES):
            raise ValueError(f"bounds must cover exactly {VARIABLE_NAMES}")
        bounds = {}
        for name in VARIABLE_NAMES:
            lower, upper = (require_finite(name, value) for value in given[name])
            if lower > upper:
                raise ValueError(f"{name} lower bound exceeds its upper bound")
            bounds[name] = (lower, upper)
        object.__setattr__(self, "bounds", MappingProxyType(bounds))
        for name in ("required_rom_rad", "servo_step_rad", "servo_horizon_rad", "verify_step_rad",
                     "mcp_step_rad", "resolution_mm"):
            require_positive(name, getattr(self, name))
        for name in ("hyperextension_rad", "mcp_flexion_max_rad", "dip_backoff_tolerance_rad",
                     "minimum_proximal_support_mm", "minimum_distal_support_mm"):
            require_nonnegative(name, getattr(self, name))
        if self.required_rom_rad > math.pi + 1e-12:
            raise ValueError("the required flexion may not exceed 180 deg (BC antiparallel to OA)")
        low, high = self.transmission_angle_range_rad
        if not 0.0 <= low < high <= math.pi:
            raise ValueError("transmission_angle_range_rad must satisfy 0 <= min < max <= 180 deg")
        if self.extended_length_bounds_mm[0] > self.extended_length_bounds_mm[1]:
            raise ValueError("extended_length_bounds_mm lower bound exceeds its upper bound")
        if self.link_length_resolution_mm is not None:
            require_positive("link_length_resolution_mm", self.link_length_resolution_mm)
        if not self.seeds:
            raise ValueError("seeds must list at least one seed")
        given = {name: value for name, value in dict(self.hypervolume_reference or {}).items()
                 if value is not None}
        unknown = set(given) - set(OBJECTIVE_NAMES)
        if unknown:
            raise ValueError(f"unknown objectives in the reference point: {sorted(unknown)}")
        reference = {"transmission": 2.3, "pad_coverage": 0.0,
                     "curl": 180.0 - math.degrees(self.required_rom_rad),
                     "finger_length": self.extended_length_bounds_mm[1]}
        object.__setattr__(self, "hypervolume_reference", MappingProxyType({**reference, **given}))
        for name in ("proximal_overhang_max_mm", "distal_overhang_max_mm", "pulley_radius_mm",
                     "integrity_depth_mm", "dorsal_space_mm", "link_radius_mm"):
            if getattr(self, name) is not None:
                require_nonnegative(name, getattr(self, name))
        if self.relative_bounds is not None:
            if set(self.relative_bounds) != set(RELATIVE_BOUNDS):
                raise ValueError(f"relative_bounds must cover exactly {RELATIVE_BOUNDS}")
            object.__setattr__(self, "relative_bounds", MappingProxyType(
                {name: tuple(float(v) for v in self.relative_bounds[name]) for name in RELATIVE_BOUNDS}))

    @property
    def search_bounds(self) -> Mapping[str, tuple[float, float]]:
        """``bounds`` narrowed by the linear constraints between the ``x``
        coordinates (:meth:`_linear_bounds`); no feasible design is lost."""

        bounds = dict(self.bounds)
        self._linear_bounds(bounds)
        return MappingProxyType(bounds)

    def _linear_bounds(self, bounds: dict[str, tuple[float, float]]) -> None:
        """Narrow the ``x`` bounds by the linear constraints, to a fixed point.

        ``B_x + BC`` lies in the extended length bounds, as far as ``B_z``
        allows; ``A_x < B_x``; ``A''_x >= A_x`` and ``B''_x <= B_x`` (the pad
        topology, unless the pads only keep off their pins: ``pads_off_pins``);
        each pad's mounting line has room for its least length;
        and, with everything on the ``x`` axis, the extended pose's pad checks
        (:meth:`_extended_pad_bounds`). A bound only ever moves inward, and
        never past the other one (an empty box is left for the constraints).
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
        for _ in range(4):
            (a_lo, _), (b_lo, b_hi) = bounds["a_x_mm"], bounds["b_x_mm"]
            c_lo, c_hi = bounds["bc_length_mm"]
            narrow("b_x_mm", reach_low - c_hi, length_high - c_lo)  # reach_low <= B_x + BC <= length_high
            narrow("bc_length_mm", reach_low - b_hi, length_high - b_lo)
            (b_lo, b_hi), (c_lo, c_hi) = bounds["b_x_mm"], bounds["bc_length_mm"]
            narrow("a_x_mm", high=b_hi)                                # A_x < B_x
            narrow("b_x_mm", low=a_lo)
            offset_lo, offset_hi = bounds["app_offset_mm"]
            boffset_lo, boffset_hi = bounds["bpp_offset_mm"]
            distal_outer = b_hi + c_hi - packaging.distal_pad.offset_mm
            if not packaging.pads_off_pins:  # (off their pins, the pads may end anywhere)
                # The proximal pad runs to A'' = A'_x + offset: A''_x >= A_x, room.
                narrow("ap_x_mm", low=max(a_lo, start + proximal_need) - offset_hi)
                narrow("a_x_mm", high=bounds["ap_x_mm"][1] + offset_hi)
                # The distal pad runs to B'' = B'_x - offset: B''_x <= B_x, room.
                narrow("bp_x_mm", high=min(b_hi, distal_outer - distal_need) + boffset_hi)
                narrow("b_x_mm", low=bounds["bp_x_mm"][0] - boffset_hi)
            if self._pads_on_axis and not packaging.pads_off_pins:
                self._extended_pad_bounds(bounds, narrow)

    @property
    def searched_drops(self) -> tuple[str, ...]:
        """The pads' drops searched (``goal.pads.*.drop_mm``), not held at the packaging's."""

        return tuple(name for name in ("proximal_drop_mm", "distal_drop_mm")
                     if self.bounds[name][0] < self.bounds[name][1])

    @property
    def _pads_on_axis(self) -> bool:
        """Whether ``A``, ``B`` and both free-length pads lie on the ``x`` axis when extended."""

        packaging = self.packaging
        return (packaging.pad_link_ratio_max is None
                and self.bounds["proximal_drop_mm"] == (0.0, 0.0) and self.bounds["distal_drop_mm"] == (0.0, 0.0)
                and self.bounds["a_z_mm"] == (0.0, 0.0) and self.bounds["b_z_mm"] == (0.0, 0.0)
                and packaging.proximal_pad.length_mm is None and packaging.distal_pad.length_mm is None)

    def _extended_pad_bounds(self, bounds, narrow) -> None:
        """The extended pose's pad checks, linear when everything lies on the ``x`` axis.

        The proximal pad ends at ``A''_x = A'_x + offset``, the distal pad
        starts at ``B''_x = B'_x - offset``, both with their mounting edge on
        ``z = 0``, as are the ``A`` and ``B`` pins: the pads keep the clearance
        from each other, from the other body's pin, and the distal pad from
        the MCP servo's shaft disk.
        """

        packaging = self.packaging
        clearance = packaging.body_clearance_mm
        a_pin = packaging.pivot_radius_mm("a") + clearance  # A against the distal pad
        b_pin = packaging.pivot_radius_mm("b") + clearance  # B against the proximal pad
        pads = clearance if packaging.pad_pad_check else -math.inf
        (a_lo, _), (b_lo, b_hi) = bounds["a_x_mm"], bounds["b_x_mm"]
        (ap_lo, ap_hi), (bp_lo, bp_hi) = bounds["ap_x_mm"], bounds["bp_x_mm"]
        offset_lo, offset_hi = bounds["app_offset_mm"]
        app_lo = ap_lo + offset_lo  # least A''_x
        boffset_lo = bounds["bpp_offset_mm"][0]
        bpp_hi = bp_hi - boffset_lo  # most B''_x
        mcp = (packaging.mcp_servo.radius_mm + packaging.mcp_pad_clearance_mm
               if packaging.pad_checks_mcp("distal_pad") else -math.inf)
        narrow("bp_x_mm", low=max(app_lo + pads, a_lo + a_pin, mcp) + boffset_lo)
        narrow("ap_x_mm", high=min(bpp_hi - pads, b_hi - b_pin) - offset_lo)
        narrow("a_x_mm", high=bpp_hi - a_pin)
        narrow("b_x_mm", low=app_lo + b_pin)

    @property
    def lower(self) -> tuple[float, ...]:
        return tuple(self.search_bounds[name][0] for name in VARIABLE_NAMES)

    @property
    def upper(self) -> tuple[float, ...]:
        return tuple(self.search_bounds[name][1] for name in VARIABLE_NAMES)

    @property
    def mcp_servo_angles_rad(self) -> tuple[float, ...]:
        """MCP angles the two servos are checked at: MCP hyperextension to
        extension (flexion only turns the PIP servo away), or 0."""

        if not self.mcp_rotation_checked:
            return (0.0,)
        return tuple(np.linspace(-self.hyperextension_rad, 0.0, 3))

    @property
    def mcp_samples_rad(self) -> tuple[float, ...]:
        """MCP flexion of the pad to MCP servo check: from extension to
        ``mcp_flexion_max_rad``, every ``mcp_step_rad``."""

        if not self.mcp_rotation_checked:
            return (0.0,)
        count = int(math.floor(self.mcp_flexion_max_rad / self.mcp_step_rad + 1e-9)) + 1
        return tuple(index * self.mcp_step_rad for index in range(count))

    @property
    def servo_samples_rad(self) -> tuple[float, ...]:
        count = int(math.floor(self.servo_horizon_rad / self.servo_step_rad + 1e-9)) + 1
        return tuple(index * self.servo_step_rad for index in range(count))

    def length_ratio_violations(self, proximal_mm: float, distal_mm: float, span_mm: float,
                                finger_mm: float) -> list[float]:
        """How far (mm) ``|OA|``, ``BC`` and ``|AB|`` lie outside their shares
        of the extended ``|OC|`` (``goal.link.*_ratio``), one value per bound
        set (each mechanism's constraint: none positive)."""

        violations = []
        for value, (low, high) in ((proximal_mm, self.proximal_length_ratio_bounds),
                                   (distal_mm, self.distal_length_ratio_bounds),
                                   (span_mm, self.joint_span_ratio_bounds)):
            violations += [v for v in (None if low is None else low * finger_mm - value,
                                       None if high is None else value - high * finger_mm) if v is not None]
        return violations

    def icr0_violations(self, icr_x_mm: float | None, finger_mm: float) -> list[float]:
        """How far (mm) the extended centre of rotation at ``icr_x_mm`` lies
        outside ``goal.link.icr0_share`` of ``|OC|`` (none: ``finger_mm``)."""

        low, high = self.icr0_share_bounds
        if low is None and high is None:
            return []
        if icr_x_mm is None or not math.isfinite(icr_x_mm):
            return [finger_mm]
        return [v for v in (None if low is None else low * finger_mm - icr_x_mm,
                            None if high is None else icr_x_mm - high * finger_mm) if v is not None]

    def within_share(self, bounds: tuple[float, float], ratio: tuple[float | None, float | None]) -> tuple[float, float]:
        """``bounds`` (mm) narrowed to what a share ``ratio`` of ``|OC|`` allows
        over the finger's range: a search box (the share itself is a constraint)."""

        (low, high), (finger_low, finger_high) = bounds, self.extended_length_bounds_mm
        low = low if ratio[0] is None else max(low, ratio[0] * finger_low)
        high = high if ratio[1] is None else min(high, ratio[1] * finger_high)
        return (low, high) if low <= high else bounds


def config_from_goal(values: Mapping[str, Any]) -> LinkageConfig:
    """A config from a ``goal`` / ``solver`` mapping, as written in the file
    (``scripts/optimize/linkage/config.yaml`` documents every entry). An
    unknown or missing entry is an error."""

    tree = _Table(values, "")
    goal, solver = tree.table("goal"), tree.table("solver")
    tree.done()
    motion, pads, link = goal.table("motion"), goal.table("pads"), goal.table("link")
    hardware, collision = goal.table("hardware"), goal.table("collision")
    loss = _Table(goal.values.pop("loss", {}), "goal.loss")  # optional: absent, the fold alone
    goal.done()
    integrity_depth = loss.values.pop("integrity_depth_mm", None)
    count_fold = loss.values.pop("fold", True)
    if not isinstance(count_fold, bool):
        raise ValueError("goal.loss.fold must be true or false")
    access_depth = float(loss.values.pop("access_depth_mm", 20.0))
    loss.done()
    angles = {name: math.radians(float(motion.take(name)))
              for name in ("dip_flexion_deg", "hyperextension_deg")}
    motion.done()

    objectives = tuple(solver.take("objectives"))
    unknown = set(objectives) - set(OBJECTIVE_NAMES)
    if unknown or len(set(objectives)) != len(objectives) or not objectives:
        raise ValueError(f"solver.objectives: list some of {OBJECTIVE_NAMES}, each once")

    finger_low, finger_high = _range(link.take("finger_length_mm"), "goal.link.finger_length_mm")
    if finger_high is None:
        raise ValueError("goal.link.finger_length_mm needs a max (it bounds every length)")
    finger = (0.0 if finger_low is None else finger_low, finger_high)
    # Optional (absent: no limit): the links may be bounded as shares instead.
    proximal_length = _range(link.values.pop("proximal_length_mm", [None, None]), "goal.link.proximal_length_mm")
    distal_low, distal_high = _range(link.values.pop("distal_length_mm", [None, None]),
                                     "goal.link.distal_length_mm")
    joint_span = _range(link.values.pop("joint_span_mm", [None, None]), "goal.link.joint_span_mm")
    span_ratio = _range(link.values.pop("joint_span_ratio", [None, None]), "goal.link.joint_span_ratio")
    proximal_ratio = _range(link.take("proximal_length_ratio"), "goal.link.proximal_length_ratio")
    distal_ratio = _range(link.take("distal_length_ratio"), "goal.link.distal_length_ratio")
    icr0_share = _range(link.values.pop("icr0_share", [None, None]), "goal.link.icr0_share")  # optional
    dorsal_space = link.values.pop("dorsal_space_mm", None)  # optional
    angle_low, angle_high = _range(link.take("transmission_angle_deg"), "goal.link.transmission_angle_deg")
    symmetric = link.take("symmetric_four_bar")
    if not isinstance(symmetric, bool):
        raise ValueError("goal.link.symmetric_four_bar must be true or false")
    link.done()
    angle = (math.radians(0.0 if angle_low is None else angle_low),
             math.radians(180.0 if angle_high is None else angle_high))

    on_links = pads.values.pop("on_links", True)  # the default since 2026-10-27
    if not isinstance(on_links, bool):
        raise ValueError("goal.pads.on_links must be true or false")
    pad_values, drops, overhangs = {}, {}, {}
    for side in ("proximal", "distal"):
        pad = pads.table(side)
        low, high = _range(pad.take("length_mm"), f"goal.pads.{side}.length_mm", scalar=True)
        if low is None or not low >= 0.0:
            raise ValueError(f"goal.pads.{side}.length_mm: the min must be a length, 0 or more")
        high = finger_high if high is None else high
        height = float(pad.take("height_mm"))
        if side == "proximal":
            past_icr = pad.take("past_icr_max_mm")
            past_icr = None if past_icr is None else float(past_icr)
        # Optional (absent: the packaging's drop, no overhang limit).
        drops[side] = _range(pad.values.pop("drop_mm"), f"goal.pads.{side}.drop_mm") if "drop_mm" in pad.values else None
        overhang = pad.values.pop("overhang_max_mm", None)
        overhangs[side] = None if overhang is None else float(overhang)
        pad.done()
        pad_values[side] = (low, high, ContactPad(height_mm=height, offset_mm=0.0, chamfer_mm=0.0,
                                                  length_mm=None, max_length_mm=high,
                                                  chamfer_free=False))  # plain blocks
    pads.done()

    pip_servo, mcp_servo = hardware.table("pip_servo"), hardware.table("mcp_servo")
    pulley = hardware.values.pop("pulley_radius_mm", None)  # optional
    hardware.done()
    pip_model = pip_servo.take("model")
    tilt_low, tilt_high = _range(pip_servo.take("tilt_angle_deg"), "goal.hardware.pip_servo.tilt_angle_deg",
                                 scalar=True)
    if tilt_low is None or tilt_high is None:
        raise ValueError("goal.hardware.pip_servo.tilt_angle_deg needs both ends")
    tilt_step = float(pip_servo.take("tilt_step_deg"))
    if not tilt_step > 0.0:
        raise ValueError("goal.hardware.pip_servo.tilt_step_deg must be positive")
    torque_ratio_max = pip_servo.take("torque_ratio_max")
    if torque_ratio_max is not None and not float(torque_ratio_max) > 0.0:
        raise ValueError("goal.hardware.pip_servo.torque_ratio_max must be positive, or null")
    pip_servo.done()
    mcp_model = mcp_servo.take("model")
    mcp_servo.done()

    param, check = collision.table("param"), collision.table("check")
    collision.done()
    diameters = param.table("pivot_diameter_mm")
    carrier_height = float(param.take("distal_carrier_height_mm"))
    link_radius = param.values.pop("link_radius_mm", None)  # optional
    param.done()
    pivot = {name: float(diameters.take(key)) for key, name in PIVOT_KEYS.items()}
    diameters.done()
    checks = {name: check.take(name) for name in COLLISION_CHECKS}
    check.done()
    for name, value in checks.items():
        if not isinstance(value, bool):
            raise ValueError(f"goal.collision.check.{name} must be true or false")
    packaging = Packaging(
        pip_servo=actuator(pip_model).geometry,
        mcp_servo=actuator(mcp_model).geometry,
        pivot_clearance_diameter_mm=pivot["a"],
        a_pivot_diameter_mm=pivot["a"], ap_pivot_diameter_mm=pivot["ap"],
        b_pivot_diameter_mm=pivot["b"], bp_pivot_diameter_mm=pivot["bp"],
        ap_clearance_diameter_mm=pivot["ap"],
        body_clearance_mm=0.0, pad_mcp_clearance_mm=0.0, overlap_depth=True,
        proximal_pad=pad_values["proximal"][2], distal_pad=pad_values["distal"][2],
        allow_collision_pip_servo_to_joint_a=not checks["a_pin_and_a_prime_pin"],
        a_pin_servo_check=checks["a_pin_and_pip_servo"],
        allow_collision_pip_servo_to_joint_bp=not checks["b_prime_pin_and_pip_servo"],
        proximal_pad_below_a=checks["a_pin_and_proximal_pad"],
        distal_pad_below_b=checks["b_pin_and_distal_pad"],
        pad_pad_check=checks["proximal_pad_and_distal_pad"],
        pads_off_pins=True,  # each pad clear of its own pins, ending anywhere (2026-10-27)
        links_clear_pads=True,  # the links bent around the pads: every face usable (2026-10-27)
        pip_servo_above_oa=False, pip_servo_pad_check=checks["pip_servo_and_proximal_pad"],
        pad_mcp_check=checks["mcp_servo_and_proximal_pad"], distal_pad_mcp_check=True,
        distal_carrier_height_mm=carrier_height if checks["distal_carrier_and_pip_servo"] else 0.0,
    )

    reference = solver.table("pareto_front")
    # Every listed objective has its entry; others may stay for another run.
    hypervolume = {name: reference.take(name) for name in objectives}
    hypervolume.update({name: reference.take(name) for name in OBJECTIVE_NAMES if name in reference.values})
    reference.done()
    search = solver.table("search_bounds")
    given = {name: _bounds(search.take(name), f"solver.search_bounds.{name}") for name in SEARCH_BOUNDS}
    relative = {name: given.pop(name) for name in RELATIVE_BOUNDS}
    bounds = dict(given)
    search.done()
    sampling = solver.table("sampling")
    steps = {name: math.radians(float(sampling.take(name)))
             for name in ("servo_step_deg", "verify_step_deg", "dip_backoff_tolerance_deg")}
    sampling.done()
    resolution = float(solver.take("prototyping_resolution_mm"))
    optimizer = solver.table("optimizer")
    seeds = tuple(int(seed) for seed in optimizer.take("seeds"))
    warm_start = tuple(str(path) for path in optimizer.values.pop("warm_start", None) or ())
    optimizer.done()
    solver.done()

    # The variables the goal bounds: |OA| (A_x), BC, and B_x within the finger.
    a_low, a_high = proximal_length
    bounds["a_x_mm"] = (resolution if a_low is None else a_low, finger_high if a_high is None else a_high)
    bounds["bc_length_mm"] = (resolution if distal_low is None else distal_low,
                              finger_high if distal_high is None else distal_high)
    bounds["b_x_mm"] = (0.0, finger_high)
    # The absolute box A' and B' span from A and B.
    for point, base, x_base, z_base in (("ap", "a", bounds["a_x_mm"], bounds["a_z_mm"]),
                                        ("bp", "b", bounds["b_x_mm"], bounds["b_z_mm"])):
        dx, dz = relative[f"{point}_from_{base}_x_mm"], relative[f"{point}_from_{base}_z_mm"]
        bounds[f"{point}_x_mm"] = (x_base[0] + dx[0], x_base[1] + dx[1])
        bounds[f"{point}_z_mm"] = (z_base[0] + dz[0], z_base[1] + dz[1])
    # The pads' drops when searched: [min, max], max null: the pad's own pivot's radius.
    for side, pivot in (("proximal", "a"), ("distal", "b")):
        if drops[side] is not None:
            low, high = drops[side]
            bounds[f"{side}_drop_mm"] = (0.0 if low is None else low,
                                         packaging.pivot_radius_mm(pivot) if high is None else high)
    tilt_rad = None
    if tilt_low == tilt_high:  # fixed
        tilt_rad = math.radians(tilt_low)
    else:  # searched in whole degrees from the least (linkage.search)
        bounds["servo_tilt_deg"] = (tilt_low, tilt_high)

    return LinkageConfig(
        bounds=bounds,
        packaging=packaging,
        required_rom_rad=angles["dip_flexion_deg"],
        hyperextension_rad=angles["hyperextension_deg"],
        mcp_flexion_max_rad=0.0,
        mcp_rotation_checked=False,
        transmission_angle_range_rad=angle,
        torque_ratio_max=None if torque_ratio_max is None else float(torque_ratio_max),
        extended_length_bounds_mm=finger,
        proximal_length_bounds_mm=proximal_length,
        joint_span_bounds_mm=joint_span,
        proximal_length_ratio_bounds=proximal_ratio,
        distal_length_ratio_bounds=distal_ratio,
        joint_span_ratio_bounds=span_ratio,
        minimum_proximal_support_mm=pad_values["proximal"][0],
        minimum_distal_support_mm=pad_values["distal"][0],
        pip_servo_tilt_rad=tilt_rad,
        servo_step_rad=steps["servo_step_deg"],
        verify_step_rad=steps["verify_step_deg"],
        dip_backoff_tolerance_rad=steps["dip_backoff_tolerance_deg"],
        resolution_mm=resolution,
        link_length_resolution_mm=resolution,
        objectives=objectives,
        hypervolume_reference=hypervolume,
        seeds=seeds,
        tilt_step_deg=tilt_step,
        symmetric_four_bar=symmetric,
        proximal_pad_past_icr_max_mm=past_icr,
        pads_on_links=on_links,
        warm_start=warm_start,
        relative_bounds=relative,
        proximal_overhang_max_mm=overhangs["proximal"],
        distal_overhang_max_mm=overhangs["distal"],
        pulley_radius_mm=None if pulley is None else float(pulley),
        icr0_share_bounds=icr0_share,
        dorsal_space_mm=None if dorsal_space is None else float(dorsal_space),
        link_radius_mm=None if link_radius is None else float(link_radius),
        count_fold=count_fold,
        integrity_depth_mm=None if integrity_depth is None else float(integrity_depth),
        access_depth_mm=access_depth,
    )


class _Table:
    """One table of the file: entries are taken by name, and :meth:`done`
    rejects any left over (a misspelt entry is an error, not a default)."""

    def __init__(self, values: Any, path: str) -> None:
        if not isinstance(values, Mapping):
            raise ValueError(f"{path or 'the config'} must be a table")
        self.values, self.path = dict(values), path

    def _name(self, key: str) -> str:
        return f"{self.path}.{key}" if self.path else key

    def take(self, key: str) -> Any:
        if key not in self.values:
            raise ValueError(f"missing config entry {self._name(key)}")
        return self.values.pop(key)

    def table(self, key: str) -> "_Table":
        return _Table(self.take(key), self._name(key))

    def done(self) -> None:
        if self.values:
            raise ValueError(f"unknown config entries {sorted(self._name(key) for key in self.values)}")


def _range(value: Any, name: str, scalar: bool = False) -> tuple[float | None, float | None]:
    """``[min, max]`` (``None``: no limit), or with ``scalar`` one number for both."""

    if scalar and isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value), float(value)
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"{name} must be [min, max]" + (" or a number" if scalar else ""))
    low, high = (None if item is None else float(item) for item in value)
    if low is not None and high is not None and low > high:
        raise ValueError(f"{name}: min exceeds max")
    return low, high


def _bounds(value: Any, name: str) -> tuple[float, float]:
    low, high = _range(value, name)
    if low is None or high is None:
        raise ValueError(f"{name} needs both ends")
    return low, high


def packaging_from_mapping(values: Mapping[str, object]) -> Packaging:
    """A :class:`Packaging` from plain values: servos by actuator name (e.g.
    ``xm335``, ``core.hardware.ACTUATORS``) or as tables, pads as tables."""

    packaging = dict(values)
    if "pin_envelope_diameter_mm" in packaging:  # its earlier name (runs before 2026-10-21)
        packaging["pivot_clearance_diameter_mm"] = packaging.pop("pin_envelope_diameter_mm")
    for joint in ("A", "Bp"):  # as written in a config
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


TUPLE_FIELDS = ("transmission_angle_range_rad", "extended_length_bounds_mm", "proximal_length_bounds_mm",
                "joint_span_bounds_mm", "proximal_length_ratio_bounds", "distal_length_ratio_bounds",
                "joint_span_ratio_bounds", "icr0_share_bounds",
                "objectives", "seeds", "warm_start")


def config_to_json(config: LinkageConfig) -> dict:
    """The config as JSON values, for ``run.json`` (with the search box it used)."""

    values = {f.name: getattr(config, f.name) for f in fields(config)}
    values["bounds"] = dict(config.bounds)
    values["search_bounds"] = dict(config.search_bounds)
    values["packaging"] = asdict(config.packaging)
    values["hypervolume_reference"] = dict(config.hypervolume_reference)
    values["relative_bounds"] = None if config.relative_bounds is None else dict(config.relative_bounds)
    return values


def config_from_json(values: dict) -> LinkageConfig:
    """Rebuild a config recorded by :func:`config_to_json`. Runs recorded as
    ``linkage_v3`` (2026-10-23 to 26) carry fields of the formulations it was
    built on, all at the values this one fixes; they are dropped."""

    known = {f.name for f in fields(LinkageConfig)}
    values = {name: value for name, value in values.items() if name in known}
    values["bounds"] = {name: tuple(bounds) for name, bounds in values["bounds"].items()}
    for name in TUPLE_FIELDS:
        if values.get(name) is not None:
            values[name] = tuple(values[name])
    values["packaging"] = packaging_from_mapping(values["packaging"])
    if values.get("relative_bounds") is not None:
        values["relative_bounds"] = {name: tuple(pair) for name, pair in values["relative_bounds"].items()}
    return LinkageConfig(**values)


__all__ = [
    "COLLISION_CHECKS",
    "OBJECTIVE_NAMES",
    "RELATIVE_BOUNDS",
    "SEARCH_BOUNDS",
    "LinkageConfig",
    "config_from_goal",
    "config_from_json",
    "config_to_json",
    "packaging_from_mapping",
]
