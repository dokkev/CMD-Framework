"""The CFL's problem configuration: :class:`LinkageConfig`, the requirements,
search box and packaging every mechanism's evaluation reads (the 2R family
wraps it as its ``goal``).

It is read from a ``goal`` / ``solver`` file by
:func:`core.optimization.linkage.goal.config_from_goal` and recorded in
``run.json`` by :mod:`core.optimization.linkage.record`. Angles in radians,
lengths in mm.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Literal

from core._validation import require_finite, require_nonnegative, require_positive
from core.morphology.mechanism.linkage.packaging import LinkagePackaging
from core.morphology.mechanism.linkage.design import DEFAULT_VALUES, VARIABLE_NAMES


CoverageProfile = Literal["study", "prototype"]
PROFILE_DEFAULT_LINKAGE_OCCLUSION: Mapping[str, bool] = MappingProxyType({"study": False, "prototype": True})
"""Whether the crossing links hide the contact faces, by default, per profile."""


def resolve_linkage_occlusion(include_linkage_occlusion: bool | None, *, profile: CoverageProfile) -> bool:
    """``include_linkage_occlusion`` as given, or the profile's default when omitted."""

    if profile not in PROFILE_DEFAULT_LINKAGE_OCCLUSION:
        raise ValueError(f"profile must be one of {tuple(PROFILE_DEFAULT_LINKAGE_OCCLUSION)}, got {profile!r}")
    if include_linkage_occlusion is None:
        return PROFILE_DEFAULT_LINKAGE_OCCLUSION[profile]
    if not isinstance(include_linkage_occlusion, bool):
        raise ValueError("include_linkage_occlusion must be a boolean")
    return include_linkage_occlusion


OBJECTIVE_NAMES = ("c_pad_mean",)
"""The prototype formulation's one objective (problem formulation A.5.4)."""
MOUNT_POLICIES = ("searched", "shallowest_supported")
"""``goal.pads.mount_policy``: the mounting depths searched, or each the least its pad's own hardware needs."""
SEARCH_BOUNDS = ("a_z_mm", "b_z_mm", "ap_from_a_x_mm", "ap_from_a_z_mm", "bp_from_b_x_mm", "bp_from_b_z_mm",
                 "app_offset_mm", "bpp_offset_mm")
"""The design variables ``solver.search_bounds`` gives; the rest come from the goal."""
RELATIVE_BOUNDS = ("ap_from_a_x_mm", "ap_from_a_z_mm", "bp_from_b_x_mm", "bp_from_b_z_mm")
"""``A'`` from ``A`` and ``B'`` from ``B``, as searched (:mod:`~core.optimization.linkage.search`)."""
COLLISION_CHECKS = ("a_pin_and_pip_servo", "a_pin_and_a_prime_pin", "b_prime_pin_and_pip_servo",
                    "proximal_pad_and_distal_pad", "pip_servo_and_proximal_pad", "mcp_servo_and_proximal_pad")
DEPRECATED_CHECKS = ("a_pin_and_proximal_pad", "b_pin_and_distal_pad", "distal_carrier_and_pip_servo")
"""Checks removed 2026-11-03, ignored in a goal: the contact interface spaces
start at a set line (no drop to choose; their link's since 2026-11-04, each
pad kept off its own pins by ``pads_off_pins``), and the distal carrier is gone."""
DEPRECATED_PARAMS = ("distal_carrier_height_mm", "proximal_carrier_height_mm")
"""The carriers' plates (removed 2026-11-03), ignored in a goal."""
DEPRECATED_VARIABLES = ("proximal_chamfer_mm", "distal_chamfer_mm", "proximal_drop_mm", "distal_drop_mm")
DEPRECATED_PAD_FIELDS = ("chamfer_mm", "chamfer_free")
"""The pads' chamfers (deprecated 2026-10-30, plain blocks since): left out
of the bounds and pads recorded before. (``DEPRECATED_VARIABLES`` also has
the pads' drops, removed 2026-11-03.)"""
DEPRECATED_PACKAGING_FIELDS = ("pip_servo_at", "proximal_pad_below_a", "distal_pad_below_b")
"""The PIP servo's place (deprecated 2026-10-30: always on ``A'``, the CoDe's)
and the pads hung below their pins or not (2026-11-03: always; since
2026-11-04 every pad is mounted on its link line, the pins collisions)."""
PIVOT_KEYS = MappingProxyType({"a": "a", "a_prime": "ap", "b": "b", "b_prime": "bp"})
"""``goal.collision.param.pivot_diameter_mm`` keys and the packaging's pivot names."""


@dataclass(frozen=True)
class LinkageConfig:
    """One study: the search box, the requirements and the packaging.

    ``bounds`` are the search box as given; :attr:`search_bounds` narrows it
    where a constraint has a closed form. Angles in radians, lengths in mm.
    """

    bounds: Mapping[str, tuple[float, float]]
    packaging: LinkagePackaging = field(default_factory=LinkagePackaging)
    required_rom_rad: float = math.radians(150.0)
    """``goal.motion.dip_flexion_deg``: the rotation of ``BC`` relative to
    ``OA`` (``phi = q_pip + q_dip``) every design must reach safely; ``k_max``
    and the effective pads are taken on ``[0, required_rom_rad]``."""
    mcp_flexion_max_rad: float = math.radians(90.0)
    """MCP flexion up to which the pads must clear the MCP servo (with
    :attr:`mcp_rotation_checked`)."""
    mcp_rotation_checked: bool = True
    """Check the pads against the MCP servo over the MCP flexion, and the two
    servos over MCP hyperextension. Off (the goal file, since 2026-10-26):
    everything at MCP 0. On: runs recorded before then."""
    transmission_angle_range_rad: tuple[float, float] = (math.radians(30.0), math.radians(150.0))
    """The transmission angle at ``B'`` stays within it over the whole motion."""
    ma_min: float | None = None
    """``goal.hardware.pip_servo.ma_min``: the least worst-case mechanical
    advantage ``MA_wc`` on ``[0, q_req]`` (CFL only; ``None``: no limit)."""
    extended_length_bounds_mm: tuple[float, float] = (90.0, 110.0)
    """``|OC|`` in the extended pose."""
    proximal_length_bounds_mm: tuple[float | None, float | None] = (None, None)
    """``|OA|`` (``None``: no limit)."""
    proximal_length_ratio_bounds: tuple[float | None, float | None] = (None, None)
    distal_length_ratio_bounds: tuple[float | None, float | None] = (None, None)
    joint_span_ratio_bounds: tuple[float | None, float | None] = (None, None)
    """The finger's segments (``|OA''|``, ``|B''C|``; 2026-11-03, before:
    ``|OA|``, ``BC``) and ``|AB|`` as shares of the extended ``|OC|``
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
    flexion_step_rad: float = math.radians(1.0)
    """The uniform ``q_2`` grid step of the motion (``code_formulation.md`` §6)."""
    verify_step_rad: float = math.radians(0.05)
    """Flexion step the front is verified on."""
    mcp_step_rad: float = math.radians(1.0)
    resolution_mm: float = 0.1
    link_length_resolution_mm: float | None = 0.1
    """The prototyping resolution: the design grid, and the bars ``|A'B|``,
    ``|AB'|`` and ``B'`` from ``B`` rounded to it (``design.decode``)."""
    objectives: tuple[str, ...] | None = None
    """Objectives to optimize, by name; ``None``: the mechanism's defaults."""
    hypervolume_reference: Mapping[str, float] | None = None
    """Per objective, in its own terms; ``c_pad_mean`` defaults to 0."""
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
    mount_policy: str = "searched"
    """How each pad's mounting depth below its reference line is set
    (``goal.pads.mount_policy``, 2026-11-04): ``searched``, a design variable
    on ``mount_offset_mm``; ``shallowest_supported``, not searched but the
    least depth (on the resolution grid, at most ``mount_offset_mm``'s max)
    that clears the pad's own joint hardware, within what that hardware
    supports (``core.optimization.mounting``; with ``align_mount_offsets``,
    both at the deeper of the two)."""
    align_mount_offsets: bool = False
    """The two pads' mounting offsets held equal (``goal.pads.align_mount_offsets``,
    2026-11-04): one offset searched on both ranges' overlap, both pads' mounting
    surfaces then the same distance below their link lines. False: each its own."""
    pads_on_links: bool = False
    """Searched with each pad on its own link only (``goal.pads.on_links``;
    false, the default since 2026-11-03, searches the pads' ends ``A''`` and
    ``B''`` with the links, so the optimizer trades a pad grown past its
    joint against the curl it costs):
    the proximal pad to ``A``, the distal one from ``B`` to ``C``, so more pad
    means a shorter four-bar. ``scripts/optimize/extend_pads.py``
    (``core.morphology.mechanism.linkage.pad_extension``) then grows the pads of the
    designs found as far as they keep usable."""
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
    may lie, as a share of ``|OC|`` from ``O`` (``core.morphology.icr``;
    the bending-point control runs); ``(None, None)``: anywhere."""
    dorsal_space_mm: float | None = None
    """``goal.link.dorsal_space_mm``: how far the extended finger's own solids
    may rise above the common datum, the line through ``O`` along the finger
    (the pads below it; the MCP servo, in the palm, left
    out; ``core.morphology.contact.loss.dorsal_space_mm``); ``None``: any."""
    link_radius_mm: float | None = None
    """``goal.collision.param.link_radius_mm``: the links and bars as capsules
    of this radius, each checked against the pads it does not carry, in the
    side view (the comparison model); ``None``: the links pass beside the
    pads (the outer layer), unchecked."""
    profile: CoverageProfile = "prototype"
    """Explicit formulation context: ``"prototype"`` (default ``include_linkage_occlusion=True``)
    or ``"study"`` (default ``include_linkage_occlusion=False``)."""
    include_linkage_occlusion: bool | None = None
    """``goal.loss.include_linkage_occlusion``: whether crossing transmission members
    (``CoDe`` bars ``A-B'``, ``A'-B``; ``2R-remote`` belt and pulleys) occlude
    contact faces in coverage evaluation. Resolved explicitly from :attr:`profile`
    when ``None``; independent of collision checks and fold loss."""
    access_depth_mm: float = 20.0
    """``goal.loss.access_depth_mm``: the depth of the accessibility profile
    (reports only, not the objective)."""

    def __post_init__(self) -> None:
        object.__setattr__(self, "include_linkage_occlusion",
                           resolve_linkage_occlusion(self.include_linkage_occlusion, profile=self.profile))
        given = {name: value for name, value in self.bounds.items() if name not in DEPRECATED_VARIABLES}
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
        for name in ("required_rom_rad", "flexion_step_rad", "verify_step_rad", "mcp_step_rad", "resolution_mm"):
            require_positive(name, getattr(self, name))
        for name in ("mcp_flexion_max_rad", "minimum_proximal_support_mm", "minimum_distal_support_mm"):
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
        reference = {"c_pad_mean": 0.0}
        object.__setattr__(self, "hypervolume_reference", MappingProxyType({**reference, **given}))
        for name in ("proximal_overhang_max_mm", "distal_overhang_max_mm", "pulley_radius_mm",
                     "dorsal_space_mm", "link_radius_mm"):
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
    def _pads_on_axis(self) -> bool:
        """Whether ``A``, ``B`` and both free-length pads lie on the ``x`` axis when extended."""

        packaging = self.packaging
        return (packaging.pad_link_ratio_max is None
                and packaging.interface_offset_mm("proximal_pad") == 0.0
                and packaging.interface_offset_mm("distal_pad") == 0.0
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
        clearance = 0.0
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
        """MCP angles the two servos are checked at: the formulation holds ``q_1 = 0``."""

        return (0.0,)

    @property
    def mcp_samples_rad(self) -> tuple[float, ...]:
        """MCP flexion of the pad to MCP servo check: from extension to
        ``mcp_flexion_max_rad``, every ``mcp_step_rad``."""

        if not self.mcp_rotation_checked:
            return (0.0,)
        count = int(math.floor(self.mcp_flexion_max_rad / self.mcp_step_rad + 1e-9)) + 1
        return tuple(index * self.mcp_step_rad for index in range(count))

    @property
    def flexion_samples_rad(self) -> tuple[float, ...]:
        """The uniform ``q_2`` grid from extension to pi."""

        count = int(math.floor(math.pi / self.flexion_step_rad + 1e-9)) + 1
        return tuple(index * self.flexion_step_rad for index in range(count))

    def module_sets_segment(self, side: str) -> bool:
        """Whether a contact module requirement (``goal.pads.<side>.length_mm``'s
        min, or a fixed length) bounds the ``side`` segment from below: then it,
        not the segment's ratio, sets that bound (the requirement hierarchy,
        2026-11-04: finger, then the contact modules, then the segments'
        proportions, then the mechanism; physical feasibility always)."""

        minimum = self.minimum_proximal_support_mm if side == "proximal" else self.minimum_distal_support_mm
        return minimum > 0.0 or getattr(self.packaging, f"{side}_pad").length_mm is not None

    def length_ratio_violations(self, proximal_mm: float, distal_mm: float, span_mm: float,
                                finger_mm: float) -> list[float]:
        """How far (mm) the proximal and distal segments and the span lie outside their shares
        of the extended ``|OC|`` (``goal.link.*_ratio``), one value per bound
        set (each mechanism's constraint: none positive). A segment's least
        share is left out where a contact module sets that segment's bound
        (:meth:`module_sets_segment`; :meth:`length_ratio_shortfalls` reports it)."""

        violations = []
        for side, value, (low, high) in (("proximal", proximal_mm, self.proximal_length_ratio_bounds),
                                         ("distal", distal_mm, self.distal_length_ratio_bounds),
                                         (None, span_mm, self.joint_span_ratio_bounds)):
            if side is not None and self.module_sets_segment(side):
                low = None  # the module's requirement wins: the proportion a preference here
            violations += [v for v in (None if low is None else low * finger_mm - value,
                                       None if high is None else value - high * finger_mm) if v is not None]
        return violations

    def length_ratio_shortfalls(self, proximal_mm: float, distal_mm: float, finger_mm: float) -> tuple[float, float]:
        """How far (mm, 0 or more) each segment falls short of its least share
        of ``|OC|`` where a contact module relaxed it (else 0: a constraint
        then): how far the anatomical proportion gave way to the module."""

        shortfalls = []
        for side, value, (low, _) in (("proximal", proximal_mm, self.proximal_length_ratio_bounds),
                                      ("distal", distal_mm, self.distal_length_ratio_bounds)):
            relaxed = low is not None and self.module_sets_segment(side)
            shortfalls.append(max(low * finger_mm - value, 0.0) if relaxed else 0.0)
        return shortfalls[0], shortfalls[1]

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


__all__ = [
    "COLLISION_CHECKS",
    "DEPRECATED_CHECKS",
    "MOUNT_POLICIES",
    "OBJECTIVE_NAMES",
    "PROFILE_DEFAULT_LINKAGE_OCCLUSION",
    "RELATIVE_BOUNDS",
    "SEARCH_BOUNDS",
    "CoverageProfile",
    "LinkageConfig",
    "resolve_linkage_occlusion",
]
