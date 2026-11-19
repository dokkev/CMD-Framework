"""Read a ``goal`` / ``solver`` file into a :class:`~core.optimization.linkage.config.LinkageConfig`.

The file (``scripts/optimize/prototype/config.yaml``) has two levels, by who
sets them: ``goal``, what the finger must do, carry and keep clear (the
problem formulation's requirements), and ``solver``, how the design is
searched. Every entry is checked: an unknown or missing one is an error.
Fixed in the code, not options: the fingertip stays ahead of the MCP, the
PIP servo's shaft is on ``A'``, the servos at their model's nominal size, no
clearance anywhere (touching is allowed, overlapping is not).
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from core.hardware import actuator
from core.morphology.mechanism.linkage.packaging import LinkagePackaging
from core.morphology.packaging.hardware import ContactPad
from core.optimization.linkage.config import (
    COLLISION_DEFAULTS,
    COLLISION_CHECKS,
    DEPRECATED_CHECKS,
    DEPRECATED_PARAMS,
    MOUNT_POLICIES,
    OBJECTIVE_NAMES,
    PIVOT_KEYS,
    PROFILE_DEFAULT_LINKAGE_OCCLUSION,
    RELATIVE_BOUNDS,
    SEARCH_BOUNDS,
    LinkageConfig,
    resolve_linkage_occlusion,
)


def config_from_goal(values: Mapping[str, Any]) -> LinkageConfig:
    """A config from a ``goal`` / ``solver`` mapping, as written in the file
    (``scripts/optimize/prototype/config.yaml`` documents every entry). An
    unknown or missing entry is an error."""

    tree = _Table(values, "")
    profile = tree.values.pop("profile", "prototype")
    if profile not in PROFILE_DEFAULT_LINKAGE_OCCLUSION:
        raise ValueError(
            f"profile must be one of {tuple(PROFILE_DEFAULT_LINKAGE_OCCLUSION)}, not {profile!r}"
        )
    goal, solver = tree.table("goal"), tree.table("solver")
    tree.done()
    motion, pads, link = goal.table("motion"), goal.table("pads"), goal.table("link")
    alignment, transmission = goal.table("alignment"), goal.table("transmission")
    align_link, align = alignment.take("joint_axes"), alignment.take("contact_faces")
    alignment.done()
    for name, value in (("joint_axes", align_link), ("contact_faces", align)):
        if not isinstance(value, bool):
            raise ValueError(f"goal.alignment.{name} must be true or false")
    if "align_mount_offsets" in pads.values:
        raise ValueError("goal.pads.align_mount_offsets moved to goal.alignment.contact_faces")
    hardware, collision = goal.table("hardware"), goal.table("collision")
    loss = _Table(goal.values.pop("loss", {}), "goal.loss")  # optional: absent, profile defaults apply
    goal.done()
    for removed in ("integrity_depth_mm", "fold"):
        if removed in loss.values:
            raise ValueError(f"goal.loss.{removed} was removed (the cylinder-based coverage replaces it)")
    access_depth = float(loss.values.pop("access_depth_mm", 20.0))
    raw_occlusion = loss.values.pop("include_linkage_occlusion", None)
    if raw_occlusion is not None and not isinstance(raw_occlusion, bool):
        raise ValueError("goal.loss.include_linkage_occlusion must be true or false")
    include_linkage_occlusion = resolve_linkage_occlusion(raw_occlusion, profile=profile)
    loss.done()
    if "hyperextension_deg" in motion.values:
        raise ValueError("goal.motion.hyperextension_deg was removed (no hyperextension requirement)")
    if "dip_flexion_deg" in motion.values:
        raise ValueError("goal.motion.dip_flexion_deg was renamed goal.motion.required_flexion_deg (q_req)")
    required_flexion = math.radians(float(motion.take("required_flexion_deg")))
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
    # Optional (absent: no limit): |OA| and |BC| as shares of the extended |OC|.
    oa_ratio = _range(link.values.pop("oa_ratio", [None, None]), "goal.link.oa_ratio")
    bc_ratio = _range(link.values.pop("bc_ratio", [None, None]), "goal.link.bc_ratio")
    dorsal_space = link.values.pop("dorsal_space_mm", None)  # optional
    if "transmission_angle_deg" in link.values:
        raise ValueError("goal.link.transmission_angle_deg moved to goal.transmission.angle_deg")
    # Optional (absent: false): the four-bar an antiparallelogram.
    symmetric = link.values.pop("symmetric_four_bar", False)
    if not isinstance(symmetric, bool):
        raise ValueError("goal.link.symmetric_four_bar must be true or false")
    link.done()
    # The CFL's transmission constraints (problem formulation A.5.2): gamma's
    # range and the least worst-case mechanical advantage.
    angle_low, angle_high = _range(transmission.take("angle_deg"), "goal.transmission.angle_deg")
    angle = (math.radians(0.0 if angle_low is None else angle_low),
             math.radians(180.0 if angle_high is None else angle_high))
    ma_min = transmission.take("ma_min")
    if ma_min is not None and not float(ma_min) > 0.0:
        raise ValueError("goal.transmission.ma_min must be positive, or null")
    transmission.done()

    on_links = pads.values.pop("on_links", False)  # the pads' ends searched (2026-11-03; on links 2026-10-27 to then)
    if not isinstance(on_links, bool):
        raise ValueError("goal.pads.on_links must be true or false")
    extend_past = pads.values.pop("extend_past_pivots", False)  # optional (absent: the pads may stop short)
    if not isinstance(extend_past, bool):
        raise ValueError("goal.pads.extend_past_pivots must be true or false")
    mount_policy = pads.values.pop("mount_policy", "searched")
    if mount_policy not in MOUNT_POLICIES:
        raise ValueError(f"goal.pads.mount_policy must be one of {MOUNT_POLICIES}")
    pad_values, overhangs, gaps, mount_offsets = {}, {}, {}, {}
    for side in ("proximal", "distal"):
        pad = pads.table(side)
        low, high = _range(pad.take("length_mm"), f"goal.pads.{side}.length_mm", scalar=True)
        if low is None or not low >= 0.0:
            raise ValueError(f"goal.pads.{side}.length_mm: the min must be a length, 0 or more")
        high = finger_high if high is None else high
        height = float(pad.take("height_mm"))
        if side == "proximal":
            past_icr = pad.values.pop("past_icr_max_mm", None)  # optional (absent: no limit)
            past_icr = None if past_icr is None else float(past_icr)
        pad.values.pop("drop_mm", None)  # removed 2026-11-03: the space starts at a set line (the link's)
        # The mounting surface's offset below the link line, searched in [min, max]
        # (2026-11-04; a number: held at it; absent: on the line).
        raw = pad.values.pop("mount_offset_mm", 0.0)
        mount_offsets[side] = _bounds(raw if isinstance(raw, (list, tuple)) else [raw, raw],
                                      f"goal.pads.{side}.mount_offset_mm")
        if mount_offsets[side][0] < 0.0:
            raise ValueError(f"goal.pads.{side}.mount_offset_mm: an offset is 0 or more (palmar of the line)")
        # Optional (absent: no overhang limit).
        overhang = pad.values.pop("overhang_max_mm", None)
        overhangs[side] = None if overhang is None else float(overhang)
        # Optional (absent: no limit): the pad's inner end within this of its pivot.
        gap = pad.values.pop("pivot_gap_max_mm", None)
        gaps[side] = None if gap is None else float(gap)
        pad.done()
        pad_values[side] = (low, high, ContactPad(height_mm=height, offset_mm=0.0, length_mm=None,
                                                  max_length_mm=high))
    pads.done()
    if align:  # the faces at one depth: the distal offset the proximal one shifted by the heights' difference
        shift = pad_values["proximal"][2].height_mm - pad_values["distal"][2].height_mm
        low = max(mount_offsets["proximal"][0], mount_offsets["distal"][0] - shift)
        high = min(mount_offsets["proximal"][1], mount_offsets["distal"][1] - shift)
        if high >= low:  # else the ranges stay as given: LinkageConfig.alignment_warning stops the run
            mount_offsets = {"proximal": (low, high), "distal": (low + shift, high + shift)}

    pip_servo, mcp_servo = hardware.table("pip_servo"), hardware.table("mcp_servo")
    pulley = hardware.values.pop("pulley_radius_mm", None)  # optional
    hardware.done()
    pip_model = pip_servo.take("model")
    # A range only: [a, a] holds the tilt at a (code_formulation.md §1, principle 5).
    tilt_low, tilt_high = _range(pip_servo.take("tilt_angle_deg"), "goal.hardware.pip_servo.tilt_angle_deg")
    if tilt_low is None or tilt_high is None:
        raise ValueError("goal.hardware.pip_servo.tilt_angle_deg needs both ends")
    tilt_step = float(pip_servo.values.pop("tilt_step_deg", 5.0))  # optional: a searched tilt's grid
    if not tilt_step > 0.0:
        raise ValueError("goal.hardware.pip_servo.tilt_step_deg must be positive")
    if "torque_ratio_max" in pip_servo.values:
        raise ValueError("goal.hardware.pip_servo.torque_ratio_max was replaced by ma_min (MA_wc >= ma_min)")
    mcp_gap_max = pip_servo.values.pop("mcp_gap_max_mm", None)  # optional (absent: no limit)
    mcp_gap_max = None if mcp_gap_max is None else float(mcp_gap_max)
    if "ma_min" in pip_servo.values:
        raise ValueError("goal.hardware.pip_servo.ma_min moved to goal.transmission.ma_min")
    pip_servo.done()
    mcp_model = mcp_servo.take("model")
    mcp_servo.done()

    # The pairs checked follow the problem formulation (A.4); a goal may
    # still set one, for a diagnosis (absent: COLLISION_DEFAULTS).
    param, check = collision.table("param"), _Table(collision.values.pop("check", {}), "goal.collision.check")
    collision.done()
    diameters = param.table("pivot_diameter_mm")
    link_radius = param.values.pop("link_radius_mm", None)  # optional
    for name in DEPRECATED_PARAMS:
        param.values.pop(name, None)
    param.done()
    pivot = {name: float(diameters.take(key)) for key, name in PIVOT_KEYS.items()}
    diameters.done()
    for name in DEPRECATED_CHECKS:
        check.values.pop(name, None)
    checks = {name: check.values.pop(name, COLLISION_DEFAULTS[name]) for name in COLLISION_CHECKS}
    check.done()
    for name, value in checks.items():
        if not isinstance(value, bool):
            raise ValueError(f"goal.collision.check.{name} must be true or false")
    packaging = LinkagePackaging(
        pip_servo=actuator(pip_model).geometry,
        mcp_servo=actuator(mcp_model).geometry,
        pivot_clearance_diameter_mm=pivot["a"],
        a_pivot_diameter_mm=pivot["a"], ap_pivot_diameter_mm=pivot["ap"],
        b_pivot_diameter_mm=pivot["b"], bp_pivot_diameter_mm=pivot["bp"],
        ap_clearance_diameter_mm=pivot["ap"],
        pad_mcp_clearance_mm=0.0, overlap_depth=True,
        proximal_pad=pad_values["proximal"][2], distal_pad=pad_values["distal"][2],
        allow_collision_pip_servo_to_joint_a=not checks["a_pin_and_a_prime_pin"],
        a_pin_servo_check=checks["a_pin_and_pip_servo"],
        allow_collision_pip_servo_to_joint_bp=not checks["b_prime_pin_and_pip_servo"],
        pin_a_b_check=checks["a_pin_and_b_pin"],
        pad_pad_check=checks["proximal_pad_and_distal_pad"],
        pads_off_pins=True,  # each pad clear of its own pins, ending anywhere (2026-10-27)
        links_clear_pads=True,  # the links bent around the pads: every face usable (2026-10-27)
        pip_servo_above_oa=False, pip_servo_pad_check=checks["pip_servo_and_proximal_pad"],
        pad_mcp_check=checks["mcp_servo_and_proximal_pad"], distal_pad_mcp_check=True,
    )

    # The hypervolume reference point (optional: absent, 0 for every objective).
    reference = _Table(solver.values.pop("pareto_front", {}), "solver.pareto_front")
    hypervolume = {name: reference.values.pop(name, 0.0) for name in objectives}
    hypervolume.update({name: reference.values.pop(name) for name in OBJECTIVE_NAMES if name in reference.values})
    reference.done()
    search = solver.table("search_bounds")
    # A and B on their phalanges' axes (goal.alignment.joint_axes): the
    # axes' offsets are not searched (absent, or [0, 0]).
    axis_names = ("a_z_mm", "b_z_mm")
    given = {name: _bounds(search.take(name), f"solver.search_bounds.{name}") for name in SEARCH_BOUNDS
             if not (align_link and name in axis_names)}
    if align_link:
        for name in axis_names:
            found = search.values.pop(name, None)
            if found is not None and _bounds(found, f"solver.search_bounds.{name}") != (0.0, 0.0):
                raise ValueError(f"solver.search_bounds.{name}: goal.alignment.joint_axes puts it at 0")
            given[name] = (0.0, 0.0)
    relative = {name: given.pop(name) for name in RELATIVE_BOUNDS}
    bounds = dict(given)
    search.done()
    sampling = solver.table("sampling")
    steps = {name: math.radians(float(sampling.take(name))) for name in ("flexion_step_deg", "verify_step_deg")}
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
    # Their shares of |OC| (oa_ratio, bc_ratio) narrow the search to what the
    # finger's length range allows; the shares themselves stay constraints.
    for name, (low, high) in (("a_x_mm", oa_ratio), ("bc_length_mm", bc_ratio)):
        old_low, old_high = bounds[name]
        new_low = old_low if low is None else max(old_low, low * finger[0])
        new_high = old_high if high is None else min(old_high, high * finger_high)
        if new_high < new_low:
            raise ValueError(f"goal.link: {name} has no room within its share of the finger's length")
        bounds[name] = (new_low, new_high)
    bounds["b_x_mm"] = (0.0, finger_high)
    bounds["proximal_mount_offset_mm"], bounds["distal_mount_offset_mm"] = mount_offsets["proximal"], mount_offsets["distal"]
    # The absolute box A' and B' span from A and B.
    for point, base, x_base, z_base in (("ap", "a", bounds["a_x_mm"], bounds["a_z_mm"]),
                                        ("bp", "b", bounds["b_x_mm"], bounds["b_z_mm"])):
        dx, dz = relative[f"{point}_from_{base}_x_mm"], relative[f"{point}_from_{base}_z_mm"]
        bounds[f"{point}_x_mm"] = (x_base[0] + dx[0], x_base[1] + dx[1])
        bounds[f"{point}_z_mm"] = (z_base[0] + dz[0], z_base[1] + dz[1])
    tilt_rad = None
    if tilt_low == tilt_high:  # fixed
        tilt_rad = math.radians(tilt_low)
    else:  # searched in whole degrees from the least (linkage.search)
        bounds["servo_tilt_deg"] = (tilt_low, tilt_high)

    return LinkageConfig(
        bounds=bounds,
        packaging=packaging,
        required_rom_rad=required_flexion,
        mcp_flexion_max_rad=0.0,
        mcp_rotation_checked=False,
        transmission_angle_range_rad=angle,
        ma_min=None if ma_min is None else float(ma_min),
        extended_length_bounds_mm=finger,
        proximal_length_bounds_mm=proximal_length,
        joint_span_bounds_mm=joint_span,
        proximal_length_ratio_bounds=proximal_ratio,
        distal_length_ratio_bounds=distal_ratio,
        joint_span_ratio_bounds=span_ratio,
        minimum_proximal_support_mm=pad_values["proximal"][0],
        minimum_distal_support_mm=pad_values["distal"][0],
        pip_servo_tilt_rad=tilt_rad,
        flexion_step_rad=steps["flexion_step_deg"],
        verify_step_rad=steps["verify_step_deg"],
        resolution_mm=resolution,
        link_length_resolution_mm=resolution,
        objectives=objectives,
        hypervolume_reference=hypervolume,
        seeds=seeds,
        tilt_step_deg=tilt_step,
        symmetric_four_bar=symmetric,
        proximal_pad_past_icr_max_mm=past_icr,
        pads_on_links=on_links,
        align_joint_axes=align_link,
        align_contact_faces=align,
        mount_policy=mount_policy,
        warm_start=warm_start,
        relative_bounds=relative,
        proximal_overhang_max_mm=overhangs["proximal"],
        distal_overhang_max_mm=overhangs["distal"],
        pads_extend_past_pivots=extend_past,
        proximal_pivot_gap_max_mm=gaps["proximal"],
        distal_pivot_gap_max_mm=gaps["distal"],
        pulley_radius_mm=None if pulley is None else float(pulley),
        icr0_share_bounds=icr0_share,
        oa_ratio_bounds=oa_ratio,
        bc_ratio_bounds=bc_ratio,
        mcp_gap_max_mm=mcp_gap_max,
        dorsal_space_mm=None if dorsal_space is None else float(dorsal_space),
        link_radius_mm=None if link_radius is None else float(link_radius),
        profile=profile,
        include_linkage_occlusion=include_linkage_occlusion,
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


__all__ = ["config_from_goal"]
