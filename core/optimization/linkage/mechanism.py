"""The four-bar linkage finger (PIP servo at ``A'``) as a pluggable mechanism.

Its study is the ``goal`` / ``solver`` file (``scripts/optimize/linkage/config.yaml``,
:func:`~core.optimization.linkage.config.config_from_goal`); the objectives
are any of ``transmission`` (``k_max``), ``pad_coverage`` (the effective pads
over ``|OC|``), ``curl`` (``180 - phi_max``) and ``finger_length``.
"""

from __future__ import annotations

import copy
import dataclasses
import math
from collections.abc import Mapping, Sequence
from types import MappingProxyType
from typing import Any

from core.morphology.linkage import FingerLinkageParameters
from core.morphology.linkage.design import DEFAULT_VALUES, VARIABLE_NAMES, decode, read_design
from core.morphology.linkage.measures import extended_icr_x

from core.optimization.linkage.config import LinkageConfig, config_from_goal, config_from_json, config_to_json
from core.optimization.linkage import search
from core.optimization.linkage.evaluate import CONSTRAINT_NAMES, LinkageEvaluation, evaluate
from core.optimization.mechanism import Mechanism, Objective, SummaryColumn
from core.optimization.optimizer import OptimizerSettings


OBJECTIVES = MappingProxyType({
    "transmission": Objective(
        value=lambda e: e.torque_ratio_max, column="torque_ratio_max",
        label="worst torque ratio k_max on [0, dip_flexion]", role="best_transmission",
        larger_is_better=False),
    "pad_coverage": Objective(
        value=lambda e: -e.pad_coverage, column="pad_coverage",
        label="effective pad coverage on [0, dip_flexion]", role="most_pad", larger_is_better=True),
    "curl": Objective(
        value=lambda e: e.curl_deg, column="curl_deg",
        label="180 deg - phi_max", role="most_curl", larger_is_better=False),
    "finger_length": Objective(
        value=lambda e: e.finger_length_mm, column="finger_length_mm",
        label="finger length |OC| [mm]", role="shortest", larger_is_better=False, default=False),
    "support_balance": Objective(
        value=lambda e: -(e.support_balance_mm + SUPPORT_TIE * (e.proximal_room_mm + e.distal_room_mm)),
        column="support_balance_mm", label="min(S_p - l_p, S_d - l_d), the total support breaking ties [mm]",
        role="most_support", larger_is_better=True, default=False),
})
SUPPORT_TIE = 1e-4
"""Weight of the total support length in ``support_balance``: below the
0.1 mm length grid over any finger's supports, so only a tie-breaker."""
LINK_COLUMNS = ("b_built_x_mm", "b_built_z_mm", "bp_from_b_x_mm", "bp_from_b_z_mm", "apb_mm", "abp_mm")
"""As built: ``B`` from ``O`` in the extended pose, ``B'`` from ``B``, and the bars."""
BOUND_TOLERANCE = 0.005
"""A variable is at a bound within this share of its search range."""


def link_values(p: FingerLinkageParameters) -> tuple[float, ...]:
    return (
        p.b_x_mm, p.b_z_mm, p.bp.x_mm - p.b_x_mm, p.bp.z_mm - p.b_z_mm,
        math.hypot(p.b_x_mm - p.ap.x_mm, p.b_z_mm - p.ap.z_mm),
        math.hypot(p.bp.x_mm - p.a_x_mm, p.bp.z_mm - p.a_z_mm),
    )


def bound_hits(x: Sequence[float], config: LinkageConfig, finger_length_mm: float) -> tuple[str, ...]:
    """The variables within :data:`BOUND_TOLERANCE` of a bound of the search
    box as given (fixed ones left out), and ``|OC|`` near its bounds, as
    ``name:low`` or ``name:high``."""

    values = dict(zip(VARIABLE_NAMES, x))
    values = {name: values.get(name, DEFAULT_VALUES.get(name)) for name in VARIABLE_NAMES}
    boxes = dict(config.bounds)
    if config.relative_bounds is not None:  # A' and B' were searched from A and B
        for point, base in (("ap", "a"), ("bp", "b")):
            for axis in ("x", "z"):
                del boxes[f"{point}_{axis}_mm"]
                name = f"{point}_from_{base}_{axis}_mm"
                values[name] = values[f"{point}_{axis}_mm"] - values[f"{base}_{axis}_mm"]
                boxes[name] = config.relative_bounds[name]
    return bound_hits_of(values, boxes, finger_length_mm, config.extended_length_bounds_mm)


def bound_hits_of(values: Mapping[str, float], boxes: Mapping[str, tuple[float, float]],
                  finger_length_mm: float, length_bounds_mm: tuple[float, float]) -> tuple[str, ...]:
    """Each ``values`` entry within :data:`BOUND_TOLERANCE` of a bound of its
    box (a fixed one left out), and ``|OC|`` near its bounds, as ``name:low``
    or ``name:high`` (the 2R's as the linkage's)."""

    boxes = {**boxes, "finger_length": length_bounds_mm}
    values = {**values, "finger_length": finger_length_mm}
    hits = []
    for name, (low, high) in boxes.items():
        width = high - low
        if width <= 0.0:
            continue
        if values[name] - low <= BOUND_TOLERANCE * width:
            hits.append(f"{name}:low")
        elif high - values[name] <= BOUND_TOLERANCE * width:
            hits.append(f"{name}:high")
    return tuple(hits)


def resolve_preset(values: dict[str, Any]) -> str | None:
    """Apply ``solver.optimizer.preset`` in place: the named entry of
    ``solver.optimizer.presets`` (its algorithm, population, generations,
    seeds...) fills the optimizer table where the table does not set the
    entry itself (a config extending this one may), both keys then dropped.
    Returns the preset's name (``None``: no preset). Idempotent."""

    solver = values.get("solver")
    table = solver.get("optimizer") if isinstance(solver, Mapping) else None
    if not isinstance(table, dict) or ("preset" not in table and "presets" not in table):
        return None
    name, presets = table.pop("preset", None), table.pop("presets", None) or {}
    if name is None:
        return None
    if name not in presets:
        raise ValueError(f"solver.optimizer.preset {name!r} is not in solver.optimizer.presets {sorted(presets)}")
    for key, value in copy.deepcopy(presets[name]).items():
        table.setdefault(key, value)
    return str(name)


def icr0_share_of(p: FingerLinkageParameters, finger_length_mm: float) -> float:
    """The extended centre of rotation's share of ``|OC|`` (NaN: the bars parallel)."""

    found = extended_icr_x((p.a_x_mm, p.a_z_mm), (p.ap.x_mm, p.ap.z_mm), (p.b_x_mm, p.b_z_mm),
                           (p.bp.x_mm, p.bp.z_mm))
    return math.nan if found is None else float(found) / finger_length_mm


class LinkageMechanism(Mechanism):
    name = "linkage"
    formulation = "linkage-v6"
    """The ``goal`` / ``solver`` config (2026-10-23): pads below their pivots,
    no clearance, the PIP servo against the proximal pad, the transmission
    angle in a range, ``pad_coverage`` (effective) and ``curl`` (``180 - phi_max``).
    Earlier formulations (``palmar-pad-v3``, ``linkage-v2-rev2``) were removed
    on 2026-10-26. v4 (2026-10-28): a face loses only what the other face's
    segment hides (``packaging.fold_free_lengths``; v3 crossed the faces'
    endless lines, which, barely flexed, could hide a whole face), and,
    optional in the goal, the pads' drops searched (``goal.pads.*.drop_mm``)
    and their overhang past their body limited (``overhang_max_mm``). v5
    (2026-10-29): optional in the goal, the integrity rule
    (``goal.loss.integrity_depth_mm``): a face's point is lost when hidden in
    the fold or with any solid but its own pad within the depth in front, at
    any pose to the required flexion; absent, v4's measure. v6 (2026-10-29):
    the distal carrier over the whole mounting line from ``B''`` (the support,
    whatever module it carries); optional in the goal, the comparison model's
    capsule links, proximal support plate and dorsal space."""
    variable_names = VARIABLE_NAMES
    constraint_names = CONSTRAINT_NAMES
    objectives = OBJECTIVES
    summary = (
        SummaryColumn("k_max", 6, lambda r: f"{float(r['torque_ratio_max']):.3f}"),
        SummaryColumn("coverage", 8, lambda r: f"{float(r['pad_coverage']):.3f}"),
        SummaryColumn("phi_max", 8, lambda r: f"{float(r['phi_max_deg']):.1f}°"),
        SummaryColumn("|OC| mm", 8, lambda r: f"{float(r['finger_length_mm']):.1f}"),
        SummaryColumn("pads P + D mm", 15, lambda r: f"{float(r['proximal_support_length_mm']):.1f} + "
                                                     f"{float(r['distal_support_length_mm']):.1f}"),
        SummaryColumn("effective P + D", 15, lambda r: f"{float(r['proximal_effective_mm']):.1f} + "
                                                       f"{float(r['distal_effective_mm']):.1f}"),
        SummaryColumn("pad clr", 8, lambda r: f"{float(r['pad_clearance_mm']):.2f}mm"),
        SummaryColumn(" bounds", 0, lambda r: f" {r['bound_hits'] or '-'}"),
    )

    def split_settings(self, values: Mapping[str, Any]) -> tuple[dict[str, Any], OptimizerSettings]:
        """The optimizer settings sit in ``solver.optimizer`` (after its
        ``preset``, :func:`resolve_preset`): ``algorithm`` (one, or a list
        whose first the settings take), ``population``,
        ``generations`` and ``seeds`` (the first one is the settings' seed;
        the config keeps them all)."""

        values = copy.deepcopy(dict(values))
        resolve_preset(values)
        solver = values.get("solver")
        table = dict(solver.get("optimizer") or {}) if isinstance(solver, Mapping) else {}
        names = {"algorithm": "algorithm", "population": "population_size", "generations": "generations",
                 "options": "options"}
        settings = {names[key]: table.pop(key) for key in list(table) if key in names}
        if isinstance(settings.get("algorithm"), list):  # several (run.py runs each): the first
            settings["algorithm"] = settings["algorithm"][0]
        seeds = table.get("seeds")
        if not isinstance(seeds, list) or not seeds:
            raise ValueError("solver.optimizer.seeds must list at least one seed")
        settings["seed"] = int(seeds[0])
        if isinstance(solver, Mapping):
            solver["optimizer"] = table
        return values, OptimizerSettings(**settings)

    def config_from_mapping(self, values: Mapping[str, Any]) -> LinkageConfig:
        return config_from_goal(values)

    def config_to_json(self, config: LinkageConfig) -> dict[str, Any]:
        return config_to_json(config)

    def config_from_json(self, values: dict[str, Any]) -> LinkageConfig:
        return config_from_json(values)

    # Studies with A' and B' bounded from A and B are searched in
    # core.optimization.linkage.search's coordinates; older ones in the design vector.

    def search_names(self, config: LinkageConfig) -> tuple[str, ...]:
        return search.search_names(config) if config.relative_bounds is not None else VARIABLE_NAMES

    def search_box(self, config: LinkageConfig) -> tuple[Sequence[float], Sequence[float]]:
        return search.search_box(config) if config.relative_bounds is not None else (config.lower, config.upper)

    def search_steps(self, config: LinkageConfig) -> tuple[float, ...]:
        if config.relative_bounds is not None:
            return search.search_steps(config)
        return super().search_steps(config)

    def to_design(self, coordinates: Sequence[float], config: LinkageConfig) -> tuple[float, ...] | None:
        if config.relative_bounds is not None:
            return search.to_design(coordinates, config)
        return tuple(coordinates)

    def prepare(self, config: LinkageConfig) -> None:
        if config.relative_bounds is not None:  # the PIP servo floor's table, per tilt searched
            for tilt in search.tilts_deg(config):
                search.least_ap_x(config, config.bounds["ap_z_mm"][0], float(tilt))

    def from_design(self, x: Sequence[float], config: LinkageConfig) -> tuple[float, ...]:
        if config.relative_bounds is not None:
            return search.from_design(x, config)
        return tuple(x)

    def refine(self, config: LinkageConfig, step_rad: float) -> LinkageConfig:
        return dataclasses.replace(config, servo_step_rad=step_rad)  # the config's backoff tolerance

    def evaluate(self, x: Sequence[float], config: LinkageConfig) -> LinkageEvaluation | None:
        try:
            parameters = decode(x, config.link_length_resolution_mm)
        except ValueError:  # B not beyond A, or the rounded bars do not close
            return None
        evaluation = evaluate(parameters, config)
        return dataclasses.replace(evaluation, bound_hits=bound_hits(x, config, evaluation.finger_length_mm))

    def read_design(self, row: Mapping[str, str]) -> list[float]:
        return read_design(row)

    def report(self, e: LinkageEvaluation) -> dict[str, Any]:
        return {
            "torque_ratio_max": e.torque_ratio_max,
            "pad_coverage": e.pad_coverage,
            "curl_deg": e.curl_deg,
            "proximal_effective_mm": e.proximal_effective_mm,
            "distal_effective_mm": e.distal_effective_mm,
            "mounted_coverage": e.mounted_coverage,
            "proximal_usable_length_mm": e.proximal_usable_length_mm,
            "distal_usable_length_mm": e.distal_usable_length_mm,
            "torque_ratio_min": e.torque_ratio_min,
            "finger_length_mm": e.finger_length_mm,
            "icr0_share": icr0_share_of(e.parameters, e.finger_length_mm),
            "proximal_room_mm": e.proximal_room_mm, "distal_room_mm": e.distal_room_mm,
            "support_balance_mm": e.support_balance_mm,
            "required_proximal_mm": e.required_proximal_mm,
            "required_distal_mm": e.required_distal_mm,
            "phi_max_deg": math.degrees(e.phi_max_rad),
            "safe_reach_deg": math.degrees(e.phi_safe_rad),
            "stop_reason": e.stop_reason,
            "collision_pair": e.collision_pair,
            "proximal_support_length_mm": e.proximal_support_length_mm,
            "distal_support_length_mm": e.distal_support_length_mm,
            "app_offset_used_mm": e.parameters.app_offset_mm,
            "bpp_offset_used_mm": e.parameters.bpp_offset_mm,
            "proximal_face_mm": e.proximal_face_mm,
            "distal_face_mm": e.distal_face_mm,
            "proximal_depth_mm": e.proximal_depth_mm,
            "distal_depth_mm": e.distal_depth_mm,
            "pad_clearance_mm": e.pad_clearance_mm,
            "min_margin_deg": math.degrees(e.min_margin_rad),
            "toggle_margin_deg": math.degrees(e.toggle_margin_rad),
            "hyperextension_phi_deg": math.degrees(e.hyperextension_phi_rad),
            "least_tip_x_mm": e.least_tip_x_mm,
            "servo_travel_deg": math.degrees(e.servo_travel_rad),
            "lever_max_mm": e.lever_max_mm,
            "driver_link_servo_clearance_mm": e.driver_link_servo_clearance_mm,
            "servo_lowest_z_mm": e.servo_lowest_z_mm,
            "bound_hits": ";".join(e.bound_hits),
            **dict(zip(LINK_COLUMNS, link_values(e.parameters))),
            "phi_max_stop": e.phi_max_stop,
            "dip_flexion_deg": math.degrees(e.required_rom_rad),
            "usable_pip_deg": math.degrees(e.usable_pip_rad),
            "proximal_length_mm": e.proximal_length_mm,
        }

    def hypervolume_reference(self, config: LinkageConfig) -> Mapping[str, float]:
        """The reference point in minimized terms (``pad_coverage`` negated)."""

        return {name: -value if self.objectives[name].larger_is_better else value
                for name, value in config.hypervolume_reference.items()}



MECHANISM = LinkageMechanism()


__all__ = ["BOUND_TOLERANCE", "LINK_COLUMNS", "MECHANISM", "OBJECTIVES", "LinkageMechanism",
           "bound_hits", "bound_hits_of", "link_values"]
