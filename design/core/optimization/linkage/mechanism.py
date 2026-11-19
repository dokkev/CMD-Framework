"""The CFL (crossed four-bar, PIP servo at ``A'``) as a pluggable mechanism.

Its problem is the ``goal`` / ``solver`` file (``scripts/optimize/linkage/config.yaml``,
:func:`~core.optimization.linkage.goal.config_from_goal`): the prototype
formulation (``design/SSoT/problem_formulation.md`` A.5), whose one objective
is the mean contact pad coverage ``c_pad_mean``.
"""

from __future__ import annotations

import copy
import dataclasses
import math
from collections.abc import Mapping, Sequence
from types import MappingProxyType
from typing import Any

from core.morphology.mechanism.linkage import FourBar
from core.morphology.mechanism.linkage.design import DEFAULT_VALUES, VARIABLE_NAMES, decode, read_design
from core.morphology.mechanism.linkage.measures import extended_icr_x

from core.optimization.linkage.config import LinkageConfig
from core.optimization.linkage.goal import config_from_goal
from core.optimization.linkage.record import config_from_json, config_to_json
from core.optimization.linkage import search
from core.optimization.linkage.evaluate import CONSTRAINT_NAMES, LinkageEvaluation, evaluate
from core.optimization.mechanism import Mechanism, Objective, SummaryColumn, evaluation_report
from core.optimization.optimizer import OptimizerSettings


OBJECTIVES = MappingProxyType({
    "c_pad_mean": Objective(
        value=lambda e: -e.c_pad_mean, column="c_pad_mean",
        label="mean cylinder-based contact pad coverage over Q_eval", role="most_coverage",
        larger_is_better=True),
    "ma_wc": Objective(
        value=lambda e: -(e.ma_wc if math.isfinite(e.ma_wc) else 0.0), column="ma_wc",
        label="worst-case mechanical advantage over [0, q_req]", role="best_transmission",
        larger_is_better=True),
})
"""The prototype formulation's objectives (problem formulation A.5.4); a
study lists the ones it optimizes (``solver.objectives``)."""
LINK_COLUMNS = ("b_built_x_mm", "b_built_z_mm", "bp_from_b_x_mm", "bp_from_b_z_mm", "apb_mm", "abp_mm")
"""As built: ``B`` from ``O`` in the extended pose, ``B'`` from ``B``, and the bars."""
BOUND_TOLERANCE = 0.005
"""A variable is at a bound within this share of its search range."""


def link_values(p: FourBar) -> tuple[float, ...]:
    return (
        p.b.x_mm, p.b.z_mm, p.bp.x_mm - p.b.x_mm, p.bp.z_mm - p.b.z_mm,
        math.hypot(p.b.x_mm - p.ap.x_mm, p.b.z_mm - p.ap.z_mm),
        math.hypot(p.bp.x_mm - p.a.x_mm, p.bp.z_mm - p.a.z_mm),
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


def icr0_share_of(p: FourBar, finger_length_mm: float) -> float:
    """The extended centre of rotation's share of ``|OC|`` (NaN: the bars parallel)."""

    found = extended_icr_x((p.a.x_mm, p.a.z_mm), (p.ap.x_mm, p.ap.z_mm), (p.b.x_mm, p.b.z_mm),
                           (p.bp.x_mm, p.bp.z_mm))
    return math.nan if found is None else float(found) / finger_length_mm


class LinkageMechanism(Mechanism):
    name = "linkage"
    formulation = "cfl-pf1"
    """The problem formulation (2026-11-18, ``design/SSoT/problem_formulation.md``):
    the flexion sampled on a uniform ``q_2`` grid, ``F_static`` / ``F_motion``,
    the transmission angle range and ``MA_wc >= ma_min`` as constraints, the
    mean cylinder-based coverage as the one objective. Earlier formulations
    (``linkage-v1`` to ``linkage-v15``) are in ``docs/decisions.md``."""
    variable_names = VARIABLE_NAMES
    constraint_names = CONSTRAINT_NAMES
    objectives = OBJECTIVES
    summary = (
        SummaryColumn("C_pad", 7, lambda r: f"{float(r['c_pad_mean']):.3f}"),
        SummaryColumn("MA_wc", 6, lambda r: f"{float(r['ma_wc']):.3f}"),
        SummaryColumn("q_max", 7, lambda r: f"{float(r['q_max_deg']):.1f}°"),
        SummaryColumn("L_f mm", 7, lambda r: f"{float(r['finger_length_mm']):.1f}"),
        SummaryColumn("pads P + D mm", 15, lambda r: f"{float(r['proximal_support_length_mm']):.1f} + "
                                                     f"{float(r['distal_support_length_mm']):.1f}"),
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
        return dataclasses.replace(config, flexion_step_rad=step_rad)

    def evaluate(self, x: Sequence[float], config: LinkageConfig) -> LinkageEvaluation | None:
        try:
            finger = decode(x, config.link_length_resolution_mm)
        except ValueError:  # B not beyond A, or the rounded bars do not close
            return None
        evaluation = evaluate(finger, config)
        return dataclasses.replace(evaluation, bound_hits=bound_hits(x, config, evaluation.finger_length_mm))

    def read_design(self, row: Mapping[str, str]) -> list[float]:
        return read_design(row)

    def report(self, e: LinkageEvaluation) -> dict[str, Any]:
        return {
            **evaluation_report(e),
            "icr0_share": icr0_share_of(e.finger.joint, e.finger_length_mm),
            "app_offset_used_mm": round(e.finger.proximal.end_mm - e.finger.joint.ap.x_mm, 9),
            "bpp_offset_used_mm": round(e.finger.joint.bp.x_mm - e.finger.distal_start.x_mm, 9),
            "servo_lowest_z_mm": e.servo_lowest_z_mm,
            **dict(zip(LINK_COLUMNS, link_values(e.finger.joint))),
        }

    def hypervolume_reference(self, config: LinkageConfig) -> Mapping[str, float]:
        """The reference point in minimized terms (``c_pad_mean`` negated)."""

        return {name: -value if self.objectives[name].larger_is_better else value
                for name, value in config.hypervolume_reference.items()}



MECHANISM = LinkageMechanism()


__all__ = ["BOUND_TOLERANCE", "LINK_COLUMNS", "MECHANISM", "OBJECTIVES", "LinkageMechanism",
           "bound_hits", "bound_hits_of", "link_values"]
