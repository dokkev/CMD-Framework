"""Contact-interface retention, the measure the mechanisms are compared by.

Each segment carries a contact module of fixed nominal envelope (thickness
``h``, nominal length, independent of the link lengths, which it may outgrow).
A design keeps of each module what clears every other solid from extension
to the required ROM, continuously: its joint-facing vertices are design
variables (:class:`~core.morphology.contact.retention.Module`), every
checked gap over the sweep is a hard constraint, and the two objectives are
the kept area over the nominal area, proximal ``r_p`` and distal ``r_d``,
maximized. A front's figure is ``max min(r_p, r_d)`` (the ``maximin`` seed).

A mechanism supplies its geometry variables, its posed solids
(:meth:`RetentionMechanism.solids`) and which pairs it checks
(:attr:`RetentionMechanism.pairs`). Transmission is a diagnostic over the
required ROM, not an objective.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

import numpy as np

from core.morphology.contact.retention import Module, Pair, Solids, pair_margins

from core.optimization.mechanism import Mechanism, Objective, SummaryColumn
from core.optimization.retention.config import (
    RetentionConfig,
    config_from_json,
    config_from_mapping,
    config_to_json,
)


MODULE_VARIABLES = ("proximal_line_mm", "proximal_palmar_mm", "distal_line_mm", "distal_palmar_mm")
"""Kept length of each module's joint-facing vertices from its outer edge:
``b`` on the mounting line and ``d`` on the palmar face."""
LENGTH_CONSTRAINTS = ("extended_length_min_mm", "extended_length_max_mm")

OBJECTIVES = MappingProxyType({
    "proximal_retention": Objective(
        value=lambda e: -e.proximal_retention, column="proximal_retention",
        label="proximal module area kept / nominal", role="best_proximal", larger_is_better=True),
    "distal_retention": Objective(
        value=lambda e: -e.distal_retention, column="distal_retention",
        label="distal module area kept / nominal", role="best_distal", larger_is_better=True),
})


@dataclass(frozen=True)
class RetentionEvaluation:
    geometry: Mapping[str, float]
    proximal_retention: float
    distal_retention: float
    margins_mm: Mapping[str, float]
    """Least margin of each checked pair over the sweep (inf: an empty module's)."""
    rom_reached_rad: float
    """Last swept flexion before the first collision (the required ROM when none)."""
    first_collision: str
    extended_length_mm: float
    constraints: Mapping[str, float]
    required_rom_rad: float
    torque_ratio_max: float
    torque_ratio_min: float
    servo_travel_rad: float
    provisional: str
    """Why the candidate is provisional (empty: it is not)."""

    @property
    def g(self) -> tuple[float, ...]:
        return tuple(self.constraints.values())

    @property
    def feasible(self) -> bool:
        return all(value <= 0.0 for value in self.constraints.values())

    @property
    def min_retention(self) -> float:
        return min(self.proximal_retention, self.distal_retention)


class RetentionMechanism(Mechanism):
    """A mechanism compared by contact retention."""

    formulation = "contact-retention-v1"
    objectives = OBJECTIVES
    geometry_names: tuple[str, ...]
    pairs: tuple[Pair, ...]
    summary = (
        SummaryColumn("r_p", 6, lambda r: f"{float(r['proximal_retention']):.3f}"),
        SummaryColumn("r_d", 6, lambda r: f"{float(r['distal_retention']):.3f}"),
        SummaryColumn("min", 6, lambda r: f"{float(r['min_retention']):.3f}"),
        SummaryColumn("ROM", 7, lambda r: f"{float(r['required_rom_deg']):.0f}°"),
        SummaryColumn("|O tip|", 8, lambda r: f"{float(r['extended_length_mm']):.1f}"),
        SummaryColumn("k", 5, lambda r: f"{float(r['torque_ratio_max']):.2f}"),
        SummaryColumn(" limit", 0, lambda r: f" {r['limiting_pair']}"
                      + (f"  [provisional: {r['provisional']}]" if r["provisional"] else "")),
    )

    @property
    def variable_names(self) -> tuple[str, ...]:
        return self.geometry_names + MODULE_VARIABLES

    @property
    def constraint_names(self) -> tuple[str, ...]:
        return tuple(pair.name for pair in self.pairs) + LENGTH_CONSTRAINTS

    # The mechanism.

    def solids(self, geometry: Mapping[str, float], modules: tuple[Module, Module],
               config: RetentionConfig) -> Solids | None:
        """The design's solids over ``config.rom_samples_rad``; ``None`` when the
        geometry describes no mechanism."""

        raise NotImplementedError

    def torque_ratio(self, geometry: Mapping[str, float], flexion_rad: np.ndarray) -> np.ndarray:
        """Servo torque per distal moment held, at each distal flexion."""

        raise NotImplementedError

    def servo_travel_rad(self, config: RetentionConfig) -> float:
        return config.required_rom_rad

    provisional: str = ""
    """Why every candidate of this mechanism is provisional (empty: none is)."""

    # Config.

    def config_from_mapping(self, values: Mapping[str, Any]) -> RetentionConfig:
        config = config_from_mapping(values)
        if set(config.bounds) != set(self.geometry_names):
            raise ValueError(f"{self.name} bounds must cover exactly {self.geometry_names}")
        return config

    def config_to_json(self, config: RetentionConfig) -> dict[str, Any]:
        return config_to_json(config)

    def config_from_json(self, values: dict[str, Any]) -> RetentionConfig:
        return config_from_json(values)

    def search_box(self, config: RetentionConfig) -> tuple[Sequence[float], Sequence[float]]:
        box = dict(self.geometry_box(config))
        box["proximal_line_mm"] = box["proximal_palmar_mm"] = (0.0, config.packaging.proximal_pad.length_mm)
        box["distal_line_mm"] = box["distal_palmar_mm"] = (0.0, config.packaging.distal_pad.length_mm)
        return (tuple(box[name][0] for name in self.variable_names),
                tuple(box[name][1] for name in self.variable_names))

    def geometry_box(self, config: RetentionConfig) -> Mapping[str, tuple[float, float]]:
        return config.bounds

    def refine(self, config: RetentionConfig, step_rad: float) -> RetentionConfig:
        import dataclasses

        return dataclasses.replace(config, rom_step_rad=step_rad)

    # Designs.

    def modules(self, x: Sequence[float], outer_mm: tuple[float, float],
                config: RetentionConfig) -> tuple[Module, Module]:
        """The two modules of a design vector, given their outer edges along
        the mounting lines (the distal one from the distal frame's origin)."""

        values = dict(zip(self.variable_names, x))
        proximal, distal = config.packaging.proximal_pad, config.packaging.distal_pad
        return (
            Module(outer_mm[0], 1.0, proximal.height_mm, proximal.length_mm,
                   values["proximal_line_mm"], values["proximal_palmar_mm"]),
            Module(outer_mm[1], -1.0, distal.height_mm, distal.length_mm,
                   values["distal_line_mm"], values["distal_palmar_mm"]),
        )

    def evaluate(self, x: Sequence[float], config: RetentionConfig) -> RetentionEvaluation | None:
        geometry = dict(zip(self.geometry_names, (float(v) for v in x)))
        try:
            solids = self.solids(geometry, self.module_outlines(x, geometry, config), config)
        except ValueError:
            return None
        if solids is None:
            return None
        samples = np.asarray(config.rom_samples_rad)
        per_sample = pair_margins(solids, self.pairs, config.packaging, config.mcp_samples_rad, len(samples))
        stacked = np.vstack(list(per_sample.values()))
        failing = np.flatnonzero(stacked.min(axis=0) < 0.0)
        if failing.size:
            first_bad = int(failing[0])
            reached = float(samples[first_bad - 1]) if first_bad > 0 else 0.0
            collision = list(per_sample)[int(np.argmin(stacked[:, first_bad]))]
        else:
            reached, collision = config.required_rom_rad, ""
        margins = {name: float(values.min()) for name, values in per_sample.items()}
        low, high = config.extended_length_bounds_mm
        constraints = {name: -value for name, value in margins.items()}
        constraints["extended_length_min_mm"] = low - solids.extended_length_mm
        constraints["extended_length_max_mm"] = solids.extended_length_mm - high
        ratio = self.torque_ratio(geometry, samples)
        return RetentionEvaluation(
            geometry=MappingProxyType(geometry),
            proximal_retention=solids.proximal.retention,
            distal_retention=solids.distal.retention,
            margins_mm=MappingProxyType(margins),
            rom_reached_rad=reached,
            first_collision=collision,
            extended_length_mm=solids.extended_length_mm,
            constraints=MappingProxyType(constraints),
            required_rom_rad=config.required_rom_rad,
            torque_ratio_max=float(ratio.max()),
            torque_ratio_min=float(ratio.min()),
            servo_travel_rad=self.servo_travel_rad(config),
            provisional=self.provisional,
        )

    def module_outlines(self, x: Sequence[float], geometry: Mapping[str, float],
                        config: RetentionConfig) -> tuple[Module, Module]:
        """The modules, their outer edges at the proximal start and the distal segment's end."""

        distal_end = self.distal_length_mm(geometry) - config.packaging.distal_pad.offset_mm
        return self.modules(x, (config.packaging.proximal_pad_start_mm, distal_end), config)

    def distal_length_mm(self, geometry: Mapping[str, float]) -> float:
        """Length of the distal segment from its frame's origin to its end."""

        raise NotImplementedError

    def report(self, e: RetentionEvaluation) -> dict[str, Any]:
        fixed = {pair.name for pair in self.pairs if pair.fixed}
        reported = {name: value for name, value in e.margins_mm.items() if name not in fixed}
        limiting = min(reported, key=reported.get) if reported else ""
        return {
            "proximal_retention": e.proximal_retention,
            "distal_retention": e.distal_retention,
            "min_retention": e.min_retention,
            "required_rom_deg": math.degrees(e.required_rom_rad),
            "rom_reached_deg": math.degrees(e.rom_reached_rad),
            "first_collision": e.first_collision,
            "limiting_pair": limiting,
            "least_margin_mm": e.margins_mm[limiting] if limiting else math.inf,
            "extended_length_mm": e.extended_length_mm,
            "torque_ratio_max": e.torque_ratio_max,
            "torque_ratio_min": e.torque_ratio_min,
            "servo_travel_deg": math.degrees(e.servo_travel_rad),
            "provisional": e.provisional,
            **{f"margin_{name}_mm": value for name, value in e.margins_mm.items()},
        }

    # Seeds: the best at each objective, the balanced one and the maximin one.

    def seed_roles(self, objectives, columns, names) -> dict[str, int]:
        roles = super().seed_roles(objectives, columns, names)
        if columns:
            roles["maximin"] = max(range(len(columns)), key=lambda i: (
                min(float(columns[i]["proximal_retention"]), float(columns[i]["distal_retention"])),
                float(columns[i]["proximal_retention"]) + float(columns[i]["distal_retention"])))
        return roles

    def role_order(self, names: Sequence[str]) -> list[str]:
        return ["maximin", *super().role_order(names)]


def front_retention(rows: Sequence[Mapping[str, Any]]) -> float:
    """A front's figure: ``max min(r_p, r_d)`` over its candidates (0 when empty)."""

    return max((min(float(r["proximal_retention"]), float(r["distal_retention"])) for r in rows),
               default=0.0)


__all__ = [
    "LENGTH_CONSTRAINTS",
    "MODULE_VARIABLES",
    "OBJECTIVES",
    "RetentionEvaluation",
    "RetentionMechanism",
    "front_retention",
]
