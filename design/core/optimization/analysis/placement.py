"""Contact placement on a fixed CFL linkage from its pole path (Study 0; experimental).

The CFL's design vector holds two kinds of variables: those that set the
four-bar's motion, and ``app_offset_mm`` / ``bpp_offset_mm``, which only set
where the two pads (and their supports) end by the joint. With the linkage
fixed, its motion is computed once and the pad arrangement is found from the
pole path: the exact clearance of the two pads over the motion (problem
formulation A.6.0, proposition 1) and the cylinder-based coverage (proposition
3), both as the study's evaluator has them, evaluated for a whole grid of
arrangements at once. No shape of the best arrangement is assumed.

This is a separate, experimental path: the study's evaluator, objective and
results are not changed by it. What it chooses is always checked by the
evaluator (:func:`place`), which alone knows the other constraints (the servo,
the pins, the supports) and the mounting rule's depths.

- :class:`Linkage`: one linkage's motion and the model on a grid of arrangements;
- :func:`place`: the arrangement for a design's linkage, checked by the evaluator;
- :class:`PlacedMechanism`: the mechanism searched over its linkage alone.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from core.geometry.pole import pole as finite_pole
from core.morphology.mechanism.linkage.design import decode
from core.optimization.analysis.tradeoff import goal_of
from core.optimization.mechanism import Mechanism
from core.optimization.mounting import mounting

POSE_STEP_DEG = 1.0
ARRANGEMENT = ("app_offset_mm", "bpp_offset_mm")
DEPTHS = ("proximal_mount_offset_mm", "distal_mount_offset_mm")


class Linkage:
    """One linkage's motion, computed once (``x``: a design vector; its pad
    arrangement is only where the searches are centred)."""

    def __init__(self, base: Mechanism, x: Sequence[float], config) -> None:
        self.base, self.config = base, config
        self.names = base.variable_names
        self.x = dict(zip(self.names, x))
        goal = goal_of(config)
        packaging = goal.packaging
        self.h = float(packaging.proximal_pad.height_mm)
        self.required = float(goal.required_rom_rad)
        self.start = packaging.proximal_pad_start_mm - packaging.proximal_pad.offset_mm
        q = np.radians(np.arange(0.0, math.degrees(self.required) + 1e-9, POSE_STEP_DEG))
        finger = decode(x, config.link_length_resolution_mm)
        sweep = finger.sweep_pip(q)
        self.complete = len(sweep.tip) == len(q)
        """The four-bar poses every flexion up to the required one."""
        self.q = q[:len(sweep.tip)]
        tips = np.asarray(sweep.tip, dtype=float)
        self.tip, self.b0 = tips[0], np.asarray(sweep.b[0], dtype=float)
        self.poles = np.array([finite_pole(a, t, tips[0]) for a, t in zip(self.q, tips)])[1:]
        self.cos, self.sin = np.cos(self.q[1:]), np.sin(self.q[1:])
        self.ap_x = finger.joint.ap.x_mm
        self.bp_minus_b = finger.joint.bp.x_mm - finger.joint.b.x_mm
        # The phalanges' structures end at their own pivot housings where those are nearer the joint (A.3).
        self.end1_least = finger.joint.a.x_mm + packaging.pivot_radius_mm("a")
        self.end2_most = self.b0[0] - packaging.pivot_radius_mm("b")
        poses = (math.pi / 2, (math.pi / 2 + self.required) / 2, self.required)
        self.pose_index = [int(np.argmin(np.abs(self.q[1:] - p))) for p in poses] if len(self.q) > 1 else []

    def ends(self, app, bpp) -> tuple[np.ndarray, np.ndarray]:
        """The proximal pad's end from ``O`` and the distal pad's start (``x``, extended pose)."""

        return self.ap_x + np.asarray(app, dtype=float), self.b0[0] + self.bp_minus_b - np.asarray(bpp, dtype=float)

    def grid(self, app, bpp, t1: float, t2: float) -> tuple[np.ndarray, np.ndarray]:
        """For every ``(app, bpp)``: the pads' least exact clearance over the
        motion and the coverage, arrays ``(len(app), len(bpp))``, with the
        contact faces ``t1`` and ``t2`` deep."""

        end1, start2 = self.ends(app, bpp)
        end1, start2 = end1[:, None, None], start2[None, :, None]
        h, tip_x, tip_z = self.h, self.tip[0], self.tip[1]
        pole, c, s = self.poles, self.cos, self.sin
        # The pads as rectangles: centres and half lengths; the distal one turned about the pole.
        ax, az, a_half = 0.5 * (self.start + end1), -t1 + h / 2, 0.5 * (end1 - self.start)
        bx, bz, b_half = 0.5 * (start2 + tip_x), tip_z - t2 + h / 2, 0.5 * (tip_x - start2)
        rx, rz = bx - pole[:, 0], bz - pole[:, 1]
        dx = pole[:, 0] + c * rx + s * rz - ax
        dz = pole[:, 1] - s * rx + c * rz - az
        thick = h / 2
        ac, asn = np.abs(c), np.abs(s)
        m = np.abs(dx) - a_half - (b_half * ac + thick * asn)                                   # e_x
        m = np.maximum(m, np.abs(dz) - thick - (b_half * asn + thick * ac))                     # e_z
        m = np.maximum(m, np.abs(c * dx - s * dz) - (a_half * ac + thick * asn) - b_half)       # R e_x
        m = np.maximum(m, np.abs(s * dx + c * dz) - (a_half * asn + thick * ac) - thick)        # R e_z
        end1, start2 = end1[..., 0], start2[..., 0]
        possible = (end1 > self.start + 1.0) & (start2 < tip_x - 1.0)
        clearance = np.where(possible, m.min(axis=2), -np.inf)
        e1, e2 = np.maximum(end1, self.end1_least), np.minimum(start2, self.end2_most)
        cover = np.zeros_like(clearance)
        for k in self.pose_index:
            px, pz, ck, sk = pole[k, 0], pole[k, 1], c[k], s[k]
            ix, iz = start2 - px, (tip_z - t2) - pz                   # the distal face's inner end, about the pole
            inner_x, inner_z = px + ck * ix + sk * iz, pz - sk * ix + ck * iz
            along = (inner_z + t1) / sk                                # to the proximal face's line, along the distal face
            meet = inner_x + along * ck                                # X, where the two face lines meet
            low = np.maximum(np.maximum(meet - end1, -along), 0.0)
            high = np.minimum(meet - self.start, (tip_x - start2) - along)
            common = np.maximum(high - low, 0.0)
            sx, sz = e2 - px, tip_z - pz                               # the distal structure's start, turned
            gap = np.hypot(px + ck * sx + sk * sz - e1, pz - sk * sx + ck * sz)
            cover = cover + 2.0 * common / (e1 + gap + (tip_x - e2))
        return clearance, cover / max(len(self.pose_index), 1)

    def best(self, app, bpp, t1: float, t2: float, count: int = 1) -> list[tuple[float, float, float]]:
        """The ``count`` arrangements of the most coverage whose pads never overlap: ``(coverage, app, bpp)``."""

        clearance, cover = self.grid(app, bpp, t1, t2)
        score = np.where(clearance >= -1e-9, cover, -np.inf)
        found = []
        for flat in np.argsort(score, axis=None)[::-1][:count]:
            i, j = np.unravel_index(flat, score.shape)
            if np.isfinite(score[i, j]):
                found.append((float(score[i, j]), float(app[i]), float(bpp[j])))
        return found

    def window(self, name: str, width: float, step: float, centre: float | None = None) -> np.ndarray:
        """The arrangement variable's values ``width`` either side of ``centre`` (the
        design's own), on ``step``, within its search box."""

        centre = self.x[name] if centre is None else centre
        low, high = goal_of(self.config).bounds[name]
        values = np.round(np.arange(centre - width, centre + width + 1e-9, step), 6)
        return values[(values >= low - 1e-9) & (values <= high + 1e-9)]

    def reference(self, t1: float, t2: float, width: float = 6.0, step: float = 0.1, count: int = 1):
        """The exhaustive search on ``step``: the correctness reference."""

        return self.best(self.window(ARRANGEMENT[0], width, step), self.window(ARRANGEMENT[1], width, step),
                         t1, t2, count)

    def fast(self, t1: float, t2: float, width: float = 6.0, coarse: float = 0.5, step: float = 0.1, count: int = 1):
        """Coarse, then ``step`` about the coarse best."""

        first = self.best(self.window(ARRANGEMENT[0], width, coarse), self.window(ARRANGEMENT[1], width, coarse),
                          t1, t2)
        if not first:
            return []
        _, app, bpp = first[0]
        return self.best(self.window(ARRANGEMENT[0], coarse, step, app), self.window(ARRANGEMENT[1], coarse, step, bpp),
                         t1, t2, count)

    def design(self, app: float, bpp: float) -> tuple[float, ...] | None:
        """The design vector with this arrangement and the mounting rule's depths (``None``: it does not mount)."""

        y = dict(self.x)
        y[ARRANGEMENT[0]], y[ARRANGEMENT[1]] = app, bpp
        depths = mounting(self.base.name, tuple(y[n] for n in self.names), self.config).depths_mm
        if None in depths:
            return None
        y[DEPTHS[0]], y[DEPTHS[1]] = depths
        return tuple(y[n] for n in self.names)


def _depths(base: Mechanism, evaluation) -> tuple[float, float]:
    report = base.report(evaluation)
    return float(report["proximal_depth_mm"]), float(report["distal_depth_mm"])


def place(base: Mechanism, x: Sequence[float], config, tries: int = 5, rounds: int = 3):
    """The pad arrangement for the linkage of ``x``, checked by the evaluator:
    ``(design vector, evaluation, evaluations spent)``; the design ``x``
    itself with its mounting depths when nothing better is accepted, and
    ``(None, None, spent)`` when that is no design either.

    The face depths the model needs come from the evaluator: the search runs
    under the depths of the design at hand, the evaluator is asked about its
    best few, and where the mounting rule gives the chosen arrangement other
    depths the search is repeated under those (a few rounds). Never worse than
    the arrangement ``x`` came with."""

    link = Linkage(base, x, config)
    start = link.design(link.x[ARRANGEMENT[0]], link.x[ARRANGEMENT[1]])
    spent = 0
    kept = (None, None, -math.inf)
    if start is not None:
        own = _evaluate(base, start, config)
        spent += 1
        if own is not None and own.feasible:
            kept = (start, own, float(base.report(own)["c_pad_mean"]))
        depths = None if own is None else _depths(base, own)
    else:
        depths = None
    if not link.complete or len(link.q) < 2:
        return kept[0], kept[1], spent
    if depths is None:  # no evaluation to read the depths from: the mounting line itself
        depths = (link.h, link.h)
    seen = set()
    for _ in range(rounds):
        if depths in seen:
            break
        seen.add(depths)
        other = None
        for cover, app, bpp in link.fast(*depths, count=tries):
            if cover <= kept[2] + 1e-12:
                break  # the rest are no better than what is kept
            design = link.design(app, bpp)
            if design is None:
                continue
            evaluation = _evaluate(base, design, config)
            spent += 1
            if evaluation is None:
                continue
            if _depths(base, evaluation) != depths:
                other = other or _depths(base, evaluation)
                if not evaluation.feasible:
                    continue
            if evaluation.feasible:
                value = float(base.report(evaluation)["c_pad_mean"])
                if value > kept[2]:
                    kept = (design, evaluation, value)
                break
        if other is None:
            break
        depths = other
    return kept[0], kept[1], spent


def _evaluate(base: Mechanism, x, config):
    try:
        return base.evaluate(x, config)
    except ValueError:
        return None


class PlacedMechanism(Mechanism):
    """``base`` (a CFL) searched over its linkage alone: each design's pad
    arrangement is replaced by :func:`place`'s before it is scored, so the two
    arrangement variables of the vector no longer matter."""

    def __init__(self, base: Mechanism) -> None:
        self.base = base
        self.variable_names = base.variable_names
        self.name, self.formulation = f"placed:{base.name}", f"{base.formulation}+placed-v1"
        self.constraint_names = tuple(base.constraint_names)
        self.objectives = base.objectives
        self.evaluations = 0
        """Evaluator calls spent (in this process)."""

    def search_names(self, config):
        return self.base.search_names(config)

    def search_box(self, config):
        return self.base.search_box(config)

    def search_steps(self, config):
        return self.base.search_steps(config)

    def to_design(self, coordinates, config):
        return self.base.to_design(coordinates, config)

    def from_design(self, x, config):
        return self.base.from_design(x, config)

    def prepare(self, config) -> None:
        self.base.prepare(config)

    def active_objectives(self, config) -> tuple[str, ...]:
        return self.base.active_objectives(config)

    def config_from_mapping(self, values):
        return self.base.config_from_mapping(values)

    def config_to_json(self, config):
        return self.base.config_to_json(config)

    def config_from_json(self, values):
        return self.base.config_from_json(values)

    def report(self, evaluation):
        return self.base.report(evaluation)

    def evaluate(self, x: Sequence[float], config):
        try:
            design, evaluation, spent = place(self.base, x, config)
        except ValueError:
            return _evaluate(self.base, x, config)
        self.evaluations += spent
        return evaluation if evaluation is not None else _evaluate(self.base, x, config)


__all__ = ["ARRANGEMENT", "Linkage", "PlacedMechanism", "place"]
