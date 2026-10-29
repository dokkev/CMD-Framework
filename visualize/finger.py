"""The mechanisms behind one interface: a design read from its study's
verified front, then posed, sketched and checked over the distal flexion ``phi``.

    finger = finger_from_row("rcj", row, study_dir)   # or visualize.studies.best(...)
    finger.margins(phi_deg)   # each checked pair's gap, (n,) per pair
    finger.icr(phi_deg)       # the distal segment's instantaneous centre of rotation
    finger.k(phi_deg)         # d(phi)/d(theta_servo): servo torque per distal moment
    finger.draw(axis, phi)    # the sketch at one flexion

All in the proximal frame (``O`` at the origin, ``+X`` distal, ``+Z``
dorsal), MCP at 0, lengths in mm, angles in degrees at this interface.
"""

from __future__ import annotations

import importlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from matplotlib.patches import Circle

import core.morphology.linkage.packaging as packaging_module
from core.morphology.linkage import FingerLinkage
from core.morphology.linkage.packaging import distal_pad_polygon, pad_margins, proximal_pad_polygon
from core.optimization.linkage.design import decode
from core.optimization.study import load_run
from visualize import sketch
from visualize.motion import index_at, servo_sweep
from visualize.style import LABELS

# The modules, not the packages' evaluate() functions of the same name.
rcj_evaluate = importlib.import_module("core.optimization.rcj.evaluate")
revolute_evaluate = importlib.import_module("core.optimization.revolute.evaluate")
parallel_evaluate = importlib.import_module("core.optimization.revolute_parallel.evaluate")
packaging_module.EXACT_BAND_MM = 1.0e6  # every gap exact (the checks bound far ones from below only)


@dataclass
class Finger:
    """One design of one mechanism."""

    mechanism: str
    row: dict[str, str]
    config: Any
    x: tuple[float, ...]
    state: Any
    """The mechanism's own model: a ``FingerLinkage`` or a geometry."""

    @property
    def label(self) -> str:
        return LABELS[self.mechanism]

    @property
    def packaging(self):
        return self.config.packaging if self.mechanism == "linkage" else self.config.goal.packaging

    @property
    def phi_max_deg(self) -> float:
        return float(self.row["phi_max_deg"])

    @property
    def pads_mm(self) -> tuple[float, float]:
        return float(self.row["proximal_effective_mm"]), float(self.row["distal_effective_mm"])

    # The linkage is posed by its servo; its phi is sampled finely once.
    def _linkage_index(self, phi_deg: np.ndarray) -> tuple[Any, np.ndarray]:
        if not hasattr(self, "_sweep"):
            self._servo, self._sweep = servo_sweep(self.state)
        return self._sweep, index_at(self._sweep, np.radians(phi_deg))

    def reachable_deg(self) -> float:
        """The most flexion the mechanism closes to (the linkage's four-bar may stop short of 180)."""

        if self.mechanism != "linkage":
            return 180.0
        sweep, _ = self._linkage_index(np.zeros(1))
        return float(min(np.degrees(sweep.total_flexion_rad[-1]), 180.0))

    def margins(self, phi_deg: np.ndarray) -> dict[tuple[str, str], np.ndarray]:
        phi_deg = np.asarray(phi_deg, dtype=float)
        if self.mechanism == "linkage":
            sweep, index = self._linkage_index(phi_deg)
            return pad_margins(self.state, self.packaging, sweep.b[index], sweep.c[index], sweep.bp[index], (0.0,))
        module = {"rcj": rcj_evaluate, "revolute_parallel": parallel_evaluate}.get(self.mechanism, revolute_evaluate)
        return module._margins(self.state, np.radians(phi_deg), self.config)

    def icr(self, phi_deg: np.ndarray) -> np.ndarray:
        """The distal segment's instantaneous centre of rotation relative to the proximal, ``(n, 2)``."""

        phi_deg = np.atleast_1d(np.asarray(phi_deg, dtype=float))
        if self.mechanism.startswith("revolute"):
            return np.broadcast_to((self.state.a_x_mm, 0.0), (len(phi_deg), 2)).copy()
        if self.mechanism == "rcj":  # the rollers' contact point, halfway along the carrier
            return (self.state.a + self.state.b(np.radians(phi_deg))) / 2.0
        sweep, index = self._linkage_index(phi_deg)
        linkage = self.state
        a = np.array((linkage.a.x_mm, linkage.a.z_mm))
        ap = np.array((linkage.ap.x_mm, linkage.ap.z_mm))
        points = []
        for bp, b in zip(sweep.bp[index], sweep.b[index]):
            # Where the lines of the two links A-B' and A'-B cross.
            d1, d2 = bp - a, b - ap
            matrix = np.array((d1, -d2)).T
            if abs(np.linalg.det(matrix)) < 1e-9:
                points.append((np.nan, np.nan))
                continue
            t = np.linalg.solve(matrix, ap - a)[0]
            points.append(a + t * d1)
        return np.array(points)

    def k(self, phi_deg: np.ndarray) -> np.ndarray:
        """``d(phi)/d(theta)``: the servo torque per unit of distal moment held."""

        phi_deg = np.atleast_1d(np.asarray(phi_deg, dtype=float))
        if self.mechanism.startswith("revolute"):
            return np.ones(len(phi_deg))
        if self.mechanism == "rcj":
            return np.full(len(phi_deg), rcj_evaluate.TORQUE_RATIO)
        sweep, index = self._linkage_index(phi_deg)
        rate = np.gradient(sweep.total_flexion_rad, np.abs(self._servo[:len(sweep.total_flexion_rad)]))
        return rate[index]

    def tip(self, phi_deg: float) -> np.ndarray:
        if self.mechanism == "linkage":
            sweep, index = self._linkage_index(np.array([phi_deg]))
            return sweep.c[index[0]]
        return self.state.tip(np.radians([phi_deg]))[0]

    def distal_pad(self, phi_deg: float) -> np.ndarray:
        if self.mechanism == "linkage":
            sweep, index = self._linkage_index(np.array([phi_deg]))
            i = index[0]
            return distal_pad_polygon(self.state, self.packaging, sweep.b[i][None], sweep.c[i][None])[0]
        return self.state.distal_pad(np.radians([phi_deg]))[0]

    def proximal_pad(self) -> np.ndarray:
        if self.mechanism == "linkage":
            return proximal_pad_polygon(self.state, self.packaging)
        return self.state.proximal_pad

    def draw(self, axis, phi_deg: float, hit: set[str] = frozenset()) -> None:
        """The sketch at one flexion (``visualize.sketch``)."""

        packaging = self.packaging
        if self.mechanism == "linkage":
            sweep, index = self._linkage_index(np.array([phi_deg]))
            sketch.draw_linkage(axis, self.state, packaging, sweep, int(index[0]), set(hit))
            return
        geo, phi = self.state, math.radians(phi_deg)
        if self.mechanism == "revolute":
            sketch.draw_revolute(axis, geo, packaging, phi, set(hit))
            return
        sketch.palm(axis, packaging)
        if self.mechanism == "rcj":
            sketch.servo(axis, geo.servo.outline)
            b = geo.b([phi])[0]
            for center, name in ((geo.a, "proximal_roller"), (b, "distal_roller")):
                sketch.roller(axis, center, geo.roller_radius_mm, name in hit)
            sketch.pad(axis, geo.proximal_pad, "proximal_pad" in hit)
            sketch.pad(axis, geo.distal_pad([phi])[0], "distal_pad" in hit)
            near, far = geo.posed(geo.distal_mount, [phi])[0]
            sketch.links(axis, ((0.0, 0.0), geo.a), (geo.a, b), (b, near, far, b))
            sketch.nodes(axis, b, near, far)
            sketch.active(axis, (0.0, 0.0), geo.a)
            return
        if self.mechanism == "revolute_parallel":  # the servo on A', a belt to the joint
            sketch.servo(axis, geo.servo.outline)
            joint = geo.joint
            sketch.pad(axis, joint.proximal_pad, "proximal_pad" in hit)
            sketch.pad(axis, joint.distal_pad([phi])[0], "distal_pad" in hit)
            a, ap, radius = geo.a, geo.ap, geo.pulley_radius_mm
            for center in (a, ap):
                axis.add_patch(Circle(center, radius, fill=False, edgecolor=sketch.INK, linewidth=1.0, zorder=3.5))
            along = (a - ap) / np.linalg.norm(a - ap)
            side = radius * np.array((-along[1], along[0]))
            for sign in (1.0, -1.0):  # the belt's two strands
                axis.plot(*np.array((ap + sign * side, a + sign * side)).T, color=sketch.INK, linewidth=1.0, zorder=3.5)
            near, far = joint.posed(joint.distal_mount, [phi])[0]
            sketch.links(axis, ((0.0, 0.0), a, ap), (a, near, far, a))
            sketch.nodes(axis, a, near, far)
            sketch.active(axis, (0.0, 0.0), ap)
            return
        # The idealized 2R: the pin at A, no servo there.
        sketch.pad(axis, geo.proximal_pad, "proximal_pad" in hit)
        sketch.pad(axis, geo.distal_pad([phi])[0], "distal_pad" in hit)
        a = np.array((geo.a_x_mm, 0.0))
        near, far = geo.posed(geo.distal_mount, [phi])[0]
        sketch.links(axis, ((0.0, 0.0), a), (a, near, far, a))
        sketch.nodes(axis, a, near, far)
        axis.add_patch(Circle(a, self.config.joint_radius_mm, facecolor="white", edgecolor=sketch.INK,
                              linewidth=1.0, zorder=4))
        sketch.active(axis, (0.0, 0.0))


def finger_from_row(mechanism: str, row: dict[str, str], study: Path) -> Finger:
    """The design of a front's ``row``; ``study``: the directory of its runs."""

    run = load_run(study / row["run"].split("/")[-1])
    config = run.config
    x = tuple(run.mechanism.read_design(row))
    if mechanism == "linkage":
        state = FingerLinkage(decode(x, config.link_length_resolution_mm))
    elif mechanism == "rcj":
        state = rcj_evaluate.geometry(x, config)
    elif mechanism == "revolute_parallel":
        state = parallel_evaluate.geometry(x, config)
    else:
        state = revolute_evaluate.geometry(x, config)
    return Finger(mechanism, row, config, x, state)


__all__ = ["Finger", "finger_from_row"]
