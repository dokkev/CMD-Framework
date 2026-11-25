"""The CFL searched in coordinates in which it always flexes to the required
angle (report/study0/study0.md). Experimental: the studies' search
(``core/optimization/linkage/search.py``) and its results are not touched.

The four-bar's six dimensions are searched as

    a_x, e, c, delta, slack, rho, branch

with ``r1 + r2 = D_max + slack`` and ``r1 - r2 = rho * D_min``, where
``D_min`` and ``D_max`` are the least and most centre distance over the
required flexion (``core.geometry.fourbar``). With ``slack > 0`` and
``|rho| < 1`` the four-bar closes on its extended assembly all the way, so
that part of ``f_motion`` holds for every point of the box. The servo's tilt
and the pads' ends stay the base search's coordinates.

Every design made here is one the base search can make: the pivots are put on
the base's grid and passed through the base's own coordinates, and a design
whose pivots the base's ranges would move (its servo floor, its boxes) is no
design (``None``). The space searched is therefore a part of the base's.

:func:`ClosureMechanism.accepts` is the analytical filter for the starts: the
transmission angle within its range over the required flexion, in closed form.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from core.geometry import fourbar
from core.morphology.mechanism.linkage.design import DEFAULT_VALUES, VARIABLE_NAMES
from core.optimization.mechanism import Mechanism

LINKAGE_NAMES = ("a_x_mm", "e_mm", "c_mm", "delta_rad", "slack_mm", "rho", "branch")
KEPT_NAMES = ("servo_tilt_from_min_deg", "app_share", "bpp_share")
"""The base search's coordinates searched as they are."""
ANGLE_STEP_RAD = 0.002
RATIO_STEP = 0.001
LEAST_SLACK_MM = 0.5
MOST_RATIO = 0.98
"""How near the closure limits the box goes: the pivots and the link lengths
are rounded after the map (0.1 mm), and must not round across a limit."""
PIVOT_TOLERANCE_MM = 1e-6
GAMMA_SAMPLES = 61


def _on_grid(value: float, step: float) -> float:
    return round(round(value / step) * step, 9)


class ClosureMechanism(Mechanism):
    """``base`` (a CFL) searched in the closure coordinates."""

    def __init__(self, base: Mechanism) -> None:
        self.base = base
        self.variable_names = base.variable_names
        self.name, self.formulation = f"closure:{base.name}", f"{base.formulation}+closure-v1"
        self.constraint_names = tuple(base.constraint_names)
        self.objectives = base.objectives

    # --- the coordinates -------------------------------------------------
    def search_names(self, config):
        kept = [name for name in self.base.search_names(config) if name in KEPT_NAMES]
        return (*LINKAGE_NAMES, *kept)

    def _base_box(self, config) -> dict[str, tuple[float, float]]:
        lower, upper = self.base.search_box(config)
        return {name: (low, high) for name, low, high in zip(self.base.search_names(config), lower, upper)}

    def search_box(self, config):
        base = self._base_box(config)
        length = base["finger_length_mm"][1]
        dx = config.relative_bounds["ap_from_a_x_mm"]
        most_e = math.hypot(max(abs(dx[0]), abs(dx[1])), max(abs(v) for v in base["ap_from_a_z_mm"]))
        most_c = math.hypot(max(abs(v) for v in base["bp_from_b_x_mm"]), max(abs(v) for v in base["bp_from_b_z_mm"]))
        most_c = min(most_c, length)
        box = {
            "a_x_mm": (config.resolution_mm, length),
            "e_mm": (config.resolution_mm, most_e),
            "c_mm": (config.resolution_mm, most_c),
            "delta_rad": (0.0, 2.0 * math.pi),
            "slack_mm": (LEAST_SLACK_MM, 2.0 * length),
            "rho": (-MOST_RATIO, MOST_RATIO),
            "branch": (0.0, 1.0),
            **{name: base[name] for name in KEPT_NAMES if name in base},
        }
        names = self.search_names(config)
        return tuple(box[name][0] for name in names), tuple(box[name][1] for name in names)

    def search_steps(self, config):
        base = dict(zip(self.base.search_names(config), self.base.search_steps(config)))
        steps = {"a_x_mm": config.resolution_mm, "e_mm": config.resolution_mm, "c_mm": config.resolution_mm,
                 "delta_rad": ANGLE_STEP_RAD, "slack_mm": config.resolution_mm, "rho": RATIO_STEP, "branch": 1.0,
                 **base}
        return tuple(steps[name] for name in self.search_names(config))

    def pivots(self, coordinates: Sequence[float], config):
        """``(A, A', B, B')`` of the coordinates, on the base's grid (``None``:
        no four-bar), and the link lengths before the grid."""

        v = dict(zip(self.search_names(config), (float(c) for c in coordinates)))
        low, high = fourbar.centre_distance_range(v["e_mm"], v["c_mm"], v["delta_rad"], config.required_rom_rad)
        total, difference = high + v["slack_mm"], v["rho"] * low
        found = fourbar.from_closure_coordinates(v["a_x_mm"], v["e_mm"], v["c_mm"], v["delta_rad"], total,
                                                 difference, 1 if v["branch"] >= 0.5 else -1)
        if found is None:
            return None
        step = config.resolution_mm
        return tuple(np.array((_on_grid(p[0], step), _on_grid(p[1], step))) for p in found)

    def to_design(self, coordinates: Sequence[float], config):
        v = dict(zip(self.search_names(config), (float(c) for c in coordinates)))
        pivots = self.pivots(coordinates, config)
        if pivots is None:
            return None
        a, ap, b, bp = pivots
        box = self._base_box(config)
        length = 0.5 * (box["finger_length_mm"][0] + box["finger_length_mm"][1])
        bc = _on_grid(length - b[0], config.resolution_mm)
        relative = {"bc_length_mm": bc, "ap_from_a_z_mm": ap[1] - a[1], "bp_from_b_x_mm": bp[0] - b[0],
                    "bp_from_b_z_mm": bp[1] - b[1]}
        if not a[0] < b[0] or any(not box[name][0] - 1e-9 <= value <= box[name][1] + 1e-9
                                  for name, value in relative.items()):
            return None
        tilt = config.bounds["servo_tilt_deg"][0] + v.get("servo_tilt_from_min_deg", 0.0)
        start = {**DEFAULT_VALUES, "a_x_mm": a[0], "a_z_mm": 0.0, "b_x_mm": b[0], "b_z_mm": 0.0, "ap_x_mm": ap[0],
                 "ap_z_mm": ap[1], "bp_x_mm": bp[0], "bp_z_mm": bp[1], "servo_tilt_deg": tilt, "bc_length_mm": bc}
        names = self.base.search_names(config)
        through = dict(zip(names, self.base.from_design(tuple(start[name] for name in VARIABLE_NAMES), config)))
        through.update({name: v[name] for name in KEPT_NAMES if name in v})
        x = self.base.to_design(tuple(through[name] for name in names), config)
        if x is None:
            return None
        made = dict(zip(VARIABLE_NAMES, x))
        if any(abs(made[name] - start[name]) > PIVOT_TOLERANCE_MM
               for name in ("a_x_mm", "b_x_mm", "ap_x_mm", "ap_z_mm", "bp_x_mm", "bp_z_mm")):
            return None  # the base's ranges would move a pivot: not a design of its space
        return x

    def from_design(self, x: Sequence[float], config):
        d = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
        pivots = [(d["a_x_mm"], d["a_z_mm"]), (d["ap_x_mm"], d["ap_z_mm"]), (d["b_x_mm"], d["b_z_mm"]),
                  (d["bp_x_mm"], d["bp_z_mm"])]
        a_x, e, c, delta, total, difference = fourbar.closure_coordinates(*pivots)
        low, high = fourbar.centre_distance_range(e, c, delta, config.required_rom_rad)
        base = dict(zip(self.base.search_names(config), self.base.from_design(x, config)))
        values = {"a_x_mm": a_x, "e_mm": e, "c_mm": c, "delta_rad": delta % (2.0 * math.pi), "slack_mm": total - high,
                  "rho": difference / low if low > 0.0 else 0.0,
                  "branch": 1.0 if fourbar.branch_of(*pivots) > 0 else 0.0, **base}
        return tuple(values[name] for name in self.search_names(config))

    # --- the analytical filter -------------------------------------------
    def accepts(self, coordinates: Sequence[float], config) -> bool:
        """The transmission angle of the coordinates' four-bar stays within
        its range over the required flexion (closed form, no evaluator)."""

        pivots = self.pivots(coordinates, config)
        if pivots is None:
            return False
        e, c, delta, r1, r2 = fourbar.shape(*pivots)
        if min(e, c, r1, r2) <= 0.0 or not fourbar.closes(e, c, delta, r1, r2, config.required_rom_rad):
            return False
        q = np.linspace(0.0, config.required_rom_rad, GAMMA_SAMPLES)
        gamma = fourbar.transmission_angle(e, c, delta, r1, r2, q, fourbar.branch_of(*pivots))
        low, high = config.transmission_angle_range_rad
        return bool(np.all(np.isfinite(gamma)) and gamma.min() >= low and gamma.max() <= high)

    # --- the rest is the base's ------------------------------------------
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
        return self.base.evaluate(x, config)


__all__ = ["ClosureMechanism", "KEPT_NAMES", "LINKAGE_NAMES"]
