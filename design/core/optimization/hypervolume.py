"""Hypervolume of a front of minimized objectives, against a reference point."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np
from pymoo.indicators.hv import HV


def hypervolume(points: Sequence[Sequence[float]], reference: Sequence[float]) -> float:
    """Area (volume) dominated by ``points`` and bounded by ``reference``, both
    in minimized objective values; points beyond the reference add nothing."""

    points = np.asarray(points, dtype=float).reshape(-1, len(reference))
    if not len(points):
        return 0.0
    return float(HV(ref_point=np.asarray(reference, dtype=float))(points))


def front_hypervolume(rows: Sequence[Mapping[str, object]], mechanism, names: Sequence[str],
                      reference: Mapping[str, float]) -> float:
    """Hypervolume of reported rows (``pareto.csv`` or ``seeds.csv``), each
    objective read from its column and minimized as the mechanism minimizes it;
    ``reference`` is given per objective in the same minimized terms."""

    points = []
    for row in rows:
        point = []
        for name in names:
            objective = mechanism.objectives[name]
            value = float(row[objective.column])
            point.append(objective.minimized(value))
        points.append(point)
    return hypervolume(points, [reference[name] for name in names])


__all__ = ["front_hypervolume", "hypervolume"]
