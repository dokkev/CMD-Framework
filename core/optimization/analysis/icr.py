"""Where a finger bends when it starts to flex: the extended pose's
instantaneous centre of rotation ``ICR0`` of the distal segment relative to
the proximal one, and its share of the finger, ``|O -> ICR0| / |OC|``.

The 2R's is its pin ``A``; the RCJ's the rollers' contact point, ``A + r``
along the finger; the linkage's where the bars ``A-B'`` and ``A'-B`` cross
(as ``visualize.finger.Finger.icr`` at 0 deg).
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence


def icr0_x_mm(mechanism: str, x: Sequence[float], variable_names: Sequence[str]) -> float:
    """``x`` of the extended pose's centre of rotation (NaN: the linkage's bars parallel)."""

    v: Mapping[str, float] = dict(zip(variable_names, (float(value) for value in x)))
    if mechanism.startswith("revolute"):
        return v["a_x_mm"]
    if mechanism == "rcj":
        return v["a_x_mm"] + v["roller_radius_mm"]
    if mechanism == "linkage":
        from core.optimization.linkage.search import extended_icr_x

        found = extended_icr_x((v["a_x_mm"], v.get("a_z_mm", 0.0)), (v["ap_x_mm"], v["ap_z_mm"]),
                               (v["b_x_mm"], v["b_z_mm"]), (v["bp_x_mm"], v["bp_z_mm"]))
        return math.nan if found is None else float(found)
    raise ValueError(f"no centre of rotation for {mechanism!r}")


def icr0_share(mechanism: str, x: Sequence[float], variable_names: Sequence[str], finger_length_mm: float) -> float:
    """``|O -> ICR0| / |OC|``, extended (``O`` at the origin, the finger along ``+X``)."""

    return icr0_x_mm(mechanism, x, variable_names) / finger_length_mm


__all__ = ["icr0_share", "icr0_x_mm"]
