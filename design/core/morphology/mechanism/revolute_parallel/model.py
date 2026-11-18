"""The belt-driven remote 2R finger decoded onto :class:`~core.morphology.finger.Finger`.

Articulation kinematics is identical to the direct-drive 2R (:class:`~core.morphology.mechanism.revolute.Pin`
at ``A``); the remote actuator shaft ``A' = A + (dx, dz)`` belongs to the actuation group.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from core.morphology.finger import Finger
from core.morphology.mechanism.revolute.model import make_finger


def decode(x: Sequence[float]) -> Finger:
    """Decode a ``revolute_parallel`` design vector to a :class:`Finger`."""

    values = [float(v) for v in x]
    if len(values) != 9:
        raise ValueError(f"revolute_parallel design vector has {len(values)} values; expected 9")
    a_x, ac, app_x, bpp_from_a, distal_drop, proximal_drop, tilt_deg, _, _ = values
    return make_finger(
        a_x,
        ac,
        app_x,
        bpp_from_a,
        math.radians(tilt_deg),
        pad_offsets_mm=(proximal_drop, distal_drop),
    )


__all__ = ["decode"]
