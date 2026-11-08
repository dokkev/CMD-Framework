"""Named reference designs of the 2-DoF polycentric finger linkage."""

from __future__ import annotations

from core.morphology.finger import PointXZ
from core.morphology.mechanism.linkage.design import make_finger


# Proportions read off the design sketch (OA scaled to 40 mm); a starting
# point, not an optimized design.
SKETCH_LINKAGE = make_finger(a=PointXZ(40.0, 0.0), ap=PointXZ(60.0, 19.3), b=PointXZ(100.0, 0.0),
                             bp=PointXZ(80.0, 19.3), bc_length_mm=40.0)


__all__ = ["SKETCH_LINKAGE"]
