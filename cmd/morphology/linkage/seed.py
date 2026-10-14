"""Named reference designs of the 2-DoF polycentric finger linkage."""

from __future__ import annotations

from . import FingerLinkage, FingerLinkageParameters, PointXZ


# Proportions read off the design sketch (OA scaled to 40 mm); a starting
# point, not an optimized design.
SKETCH_LINKAGE = FingerLinkage(
    FingerLinkageParameters(
        a_x_mm=40.0,
        b_x_mm=100.0,
        ap=PointXZ(60.0, 19.3),
        bp=PointXZ(80.0, 19.3),
        bc_length_mm=40.0,
    )
)

__all__ = ["SKETCH_LINKAGE"]
