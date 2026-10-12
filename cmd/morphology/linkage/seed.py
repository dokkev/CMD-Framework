"""Named initial designs for CLEAR linkage studies."""

from . import FingerLinkage, FingerLinkageParameters, PointXZ


# Stage-0 seed with O' at (15, 15) and A', B', B, C on z = 0 (OA', BB' and BC
# parallel to +X): among Pareto designs of scripts/optimize/run_stage0.py with
# an MCP range of at least 80 deg and a parallel error below 5 deg, the one with
# the best worst-case transmission quality, on the 0.1 mm search grid.
BASELINE_LINKAGE_V0 = FingerLinkage(
    FingerLinkageParameters(
        op_from_o=PointXZ(15.0, 15.0),
        a_from_o=PointXZ(45.8, 20.0),
        ap_from_o=PointXZ(44.3, 0.0),
        b_from_a=PointXZ(31.0, -20.0),
        bp_from_a=PointXZ(7.8, -20.0),
        bc_length_mm=20.4,
        cp_from_b=PointXZ(-11.6, 7.3),
    )
)


__all__ = ["BASELINE_LINKAGE_V0"]
