"""Named linkage designs: the Stage-0 seed and the selected Stage-0 result."""

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


# Stage-0 selection (keep BASELINE_LINKAGE_V0 as the comparison reference).
# scripts/optimize/run_stage0.py at commit b7a495e with seed 1: Pareto
# candidate #122, chosen from the verified shortlist as the best transmission
# quality with a facing error of at most 5 deg.
#   worst transmission quality 0.692 (min margin 43 deg over the safe range)
#   grasp at MCP 52 deg (PIP 75, DIP 101), facing error 3.9 deg, gap 24.3 mm
#   safe range MCP 0-72 deg; first near-singular pose at 79.1 deg, so a
#   mechanical stopper belongs between 72 and 79 deg MCP.
SELECTED_LINKAGE_V1 = FingerLinkage(
    FingerLinkageParameters(
        op_from_o=PointXZ(27.0, 5.3),
        a_from_o=PointXZ(38.3, 19.4),
        ap_from_o=PointXZ(32.7, 0.0),
        b_from_a=PointXZ(40.0, -19.4),
        bp_from_a=PointXZ(7.7, -19.4),
        bc_length_mm=29.2,
        cp_from_b=PointXZ(-13.4, 7.8),
    )
)


__all__ = ["BASELINE_LINKAGE_V0", "SELECTED_LINKAGE_V1"]
