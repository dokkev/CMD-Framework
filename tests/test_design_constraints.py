"""Tests for hand actuator packaging constraints."""

import pytest

from cmd.design import check_actuator_packaging
from cmd.morphology import HandLayout, HandLayoutParameters, HandPlanePoint


def make_layout(
    *,
    middle=(200.0, 0.0),
    ring=(400.0, 0.0),
    little=(600.0, 0.0),
    thumb=(1000.0, 1000.0),
    ro_raa=100.0,
):
    return HandLayout(
        HandLayoutParameters(
            middle_mcp_aa=HandPlanePoint(*middle),
            ring_mcp_aa=HandPlanePoint(*ring),
            little_mcp_aa=HandPlanePoint(*little),
            thumb_cmc_ro=HandPlanePoint(*thumb),
            theta_rad=0.0,
            ro_raa_distance_mm=ro_raa,
            phi_rad=0.0,
        )
    )


def test_packaging_reports_overlapping_actuator_pairs() -> None:
    layout = make_layout(middle=(10.0, 0.0))

    result = check_actuator_packaging(layout)

    assert not result.feasible
    assert ("index_mcp_aa", "middle_mcp_aa") in result.violated_actuator_pairs
    overlap_by_pair = {
        (first, second): depth_mm
        for first, second, depth_mm in result.obb_overlap_depths_mm
    }
    assert overlap_by_pair[("index_mcp_aa", "middle_mcp_aa")] == pytest.approx(9.0)


def test_packaging_accepts_a_separated_reference_assembly() -> None:
    result = check_actuator_packaging(make_layout())

    assert result.feasible
    assert result.violated_actuator_pairs == ()
    assert result.obb_overlap_depths_mm == ()
