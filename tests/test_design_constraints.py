"""Tests for hand actuator packaging constraints."""

import pytest

from cmd.design import check_actuator_packaging
from cmd.hardware import Actuator, DShapedExtrusion, MechanicalParameters
from cmd.morphology import (
    HandMorphology,
    HandMorphologyParameters,
    HandPlanePoint,
)


def make_morphology(
    *,
    middle=(200.0, 0.0),
    ring=(400.0, 0.0),
    little=(600.0, 0.0),
    thumb=(1000.0, 1000.0),
    ro_raa=100.0,
    raa_mcp=100.0,
):
    return HandMorphology(
        HandMorphologyParameters(
            middle_mcp_aa=HandPlanePoint(*middle),
            ring_mcp_aa=HandPlanePoint(*ring),
            little_mcp_aa=HandPlanePoint(*little),
            thumb_cmc_ro=HandPlanePoint(*thumb),
            theta_rad=0.0,
            ro_raa_distance_mm=ro_raa,
            raa_mcp_fe_distance_mm=raa_mcp,
            phi_rad=0.0,
        )
    )


def test_packaging_reports_overlapping_actuator_pairs() -> None:
    morphology = make_morphology(middle=(10.0, 0.0))

    result = check_actuator_packaging(morphology)

    assert not result.feasible
    assert ("index_mcp_aa", "middle_mcp_aa") in result.violated_actuator_pairs
    overlap_by_pair = {
        (first, second): depth_mm
        for first, second, depth_mm in result.obb_overlap_depths_mm
    }
    assert overlap_by_pair[("index_mcp_aa", "middle_mcp_aa")] == pytest.approx(9.0)


def test_packaging_accepts_a_separated_reference_assembly() -> None:
    result = check_actuator_packaging(make_morphology())

    assert result.feasible
    assert result.violated_actuator_pairs == ()
    assert result.obb_overlap_depths_mm == ()


def test_packaging_uses_actuator_geometry_instead_of_duplicated_dimensions() -> None:
    morphology = make_morphology(middle=(10.0, 0.0))
    compact_actuator = Actuator(
        geometry=DShapedExtrusion(radius_mm=1.0, rear_extent_mm=2.0, depth_mm=2.0),
        mechanics=MechanicalParameters(gear_ratio=1.0, max_output_torque_nm=1.0, mass_g=1.0),
    )

    default_result = check_actuator_packaging(morphology)
    compact_result = check_actuator_packaging(morphology, compact_actuator)

    assert ("index_mcp_aa", "middle_mcp_aa") in default_result.violated_actuator_pairs
    assert ("index_mcp_aa", "middle_mcp_aa") not in compact_result.violated_actuator_pairs


def test_packaging_rejects_wrong_inputs() -> None:
    with pytest.raises(TypeError, match="HandMorphology"):
        check_actuator_packaging(object())
    with pytest.raises(TypeError, match="Actuator"):
        check_actuator_packaging(make_morphology(), object())
