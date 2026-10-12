from dataclasses import fields

import pytest

from cmd.morphology import BASELINE_LINKAGE_V0, FingerLinkage, FingerLinkageParameters, PointXZ


def make_parameters(*, b_from_a: PointXZ = PointXZ(30.0, -5.0)) -> FingerLinkageParameters:
    return FingerLinkageParameters(
        o_prime_from_o=PointXZ(0.0, 10.0),
        a_from_o=PointXZ(10.0, 5.0),
        a_prime_from_o=PointXZ(4.0, 9.0),
        b_from_a=b_from_a,
        b_prime_from_a=PointXZ(9.0, 8.0),
        bc_length_mm=20.0,
        c_prime_from_b=PointXZ(8.0, 6.0),
    )


def test_reference_positions_are_composed_from_named_point_translations() -> None:
    linkage = FingerLinkage(make_parameters())
    reference = linkage.reference_positions

    assert reference.o == PointXZ(0.0, 0.0)
    assert reference.op == PointXZ(0.0, 10.0)
    assert reference.a == PointXZ(10.0, 5.0)
    assert reference.ap == PointXZ(4.0, 9.0)
    assert reference.b == PointXZ(40.0, 0.0)
    assert reference.bp == PointXZ(19.0, 13.0)
    assert reference.c == PointXZ(60.0, 0.0)
    assert reference.cp == PointXZ(48.0, 6.0)


def test_fully_extended_checks_bc_parallel_to_oc_without_anchor_collinearity() -> None:
    linkage = FingerLinkage(make_parameters())
    reference = linkage.reference_positions
    bc = PointXZ(linkage.parameters.bc_length_mm, 0.0)
    oc = PointXZ(
        reference.c.x_mm - reference.o.x_mm,
        reference.c.z_mm - reference.o.z_mm,
    )
    cross_2d = bc.x_mm * oc.z_mm - bc.z_mm * oc.x_mm
    dot_2d = bc.x_mm * oc.x_mm + bc.z_mm * oc.z_mm

    assert cross_2d == pytest.approx(0.0, abs=1e-12)
    assert dot_2d > 0.0
    assert reference.a.z_mm != pytest.approx(0.0)
    assert reference.ap.z_mm != pytest.approx(0.0)
    assert reference.bp.z_mm != pytest.approx(0.0)


def test_main_link_and_coupler_lengths_are_derived() -> None:
    linkage = FingerLinkage(make_parameters())

    assert linkage.oa_length_mm == pytest.approx(125.0**0.5)
    assert linkage.ab_length_mm == pytest.approx(925.0**0.5)
    assert linkage.parameters.bc_length_mm == pytest.approx(20.0)
    assert linkage.coupler_1_length_mm == pytest.approx(370.0**0.5)
    assert linkage.coupler_2_length_mm == pytest.approx(1945.0**0.5)
    parameter_names = {field.name for field in fields(linkage.parameters)}
    assert parameter_names == {
        "o_prime_from_o",
        "a_from_o",
        "a_prime_from_o",
        "b_from_a",
        "b_prime_from_a",
        "bc_length_mm",
        "c_prime_from_b",
    }
    assert "coupler_1_length_mm" not in parameter_names
    assert "coupler_2_length_mm" not in parameter_names


def test_baseline_v0_preserves_pre_refactor_reference_geometry() -> None:
    reference = BASELINE_LINKAGE_V0.reference_positions
    expected = {
        "o": PointXZ(0.0, 0.0),
        "op": PointXZ(10.0, 10.0),
        "a": PointXZ(35.005085389255939, 4.997358991597482),
        "ap": PointXZ(30.001126645511164, 0.0),
        "b": PointXZ(60.010872527309886, 0.001299583790577),
        "bp": PointXZ(40.003104036046295, 0.001299583790577),
        "c": PointXZ(80.010872527309886, 0.001299583790577),
        "cp": PointXZ(50.010872527309886, 5.001299583790577),
    }

    for name, expected_point in expected.items():
        actual = getattr(reference, name)
        assert actual.x_mm == pytest.approx(expected_point.x_mm, abs=1e-12)
        assert actual.z_mm == pytest.approx(expected_point.z_mm, abs=1e-12)

    assert BASELINE_LINKAGE_V0.oa_length_mm == pytest.approx(35.36, abs=1e-12)
    assert BASELINE_LINKAGE_V0.ab_length_mm == pytest.approx(25.50, abs=1e-12)
    assert BASELINE_LINKAGE_V0.parameters.bc_length_mm == pytest.approx(20.00)
    assert BASELINE_LINKAGE_V0.coupler_1_length_mm == pytest.approx(
        31.625310461889921,
        abs=1e-12,
    )
    assert BASELINE_LINKAGE_V0.coupler_2_length_mm == pytest.approx(
        20.625298247079598,
        abs=1e-12,
    )
