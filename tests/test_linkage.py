from dataclasses import FrozenInstanceError, fields, replace

import pytest

from cmd.morphology import BASELINE_LINKAGE_V0, FingerLinkage, FingerLinkageParameters, PointYZ


def make_parameters(*, b_from_a: PointYZ = PointYZ(30.0, -5.0)) -> FingerLinkageParameters:
    return FingerLinkageParameters(
        o_prime_from_o=PointYZ(0.0, 10.0),
        a_from_o=PointYZ(10.0, 5.0),
        a_prime_from_o=PointYZ(4.0, 9.0),
        b_from_a=b_from_a,
        b_prime_from_a=PointYZ(9.0, 8.0),
        bc_length_mm=20.0,
        c_prime_from_b=PointYZ(8.0, 6.0),
    )


def test_reference_positions_are_composed_from_named_point_translations() -> None:
    linkage = FingerLinkage(make_parameters())
    reference = linkage.reference_positions

    assert reference.o == PointYZ(0.0, 0.0)
    assert reference.op == PointYZ(0.0, 10.0)
    assert reference.a == PointYZ(10.0, 5.0)
    assert reference.ap == PointYZ(4.0, 9.0)
    assert reference.b == PointYZ(40.0, 0.0)
    assert reference.bp == PointYZ(19.0, 13.0)
    assert reference.c == PointYZ(60.0, 0.0)
    assert reference.cp == PointYZ(48.0, 6.0)


def test_fully_extended_checks_bc_parallel_to_oc_without_anchor_collinearity() -> None:
    linkage = FingerLinkage(make_parameters())
    reference = linkage.reference_positions
    bc = PointYZ(linkage.parameters.bc_length_mm, 0.0)
    oc = PointYZ(
        reference.c.y_mm - reference.o.y_mm,
        reference.c.z_mm - reference.o.z_mm,
    )
    cross_2d = bc.y_mm * oc.z_mm - bc.z_mm * oc.y_mm
    dot_2d = bc.y_mm * oc.y_mm + bc.z_mm * oc.z_mm

    assert cross_2d == pytest.approx(0.0, abs=1e-12)
    assert dot_2d > 0.0
    assert reference.a.z_mm != pytest.approx(0.0)
    assert reference.ap.z_mm != pytest.approx(0.0)
    assert reference.bp.z_mm != pytest.approx(0.0)


@pytest.mark.parametrize(
    "parameters",
    [
        make_parameters(b_from_a=PointYZ(30.0, -4.0)),
        make_parameters(b_from_a=PointYZ(-70.0, -5.0)),
    ],
)
def test_reference_rejects_bc_not_parallel_or_not_same_direction(parameters) -> None:
    with pytest.raises(ValueError, match="BC parallel to OC"):
        FingerLinkage(parameters)


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


def test_planar_convention_has_no_independent_x_coordinate() -> None:
    linkage = FingerLinkage(make_parameters())

    assert linkage.flexion_axis_xyz == (1.0, 0.0, 0.0)
    assert not hasattr(linkage.parameters.a_from_o, "x_mm")


def test_parameter_dataclasses_are_immutable() -> None:
    parameters = make_parameters()

    with pytest.raises(FrozenInstanceError):
        parameters.a_from_o = PointYZ(1.0, 0.0)
    with pytest.raises(FrozenInstanceError):
        parameters.a_from_o.y_mm = 1.0


@pytest.mark.parametrize(
    "constructor",
    [lambda: PointYZ(float("nan"), 0.0), lambda: PointYZ(0.0, float("inf"))],
)
def test_nonfinite_coordinates_are_rejected(constructor) -> None:
    with pytest.raises(ValueError):
        constructor()


@pytest.mark.parametrize("bc_length_mm", [0.0, -1.0, float("inf"), float("nan")])
def test_bc_length_must_be_finite_and_positive(bc_length_mm) -> None:
    with pytest.raises(ValueError, match="bc_length_mm"):
        replace(make_parameters(), bc_length_mm=bc_length_mm)


def test_baseline_v0_preserves_pre_refactor_reference_geometry() -> None:
    reference = BASELINE_LINKAGE_V0.reference_positions
    expected = {
        "o": PointYZ(0.0, 0.0),
        "op": PointYZ(10.0, 10.0),
        "a": PointYZ(35.005085389255939, 4.997358991597482),
        "ap": PointYZ(30.001126645511164, 0.0),
        "b": PointYZ(60.010872527309886, 0.001299583790577),
        "bp": PointYZ(40.003104036046295, 0.001299583790577),
        "c": PointYZ(80.010872527309886, 0.001299583790577),
        "cp": PointYZ(50.010872527309886, 5.001299583790577),
    }

    for name, expected_point in expected.items():
        actual = getattr(reference, name)
        assert actual.y_mm == pytest.approx(expected_point.y_mm, abs=1e-12)
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
