from dataclasses import fields

import pytest

from core.experimental.linkage import BASELINE_LINKAGE_V0, FingerLinkage, FingerLinkageParameters, PointXZ


def make_parameters(*, b_from_a: PointXZ = PointXZ(30.0, -5.0)) -> FingerLinkageParameters:
    return FingerLinkageParameters(
        op_from_o=PointXZ(0.0, 10.0),
        a_from_o=PointXZ(10.0, 5.0),
        ap_from_o=PointXZ(4.0, 9.0),
        b_from_a=b_from_a,
        bp_from_a=PointXZ(9.0, 8.0),
        bc_length_mm=20.0,
        cp_from_b=PointXZ(8.0, 6.0),
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


def test_bc_is_parallel_to_x_while_b_and_c_may_sit_off_the_x_axis() -> None:
    # B = (40, 3): BC stays along +X, so C = (60, 3) and OC is not parallel to BC.
    reference = FingerLinkage(make_parameters(b_from_a=PointXZ(30.0, -2.0))).reference_positions

    assert reference.b == PointXZ(40.0, 3.0)
    assert reference.c == PointXZ(60.0, 3.0)
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
        "op_from_o",
        "a_from_o",
        "ap_from_o",
        "b_from_a",
        "bp_from_a",
        "bc_length_mm",
        "cp_from_b",
    }
    assert "coupler_1_length_mm" not in parameter_names
    assert "coupler_2_length_mm" not in parameter_names


def test_baseline_v0_reference_geometry() -> None:
    reference = BASELINE_LINKAGE_V0.reference_positions
    expected = {
        "o": PointXZ(0.0, 0.0),
        "op": PointXZ(15.0, 15.0),
        "a": PointXZ(45.8, 20.0),
        "ap": PointXZ(44.3, 0.0),
        "b": PointXZ(76.8, 0.0),
        "bp": PointXZ(53.6, 0.0),
        "c": PointXZ(97.2, 0.0),
        "cp": PointXZ(65.2, 7.3),
    }

    for name, expected_point in expected.items():
        actual = getattr(reference, name)
        assert actual.x_mm == pytest.approx(expected_point.x_mm, abs=1e-12)
        assert actual.z_mm == pytest.approx(expected_point.z_mm, abs=1e-12)

    assert BASELINE_LINKAGE_V0.oa_length_mm == pytest.approx((45.8**2 + 20.0**2) ** 0.5)
    assert BASELINE_LINKAGE_V0.ab_length_mm == pytest.approx((31.0**2 + 20.0**2) ** 0.5)
    assert BASELINE_LINKAGE_V0.coupler_1_length_mm == pytest.approx((38.6**2 + 15.0**2) ** 0.5)
    assert BASELINE_LINKAGE_V0.coupler_2_length_mm == pytest.approx((20.9**2 + 7.3**2) ** 0.5)


def test_selected_v1_is_the_verified_stage0_choice() -> None:
    from core.experimental.linkage import SELECTED_LINKAGE_V1

    reference = SELECTED_LINKAGE_V1.reference_positions
    for point in (reference.ap, reference.bp, reference.b, reference.c):
        assert point.z_mm == pytest.approx(0.0, abs=1e-12)
    assert reference.c == PointXZ(107.5, 0.0)
    assert SELECTED_LINKAGE_V1 != BASELINE_LINKAGE_V0
