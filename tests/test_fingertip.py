
import math

import pytest

from cmd.morphology import Fingertip, FingertipParameters
from cmd.morphology.pad import (
    FingertipFamilyParameters,
)


def test_default_parameters_construct_successfully() -> None:
    fingertip = Fingertip()

    assert fingertip.parameters == FingertipParameters()
    assert fingertip.half_width_mm == pytest.approx(12.5)
    assert fingertip.total_length_mm == pytest.approx(40.0)
    assert fingertip.carrier_length_mm == pytest.approx(35.0)
    assert fingertip.straight_length_mm == pytest.approx(25.0)
    assert fingertip.parameters.family.carrier_distal_inset_mm == pytest.approx(5.0)


def test_straight_body_center_and_edge_surface_values() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.family.pad

    assert fingertip.lower_surface_z_mm(0.0, 0.0) == pytest.approx(
        -pad.vertical_pad_height_mm - pad.elliptical_pad_height_mm
    )
    assert fingertip.lower_surface_z_mm(
        0.0, pad.vertical_pad_width_mm / 2.0
    ) == pytest.approx(
        -pad.vertical_pad_height_mm
    )
    assert fingertip.lower_surface_z_mm(
        fingertip.straight_length_mm, -pad.vertical_pad_width_mm / 2.0
    ) == pytest.approx(
        -pad.vertical_pad_height_mm
    )


def test_distal_surface_matches_straight_body_at_join() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.family.pad

    for y_mm in (-12.5, -6.0, 0.0, 6.0, 12.5):
        assert fingertip.lower_surface_z_mm(fingertip.straight_length_mm, y_mm) == pytest.approx(
            fingertip.lower_surface_z_mm(fingertip.straight_length_mm - 1e-9, y_mm)
        )


def test_distal_half_ellipsoid_reaches_the_equator_at_the_tip() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.family.pad

    assert fingertip.lower_surface_z_mm(fingertip.total_length_mm, 0.0) == pytest.approx(
        -pad.vertical_pad_height_mm
    )
    assert fingertip.upper_surface_z_mm(fingertip.total_length_mm, 0.0) == pytest.approx(
        0.0
    )
    assert fingertip.half_width_at_x_mm(fingertip.total_length_mm) == pytest.approx(
        0.0
    )


def test_distal_surface_is_the_ellipsoid_along_x_with_lateral_y() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.family.pad
    x_mm, y_mm = fingertip.straight_length_mm + 6.0, 4.0
    expected = -pad.vertical_pad_height_mm - fingertip.c * math.sqrt(
        1.0 - (y_mm / fingertip.a) ** 2
        - ((x_mm - fingertip.straight_length_mm) / fingertip.b) ** 2
    )

    assert fingertip.lower_surface_z_mm(x_mm, y_mm) == pytest.approx(expected)
    with pytest.raises(ValueError):
        fingertip.lower_surface_z_mm(fingertip.total_length_mm + 1.0, 0.0)
    with pytest.raises(ValueError):
        fingertip.lower_surface_z_mm(0.0, fingertip.half_width_mm + 1.0)


def test_surface_is_symmetric_about_the_centerline() -> None:
    fingertip = Fingertip()
    for x_mm, y_mm in ((0.0, 4.0), (25.0, 4.0), (30.0, 3.0), (39.0, 0.3)):
        assert fingertip.lower_surface_z_mm(x_mm, y_mm) == pytest.approx(
            fingertip.lower_surface_z_mm(x_mm, -y_mm)
        )
        assert fingertip.is_inside_pad_footprint(x_mm, y_mm)
        assert fingertip.is_inside_pad_footprint(x_mm, -y_mm)


def test_digit_length_changes_only_the_proximal_straight_section() -> None:
    family = FingertipFamilyParameters()
    short = Fingertip(FingertipParameters(family=family, total_length_mm=40.0))
    long = Fingertip(FingertipParameters(family=family, total_length_mm=60.0))

    assert long.straight_length_mm - short.straight_length_mm == pytest.approx(20.0)
    assert long.carrier_length_mm - short.carrier_length_mm == pytest.approx(20.0)
    assert short.carrier_length_mm - short.straight_length_mm == pytest.approx(10.0)
    assert long.carrier_length_mm - long.straight_length_mm == pytest.approx(10.0)
    assert long.lower_surface_z_mm(long.straight_length_mm + 5.0, 0.0) == pytest.approx(
        short.lower_surface_z_mm(short.straight_length_mm + 5.0, 0.0)
    )
