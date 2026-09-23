import math
from dataclasses import fields

import pytest

from cmd.morphology import (
    Fingertip,
    FingertipParameters,
    PadParameters,
    PhalanxParameters,
)


def test_default_parameters_construct_successfully() -> None:
    fingertip = Fingertip()

    assert fingertip.parameters == FingertipParameters()
    assert fingertip.half_width_mm == pytest.approx(12.5)
    assert fingertip.total_length_mm == pytest.approx(40.0)
    assert fingertip.carrier_length_mm == pytest.approx(35.0)
    assert fingertip.straight_length_mm == pytest.approx(25.0)


def test_parameter_fields_use_physical_names_and_no_stored_tip_duplicates() -> None:
    assert {field.name for field in fields(PadParameters)} == {
        "vertical_pad_width_mm",
        "vertical_pad_height_mm",
        "elliptical_pad_width_mm",
        "elliptical_pad_height_mm",
        "pad_length_mm",
        "tip_length_mm",
    }
    assert {field.name for field in fields(PhalanxParameters)} == {
        "carrier_width_mm",
        "carrier_height_mm",
        "stem_width_mm",
        "stem_height_mm",
        "carrier_length_mm",
    }


def test_straight_body_center_and_edge_surface_values() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.pad

    assert fingertip.lower_surface_z_mm(0.0, 0.0) == pytest.approx(
        -pad.vertical_pad_height_mm - pad.elliptical_pad_height_mm
    )
    assert fingertip.lower_surface_z_mm(
        pad.vertical_pad_width_mm / 2.0, 0.0
    ) == pytest.approx(
        -pad.vertical_pad_height_mm
    )
    assert fingertip.lower_surface_z_mm(
        -pad.vertical_pad_width_mm / 2.0, fingertip.straight_length_mm
    ) == pytest.approx(
        -pad.vertical_pad_height_mm
    )


def test_distal_surface_matches_straight_body_at_join() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.pad

    for x_mm in (-12.5, -6.0, 0.0, 6.0, 12.5):
        assert fingertip.lower_surface_z_mm(x_mm, fingertip.straight_length_mm) == pytest.approx(
            fingertip.lower_surface_z_mm(x_mm, fingertip.straight_length_mm - 1e-9)
        )


def test_distal_half_ellipsoid_reaches_the_equator_at_the_tip() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.pad

    assert fingertip.lower_surface_z_mm(0.0, fingertip.total_length_mm) == pytest.approx(
        -pad.vertical_pad_height_mm
    )
    assert fingertip.upper_surface_z_mm(0.0, fingertip.total_length_mm) == pytest.approx(
        0.0
    )
    assert fingertip.half_width_at_y_mm(fingertip.total_length_mm) == pytest.approx(
        0.0
    )


def test_distal_footprint_shrinks_with_the_half_ellipsoid() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.pad
    samples = [
        fingertip.half_width_at_y_mm(
            fingertip.straight_length_mm + fraction * pad.tip_length_mm
        )
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0)
    ]

    assert samples[0] == pytest.approx(fingertip.tip_half_width_mm)
    assert samples[-1] == pytest.approx(0.0)
    assert all(left >= right for left, right in zip(samples, samples[1:]))
    assert samples[2] == pytest.approx(fingertip.tip_half_width_mm * math.sqrt(0.75))


def test_distal_lower_surface_uses_the_ellipsoid_equation() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.pad
    y_mm = fingertip.straight_length_mm + 0.5 * pad.tip_length_mm
    expected_z_mm = (
        -pad.vertical_pad_height_mm
        - pad.elliptical_pad_height_mm * math.sqrt(0.75)
    )

    assert fingertip.lower_surface_z_mm(0.0, y_mm) == pytest.approx(expected_z_mm)
    assert fingertip.upper_surface_z_mm(0.0, y_mm) == pytest.approx(0.0)


def test_surface_is_symmetric_about_the_centerline() -> None:
    fingertip = Fingertip()
    for y_mm, x_mm in ((0.0, 4.0), (25.0, 4.0), (30.0, 3.0), (39.0, 0.3)):
        assert fingertip.lower_surface_z_mm(x_mm, y_mm) == pytest.approx(
            fingertip.lower_surface_z_mm(-x_mm, y_mm)
        )
        assert fingertip.is_inside_pad_footprint(x_mm, y_mm)
        assert fingertip.is_inside_pad_footprint(-x_mm, y_mm)


def test_invalid_distal_carrier_penetration_is_rejected() -> None:
    with pytest.raises(ValueError, match="breaks through the half-ellipsoid"):
        FingertipParameters(
            phalanx=PhalanxParameters(
                stem_height_mm=8.0,
                carrier_length_mm=39.0,
            )
        )


def test_invalid_stem_dimensions_are_rejected() -> None:
    with pytest.raises(ValueError, match="stem_width"):
        FingertipParameters(phalanx=PhalanxParameters(stem_width_mm=25.0))

    with pytest.raises(ValueError, match="outside the half-ellipsoid"):
        FingertipParameters(
            phalanx=PhalanxParameters(stem_width_mm=20.0, carrier_length_mm=39.0),
        )


def test_carrier_cross_section_is_derived_deterministic_and_symmetric() -> None:
    fingertip = Fingertip()
    cross_section = fingertip.carrier_cross_section

    assert cross_section == fingertip.carrier_cross_section
    assert len(cross_section) == 8
    assert all(
        any(
            x_other == pytest.approx(-x) and z_other == pytest.approx(z)
            for x_other, z_other in cross_section
        )
        for x, z in cross_section
    )
    assert fingertip.carrier_cross_section[0] == pytest.approx((-12.5, 0.0))
    assert fingertip.outer_envelope_z_bounds_mm(0.0, 0.0) == pytest.approx((-16.0, 0.0))


def test_queries_outside_geometry_domain_raise() -> None:
    fingertip = Fingertip()

    with pytest.raises(ValueError, match="y_mm"):
        fingertip.half_width_at_y_mm(-1.0)
    with pytest.raises(ValueError, match="x_mm"):
        fingertip.lower_surface_z_mm(13.0, 0.0)
    assert not fingertip.is_inside_pad_footprint(13.0, 0.0)


def test_dorsal_nail_rounds_off_beyond_the_stem() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.pad
    assert fingertip.dorsal_plate_half_width_at_y_mm(fingertip.straight_length_mm) == pytest.approx(12.5)
    assert fingertip.dorsal_plate_half_width_at_y_mm(fingertip.straight_length_mm + 0.6 * pad.tip_length_mm) == pytest.approx(10.0)
    assert fingertip.dorsal_plate_half_width_at_y_mm(fingertip.dorsal_plate_length_mm) == pytest.approx(0.0)
    assert fingertip.carrier_length_mm < fingertip.dorsal_plate_length_mm


def test_tip_dimensions_are_derived_from_canonical_sources() -> None:
    parameters = FingertipParameters(
        pad=PadParameters(elliptical_pad_height_mm=12.0),
        phalanx=PhalanxParameters(carrier_width_mm=30.0),
    )
    fingertip = Fingertip(parameters)

    assert fingertip.tip_length_mm == pytest.approx(parameters.pad.tip_length_mm)
    assert fingertip.tip_half_width_mm == pytest.approx(15.0)
    assert fingertip.tip_height_mm == pytest.approx(12.0)
    assert fingertip.dorsal_plate_half_width_at_y_mm(fingertip.straight_length_mm) == pytest.approx(15.0)
    assert fingertip.lower_surface_z_mm(0.0, fingertip.straight_length_mm) == pytest.approx(-18.0)


def test_tip_y_and_z_are_not_independent_parameter_fields() -> None:
    assert not hasattr(PadParameters, "tip_y_mm")
    assert not hasattr(PadParameters, "tip_z_mm")
    assert not hasattr(PhalanxParameters, "tip_y_mm")
    assert not hasattr(PhalanxParameters, "tip_z_mm")
