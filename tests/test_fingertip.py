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


def test_straight_body_center_and_edge_surface_values() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.pad

    assert fingertip.lower_surface_z_mm(0.0, 0.0) == pytest.approx(
        -pad.pad_straight_height_mm - pad.ellipse_height_mm
    )
    assert fingertip.lower_surface_z_mm(pad.width_mm / 2.0, 0.0) == pytest.approx(
        -pad.pad_straight_height_mm
    )
    assert fingertip.lower_surface_z_mm(-pad.width_mm / 2.0, pad.body_length_mm) == pytest.approx(
        -pad.pad_straight_height_mm
    )


def test_distal_surface_matches_straight_body_at_join() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.pad

    for x_mm in (-12.5, -6.0, 0.0, 6.0, 12.5):
        assert fingertip.lower_surface_z_mm(x_mm, pad.body_length_mm) == pytest.approx(
            fingertip.lower_surface_z_mm(x_mm, pad.body_length_mm - 1e-9)
        )


def test_distal_cap_closes_at_the_distal_end() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.pad
    end_z_mm = -(pad.pad_straight_height_mm + pad.ellipse_height_mm) / 2.0

    assert fingertip.lower_surface_z_mm(0.0, fingertip.total_length_mm) == pytest.approx(
        end_z_mm
    )
    assert fingertip.upper_surface_z_mm(0.0, fingertip.total_length_mm) == pytest.approx(
        end_z_mm
    )
    assert fingertip.half_width_at_y_mm(fingertip.total_length_mm) == pytest.approx(
        pad.width_mm / 2.0
    )


def test_distal_footprint_keeps_constant_width_through_rounded_cap() -> None:
    fingertip = Fingertip()
    pad = fingertip.parameters.pad
    samples = [
        fingertip.half_width_at_y_mm(pad.body_length_mm + fraction * pad.tip_length_mm)
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0)
    ]

    assert samples == pytest.approx([pad.width_mm / 2.0] * len(samples))


def test_surface_is_symmetric_about_the_centerline() -> None:
    fingertip = Fingertip()
    for y_mm, x_mm in ((0.0, 4.0), (25.0, 4.0), (30.0, 3.0), (39.0, 0.3)):
        assert fingertip.lower_surface_z_mm(x_mm, y_mm) == pytest.approx(
            fingertip.lower_surface_z_mm(-x_mm, y_mm)
        )
        assert fingertip.is_inside_pad_footprint(x_mm, y_mm)
        assert fingertip.is_inside_pad_footprint(-x_mm, y_mm)


def test_invalid_distal_carrier_penetration_is_rejected() -> None:
    with pytest.raises(ValueError, match="maximum compliant pad depth"):
        FingertipParameters(
            phalanx=PhalanxParameters(
                stem_height_mm=17.0,
                carrier_extension_mm=14.0,
            )
        )


def test_invalid_stem_dimensions_are_rejected() -> None:
    with pytest.raises(ValueError, match="stem_width"):
        FingertipParameters(phalanx=PhalanxParameters(stem_width_mm=25.0))

    with pytest.raises(ValueError, match="narrower than the dorsal plate"):
        FingertipParameters(
            phalanx=PhalanxParameters(stem_width_mm=20.0, carrier_extension_mm=14.0),
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
    assert fingertip.dorsal_plate_half_width_at_y_mm(pad.body_length_mm) == pytest.approx(12.5)
    assert fingertip.dorsal_plate_half_width_at_y_mm(pad.body_length_mm + 0.6 * pad.tip_length_mm) == pytest.approx(10.0)
    assert fingertip.dorsal_plate_half_width_at_y_mm(fingertip.dorsal_plate_length_mm) == pytest.approx(0.0)
    assert fingertip.carrier_length_mm < fingertip.dorsal_plate_length_mm
