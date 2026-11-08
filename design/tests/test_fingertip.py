
import math

import pytest

from core.morphology import Fingertip, FingertipParameters
from core.morphology.pad import (
    FingertipFamilyParameters,
    PadParameters,
    PhalanxParameters,
    stem_corner_pad_thickness_mm,
)


def test_default_parameters_construct_successfully() -> None:
    fingertip = Fingertip()

    assert fingertip.parameters == FingertipParameters()
    assert fingertip.half_width_mm == pytest.approx(8.0)
    assert fingertip.total_length_mm == pytest.approx(24.0)
    assert fingertip.carrier_length_mm == pytest.approx(21.0)
    assert fingertip.straight_length_mm == pytest.approx(16.0)
    assert fingertip.parameters.family.carrier_distal_inset_mm == pytest.approx(3.0)


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

    for y_mm in (-8.0, -4.0, 0.0, 4.0, 8.0):
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
    for x_mm, y_mm in ((0.0, 4.0), (16.0, 4.0), (19.0, 3.0), (23.5, 0.3)):
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
    assert short.carrier_length_mm - short.straight_length_mm == pytest.approx(5.0)
    assert long.carrier_length_mm - long.straight_length_mm == pytest.approx(5.0)
    assert long.lower_surface_z_mm(long.straight_length_mm + 5.0, 0.0) == pytest.approx(
        short.lower_surface_z_mm(short.straight_length_mm + 5.0, 0.0)
    )


def test_rotation_axis_keeps_its_place_relative_to_the_tip() -> None:
    family = FingertipFamilyParameters(rotation_axis_offset_mm=2.0)
    short = Fingertip(FingertipParameters(family=family, total_length_mm=40.0))
    long = Fingertip(FingertipParameters(family=family, total_length_mm=60.0))

    assert short.rotation_axis_mm(20.0) == pytest.approx((20.0, 0.0, 2.0))
    assert long.rotation_axis_mm(20.0) == pytest.approx((40.0, 0.0, 2.0))
    assert short.pad_depth_below_rotation_axis_mm == pytest.approx(2.0 + 3.0 + 7.0)
    for offset_mm in (-6.0, 4.0):
        with pytest.raises(ValueError, match="rotation_axis_offset_mm"):
            FingertipFamilyParameters(rotation_axis_offset_mm=offset_mm)


def test_stem_corner_pad_thickness_is_the_distance_to_the_pad_boundary() -> None:
    # A round pad centred on the shoulder plane: the corner-to-arc distance is
    # the radius minus the corner's distance from the centre.
    pad = PadParameters(
        vertical_pad_width_mm=20.0, vertical_pad_height_mm=1e-9,
        elliptical_pad_width_mm=20.0, elliptical_pad_height_mm=10.0,
    )
    phalanx = PhalanxParameters(stem_width_mm=6.0, stem_height_mm=4.0, carrier_width_mm=20.0)
    assert stem_corner_pad_thickness_mm(pad, phalanx) == pytest.approx(
        10.0 - 5.0, abs=1e-3
    )
    assert Fingertip().stem_corner_pad_thickness_mm >= 3.0
    with pytest.raises(ValueError, match="minimum_pad_thickness_mm"):
        FingertipFamilyParameters(minimum_pad_thickness_mm=9.0)


def test_every_pad_kind_shares_the_pad_interface(tmp_path) -> None:
    import numpy as np
    import trimesh

    from core.morphology.hand_layout import fingertip_attachment
    from core.morphology.pad import MeshPad, Pad, RectangularPad

    block = RectangularPad(length_mm=24.0, width_mm=16.0, thickness_mm=10.0, rotation_axis_z_mm=-1.0)
    box = trimesh.creation.box(extents=(24.0, 16.0, 10.0))
    box.apply_translation((12.0, 0.0, -5.0))
    path = tmp_path / "pad.stl"
    box.export(path)
    mesh = MeshPad.from_file(path, rotation_axis_z_mm=-1.0)
    pads = (Fingertip(), block, mesh)

    for pad in pads:
        assert isinstance(pad, Pad)
        assert pad.length_mm == pytest.approx(24.0)
        assert pad.rotation_axis_mm(20.0) == pytest.approx((4.0, 0.0, pad.rotation_axis_z_mm))
        assert fingertip_attachment(pad, 20.0) is not None
    for pad in (block, mesh):
        outline = np.array(pad.side_silhouette_mm(8))
        assert pad.lowest_z_mm == pytest.approx(-10.0)
        assert outline[:, 0].min() == pytest.approx(0.0)
        assert outline[:, 1].max() == pytest.approx(0.0)


def test_mesh_pad_must_start_at_the_proximal_end() -> None:
    from core.morphology.pad import MeshPad

    with pytest.raises(ValueError, match="x = 0"):
        MeshPad([(1.0, 0.0, 0.0), (2.0, 0.0, 0.0), (1.0, 1.0, 0.0)], [(0, 1, 2)])
