"""The contact pads under O-A'' and B''-C: geometry and collision pairs."""

import dataclasses
import math

import numpy as np
import pytest

from core.morphology.linkage import FingerLinkage
from core.morphology.linkage.packaging import (
    COMPONENTS,
    Packaging,
    checked_pairs,
    ContactPad,
    PadSpan,
    distal_pad_polygon,
    layer_separated_pairs,
    point_segment_distance,
    pad_outline,
    pad_spans,
    proximal_pad_polygon,
    pad_margins,
)
from core.optimization.linkage import decode, encode

# The earlier 6.5 mm pin envelope and the default pads.
HOUSING_ONLY = Packaging(pivot_clearance_diameter_mm=6.5)
GOOD = (36.0, 79.3, 10.0, 41.6, 12.6, 63.3, 17.4, 41.9, 30.0, 0.0)
MCP_SAMPLES = tuple(math.radians(degrees) for degrees in range(91))


def xz(point) -> np.ndarray:
    return np.array((point.x_mm, point.z_mm))


def test_reference_support_endpoints_project_a_prime_and_b_prime() -> None:
    linkage = FingerLinkage(decode(GOOD))
    pose = linkage.solve(0.0, 0.0)

    assert (linkage.app.x_mm, linkage.app.z_mm) == (41.6, 0.0)
    assert (linkage.bpp.x_mm, linkage.bpp.z_mm) == (63.3, 10.0)
    assert xz(pose.app) == pytest.approx(xz(linkage.app))
    assert xz(pose.bpp) == pytest.approx(xz(linkage.bpp))
    spans = pad_spans(decode(GOOD), HOUSING_ONLY)
    # The proximal pad runs on O-A'' from past the MCP servo (pad_mcp_check).
    start = HOUSING_ONLY.proximal_pad_start_mm
    assert spans["proximal_pad"].length_mm == pytest.approx(np.linalg.norm(xz(linkage.app)) - start)
    assert spans["distal_pad"].length_mm == pytest.approx(np.linalg.norm(xz(linkage.c) - xz(linkage.bpp)))


def test_support_endpoints_ride_rigidly_on_their_bodies() -> None:
    linkage = FingerLinkage(decode(GOOD))
    reference = linkage.solve(0.0, 0.0)

    def distances(pose):
        return ([np.linalg.norm(xz(pose.app) - xz(getattr(pose, name))) for name in ("o", "a", "ap")]
                + [np.linalg.norm(xz(pose.bpp) - xz(getattr(pose, name))) for name in ("b", "c", "bp")])

    for q_mcp, q_pip in ((0.0, 0.6), (0.7, 1.2), (-0.17, 1.6)):
        pose = linkage.solve(q_mcp, q_pip)
        assert distances(pose) == pytest.approx(distances(reference))
        # B'' stays on B-C, not on a fixed horizontal line.
        (ux, uz), (vx, vz) = xz(pose.c) - xz(pose.b), xz(pose.bpp) - xz(pose.b)
        cross = ux * vz - uz * vx
        assert cross == pytest.approx(0.0, abs=1e-9)
    assert linkage.solve(0.0, 1.2).bpp.z_mm != pytest.approx(linkage.bpp.z_mm, abs=1.0)


def test_pads_hang_palmar_of_their_mounting_lines_and_ride_on_their_links() -> None:
    linkage = FingerLinkage(decode(GOOD))
    t, k = HOUSING_ONLY.proximal_pad.height_mm, HOUSING_ONLY.proximal_pad.chamfer_mm
    start = HOUSING_ONLY.proximal_pad_start_mm
    # The proximal pad: a-b on O-A'', a-c square to it, the corner d under A'' chamfered.
    assert proximal_pad_polygon(linkage, HOUSING_ONLY) == pytest.approx(np.array(
        ((start, -t), (41.6 - k, -t), (41.6, -t + k), (41.6, 0.0), (start, 0.0))))

    sweep = linkage.sweep_pip(np.radians([0.0, 30.0, 70.0]))
    distal = distal_pad_polygon(linkage, HOUSING_ONLY, sweep.b, sweep.c)
    for i in range(3):
        pose = linkage.solve(0.0, float(sweep.q_pip_rad[i]))
        pad = distal[i]
        e = (xz(pose.c) - xz(pose.b)) / linkage.parameters.bc_length_mm
        palmar = np.array((e[1], -e[0]))
        # a-b is B''-C; the block is t palmar of it, chamfered at the corner c under B''.
        assert pad[3] == pytest.approx(xz(pose.bpp))
        assert pad[2] == pytest.approx(xz(pose.c))
        assert pad[1] == pytest.approx(xz(pose.c) + t * palmar)
        assert pad[0] == pytest.approx(xz(pose.bpp) + k * e + t * palmar)
        assert pad[4] == pytest.approx(xz(pose.bpp) + (t - k) * palmar)


def test_the_mcp_check_moves_the_proximal_pad_off_o() -> None:
    off = dataclasses.replace(HOUSING_ONLY, pad_mcp_check=False)
    linkage = FingerLinkage(decode(GOOD))

    # Off: the pad starts at O and the MCP servo is not checked.
    assert proximal_pad_polygon(linkage, off)[-1] == pytest.approx((0.0, 0.0))
    assert not any("mcp_servo" in pair for pair in checked_pairs(off))
    # On: it starts past the servo's shaft disk, and both pads are checked against the servo.
    on_start = HOUSING_ONLY.mcp_servo.radius_mm + HOUSING_ONLY.body_clearance_mm
    assert proximal_pad_polygon(linkage, HOUSING_ONLY)[-1] == pytest.approx((on_start, 0.0))
    assert {("mcp_servo", "proximal_pad"), ("distal_pad", "mcp_servo")} <= set(checked_pairs(HOUSING_ONLY))
    assert pad_spans(decode(GOOD), HOUSING_ONLY)["proximal_pad"].length_mm == pytest.approx(41.6 - on_start)
    assert pad_spans(decode(GOOD), off)["proximal_pad"].length_mm == pytest.approx(41.6)
    # The default offset clears the servo over MCP hyperextension and flexion.
    reference = [np.array([[p.x_mm, p.z_mm]]) for p in (linkage.b, linkage.c, linkage.bp)]
    margins = pad_margins(linkage, HOUSING_ONLY, *reference, MCP_SAMPLES)
    assert margins[("mcp_servo", "proximal_pad")][0] >= 0.0


def test_pad_offsets_move_the_outer_ends_inward() -> None:
    offset = dataclasses.replace(HOUSING_ONLY, proximal_pad=ContactPad(offset_mm=3.0),
                                 distal_pad=ContactPad(offset_mm=4.0))
    linkage = FingerLinkage(decode(GOOD))
    base, moved = (pad_spans(decode(GOOD), packaging) for packaging in (HOUSING_ONLY, offset))

    # The proximal pad starts 3 mm further right; the distal pad starts 4 mm short of C.
    assert proximal_pad_polygon(linkage, offset)[-1][0] == pytest.approx(HOUSING_ONLY.proximal_pad_start_mm + 3.0)
    tip = distal_pad_polygon(linkage, offset, np.array([xz(linkage.b)]), np.array([xz(linkage.c)]))[0, 2]
    assert tip == pytest.approx(xz(linkage.c) - (4.0, 0.0))
    assert moved["proximal_pad"].length_mm == pytest.approx(base["proximal_pad"].length_mm - 3.0)
    assert moved["distal_pad"].length_mm == pytest.approx(base["distal_pad"].length_mm - 4.0)


def test_a_fixed_pad_length_sets_the_pad_and_needs_room_on_its_line() -> None:
    from core.morphology.linkage.packaging import pad_spans

    fixed = dataclasses.replace(HOUSING_ONLY, proximal_pad=ContactPad(length_mm=20.0),
                                distal_pad=ContactPad(length_mm=30.0))
    parameters = decode(GOOD)
    spans = pad_spans(parameters, fixed)

    # Proximal: right from its start; distal: left from C. Both fit on GOOD.
    assert spans["proximal_pad"].start_mm == pytest.approx(fixed.proximal_pad_start_mm)
    assert spans["proximal_pad"].length_mm == pytest.approx(20.0)
    assert spans["distal_pad"].end_mm == pytest.approx(parameters.bc_length_mm)
    assert spans["distal_pad"].length_mm == pytest.approx(30.0)
    # A pad longer than its mounting line's room shows as a span past the room.
    too_long = pad_spans(parameters, dataclasses.replace(HOUSING_ONLY, distal_pad=ContactPad(length_mm=50.0)))
    assert too_long["distal_pad"].length_mm > too_long["distal_pad"].room_mm


def test_the_pip_servo_can_sit_on_a_and_carry_the_proximal_pad() -> None:
    on_a = dataclasses.replace(HOUSING_ONLY, pip_servo_at="a")
    linkage = FingerLinkage(decode(GOOD))
    body = on_a.pip_servo

    # The servo's shaft is on A, its body back along -X.
    footprint = on_a.pip_footprint(linkage)
    assert footprint.pivot == (36.0, 0.0) and footprint.rear == (-1.0, 0.0)
    # The proximal pad lies on the body's palmar face, as long as the body.
    pad = proximal_pad_polygon(linkage, on_a)
    assert pad[:, 1].max() == pytest.approx(-body.radius_mm)
    assert pad[:, 0].min() == pytest.approx(36.0 - body.rear_extent_mm)
    assert pad[:, 0].max() == pytest.approx(36.0 + body.radius_mm)
    assert pad_spans(decode(GOOD), on_a)["proximal_pad"].length_mm == pytest.approx(
        body.rear_extent_mm + body.radius_mm)


def test_a_chamfer_trims_only_the_palmar_corner_facing_the_pip() -> None:
    span = PadSpan(0.0, 30.0, 30.0)
    square = pad_outline(span, ContactPad(chamfer_mm=0.0), "end")
    chamfered = pad_outline(span, ContactPad(), "end")

    assert len(square) == 4 and len(chamfered) == 5
    # a-b, the mounting length, is untouched.
    assert {(0.0, 0.0), (30.0, 0.0)} <= {tuple(point) for point in chamfered}
    encoded = encode(FingerLinkage(decode(GOOD)))
    assert encoded[:-2] == (*GOOD, 0.0, 0.0, 0.0, 0.0)
    assert all(math.isnan(value) for value in encoded[-2:])  # the drops: the packaging's


def test_a_link_across_a_pad_ends_its_free_length() -> None:
    from core.morphology.linkage.packaging import usable_pad_lengths

    linkage = FingerLinkage(decode(GOOD))
    spans = pad_spans(linkage.parameters, HOUSING_ONLY)
    b, c = xz(linkage.b)[None], xz(linkage.c)[None]
    # The driver link A-B' straight down through the proximal pad at x = A_x = 36;
    # the coupler A'-B stays dorsal of both pads.
    free = usable_pad_lengths(linkage, HOUSING_ONLY, b, np.array([[36.0, -20.0]]), c)
    assert free["proximal_pad"][0] == pytest.approx(36.0 - spans["proximal_pad"].start_mm)
    assert free["distal_pad"][0] == pytest.approx(spans["distal_pad"].length_mm)


def test_only_a_crossing_of_the_pad_block_counts() -> None:
    from core.morphology.linkage.packaging import _block_extent

    # A 5 mm block over 10 < u < 40: a segment across it, one palmar of it.
    across = _block_extent(np.array([[20.0, 5.0]]), np.array([[30.0, -15.0]]), 10.0, 40.0, 5.0)
    below = _block_extent(np.array([[15.0, -8.0]]), np.array([[35.0, -8.0]]), 10.0, 40.0, 5.0)
    assert (across[0][0], across[1][0]) == pytest.approx((22.5, 25.0))
    assert (below[0][0], below[1][0]) == (np.inf, -np.inf)


def test_y_layers_decide_which_pairs_are_checked() -> None:
    pairs = checked_pairs(HOUSING_ONLY)

    # The link plates sit outside the central layer of the pads and servos.
    assert set(layer_separated_pairs(HOUSING_ONLY)) == {
        ("driver_link", "proximal_pad"), ("coupler", "proximal_pad"),
        ("distal_pad", "driver_link"), ("coupler", "distal_pad")}
    assert not any("driver_link" in pair or "coupler" in pair for pair in pairs)
    # A body's own parts and its joints' pins are its interfaces, not collisions.
    for pad, own in (("proximal_pad", ("pip_servo", "a_pin", "ap_pin")),
                         ("distal_pad", ("b_pin", "bp_pin"))):
        assert all(tuple(sorted((pad, part))) not in pairs for part in own)
    assert ("distal_pad", "proximal_pad") in pairs
    assert ("distal_pad", "pip_servo") in pairs
    assert set(COMPONENTS) >= {name for pair in pairs for name in pair}


def test_a_link_crossing_a_pad_in_the_side_view_is_not_a_collision() -> None:
    from core.morphology.linkage.packaging import _inside

    linkage = FingerLinkage(decode(GOOD))
    pad = proximal_pad_polygon(linkage, HOUSING_ONLY)
    # Flexed, the driver link A-B' passes through the proximal pad in the side view...
    pose = linkage.solve(0.0, math.radians(60.0))
    along = np.linspace(0.0, 1.0, 50)[:, None]
    link = xz(pose.a) + along * (xz(pose.bp) - xz(pose.a))
    assert _inside(link[None], pad[None]).any()
    # ...yet it is no checked pair: the link plates are in other layers.
    assert not any("driver_link" in pair for pair in checked_pairs(HOUSING_ONLY))


