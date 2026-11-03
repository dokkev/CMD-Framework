"""The contact pads under O-A'' and B''-C: geometry and collision pairs."""

import dataclasses
import math

import numpy as np
import pytest

from core.morphology.mechanism.linkage import Finger
from core.morphology.mechanism.linkage.packaging import (
    COMPONENTS,
    LinkagePackaging,
    checked_pairs,
    distal_pad_polygon,
    layer_separated_pairs,
    pad_margins,
    pad_spans,
    pip_footprint,
    proximal_pad_polygon,
)
from core.morphology.packaging.hardware import ContactPad
from core.morphology.packaging.shapes import PadSpan, pad_outline, point_segment_distance
from core.morphology.mechanism.linkage.design import decode, encode

# The earlier 6.5 mm pin envelope and the default pads.
HOUSING_ONLY = LinkagePackaging(pivot_clearance_diameter_mm=6.5)
GOOD = (36.0, 79.3, 10.0, 41.6, 12.6, 63.3, 17.4, 41.9, 30.0, 0.0)
MCP_SAMPLES = tuple(math.radians(degrees) for degrees in range(91))


def xz(point) -> np.ndarray:
    return np.array((point.x_mm, point.z_mm))


def test_reference_support_endpoints_project_a_prime_and_b_prime() -> None:
    linkage = decode(GOOD)
    pose = linkage.solve(0.0, 0.0)

    assert (linkage.proximal_end.x_mm, linkage.proximal_end.z_mm) == (41.6, 0.0)
    assert (linkage.distal_start.x_mm, linkage.distal_start.z_mm) == (63.3, 10.0)
    assert xz(pose.proximal_end) == pytest.approx(xz(linkage.proximal_end))
    assert xz(pose.distal_start) == pytest.approx(xz(linkage.distal_start))
    spans = pad_spans(decode(GOOD), HOUSING_ONLY)
    # The proximal pad runs on O-A'' from past the MCP servo (pad_mcp_check).
    start = HOUSING_ONLY.proximal_pad_start_mm
    assert spans["proximal_pad"].length_mm == pytest.approx(np.linalg.norm(xz(linkage.proximal_end)) - start)
    assert spans["distal_pad"].length_mm == pytest.approx(np.linalg.norm(xz(linkage.tip) - xz(linkage.distal_start)))


def test_support_endpoints_ride_rigidly_on_their_bodies() -> None:
    linkage = decode(GOOD)
    reference = linkage.solve(0.0, 0.0)

    def distances(pose):
        return ([np.linalg.norm(xz(pose.proximal_end) - xz(getattr(pose, name))) for name in ("o", "a", "ap")]
                + [np.linalg.norm(xz(pose.distal_start) - xz(getattr(pose, name))) for name in ("b", "tip", "bp")])

    for q_mcp, q_pip in ((0.0, 0.6), (0.7, 1.2), (-0.17, 1.6)):
        pose = linkage.solve(q_mcp, q_pip)
        assert distances(pose) == pytest.approx(distances(reference))
        # B'' stays on B-C, not on a fixed horizontal line.
        (ux, uz), (vx, vz) = xz(pose.tip) - xz(pose.b), xz(pose.distal_start) - xz(pose.b)
        cross = ux * vz - uz * vx
        assert cross == pytest.approx(0.0, abs=1e-9)
    assert linkage.solve(0.0, 1.2).distal_start.z_mm != pytest.approx(linkage.distal_start.z_mm, abs=1.0)


def test_pads_hang_palmar_of_their_mounting_lines_and_ride_on_their_links() -> None:
    linkage = decode(GOOD)
    t = HOUSING_ONLY.proximal_pad.height_mm
    r = HOUSING_ONLY.interface_offset_mm("proximal_pad")
    start = HOUSING_ONLY.proximal_pad_start_mm
    # The proximal pad: a-b along O-A'', a plain block t deep from the line
    # tangent to A's housing (r palmar of OA).
    assert proximal_pad_polygon(linkage, HOUSING_ONLY) == pytest.approx(np.array(
        ((start, -r - t), (41.6, -r - t), (41.6, -r), (start, -r))))

    sweep = linkage.sweep_pip(np.radians([0.0, 30.0, 70.0]))
    distal = distal_pad_polygon(linkage, HOUSING_ONLY, sweep.b, sweep.tip)
    for i in range(3):
        pose = linkage.solve(0.0, float(sweep.q_pip_rad[i]))
        pad = distal[i]
        e = (xz(pose.tip) - xz(pose.b)) / linkage.distal.end_mm
        palmar = np.array((e[1], -e[0]))
        # a-b is B''-C, r palmar of BC (tangent to B's housing); the block t deep from there.
        r = HOUSING_ONLY.interface_offset_mm("distal_pad")
        assert pad[3] == pytest.approx(xz(pose.distal_start) + r * palmar)
        assert pad[2] == pytest.approx(xz(pose.tip) + r * palmar)
        assert pad[1] == pytest.approx(xz(pose.tip) + (r + t) * palmar)
        assert pad[0] == pytest.approx(xz(pose.distal_start) + (r + t) * palmar)


def test_the_mcp_check_moves_the_proximal_pad_off_o() -> None:
    off = dataclasses.replace(HOUSING_ONLY, pad_mcp_check=False)
    linkage = decode(GOOD)

    # Off: the pad starts at O and the MCP servo is not checked.
    r = HOUSING_ONLY.interface_offset_mm("proximal_pad")
    assert proximal_pad_polygon(linkage, off)[-1] == pytest.approx((0.0, -r))
    assert not any("mcp_servo" in pair for pair in checked_pairs(off))
    # On: it starts past the servo's shaft disk, and both pads are checked against the servo.
    on_start = HOUSING_ONLY.mcp_servo.radius_mm + HOUSING_ONLY.body_clearance_mm
    assert proximal_pad_polygon(linkage, HOUSING_ONLY)[-1] == pytest.approx((on_start, -r))
    assert {("mcp_servo", "proximal_pad"), ("distal_pad", "mcp_servo")} <= set(checked_pairs(HOUSING_ONLY))
    assert pad_spans(decode(GOOD), HOUSING_ONLY)["proximal_pad"].length_mm == pytest.approx(41.6 - on_start)
    assert pad_spans(decode(GOOD), off)["proximal_pad"].length_mm == pytest.approx(41.6)
    # The default offset clears the servo over MCP hyperextension and flexion.
    reference = [np.array([[p.x_mm, p.z_mm]]) for p in (linkage.joint.b, linkage.tip, linkage.joint.bp)]
    margins = pad_margins(linkage, HOUSING_ONLY, *reference, MCP_SAMPLES)
    assert margins[("mcp_servo", "proximal_pad")][0] >= 0.0


def test_pad_offsets_move_the_outer_ends_inward() -> None:
    offset = dataclasses.replace(HOUSING_ONLY, proximal_pad=ContactPad(offset_mm=3.0),
                                 distal_pad=ContactPad(offset_mm=4.0))
    linkage = decode(GOOD)
    base, moved = (pad_spans(decode(GOOD), packaging) for packaging in (HOUSING_ONLY, offset))

    # The proximal pad starts 3 mm further right; the distal pad starts 4 mm short of C.
    assert proximal_pad_polygon(linkage, offset)[-1][0] == pytest.approx(HOUSING_ONLY.proximal_pad_start_mm + 3.0)
    tip = distal_pad_polygon(linkage, offset, np.array([xz(linkage.joint.b)]), np.array([xz(linkage.tip)]))[0, 2]
    assert tip == pytest.approx(xz(linkage.tip) - (4.0, HOUSING_ONLY.interface_offset_mm("distal_pad")))
    assert moved["proximal_pad"].length_mm == pytest.approx(base["proximal_pad"].length_mm - 3.0)
    assert moved["distal_pad"].length_mm == pytest.approx(base["distal_pad"].length_mm - 4.0)


def test_a_fixed_pad_length_sets_the_pad_and_needs_room_on_its_line() -> None:
    from core.morphology.mechanism.linkage.packaging import pad_spans

    fixed = dataclasses.replace(HOUSING_ONLY, proximal_pad=ContactPad(length_mm=20.0),
                                distal_pad=ContactPad(length_mm=30.0))
    finger = decode(GOOD)
    spans = pad_spans(finger, fixed)

    # Proximal: right from its start; distal: left from C. Both fit on GOOD.
    assert spans["proximal_pad"].start_mm == pytest.approx(fixed.proximal_pad_start_mm)
    assert spans["proximal_pad"].length_mm == pytest.approx(20.0)
    assert spans["distal_pad"].end_mm == pytest.approx(finger.distal.end_mm)
    assert spans["distal_pad"].length_mm == pytest.approx(30.0)
    # A pad longer than its mounting line's room shows as a span past the room.
    too_long = pad_spans(finger, dataclasses.replace(HOUSING_ONLY, distal_pad=ContactPad(length_mm=50.0)))
    assert too_long["distal_pad"].length_mm > too_long["distal_pad"].room_mm


def test_a_pad_is_a_plain_block_and_a_recorded_chamfer_is_left_out() -> None:
    from core.morphology.mechanism.linkage.design import read_design
    from core.optimization.linkage.config import packaging_from_mapping

    block = pad_outline(PadSpan(0.0, 30.0, 30.0), ContactPad(height_mm=5.0))
    assert block == pytest.approx(np.array(((0.0, -5.0), (30.0, -5.0), (30.0, 0.0), (0.0, 0.0))))
    assert encode(decode(GOOD)) == (*GOOD, 0.0, 0.0)
    # Runs before the chamfers went (deprecated): their chamfers are left out.
    row = {name: str(value) for name, value in zip(("a_x_mm", "b_x_mm", "b_z_mm", "ap_x_mm", "ap_z_mm", "bp_x_mm",
                                                    "bp_z_mm", "servo_tilt_deg", "bc_length_mm", "a_z_mm"), GOOD)}
    assert read_design({**row, "proximal_chamfer_mm": "2.5"}) == read_design(row)
    recorded = {"height_mm": 5.0, "chamfer_mm": 2.5, "chamfer_free": True}
    plain = LinkagePackaging(proximal_pad=ContactPad(height_mm=5.0))
    assert packaging_from_mapping({"proximal_pad": recorded}) == plain
    with pytest.raises(ValueError):  # a vector of a length it never had (13: an old one with its chamfers?)
        decode((*GOOD, 0.0, 0.0, 2.5))
    # The PIP servo on A' (the only one modelled): a recorded "ap" is left out, "a" refused.
    assert packaging_from_mapping({"pip_servo_at": "ap"}) == LinkagePackaging()
    with pytest.raises(ValueError):
        packaging_from_mapping({"pip_servo_at": "a"})


def test_a_link_across_a_pad_ends_its_free_length() -> None:
    from core.morphology.mechanism.linkage.packaging import usable_pad_lengths

    linkage = decode(GOOD)
    spans = pad_spans(linkage, HOUSING_ONLY)
    b, c = xz(linkage.joint.b)[None], xz(linkage.tip)[None]
    # The driver link A-B' straight down through the proximal pad at x = A_x = 36;
    # the coupler A'-B stays dorsal of both pads.
    free = usable_pad_lengths(linkage, HOUSING_ONLY, b, np.array([[36.0, -20.0]]), c)
    assert free["proximal_pad"][0] == pytest.approx(36.0 - spans["proximal_pad"].start_mm)
    assert free["distal_pad"][0] == pytest.approx(spans["distal_pad"].length_mm)


def test_only_a_crossing_of_the_pad_block_counts() -> None:
    from core.morphology.mechanism.linkage.packaging import _block_extent

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
    from core.morphology.packaging.shapes import points_inside

    linkage = decode(GOOD)
    pad = proximal_pad_polygon(linkage, HOUSING_ONLY)
    # Flexed, the driver link A-B' passes through the proximal pad in the side view...
    pose = linkage.solve(0.0, math.radians(140.0))
    along = np.linspace(0.0, 1.0, 50)[:, None]
    link = xz(pose.a) + along * (xz(pose.bp) - xz(pose.a))
    assert points_inside(link[None], pad[None]).any()
    # ...yet it is no checked pair: the link plates are in other layers.
    assert not any("driver_link" in pair for pair in checked_pairs(HOUSING_ONLY))


