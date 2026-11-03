"""core.morphology as a library: the common packaging (rigid bodies in Y
layers, their gaps) and the linkage built on it, used without a study."""

import dataclasses
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from core.morphology.mechanism.linkage import (
    Finger,
    FourBar,
    LinkagePackaging,
    MotionSettings,
    PointXZ,
    Segment,
    ServoMount,
    checked_pairs,
    decode,
    encode,
    grow_pads,
    layout,
    linkage_icr_x,
    make_finger,
    pad_margins,
    reference_poses,
    sweep_motion,
)
from core.morphology.mechanism.linkage.model import four_bar
from core.morphology.mechanism.linkage.packaging import JOINTS, PARTS, PIN_CHECKED, PINS, assembly, pad_spans
from core.morphology.packaging import (
    PALM,
    Assembly,
    Bar,
    Disk,
    Footprint,
    Joint,
    KeepOut,
    Layout,
    Packaging,
    Pair,
    Part,
    Polygon,
    Poses,
    ServoFootprint,
    margins,
)
from core.morphology.packaging import gaps

ROOT = Path(__file__).resolve().parents[1]
CENTRAL, OUTER = frozenset({"central"}), frozenset({"outer"})
SQUARE = np.array(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)))
X = (40.2, 72.8, 0.0, 44.9, 11.6, 63.8, 10.0, 25.0, 27.2, 0.0, -4.7, -9.0, 3.0, 3.0)
"""A production CoDe design."""
PRODUCTION_A, PRODUCTION_AP = PointXZ(40.0, 0.0), PointXZ(45.0, 11.0)
PRODUCTION_B, PRODUCTION_BP = PointXZ(75.0, 0.0), PointXZ(60.0, 10.0)
PRODUCTION = LinkagePackaging(
    pivot_clearance_diameter_mm=6.0, body_clearance_mm=0.0, a_pivot_diameter_mm=6.0, ap_pivot_diameter_mm=10.0,
    b_pivot_diameter_mm=6.0, bp_pivot_diameter_mm=6.0, ap_clearance_diameter_mm=10.0, pad_mcp_clearance_mm=0.0,
    allow_collision_pip_servo_to_joint_a=True,
    pip_servo_above_oa=False, pip_servo_pad_check=True, a_pin_servo_check=True, links_clear_pads=True,
    pads_off_pins=True, overlap_depth=True)


def toy_layout(**changes) -> Layout:
    parts = (Part("pad", "lower", CENTRAL, pad=True), Part("servo", PALM, CENTRAL), Part("pin", "lower", CENTRAL | OUTER),
             Part("block", "upper", CENTRAL), Part("bar", "link", OUTER))
    values = dict(parts=parts, joints=(Joint("pin", ("lower", "link")),))
    return Layout(**{**values, **changes})


# The rule.

def test_parts_of_different_bodies_sharing_a_layer_are_checked():
    pairs = {pair.key: pair for pair in toy_layout().pairs}
    # One body's parts are not checked; the bar shares no layer with the pad or the block.
    assert set(pairs) == {("pad", "servo"), ("block", "pad"), ("pin", "servo"), ("block", "pin"), ("block", "servo")}
    # A pad keeps the MCP clearance from the palm's servo, the rest the body clearance.
    assert pairs[("pad", "servo")].clearance == "mcp" and pairs[("block", "pin")].clearance == "body"
    # The pin joins its body and the link: the joint, not a collision. Without it, the bar meets it.
    assert ("bar", "pin") in {pair.key for pair in toy_layout(joints=()).pairs}


def test_keep_outs_add_pairs_of_one_body_and_exemptions_leave_pairs_out():
    pairs = toy_layout(keep_outs=(KeepOut("pad", "pin", touch=True),), unchecked=frozenset({("block", "pad")}))
    assert ("block", "pad") not in {pair.key for pair in pairs.pairs}
    assert pairs.kept == (Pair("pad", "pin", "touch"),)
    assert pairs.pair("pin", "pad") == Pair("pad", "pin", "touch") and pairs.pair("block", "pad") is None
    assert toy_layout().separated(["pad"]) == (("bar", "pad"),)
    with pytest.raises(ValueError):
        toy_layout(keep_outs=(KeepOut("pad", "nothing"),)).pairs


# The gaps.

def toy_assembly(**shapes) -> Assembly:
    parts = tuple(Part(name, f"body_{name}", CENTRAL) for name in shapes)
    return Assembly(Layout(parts), shapes)


def test_margins_measure_each_kind_of_pair():
    packaging = Packaging(body_clearance_mm=0.5, overlap_depth=True)
    built = toy_assembly(square=Polygon(SQUARE), shifted=Polygon(SQUARE + (0.75, 0.0)), disk=Disk("p", 1.0),
                         other=Disk("q", 0.5), bar=Bar("q", "r"), servo=Footprint(ServoFootprint((10.0, 0.0), (1.0, 0.0),
                                                                                                     2.0, 5.0)))
    poses = Poses(1, points={"p": np.array((3.0, 0.5)), "q": np.array((3.0, 3.0)), "r": np.array((6.0, 3.0))})
    found = margins(built, poses, packaging, [Pair("shifted", "square"), Pair("disk", "square"), Pair("disk", "other"),
                                              Pair("bar", "square"), Pair("bar", "servo", "touch")])
    assert found[("shifted", "square")][0] == pytest.approx(-0.25 - 0.5)       # overlapping by its depth
    assert found[("disk", "square")][0] == pytest.approx(1.0 - 0.5)             # 2 from the edge, less the radius
    assert found[("disk", "other")][0] == pytest.approx(2.5 - 1.5 - 0.5)
    assert found[("bar", "square")][0] == pytest.approx(math.hypot(2.0, 2.0) - 0.5)
    assert found[("bar", "servo")][0] == pytest.approx(math.hypot(4.0, 3.0) - 2.0)  # to the servo's round front
    # Snapped: a part set exactly the clearance away does not fail by rounding.
    touching = margins(built, poses, Packaging(body_clearance_mm=1.0 + 1e-12), [Pair("disk", "square")])
    assert touching[("disk", "square")][0] == 0.0


def test_far_pairs_may_report_a_lower_bound_and_one_bodys_are_exact():
    packaging = Packaging(body_clearance_mm=0.0)
    far = (gaps.EXACT_BAND_MM + 5.0, gaps.EXACT_BAND_MM + 5.0)
    built = toy_assembly(square=Polygon(SQUARE), far=Polygon(SQUARE + far))
    found = margins(built, Poses(1), packaging, [Pair("far", "square")])
    exact = math.hypot(*far) - math.sqrt(2.0)
    assert found[("far", "square")][0] <= exact + 1e-9 and found[("far", "square")][0] > gaps.EXACT_BAND_MM
    one_body = Assembly(Layout((Part("square", "a", CENTRAL), Part("far", "a", CENTRAL))), built.shapes)
    assert margins(one_body, Poses(1), packaging, [Pair("far", "square")])[("far", "square")][0] \
        == pytest.approx(exact)


def test_moving_bodies_are_posed_by_their_frame():
    built = toy_assembly(square=Polygon(SQUARE), moving=Polygon(SQUARE))
    built = Assembly(Layout((Part("square", "fixed", CENTRAL), Part("moving", "arm", CENTRAL))), built.shapes)
    poses = Poses(2, origin={"arm": np.array(((3.0, 0.0), (0.0, 3.0)))}, axis={"arm": np.array(((1.0, 0.0), (0.0, 1.0)))})
    found = margins(built, poses, Packaging(body_clearance_mm=0.0), [Pair("moving", "square")])
    # Shifted along x by 3 (2 clear), then turned a quarter and set at (0, 3): spans x -1..0, y 3..4 (2 clear).
    assert found[("moving", "square")] == pytest.approx([2.0, 2.0])


def test_the_palm_turns_its_servo_over_the_mcp_samples():
    packaging = Packaging(body_clearance_mm=0.0, pad_mcp_clearance_mm=0.0)
    servo = packaging.mcp_footprint()
    pad = SQUARE * 5.0 + (20.0, -20.0)
    built = Assembly(Layout((Part("mcp", PALM, CENTRAL), Part("pad", "proximal", CENTRAL, pad=True))),
                     {"mcp": Footprint(servo), "pad": Polygon(pad)})
    samples = np.radians([0.0, 45.0, 90.0])
    found = margins(built, Poses(1, mcp_samples_rad=tuple(samples)), packaging, built.layout.pairs)
    assert found[("mcp", "pad")][0] == pytest.approx(gaps.proximal_pad_mcp_gap(pad, packaging, tuple(samples)))
    with pytest.raises(ValueError):  # the palm holds the packaging's MCP servo only
        margins(Assembly(built.layout, {**built.shapes, "mcp": Footprint(servo.turned(0.1))}), Poses(1), packaging)


# The packaging: common and the linkage's.

def test_the_common_packaging_knows_no_linkage():
    common = Packaging()
    for name in ("a_pivot_diameter_mm", "pads_off_pins", "allow_collision_pip_servo_to_joint_bp"):
        assert not hasattr(common, name)
    assert common.pivot_radius_mm("a") == common.pin_radius_mm
    linkage = LinkagePackaging(a_pivot_diameter_mm=10.0)
    assert isinstance(linkage, Packaging) and linkage.pivot_radius_mm("a") == 5.0
    assert linkage.pivot_radius_mm("b") == linkage.pin_radius_mm
    with pytest.raises(ValueError):
        LinkagePackaging(ap_clearance_diameter_mm=-1.0)


# The linkage, as a library.

def test_the_linkage_lays_its_parts_out_by_the_rule():
    layout_ = layout(PRODUCTION)
    names = {part.name for part in PARTS}
    assert {name for pair in layout_.pairs for name in pair.key} <= names
    # The pins join the bodies the joints say; none of them is checked against those.
    for joint in JOINTS:
        bodies = set(joint.bodies)
        assert not any(joint.pin in pair.key and {layout_.part(n).body for n in pair.key} & bodies - {
            layout_.part(joint.pin).body} for pair in layout_.pairs)
    # A pin's housing is checked against the PIP servo and the pads only, and
    # the distal carrier against the MCP servo and the proximal pad.
    for pair in layout_.pairs:
        pin = set(pair.key) & set(PINS)
        assert not pin or set(pair.key) - pin <= PIN_CHECKED
    checked = {pair.key for pair in layout_.pairs}
    assert {("b_pin", "pip_servo"), ("b_pin", "proximal_pad"), ("a_pin", "distal_pad")} <= checked
    assert not {("a_pin", "coupler"), ("a_pin", "mcp_servo"), ("a_pin", "b_pin")} & checked


def test_a_finger_is_its_segments_its_joint_and_its_servo():
    finger = decode(X, 0.1)
    assert isinstance(finger, Finger) and isinstance(finger.joint, FourBar) and finger.middle is None
    # The proximal segment from O to A'', the distal one from B'' to C, its frame on B.
    assert finger.proximal.start_mm == 0.0 and finger.proximal.end_mm == pytest.approx(finger.proximal_end.x_mm)
    assert finger.distal.end_mm == pytest.approx(finger.tip.x_mm - finger.joint.b.x_mm)          # |BC|
    assert finger.distal.length_mm == pytest.approx(finger.tip.x_mm - finger.distal_start.x_mm)           # |B''C|
    # A pad's room is its segment's length less the MCP clearance and the tip's offset.
    spans = pad_spans(finger, PRODUCTION)
    assert spans["proximal_pad"].room_mm == pytest.approx(finger.proximal.length_mm - PRODUCTION.proximal_pad_start_mm)
    assert spans["distal_pad"].room_mm == pytest.approx(finger.distal.length_mm - PRODUCTION.distal_pad.offset_mm)
    # The design vector round-trips through the finger.
    assert encode(decode(encode(finger))) == pytest.approx(encode(finger), nan_ok=True)
    # The servo's tilt is how it is mounted, not the joint's.
    assert finger.servo == ServoMount(math.radians(X[7]))
    sketch = make_finger(PRODUCTION_A, PRODUCTION_AP, PRODUCTION_B, PRODUCTION_BP, 30.0)
    assert sketch.distal == Segment(PRODUCTION_BP.x_mm - PRODUCTION_B.x_mm, 30.0)
    with pytest.raises(ValueError):
        make_finger(PRODUCTION_A, PRODUCTION_AP, PRODUCTION_B, PRODUCTION_BP, 0.0)


def test_the_finger_places_its_points_on_the_frame_its_joint_poses():
    finger = decode(X, 0.1)
    q = math.radians(60.0)
    pose = finger.solve(0.0, q)
    sweep = finger.sweep_pip([q])
    b = pose.distal_origin
    assert pose.tip.array == pytest.approx(sweep.tip[0])
    # C and B'' stay on the distal segment: |BC| and |BB''| as extended, B'' on the line B -> C.
    assert b.distance_to(pose.tip) == pytest.approx(finger.distal.end_mm)
    assert b.distance_to(pose.distal_start) == pytest.approx(finger.distal.start_mm)
    assert pose.distal_start.distance_to(pose.tip) == pytest.approx(finger.distal.length_mm)
    # The MCP turns the whole finger about O.
    turned = finger.solve(math.radians(30.0), q)
    assert math.hypot(turned.tip.x_mm, turned.tip.z_mm) == pytest.approx(math.hypot(pose.tip.x_mm, pose.tip.z_mm))


def test_the_linkage_needs_a_four_bar_joint():
    finger = decode(X, 0.1)
    other = dataclasses.replace(finger, joint=object())
    for check in (lambda: assembly(other, PRODUCTION), lambda: four_bar(other)):
        with pytest.raises(TypeError):
            check()
    assert four_bar(finger) is finger.joint


def test_the_design_vector_reads_its_past_layouts():
    current = decode(X, 0.1)
    # 16: with the deprecated chamfers between the offsets and the drops.
    assert encode(decode((*X[:12], 2.5, 2.5, *X[12:]), 0.1)) == pytest.approx(encode(current))
    # Before the offsets and the drops: those at their defaults.
    assert encode(decode(X[:12], 0.1))[:12] == pytest.approx(encode(current)[:12])
    assert all(math.isnan(v) for v in encode(decode(X[:12], 0.1))[12:])
    for length in (8, 13, 15, 17):
        with pytest.raises(ValueError):
            decode((*X, 0.0, 0.0, 0.0)[:length], 0.1)


def test_a_linkage_is_checked_and_swept_without_a_study():
    linkage = decode(X, 0.1)
    rest = reference_poses(linkage)
    found = pad_margins(linkage, PRODUCTION, *(rest.points[name] for name in ("b", "c", "bp")))
    assert set(found) == set(checked_pairs(PRODUCTION))
    assert min(float(margin.min()) for margin in found.values()) >= 0.0
    step = math.radians(1.0)
    settings = MotionSettings(servo_samples_rad=tuple(np.arange(0.0, math.pi + 1e-9, step)), servo_step_rad=step,
                              hyperextension_rad=0.0, mcp_samples_rad=(0.0,),
                              transmission_angle_range_rad=(math.radians(30.0), math.radians(150.0)),
                              dip_backoff_tolerance_rad=math.radians(0.05))
    motion = sweep_motion(linkage, PRODUCTION, settings, math.radians(150.0))
    assert motion.phi_safe_rad >= math.radians(150.0) - 1e-9
    assert motion.torque_ratio_max > 0.0 and motion.pad_clearance_mm >= 0.0
    assert set(motion.fold_free) == {"proximal_pad", "distal_pad"}
    built = assembly(linkage, PRODUCTION)
    assert set(built.shapes) == {part.name for part in built.layout.parts}


def test_pads_grow_to_the_most_coverage_with_the_shortest_extension():
    # Coverage grows with each extension up to 6 and 4 mm, then holds; past 9 mm a pad is not acceptable.
    def coverage(y):
        return min(y[10], 6.0) + min(y[11], 4.0)

    def screen(y):
        return coverage(y) if y[10] <= 9.0 and y[11] <= 9.0 else -math.inf

    x = [0.0] * 14
    y, proximal, distal, checked = grow_pads(x, screen, lambda y: y if screen(y) > -math.inf else None, coverage)
    assert coverage(checked) == pytest.approx(10.0)
    assert proximal == pytest.approx(6.0, abs=0.1) and distal == pytest.approx(4.0, abs=0.1)


def test_the_morphology_imports_no_study():
    code = ("import sys, core.morphology.mechanism.linkage, core.morphology.packaging, core.morphology.contact; "
            "print(any(name.startswith('core.optimization') for name in sys.modules))")
    out = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "False"


def test_the_finger_tracks_its_centre_of_rotation_at_every_step():
    finger = decode(X, 0.1)
    joint = finger.joint
    # Extended: where the bars A-B' and A'-B cross, as the measures read it.
    assert finger.icr == joint.icr and finger.icr.x_mm == pytest.approx(linkage_icr_x(finger))
    q = np.radians(np.arange(0.0, 91.0, 5.0))
    sweep = finger.sweep_pip(q)
    for i, angle in enumerate(q):
        pose = finger.solve(0.0, float(angle))
        assert pose.icr.array == pytest.approx(sweep.icr[i])
        # On both bars' lines: no velocity of B' or B across their bar about the centre.
        for pivot, moving in ((pose.a, pose.bp), (pose.ap, pose.b)):
            bar, to_icr = moving - pivot, pose.icr - pivot
            assert bar.x_mm * to_icr.z_mm - bar.z_mm * to_icr.x_mm == pytest.approx(0.0, abs=1e-6)
    # It is the distal segment's centre of rotation: B and C move about it as rigid.
    step = 1e-6
    for i in (3, 10):
        here, there = finger.solve(0.0, float(q[i])), finger.solve(0.0, float(q[i]) + step)
        turn = there.distal_rad - here.distal_rad
        for point in ("b", "tip"):
            moved = here.icr + (getattr(here, point) - here.icr).rotated(turn)
            assert moved.distance_to(getattr(there, point)) < 1e-9
    # Turned with the MCP, and over the swept motion.
    turned = finger.solve(math.radians(30.0), float(q[4]))
    assert turned.icr.array == pytest.approx(finger.solve(0.0, float(q[4])).icr.rotated(-math.radians(30.0)).array)
    step = math.radians(1.0)
    settings = MotionSettings(servo_samples_rad=tuple(np.arange(0.0, math.pi + 1e-9, step)), servo_step_rad=step,
                              hyperextension_rad=0.0, mcp_samples_rad=(0.0,),
                              transmission_angle_range_rad=(math.radians(30.0), math.radians(150.0)),
                              dip_backoff_tolerance_rad=math.radians(0.05))
    motion = sweep_motion(finger, PRODUCTION, settings, math.radians(150.0))
    assert motion.icr_mm.shape[1] == 2 and motion.icr_mm[0] == pytest.approx(finger.icr.array)


def test_the_2r_is_a_pin_its_finger_owns():
    from core.morphology.finger import Kinematics
    from core.morphology.mechanism import revolute

    x = (40.0, 30.0, 34.0, 4.0, 3.0, 3.0, 10.0)
    finger = revolute.decode(x)
    assert isinstance(finger.joint, revolute.Pin) and finger.icr == PointXZ(40.0, 0.0)
    assert finger.tip == PointXZ(70.0, 0.0) and finger.proximal_end == PointXZ(34.0, 0.0) and finger.distal_start == PointXZ(44.0, 0.0)
    assert finger.servo == ServoMount(math.radians(10.0))
    geo = revolute.geometry(x, PRODUCTION)
    q = np.radians(np.arange(0.0, 151.0, 10.0))
    kinematics = finger.kinematics(q)
    assert isinstance(kinematics, Kinematics)
    assert kinematics.tip == pytest.approx(geo.tip(q))                       # the solids' tip
    assert kinematics.icr == pytest.approx(np.broadcast_to((40.0, 0.0), (len(q), 2)))
    assert kinematics.transmission_ratio == pytest.approx(np.ones(len(q)))
    assert np.hypot(*(kinematics.distal_start - kinematics.icr).T) == pytest.approx(np.full(len(q), 4.0))
    pose = finger.solve(math.radians(20.0), math.radians(50.0))
    assert pose.tip.array == pytest.approx(finger.kinematics([math.radians(50.0)], math.radians(20.0)).tip[0])
    with pytest.raises(TypeError):  # the CoDe's checks need its four-bar
        four_bar(finger)


def test_the_finger_gives_its_kinematics_whatever_its_joint():
    finger = decode(X, 0.1)
    q = np.radians(np.arange(0.0, 121.0, 2.0))
    kinematics = finger.kinematics(q)
    sweep = finger.sweep_pip(q)
    assert kinematics.flexion_rad == pytest.approx(sweep.total_flexion_rad)
    assert kinematics.icr == pytest.approx(sweep.icr) and kinematics.tip == pytest.approx(sweep.tip)
    for i in (0, 20, 50):
        pose = finger.solve(0.0, float(q[i]))
        assert kinematics.distal_start[i] == pytest.approx(pose.distal_start.array) and kinematics.proximal_end == pytest.approx(pose.proximal_end.array)
    # The ratio: the flexion per servo turn, as the motion's k reads it.
    assert np.all(kinematics.transmission_ratio > 0.0)
    turned = finger.kinematics(q, math.radians(30.0))
    assert np.hypot(*turned.icr.T) == pytest.approx(np.hypot(*kinematics.icr.T))


def test_the_finger_pip_is_the_distal_segments_flexion_from_the_proximal():
    finger = decode(X, 0.1)
    for degrees in (-10.0, 0.0, 35.0, 90.0, 140.0):
        q = math.radians(degrees)
        pose = finger.solve(0.0, q)
        along = pose.tip - pose.b  # B -> C, along +X extended
        assert -math.atan2(along.z_mm, along.x_mm) == pytest.approx(q)
        assert pose.q_pip_rad == pytest.approx(pose.q_driver_rad + pose.q_dip_rad)
        # The same pose solved by the driver link's rotation.
        by_driver = finger.joint.solve_driver(pose.q_driver_rad)
        assert by_driver.b.array == pytest.approx(pose.b.array) and by_driver.q_pip_rad == pytest.approx(q)
    q = np.radians(np.arange(-10.0, 141.0, 5.0))
    by_pip = finger.sweep_pip(q)
    by_driver = finger.place(finger.joint.sweep_driver(by_pip.q_driver_rad))
    assert by_pip.q_pip_rad == pytest.approx(q)
    assert by_driver.tip == pytest.approx(by_pip.tip) and by_driver.q_pip_rad == pytest.approx(q)


def test_a_margin_within_rounding_of_touching_is_no_violation():
    from core.morphology.packaging.hardware import MARGIN_TOLERANCE_MM, violation_mm

    assert violation_mm(-1e-16) == 0.0 and violation_mm(1e-16) == 0.0
    assert violation_mm(2.0) == -2.0 and violation_mm(-0.5) == 0.5
    assert violation_mm(-2 * MARGIN_TOLERANCE_MM) > 0.0
