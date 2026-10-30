"""core.morphology as a library: the common packaging (rigid bodies in Y
layers, their gaps) and the linkage built on it, used without a study."""

import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from core.morphology.linkage import (
    FingerLinkage,
    LinkagePackaging,
    MotionSettings,
    checked_pairs,
    decode,
    grow_pads,
    layout,
    pad_margins,
    reference_poses,
    sweep_motion,
)
from core.morphology.linkage.packaging import JOINTS, NOT_YET_CHECKED, PARTS, assembly
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
X = (40.2, 72.8, 0.0, 44.9, 11.6, 63.8, 10.0, 25.0, 27.2, 0.0, -4.7, -9.0, 0.0, 0.0, 3.0, 3.0)
"""A production CoDe design."""
PRODUCTION = LinkagePackaging(
    pivot_clearance_diameter_mm=6.0, body_clearance_mm=0.0, a_pivot_diameter_mm=6.0, ap_pivot_diameter_mm=10.0,
    b_pivot_diameter_mm=6.0, bp_pivot_diameter_mm=6.0, ap_clearance_diameter_mm=10.0, pad_mcp_clearance_mm=0.0,
    allow_collision_pip_servo_to_joint_a=True, proximal_pad_below_a=True, distal_pad_below_b=True,
    pip_servo_above_oa=False, pip_servo_pad_check=True, a_pin_servo_check=True, links_clear_pads=True,
    pads_off_pins=True, distal_carrier_height_mm=4.0, overlap_depth=True)


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
    for name in ("a_pivot_diameter_mm", "pip_servo_at", "pads_off_pins", "allow_collision_pip_servo_to_joint_bp"):
        assert not hasattr(common, name)
    assert common.pivot_radius_mm("a") == common.pin_radius_mm
    linkage = LinkagePackaging(a_pivot_diameter_mm=10.0)
    assert isinstance(linkage, Packaging) and linkage.pivot_radius_mm("a") == 5.0
    assert linkage.pivot_radius_mm("b") == linkage.pin_radius_mm
    with pytest.raises(ValueError):
        LinkagePackaging(pip_servo_at="b")


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
    # Without the exemptions, the rule would check the not yet checked pairs too.
    every = {pair.key for pair in Layout(layout_.parts, layout_.joints).pairs}
    assert NOT_YET_CHECKED <= every and not NOT_YET_CHECKED & {pair.key for pair in layout_.pairs}


def test_a_linkage_is_checked_and_swept_without_a_study():
    linkage = FingerLinkage(decode(X, 0.1))
    rest = reference_poses(linkage)
    found = pad_margins(linkage, PRODUCTION, *(rest.points[name] for name in ("b", "c", "bp")))
    assert set(found) == set(checked_pairs(PRODUCTION)) | {("distal_carrier", "pip_servo")}
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

    x = [0.0] * 16
    y, proximal, distal, checked = grow_pads(x, screen, lambda y: y if screen(y) > -math.inf else None, coverage)
    assert coverage(checked) == pytest.approx(10.0)
    assert proximal == pytest.approx(6.0, abs=0.1) and distal == pytest.approx(4.0, abs=0.1)


def test_the_morphology_imports_no_study():
    code = ("import sys, core.morphology.linkage, core.morphology.packaging, core.morphology.contact; "
            "print(any(name.startswith('core.optimization') for name in sys.modules))")
    out = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "False"
