"""Study 2 in the form of the paper's Eq. 9: the pivots' heights free, Delta_MCP a third objective
(redesign.py --objective eq9; experimental, docs/experimental/pr_mr.md)."""

from __future__ import annotations

import dataclasses
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from core.optimization.analysis.contact_kinematics import ContactMechanism, front_of, in_base_frame, paths
from core.optimization.mechanism import mechanism
from core.optimization.study import load_yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "scripts" / "optimize" / "prototype" / "config.yaml"
for path in (ROOT / "scripts" / "study", ROOT / "scripts" / "study" / "study2_morphology_adaptation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import redesign  # noqa: E402

PINNED = ("goal.pads.extend_past_pivots=false", "goal.link.oa_ratio=[null, null]", "goal.link.bc_ratio=[null, null]",
          "goal.collision.param.pivot_diameter_mm.a=6", "goal.collision.param.pivot_diameter_mm.a_prime=10",
          "goal.collision.param.pivot_diameter_mm.b=6", "goal.collision.param.pivot_diameter_mm.b_prime=6",
          "goal.pads.proximal.height_mm=5", "goal.pads.distal.height_mm=5",
          "goal.link.proximal_length_ratio=[null, null]", "goal.link.distal_length_ratio=[null, null]")
REVOLUTE = (mechanism("revolute"), (*PINNED, "goal.link.finger_length_mm=[90, 100]", "goal.alignment.contact_faces=false",
                                    "goal.pads.proximal.length_mm=[20, null]", "goal.pads.distal.length_mm=[25, null]",
                                    "goal.motion.required_flexion_deg=120", "goal.hardware.pip_servo.tilt_angle_deg=[90, 90]"),
            (45.0, 55.0, 30.0, 29.0, 0.0, 0.0, 90.0))
RCJ = (mechanism("rcj"), (*PINNED, "goal.pads.proximal.length_mm=[0, null]", "goal.pads.distal.length_mm=[0, null]",
                          "goal.link.finger_length_mm=[99.5, 100]", "goal.link.proximal_length_mm=[40, 60]"),
       (52.0, 9.5, 29.0, 39.5, 1.0, 9.5, 9.5, 0.0))
PARALLEL = (mechanism("revolute_parallel"), (*PINNED, "goal.pads.proximal.length_mm=[0, null]",
                                             "goal.pads.distal.length_mm=[20, null]",
                                             "goal.link.finger_length_mm=[99.5, 100]",
                                             "goal.link.proximal_length_mm=[40, 60]",
                                             "goal.motion.required_flexion_deg=110", "goal.hardware.pulley_radius_mm=6"),
            (50.0, 50.0, 38.6, 25.0, 3.0, 3.0, 20.0, 5.0, 15.0))
CASES = {"revolute": REVOLUTE, "rcj": RCJ, "revolute_parallel": PARALLEL}


def load(case):
    base, overrides, x = CASES[case]
    return base, load_yaml(base, CONFIG, overrides)[0], x


def test_the_front_of_three_objectives_keeps_what_nothing_beats():
    points = [(1.0, 1.0, -0.70), (0.5, 2.0, -0.70), (1.0, 1.0, -0.70), (2.0, 2.0, -0.60), (0.4, 3.0, -0.75)]
    assert front_of(points) == [4, 1, 0]  # by rising first value; the repeat and the beaten point out


@pytest.mark.parametrize("case", sorted(CASES))
def test_the_heights_at_zero_are_the_studies_joint(case):
    base, config, x = load(case)
    free = redesign.free_heights(config)
    padded = redesign.with_heights(base, x, free)
    assert len(padded) == len(x) + 2 and padded[-2:] == (0.0, 0.0)
    plain, zero = base.evaluate(x, config), base.evaluate(padded, free)
    assert tuple(plain.g) == tuple(zero.g)
    assert base.report(plain)["c_pad_mean"] == base.report(zero)["c_pad_mean"]
    assert redesign.heights_of(base, padded) == (0.0, 0.0) and redesign.heights_of(base, x) == (0.0, 0.0)


@pytest.mark.parametrize("case", sorted(CASES))
def test_the_heights_are_searched_after_the_joints_own_coordinates(case):
    base, config, x = load(case)
    free = redesign.free_heights(config)
    names = base.search_names(free)
    assert len(names) == len(base.search_names(config)) + 2
    i = names.index("a_z_mm")
    lower, upper = base.search_box(free)
    assert (lower[i], upper[i]) == redesign.FREE_HEIGHTS["a_z_mm"]
    assert (lower[i + 1], upper[i + 1]) == redesign.FREE_HEIGHTS["b_z_mm"]
    coordinates = list(base.from_design(redesign.with_heights(base, x, free), free))
    coordinates[i:i + 2] = (2.0, 1.0)
    found = base.to_design(coordinates, free)
    assert found is not None and redesign.heights_of(base, found) == (2.0, 1.0)
    assert base.to_design(base.from_design(found, free), free) == pytest.approx(found)


@pytest.mark.parametrize("case", ("revolute", "rcj"))
def test_a_raised_joint_lifts_the_distal_segment_and_keeps_the_pads_on_their_lines(case):
    base, config, x = load(case)
    module = __import__(f"core.optimization.{case}.evaluate", fromlist=["geometry"])
    flat, raised = module.geometry(x, config), module.geometry((*x, 2.0, 0.5), config)
    q = np.radians([0.0, 60.0])
    lift = np.array((0.0, 2.0))
    # The whole distal segment rides 2 mm higher, its link line 0.5 mm below the joint; the proximal pad stays.
    turned = np.stack((-0.5 * np.sin(q), -0.5 * np.cos(q)), axis=1)
    assert raised.tip(q) == pytest.approx(flat.tip(q) + lift + turned)
    assert raised.proximal_pad == pytest.approx(flat.proximal_pad)
    assert raised.distal_pad_local == pytest.approx(flat.distal_pad_local - (0.0, 0.5))


def test_the_base_frame_change_is_zero_against_itself_and_the_face_shift_when_the_face_moves():
    base, config, x = load("revolute")
    found = paths(base, x, config)
    depth = float(base.report(base.evaluate(x, config))["proximal_depth_mm"])
    scored = ContactMechanism(base, found["contact"], 90.0, reference_depth_mm=depth)
    assert tuple(scored.objectives) == ("contact_change", "mcp_change", "c_pad_mean")
    own = scored.evaluate(x, config)
    assert own.contact_change == pytest.approx(0.0, abs=1e-12) and own.mcp_change == pytest.approx(0.0, abs=1e-12)
    # A reference 1 mm shallower: the same path, seen from F_1, is 1 mm off.
    shallower = ContactMechanism(base, found["contact"], 90.0, reference_depth_mm=depth - 1.0).evaluate(x, config)
    assert shallower.mcp_change * 90.0 == pytest.approx(1.0)
    assert in_base_frame(found["contact"], depth)[0] == pytest.approx(found["contact"][0] - (0.0, depth))


def test_the_summary_pick_is_the_most_coverage_then_the_least_change():
    rows = [{"c_pad_mean": 0.70, "contact_change": 0.01, "mcp_change_mm": 2.0},
            {"c_pad_mean": 0.74, "contact_change": 0.03, "mcp_change_mm": 3.0},
            {"c_pad_mean": 0.74, "contact_change": 0.02, "mcp_change_mm": 5.0},
            {"c_pad_mean": 0.737, "contact_change": 0.005, "mcp_change_mm": 1.0}]
    assert redesign.pick(rows) is rows[2]
    assert redesign.pick(rows, redesign.NEAR_C_PAD) is rows[3]
    assert redesign.pick([]) is None


def test_the_linkage_frees_a_and_b_as_before():
    config = load_yaml(mechanism("linkage"), CONFIG, PINNED)[0]
    free = redesign.free_heights(config)
    assert free.bounds["a_z_mm"] == redesign.FREE_HEIGHTS["a_z_mm"] and not free.align_joint_axes


@pytest.mark.parametrize("case", sorted(CASES))
def test_above_keeps_the_pivots_on_or_above_their_lines(case):
    base, config, _ = load(case)
    free = redesign.free_heights(config, redesign.ABOVE_HEIGHTS)
    names = base.search_names(free)
    i = names.index("a_z_mm")
    lower, upper = base.search_box(free)
    assert (lower[i], lower[i + 1]) == (0.0, 0.0) and (upper[i], upper[i + 1]) == (8.0, 10.0)
