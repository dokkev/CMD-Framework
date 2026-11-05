"""The study's mounting policy (core.optimization.mounting): each pad at the
least depth its own joint hardware needs, within what that hardware supports."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from core.optimization.analysis.tradeoff import Modules, with_modules
from core.optimization.linkage.search import from_design, search_names, to_design
from core.optimization.mechanism import mechanism
from core.optimization.mounting import footprint_reach_mm, mounting
from core.optimization.study import load_yaml

CONFIG = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "linkage" / "config.yaml"
RULES = ("goal.link.finger_length_mm=[90, 110]", "goal.pads.proximal.length_mm=[20, null]",
         "goal.pads.distal.length_mm=[25, null]", "goal.collision.check.a_pin_and_pip_servo=false",
         "goal.collision.check.a_pin_and_a_prime_pin=true", "goal.hardware.pip_servo.tilt_angle_deg=10",
         "goal.hardware.pip_servo.tilt_step_deg=1", "goal.pads.proximal.past_icr_max_mm=null",
         "goal.link.distal_length_mm=[25, null]", "goal.link.joint_span_ratio=[null, null]",
         "goal.link.proximal_length_ratio=[null, null]", "goal.link.distal_length_ratio=[null, null]",
         "goal.pads.on_links=false", "goal.motion.dip_flexion_deg=120",
         "goal.pads.mount_policy=shallowest_supported")
LINKAGE, REVOLUTE = mechanism("linkage"), mechanism("revolute")
CODE = [40.6, 66.0, 0.0, 44.4, 12.1, 56.9, 6.0, 10.0, 33.9, 0.0, -7.0, -12.1, 0.0, 0.0]
"""Both pads short of their pins: the proximal one ends 3 mm before A, the distal one starts 3 mm past B."""
CODE_UNDER_B = [*CODE[:11], -5.0, 0.0, 3.0]
"""The distal pad run back under B (its end 4.1 mm behind B), at the policy's depth."""
DD = [45.0, 55.0, 30.0, 29.0, 0.0, 0.0, 90.0]
"""Both pads short of the servo's disk (9.5 mm about A)."""


def load(*extra: str):
    return (load_yaml(LINKAGE, CONFIG, (*RULES, *extra))[0], load_yaml(REVOLUTE, CONFIG, (*RULES, *extra))[0])


def test_a_pad_short_of_its_hardware_mounts_on_its_line():
    code, dd = load()
    for name, x, config in (("linkage", CODE, code), ("revolute", DD, dd)):
        found = mounting(name, x, config)
        assert found.depths_mm == (0.0, 0.0) and found.cases == ("A", "A")


def test_a_pad_under_its_hardware_mounts_at_the_least_depth_that_clears_it():
    code, dd = load()
    # CoDe: the distal pad under B, as deep as B's housing reaches, no deeper.
    found = mounting("linkage", CODE_UNDER_B, code)
    radius = code.packaging.pivot_radius_mm("b") + code.packaging.body_clearance_mm
    assert found.depths_mm == (0.0, pytest.approx(radius)) and found.cases == ("A", "B")
    assert found.native_mm[1] == pytest.approx(radius)
    # The 2R-DD: the least depth depends on how far the pad runs under the servo.
    partly = mounting("revolute", [45.0, 55.0, 40.0, 29.0, 0.0, 0.0, 90.0], dd)  # its end 5 mm short of A
    fully = mounting("revolute", [45.0, 55.0, 40.0, 29.0, 0.0, 0.0, 0.0], dd)  # under the servo laid back
    servo = dd.goal.packaging.pip_servo.radius_mm
    assert partly.cases[0] == fully.cases[0] == "B"
    assert 0.0 < partly.depths_mm[0] < servo and fully.depths_mm[0] == pytest.approx(servo)
    # The least: a step shallower collides, as the evaluator sees it.
    shallower = [45.0, 55.0, 40.0, 29.0, 0.0, round(partly.depths_mm[0] - 0.1, 9), 90.0]
    assert REVOLUTE.evaluate(shallower, dd).constraints["pip_servo_pad_mm"] > 0.0


def test_the_two_depths_are_independent():
    code, dd = load()
    assert mounting("linkage", CODE_UNDER_B, code).depths_mm == (0.0, 3.0)
    assert mounting("revolute", [45.0, 55.0, 40.0, 29.0, 0.0, 0.0, 90.0], dd).cases == ("B", "A")


def test_the_2r_dds_servo_supports_its_distal_pad_only_when_the_depths_are_aligned():
    _, dd = load()
    # The servo is on the proximal body; the distal pad hangs from a truss off
    # the output at A: under the servo it would need a spacer (case C).
    under = [45.0, 55.0, 30.0, 4.0, 0.0, 0.0, 90.0]
    found = mounting("revolute", under, dd)
    assert found.native_mm[1] == 0.0 and found.cases[1] == "C" and not found.supported
    # Aligned: one depth for both pads, the deeper least one, the distal pad
    # carried to the servo's reach.
    _, aligned = load("goal.pads.align_mount_offsets=true")
    both = [45.0, 55.0, 44.0, 4.0, 0.0, 0.0, 90.0]  # the proximal pad under the servo's lowest too
    found = mounting("revolute", both, aligned)
    servo = aligned.goal.packaging.pip_servo.radius_mm
    assert found.depths_mm == (servo, servo) and found.native_mm == (pytest.approx(servo), pytest.approx(servo))
    assert found.cases == ("B", "B")
    x = [45.0, 55.0, 44.0, 4.0, servo, servo, 90.0]
    assert REVOLUTE.evaluate(x, aligned).constraints["mount_depth_mm"] == 0.0
    # The proximal pad only partly under it: the servo carries it no deeper
    # than its housing reaches over the pad, short of the distal pad's depth.
    found = mounting("revolute", [45.0, 55.0, 40.0, 4.0, 0.0, 0.0, 90.0], aligned)
    assert found.depths_mm == (8.7, 8.7) and found.native_mm == (8.1, pytest.approx(servo))
    assert found.cases == ("C", "B")
    # A proximal pad short of the servo would hang at that depth with nothing over it.
    assert mounting("revolute", under, aligned).cases == ("C", "B")


def test_a_servo_housing_carries_its_own_segments_pad_as_deep_as_it_reaches_over_it():
    code, dd = load()
    # CoDe: A' low, the PIP servo hangs below OA over the proximal pad's end,
    # short of A. The servo is fixed on the proximal segment: it carries the
    # pad (2026-11-05), at the least depth clearing it.
    low = [*CODE[:4], 6.0, *CODE[5:10], -6.9, *CODE[11:]]
    found = mounting("linkage", low, code)
    assert found.cases == ("B", "A") and found.depths_mm[0] == found.native_mm[0] == 2.5
    assert LINKAGE.evaluate([*low[:12], 0.0, 0.0], code).constraints["mount_depth_mm"] == pytest.approx(2.5)
    assert LINKAGE.evaluate([*low[:12], 2.5, 0.0], code).constraints["mount_depth_mm"] == 0.0
    # The 2R-DD's alike: the servo's footprint over the pad, not its whole reach.
    partly = mounting("revolute", [45.0, 55.0, 40.0, 29.0, 0.0, 0.0, 90.0], dd)  # its end 5 mm short of A
    assert partly.native_mm[0] == 8.1 < dd.goal.packaging.pip_servo.radius_mm


def test_a_housing_reaches_only_over_the_span_it_shares_with_the_pad():
    disk = np.array([(10.0 * math.cos(t), 10.0 * math.sin(t)) for t in np.linspace(0.0, 2.0 * math.pi, 721)])
    assert footprint_reach_mm(disk, (-20.0, 20.0), 0.0) == pytest.approx(10.0)  # under its lowest point
    assert footprint_reach_mm(disk, (-20.0, -6.0), 0.0) == pytest.approx(8.0, abs=1e-3)  # 6 mm short of it
    assert footprint_reach_mm(disk, (-20.0, -6.0), 0.5) == pytest.approx(8.5, abs=1e-3)  # plus the clearance
    assert footprint_reach_mm(disk, (12.0, 20.0), 0.5) == 0.0  # the pad skirts it: nothing over the pad


def test_a_depth_beyond_the_native_support_is_no_design():
    # Deeper than the allowed range: B's housing needs 3 mm, 2 are allowed.
    capped, _ = load("goal.pads.distal.mount_offset_mm=[0, 2]")
    found = mounting("linkage", CODE_UNDER_B, capped)
    assert found.cases[1] == "C" and not found.supported
    assert LINKAGE.evaluate([*CODE_UNDER_B[:13], 2.0], capped).constraints["mount_depth_mm"] > 0.0


def test_an_evaluated_design_must_carry_the_policys_depths():
    code, _ = load()
    assert LINKAGE.evaluate(CODE_UNDER_B, code).constraints["mount_depth_mm"] == 0.0
    deeper = [*CODE_UNDER_B[:13], 3.5]  # hung lower than it needs
    shallower = [*CODE_UNDER_B[:13], 0.0]  # into B's housing
    assert LINKAGE.evaluate(deeper, code).constraints["mount_depth_mm"] == pytest.approx(0.5)
    assert LINKAGE.evaluate(shallower, code).constraints["mount_depth_mm"] == pytest.approx(3.0)
    # The search sets them, not searches them: its vector is the geometry evaluated.
    assert not any("mount" in name for name in search_names(code))
    assert to_design(from_design(CODE_UNDER_B, code), code) == tuple(CODE_UNDER_B)


def test_the_pads_against_each_other_and_the_motion_stay_the_evaluators():
    code, _ = load()
    # A thick pair: the depths are the policy's whatever the thickness (the
    # hardware is above the pads), and are not deepened to dodge the fold.
    thick = with_modules(code, Modules(20.0, 16.0, 35.0, 16.0))
    thin = with_modules(code, Modules(20.0, 3.0, 35.0, 3.0))
    assert mounting("linkage", CODE_UNDER_B, thick) == mounting("linkage", CODE_UNDER_B, thin)
    evaluation = LINKAGE.evaluate(CODE_UNDER_B, thick)
    static = ("valid_reference", "pivot_pin_mm", "pin_spacing_mm", "proximal_support_mm", "distal_support_mm",
              "mount_depth_mm", "pip_servo_pad_mm", "pad_collision_mm")
    assert all(evaluation.constraints[name] <= 0.0 for name in static)  # it mounts, extended
    assert not evaluation.feasible and evaluation.constraints["rom_deg"] > 0.0  # the pads meet in the fold


def test_the_policy_is_the_studys_and_one_per_pad():
    code, dd = load()
    assert code.mount_policy == dd.goal.mount_policy == "shallowest_supported"
    production, _ = load("goal.pads.mount_policy=searched")
    assert production.mount_policy == "searched"
    assert LINKAGE.evaluate([*CODE_UNDER_B[:13], 3.5], production).constraints["mount_depth_mm"] == 0.0
    # Aligned, CoDe's pads share the deeper least depth too, each against its own housings.
    aligned, _ = load("goal.pads.align_mount_offsets=true")
    found = mounting("linkage", CODE_UNDER_B, aligned)
    assert found.depths_mm == (3.0, 3.0) and found.cases[1] == "B"


def test_the_2r_dd_searches_no_depth_and_the_belt_2r_takes_no_policy():
    from core.optimization.revolute.search import search_names as revolute_names

    code, dd = load()
    assert not any("drop" in name for name in revolute_names(dd))
    assert len(revolute_names(dd)) == len(revolute_names(load("goal.pads.mount_policy=searched")[1])) - 2
    with pytest.raises(ValueError, match="belt 2R"):
        load_yaml(mechanism("revolute_parallel"), CONFIG, RULES)


RCJ = mechanism("rcj")
RCJ_X = [36.0, 9.5, 35.0, 36.0, 8.6, 0.0, 0.0, 10.0]
"""The proximal pad to A (under its roller and the servo, tilted 10 deg), the distal one 8.6 mm past B (under its roller's side)."""


def test_the_rcj_mounts_each_pad_under_its_own_roller():
    from core.optimization.mounting import disk_reach_mm

    config = load_yaml(RCJ, CONFIG, (*RULES,))[0]
    found = mounting("rcj", RCJ_X, config)
    # The proximal pad reaches A: the roller's (and the servo's) lowest point, r below OA.
    assert found.cases == ("B", "B") and found.depths_mm[0] == found.native_mm[0] == 9.5
    # The distal pad starts 8.6 mm past B: as deep as the roller's side there, sqrt(9.5^2 - 8.6^2).
    assert found.depths_mm[1] == found.native_mm[1] == pytest.approx(4.1)
    assert disk_reach_mm((0.0, 0.0), 9.5, (8.6, 30.0)) == pytest.approx((9.5 ** 2 - 8.6 ** 2) ** 0.5)
    x = [*RCJ_X[:5], *found.depths_mm[::-1], 10.0]  # distal_drop_mm, proximal_drop_mm, the tilt
    assert RCJ.evaluate(x, config).constraints["mount_depth_mm"] == 0.0
    assert RCJ.evaluate(RCJ_X, config).constraints["mount_depth_mm"] == pytest.approx(9.5)
    # The search sets them: no drop searched, and its designs carry the policy's depths.
    assert not any("drop" in name for name in RCJ.search_names(config))
    assert RCJ.to_design(RCJ.from_design(x, config), config) == tuple(x)
    # A roller larger than the depth range allows (r 12 > 10 mm): a spacer, no design.
    big = mounting("rcj", [33.0, 12.0, 33.0, 33.0, 8.6, 0.0, 0.0, 10.0], config)
    assert big.cases[0] == "C" and not big.supported


def test_the_idealized_rcj_mounts_its_pads_under_its_rollers_alone():
    rcj = load_yaml(RCJ, CONFIG, (*RULES,))[0]
    ideal = load_yaml(mechanism("rcj_idealized"), CONFIG, (*RULES,))[0]
    # The proximal pad ending 6 mm short of A: under the untilted servo's floor, 9.5 mm;
    # without the servo, under the roller's side alone, sqrt(9.5^2 - 6^2) = 7.37.
    short = [36.0, 9.5, 35.0, 30.0, 8.6, 0.0, 0.0, 0.0]
    assert mounting("rcj", short, rcj).depths_mm[0] == 9.5
    found = mounting("rcj_idealized", short, ideal)
    assert found.depths_mm[0] == found.native_mm[0] == pytest.approx(7.4)
    # Its rollers as small as A's pin housing: the pads as shallow as they are.
    assert ideal.bounds["roller_radius_mm"][0] == ideal.goal.packaging.pivot_radius_mm("a") == 3.0
    small = mounting("rcj_idealized", [36.0, 4.0, 35.0, 36.0, 8.6, 0.0, 0.0, 0.0], ideal)
    assert small.depths_mm == (4.0, 0.0) and small.cases == ("B", "A")
