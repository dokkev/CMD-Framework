"""Study 2: a redesign with the contact-centric kinematics as its target (problem formulation A.6.3)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

from core.optimization.analysis.contact_kinematics import (INVALID, ContactMechanism, contact_path, front,
                                                           path_change, paths)
from core.optimization.mechanism import mechanism

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "scripts" / "study"
if str(STUDY) not in sys.path:
    sys.path.insert(0, str(STUDY))

import shared  # noqa: E402

Q = np.linspace(0.0, math.radians(120.0), 61)


def revolute_tips(axis_x: float, tip_x: float = 90.0) -> np.ndarray:
    return np.array([(axis_x + (tip_x - axis_x) * math.cos(q), -(tip_x - axis_x) * math.sin(q)) for q in Q])


@pytest.fixture(scope="module")
def references():
    return {r.reference_id: r for r in shared.load_references(
        ("matched",), ROOT, "scripts/study/study2_morphology_adaptation/references_nominal15.yaml")}


def test_the_contact_path_starts_under_the_fingertip_and_keeps_the_faces_offset():
    path = contact_path(revolute_tips(50.0), Q, 5.0, 5.0)
    assert path[0] == pytest.approx((90.0, 0.0))  # both faces 5 mm deep: on one line when extended
    assert contact_path(revolute_tips(50.0), Q, 8.0, 5.0)[0] == pytest.approx((90.0, 3.0))


def test_a_fixed_axis_moves_the_contact_by_2_sin_half_q_per_depth():
    # Both faces dh deeper about the same axis: A.6.0 P4, the bound no redesign of a fixed axis beats.
    thin = contact_path(revolute_tips(50.0), Q, 3.0, 3.0)
    for dh in (2.0, 4.0, 6.0):
        thick = contact_path(revolute_tips(50.0), Q, 3.0 + dh, 3.0 + dh)
        assert path_change(thick, thin, 90.0) == pytest.approx(2 * math.sin(Q[-1] / 2) * dh / 90.0, rel=1e-9)
    # The axis moved along the link instead: the contact moves with the skeleton.
    moved = contact_path(revolute_tips(47.0), Q, 3.0, 3.0)
    assert path_change(moved, thin, 90.0) == pytest.approx(path_change(revolute_tips(47.0), revolute_tips(50.0), 90.0))


def test_a_path_that_is_missing_has_no_change():
    thin = contact_path(revolute_tips(50.0), Q, 3.0, 3.0)
    assert path_change(None, thin, 90.0) == INVALID and path_change(thin[:10], thin, 90.0) == INVALID


def test_the_front_keeps_what_nothing_beats_by_rising_change():
    points = [(0.03, 0.70), (0.01, 0.60), (0.02, 0.55), (0.05, 0.70), (0.04, 0.75)]
    assert [points[i] for i in front(points)] == [(0.01, 0.60), (0.03, 0.70), (0.04, 0.75)]


def test_a_finger_against_itself_has_no_change_and_its_own_coverage(references):
    for name in ("code_matched", "2r_dd_tilt0_matched"):
        reference = references[name]
        base = mechanism(reference.mechanism)
        own = base.evaluate(reference.x, reference.config)
        found = paths(base, reference.x, reference.config)
        if found is None:
            continue
        assert found["tip"].shape == found["contact"].shape == (61, 2)
        scored = ContactMechanism(base, found["contact"], 90.0).evaluate(reference.x, reference.config)
        assert scored.feasible == bool(own.feasible) and tuple(scored.g) == tuple(own.g)
        if own.feasible:
            assert scored.contact_change == pytest.approx(0.0, abs=1e-12)
            assert scored.c_pad_mean == pytest.approx(float(base.report(own)["c_pad_mean"]))
        else:  # not a design: it has no place on the front
            assert scored.contact_change == INVALID and scored.c_pad_mean == 0.0
