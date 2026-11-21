# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The reorientation goals: drawn within range, far enough apart, and reproducible as a fixed sequence."""

import math

import torch

from code_isaaclab.tasks.reorient.code_mdp import draw_goals, goal_sequence

MAX, STEP = math.radians(30.0), math.radians(25.0)


def test_draw_goals_respects_range_and_step():
    previous = torch.linspace(-MAX, MAX, 101)
    for _ in range(20):
        goals = draw_goals(previous, MAX, STEP, torch.rand_like)
        assert bool((goals.abs() <= MAX + 1e-6).all())
        assert bool(((goals - previous).abs() >= STEP - 1e-6).all())
        previous = goals


def test_goal_sequence_is_fixed_by_its_seed():
    a = goal_sequence(64, 40, MAX, STEP, seed=1000)
    b = goal_sequence(64, 40, MAX, STEP, seed=1000)
    c = goal_sequence(64, 40, MAX, STEP, seed=1001)
    assert torch.equal(a, b)
    assert not torch.equal(a, c)
    steps = torch.diff(torch.cat([torch.zeros(64, 1), a], dim=1), dim=1).abs()
    assert bool((steps >= STEP - 1e-6).all())
    assert bool((a.abs() <= MAX + 1e-6).all())

