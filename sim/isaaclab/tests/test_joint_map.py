# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The flexion-to-servo map: inverse, transmission ratio and gain schedule on a known table."""

import math

import numpy as np
import pytest

from code_isaaclab.joint_map import FlexionServoMap, flexion_maps


def _table(fn, lo=-10.0, hi=120.0):
    q = np.arange(lo, hi + 0.5, 1.0)
    return {"pip_deg": q.tolist(), "ap_deg": np.degrees(fn(np.radians(q))).tolist()}


SMOOTH = lambda q: 0.7 * q + 0.1 * np.sin(q)  # noqa: E731  dtheta/dq = 0.7 + 0.1 cos q, in [0.6, 0.8]


def test_servo_and_flexion_are_inverse():
    m = FlexionServoMap.from_table("index", _table(SMOOTH))
    q = np.radians(np.linspace(-5.0, 115.0, 200))
    assert np.max(np.abs(m.flexion(m.servo(q)) - q)) < 1e-9


def test_ratio_matches_the_derivative_within_table_resolution():
    m = FlexionServoMap.from_table("index", _table(SMOOTH))
    q = np.radians(np.linspace(0.0, 110.0, 50))
    exact = 0.7 + 0.1 * np.cos(q)
    assert np.max(np.abs(m.ratio(q) - exact)) < 1e-4  # central difference, 1 deg step


def test_servo_gain_equalizes_flexion_stiffness():
    m = FlexionServoMap.from_table("index", _table(SMOOTH))
    q = np.radians(np.linspace(0.0, 110.0, 50))
    k = 3.0
    assert np.allclose(m.servo_gain(q, k) * m.ratio(q) ** 2, k)


def test_decreasing_servo_and_actuator_zero():
    m = FlexionServoMap.from_table("index", _table(lambda q: -0.6 * q), zero_deg=20.0)
    q = np.radians(30.0)
    assert m.flexion(m.servo(q)) == pytest.approx(q)
    assert m.actuator(q) == pytest.approx(-0.6 * q - math.radians(20.0))


def test_non_monotonic_table_is_rejected():
    with pytest.raises(ValueError):
        FlexionServoMap.from_table("index", _table(lambda q: np.cos(q)))


def test_flexion_maps_reads_a_description():
    desc = {"joints": [{"name": "index_pip_servo", "actuator": {"zero_deg": 5.0}}],
            "four_bar": {"index": {**_table(SMOOTH), "a_deg": [], "b_deg": []}}}
    maps = flexion_maps(desc)
    assert set(maps) == {"index"} and maps["index"].zero == pytest.approx(math.radians(5.0))


def test_torch_tables_agree_with_the_numpy_map():
    torch = pytest.importorskip("torch")
    from code_isaaclab.flexion_control import FlexionTables

    table = _table(SMOOTH)
    m = FlexionServoMap.from_table("index", table)
    t = FlexionTables({"index": table}, dtype=torch.float64)
    q = np.radians(np.linspace(-5.0, 115.0, 101))
    qt = torch.tensor(q, dtype=torch.float64)
    assert np.allclose(t.servo("index_pip_servo", qt).numpy(), m.servo(q), atol=1e-12)
    assert np.allclose(t.flexion("index_pip_servo", t.servo("index_pip_servo", qt)).numpy(), q, atol=1e-9)
    assert np.allclose(t.ratio("index_pip_servo", qt).numpy(), m.ratio(q), atol=1e-12)
