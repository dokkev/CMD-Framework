# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Unit tests for ``code_isaaclab.assets`` and ``code_isaaclab.loops`` (no Isaac Lab runtime required)."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import torch

from code_isaaclab.assets import (
    FORMAT,
    Description,
    build_newton,
    quat_wxyz_to_matrix,
    quat_xyzw_to_matrix,
    rotation_about,
    write_urdf,
    write_usd,
)
from code_isaaclab.loops import (
    FourBarTables,
    loop_residual_mm,
    measure_finger_pip_deg,
    stiffen_mujoco_loops,
    sync_four_bar_joints_on_reset,
)


def make_fixture_description(*, with_loops: bool = True) -> Description:
    """Self-contained ``code_hand_sim/1`` fixture (one four-bar finger + one two-joint thumb) for unit tests."""
    pip = [-10.0, 0.0, 60.0, 120.0]
    ap = [-4.0, 0.0, 25.0, 50.0]
    a = [-5.0, 0.0, 30.0, 62.0]
    b = [-6.0, 0.0, 35.0, 70.0]
    closed_joints = [
        {
            "name": "index_pip_servo",
            "parent": "index_proximal",
            "child": "index_servo_link",
            "anchor": [0.058, 0.0, 0.012],
            "axis": [0.0, 1.0, 0.0],
            "limits_deg": [-4.0, 50.0],
            "actuator": {"zero_deg": 0.0, "servo": "xm335"},
        },
        {
            "name": "index_b",
            "parent": "index_servo_link",
            "child": "index_distal",
            "anchor": [0.086, 0.0, 0.0],
            "axis": [0.0, 1.0, 0.0],
            "limits_deg": [-6.0, 70.0],
            "actuator": None,
        },
        {
            "name": "index_a",
            "parent": "index_proximal",
            "child": "index_driver_link",
            "anchor": [0.055, 0.0, 0.0],
            "axis": [0.0, 1.0, 0.0],
            "limits_deg": [-5.0, 62.0],
            "actuator": None,
        },
    ]
    open_joints = [
        {
            "name": "index_pip",
            "parent": "index_proximal",
            "child": "index_distal",
            "anchor": [0.055, 0.0, 0.0],
            "axis": [0.0, 1.0, 0.0],
            "limits_deg": [-10.0, 120.0],
            "actuator": {"zero_deg": 0.0, "servo": "xm335"},
        }
    ]
    data = {
        "format": FORMAT,
        "source": {"commit": "deadbeef12345678", "dirty": False, "parameters": "fixture.yaml", "date": "2026-11-17"},
        "units": {"length": "m", "angle": "deg", "mass": "kg"},
        "bodies": [
            body
            for body in (
                {
                    "name": "palm",
                    "origin": [0.0, 0.0, 0.0],
                    "mass": 0.3,
                    "com": [0.0, 0.0, 0.0],
                    "fixed": True,
                    "capsule": {"start": [0.0, 0.0, 0.0], "end": [0.0, -0.04, 0.0], "radius": 0.015},
                },
                {
                    "name": "index_proximal",
                    "origin": [0.02, 0.0, 0.0],
                    "mass": 0.03,
                    "com": [0.035, 0.0, 0.0],
                    "capsule": {"start": [0.02, 0.0, 0.0], "end": [0.055, 0.0, 0.0], "radius": 0.004},
                    "boxes": [
                        {
                            "name": "index_proximal_pad",
                            "center": [0.038, 0.0, -0.006],
                            "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
                            "half_extents": [0.015, 0.002, 0.009],
                        }
                    ],
                },
                {
                    "name": "index_servo_link",
                    "origin": [0.058, 0.0, 0.012],
                    "mass": 0.003,
                    "com": [0.072, 0.0, 0.006],
                    "capsule": {"start": [0.058, 0.0, 0.012], "end": [0.086, 0.0, 0.0], "radius": 0.0025},
                },
                {
                    "name": "index_driver_link",
                    "origin": [0.055, 0.0, 0.0],
                    "mass": 0.003,
                    "com": [0.067, 0.0, 0.003],
                    "capsule": {"start": [0.055, 0.0, 0.0], "end": [0.079, 0.0, 0.006], "radius": 0.0025},
                },
                {
                    "name": "index_distal",
                    "origin": [0.086, 0.0, 0.0],
                    "mass": 0.01,
                    "com": [0.097, 0.0, 0.0],
                    "capsule": {"start": [0.079, 0.0, 0.006], "end": [0.108, 0.0, 0.0], "radius": 0.004},
                    "boxes": [
                        {
                            "name": "index_distal_pad",
                            "center": [0.095, 0.0, -0.006],
                            "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
                            "half_extents": [0.012, 0.002, 0.009],
                        }
                    ],
                },
                {
                    "name": "thumb_distal",
                    "origin": [-0.03, 0.02, 0.0],
                    "mass": 0.015,
                    "com": [-0.03, 0.035, 0.0],
                    "capsule": {"start": [-0.03, 0.02, 0.0], "end": [-0.03, 0.05, 0.0], "radius": 0.005},
                },
            )
            if with_loops or body["name"] not in ("index_servo_link", "index_driver_link")
        ],
        "collision": {
            "filtered_pairs": (
                [
                    ["palm", "index_proximal"],
                    ["index_proximal", "index_servo_link"],
                    ["index_proximal", "index_driver_link"],
                    ["index_servo_link", "index_distal"],
                    ["index_driver_link", "index_distal"],
                ]
                if with_loops
                else [["palm", "index_proximal"]]
            )
        },
        "joints": [
            {
                "name": "index_fe",
                "parent": "palm",
                "child": "index_proximal",
                "anchor": [0.02, 0.0, 0.0],
                "axis": [0.0, 1.0, 0.0],
                "limits_deg": [-10.0, 100.0],
                "actuator": {"zero_deg": 0.0, "servo": "xm335"},
            },
            *(closed_joints if with_loops else open_joints),
            {
                "name": "thumb_ip",
                "parent": "palm",
                "child": "thumb_distal",
                "anchor": [-0.03, 0.02, 0.0],
                "axis": [1.0, 0.0, 0.0],
                "limits_deg": [-10.0, 90.0],
                "actuator": {"zero_deg": 0.0, "servo": "xm335"},
            },
        ],
        "loops": (
            [
                {
                    "name": "index_bp",
                    "parent": "index_driver_link",
                    "child": "index_distal",
                    "anchor": [0.079, 0.0, 0.006],
                    "axis": [0.0, 1.0, 0.0],
                }
            ]
            if with_loops
            else []
        ),
        "servo": {
            "model": "XM335-T323-T",
            "supply_voltage_v": 11.1,
            "stall_torque_nm": 1.03,
            "no_load_speed_rad_s": 5.550147021341967,
            "duty_per_rad": 11.509510008905426,
            "duty_limit": 1.0,
            "armature_kg_m2": 0.0031,
        },
        "four_bar": ({"index": {"pip_deg": pip, "ap_deg": ap, "a_deg": a, "b_deg": b}} if with_loops else {}),
        "tips": {"index": [0.108, 0.0, 0.0], "thumb": [-0.03, 0.05, 0.0]},
    }
    return Description(data)


def test_description_validates_format_and_queries_metadata():
    with pytest.raises(ValueError, match="code_hand_sim/1"):
        Description({"format": "other/0"})

    desc = make_fixture_description()
    assert desc.actuated() == ["index_fe", "index_pip_servo", "thumb_ip"]
    assert desc.passive() == ["index_b", "index_a"]
    assert desc.four_bar_digits() == ("index",)
    assert desc.fingertip_bodies() == ["index_distal", "thumb_distal"]

    pose_pip = desc.closed_pose("index", 30.0)
    pose_servo = desc.closed_pose_from_servo("index", pose_pip["index_pip_servo"])
    for key in pose_pip:
        assert pose_servo[key] == pytest.approx(pose_pip[key], abs=1e-9)

    with pytest.raises(KeyError):
        desc.body("missing_body")
    with pytest.raises(KeyError):
        desc.joint("missing_joint")


def test_quaternions_and_relative_rotation_conventions():
    angle = math.radians(37.0)
    c, s = math.cos(angle / 2.0), math.sin(angle / 2.0)
    r_xyzw = quat_xyzw_to_matrix(np.array([0.0, s, 0.0, c]))
    r_wxyz = quat_wxyz_to_matrix(np.array([c, 0.0, s, 0.0]))
    assert np.allclose(r_xyzw, r_wxyz)
    assert rotation_about(np.eye(3), r_xyzw, np.array([0.0, 1.0, 0.0])) == pytest.approx(37.0, abs=1e-9)


def test_write_urdf_and_usd_author_loops_colliders_and_passive_margins(tmp_path: Path):
    from pxr import Usd, UsdPhysics

    desc = make_fixture_description()
    urdf_path = write_urdf(desc, tmp_path / "hand.urdf", passive_limit_margin_deg=2.5)
    root = ET.parse(urdf_path).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    assert "index_bp_loop_parent_anchor" in joints and "index_bp_loop_child_anchor" in joints
    passive_b = joints["index_b"].find("limit")
    assert float(passive_b.get("lower")) == pytest.approx(math.radians(-8.5), abs=1e-6)
    assert float(passive_b.get("upper")) == pytest.approx(math.radians(72.5), abs=1e-6)
    # Actuated joints keep their exact exported limits.
    fe_lim = joints["index_fe"].find("limit")
    assert float(fe_lim.get("upper")) == pytest.approx(math.radians(100.0), abs=1e-6)

    usd_path = write_usd(
        desc,
        tmp_path / "hand_ball_colliders.usda",
        loop="ball",
        colliders=True,
        passive_limit_margin_deg=2.5,
    )
    stage = Usd.Stage.Open(str(usd_path))
    assert stage.GetDefaultPrim().HasAPI(UsdPhysics.ArticulationRootAPI)
    cut = UsdPhysics.SphericalJoint(stage.GetPrimAtPath("/code_hand/joints/index_bp"))
    assert cut.GetExcludeFromArticulationAttr().Get() is True
    b_joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath("/code_hand/joints/index_b"))
    assert b_joint.GetUpperLimitAttr().Get() == pytest.approx(72.5)
    pad_prim = stage.GetPrimAtPath("/code_hand/index_proximal/index_proximal_pad")
    assert pad_prim.HasAPI(UsdPhysics.CollisionAPI)
    prox_prim = stage.GetPrimAtPath("/code_hand/index_proximal")
    assert prox_prim.HasAPI(UsdPhysics.FilteredPairsAPI)


def test_newton_builder_and_mujoco_loop_stiffening():
    from newton.solvers import SolverMuJoCo

    desc = make_fixture_description(with_loops=True)
    builder, _, loop_ids = build_newton(desc, loop="ball")
    assert len(loop_ids) == 1
    model = builder.finalize()
    assert int(model.joint_articulation.numpy()[loop_ids[0]]) == -1
    solver = SolverMuJoCo(model)
    assert stiffen_mujoco_loops(solver, step_dt=0.003, min_timeconst=0.004, solimp=(0.98, 0.995)) is True
    solref = solver.mjw_model.eq_solref.numpy().reshape(-1, 2)
    solimp = solver.mjw_model.eq_solimp.numpy().reshape(-1, 5)
    # 2 * step_dt = 0.006 > min_timeconst 0.004 -> clamped to 0.006 for MuJoCo stability.
    assert solref[0, 0] == pytest.approx(0.006)
    assert tuple(solimp[0, :2]) == pytest.approx((0.98, 0.995))

    # Open-chain model (no equality constraints) is a clean no-op.
    open_desc = make_fixture_description(with_loops=False)
    open_builder, _, open_loops = build_newton(open_desc, loop="none")
    assert open_loops == []
    open_solver = SolverMuJoCo(open_builder.finalize())
    assert stiffen_mujoco_loops(open_solver, step_dt=0.002) is False
    assert stiffen_mujoco_loops(object(), step_dt=0.002) is False


def test_four_bar_tables_vectorized_closure_and_reset_hook():
    desc = make_fixture_description()
    names = [j["name"] for j in desc.joints]
    tables = FourBarTables.from_description(desc, names)

    q = torch.zeros((3, len(names)), dtype=torch.float32)
    closed = tables.close_from_pip(q, torch.tensor([0.0, math.radians(30.0), math.radians(90.0)]))
    ref_30 = desc.closed_pose("index", 30.0)
    assert math.degrees(float(closed[1, names.index("index_pip_servo")])) == pytest.approx(
        ref_30["index_pip_servo"], abs=1e-4
    )
    assert math.degrees(float(closed[1, names.index("index_a")])) == pytest.approx(ref_30["index_a"], abs=1e-4)
    assert math.degrees(float(closed[1, names.index("index_b")])) == pytest.approx(ref_30["index_b"], abs=1e-4)

    # Corrupt passive joints and verify close_from_servo and the reset hook restore them.
    corrupted = closed.clone()
    corrupted[:, names.index("index_a")] = 0.0
    corrupted[:, names.index("index_b")] = 0.0
    restored = tables.close_from_servo(corrupted)
    assert torch.allclose(restored, closed, atol=1e-6)

    written: dict[str, object] = {}

    class _FakeRobot:
        joint_names = names

        class data:
            joint_pos = corrupted.clone()
            joint_vel = torch.ones_like(corrupted)

        @staticmethod
        def write_joint_state_to_sim(pos, vel, env_ids=None):
            written["pos"] = pos
            written["vel"] = vel
            written["env_ids"] = env_ids

    class _FakeEnv:
        num_envs = 3
        scene = {"robot": _FakeRobot()}

    env_ids = torch.tensor([1, 2])
    sync_four_bar_joints_on_reset(_FakeEnv(), env_ids, tables=desc.four_bar)
    assert torch.allclose(written["pos"], closed[env_ids], atol=1e-6)
    assert float(written["vel"][:, names.index("index_a")].abs().max()) == 0.0
    assert float(written["vel"][:, names.index("index_b")].abs().max()) == 0.0
    assert float(written["vel"][:, names.index("index_fe")].min()) == 1.0


def test_loop_residual_and_pip_measurement_at_zero_pose():
    desc = make_fixture_description()
    body_names = [b["name"] for b in desc.bodies]
    pos = np.array([b["origin"] for b in desc.bodies], dtype=float)
    quat = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (len(body_names), 1))
    assert loop_residual_mm(desc, body_names, pos, quat) == pytest.approx(0.0, abs=1e-9)
    assert measure_finger_pip_deg(desc, "index", body_names, quat) == pytest.approx(0.0, abs=1e-9)

    # Displace the distal segment by 2 mm along X -> residual equals 2.0 mm.
    shifted = pos.copy()
    shifted[body_names.index("index_distal"), 0] += 0.002
    assert loop_residual_mm(desc, body_names, shifted, quat) == pytest.approx(2.0, abs=1e-9)
