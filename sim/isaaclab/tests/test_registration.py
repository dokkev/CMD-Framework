# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Registration and configuration checks that do not launch a simulator."""

import gymnasium as gym
import pytest

from isaaclab.utils import configclass

import code_isaaclab.tasks  # noqa: F401
from code_isaaclab.hands import HANDS, get_hand, register_hand
from code_isaaclab.hands.code_hand import make_code_hand_spec
from code_isaaclab.physics import HandPhysicsCfg
from code_isaaclab.tasks.reorient.env_cfg import AllegroReorientEnvCfg, HandReorientEnvCfg


def test_reorient_task_is_registered_with_project_entry_points():
    spec = gym.registry["CoDe-Reorient-Cube-Allegro"]
    assert spec.kwargs["env_cfg_entry_point"].startswith("code_isaaclab.tasks.reorient")
    assert spec.kwargs["rsl_rl_cfg_entry_point"].startswith("code_isaaclab.tasks.reorient")


def test_the_env_cfg_takes_its_hand_from_the_spec():
    cfg = AllegroReorientEnvCfg()
    hand = get_hand("allegro")
    assert cfg.actuated_joint_names == list(hand.actuated_joint_names)
    assert cfg.fingertip_body_names == list(hand.fingertip_body_names)
    # configclass copies on assignment: compare by value
    assert cfg.scene.robot == hand.robot_cfg and cfg.scene.object == hand.object_cfg
    assert cfg.decimation == hand.decimation
    # Open-chain reference hand leaves closed-loop hooks inactive.
    assert cfg.events.stiffen_loops is None
    assert cfg.events.close_four_bar is None


def test_both_backends_are_presets_and_mjwarp_is_sized_for_the_hand():
    cfg = AllegroReorientEnvCfg()
    assert isinstance(cfg.sim.physics, HandPhysicsCfg)
    names = {"newton_mjwarp", "newton_mjwarp_hydro", "newton_vbd"}
    assert names <= set(vars(cfg.sim.physics))
    assert cfg.sim.physics.newton_mjwarp.solver_cfg.njmax == get_hand("allegro").mjwarp_buffers["njmax"]
    assert cfg.sim.physics.newton_vbd.solver_cfg.class_type.endswith("vbd_manager:NewtonVBDJointSyncManager")
    hydro = cfg.sim.physics.newton_mjwarp_hydro
    assert hydro.solver_cfg.use_mujoco_contacts is False
    assert hydro.collision_cfg.sdf_hydroelastic_config is not None and hydro.default_shape_cfg.is_hydroelastic


def test_coupled_vbd_cfg_assigns_rigid_and_deformable_entries():
    from code_isaaclab.physics import coupled_vbd_cfg

    cfg = coupled_vbd_cfg(rigid_bodies=["/World/envs/env_.*/Robot"], proxied_rigid_bodies=["/World/envs/env_.*/Robot"])
    rigid, soft = cfg.solver_cfg.entries
    assert type(rigid.solver_cfg).__name__ == "MJWarpSolverCfg" and rigid.bodies == ["/World/envs/env_.*/Robot"]
    assert type(soft.solver_cfg).__name__ == "VBDSolverCfg" and soft.all_particles


def test_hands_register_once():
    with pytest.raises(ValueError):
        register_hand(HANDS["allegro"])


def test_code_hand_spec_wires_closed_loop_events_and_passive_actuators(monkeypatch):
    from test_assets_and_loops import make_fixture_description

    desc = make_fixture_description(with_loops=True)
    spec = make_code_hand_spec(desc, "/tmp/fixture_code_hand.usda", name="fixture_code")
    monkeypatch.setitem(HANDS, spec.name, spec)

    @configclass
    class FixtureReorientEnvCfg(HandReorientEnvCfg):
        hand = "fixture_code"

    cfg = FixtureReorientEnvCfg()
    assert cfg.actuated_joint_names == ["index_fe", "index_pip_servo", "thumb_ip"]
    assert cfg.fingertip_body_names == ["index_distal", "thumb_distal"]
    assert "servos" in cfg.scene.robot.actuators and "passive" in cfg.scene.robot.actuators
    assert cfg.events.stiffen_loops is not None and cfg.events.stiffen_loops.mode == "startup"
    assert cfg.events.close_four_bar is not None and cfg.events.close_four_bar.mode == "reset"
    assert cfg.sim.physics.newton_mjwarp.solver_cfg.njmax == 300

    # Open-chain export (e.g. a 2R-DD baseline) omits passive group and loop events automatically.
    open_spec = make_code_hand_spec(
        make_fixture_description(with_loops=False), "/tmp/fixture_open_hand.usda", name="fixture_open"
    )
    assert "passive" not in open_spec.robot_cfg.actuators
    assert open_spec.stiffen_mjwarp_loops is False and open_spec.four_bar_tables is None
