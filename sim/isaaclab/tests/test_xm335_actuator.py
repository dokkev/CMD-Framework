# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The XM335 servo configs reproduce the drive law exported by ``design/core/hardware/xm335.py``."""

import pytest
import torch

# The export's servo block at 11.1 V (design/scripts/export/code_hand_sim.py), copied so this test needs no export.
SERVO = {
    "model": "XM335-T323-T",
    "supply_voltage_v": 11.1,
    "stall_torque_nm": 1.03,
    "no_load_speed_rad_s": 5.550147021341967,
    "duty_per_rad": 11.509510008905426,
    "duty_limit": 1.0,
    "armature_kg_m2": 0.005,
}


def test_dc_motor_config_is_the_xm335_law():
    from isaaclab.actuators import DCMotor
    from isaaclab.utils.types import ArticulationActions

    from code_isaaclab.actuators import xm335_actuator_cfg, xm335_law

    cfg = xm335_actuator_cfg(SERVO, ["j"], mode="dc_motor")
    actuator = DCMotor(
        cfg,
        joint_names=["j"],
        joint_ids=slice(None),
        num_envs=1,
        device="cpu",
        stiffness=cfg.stiffness,
        damping=cfg.damping,
    )
    stall = SERVO["stall_torque_nm"]
    for error in torch.linspace(-0.5, 0.5, 21):
        for speed in torch.linspace(-12.0, 12.0, 25):
            action = ArticulationActions(
                joint_positions=error.reshape(1, 1), joint_velocities=torch.zeros(1, 1), joint_efforts=torch.zeros(1, 1)
            )
            out = actuator.compute(action, torch.zeros(1, 1), speed.reshape(1, 1)).joint_efforts
            # The law, capped at the stall torque (braking beyond it is not modelled).
            expected = torch.clamp(xm335_law(SERVO, error, speed), -stall, stall)
            assert float(out) == pytest.approx(float(expected), abs=1e-6)


def test_gains_follow_the_servo_block():
    from code_isaaclab.actuators import xm335_actuator_cfg, xm335_gains

    g = xm335_gains(SERVO)
    assert g["kp"] == pytest.approx(1.03 * 11.509510008905426)
    assert g["kd"] == pytest.approx(1.03 / 5.550147021341967)
    implicit = xm335_actuator_cfg(SERVO, ["j"], mode="implicit")
    assert implicit.stiffness == pytest.approx(g["kp"]) and implicit.effort_limit_sim == pytest.approx(1.03)
    with pytest.raises(ValueError):
        xm335_gains({**SERVO, "model": "XL330"})


def test_saturation_can_be_switched_off_in_implicit_mode():
    import pytest

    from code_isaaclab.actuators import UNLIMITED, xm335_actuator_cfg

    servo = {"model": "XM335-T323-T", "stall_torque_nm": 1.03, "no_load_speed_rad_s": 5.55, "duty_per_rad": 11.5,
             "armature_kg_m2": 0.0031}
    limited = xm335_actuator_cfg(servo, ["j"], mode="implicit")
    free = xm335_actuator_cfg(servo, ["j"], mode="implicit", saturate_effort=False)
    assert limited.effort_limit_sim == pytest.approx(1.03) and free.effort_limit_sim == UNLIMITED
    assert free.velocity_limit_sim == pytest.approx(5.55) and free.stiffness == limited.stiffness
    with pytest.raises(ValueError):
        xm335_actuator_cfg(servo, ["j"], mode="dc_motor", saturate_effort=False)
