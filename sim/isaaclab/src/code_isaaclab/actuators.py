# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Servo models as Isaac Lab actuator configs, built from the ``servo`` block of a design export.

The ROBOTIS XM335 in Position Control Mode, as ``design/core/hardware/xm335.py`` models it::

    tau = tau_stall * (clip(K * e, -1, 1) - qd / w_noload)

(duty from a proportional position loop, then a coreless DC motor's torque-speed line). That law is exactly a PD
law with ``kp = tau_stall * K`` and ``kd = tau_stall / w_noload`` (the back-EMF damping), clipped to the DC motor's
four-quadrant envelope ``tau_stall * (+-1 - qd / w_noload)``. Below saturation the PD term already lies inside the
envelope; above it the envelope is the law. So:

- ``"dc_motor"``: :class:`DCMotorCfg` with those values reproduces the law. Isaac Lab authors it as a Newton-native
  actuator (``SimulationCfg.use_newton_actuators``), so every Newton solver receives the same explicit torque.
- ``"implicit"``: :class:`ImplicitActuatorCfg` with the same ``kp``/``kd``, a constant effort limit and the no-load
  speed as velocity limit. It is the law's small-error linearisation: the solver integrates the PD implicitly
  (stable at any armature), but the torque limit does not fall with speed, so large moves are too fast.

Nothing here imports ``design/core``: the values come from the export (``code_hand.json`` ``servo``).
"""

from __future__ import annotations

from typing import Literal

from isaaclab.actuators import ActuatorBaseCfg, DCMotorCfg, ImplicitActuatorCfg

ServoMode = Literal["dc_motor", "implicit"]


def xm335_gains(servo: dict) -> dict[str, float]:
    """``kp`` [N m/rad], ``kd`` [N m s/rad], ``stall`` [N m], ``no_load`` [rad/s], ``armature`` [kg m^2]."""
    if servo.get("model") != "XM335-T323-T":
        raise ValueError(f"not an XM335-T323-T servo block: {servo.get('model')!r}")
    stall, no_load = float(servo["stall_torque_nm"]), float(servo["no_load_speed_rad_s"])
    if float(servo.get("duty_limit", 1.0)) != 1.0:
        raise ValueError("a PWM limit below 100 % is not representable as a DC-motor clamp; extend the model")
    return {
        "kp": stall * float(servo["duty_per_rad"]),
        "kd": stall / no_load,
        "stall": stall,
        "no_load": no_load,
        "armature": float(servo["armature_kg_m2"]),
    }


def xm335_actuator_cfg(
    servo: dict, joint_names: list[str], mode: ServoMode = "dc_motor", armature: float | None = None
) -> ActuatorBaseCfg:
    """The actuator config of a group of XM335-driven joints. ``armature`` overrides the export's placeholder."""
    g = xm335_gains(servo)
    j = g["armature"] if armature is None else armature
    if mode == "dc_motor":
        return DCMotorCfg(
            joint_names_expr=list(joint_names),
            stiffness=g["kp"],
            damping=g["kd"],
            saturation_effort=g["stall"],
            actuator_velocity_limit=g["no_load"],
            actuator_effort_limit=g["stall"],
            armature=j,
        )
    if mode == "implicit":
        return ImplicitActuatorCfg(
            joint_names_expr=list(joint_names),
            stiffness=g["kp"],
            damping=g["kd"],
            effort_limit_sim=g["stall"],
            velocity_limit_sim=g["no_load"],
            armature=j,
        )
    raise ValueError(f"unknown servo mode {mode!r}")


def xm335_law(servo: dict, error_rad, velocity_rad_s):
    """The reference torque [N m] of the law above (scalars or tensors), for checks."""
    import torch

    g = xm335_gains(servo)
    duty = torch.clamp(torch.as_tensor(error_rad) * float(servo["duty_per_rad"]), -1.0, 1.0)
    return g["stall"] * (duty - torch.as_tensor(velocity_rad_s) / g["no_load"])
