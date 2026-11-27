# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Finger walk (attempt 7b', ``docs/experimental/rl_pip_only.md``): three PIPs roll a cylinder along the proximal pads
to alternating targets, MCP locked."""

import gymnasium as gym

for _v in ("", "Touch", "TouchDir", "ObsS", "ObsL"):
    gym.register(
        id="CoDe-Finger-Walk-O" + (f"-{_v}" if _v else ""),
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:FingerWalkO{_v}EnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:FingerWalkPPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )
    for _key in ("O", "R", "U", "R5", "U5", "R9", "U9"):
        gym.register(
            id=f"CoDe-Finger-Walk-{_key}" + (f"-{_v}" if _v else "") + "-Eval",
            entry_point="isaaclab.envs:ManagerBasedRLEnv",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": f"{__name__}.env_cfg:FingerWalk{_key}{_v}EvalEnvCfg",
                "rsl_rl_cfg_entry_point": f"{__name__}.agents:FingerWalkPPORunnerCfg",
                "default_agent": "rsl_rl",
            },
        )

from .grid import GRID  # noqa: E402

for _key in GRID:  # attempt 9: the Study 3 hand grid (closed loop)
    for _sfx in ("", "-Eval"):
        gym.register(
            id=f"CoDe-Finger-Walk-{_key}{_sfx}",
            entry_point="isaaclab.envs:ManagerBasedRLEnv",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": f"{__name__}.env_cfg:FingerWalk{_key}{_sfx.lstrip('-')}EnvCfg",
                "rsl_rl_cfg_entry_point": f"{__name__}.agents:FingerWalkPPORunnerCfg",
                "default_agent": "rsl_rl",
            },
        )
