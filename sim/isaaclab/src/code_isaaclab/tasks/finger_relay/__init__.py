# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Finger-relay dial (attempt 7a', ``docs/experimental/rl_pip_only.md``): index and middle PIP only, MCP locked."""

import gymnasium as gym

for _v in ("", "Touch"):
    gym.register(
        id="CoDe-Finger-Relay-O" + (f"-{_v}" if _v else ""),
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:FingerRelayO{_v}EnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:FingerRelayPPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )
    for _key in ("O", "R", "U", "R5", "U5", "R9", "U9"):
        gym.register(
            id=f"CoDe-Finger-Relay-{_key}" + (f"-{_v}" if _v else "") + "-Eval",
            entry_point="isaaclab.envs:ManagerBasedRLEnv",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": f"{__name__}.env_cfg:FingerRelay{_key}{_v}EvalEnvCfg",
                "rsl_rl_cfg_entry_point": f"{__name__}.agents:FingerRelayPPORunnerCfg",
                "default_agent": "rsl_rl",
            },
        )
