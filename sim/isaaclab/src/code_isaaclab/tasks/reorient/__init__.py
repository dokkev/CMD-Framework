# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""In-hand cube reorientation tasks, one per registered hand.

Train on MuJoCo-Warp (the default) and replay the policy on VBD for verification:

    uv run --extra rsl-rl isaaclab train --rl_library rsl_rl --task CoDe-Reorient-Cube-Allegro physics=newton_mjwarp
    uv run --extra rsl-rl isaaclab play  --rl_library rsl_rl --task CoDe-Reorient-Cube-Allegro physics=newton_vbd
"""

import gymnasium as gym

gym.register(
    id="CoDe-Reorient-Cube-Allegro",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:AllegroReorientEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents:AllegroReorientPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

# The CoDe hand, one task per contact interface (code_env_cfg.py). Same MDP, robot and PPO settings; only the
# pads' contact surface differs. ``-Main``: Study 3A's main experiment (non-overlapping goals, smoothed actions).
for _variant, _cfg in (
    ("CoDeFlat", "CoDeFlatReorientEnvCfg"),
    ("CoDeCrowned", "CoDeCrownedReorientEnvCfg"),
    ("CoDeFlat-Main", "CoDeFlatReorientMainEnvCfg"),
    ("CoDeCrowned-Main", "CoDeCrownedReorientMainEnvCfg"),
):
    gym.register(
        id=f"CoDe-Reorient-Cube-{_variant}",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.code_env_cfg:{_cfg}",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeReorientPPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )

# Hand O's fingers with a thumb pad, transfer-ready conventions (o_env_cfg.py); R and U are the same task on the
# redesigned fingers (for zero-shot transfer of an O policy).
for _hand in ("O", "R", "U"):
    gym.register(
        id=f"CoDe-Reorient-Cube-{_hand}",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.o_env_cfg:CoDeReorientCube{_hand}EnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeReorientOPPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )

# Continuous rotation about +z (o_env_cfg.CoDeReorientCubeSpinEnvCfg): goals step on by 30 deg, success at 0.1 rad,
# smoothness penalties.
for _hand in ("O", "R", "U"):
    gym.register(
        id=f"CoDe-Reorient-Cube-{_hand}-Spin",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.o_env_cfg:CoDeReorientCube{_hand}SpinEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeReorientOPPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )

# Goals within the hand's reachable set (o_env_cfg.CoDeReorientCubeReachEnvCfg): +-30 deg about z, 40 deg apart, success
# at 0.1 rad, smoothness and fall penalties.
for _hand in ("O", "R", "U"):
    gym.register(
        id=f"CoDe-Reorient-Cube-{_hand}-Reach",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.o_env_cfg:CoDeReorientCube{_hand}ReachEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeReorientOPPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )
