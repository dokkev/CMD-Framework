# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""In-hand rotation of a cube about world z by the CoDe hand, after Hora and LEAP Hand (:mod:`.env_cfg`); hands O, R, U.

    uv run python scripts/rotate_cube/gen_grasps.py --hand O        # the grasp cache, first
    uv run python scripts/reorient_o/train.py --task CoDe-Rotate-Cube-O --experiment code_rotate_cube ...
"""

import gymnasium as gym

for _hand in ("O", "R", "U"):
    gym.register(
        id=f"CoDe-Rotate-Cube-{_hand}",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCube{_hand}EnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCubePPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )
