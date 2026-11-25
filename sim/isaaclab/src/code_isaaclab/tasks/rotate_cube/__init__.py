# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""In-hand rotation of a cube about world z by the CoDe hand, after Hora and LEAP Hand (:mod:`.env_cfg`); hands O, R, U.
The defaults are the s15 policy's settings (``report/study4/in-hand.md``).

    uv run python scripts/rotate_cube/gen_grasps.py --hand O --out_dir <dir>   # a grasp cache, if none exists
    uv run python scripts/rotate_cube/train.py --iterations 200 --num_envs 1024 --checkpoint <model.pt>
"""

import gymnasium as gym

for _hand in ("O", "R", "U", "R5", "U5", "R9", "U9", "RL", "RL60", "UL120", "UL"):
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
