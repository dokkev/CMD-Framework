# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""In-hand cylinder rotation (Hora-style, continuous spin about the cylinder's own axis) on the CoDe hand with hand
O's fingers (:mod:`.env_cfg`). ``-Smooth`` (O, R, U): smoother actions, penalties on joint
speed, cylinder speed and wobble, and hand-independent observations (pad-face frame) for zero-shot transfer from O.

    uv run --extra rsl-rl isaaclab train --rl_library rsl_rl --task CoDe-Rotate-Cylinder-O physics=newton_mjwarp
    uv run --extra rsl-rl isaaclab play  --rl_library rsl_rl --task CoDe-Rotate-Cylinder-O physics=newton_mjwarp
"""

import gymnasium as gym

for _id, _cfg in (("O", "CoDeRotateCylinderOEnvCfg"), ("O-Smooth", "CoDeRotateCylinderOSmoothEnvCfg"),
                  ("R-Smooth", "CoDeRotateCylinderRSmoothEnvCfg"), ("U-Smooth", "CoDeRotateCylinderUSmoothEnvCfg")):
    gym.register(
        id=f"CoDe-Rotate-Cylinder-{_id}",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:{_cfg}",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotatePPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )
