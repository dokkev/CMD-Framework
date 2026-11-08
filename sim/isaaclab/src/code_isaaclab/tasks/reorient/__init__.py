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
