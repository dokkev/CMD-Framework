# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""RSL-RL runner config for in-hand cylinder rotation: upstream's Allegro manager-based PPO settings with a shorter
discount horizon (the rotation reward is dense; gamma 0.99 instead of 0.998)."""

from isaaclab.utils import configclass

from isaaclab_tasks.core.reorient.config.allegro_hand.agents.rsl_rl_ppo_cfg import AllegroHandManagerPPORunnerCfg


@configclass
class CoDeRotatePPORunnerCfg(AllegroHandManagerPPORunnerCfg):
    experiment_name = "code_rotate_cylinder_o"
    max_iterations = 500
    save_interval = 25

    def __post_init__(self):
        super().__post_init__()
        self.algorithm.gamma = 0.99
