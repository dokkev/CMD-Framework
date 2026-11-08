# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""RSL-RL runner configs: upstream's Allegro manager-based PPO settings, under project experiment names."""

from isaaclab.utils import configclass

from isaaclab_tasks.core.reorient.config.allegro_hand.agents.rsl_rl_ppo_cfg import AllegroHandManagerPPORunnerCfg


@configclass
class AllegroReorientPPORunnerCfg(AllegroHandManagerPPORunnerCfg):
    experiment_name = "code_reorient_allegro"
