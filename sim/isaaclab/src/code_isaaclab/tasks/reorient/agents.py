# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""RSL-RL runner configs: upstream's Allegro manager-based PPO settings, under project experiment names."""

from isaaclab.utils import configclass

from isaaclab_tasks.core.reorient.config.allegro_hand.agents.rsl_rl_ppo_cfg import AllegroHandManagerPPORunnerCfg


@configclass
class AllegroReorientPPORunnerCfg(AllegroHandManagerPPORunnerCfg):
    experiment_name = "code_reorient_allegro"


@configclass
class CoDeReorientPPORunnerCfg(AllegroHandManagerPPORunnerCfg):
    """The Study 3 pilot's runner: upstream's Allegro manager-based PPO settings, unchanged, for every contact
    interface (only the experiment name and the checkpoint interval are the project's; the iteration budget is
    set per run by the training script)."""

    experiment_name = "study3_code_reorient"
    save_interval = 25
