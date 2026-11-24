# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""RSL-RL PPO for cube rotation, after Hora's and LEAP Hand's working settings: [512, 256, 128] ELU MLPs with
observation normalisation, initial std 1.0, no entropy bonus, 16 steps per environment, 5 epochs of 4 mini-batches,
learning rate 5e-3 adaptive at KL 0.02, value loss x2 clipped, gamma 0.99, lambda 0.95, gradient norm 1."""

import os

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class CoDeRotateCubePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 16
    max_iterations = 3000
    save_interval = 100
    experiment_name = "code_rotate_cube"
    actor = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[512, 256, 128], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=2.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.0, num_learning_epochs=5,
        num_mini_batches=4, learning_rate=5.0e-3, schedule="adaptive", gamma=0.99, lam=0.95, desired_kl=0.02,
        max_grad_norm=1.0,
    )

    def __post_init__(self):
        super().__post_init__()
        if os.environ.get("CODE_ROTATE_CUBE_HORA", "0") == "1":
            self.num_steps_per_env = 8  # Hora's horizon_length
