# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""OmniReset's PPO (``omnireset/config/ur5e_robotiq_2f85/agents/rsl_rl_cfg.py`` ``Base_PPORunnerCfg``, UWLab
2b40afb), unchanged: 32 steps per environment, [512, 256, 128, 64] ELU actor and critic with observation normalisation,
a heteroscedastic Gaussian (state-dependent log std, initial std 1.0, clamped to [0.001, 2.0]), value loss x1 clipped,
clip 0.2, entropy 0.006, 5 epochs of 4 mini-batches, learning rate 1e-4 adaptive (KL 0.01), gamma 0.99, lambda 0.95,
gradient norm 1, a checkpoint every 100 iterations; gamma 0.997 instead of 0.99, their 10 s horizon (1 / (1 - gamma)
steps at 10 Hz) at our 30 Hz control. Their actor and critic both read the policy group; with
``CODE_OMNIRESET_ASYM_CRITIC=1`` the critic reads the privileged group instead."""

import os

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class CoDeRotateCubeOmniResetPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32
    max_iterations = 4000
    save_interval = 100
    experiment_name = "code_rotate_cube_omnireset"
    obs_groups = {"actor": ["policy"], "critic": ["policy"]}
    actor = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128, 64], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.HeteroscedasticGaussianDistributionCfg(
            init_std=1.0, std_type="log", std_range=(0.001, 2.0)),
    )  # fmt: skip
    critic = RslRlMLPModelCfg(hidden_dims=[512, 256, 128, 64], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, normalize_advantage_per_mini_batch=False, clip_param=0.2,
        entropy_coef=0.006, num_learning_epochs=5, num_mini_batches=4, learning_rate=1.0e-4, schedule="adaptive",
        gamma=0.997, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    )  # fmt: skip

    def __post_init__(self):
        super().__post_init__()
        if os.environ.get("CODE_OMNIRESET_ASYM_CRITIC", "0") == "1":
            self.obs_groups = {"actor": ["policy"], "critic": ["critic"]}
