# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""PPO for the finger walk: v2c's settings (entropy 0.001, std capped at 1.0, asymmetric critic), smaller nets for
a 3-dimensional action, a checkpoint every 50 iterations."""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


@configclass
class FingerWalkPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32
    max_iterations = 1000
    save_interval = 50
    experiment_name = "code_finger_walk"
    obs_groups = {"actor": ["policy"], "critic": ["critic"]}
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.HeteroscedasticGaussianDistributionCfg(
            init_std=1.0, std_type="log", std_range=(0.001, 1.0)),
    )  # fmt: skip
    critic = RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, normalize_advantage_per_mini_batch=False, clip_param=0.2,
        entropy_coef=0.001, num_learning_epochs=5, num_mini_batches=4, learning_rate=3.0e-4, schedule="adaptive",
        gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    )  # fmt: skip
