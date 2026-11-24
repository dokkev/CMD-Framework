# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""RSL-RL PPO for cube rotation, after Hora's and LEAP Hand's working settings: [512, 256, 128] ELU MLPs with
observation normalisation, initial std 1.0, no entropy bonus, 5 epochs of 4 mini-batches, value loss x2 clipped,
lambda 0.95, gradient norm 1.

The defaults are s15's fine-tuning settings (learning rate 2.5e-5 fixed, gamma 0.997, 32 steps per environment, a
checkpoint every 40 iterations), for continuing from its checkpoint. Earlier stages of its lineage trained from scratch
with Hora's: ``agent.algorithm.learning_rate=0.005 agent.algorithm.schedule=adaptive agent.algorithm.gamma=0.99
agent.num_steps_per_env=16`` (KL target 0.02)."""

import os

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg, RslRlRNNModelCfg


@configclass
class CoDeRotateCubePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 32
    max_iterations = 3000
    save_interval = 40
    experiment_name = "code_rotate_cube"
    actor = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )  # fmt: skip
    critic = RslRlMLPModelCfg(hidden_dims=[512, 256, 128], activation="elu", obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=2.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.0, num_learning_epochs=5,
        num_mini_batches=4, learning_rate=2.5e-5, schedule="fixed", gamma=0.997, lam=0.95, desired_kl=0.02,
        max_grad_norm=1.0,
    )  # fmt: skip

    def __post_init__(self):
        super().__post_init__()
        if os.environ.get("CODE_ROTATE_CUBE_HORA", "0") == "1":
            self.num_steps_per_env = 8  # Hora's horizon_length
        if os.environ.get("CODE_ROTATE_CUBE_GRU", "0") == "1":
            # LEAP Hand Isaac Lab's recurrent actor-critic: a 256-unit GRU before the same MLP heads, trained from
            # scratch at learning rate 5e-4, adaptive (KL 0.02). rsl_rl's RNNModel feeds the GRU output alone to the
            # MLP (LEAP also concatenates the observation and layer-normalises).
            gru = dict(rnn_type="gru", rnn_hidden_dim=256, rnn_num_layers=1, hidden_dims=[512, 256, 128],
                       activation="elu", obs_normalization=True)
            gaussian = RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0)
            self.actor = RslRlRNNModelCfg(**gru, distribution_cfg=gaussian)
            self.critic = RslRlRNNModelCfg(**gru)
            self.algorithm.learning_rate = 5.0e-4
            self.algorithm.schedule = "adaptive"
            self.algorithm.desired_kl = 0.02
            self.save_interval = 100
