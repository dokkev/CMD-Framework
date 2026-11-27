# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Cube rotation by hand O with OmniReset's recipe (:mod:`.env_cfg`, ``docs/experimental/omnireset_recipe.md``).

    uv run python scripts/rotate_cube/gen_omnireset_resets.py --type all          # the four reset datasets
    uv run python scripts/rotate_cube/train.py --task CoDe-Rotate-Cube-O-OmniReset --experiment \\
        code_rotate_cube_omnireset --iterations 500 --num_envs 4096 --seed 20
    uv run --extra rsl-rl python scripts/rotate_cube/eval_policy.py --task CoDe-Rotate-Cube-O-OmniReset-Eval ...
"""

import gymnasium as gym

for _suffix, _cfg, _agent in (
    ("", "CoDeRotateCubeOOmniResetEnvCfg", "CoDeRotateCubeOmniResetPPORunnerCfg"),
    ("-Eval", "CoDeRotateCubeOOmniResetEvalEnvCfg", "CoDeRotateCubeOmniResetPPORunnerCfg"),
    ("V2", "CoDeRotateCubeOOmniResetV2EnvCfg", "CoDeRotateCubeOmniResetV2PPORunnerCfg"),
    ("V2-Eval", "CoDeRotateCubeOOmniResetV2EvalEnvCfg", "CoDeRotateCubeOmniResetV2PPORunnerCfg"),
):
    gym.register(
        id=f"CoDe-Rotate-Cube-O-OmniReset{_suffix}",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:{_cfg}",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:{_agent}",
            "default_agent": "rsl_rl",
        },
    )

for _key in ("R", "U", "R5", "U5", "R9", "U9", "RL", "RL60", "UL120", "UL", "MR", "MRM", "MRbz", "MRMbz", "MR5", "MRM5", "MR9", "MRM9", "MRS"):  # zero-shot transfer of the O-trained v2 policy
    gym.register(
        id=f"CoDe-Rotate-Cube-{_key}-OmniResetV2-Eval",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCube{_key}OmniResetV2EvalEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCubeOmniResetV2PPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )

for _key in ("R", "U", "R5", "U5", "R9", "U9"):  # attempt 6: fine-tuning the O-trained v2 policy on each hand
    gym.register(
        id=f"CoDe-Rotate-Cube-{_key}-OmniResetV2",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCube{_key}OmniResetV2EnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCubeOmniResetV2PPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )

for _key in ("O", "R", "U", "R5", "U5", "R9", "U9", "RL", "RL60", "UL120", "UL"):  # the same, through the contact-referenced joint maps
    gym.register(
        id=f"CoDe-Rotate-Cube-{_key}-OmniResetV2-Eval-Mapped",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCube{_key}OmniResetV2EvalMappedEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCubeOmniResetV2PPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )

# the fingertip-region cylinder task (docs/experimental/rl_cylinder.md): training on O, evaluation on every hand
gym.register(
    id="CoDe-Rotate-Cylinder-O-Fingertip",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCylinderOFingertipEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCylinderFingertipPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
for _key in ("O", "R", "U", "R5", "U5", "R9", "U9", "RL", "RL60", "UL120", "UL"):
    for _suffix in ("", "Mapped"):
        gym.register(
            id=f"CoDe-Rotate-Cylinder-{_key}-Fingertip-Eval" + (f"-{_suffix}" if _suffix else ""),
            entry_point="isaaclab.envs:ManagerBasedRLEnv",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCylinder{_key}FingertipEval{_suffix}EnvCfg",
                "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCylinderFingertipPPORunnerCfg",
                "default_agent": "rsl_rl",
            },
        )

# attempt 4: the object-blind actor (training on O, evaluation on every hand)
gym.register(
    id="CoDe-Rotate-Cube-O-OmniResetV2Blind",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCubeOOmniResetV2BlindEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCubeOmniResetV2BlindPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
for _key in ("O", "R", "U", "R5", "U5", "R9", "U9", "RL", "RL60", "UL120", "UL"):
    gym.register(
        id=f"CoDe-Rotate-Cube-{_key}-OmniResetV2Blind-Eval",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCube{_key}OmniResetV2BlindEvalEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCubeOmniResetV2BlindPPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )

# attempt 4b: the actor's cube pose through a delayed, noisy sensor (training on O, evaluation on every hand)
for _v in ("Noisy", "NoisyHalf"):
    gym.register(
        id=f"CoDe-Rotate-Cube-O-OmniResetV2{_v}",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCubeOOmniResetV2{_v}EnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCubeOmniResetV2PPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )
    for _key in ("O", "R", "U", "R5", "U5", "R9", "U9", "RL", "RL60", "UL120", "UL"):
        gym.register(
            id=f"CoDe-Rotate-Cube-{_key}-OmniResetV2{_v}-Eval",
            entry_point="isaaclab.envs:ManagerBasedRLEnv",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCube{_key}OmniResetV2{_v}EvalEnvCfg",
                "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCubeOmniResetV2PPORunnerCfg",
                "default_agent": "rsl_rl",
            },
        )

# the cylinder task with a blind actor that reads per-pad contact (docs/experimental/rl_cylinder.md)
gym.register(
    id="CoDe-Rotate-Cylinder-O-FingertipTouch",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCylinderOFingertipTouchEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCylinderFingertipPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
for _key in ("O", "R", "U", "R5", "U5", "R9", "U9", "RL", "RL60", "UL120", "UL"):
    gym.register(
        id=f"CoDe-Rotate-Cylinder-{_key}-FingertipTouch-Eval",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{__name__}.env_cfg:CoDeRotateCylinder{_key}FingertipTouchEvalEnvCfg",
            "rsl_rl_cfg_entry_point": f"{__name__}.agents:CoDeRotateCylinderFingertipPPORunnerCfg",
            "default_agent": "rsl_rl",
        },
    )
