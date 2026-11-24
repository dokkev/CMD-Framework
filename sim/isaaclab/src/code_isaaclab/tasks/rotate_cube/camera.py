# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Video camera for cube rotation; no Isaac Lab imports at module level, so the training launcher
(``scripts/rotate_cube/train.py``) can load it by path without starting the simulator."""

from __future__ import annotations

import math

CAMERA_EYE_ENV = (0.17, -0.10, 0.63)
CAMERA_LOOKAT_ENV = (0.04, 0.02, 0.535)
"""Video camera relative to environment 0's origin (the palm-up hand's root at z = 0.5; the cube near (0.04, 0.025, 0.54)),
from beyond the fingertips and above. The Newton GL viewer takes world coordinates: :func:`env0_camera`."""


def env0_origin(num_envs: int, spacing: float) -> tuple[float, float, float]:
    """Environment 0's origin in Isaac Lab's centred clone grid (``isaaclab.cloner.clone_plan.grid_transforms``)."""
    rows = int(math.ceil(num_envs / math.sqrt(num_envs)))
    cols = int(math.ceil(num_envs / rows))
    return ((rows - 1) / 2 * spacing, -(cols - 1) / 2 * spacing, 0.0)


def env0_camera(num_envs: int, spacing: float = 0.5) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """World eye and look-at points framing environment 0's hand (:data:`CAMERA_EYE_ENV`, :data:`CAMERA_LOOKAT_ENV`)."""
    o = env0_origin(num_envs, spacing)
    return (tuple(round(o[i] + CAMERA_EYE_ENV[i], 4) for i in range(3)),
            tuple(round(o[i] + CAMERA_LOOKAT_ENV[i], 4) for i in range(3)))


def recording_visualizer_cfg(num_envs: int = 128, spacing: float = 0.5):
    """The Newton GL visualizer settings for video clips, as cheap as possible: offscreen (``headless``: it renders only
    when the video recorder grabs a frame, never on a plain step), the first 4 worlds only, 960 x 540, no live plots or
    picking. A hint (``sim.default_visualizer_cfg``): it adds no visualizer unless one is selected
    (``--viz newton_gl``)."""
    from isaaclab_visualizers.newton.newton_visualizer_cfg import NewtonGLVisualizerCfg

    eye, lookat = env0_camera(num_envs, spacing)
    return NewtonGLVisualizerCfg(
        headless=True,
        max_visible_envs=4,
        randomly_sample_visible_envs=False,
        window_width=960,
        window_height=540,
        enable_live_plots=False,
        enable_picking=False,
        enable_markers=True,
        eye=eye,
        lookat=lookat,
    )
