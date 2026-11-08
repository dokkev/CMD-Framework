# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""VBD manager that keeps joint coordinates in sync with VBD's body states.

VBD integrates rigid bodies in maximal coordinates (body poses). Isaac Lab 3.0's Newton integration reads
articulation joint positions and velocities from ``state.joint_q``/``joint_qd``. It does not recompute
them after a VBD step, so observations and joint-space actions see the reset values forever.

A pure-Newton probe shows that SolverVBD tracks a revolute drive target, and that ``joint_q`` is right only
after ``newton.eval_ik``. This manager runs ``eval_ik`` after every physics step. Selected through
``VBDSolverCfg.class_type``; Isaac Lab itself is unchanged.
"""

from __future__ import annotations

import newton

from isaaclab_newton.physics.vbd_manager import NewtonVBDManager


class NewtonVBDJointSyncManager(NewtonVBDManager):
    """:class:`NewtonVBDManager` plus inverse kinematics after each step, so joint states are current."""

    @classmethod
    def _simulate_physics_only(cls) -> None:
        super()._simulate_physics_only()
        model, state = cls.backend.model, cls.backend.state_0
        if model.joint_count > 0:
            newton.eval_ik(model, state, state.joint_q, state.joint_qd)
