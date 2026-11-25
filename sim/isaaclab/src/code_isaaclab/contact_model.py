# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The hand-cube contact model ``pad13_condim4_v1`` for MuJoCo-Warp: values set on Newton's model builder before it is
finalized, so they reach the solver at conversion (``condim`` cannot change afterwards).

Physically motivated (``docs/experimental/omnireset_recipe.md`` section 4 records the probe that compared it with the
default model on held-out grasps), not tuned for reward:

- friction: pads 1.3 (Study 3's pad), the cube and every other hand collider 0.5 (printed plastic, aluminium);
  MuJoCo takes the larger of two shapes' coefficients, so the pads alone set the grip;
- ``condim`` 4 on the pads and the cube, torsional friction 0.003 m on both: a uniform-pressure disc patch of radius
  a resists twist up to (2/3) mu N a; a ~3 mm patch at mu 1.3 gives 0.0026 m. A domed pad touches at one point, so
  without it a pad resists no twist;
- ``impratio`` 1000 (elliptic cone), Study 3's fix for creep at 10 (set in the solver config, :func:`apply`);
- ``solimp`` (0.95, 0.99, 0.001, 0.5, 2) and ``solref`` time constant 0.012 s, damping ratio 1 (Newton's
  ``convert_solref``: ``kd = 2 / tc``, ``ke = (kd / 2)^2``) on the hand and the cube: about 6.5 kN/m effective, 1 mm of
  pad compression at 5 N grasp forces (the default model gives ~1.1 kN/m and 3.6 mm p90 penetration).

Isaac Lab and Newton are unchanged: :class:`NewtonMJWarpContactManager` is selected through ``MJWarpSolverCfg``'s
``class_type`` and uses the manager's ``_prepare_builder_for_finalize`` hook.
"""

from __future__ import annotations

import logging

from isaaclab_newton.physics.mjwarp_manager import NewtonMJWarpManager

logger = logging.getLogger(__name__)

NAME = "pad13_condim4_v1"
PAD_MU, CUBE_MU, HAND_MU = 1.3, 0.5, 0.5
TORSIONAL_M = 0.003
CONDIM = 4
IMPRATIO = 1000.0
SOLIMP = (0.95, 0.99, 0.001, 0.5, 2.0)
SOLREF_TIMECONST_S = 0.012
KD = 2.0 / SOLREF_TIMECONST_S
KE = (KD / 2.0) ** 2
NJMAX_SCALE = 1.6
"""condim 4 adds a constraint row per contact; the probe ran with 1.6 x the hand's ``njmax``."""


def describe() -> dict:
    """The model's values, for manifests and dataset metadata."""
    return {
        "name": NAME, "pad_mu": PAD_MU, "cube_mu": CUBE_MU, "hand_mu": HAND_MU, "torsional_m": TORSIONAL_M,
        "condim_pad_cube": CONDIM, "cone": "elliptic", "impratio": IMPRATIO, "solimp": list(SOLIMP),
        "solref_timeconst_s": SOLREF_TIMECONST_S, "solref_dampratio": 1.0, "shape_ke": KE, "shape_kd": KD,
    }  # fmt: skip


def shape_kind(label: str) -> str:
    if "/object" in label:
        return "cube"
    if label.endswith("_pad"):
        return "pad"
    if "/Robot/" in label:
        return "hand"
    return "other"


class NewtonMJWarpContactManager(NewtonMJWarpManager):
    """:class:`NewtonMJWarpManager` that applies the contact model to the builder's shapes before finalizing."""

    @classmethod
    def _prepare_builder_for_finalize(cls, builder) -> None:
        super()._prepare_builder_for_finalize(builder)
        from newton._src.solvers.mujoco.solver_mujoco import vec5

        condim = builder.custom_attributes["mujoco:condim"]
        solimp = builder.custom_attributes["mujoco:geom_solimp"]
        for attr in (condim, solimp):
            if attr.values is None:
                attr.values = {}
        counts = {"pad": 0, "cube": 0, "hand": 0}
        for i, label in enumerate(builder.shape_label):
            kind = shape_kind(label)
            if kind == "other":
                continue
            counts[kind] += 1
            builder.shape_material_mu[i] = {"pad": PAD_MU, "cube": CUBE_MU, "hand": HAND_MU}[kind]
            builder.shape_material_ke[i] = KE
            builder.shape_material_kd[i] = KD
            solimp.values[i] = vec5(*SOLIMP)
            if kind in ("pad", "cube"):
                builder.shape_material_mu_torsional[i] = TORSIONAL_M
                condim.values[i] = CONDIM
        if not counts["pad"] or not counts["cube"]:
            raise RuntimeError(f"contact model {NAME}: no pad or cube shapes found ({counts})")
        logger.info(f"contact model {NAME} applied to shapes {counts}")


def apply(physics) -> None:
    """Select the model on a ``HandPhysicsCfg``'s MuJoCo-Warp preset (after ``apply_hand_buffers``)."""
    from isaaclab_newton.physics import NewtonCfg

    old = physics.newton_mjwarp
    solver = old.solver_cfg.replace(
        class_type=f"{__name__}:NewtonMJWarpContactManager", impratio=IMPRATIO, cone="elliptic",
        njmax=int(old.solver_cfg.njmax * NJMAX_SCALE),
    )  # fmt: skip
    preset = NewtonCfg(solver_cfg=solver, num_substeps=old.num_substeps)
    physics.newton_mjwarp = preset
    physics.default = preset


__all__ = ["NAME", "NewtonMJWarpContactManager", "apply", "describe"]
