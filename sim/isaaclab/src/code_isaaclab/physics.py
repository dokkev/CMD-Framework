# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Physics backends, selected per run with ``physics=<name>``, separate from the task and the hand.

- ``newton_mjwarp`` (default): MuJoCo-Warp, the training backend. Same solver settings as Isaac Lab's
  Allegro reorientation task.
- ``newton_mjwarp_hydro``: the same MuJoCo-Warp dynamics, with Newton's collision pipeline and SDF-based
  hydroelastic contacts in place of MuJoCo's point contacts. Contact becomes a distributed pressure field,
  so a policy trained on point contacts can be checked against distributed, compliant contact.
- ``newton_vbd``: Newton's Vertex Block Descent solver for everything, rigid and deformable, in one solver.
  It is the verification backend. Its manager recomputes joint states from VBD's body poses after each
  step (:mod:`code_isaaclab.vbd_manager`), which Isaac Lab 3.0 does not do. A policy trained on
  MuJoCo-Warp is replayed on a different solver (sim-to-sim check). VBD also simulates deformable bodies,
  which later compliant contact interfaces will need.

A new backend is one more field here. Tasks and hands do not change.
"""

from isaaclab.utils import configclass
from isaaclab_newton.physics import (
    HydroelasticSDFCfg,
    MJWarpSolverCfg,
    NewtonCfg,
    NewtonCollisionPipelineCfg,
    NewtonShapeCfg,
    NewtonSoftContactCfg,
    VBDSolverCfg,
)

from isaaclab_tasks.utils import PresetCfg


@configclass
class HydroelasticShapeCfg(NewtonShapeCfg):
    """Default shape properties with hydroelastic contact on every collider.

    Forwarded to Newton's ``ModelBuilder.default_shape_cfg``. The USD importer falls back to these defaults
    for colliders without Newton SDF attributes, so existing assets (an instanceable hand, the cube) need no
    re-authoring. Each mesh gets an SDF of at most ``sdf_max_resolution`` cells per axis (divisible by 8);
    planes do not support hydroelastic contact, and Newton clears the flag on them.
    """

    is_hydroelastic: bool = True
    sdf_max_resolution: int | None = 64
    kh: float = 1.0e10
    """Hydroelastic stiffness [N/m^3] (Newton's default)."""


_MJWARP_SOLVER = dict(
    integrator="implicitfast", njmax=80, nconmax=70, impratio=10.0, cone="elliptic", update_data_interval=2
)


@configclass
class HandPhysicsCfg(PresetCfg):
    """Physics backend presets for the hand tasks."""

    newton_mjwarp = NewtonCfg(solver_cfg=MJWarpSolverCfg(**_MJWARP_SOLVER), num_substeps=2)
    newton_mjwarp_hydro = NewtonCfg(
        solver_cfg=MJWarpSolverCfg(**_MJWARP_SOLVER, use_mujoco_contacts=False),
        collision_cfg=NewtonCollisionPipelineCfg(sdf_hydroelastic_config=HydroelasticSDFCfg()),
        default_shape_cfg=HydroelasticShapeCfg(),
        num_substeps=2,
    )
    newton_vbd = NewtonCfg(
        # joint states recomputed from VBD's body poses after each step (Isaac Lab 3.0 does not)
        # 30 iterations x 4 substeps: zero-action joint tracking 0.057 rad (10 x 2: 0.176; MJWarp: 0.029)
        solver_cfg=VBDSolverCfg(iterations=30, class_type="code_isaaclab.vbd_manager:NewtonVBDJointSyncManager"),
        num_substeps=4,
    )
    default = newton_mjwarp


def apply_hand_buffers(physics: HandPhysicsCfg, buffers: dict[str, int]) -> None:
    """Size the MuJoCo-Warp buffers for a hand (its joint and contact counts set them)."""
    for preset in (physics.newton_mjwarp, physics.newton_mjwarp_hydro, physics.default):  # ``default`` may be a copy
        if isinstance(preset.solver_cfg, MJWarpSolverCfg):
            for name, value in buffers.items():
                setattr(preset.solver_cfg, name, value)


def coupled_vbd_cfg(
    rigid_bodies: list[str],
    proxied_rigid_bodies: list[str],
    static_shapes_in_vbd: bool = True,
    soft_contact_ke: float = 8.0e3,
    soft_contact_mu: float = 1.0,
    vbd_iterations: int = 10,
    num_substeps: int = 2,
    coupling_iterations: int = 1,
    **proxy_options,
) -> NewtonCfg:
    """MuJoCo-Warp for the rigid bodies, VBD for the deformable ones, coupled by Newton's proxy coupler.

    The rigid entry (the hand articulation, the object) keeps the training solver's dynamics and joint
    drives. The deformable entry owns every particle, i.e. every deformable body (the compliant contact
    pads), and runs on VBD; with ``static_shapes_in_vbd`` it also sees static colliders such as the ground.
    ``proxied_rigid_bodies`` are rigid bodies exposed inside VBD as proxies, so the pads can touch them;
    contact impulses go back to MuJoCo-Warp
    one coupled step later. Status (``scripts/contact_experiments.py``, 2026-11-08): the coupled step runs,
    but a free, light rigid body is ejected or interpenetrates the pad, so this is not a task preset yet.
    Body selectors are prim-path regexes, so this is scene-specific and built per
    task rather than a fixed preset. The proxy contact and soft-contact settings follow Isaac Lab's
    ``franka_soft`` coupled task (full-surface rigid-soft contact, collision every step).
    ``proxy_options`` go to :class:`CouplerProxyMappingCfg` (``mode``, ``proxy_relaxation``,
    ``proxy_relaxation_mode``, ``mass_scale``, ...).

    Uses ``isaaclab_contrib.coupling``, which wraps Newton's experimental coupled solvers. Newton contact
    sensors are not supported under a coupled solver.
    """
    from isaaclab_contrib.coupling import CouplerEntryCfg, CouplerProxyCfg, CouplerProxyMappingCfg

    return NewtonCfg(
        solver_cfg=CouplerProxyCfg(
            entries=[
                CouplerEntryCfg(name="rigid", solver_cfg=MJWarpSolverCfg(**_MJWARP_SOLVER), bodies=rigid_bodies),
                CouplerEntryCfg(
                    name="deformable",
                    solver_cfg=VBDSolverCfg(iterations=vbd_iterations, rigid_body_particle_contact_buffer_size=256),
                    all_particles=True,
                    include_static_shapes=static_shapes_in_vbd,
                ),
            ],
            proxies=[
                CouplerProxyMappingCfg(
                    source="rigid",
                    destination="deformable",
                    bodies=proxied_rigid_bodies,
                    collide_interval=1,
                    collision_pipeline=NewtonCollisionPipelineCfg(enable_rigid_soft_full_surface_contact=True),
                    **proxy_options,
                )
            ],
            iterations=coupling_iterations,
        ),
        soft_contact_cfg=NewtonSoftContactCfg(
            soft_contact_ke=soft_contact_ke, soft_contact_kd=1.0e-2, soft_contact_mu=soft_contact_mu
        ),
        num_substeps=num_substeps,
    )


__all__ = ["HandPhysicsCfg", "HydroelasticShapeCfg", "apply_hand_buffers", "coupled_vbd_cfg"]
