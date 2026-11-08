# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Minimal contact-model experiments: a rigid block dropped onto a contact pad.

    uv run python scripts/contact_experiments.py point         # rigid pad, MuJoCo-Warp point contact
    uv run python scripts/contact_experiments.py pipeline      # control: Newton collision pipeline, no hydroelastic
    uv run python scripts/contact_experiments.py hydro         # rigid pad, MuJoCo-Warp + hydroelastic contact
    uv run python scripts/contact_experiments.py vbd_coupled   # VBD deformable pad; block on MuJoCo-Warp, coupled

Run one experiment per process (Newton keeps one solver per process). Each prints a JSON line:
- the block's final height and its lowest height;
- the pad's compression (deformable pad only);
- whether every state stayed finite.
Primitive shapes only, so hydroelastic needs no SDF authoring.
"""

from __future__ import annotations

import argparse
import json

import torch
import warp as wp

BLOCK = 0.04  # block edge [m]
PAD = (0.08, 0.08, 0.02)  # pad size [m]
DROP = 0.06  # block bottom above the pad top at start [m]
STEPS = 240  # at 1/240 s: 1 s


def scene_cfg(deformable_pad: bool):
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.utils import configclass

    pad_center = PAD[2] / 2
    if deformable_pad:
        from isaaclab_newton.sim.schemas import NewtonDeformableBodyPropertiesCfg
        from isaaclab_newton.sim.spawners.materials import NewtonDeformableBodyMaterialCfg

        pad = DeformableObjectCfg(
            prim_path="{ENV_REGEX_NS}/Pad",
            spawn=sim_utils.MeshCuboidCfg(
                size=PAD,
                edge_refinement=8.0,
                deformable_props=NewtonDeformableBodyPropertiesCfg(),
                physics_material=NewtonDeformableBodyMaterialCfg(
                    density=1000.0, k_mu=2e4, k_lambda=2e4, particle_radius=0.0025
                ),
            ),
            init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.0, 0.0, pad_center)),
        )
    else:
        pad = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Pad",
            spawn=sim_utils.CuboidCfg(
                size=PAD,
                rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
                collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, pad_center)),
        )

    @configclass
    class Scene(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
        block = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Block",
            spawn=sim_utils.CuboidCfg(
                size=(BLOCK, BLOCK, BLOCK),
                rigid_props=sim_utils.UsdPhysicsRigidBodyCfg(),
                mass_props=sim_utils.MassCfg(mass=0.1),
                collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, PAD[2] + DROP + BLOCK / 2)),
        )

    Scene.pad = pad
    Scene.__annotations__["pad"] = type(pad)
    return Scene(num_envs=1, env_spacing=1.0)


def physics_cfg(name: str, coupling: dict | None = None):
    from code_isaaclab.physics import HandPhysicsCfg, coupled_vbd_cfg

    presets = HandPhysicsCfg()
    if name == "point":
        return presets.newton_mjwarp
    if name == "pipeline":  # control: separates the collision pipeline from the hydroelastic model
        from isaaclab_newton.physics import NewtonCfg, NewtonCollisionPipelineCfg

        solver = presets.newton_mjwarp_hydro.solver_cfg.copy()
        return NewtonCfg(solver_cfg=solver, collision_cfg=NewtonCollisionPipelineCfg(), num_substeps=2)
    if name == "hydro":
        return presets.newton_mjwarp_hydro
    if name == "vbd_coupled":
        return coupled_vbd_cfg(
            rigid_bodies=[r"/World/envs/env_.*/Block"],
            proxied_rigid_bodies=[r"/World/envs/env_.*/Block"],
            **(coupling or {}),
        )
    raise ValueError(name)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("experiment", choices=("point", "pipeline", "hydro", "vbd_coupled"))
    parser.add_argument(
        "--coupling",
        type=json.loads,
        default={},
        help='vbd_coupled options as JSON, e.g. \'{"mode": "staggered", "coupling_iterations": 3}\'',
    )
    args = parser.parse_args()

    from isaaclab.scene import InteractiveScene
    from isaaclab.sim import SimulationCfg, build_simulation_context

    deformable = args.experiment == "vbd_coupled"
    sim_cfg = SimulationCfg(dt=1 / 240, device="cuda:0", physics=physics_cfg(args.experiment, args.coupling))
    result = {
        "experiment": args.experiment,
        "solver": type(sim_cfg.physics.solver_cfg).__name__,
        "coupling": args.coupling,
    }
    with build_simulation_context(sim_cfg=sim_cfg) as sim:
        scene = InteractiveScene(scene_cfg(deformable))
        sim.reset()
        block, pad = scene["block"], scene["pad"]
        to_t = lambda v: v if isinstance(v, torch.Tensor) else wp.to_torch(v)  # noqa: E731
        pad_top0 = float(to_t(pad.data.nodal_pos_w)[..., 2].max()) if deformable else PAD[2]
        heights, pad_tops, finite = [], [], True
        for _ in range(STEPS):
            scene.write_data_to_sim()
            sim.step(render=False)
            scene.update(sim.cfg.dt)
            z = float(to_t(block.data.root_pos_w)[0, 2])
            heights.append(z)
            finite &= bool(torch.isfinite(to_t(block.data.root_pos_w)).all())
            if deformable:
                nodes = to_t(pad.data.nodal_pos_w)
                finite &= bool(torch.isfinite(nodes).all())
                pad_tops.append(float(nodes[..., 2].max()))
        rest = PAD[2] + BLOCK / 2  # block centre resting on an undeformed pad
        result.update(
            final_block_z_m=heights[-1],
            min_block_z_m=min(heights),
            rigid_rest_z_m=rest,
            penetration_below_rigid_rest_mm=1000 * (rest - min(heights)),
            finite=finite,
        )
        if deformable:
            result.update(
                pad_top_initial_m=pad_top0,
                pad_top_final_m=pad_tops[-1],
                pad_compression_mm=1000 * (pad_top0 - min(pad_tops)),
            )
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
