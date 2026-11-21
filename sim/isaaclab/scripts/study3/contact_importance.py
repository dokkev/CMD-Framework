# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3: does a contact offset matter more at the distal face (relative contact trajectory) or at the proximal
face (relative to the MCP)? A scripted grasp, no learning.

    uv run python scripts/study3/contact_importance.py --variant O|P1|P2|P4|D1|D2|D4 --scenario fingers|thumb

The hand is Study 3's H0 (CFL, 120 deg, 3 mm pads). P(delta) moves every body the MCP flexion carries ``delta`` mm
palmar (relative contact trajectory kept exactly); D(delta) thickens the distal pads by ``delta`` mm on their
contact side (only the relative contact trajectory changes) (``code_isaaclab.assets.perturb``).

A square bar (30 x 30 mm, 70 mm long across the fingers, 50 g) lies on the proximal pads from x = 45 to 75 mm, over the
middle finger (a cylinder rolls freely on the flat pads of the palm-up hand and was replaced, 2026-11-21). Commanded in flexion (``FlexionEMAAction``, alpha 1): settle 1 s; flex the PIPs to 120 deg over
1.5 s, wrapping the distal pads over its far top edge; ``thumb``: move the thumb, by inverse kinematics solved once on O,
to press the bar's top 3 mm deep from the palmar side; hold 1.5 s; then pull the bar away from the palm with a force
rising 3 N/s. The bar's position and the thumb target are O's for every variant.

Records: per-pad contact forces, the thumb's force and the bar's position (hand frame) over the last second of
the hold; the pull force at which the bar has moved 5 mm from its held position (the grasp's strength); loop
residual. Writes ``logs/study3_contact/<run>.json``.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import sys
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

BASE = common.REPO_ROOT / "design" / "output" / "sim" / "study3" / "hands" / "cfl_q120_h3to7_h0.json"
OUT = common.SIM_ROOT / "logs" / "study3_contact"
BAR = (0.030, 0.070, 0.030)  # x (along the fingers), y (across), z (height) [m]
MASS = 0.050
FRICTION = 1.0
"""Bar friction coefficient (rubber pads on a held part; MuJoCo uses the larger of the two geoms')."""
X_HAND, Y_HAND = 0.060, -0.025
PRESS = 0.003
THUMB_RADIUS = 0.006
HAND_Z = 0.5
STEPS = {"settle": 30, "wrap": 45, "thumb": 30, "hold": 45, "pull": 450}
PULL_RATE_N_S = 3.0
SLIP_M = 0.005


def variant_description(variant: str) -> Path:
    from code_isaaclab.assets.perturb import offset_distal_faces, offset_fingers

    path = OUT / "descriptions" / f"h0_{variant}.json"
    if path.exists():
        return path
    desc = json.loads(BASE.read_text())
    if variant != "O":
        delta = float(variant[1:]) * 1e-3
        desc = offset_fingers(desc, delta) if variant[0] == "P" else offset_distal_faces(desc, delta)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(desc, indent=1))
    return path


def proximal_face_z(desc: dict) -> float:
    """The middle finger's proximal pad contact face, hand z [m] (palmar = more negative)."""
    box = next(b for body in desc["bodies"] for b in (body.get("boxes") or []) if b["name"] == "middle_proximal_pad")
    return float(box["center"][2]) - float(box["half_extents"][1])


def thumb_ik(desc: dict, target_hand: np.ndarray) -> dict[str, float]:
    from scipy.optimize import minimize

    joints = {j["name"]: j for j in desc["joints"]}
    chain = ["thumb_ro", "thumb_raa", "thumb_mcp", "thumb_ip"]
    tip0 = np.asarray(desc["tips"]["thumb"], dtype=float)

    def rot(axis, angle):
        a = np.asarray(axis, float) / np.linalg.norm(axis)
        k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k

    def fk(q):
        p = tip0.copy()
        for name, angle in reversed(list(zip(chain, q))):
            c = np.asarray(joints[name]["anchor"], float)
            p = c + rot(joints[name]["axis"], angle) @ (p - c)
        return p

    bounds = [tuple(math.radians(v) for v in joints[n]["limits_deg"]) for n in chain]
    best = None
    for start in ([-1.5, 0.3, 1.4, 0.0], [-1.4, 0.3, 1.2, 0.3], [-1.2, 0.2, 1.0, 0.5]):
        r = minimize(lambda q: float(np.sum((fk(q) - target_hand) ** 2)), start, bounds=bounds, method="L-BFGS-B")
        best = r if best is None or r.fun < best.fun else best
    err = math.sqrt(best.fun)
    if err > 5e-4:
        raise RuntimeError(f"thumb cannot reach {target_hand} (error {1e3 * err:.2f} mm)")
    return dict(zip(chain, (float(v) for v in best.x))) | {"_ik_error_m": err}


def main(argv=None) -> int:
    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--variant", default="O")
    parser.add_argument("--scenario", default="fingers", choices=("fingers", "thumb"))
    parser.add_argument("--friction", type=float, default=FRICTION)
    parser.add_argument("--target_deg", type=float, default=120.0, help="PIP flexion commanded at the end of the wrap")
    parser.add_argument("--tag", default="")
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from isaaclab.app import launch_simulation
    from isaaclab.utils import configclass

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.flexion_control import FlexionEMAActionCfg
    from code_isaaclab.hands import get_hand
    from code_isaaclab.hands.description_hands import bar_cfg, register_description_hand
    from code_isaaclab.tasks.reorient.code_env_cfg import CoDeReorientMainEnvCfg

    physics = next((t.split("=", 1)[1] for t in hydra_args if t.startswith("physics=")), "newton_mjwarp")
    base_desc = json.loads(BASE.read_text())
    path = variant_description(args.variant)
    desc = json.loads(path.read_text())
    face_o = proximal_face_z(base_desc)
    face = proximal_face_z(desc)
    # the bar rests on this variant's pads; x, y and the thumb target are O's
    pos_w = (X_HAND, -Y_HAND, HAND_Z - (face - BAR[2] / 2) + 0.001)
    name = register_description_hand(f"s3c_{args.variant}", path, bar_cfg(BAR, MASS, pos_w, friction=args.friction))
    thumb = None
    if args.scenario == "thumb":
        thumb = thumb_ik(base_desc, np.array([X_HAND - 0.002, Y_HAND, face_o - BAR[2] - THUMB_RADIUS + PRESS]))

    @configclass
    class ContactCfg(CoDeReorientMainEnvCfg):
        hand: str = name

    env_cfg = ContactCfg()
    env_cfg.sim.physics = getattr(env_cfg.sim.physics, physics)  # resolve the preset (no Hydra here)
    env_cfg.scene.num_envs = 1
    env_cfg.events.reset_object = None
    env_cfg.events.reset_hand.params["joint_position_noise"] = 0.0
    env_cfg.terminations.object_out_of_reach = None
    env_cfg.episode_length_s = 1.0e4
    old = env_cfg.actions.joint_pos
    env_cfg.actions.joint_pos = FlexionEMAActionCfg(asset_name="robot", joint_names=old.joint_names, alpha=1.0,
                                                    four_bar_tables=get_hand(name).four_bar_tables,
                                                    flexion_limits_deg=(0.0, 120.0))
    bodies = common.add_sensors(env_cfg, physics)
    args.device = env_cfg.sim.device

    log = []
    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make("CoDe-Reorient-Cube-CoDeFlat-Main", cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset(seed=0)
        u = env.unwrapped
        robot, obj = u.scene["robot"], u.scene["object"]
        probe = common.Probe(env, bodies, description=path)
        term = u.action_manager.get_term("joint_pos")
        names = list(term._joint_names)
        lo, hi = term._cmd_limits[0, :, 0], term._cmd_limits[0, :, 1]
        sensor = u.scene.sensors["cube_contacts"]
        thumb_cols = [i for i, b in enumerate(bodies) if b.startswith("thumb")]
        origin = u.scene.env_origins[0]
        dt = u.step_dt

        def command(pip_deg: float, thumb_frac: float) -> torch.Tensor:
            cmd = torch.zeros(1, len(names), device=u.device)
            for k, n in enumerate(names):
                if n.endswith("_pip_servo") and not n.startswith("thumb"):
                    cmd[0, k] = math.radians(pip_deg)
                elif thumb is not None and n in thumb:
                    cmd[0, k] = thumb_frac * thumb[n]
            return 2.0 * (cmd - lo) / (hi - lo) - 1.0

        def bar_hand() -> np.ndarray:
            p = (common.torch_of(obj.data.root_pos_w)[0] - origin).detach().cpu().numpy()
            return np.array([p[0], -p[1], HAND_Z - p[2]])

        def record(phase: str, force: float = 0.0):
            m = probe.step_metrics()
            matrix = common.torch_of(sensor.data.force_matrix_w)[0, 0].norm(dim=-1)
            log.append({"phase": phase, "pull_n": force, "cyl_hand_m": bar_hand().tolist(),
                        "pad_force_n": m["pad_force_n"][0].tolist() if "pad_force_n" in m else None,
                        "thumb_force_n": float(matrix[thumb_cols].sum()),
                        "loop_mm": float(m["loop_residual_mm"][0])})

        schedule = ([("settle", 0.0, 0.0)] * STEPS["settle"]
                    + [("wrap", args.target_deg * (i + 1) / STEPS["wrap"], 0.0) for i in range(STEPS["wrap"])])
        if thumb is not None:
            schedule += [("thumb", args.target_deg, (i + 1) / STEPS["thumb"]) for i in range(STEPS["thumb"])]
        schedule += [("hold", args.target_deg, 1.0 if thumb is not None else 0.0)] * STEPS["hold"]
        for phase, pip, frac in schedule:
            env.step(command(pip, frac))
            record(phase)
        held = np.mean([r["cyl_hand_m"] for r in log[-30:]], axis=0)
        strength = None
        for i in range(STEPS["pull"]):
            force = PULL_RATE_N_S * dt * (i + 1)
            f = torch.zeros(1, 1, 3, device=u.device)
            f[0, 0, 2] = force  # world +z: away from the palm (the palm faces up)
            obj.permanent_wrench_composer.set_forces_and_torques_index(
                forces=f, torques=torch.zeros_like(f), body_ids=[0], env_ids=torch.tensor([0], device=u.device),
                is_global=True)
            env.step(command(args.target_deg, 1.0 if thumb is not None else 0.0))
            record("pull", force)
            if strength is None and np.linalg.norm(np.array(log[-1]["cyl_hand_m"]) - held) > SLIP_M:
                strength = force
                break

    hold = [r for r in log if r["phase"] == "hold"][-30:]
    pads = np.mean([r["pad_force_n"] for r in hold], axis=0) if hold[0]["pad_force_n"] is not None else None
    result = {
        "variant": args.variant, "scenario": args.scenario, "physics": physics, "friction": args.friction,
        "target_deg": args.target_deg, "description": str(path),
        "perturbation": desc.get("perturbation"), "thumb_ik": thumb,
        "hold": {"cyl_hand_mm": (1e3 * held).tolist(),
                 "pad_force_n": None if pads is None else dict(zip(common.PAD_BODIES, pads.tolist())),
                 "proximal_pads_n": None if pads is None else float(pads[0::2].sum()),
                 "distal_pads_n": None if pads is None else float(pads[1::2].sum()),
                 "thumb_n": float(np.mean([r["thumb_force_n"] for r in hold])),
                 "loop_mm_max": max(r["loop_mm"] for r in log)},
        "pull_out_n": strength, "pull_limit_n": PULL_RATE_N_S * dt * STEPS["pull"],
    }
    run = common.run_id(f"contact_{args.variant}_{args.scenario}{('_' + args.tag) if args.tag else ''}")
    path_out = common.write_json(OUT / f"{run}.json", {**result, "log": log, "manifest": common.manifest(command=sys.argv)})
    print(json.dumps({k: v for k, v in result.items() if k not in ("thumb_ik",)}, indent=1))
    print(path_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
