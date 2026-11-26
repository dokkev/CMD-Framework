# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Generate the four reset datasets of ``CoDe-Rotate-Cube-O-OmniReset`` (OmniReset's reset-state collection).

    env -u DISPLAY -u WAYLAND_DISPLAY PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run python \\
        scripts/rotate_cube/gen_omnireset_resets.py [--type all] [--count 10000] [--num_envs 4096] [--hold_s 2] \\
        [--out_dir <dir>] [--seed_start 0] physics=newton_mjwarp

OmniReset (UWLab ``scripts_v2/tools/record_reset_states.py`` with ``omnireset/config/ur5e_robotiq_2f85/
reset_states_cfg.py``) samples raw states per reset type, simulates each for a 2 s episode with zero actions, and keeps
the settled final state (``StableStateRecorder.record_pre_reset``) when ``check_reset_state_success`` passes: no joint
faster than twice its limit, 5 consecutive stable steps (joint speeds summed < 5 rad/s, object speed summed < 0.1 m/s
and < 1 rad/s), the object within ``max_object_pos_deviation`` of where it started (reaching: any; near-object 1 cm;
grasped 5 cm; near-goal 2.5 cm), above the floor, and collision-free. It runs with low friction (0.3 static, 0.2
dynamic) so the kept states hold without it, and collects 10,000 states per type.

Here, per type (raw sampling in :meth:`~code_isaaclab.tasks.rotate_cube_omnireset.mdp.OmniResetStates._generate`,
ranges in ``env_cfg.GEN``):

- ``ObjectAnywhereHandAnywhere`` (reaching): every actuated joint uniform over its range (PIP flexion 0-120 deg), the
  cube dropped 5-25 mm over the pads (+-15 mm) at a uniform random orientation; kept if it settles on the hand;
- ``ObjectAnywhereHandGrasped`` (stable grasp, OmniReset's grasp sampler + object anywhere): a pinch with each joint
  uniform in a window around the cradle grasp, clipped to its limits without piling up at them (the thumb rotation
  spans -90..-50 deg instead of sitting at -90), each finger lifted off with probability 0.25 (regrasp-like), the
  fingers closing 0-10 deg onto the cube, which lies anywhere within +-8 mm of its rest at any yaw and up to 10 deg
  tilt; kept if the thumb and at least one finger touch the cube;
- ``ObjectRestingHandNear`` (near-object): a stable grasp from the grasp dataset with the fingers and thumb opened by
  0-20 deg (OmniReset: the gripper +-2 cm, +-pi/16 off a grasp);
- ``ObjectNearGoalHandGrasped`` (near-goal): a stable grasp with the cube within 15 mm of its rest and lying flat
  (tilt < 10 deg), jittered by +-2 deg (OmniReset: +-1 cm, +-pi/32), with the goal yaw within +-0.3 rad of the cube's
  (half; inside 1.5 x the success threshold) or 0.3-0.8 rad ahead (half; OmniReset's ``assembly_success_prob`` 0.5).

The datasets depend on the type order (``--type all`` runs grasped before near-object and near-goal). Penetration is
not checked separately: unstable overlaps fail the stability and deviation tests. Writes ``<out_dir>/resets_<type>.pt``
(refusing to overwrite) and ``.json`` (counts, acceptance, ranges). ``--count 10000`` is OmniReset's.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import sys
import time
from pathlib import Path

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

TASK = "CoDe-Rotate-Cube-O-OmniReset"
"""The default task (the cube); ``--task CoDe-Rotate-Cylinder-O-Fingertip`` makes the cylinder's datasets."""
ORDER = ("ObjectAnywhereHandAnywhere", "ObjectAnywhereHandGrasped", "ObjectRestingHandNear",
         "ObjectNearGoalHandGrasped")
MAX_DEVIATION_M = {"ObjectAnywhereHandAnywhere": math.inf, "ObjectRestingHandNear": 0.01,
                   "ObjectAnywhereHandGrasped": 0.05, "ObjectNearGoalHandGrasped": 0.025,
                   "ObjectMidTurnHandWound": 0.025}  # fmt: skip
MIDTURN = "ObjectMidTurnHandWound"
FINGER_BODIES = [f"{f}_{s}" for f in ("index", "middle", "ring") for s in ("proximal", "distal")]


def main(argv=None) -> int:
    from isaaclab.app import add_launcher_args

    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--task", default=TASK)
    parser.add_argument("--type", default="all", choices=("all", *ORDER, MIDTURN),
                        help="all: OmniReset's four types; ObjectMidTurnHandWound (v2) needs --source_rows")
    parser.add_argument("--source_rows", type=Path, help="harvested states (harvest_omnireset_midturn.py)")
    parser.add_argument("--render", action="store_true", help="a PNG of four kept states per type")
    parser.add_argument("--count", type=int, default=10000)
    parser.add_argument("--num_envs", type=int, default=4096)
    parser.add_argument("--hold_s", type=float, default=2.0)
    parser.add_argument("--max_rounds", type=int, default=60)
    parser.add_argument("--seed_start", type=int, default=0)
    parser.add_argument("--out_dir", type=Path)
    parser.add_argument("--friction", type=float, default=None,
                        help="friction of every hand and cube shape while collecting (OmniReset: 0.3; default: the "
                             "contact model's)")
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=[])
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from isaaclab.app import launch_simulation
    from isaaclab.sensors import ContactSensorCfg

    from isaaclab_tasks.utils import resolve_task_config

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.tasks.rotate_cube_omnireset.mdp import object_tilt, reset_term

    env_cfg, _ = resolve_task_config(args.task, "rsl_rl_cfg_entry_point")
    out_dir = args.out_dir or Path(env_cfg.reset_datasets)
    types = ORDER if args.type == "all" else (args.type,)
    for t in types:
        if (out_dir / f"resets_{t}.pt").exists():
            parser.error(f"destination already exists: {out_dir / f'resets_{t}.pt'}")
    grasp_path = out_dir / "resets_ObjectAnywhereHandGrasped.pt"
    first = types[0]
    params = env_cfg.events.reset_grasp.params
    params.pop("datasets", None)
    params.pop("probs", None)
    params["generate"] = first
    if first in ("ObjectRestingHandNear", "ObjectNearGoalHandGrasped"):
        params["grasp_rows"] = str(grasp_path)
    if first == MIDTURN:
        if not args.source_rows:
            parser.error("ObjectMidTurnHandWound needs --source_rows")
        params["grasp_rows"] = str(args.source_rows)
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.episode_length_s = 1.0e4
    env_cfg.terminations.abnormal_robot = None
    env_cfg.terminations.non_finite_state = None
    env_cfg.rewards.abnormal_robot = None
    env_cfg.terminations.cube_out_of_reach = None
    # the cylinder task's palm-contact end would reset (and regenerate) worlds mid-hold; the on_palm test rejects them
    if getattr(env_cfg.terminations, "object_palm_contact", None) is not None:
        env_cfg.terminations.object_palm_contact = None
    for term in ("servo_gains",):
        setattr(env_cfg.events, term, None)
    env_cfg.actions.joint_pos.gain_log_range = (1.0, 1.0)
    from code_isaaclab import contact_model

    if args.friction is not None:  # OmniReset collects at low friction (0.3) so the kept states hold without grip
        contact_model.PAD_MU = contact_model.CUBE_MU = contact_model.HAND_MU = args.friction
    env_cfg.events.cube_mass = None
    env_cfg.scene.obj_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/object",
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{b}" for b in FINGER_BODIES + ["thumb_distal"]],
    )
    args.device = env_cfg.sim.device
    rest = torch.tensor(env_cfg.grasp["rest_env"])
    started = time.time()
    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make(args.task, cfg=env_cfg)
        cleanup.callback(env.close)
        u = env.unwrapped
        n, dev = u.num_envs, u.device
        robot, obj, sensor = u.scene["robot"], u.scene["object"], u.scene.sensors["obj_contacts"]
        term = reset_term(u)
        zero = torch.zeros(n, u.action_manager.total_action_dim, device=dev)
        rest_d = rest.to(dev)
        tq = lambda v: v if isinstance(v, torch.Tensor) else v.torch  # noqa: E731
        vel_limit = tq(robot.data.joint_vel_limits)
        seed = args.seed_start
        for kind in types:
            term.generate = kind
            term.names = [kind]
            if kind in ("ObjectRestingHandNear", "ObjectNearGoalHandGrasped"):
                from code_isaaclab.tasks.rotate_cube_omnireset.mdp import load_reset_dataset

                rows = load_reset_dataset(grasp_path, robot.joint_names, params["goal_step"])
                if kind == "ObjectNearGoalHandGrasped":  # near the goal pose: on the rest position, lying flat
                    tilt = object_tilt(u, rows["quat"].to(dev)).cpu()
                    near = ((rows["pos"] - rest).norm(dim=1) < 0.015) & (tilt < math.radians(10))
                    rows = {k: v[near] for k, v in rows.items()}
                    if not len(rows["q"]):
                        raise RuntimeError("no stable grasp near the goal pose to start near-goal states from")
                term.grasp_rows = {k: v.to(dev) for k, v in rows.items()}
            kept = {k: [] for k in ("q", "qd", "target", "pos", "quat", "vel", "goal_offset", "progress", "lifted")}
            tried, reasons = 0, {}
            for rnd in range(args.max_rounds):
                env.reset(seed=seed)
                seed += 1
                start = tq(obj.data.root_pos_w).clone()
                stable = torch.zeros(n, dtype=torch.long, device=dev)
                abnormal = torch.zeros(n, dtype=torch.bool, device=dev)
                for _ in range(int(args.hold_s / u.step_dt)):
                    env.step(zero)
                    qd = tq(robot.data.joint_vel)
                    abnormal |= (qd.abs() > 2.0 * vel_limit).any(dim=1)
                    ok = (qd.abs().sum(dim=1) < 5.0) & (tq(obj.data.root_lin_vel_w).abs().sum(dim=1) < 0.1) & (
                        tq(obj.data.root_ang_vel_w).abs().sum(dim=1) < 1.0)  # fmt: skip
                    stable = torch.where(ok, stable + 1, 0)
                pos = tq(obj.data.root_pos_w) - u.scene.env_origins
                quat = tq(obj.data.root_quat_w)
                deviation = (tq(obj.data.root_pos_w) - start).norm(dim=1)
                touch = tq(sensor.data.force_matrix_w)[:, 0].norm(dim=-1) > 0.05
                tests = {
                    "abnormal": ~abnormal,
                    "unstable": stable >= 5,
                    "deviation": deviation <= MAX_DEVIATION_M[kind],
                    "off_hand": ((pos - rest_d).norm(dim=1) < 0.08) & (pos[:, 2] > rest_d[2] - 0.03),
                    "finite": torch.isfinite(pos).all(dim=1) & torch.isfinite(tq(robot.data.joint_pos)).all(dim=1),
                }
                if "palm_contact" in u.scene.sensors:  # the cylinder task: never on the palm
                    palm = u.scene.sensors["palm_contact"].data.force_matrix_w.torch.reshape(n, -1, 3).norm(dim=-1)
                    tests["on_palm"] = palm.max(dim=1).values <= 0.05
                if kind == "ObjectAnywhereHandGrasped":
                    tests["not_held"] = touch[:, -1] & touch[:, :-1].any(dim=1)
                if kind == MIDTURN:  # still held near the pads, lying near flat
                    tests["not_held"] = touch[:, -1] & touch[:, :-1].any(dim=1)
                    tests["off_pads"] = ((pos - rest_d).norm(dim=1) < 0.03) & (object_tilt(u, quat) < math.radians(20))
                if kind == "ObjectNearGoalHandGrasped":
                    tests["off_goal"] = ((pos - rest_d).norm(dim=1) < 0.015) & (object_tilt(u, quat) < math.radians(10))
                keep = torch.ones(n, dtype=torch.bool, device=dev)
                for name, passed in tests.items():
                    reasons[name] = reasons.get(name, 0) + int((keep & ~passed).sum())
                    keep &= passed
                tried += n
                for key, val in (("q", tq(robot.data.joint_pos)), ("qd", tq(robot.data.joint_vel)),
                                 ("target", tq(robot.data.joint_pos_target)), ("pos", pos), ("quat", quat),
                                 ("vel", torch.cat((tq(obj.data.root_lin_vel_w), tq(obj.data.root_ang_vel_w)), 1)),
                                 ("goal_offset", term.goal_offset)):  # fmt: skip
                    kept[key].append(val[keep].cpu())
                prog = getattr(term, "gen_progress", None)
                prog = prog if prog is not None else torch.full((n,), math.nan, device=dev)
                kept["progress"].append(prog[keep].cpu())
                lifted = getattr(term, "gen_lifted", None) if kind == MIDTURN else None
                kept["lifted"].append((lifted if lifted is not None else torch.zeros(n, dtype=torch.bool,
                                                                                      device=dev))[keep].cpu())
                total = sum(len(x) for x in kept["q"])
                print(f"{kind} round {rnd}: kept {int(keep.sum())}/{n}, total {total}; rejected (first failing test) "
                      f"{reasons}", flush=True)
                if total >= args.count:
                    break
            data = {k: torch.cat(v)[: args.count] for k, v in kept.items()}
            out_dir.mkdir(parents=True, exist_ok=True)
            if args.render:
                render(u, keep, out_dir / f"resets_{kind}_samples.png")
            accepted = sum(len(x) for x in kept["q"])
            meta = {
                "type": kind, "task": args.task, "joint_names": list(robot.joint_names), "hand": env_cfg.hand_key,
                "pad_profile": env_cfg.pad_profile().__dict__ | {"key": env_cfg.pad_profile().key()},
                "servo_colliders": "pip", "hold_s": args.hold_s, "seed_start": args.seed_start, "tried": tried,
                "accepted": accepted, "acceptance": accepted / max(tried, 1), "rejected_first_failing_test": reasons,
                "stored": int(data["q"].shape[0]), "generation_friction": args.friction,
                "contact_model": env_cfg.contact_model,
                "ranges": {k: v for k, v in params["gen"].items()}, "goal_step_rad": params["goal_step"],
                "physics": [a for a in hydra_args if a.startswith("physics=")],
            }  # fmt: skip
            out_dir.mkdir(parents=True, exist_ok=True)
            with (out_dir / f"resets_{kind}.pt").open("xb") as stream:
                torch.save({**data, **{k: meta[k] for k in ("joint_names", "type", "hand")}}, stream)
            (out_dir / f"resets_{kind}.json").write_text(json.dumps(meta, indent=1, default=str) + "\n")
            path = out_dir / f"resets_{kind}.pt"
            print(f"{path}: {meta['stored']} states; acceptance {accepted}/{tried}", flush=True)
    print(f"wall {time.time() - started:.0f} s")
    return 0


def render(u, keep, path) -> None:
    """Four kept states of the last round, drawn with Newton's GL viewer offscreen, colliders shown (the hand's
    drawing meshes are Isaac Lab's; the colliders are what the physics sees)."""
    import pyglet

    if not os.environ.get("DISPLAY"):
        pyglet.options["headless"] = True
    import newton
    import numpy as np
    import warp as wp
    from PIL import Image

    from code_isaaclab.tasks.rotate_cube.camera import CAMERA_EYE_ENV, CAMERA_LOOKAT_ENV

    viewer = newton.viewer.ViewerGL(width=960, height=540, headless=True)
    viewer.set_model(u.sim.physics_manager.get_model())
    viewer.show_collision = True
    viewer.set_world_offsets((0.0, 0.0, 0.0))
    frames = []
    for k in keep.nonzero().flatten()[:4].tolist():
        viewer.set_visible_worlds([k])
        o = u.scene.env_origins[k].tolist()
        eye = [o[i] + CAMERA_EYE_ENV[i] for i in range(3)]
        d = [CAMERA_LOOKAT_ENV[i] - CAMERA_EYE_ENV[i] for i in range(3)]
        viewer.set_camera(pos=wp.vec3(*eye), pitch=math.degrees(math.atan2(d[2], math.hypot(d[0], d[1]))),
                          yaw=math.degrees(math.atan2(d[1], d[0])))
        for _ in range(2):
            viewer.begin_frame(0.0)
            viewer.log_state(u.sim.physics_manager.get_state_0())
            viewer.end_frame()
        frames.append(viewer.get_frame().numpy())
    viewer.close()
    if len(frames) >= 2:
        grid = np.concatenate([np.concatenate(frames[i:i + 2], axis=1) for i in range(0, len(frames) - 1, 2)], axis=0)
        Image.fromarray(grid).save(path)
        print(path, flush=True)


if __name__ == "__main__":
    os.environ.setdefault("PYTEST_DISABLE_PLUGIN_AUTOLOAD", "1")
    raise SystemExit(main())
