"""Attempt 7b' checks (docs/experimental/rl_pip_only.md), one hand per process:

    uv run --extra rsl-rl python scripts/finger_relay/walk_checks.py --checkpoint <model.pt> --hand U --out <json> physics=newton_mjwarp

 (1) start geometry in each hand's face frame; (2) the cause of each early end under the transfer policy
(beyond the tips, below the face, off the side); (3) one open-loop flex/extend sequence on every hand."""
import argparse, json, sys, math
from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli
p = argparse.ArgumentParser(); p.add_argument("--checkpoint", required=True); p.add_argument("--out", required=True); p.add_argument("--hand", required=True)
add_launcher_args(p); p.set_defaults(device=None, visualizer=[])
args, hydra = setup_preset_cli(p, None); sys.argv = [sys.argv[0]] + hydra
import gymnasium as gym, torch, numpy as np
from importlib.metadata import version
import code_isaaclab.tasks  # noqa
from code_isaaclab.assets.description import Description
from code_isaaclab.tasks.rotate_cube.grasp import HANDS, face_frame
from code_isaaclab.tasks.finger_relay.env_cfg import tip_point_face
from code_isaaclab.tasks.finger_walk.mdp import walk_goal
from code_isaaclab.tasks.finger_walk import env_cfg as W
from code_isaaclab.hands.code_variants import HAND_ROOT_POS
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg
out = {}

def pad_range(desc, body, pad):  # the pad box's x extent and face height in the face frame at q = 0, MCP 0
    box = next(b for b in desc.body(body)["boxes"] if b["name"] == pad)
    c, h = np.asarray(box["center"]), box["half_extents"]
    o, r = face_frame(desc, 0.0); o, r = np.asarray(o), np.asarray(r).reshape(3, 3)
    pts = [r.T @ (np.asarray(HAND_ROOT_POS) + np.diag([1., -1., -1.]) @ np.array([c[0] + s * h[0], c[1], c[2] - h[1]]) - o) for s in (-1, 1)]
    return [round(float(pts[0][0]) * 1e3, 1), round(float(pts[1][0]) * 1e3, 1), round(float(pts[0][2]) * 1e3, 2)]

for H in (args.hand,):
    d = Description.load(HANDS[H])
    geo = {"middle_proximal_pad_x_from_to_z_mm": pad_range(d, "middle_proximal", "middle_proximal_pad"),
           "middle_tip_end_mm_at_q": {q: [round(float(v) * 1e3, 1) for v in tip_point_face(d, "middle", q)] for q in (0, 25, 45, 60, 90)},
           "cylinder_start_x_mm": [v * 1e3 for v in W.START_X_M], "cylinder_centre_z_mm": (W.RADIUS_M + 0.0015) * 1e3,
           "targets_mm": [v * 1e3 for v in W.TARGETS_M], "success_mm": W.SUCCESS_M * 1e3}
    out[H] = {"geometry": geo}
    print(H, json.dumps(geo), flush=True)

SEQ = [("open", 0, 0, 30), ("flex100", 0, 100, 60), ("hold100", 100, 100, 30), ("ext40", 100, 40, 60), ("flex100b", 40, 100, 60),
       ("ext30", 100, 30, 60), ("flex70", 30, 70, 45), ("ext0", 70, 0, 45)]
for H in (args.hand,):
    task = f"CoDe-Finger-Walk-{H}-Eval"
    cfg, agent = resolve_task_config(task, "rsl_rl_cfg_entry_point")
    n = 512
    cfg.scene.num_envs = n; cfg.seed = agent.seed = 94000; args.device = cfg.sim.device
    agent = handle_deprecated_rsl_rl_cfg(agent, version("rsl-rl-lib"))
    with launch_simulation(cfg, args), torch.inference_mode():
        env = gym.make(task, cfg=cfg); u = env.unwrapped
        wrapped = RslRlVecEnvWrapper(env)
        g = walk_goal(u)
        # (2) the policy: the first end of each episode and its cause
        runner = create_rsl_rl_runner(wrapped, agent); runner.load(args.checkpoint, load_cfg={"actor": True}, map_location=wrapped.device)
        policy = runner.get_inference_policy(device=wrapped.device)
        env.reset(seed=94000); obs = wrapped.get_observations()
        f0 = g.face().clone()
        cause = torch.zeros(n, dtype=torch.long, device=u.device)  # 0 none, 1 beyond tips, 2 below face, 3 side, 4 other
        alive = torch.ones(n, dtype=torch.bool, device=u.device)
        last = g.face().clone()
        for i in range(int(wrapped.max_episode_length) - 1):
            obs, _, dones, _ = wrapped.step(policy(obs))
            ended = dones.bool() & alive
            if ended.any():
                c = torch.full_like(cause, 4)
                c = torch.where(last[:, 0] > W.X_MAX_M - 0.01, 1, c)
                c = torch.where(last[:, 2] < W.Z_MIN_M + 0.005, 2, c)
                c = torch.where(last[:, 1].abs() > W.Y_MAX_M - 0.01, 3, c)
                cause = torch.where(ended, c, cause)
                alive &= ~ended
            last = torch.where(alive[:, None], g.face(), last)
        counts = {k: int((cause == v).sum()) for k, v in (("beyond_tips", 1), ("below_face", 2), ("off_side", 3), ("other", 4))}
        start = {"x_mm_mean": float(f0[:, 0].mean()) * 1e3, "z_mm_mean": float(f0[:, 2].mean()) * 1e3,
                 "y_mm_mean": float(f0[:, 1].mean()) * 1e3}
        out[H]["policy_ends"] = {**counts, "episodes": n, "last_x_mm_of_ended": float(last[~alive, 0].mean()) * 1e3 if (~alive).any() else None,
                                 "last_z_mm_of_ended": float(last[~alive, 2].mean()) * 1e3 if (~alive).any() else None}
        out[H]["start_state_t0"] = start
        print(H, "start", start, "ends", out[H]["policy_ends"], flush=True)
        # (3) open loop: the same flexion sequence on every finger
        env.reset(seed=94000)
        x0 = g.face()[:, 0].clone()
        alive = torch.ones(n, dtype=torch.bool, device=u.device)
        trace = []
        for name, a, b, k in SEQ:
            for j in range(k):
                deg = a + (b - a) * (j + 1) / k
                _, _, dones, _ = wrapped.step(torch.full((n, 3), -1 + 2 * deg / 120, device=u.device))
                alive &= ~dones.bool()
            f = g.face()
            trace.append({"phase": name, "x_mm_mean_alive": float(f[alive, 0].mean()) * 1e3 if alive.any() else None,
                          "x_mm_p10_p90": torch.quantile(f[alive, 0], torch.tensor([0.1, 0.9], device=f.device)).mul(1e3).tolist() if alive.any() else None,
                          "alive": float(alive.float().mean())})
        out[H]["open_loop"] = {"start_x_mm_mean": float(x0.mean()) * 1e3, "trace": trace}
        print(H, "open loop", json.dumps(out[H]["open_loop"]), flush=True)
        env.close()
open(args.out, "w").write(json.dumps(out, indent=1) + "\n")
