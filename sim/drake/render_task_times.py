"""The three Drake tasks on hand O (pad friction 1.3) rendered at t = 0, 1 and 2 s after each task starts:
grasp pull-out (t = 0 when the -z load ramp starts on the settled grasp), grasp acquisition (t = 0 when the open
finger starts closing) and experiment 28z's fingertip disk (t = 0 when the finger starts closing; frames at --dial_times,
default 1, 2.5 and 4 s: the touch angle, mid-sweep, the sweep's end). Same placements and settings as render_tasks.py; drawn on the hand's URDF geometry (render3d.render_on_hand); one row per task in logs/task_times.png.

    uv run python render_task_times.py [--times 0 1 2] [--dial_times 1 2.5 4]
"""
import argparse
import json
import math

import numpy as np

import render3d as R3
import render_tasks as RT
import two_pad_drake as T


def steps(t_s: float) -> int:
    return int(round(t_s / T.DT_ENV))


def pullout(times):
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = RT.setup()
    s1 = T.Stage(RT.HAND, RT.MU, RT.RHO, "point", 1e-3, True, k, d)
    s1.plant.GetJointByName("slide_x").set_translation(s1.pctx, x0[0])
    s1.plant.GetJointByName("slide_z").set_translation(s1.pctx, x0[1])
    s1.ctrl.hold_at = np.array([x0[0], x0[1], 0.0])
    s1.sim.Initialize()
    for i in range(70):
        s1.ctrl.theta_cmd = theta_cmd * min(1.0, (i + 1) / 40)
        s1.step()
    state = s1.plant.GetPositionsAndVelocities(s1.pctx).copy()
    s = T.Stage(RT.HAND, RT.MU, RT.RHO, "point", 1e-3, False, k, d)
    s.plant.SetPositionsAndVelocities(s.pctx, state)
    s.ctrl.theta_cmd = theta_cmd
    s.sim.Initialize()
    for _ in range(120):
        s.step()
    direction = np.array(T.DIRECTIONS["-z"])
    want = {steps(t): t for t in times}
    frames, load = {}, np.zeros(2)
    for i in range(max(want) + 1):
        if i in want:
            frames[want[i]] = RT.frame(s, load)
        load = T.RATE_N_S * T.DT_ENV * (i + 1) * direction
        s.ctrl.load = load
        s.step()
    return frames


def acquire(times):
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = RT.setup()
    T.SPIN_DAMPING = 2.0
    s = T.Stage(RT.HAND, RT.MU, RT.RHO, "point", 1e-3, False, k, d)
    start = np.array([x0[0] + 6e-3, x0[1]])
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, start[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, start[1])
    load = np.array([1.5, 1.5]) / T.FINGERS
    s.ctrl.load = load
    s.sim.Initialize()
    want = {steps(t): t for t in times}
    frames = {}
    for i in range(max(want) + 1):
        if i in want:
            frames[want[i]] = RT.frame(s, load)
        s.ctrl.theta_cmd = theta_cmd * min(1.0, (i + 1) / 60)
        s.step()
    T.SPIN_DAMPING = 0.0
    return frames


def dial(times):
    g = [x for x in json.loads((T.OUT / "grasps_r10.json").read_text())["grasps"] if x["q0_deg"] == RT.DIAL_Q0][0]
    geom, qs, th, q_cmd, theta_cmd, k, d, x0 = RT.setup(g, RT.DIAL_RHO)
    T.DIAL_Z_STIFFNESS, T.DIAL_BRAKE_NM = 300.0, 0.0
    s = T.Stage(RT.HAND, RT.MU, RT.DIAL_RHO, "point", 1e-3, "xz", k, d)
    centre = np.array([x0[0], x0[1] - 3e-3])
    s.plant.GetJointByName("slide_x").set_translation(s.pctx, centre[0])
    s.plant.GetJointByName("slide_z").set_translation(s.pctx, centre[1])
    s.ctrl.hold_at = np.array([centre[0], centre[1], 0.0])
    s.sim.Initialize()
    q1 = min(q_cmd + math.radians(45.0), math.radians(120.0))
    want = {steps(t): t for t in times}
    frames = {}
    for i in range(max(want) + 1):
        if i in want:
            frames[want[i]] = RT.frame(s)
        q = q1 * min(1.0, (i + 1) / 90)
        s.ctrl.theta_cmd = float(np.interp(q, qs, th))
        s.step()
    T.DIAL_Z_STIFFNESS = 0.0
    return frames, [1e3 * centre[0], 1e3 * centre[1]]


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--times", type=float, nargs="+", default=[0.0, 1.0, 2.0])
    p.add_argument("--dial_times", type=float, nargs="+", default=[1.0, 2.5, 4.0],
                   help="the fingertip disk's frames [s from the start: 1 s to the touch angle, 3 s sweep, 1 s hold]")
    a = p.parse_args()
    geom = T.finger_geometry(RT.HAND)
    snap = {"hand": RT.HAND, "mu": RT.MU, "rho_mm": RT.RHO, "q0_deg": RT.G["q0_deg"], "pads": geom["pads"],
            "b_mm": geom["b"].tolist()}
    tasks = {"hold": pullout(a.times), "acquire": acquire(a.times)}
    snap["tipdial"] = RT.tipdial(a.dial_times)
    for k in ("hold", "acquire"):
        snap[k] = {"frames": [tasks[k][t] for t in a.times]}
    (T.OUT / "task_times.json").write_text(json.dumps({"times_s": a.times, **snap}, indent=1))

    paths = {}
    for i, (t, st) in enumerate(zip(a.dial_times, snap["tipdial"]["frames"])):
        sc = R3.tipdial_scene(st, snap["tipdial"], T.OUT / "render3d" / f"t_tipdial{i}")
        out = T.OUT / f"render_t_tipdial_{t:g}s.png"
        R3.render_on_hand(sc, out, st["distal"][2], camera=R3.TIPDIAL_CAMERA, ref=R3.TIPDIAL_REF,
                          fingers=R3.TIPDIAL_FINGERS, curled=R3.TIPDIAL_CURLED)
        paths[("tipdial", i)] = out
    for task in ("hold", "acquire"):
        frames = snap[task]["frames"]
        for i, (t, st) in enumerate(zip(a.times, frames)):
            ghost = frames[0]["centre"] if i > 0 and task in ("hold", "acquire") else None
            sc = R3.build(task, st, snap, T.OUT / "render3d" / f"t_{task}{i}", ghost=ghost,
                          turn_sign=math.copysign(1.0, frames[-1]["spin"] or 1.0), finger=False, span=R3.task_span())
            out = T.OUT / f"render_t_{task}_{t:g}s.png"
            R3.render_on_hand(sc, out, st["distal"][2])
            paths[(task, i)] = out

    from PIL import Image
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = {"hold": "grasp pull-out\n(−z load from t = 0)", "acquire": "grasp acquisition\n(closing from t = 0)",
             "tipdial": "fingertip disk (z axis)\n(closing from t = 0)"}
    fig, axes = plt.subplots(3, len(a.times), figsize=(3.2 * len(a.times), 8.0))
    for r, task in enumerate(("hold", "acquire", "tipdial")):
        for c, t in enumerate(a.dial_times if task == "tipdial" else a.times):
            ax = axes[r, c]
            ax.imshow(np.asarray(Image.open(paths[(task, c)]).convert("RGB")))
            ax.axis("off")
            st = snap[task]["frames"][c]
            load = np.linalg.norm(st.get("load_n", [0, 0]))
            extra = f"load {load:.1f} N" if task == "hold" else (f"turned {st['rotation_deg'] + 0.0:.0f}°" if task == "tipdial" else "")
            ax.set_title(f"t = {t:g} s" + (f"\n{extra}" if extra else ""), fontsize=9)
        axes[r, 0].text(-0.04, 0.5, names[task], transform=axes[r, 0].transAxes, rotation=90, ha="right",
                        va="center", fontsize=9)
    fig.tight_layout()
    out = T.OUT / "task_times.png"
    fig.savefig(out, dpi=150)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
