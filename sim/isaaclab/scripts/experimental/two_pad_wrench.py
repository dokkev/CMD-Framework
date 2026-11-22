# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Experimental (playground, not the SSoT): a cylinder held by both pads, free in the plane, loaded until the grasp
gives way; is a pad-friction effect on that load measured alike after each redesign, with the distal force matched?

    uv run python scripts/experimental/two_pad_wrench.py --hand O|R|U --mu MU --condition A|B [--radius_mm 15|20]

Pre-registered in ``docs/experimental/separability.md`` (third experiment, Stage B). Hands O, R, U (CFL, 120 deg,
3 -> 7 mm, flat pads with friction ``mu``; the cylinder's own coefficient is 0.01). Placements from e17
(``design/output/sim/study3/two_pad_grasps.json``): a cylinder where O's two pads touch it at q0, at the same height
above each hand's own proximal face; one environment per placement and load direction. Gravity off.

1. The cylinder is held by stiff drives on its planar joints (x, z slides and y spin) while the PIP closes to
   q0 + ``--delta_deg``; the drives are then released over 30 steps and the grasp settles (Gate 0, release).
2. Condition C (joint-torque reuse): after closing in position control on the held cylinder, the PIP hands over
   (30 steps, stiffness to 0) to a feedforward torque, O's flexion torque per finger at its condition-A grasp
   through this hand's own dtheta/dq; then the cylinder is released.
   Condition B: the PIP command is adjusted by force feedback until the distal pads' total normal force equals O's
   (condition A, same mu and placement, latest run).
3. A force on the cylinder's centre rises at ``--rate`` N/s along d (proximal-face frame, object on z < 0):
   +x (to the tip), +x-z, -z (off the face), -x (to the MCP). F_limit is the load at the first of: slip (a pad
   contact's tangential relative speed above 10 mm/s; 5 and 20 also kept), contact loss (proximal or distal total
   normal force below 0.2 N), collapse (5 mm from the pre-ramp pose).

Records per environment: q* (each finger's measured flexion), each pad's normal force vector (its direction is the
contact normal), friction force and contact point (proximal-face frame, and relative to the cylinder's centre), the
cylinder's pose, the Gate 0 checks, F_limit by event, and the slipping pad's |f_t|/|f_n| at slip. Writes
``logs/experimental/two_pad/<run>.json``.
"""

from __future__ import annotations

import argparse
import contextlib
import glob
import json
import math
import sys
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "study3"))
sys.path.insert(0, str(HERE))
import common  # noqa: E402
from separability_quasistatic import FINGER_Y_M, HAND_Z, HANDS, MCP_X_M, proximal_face_z  # noqa: E402

OUT = common.SIM_ROOT / "logs" / "experimental" / "two_pad"
GRASPS_JSON = common.SIM_ROOT.parents[1] / "design" / "output" / "sim" / "study3" / "two_pad_grasps.json"
S2 = math.sqrt(0.5)
DIRECTIONS = {"+x": (1.0, 0.0), "+x-z": (S2, -S2), "-z": (0.0, -1.0), "-x": (-1.0, 0.0)}  # proximal-face frame
LENGTH_M, MASS_KG = 0.07, 0.05
K_Q, D_Q = 5.0, 0.08
STALL_NM = 1.03  # XM335 stall torque at 11.1 V (the simulation runs without torque limits)
FINGERS = ("index", "middle", "ring")
SLIP_MM_S = (5.0, 10.0, 20.0)
PRIMARY = 1
LOSS_N, COLLAPSE_MM, ROLL_MM = 0.2, 5.0, 1.0
GATE_N, GATE_SPEED_MM_S, GATE_SPIN_RAD_S, GATE_ANGLE_DEG, GATE_MATCH = 0.2, 2.0, 0.1, 2.0, 0.05


def o_targets(mu: float, radius: float) -> dict:
    runs = sorted(glob.glob(str(OUT / f"tp_O_mu{mu:g}_A_r{radius:g}_*.json")))
    if not runs:
        raise FileNotFoundError(f"condition B needs O's condition A run at mu {mu:g}")
    d = json.loads(Path(runs[-1]).read_text())
    return {(e["rho_mm"], e["q0_deg"], e["direction"]): e["pre"]["n_distal_total"] for e in d["envs"]}


def o_torques(mu: float, radius: float) -> dict:
    """Condition C: O's flexion torque per finger at its condition-A grasp [N m], (index, middle, ring) per
    (radius, q0, direction). Exact for the scheduled servo PD at rest: tau_theta = K_q / r(q*)^2 (theta(q_cmd) -
    theta(q*)), tau_q = tau_theta r(q*), with O's transmission theta(q) and r = dtheta/dq at q*."""
    from code_isaaclab.joint_map import flexion_maps

    runs = sorted(glob.glob(str(OUT / f"tp_O_mu{mu:g}_A_r{radius:g}_*.json")))
    runs = [r for r in runs if json.loads(Path(r).read_text()).get("impratio") == 1000.0]
    if not runs:
        raise FileNotFoundError(f"condition C needs O's condition A run at mu {mu:g}, radius {radius:g} mm")
    d = json.loads(Path(runs[-1]).read_text())
    maps = flexion_maps(json.loads(HANDS["O"].read_text()))
    out = {}
    for e in d["envs"]:
        qc = math.radians(e["q_cmd_deg"])
        taus = []
        for f, q_deg in zip(FINGERS, e["pre"]["q_finger_deg"]):
            m, q = maps[f], math.radians(q_deg)
            r = float(m.ratio(q))
            taus.append(K_Q * float(m.servo(qc) - m.servo(q)) / r)
        out[(e["rho_mm"], e["q0_deg"], e["direction"])] = {"tau": taus, "q": list(e["pre"]["q_finger_deg"])}
    return out


def main(argv=None) -> int:
    from isaaclab_tasks.utils import setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hand", required=True,
                        help="O, R, U, the suffix of another 3 -> 7 mm hand (cfl_q120_h3to7_<suffix>.json, e.g. d2p25_s0), "
                             "or h3to<t>_<suffix> for another thickness (e.g. h3to5_hr)")
    parser.add_argument("--mu", type=float, required=True)
    parser.add_argument("--condition", default="A", choices=("A", "B", "C", "D"))
    parser.add_argument("--stiffness_scale", type=float, default=1.0,
                        help="condition D: flexion stiffness as a multiple of K0 = 5 N m/rad (damping scales with its root)")
    parser.add_argument("--delta_deg", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=2.0, help="load ramp [N/s]")
    parser.add_argument("--max_force", type=float, default=80.0)
    parser.add_argument("--grasps", type=Path, default=GRASPS_JSON)
    parser.add_argument("--radius_mm", type=float, default=15.0)
    parser.add_argument("--impratio", type=float, default=None, help="MuJoCo friction-to-normal impedance ratio (default: the preset's 10)")
    parser.add_argument("--render", type=Path, default=None,
                        help="also render environment --render_env with Newton's GL viewer (headless) once the grasp has "
                             "settled, as <dir>/<run>_env<k>.png")
    parser.add_argument("--render_env", type=int, nargs="+", default=(0,))
    parser.add_argument("--render_only", action="store_true", help="stop after rendering (no load ramp, no log)")
    parser.add_argument("--camera", type=float, nargs=5, default=(0.068, -0.145, 0.018, -5.0, 90.0),
                        help="camera offset from the environment origin + hand height (x, y, z [m]), pitch, yaw [deg]")
    parser.add_argument("--verbose", action="store_true", help="print the grasp every 10 steps before the load")
    parser.add_argument("--only", type=int, default=None, help="first N placements (a smoke test)")
    args, hydra_args = setup_preset_cli(parser, argv)
    sys.argv = [sys.argv[0]] + hydra_args

    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.app import launch_simulation
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sensors import ContactSensorCfg
    import isaaclab.sim as sim_utils
    from isaaclab.utils import configclass

    import code_isaaclab.tasks  # noqa: F401
    from code_isaaclab.assets.roller import write_planar_cylinder_usd
    from code_isaaclab.flexion_control import FlexionEMAActionCfg
    from code_isaaclab.hands import get_hand
    from code_isaaclab.hands.code_variants import ASSET_DIR
    from code_isaaclab.hands.description_hands import cylinder_cfg, register_description_hand
    from code_isaaclab.tasks.reorient.code_env_cfg import CoDeReorientMainEnvCfg

    physics = next((t.split("=", 1)[1] for t in hydra_args if t.startswith("physics=")), "newton_mjwarp")
    if args.hand not in HANDS:  # "<suffix>" for a 3 -> 7 mm hand, "h3to<t>_<suffix>" for another thickness
        stem = args.hand if args.hand.startswith("h3to") else f"h3to7_{args.hand}"
        HANDS[args.hand] = HANDS["O"].with_name(f"cfl_q120_{stem}.json")
    desc = json.loads(HANDS[args.hand].read_text())
    face = proximal_face_z(desc)
    face_w = HAND_Z - face  # world height of this hand's proximal contact face (palm up: the object sits above it)
    placements = [g for g in json.loads(args.grasps.read_text())["grasps"] if g["rho_mm"] == args.radius_mm]
    placements = placements[: args.only]
    envs = [(g, name) for g in placements for name in DIRECTIONS]
    targets = o_targets(args.mu, args.radius_mm) if args.condition == "B" else None
    torques = o_torques(args.mu, args.radius_mm) if args.condition in ("C", "D") else None
    results = run_pass(args, physics, face_w, args.radius_mm, envs, targets, torques, ImplicitActuatorCfg, launch_simulation,
                       ArticulationCfg, ContactSensorCfg, sim_utils, configclass, write_planar_cylinder_usd,
                       FlexionEMAActionCfg, get_hand, ASSET_DIR, cylinder_cfg, register_description_hand,
                       CoDeReorientMainEnvCfg)
    return finish(args, physics, results)


def finish(args, physics, results) -> int:
    if not results:  # --render_only
        return 0
    cond = args.condition if args.condition != "D" else f"D{args.stiffness_scale:g}"
    run = common.run_id(f"tp_{args.hand}_mu{args.mu:g}_{cond}_r{args.radius_mm:g}")
    out = common.write_json(OUT / f"{run}.json", {
        "hand": args.hand, "mu": args.mu, "condition": cond, "stiffness_scale": args.stiffness_scale,
        "delta_deg": args.delta_deg,
        "rate_n_s": args.rate, "max_force_n": args.max_force, "flexion_stiffness": K_Q, "flexion_damping": D_Q,
        "impratio": args.impratio, "slip_mm_s": SLIP_MM_S, "primary_slip_mm_s": SLIP_MM_S[PRIMARY], "physics": physics,
        "gate": {"pad_n": GATE_N, "speed_mm_s": GATE_SPEED_MM_S, "spin_rad_s": GATE_SPIN_RAD_S,
                 "angle_deg": GATE_ANGLE_DEG, "match": GATE_MATCH},
        "envs": results, "manifest": common.manifest(command=sys.argv)})
    ok = [e for e in results if e["gate0"]["pass"]]
    print(f"Gate 0 passed {len(ok)}/{len(results)}")
    for e in results[:: len(DIRECTIONS)]:
        g = e["gate0"]
        print(f"rho {e['rho_mm']:g} q0 {e['q0_deg']:5.1f}: q* {np.mean(e['pre']['q_star_deg']):6.2f} "
              f"N_p {e['pre']['n_proximal_total']:6.2f} N_d {e['pre']['n_distal_total']:6.2f} gate {g['pass']} "
              f"(pads {g['pads']}, still {g['still']}, angle {g['angle']}) "
              + " ".join(f"{x['direction']}:{x['limit']['f_limit_n']}/{x['limit']['event']}"
                         for x in results if x['rho_mm'] == e['rho_mm'] and x['q0_deg'] == e['q0_deg']))
    print(out)
    return 0


def render(args, u, origins, face_w) -> None:
    """Newton's GL viewer, headless (EGL), on the current state; one PNG per requested environment."""
    import os

    import pyglet

    if not os.environ.get("DISPLAY"):
        pyglet.options["headless"] = True
    import newton
    import warp as wp
    from PIL import Image

    manager = u.sim.physics_manager
    model = manager.get_model()
    colors = model.shape_color.numpy()
    for k, label in enumerate(model.shape_label):  # pads amber, the cylinder blue, the rest light grey
        colors[k] = (0.93, 0.66, 0.22) if label.endswith("_pad") else (0.35, 0.55, 0.85) if "/Cyl/" in label \
            else (0.78, 0.78, 0.80)
    model.shape_color.assign(colors)
    if model.shape_opacity is not None:  # hide the palm's placeholder capsule and the thumb
        opacity = model.shape_opacity.numpy()
        for k, label in enumerate(model.shape_label):
            if "/Robot/palm" in label or "/Robot/thumb" in label:
                opacity[k] = 0.0
        model.shape_opacity.assign(opacity)
    viewer = newton.viewer.ViewerGL(width=1600, height=1100, headless=True)
    viewer.set_model(model)
    viewer.renderer.draw_sky = False  # white background for figures
    viewer.renderer.sky_upper = viewer.renderer.sky_lower = (1.0, 1.0, 1.0)
    viewer.show_collision = True
    viewer.show_ground = False
    args.render.mkdir(parents=True, exist_ok=True)
    stem = f"tp_{args.hand}_mu{args.mu:g}_{args.condition}_r{args.radius_mm:g}"
    viewer.set_world_offsets((0.0, 0.0, 0.0))  # the bodies already carry their environment's origin
    for k in args.render_env:
        viewer.set_visible_worlds([k])
        o = common.torch_of(origins)[k].tolist()
        dx, dy, dz, pitch, yaw = args.camera
        viewer.set_camera(pos=wp.vec3(o[0] + dx, o[1] + dy, face_w + dz), pitch=pitch, yaw=yaw)
        for _ in range(2):  # the first frame after a camera move can be stale
            viewer.begin_frame(0.0)
            viewer.log_state(manager.get_state_0())
            viewer.end_frame()
        image = viewer.get_frame().numpy()
        path = args.render / f"{stem}_env{k}.png"
        Image.fromarray(image).save(path)
        print(f"rendered {path}")
    viewer.close()


def run_pass(args, physics, face_w, rho_mm, envs, targets, torques, ImplicitActuatorCfg, launch_simulation,
             ArticulationCfg, ContactSensorCfg, sim_utils, configclass, write_planar_cylinder_usd, FlexionEMAActionCfg,
             get_hand, ASSET_DIR, cylinder_cfg, register_description_hand, CoDeReorientMainEnvCfg) -> list[dict]:
    rho = rho_mm * 1e-3
    usd = ASSET_DIR / f"planar_cylinder_r{rho_mm:g}.usda"
    if not usd.exists():
        write_planar_cylinder_usd(usd, rho, LENGTH_M, MASS_KG)
    name = register_description_hand(f"tp_{args.hand}_mu{args.mu:g}", HANDS[args.hand],
                                     cylinder_cfg(0.01, 0.02, 0.01, (0.3, 0.3, 0.1), kinematic=True),
                                     pad_friction=args.mu)

    @configclass
    class Cfg(CoDeReorientMainEnvCfg):
        hand: str = name

    env_cfg = Cfg()
    env_cfg.sim.physics = getattr(env_cfg.sim.physics, physics)
    env_cfg.sim.gravity = (0.0, 0.0, 0.0)
    if args.impratio is not None:
        env_cfg.sim.physics.solver_cfg.impratio = args.impratio
    env_cfg.scene.num_envs = len(envs)
    env_cfg.events.reset_object = None
    env_cfg.events.reset_hand.params["joint_position_noise"] = 0.0
    env_cfg.terminations.object_out_of_reach = None
    env_cfg.episode_length_s = 1.0e4
    old = env_cfg.actions.joint_pos
    env_cfg.actions.joint_pos = FlexionEMAActionCfg(asset_name="robot", joint_names=old.joint_names, alpha=1.0,
                                                    four_bar_tables=get_hand(name).four_bar_tables,
                                                    flexion_limits_deg=(0.0, 120.0), flexion_stiffness=K_Q,
                                                    flexion_damping=D_Q)
    env_cfg.scene.cyl = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Cyl",
        spawn=sim_utils.UsdFileCfg(usd_path=str(usd)),
        init_state=ArticulationCfg.InitialStateCfg(pos=(MCP_X_M, -FINGER_Y_M, face_w),
                                                   joint_pos={"slide_x": 0.0, "slide_z": 0.0, "spin": 0.0}),
        actuators={"slides": ImplicitActuatorCfg(joint_names_expr=["slide_x", "slide_z"], stiffness=1.0e5,
                                                 damping=1.0e3, effort_limit_sim=1.0e4),
                   "spin": ImplicitActuatorCfg(joint_names_expr=["spin"], stiffness=10.0, damping=0.1,
                                               effort_limit_sim=100.0)},
    )
    bodies = common.hand_bodies()
    env_cfg.scene.cyl_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Cyl/roller", track_contact_points=True, track_friction_forces=True,
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{b}" for b in bodies])
    args.device = env_cfg.sim.device

    out = []
    with launch_simulation(env_cfg, args), contextlib.ExitStack() as cleanup:
        env = gym.make("CoDe-Reorient-Cube-CoDeFlat-Main", cfg=env_cfg)
        cleanup.callback(env.close)
        env.reset(seed=0)
        u = env.unwrapped
        n, dev = u.num_envs, u.device
        cyl, robot = u.scene["cyl"], u.scene["robot"]
        sensor = u.scene.sensors["cyl_contacts"]
        origins = u.scene.env_origins
        jx, jz, js = (cyl.find_joints(j)[0][0] for j in ("slide_x", "slide_z", "spin"))
        rb = cyl.find_bodies("roller")[0][0]
        ids = torch.arange(n, device=dev)
        x_c = torch.tensor([g["x_c_mm"] * 1e-3 for g, _ in envs], device=dev)
        pos = common.torch_of(cyl.data.joint_pos).clone()
        pos[:, jx], pos[:, jz], pos[:, js] = x_c, rho, 0.0
        cyl.write_joint_position_to_sim_index(position=pos, env_ids=ids)
        cyl.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(pos), env_ids=ids)
        cyl.set_joint_position_target_index(target=pos.clone())
        term = u.action_manager.get_term("joint_pos")
        names = list(term._joint_names)
        lo, hi = term._cmd_limits[0, :, 0], term._cmd_limits[0, :, 1]
        pip = [k for k, nm in enumerate(names) if nm.endswith("_pip_servo") and not nm.startswith("thumb")]
        pip_fingers = [names[k].split("_")[0] for k in pip]
        servo_ids = [robot.find_joints(names[k])[0][0] for k in pip]
        cols = {f: (bodies.index(f"{f}_proximal"), bodies.index(f"{f}_distal")) for f in FINGERS}
        pad_cols = [c for f in FINGERS for c in cols[f]]
        other_cols = [k for k in range(len(bodies)) if k not in pad_cols]
        pad_body_ids = [robot.find_bodies(f"{f}_{s}")[0][0] for f in FINGERS for s in ("proximal", "distal")]
        d_face = torch.tensor([DIRECTIONS[d] for _, d in envs], device=dev)
        d_world = torch.stack((d_face[:, 0], -d_face[:, 1]), dim=1)  # world x, z of the load (face z is world -z)

        def command(q_deg: torch.Tensor) -> torch.Tensor:  # (n,) the same for every finger, or (n, fingers)
            cmd = torch.zeros(n, len(names), device=dev)
            for i, k in enumerate(pip):
                cmd[:, k] = torch.deg2rad(q_deg if q_deg.dim() == 1 else q_deg[:, i])
            return 2.0 * (cmd - lo) / (hi - lo) - 1.0

        def load(f: float | torch.Tensor):
            f = torch.as_tensor(f, device=dev, dtype=torch.float32).expand(n)
            cyl.set_joint_effort_target_index(target=torch.stack((f * d_world[:, 0], f * d_world[:, 1]), dim=1),
                                              joint_ids=[jx, jz])

        def to_face(v: torch.Tensor) -> torch.Tensor:  # (..., 3) world vector -> (..., 2) proximal-face frame
            return torch.stack((v[..., 0], -v[..., 2]), dim=-1)

        def face_point(p: torch.Tensor) -> torch.Tensor:  # env-local world point -> proximal-face frame
            return torch.stack((p[..., 0] - MCP_X_M, face_w - p[..., 2]), dim=-1)

        def state() -> dict:
            normal = common.torch_of(sensor.data.normal_force_matrix_w)[:, 0]  # (n, B, 3), force on the cylinder
            friction = common.torch_of(sensor.data.friction_force_matrix_w)[:, 0]
            point = common.torch_of(sensor.data.contact_pos_w)[:, 0] - origins[:, None]
            total = common.torch_of(sensor.data.force_matrix_w)[:, 0]
            c = common.torch_of(cyl.data.body_link_pos_w)[:, rb] - origins
            v = common.torch_of(cyl.data.body_link_lin_vel_w)[:, rb]
            w = common.torch_of(cyl.data.body_link_ang_vel_w)[:, rb]
            pb = common.torch_of(robot.data.body_link_pos_w)[:, pad_body_ids] - origins[:, None]
            vb = common.torch_of(robot.data.body_link_lin_vel_w)[:, pad_body_ids]
            wb = common.torch_of(robot.data.body_link_ang_vel_w)[:, pad_body_ids]
            nrm, fr, pt = normal[:, pad_cols], friction[:, pad_cols], point[:, pad_cols]
            mag = nrm.norm(dim=-1)
            touching = (mag > 1e-6) & torch.isfinite(pt).all(dim=-1)
            p_safe = torch.where(touching[..., None], pt, pb)
            v_obj = v[:, None] + torch.cross(w[:, None].expand_as(p_safe), p_safe - c[:, None], dim=-1)
            v_pad = vb + torch.cross(wb, p_safe - pb, dim=-1)
            rel = v_obj - v_pad
            unit = nrm / mag.clamp_min(1e-9)[..., None]
            tang = rel - (rel * unit).sum(-1, keepdim=True) * unit
            slip = torch.where(touching, tang.norm(dim=-1), torch.zeros_like(mag))
            theta = common.torch_of(robot.data.joint_pos)[:, servo_ids]
            q_star = torch.stack([term._tables.flexion(names[k], theta[:, i]) for i, k in enumerate(pip)], dim=1)
            return {"normal": nrm, "friction": fr, "point": pt, "mag": mag, "touching": touching, "slip": slip,
                    "centre": c, "v": v, "w": w, "q_star": q_star,
                    "other_n": total[:, other_cols].norm(dim=-1).sum(dim=1)}

        q_cmd = torch.tensor([g["q0_deg"] + args.delta_deg for g, _ in envs], device=dev)
        tau_scale = [0.0]
        tau_q = None if torques is None else torch.tensor(
            [[torques[(g["rho_mm"], g["q0_deg"], d)]["tau"][FINGERS.index(f)] for f in pip_fingers] for g, d in envs],
            device=dev)
        q_ref = None if args.condition != "D" else torch.tensor(  # condition D: O's grasp flexion per finger [deg]
            [[torques[(g["rho_mm"], g["q0_deg"], d)]["q"][FINGERS.index(f)] for f in pip_fingers] for g, d in envs],
            device=dev)

        def servo_torque_share() -> torch.Tensor:  # static servo torque over the XM335 stall torque, per finger
            theta = common.torch_of(robot.data.joint_pos)[:, servo_ids]
            q_now = torch.stack([term._tables.flexion(names[k], theta[:, i]) for i, k in enumerate(pip)], dim=1)
            r = torch.stack([term._tables.ratio(names[k], q_now[:, i]) for i, k in enumerate(pip)], dim=1)
            target = torch.deg2rad(q_cmd if q_cmd.dim() == 2 else q_cmd[:, None].expand_as(q_now))
            tau = float(term.cfg.flexion_stiffness or 0.0) * (target - q_now)
            if tau_q is not None:
                tau = tau + tau_scale[0] * tau_q
            return (tau / r).abs() / STALL_NM

        def step(action):  # condition C: the PIP servos also get O's flexion torque through this hand's own ratio
            if tau_q is not None and tau_scale[0] > 0.0:
                theta = common.torch_of(robot.data.joint_pos)[:, servo_ids]
                r = torch.stack([term._tables.ratio(names[k], term._tables.flexion(names[k], theta[:, i]))
                                 for i, k in enumerate(pip)], dim=1)
                robot.set_joint_effort_target_index(target=(tau_scale[0] * tau_q / r).contiguous(),
                                                    joint_ids=servo_ids)
            env.step(action)

        tick = [0]

        def report(phase: str):
            tick[0] += 1
            if args.verbose and tick[0] % 10 == 0:
                s_ = state()
                c_ = 1e3 * face_point(s_["centre"])
                print(phase, tick[0], " | ".join(
                    f"q* {math.degrees(float(s_['q_star'][e].mean())):6.2f} Np {float(s_['mag'][e, 0::2].sum()):6.2f} "
                    f"Nd {float(s_['mag'][e, 1::2].sum()):6.2f} c ({float(c_[e, 0]):6.2f},{float(c_[e, 1]):6.2f}) "
                    f"slip {1e3 * float(s_['slip'][e].max()):6.2f}" for e in range(0, n, len(DIRECTIONS))))
        for i in range(40):  # 1: close on the held cylinder
            load(0.0)
            step(command(q_cmd * (i + 1) / 40))
            report("close")
        for _ in range(30):
            load(0.0)
            step(command(q_cmd))
            report("hold")
        if tau_q is not None:  # C: hand over to O's torque; D: to O's impedance; the cylinder still held
            k1 = 0.0 if args.condition == "C" else K_Q * args.stiffness_scale
            d1 = D_Q if args.condition == "C" else D_Q * math.sqrt(args.stiffness_scale)
            q_start = q_cmd[:, None].expand(-1, len(pip)).clone()
            for i in range(30):
                s_ = (i + 1) / 30
                term.cfg.flexion_stiffness = K_Q + (k1 - K_Q) * s_
                term.cfg.flexion_damping = D_Q + (d1 - D_Q) * s_
                tau_scale[0] = s_
                if q_ref is not None:
                    q_cmd = q_start + (q_ref - q_start) * s_
                load(0.0)
                step(command(q_cmd))
                report("switch")
            for _ in range(30):
                load(0.0)
                step(command(q_cmd))
                report("torque")
        for i in range(30):  # release the drives
            s = 1.0 - (i + 1) / 30
            cyl.write_joint_stiffness_to_sim_index(stiffness=torch.full((n, 2), 1.0e5 * s, device=dev),
                                                   joint_ids=[jx, jz])
            cyl.write_joint_damping_to_sim_index(damping=torch.full((n, 2), 1.0e3 * s + 0.5 * (1 - s), device=dev),
                                                 joint_ids=[jx, jz])
            cyl.write_joint_stiffness_to_sim_index(stiffness=torch.full((n, 1), 10.0 * s, device=dev), joint_ids=[js])
            cyl.write_joint_damping_to_sim_index(damping=torch.full((n, 1), 0.1 * s + 1e-4, device=dev), joint_ids=[js])
            load(0.0)
            step(command(q_cmd))
            report("release")

        def settle(steps: int, feedback: torch.Tensor | None = None) -> tuple[dict, torch.Tensor, torch.Tensor]:
            nonlocal q_cmd
            speed = torch.zeros(n, device=dev)
            spin = torch.zeros(n, device=dev)
            for k in range(steps):
                load(0.0)
                step(command(q_cmd))
                report("settle")
                st = state()
                if feedback is not None:
                    nd = st["mag"][:, 1::2].sum(dim=1)
                    q_cmd = torch.clamp(q_cmd + 0.1 * (feedback - nd), 30.0, 120.0)
                if k >= steps - 10:
                    speed = torch.maximum(speed, 1e3 * st["v"][:, [0, 2]].norm(dim=-1))
                    spin = torch.maximum(spin, st["w"][:, 1].abs())
            return st, speed, spin

        st_rel, speed_rel, spin_rel = settle(90)  # 2: Gate 0 at release
        if targets is not None:  # 3: match O's distal force
            goal = torch.tensor([targets[(g["rho_mm"], g["q0_deg"], d)] for g, d in envs], device=dev)
            settle(120, feedback=goal)
            st, speed, spin = settle(30)
        else:
            goal = None
            st, speed, spin = st_rel, speed_rel, spin_rel

        def gate(s: dict, sp: torch.Tensor, sn: torch.Tensor) -> dict:
            pads_ok = (s["mag"] >= GATE_N).all(dim=1)
            still = (sp < GATE_SPEED_MM_S) & (sn < GATE_SPIN_RAD_S)
            nf = to_face(s["normal"])  # (n, 6, 2)
            unit = nf / nf.norm(dim=-1, keepdim=True).clamp_min(1e-9)
            cosang = (unit[:, 0::2] * unit[:, 1::2]).sum(-1).clamp(-1, 1)  # per finger: proximal . distal
            ang = torch.rad2deg(torch.acos(cosang))
            qf = torch.rad2deg(torch.stack([s["q_star"][:, pip_fingers.index(f)] for f in FINGERS], dim=1))
            angle_ok = ((ang - qf).abs() <= GATE_ANGLE_DEG).all(dim=1) & pads_ok
            return {"pads": pads_ok, "still": still, "angle": angle_ok, "normal_angle_deg": ang, "q_finger_deg": qf}

        g_rel = gate(st_rel, speed_rel, spin_rel)
        g_pre = gate(st, speed, spin)
        matched = torch.ones(n, dtype=torch.bool, device=dev) if goal is None else \
            ((st["mag"][:, 1::2].sum(dim=1) - goal).abs() <= GATE_MATCH * goal)
        passed = g_pre["pads"] & g_pre["still"] & g_pre["angle"] & matched

        if args.render is not None:
            render(args, u, origins, face_w)
            if args.render_only:
                return []

        # 4: load ramp
        share_pre = servo_torque_share()
        share_max = share_pre.max(dim=1).values.clone()
        c0 = st["centre"].clone()
        inf = torch.full((n,), float("nan"), device=dev)
        f_slip = [inf.clone() for _ in SLIP_MM_S]
        f_loss, f_collapse, f_roll = inf.clone(), inf.clone(), inf.clone()
        mu_eff = inf.clone()
        slip_pad = torch.full((n,), -1, device=dev, dtype=torch.long)
        trace = []
        dt = u.step_dt
        steps = int(args.max_force / (args.rate * dt)) + 1
        for i in range(steps):
            f = args.rate * dt * (i + 1)
            load(f)
            step(command(q_cmd))
            s = state()
            share_max = torch.maximum(share_max, servo_torque_share().max(dim=1).values)
            disp = 1e3 * (s["centre"] - c0)[:, [0, 2]].norm(dim=-1)
            vmax, pad = s["slip"].max(dim=1)
            vmax_mm = 1e3 * vmax
            for k, thr in enumerate(SLIP_MM_S):
                hit = torch.isnan(f_slip[k]) & (vmax_mm > thr)
                if k == PRIMARY:
                    sel = s["mag"].gather(1, pad[:, None])[:, 0].clamp_min(1e-9)
                    ratio = s["friction"].norm(dim=-1).gather(1, pad[:, None])[:, 0] / sel
                    mu_eff = torch.where(hit, ratio, mu_eff)
                    slip_pad = torch.where(hit, pad, slip_pad)
                f_slip[k] = torch.where(hit, torch.full_like(inf, f), f_slip[k])
            lost = (s["mag"][:, 0::2].sum(dim=1) < LOSS_N) | (s["mag"][:, 1::2].sum(dim=1) < LOSS_N)
            f_loss = torch.where(torch.isnan(f_loss) & lost, torch.full_like(inf, f), f_loss)
            f_collapse = torch.where(torch.isnan(f_collapse) & (disp > COLLAPSE_MM), torch.full_like(inf, f), f_collapse)
            f_roll = torch.where(torch.isnan(f_roll) & torch.isnan(f_slip[PRIMARY]) & (disp > ROLL_MM),
                                 torch.full_like(inf, f), f_roll)
            if i % 5 == 0:
                trace.append([round(f, 3)] + [[round(float(x), 3) for x in row] for row in torch.stack(
                    (disp, vmax_mm, s["mag"][:, 0::2].sum(dim=1), s["mag"][:, 1::2].sum(dim=1)), dim=1).cpu()])
            done = ~(torch.isnan(f_slip[PRIMARY]) & torch.isnan(f_loss) & torch.isnan(f_collapse))
            if bool(done.all()):
                break

        def val(x) -> float | None:
            x = float(x)
            return None if math.isnan(x) else x

        def pads_record(s: dict, e: int) -> list[dict]:
            c = face_point(s["centre"][e])
            rec = []
            for j, (f, side) in enumerate((f, side) for f in FINGERS for side in ("proximal", "distal")):
                p = face_point(s["point"][e, j]) if bool(s["touching"][e, j]) else None
                rec.append({"pad": f"{f}_{side}", "n_n": float(s["mag"][e, j]),
                            "normal_face": to_face(s["normal"][e, j]).tolist(),
                            "friction_face": to_face(s["friction"][e, j]).tolist(),
                            "point_face_mm": None if p is None else (1e3 * p).tolist(),
                            "point_rel_centre_mm": None if p is None else (1e3 * (p - c)).tolist()})
            return rec

        for e, (g, dname) in enumerate(envs):
            events = {"slip": val(f_slip[PRIMARY][e]), "loss": val(f_loss[e]), "collapse": val(f_collapse[e])}
            hit = {k: v for k, v in events.items() if v is not None}
            first = min(hit, key=hit.get) if hit else None
            out.append({
                "rho_mm": g["rho_mm"], "q0_deg": g["q0_deg"], "x_c_mm": g["x_c_mm"], "direction": dname,
                "q_star_predicted_deg": g.get(args.hand), "q_cmd_deg": float(q_cmd[e].float().mean()),
                "target_n_distal": None if goal is None else float(goal[e]),
                "target_tau_q_nm": None if tau_q is None else tau_q[e].tolist(),
                "q_ref_deg": None if q_ref is None else q_ref[e].tolist(),
                "servo_torque_share_pre": share_pre[e].tolist(), "servo_torque_share_max": float(share_max[e]),
                "release": {"q_star_deg": np.degrees(st_rel["q_star"][e].cpu().numpy()).tolist(),
                            "n_proximal_total": float(st_rel["mag"][e, 0::2].sum()),
                            "n_distal_total": float(st_rel["mag"][e, 1::2].sum()),
                            "centre_face_mm": (1e3 * face_point(st_rel["centre"][e])).tolist(),
                            "pads": bool(g_rel["pads"][e]), "still": bool(speed_rel[e] < GATE_SPEED_MM_S),
                            "angle": bool(g_rel["angle"][e])},
                "pre": {"q_star_deg": np.degrees(st["q_star"][e].cpu().numpy()).tolist(),
                        "q_finger_deg": g_pre["q_finger_deg"][e].tolist(),
                        "normal_angle_deg": g_pre["normal_angle_deg"][e].tolist(),
                        "n_proximal_total": float(st["mag"][e, 0::2].sum()),
                        "n_distal_total": float(st["mag"][e, 1::2].sum()),
                        "other_contact_n": float(st["other_n"][e]),
                        "centre_face_mm": (1e3 * face_point(st["centre"][e])).tolist(),
                        "centre_offset_mm": (1e3 * (face_point(st["centre"][e]) - torch.tensor(
                            [g["x_c_mm"] * 1e-3, -rho], device=dev))).tolist(),
                        "speed_mm_s": float(speed[e]), "spin_rad_s": float(spin[e]),
                        "pads": pads_record(st, e)},
                "gate0": {"pass": bool(passed[e]), "pads": bool(g_pre["pads"][e]), "still": bool(g_pre["still"][e]),
                          "angle": bool(g_pre["angle"][e]), "matched": bool(matched[e])},
                "limit": {"f_limit_n": None if first is None else hit[first], "event": first, **events,
                          "slip_by_threshold": {f"{t:g}": val(f_slip[k][e]) for k, t in enumerate(SLIP_MM_S)},
                          "roll_n": val(f_roll[e]), "mu_eff_at_slip": val(mu_eff[e]),
                          "slip_pad": None if int(slip_pad[e]) < 0 else
                          [f"{f}_{s}" for f in FINGERS for s in ("proximal", "distal")][int(slip_pad[e])]},
                "trace": [[row[0], *row[1 + e]] for row in trace],  # load, displacement, max slip, N_p, N_d
            })
    return out


if __name__ == "__main__":
    raise SystemExit(main())
