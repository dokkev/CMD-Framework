# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Where the PIP servo colliders (``assets.builders.write_usd``'s ``servo_colliders``) overlap other colliders.

    uv run python scripts/rotate_cube/check_servo_colliders.py [--hand O] [--cache <dir>/O.pt ...] [--out <json>]

Pure geometry from the description (no simulator): every collider as a convex hull in the hand frame (capsules from
surface samples, pads as their boxes, a superset of any crowned or domed pad, the palm housing hulls, the PIP servo
bodies from ``servo_mesh``), posed by forward kinematics of the description's joints (the four-bars closed from the
export's tables). For a pair of hulls, the linear program ``max t s.t. n_i . x + d_i <= -t`` over both hulls' facets
gives ``t > 0`` when they overlap (``t`` is the depth of the deepest common point, at most half the penetration
across a slab) and ``t < 0`` when they are apart.

Checked, for each finger's PIP servo collider:

- its own finger's links (mcp, servo link, driver link, distal) over MCP flexion -10..100 deg and PIP flexion
  0, 60, 120 deg: which to filter;
- the palm housing over the same range (the description already filters the palm against every proximal link);
- the neighbouring fingers at every abduction pair in {-30, 0, 30} deg and MCP/PIP at the grasp and the extremes;
- the thumb, and the cube, at the rest grasp and in every state of the grasp caches given.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

from code_isaaclab.assets.builders import housing_collider, servo_mesh
from code_isaaclab.assets.description import Description
from code_isaaclab.hands.code_variants import HAND_ROOT_POS
from code_isaaclab.hands.description_hands import with_thumb_pad
from code_isaaclab.tasks.rotate_cube.grasp import CUBE_SIZE_M, GRASP_MCP_DEG, GRASP_PIP_DEG, HANDS, pinch_grasp

FINGERS = ("index", "middle", "ring")
OWN_LINKS = ("mcp", "servo_link", "driver_link", "distal")


def _quat_wxyz(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def _rot(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


class Hand:
    """Collider point sets per body (hand frame at the geometric zero) and forward kinematics."""

    def __init__(self, desc: Description):
        self.desc = desc
        self.shapes: dict[str, list[tuple[str, np.ndarray]]] = {}
        for body in desc.bodies:
            name, items = body["name"], []
            origin = np.asarray(body["origin"], float)
            if body.get("housing"):
                for box in body["housing"]:
                    verts, _ = housing_collider(box, origin)
                    items.append((box["name"], verts + origin))
            else:
                cap = body["capsule"]
                s, e, r = np.asarray(cap["start"], float), np.asarray(cap["end"], float), float(cap["radius"])
                sphere = np.array([[math.cos(a) * math.sin(b), math.sin(a) * math.sin(b), math.cos(b)]
                                   for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)
                                   for b in np.linspace(0, math.pi, 7)])  # fmt: skip
                items.append(("capsule", np.vstack([s + r * sphere, e + r * sphere])))
            for box in body.get("boxes", []):
                rot, h = _quat_wxyz(box["quat_wxyz"]), np.asarray(box["half_extents"], float)
                corners = np.array(list(itertools.product((-1, 1), repeat=3))) * h
                items.append((box["name"], np.asarray(box["center"], float) + corners @ rot.T))
            self.shapes[name] = items
        sv, _ = servo_mesh(desc.servo["shape"])
        for item in desc.data.get("visuals") or []:
            if item.get("shape") == "servo" and item["servo"].endswith("/pip"):
                pts = np.asarray(item["position"], float) + sv @ _quat_wxyz(item["quat_wxyz"]).T
                self.shapes[item["body"]].append((item["name"] + "_collider", pts))
        self.joints = {j["name"]: j for j in desc.joints}
        self.children = {}
        for j in desc.joints:
            self.children.setdefault(j["parent"], []).append(j)

    def fk(self, deg: dict[str, float]) -> dict[str, np.ndarray]:
        out = {"palm": np.eye(4)}
        stack = ["palm"]
        while stack:
            parent = stack.pop()
            for j in self.children.get(parent, []):
                m = np.eye(4)
                r, p = _rot(j["axis"], math.radians(deg.get(j["name"], 0.0))), np.asarray(j["anchor"], float)
                m[:3, :3], m[:3, 3] = r, p - r @ p
                out[j["child"]] = out[parent] @ m
                stack.append(j["child"])
        return out

    def pose(self, mcp: dict, pip: dict, aa: dict | None = None, thumb: dict | None = None) -> dict[str, float]:
        deg = {}
        for f in FINGERS:
            deg[f"{f}_fe"] = mcp.get(f, GRASP_MCP_DEG)
            deg[f"{f}_aa"] = (aa or {}).get(f, 0.0)
            deg.update(self.desc.closed_pose(f, pip.get(f, GRASP_PIP_DEG)))
        deg.update(thumb or {})
        return deg

    def posed(self, tf: dict[str, np.ndarray], body: str) -> list[tuple[str, np.ndarray]]:
        m = tf[body]
        return [(n, pts @ m[:3, :3].T + m[:3, 3]) for n, pts in self.shapes[body]]


_HULLS: dict[int, tuple] = {}


def overlap(a: np.ndarray, b: np.ndarray) -> float:
    """``t`` [m]: > 0 overlap depth, < 0 apart (the LP of the module docstring)."""
    ha, hb = ConvexHull(a).equations, ConvexHull(b).equations
    eq = np.vstack([ha, hb])
    a_ub = np.hstack([eq[:, :3], np.ones((len(eq), 1))])
    res = linprog(c=[0, 0, 0, -1], A_ub=a_ub, b_ub=-eq[:, 3], bounds=[(None, None)] * 3 + [(None, 0.05)])
    return float(res.x[3]) if res.success else float("nan")


def servo_of(hand: Hand, tf, finger) -> np.ndarray:
    return next(p for n, p in hand.posed(tf, f"{finger}_proximal") if n.endswith("_collider"))


def cube_points(pos_env, quat_xyzw) -> np.ndarray:
    """The cube's corners in the hand frame (the env frame is palm up: half a turn about x at HAND_ROOT_POS)."""
    x, y, z, w = quat_xyzw
    r = _quat_wxyz((w, x, y, z))
    corners = np.array(list(itertools.product((-1, 1), repeat=3))) * CUBE_SIZE_M / 2
    env = np.asarray(pos_env, float) + corners @ r.T
    flip = np.diag([1.0, -1.0, -1.0])
    return (env - np.asarray(HAND_ROOT_POS)) @ flip.T


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hand", default="O")
    parser.add_argument("--cache", type=Path, action="append", default=[])
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    desc = with_thumb_pad(Description.load(HANDS[args.hand]))
    hand = Hand(desc)
    g = pinch_grasp(desc)
    thumb = {"thumb_ro": -90.0, "thumb_raa": 20.0, "thumb_mcp": g["thumb_mcp_deg"], "thumb_ip": g["thumb_ip_deg"]}
    report: dict = {"own_links": {}, "palm": {}, "neighbours": {}, "thumb_and_cube": {}}

    # own links and palm over the MCP and PIP ranges
    for f in FINGERS:
        worst = {link: -1.0 for link in OWN_LINKS}
        worst_palm = -1.0
        for mcp in (-10.0, 0.0, 35.0, 60.0, 80.0, 100.0):
            for pip in (0.0, 30.0, 60.0, 90.0, 120.0):
                tf = hand.fk(hand.pose({f: mcp}, {f: pip}, thumb=thumb))
                servo = servo_of(hand, tf, f)
                for link in OWN_LINKS:
                    for _, pts in hand.posed(tf, f"{f}_{link}"):
                        worst[link] = max(worst[link], overlap(servo, pts))
                for _, pts in hand.posed(tf, "palm"):
                    worst_palm = max(worst_palm, overlap(servo, pts))
        report["own_links"][f] = {k: round(1e3 * v, 2) for k, v in worst.items()}
        report["palm"][f] = round(1e3 * worst_palm, 2)

    # neighbouring fingers: abduction pairs, MCP / PIP at the grasp and the extremes
    for f, n in (("index", "middle"), ("middle", "ring"), ("middle", "index"), ("ring", "middle")):
        worst = -1.0
        for af, an in itertools.product((-30.0, 0.0, 30.0), repeat=2):
            for mcp_f, mcp_n in itertools.product((0.0, 35.0, 100.0), repeat=2):
                for pip_f, pip_n in itertools.product((0.0, 90.0, 120.0), repeat=2):
                    tf = hand.fk(hand.pose({f: mcp_f, n: mcp_n}, {f: pip_f, n: pip_n}, {f: af, n: an}, thumb))
                    servo = servo_of(hand, tf, f)
                    for link in ("mcp", "proximal", "servo_link", "driver_link", "distal"):
                        for _, pts in hand.posed(tf, f"{n}_{link}"):
                            worst = max(worst, overlap(servo, pts))
        report["neighbours"][f"{f}_servo_vs_{n}"] = round(1e3 * worst, 2)

    # thumb and cube at the rest grasp, then over the caches
    def thumb_cube(tf, pos, quat) -> tuple[float, float]:
        cube = cube_points(pos, quat)
        thumb_bodies = ("thumb_cmc", "thumb_metacarpal", "thumb_proximal", "thumb_distal")
        tb = [p for b in thumb_bodies for _, p in hand.posed(tf, b)]
        worst_t, worst_c = -1.0, -1.0
        for f in FINGERS:
            servo = servo_of(hand, tf, f)
            worst_c = max(worst_c, overlap(servo, cube))
            for p in tb:
                worst_t = max(worst_t, overlap(servo, p))
        return worst_t, worst_c

    tf = hand.fk(hand.pose({}, {}, thumb=thumb))
    t, c = thumb_cube(tf, g["rest_env"], g["rest_quat_xyzw"])
    report["thumb_and_cube"]["rest_grasp"] = {"thumb_mm": round(1e3 * t, 2), "cube_mm": round(1e3 * c, 2)}
    for path in args.cache:
        import torch

        data = torch.load(path, map_location="cpu", weights_only=True)
        names = list(data["joint_names"])
        q = np.degrees(data["q"].numpy())
        ts, cs = [], []
        for k in range(len(q)):
            tf = hand.fk(dict(zip(names, q[k], strict=True)))
            t, c = thumb_cube(tf, data["pos"][k].numpy(), data["quat"][k].numpy())
            ts.append(t)
            cs.append(c)
        ts, cs = np.array(ts), np.array(cs)
        report["thumb_and_cube"][str(path)] = {
            "states": len(q), "thumb_overlap_states": int((ts > 0).sum()), "thumb_max_mm": round(1e3 * ts.max(), 2),
            "cube_overlap_states": int((cs > 0).sum()), "cube_max_mm": round(1e3 * cs.max(), 2),
            "cube_within_1mm_states": int((cs > -1e-3).sum()),
        }  # fmt: skip
    text = json.dumps(report, indent=1)
    print(text)
    if args.out:
        args.out.write_text(text + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
