"""Prototype thumb (O) against the oblique-CMC thumb layout (O_thumbopt) on the Drake tasks where the thumb matters
(docs/experimental/drake_thumbopt.md). The fingers are identical on both hands (checked: the descriptions' finger
joints, bodies, pads, four-bar tables and servo agree exactly), so every planar task module runs with finger "O"; only
the thumb's pose, pad normal and servo compliance change. The task modules are not edited.

- ``pose``: the pre-registered pinch pose rule (``POSE_RULE``, ``BANDS``) on a hand's URDF.
  ``thumb_in_plane`` is ``pinch3_pose.thumb_in_plane`` for any hand and thumb pose (checked against it on O's
  historical pose).
- ``draws33b`` / ``draws36``: the 33b (three-finger pinch rotate) and 36 (pinch peg-in-hole) draws with the hand's
  rule pose (``pinch3.make_draws`` / ``peg_pinch.make_draws`` with ``pinch3_pose.thumb_in_plane`` replaced in this
  process); then ``pinch_rotate.py run`` / ``peg_pinch.py run`` as usual (finger O, a per-hand tag).
- ``reach``: experiment 33's opposing-pad placements against the hand's real thumb (the committed
  ``pinch_thumb_ik.py`` logic, copied and made per-hand: the thumb pad face and normal from the URDF's pad box).

    uv run python thumbopt_drake.py pose --hand O O_thumbopt [--out logs/thumbopt_pose.json]
    uv run python thumbopt_drake.py draws33b --hand O_thumbopt --out logs/pinch3_tho_O_thumbopt.json
    uv run python thumbopt_drake.py draws36 --hand O_thumbopt --out logs/peg_tho_O_thumbopt.json
    uv run python thumbopt_drake.py reach --hand O_thumbopt --out logs/thumbopt_reach_O_thumbopt.json [--seed 0]
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import load_hand as LH

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
THUMB_JOINTS = ("thumb_ro", "thumb_raa", "thumb_mcp", "thumb_ip")
P_I, N_I = np.array([-0.0065, 0.0, -0.006]), np.array([0.0, 0.0, -1.0])  # index distal pad face centre, normal
HIST_POSE_O = {"thumb_ro": -90.0, "thumb_raa": 10.0, "thumb_mcp": 45.0, "thumb_ip": -7.5}
POSE_RULE = {"mid_plane_mm": 3.0, "tangential_mm": 2.0, "clear_mm": 0.0, "fe_deg": (30.0, 100.0, 2.5),
             "grid_deg": 5.0, "refine_deg": 1.0, "refine_half_deg": 5.0, "refine_top": 20}
BANDS = {"33b_small": {"gap_mm": (16.0, 20.0), "lateral": "mid", "task_fe": 72.5, "pip_deg": 0.0},
         "33b_large": {"gap_mm": (20.0, 24.0), "lateral": "mid", "task_fe": 70.0, "pip_deg": 0.0},
         "36": {"gap_mm": (6.0, 9.0), "lateral": "index", "task_fe": 72.5, "pip_deg": 0.0}}
"""The pinch pose rule (pre-registered; the same on every hand). Fingers: index FE free on a 2.5 deg grid over
fe_deg, AA 0, PIP 0 (as pinch3_pose; the tasks then flex or extend the PIP from it), ring curled (FE 95, PIP 100); the middle at the index's FE for 33b (lateral "mid": the thumb faces
the index-middle mid-plane) and curled like the ring for 36 (lateral "index": the thumb faces the index alone). The
thumb within its URDF limits, its pad face centre within mid_plane_mm of that lateral plane, within tangential_mm of the
index distal pad's normal line (in the plane) and at gap_mm (the band's object size: 33b sphere diameters 16-20 mm
with the task's FE-72.5 label and 20-24 mm with FE 70; 36 peg diameters 6-9 mm) from the index pad face along it; its
links clear of the palm and the fingers (signed distance > clear_mm; the thumb's CMC body and the metacarpal-palm pair
excepted, as pinch_thumb_ik.py). Minimise the 3D angle between the thumb pad's outward normal and the index pad's
inward normal: a 5 deg thumb grid at every FE, then a 1 deg grid +-5 deg round the 20 best clear grid poses (at their
FE and FE +-1.25). The planar task models do not depend on FE (the MCP is fixed, gravity off): FE only places the
thumb relative to the finger."""


class HandModel:
    """The hand's URDF in Drake (no collision filters), palm at the world origin, with fast numpy FK of the thumb."""

    def __init__(self, hand: str):
        from pydrake.math import RigidTransform
        from pydrake.multibody.parsing import Parser
        from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
        from pydrake.systems.framework import DiagramBuilder

        self.hand = hand
        urdf, description = LH.paths(hand, visuals=False)
        self.desc = json.loads(description.read_text())
        b = DiagramBuilder()
        self.plant, self.sg = AddMultibodyPlantSceneGraph(b, 0.0)
        self.model = Parser(self.plant).AddModels(str(urdf))[0]
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("palm", self.model),
                              RigidTransform())
        self.plant.Finalize()
        self.diagram = b.Build()
        self.ctx = self.diagram.CreateDefaultContext()
        self.pctx = self.plant.GetMyMutableContextFromRoot(self.ctx)
        self.sctx = self.sg.GetMyContextFromRoot(self.ctx)
        self.J = lambda n: self.plant.GetJointByName(n, self.model)
        self.B = lambda n: self.plant.GetBodyByName(n, self.model)
        insp = self.sg.model_inspector()
        self.body_of = {}
        for gid in insp.GetAllGeometryIds():
            body = self.plant.GetBodyFromFrameId(insp.GetFrameId(gid))
            if body is not None:
                self.body_of[gid] = (body.name(), insp.GetName(gid))
        self.pad_face, self.pad_normal = self._pad()
        self.lo = np.array([self.J(n).position_lower_limits()[0] for n in THUMB_JOINTS])
        self.hi = np.array([self.J(n).position_upper_limits()[0] for n in THUMB_JOINTS])

    def _pad(self):
        """The thumb pad box (thumb_distal's collision box): face centre (m) and outward normal in thumb_distal."""
        insp = self.sg.model_inspector()
        for gid, (body, name) in self.body_of.items():
            shape = insp.GetShape(gid)
            if body == "thumb_distal" and type(shape).__name__ == "Box":
                X = insp.GetPoseInFrame(gid)
                size = np.array([shape.width(), shape.depth(), shape.height()])
                k = int(np.argmin(size))
                n = X.rotation().matrix()[:, k]
                c = X.translation()
                # outward: away from the distal link's centre line (its capsule runs from the IP anchor at the
                # origin to the end effector)
                axis_pt = self._axis_point(c)
                if (c - axis_pt) @ n < 0:
                    n = -n
                return c + 0.5 * size[k] * n, n
        raise RuntimeError("no thumb pad box")

    def _axis_point(self, c):
        insp = self.sg.model_inspector()
        for gid, (body, name) in self.body_of.items():
            shape = insp.GetShape(gid)
            if body == "thumb_distal" and type(shape).__name__ == "Cylinder":
                X = insp.GetPoseInFrame(gid)
                ax = X.rotation().matrix()[:, 2]
                p = X.translation()
                return p + ((c - p) @ ax) * ax
        return np.zeros(3)

    def set_fingers(self, fe_deg: float, pip_deg: float = 0.0, middle: bool = True):
        LH.set_pose(self.plant, self.pctx, self.model, self.desc, pip_deg)
        for d in ("ring",) if middle else ("ring", "middle"):
            for k, v in LH.table(self.desc, d, 100).items():
                self.J(k).set_angle(self.pctx, v)
            self.J(f"{d}_fe").set_angle(self.pctx, math.radians(95))
        for d in ("index", "middle") if middle else ("index",):
            self.J(f"{d}_fe").set_angle(self.pctx, math.radians(fe_deg))
            self.J(f"{d}_aa").set_angle(self.pctx, 0.0)

    def set_thumb(self, deg):
        for n, v in zip(THUMB_JOINTS, deg):
            self.J(n).set_angle(self.pctx, math.radians(float(v)))

    def chain(self):
        """The thumb chain for numpy FK: per joint, the fixed transform parent body -> joint frame (4x4) and the axis
        in the joint frame; the last child is thumb_distal. The palm is the world."""
        out = []
        for n in THUMB_JOINTS:
            j = self.J(n)
            X_PF = j.frame_on_parent().GetFixedPoseInBodyFrame().GetAsMatrix4()
            X_CM = j.frame_on_child().GetFixedPoseInBodyFrame().GetAsMatrix4()
            out.append((X_PF, np.asarray(j.revolute_axis(), float), np.linalg.inv(X_CM)))
        return out

    def fk_batch(self, deg: np.ndarray):
        """Thumb pad face centre (m) and outward normal in the palm frame for N thumb poses (deg, N x 4)."""
        q = np.radians(deg)
        n = len(q)
        T = np.repeat(np.eye(4)[None], n, axis=0)
        for i, (X_PF, a, X_MC) in enumerate(self.chain_cache):
            c, s = np.cos(q[:, i]), np.sin(q[:, i])
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = np.eye(3)[None] + s[:, None, None] * K[None] + (1 - c)[:, None, None] * (K @ K)[None]
            Rj = np.repeat(np.eye(4)[None], n, axis=0)
            Rj[:, :3, :3] = R
            T = T @ X_PF[None] @ Rj @ X_MC[None]
        p = (T[:, :3, :3] @ self.pad_face) + T[:, :3, 3]
        nv = T[:, :3, :3] @ self.pad_normal
        return p, nv

    def clearance(self) -> float:
        """Smallest signed distance (mm) of the thumb's links to the palm and the fingers (the CMC body and the
        metacarpal-palm pair excepted, as pinch_thumb_ik.py)."""
        qo = self.sg.get_query_output_port().Eval(self.sctx)
        best = np.inf
        for pr in qo.ComputeSignedDistancePairwiseClosestPoints(0.03):
            a, b = self.body_of.get(pr.id_A), self.body_of.get(pr.id_B)
            if a is None or b is None:
                continue
            ta, tb = a[0].startswith("thumb"), b[0].startswith("thumb")
            if ta == tb:
                continue
            th, other = (a, b) if ta else (b, a)
            if th[0] == "thumb_cmc" or (th[0] == "thumb_metacarpal" and other[0] == "palm"):
                continue
            best = min(best, 1e3 * pr.distance)
        return float(best)

    def plane(self):
        """The index planar frame (pinch3_pose's: index FE anchor, index_proximal's axes) in the world."""
        from pydrake.math import RigidTransform

        fe_anchor = np.asarray(next(j for j in self.desc["joints"] if j["name"] == "index_fe")["anchor"], float)
        o_prox = np.asarray(next(b for b in self.desc["bodies"] if b["name"] == "index_proximal")["origin"], float)
        Xprox = self.plant.EvalBodyPoseInWorld(self.pctx, self.B("index_proximal"))
        return RigidTransform(Xprox.rotation(), Xprox.multiply(fe_anchor - o_prox))


def thumb_in_plane(hand: str, pose: dict, fe_deg: float = 72.5, pip_deg: float = 0.0, hm: HandModel | None = None,
                   middle: bool = True):
    """pinch3_pose.thumb_in_plane for any hand and thumb pose (the same keys; the pad face from the URDF's box)."""
    hm = hm or HandModel(hand)
    plant, ctx, desc = hm.plant, hm.pctx, hm.desc
    hm.set_fingers(fe_deg, pip_deg, middle)
    hm.set_thumb([pose[n] for n in THUMB_JOINTS])
    inv = hm.plane().inverse()
    Xt = plant.EvalBodyPoseInWorld(ctx, hm.B("thumb_distal"))
    pt_w, nt_w = Xt.multiply(hm.pad_face), Xt.rotation().multiply(hm.pad_normal)
    pt, nt = 1e3 * inv.multiply(pt_w), inv.rotation().multiply(nt_w)
    Xi, Xm = plant.EvalBodyPoseInWorld(ctx, hm.B("index_distal")), plant.EvalBodyPoseInWorld(ctx, hm.B("middle_distal"))
    pi, pm = 1e3 * inv.multiply(Xi.multiply(P_I)), 1e3 * inv.multiply(Xm.multiply(P_I))
    ni = inv.rotation().multiply(Xi.rotation().multiply(N_I))
    y_mid = 0.5 * (pi[1] + pm[1])
    tilt = math.degrees(math.asin(min(1.0, abs(float(nt[1])))))
    n_in = np.array([nt[0], nt[2]]) / math.hypot(nt[0], nt[2])
    k_theta = desc["servo"]["derived"]["stiffness_nm_per_rad"]
    comp, arms = 0.0, {}
    for name in THUMB_JOINTS:
        j = hm.J(name)
        Xc = plant.EvalBodyPoseInWorld(ctx, j.child_body())
        jd = next(x for x in desc["joints"] if x["name"] == name)
        cb = next(b for b in desc["bodies"] if b["name"] == jd["child"])
        o_w = Xc.multiply(np.asarray(jd["anchor"], float) - np.asarray(cb["origin"], float))
        a_w = Xc.rotation().multiply(np.asarray(jd["axis"], float))
        arm = float(a_w @ np.cross(pt_w - o_w, nt_w))
        arms[name] = 1e3 * arm
        comp += arm ** 2 / k_theta
    comp_mi = sum((arms[n] * 1e-3) ** 2 / k_theta for n in ("thumb_mcp", "thumb_ip"))
    opp3 = math.degrees(math.acos(float(np.clip(-(nt @ ni), -1, 1))))
    return {"hand": hand, "thumb_pose_deg": {n: float(pose[n]) for n in THUMB_JOINTS},
            "fe_deg": fe_deg, "pip_deg": pip_deg,
            "thumb_face_mm": [float(pt[0]), float(pt[2])], "thumb_face_y_mm": float(pt[1]), "mid_plane_y_mm": y_mid,
            "thumb_normal_in_plane": n_in.tolist(), "thumb_normal_tilt_deg": tilt,
            "index_pad_mm": [float(pi[0]), float(pi[2])], "middle_pad_y_mm": float(pm[1]),
            "index_pad_y_mm": float(pi[1]), "index_normal": [float(ni[0]), float(ni[2])],
            "opposition_deg": math.degrees(math.acos(float(np.clip(-(n_in @ np.array([ni[0], ni[2]]) /
                                                                     math.hypot(ni[0], ni[2])), -1, 1)))),
            "opposition_3d_deg": opp3, "gap_mm": float((pt - pi) @ ni), "tangential_mm": float((pt - pi) @ np.array([ni[2], 0.0, -ni[0]])),
            "k_theta_nm_rad": k_theta, "lever_arms_mm": arms, "k_thumb_n_m": 1.0 / comp,
            "k_thumb_mcp_ip_only_n_m": 1.0 / comp_mi if comp_mi > 0 else None,
            "clearance_mm": hm.clearance()}


def search_pose(hand: str, band: str, hm: HandModel | None = None, verbose: bool = True) -> dict:
    """POSE_RULE on the hand for one band (BANDS)."""
    hm = hm or HandModel(hand)
    hm.chain_cache = hm.chain()
    rule, bd = POSE_RULE, BANDS[band]
    middle = bd["lateral"] == "mid"
    lo, hi = np.degrees(hm.lo), np.degrees(hm.hi)
    frames = {}

    def frame(fe):
        if fe not in frames:
            hm.set_fingers(fe, bd["pip_deg"], middle)
            inv = hm.plane().inverse()
            Xi = hm.plant.EvalBodyPoseInWorld(hm.pctx, hm.B("index_distal"))
            Xm = hm.plant.EvalBodyPoseInWorld(hm.pctx, hm.B("middle_distal"))
            pi = 1e3 * inv.multiply(Xi.multiply(P_I))
            pm = 1e3 * inv.multiply(Xm.multiply(P_I))
            ni = inv.rotation().multiply(Xi.rotation().multiply(N_I))  # index pad outward normal (toward the thumb)
            y_ref = 0.5 * (pi[1] + pm[1]) if middle else pi[1]
            frames[fe] = (inv.rotation().matrix(), inv.translation(), pi, ni, np.array([ni[2], 0.0, -ni[0]]), y_ref)
        return frames[fe]

    def evaluate(deg, fe):
        Rinv, tinv, pi, ni, ti, y_ref = frame(fe)
        p, nv = hm.fk_batch(deg)
        p = 1e3 * (p @ Rinv.T + tinv)
        nv = nv @ Rinv.T
        d = p - pi
        gap, tang, dy = d @ ni, d @ ti, p[:, 1] - y_ref
        ang = np.degrees(np.arccos(np.clip(-(nv @ ni), -1, 1)))
        ok = ((gap >= bd["gap_mm"][0]) & (gap <= bd["gap_mm"][1]) & (np.abs(tang) <= rule["tangential_mm"])
              & (np.abs(dy) <= rule["mid_plane_mm"]))
        return ang, ok

    def grid(lo_, hi_, step):
        axes = [np.arange(l, h + 1e-9, step) for l, h in zip(lo_, hi_)]
        return np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 4)

    def clear(fe, deg):
        hm.set_fingers(fe, bd["pip_deg"], middle)
        hm.set_thumb(deg)
        return hm.clearance() > rule["clear_mm"]

    g = grid(lo, hi, rule["grid_deg"])
    cands = []
    for fe in np.arange(*rule["fe_deg"][:2], rule["fe_deg"][2]).tolist() + [rule["fe_deg"][1]]:
        ang, ok = evaluate(g, fe)
        cands += [(float(a_), fe, x) for a_, x in zip(ang[ok], g[ok])]
    cands.sort(key=lambda t: t[0])
    seeds = []
    for a_, fe, x in cands:
        if clear(fe, x):
            seeds.append((a_, fe, x))
            if len(seeds) >= rule["refine_top"]:
                break
    if verbose:
        print(f"{hand} {band}: {len(cands)} grid poses meet the placement, {len(seeds)} clear seeds; best grid "
              f"{seeds[0][0] if seeds else float('nan'):.1f} deg", flush=True)
    best = (np.inf, None, None)
    h = rule["refine_half_deg"]
    for _, fe0, s in seeds:
        gg = grid(np.maximum(lo, s - h), np.minimum(hi, s + h), rule["refine_deg"])
        for fe in (fe0 - 1.25, fe0, fe0 + 1.25):
            a2, ok2 = evaluate(gg, fe)
            c2, ca2 = gg[ok2], a2[ok2]
            for k in np.argsort(ca2):
                if ca2[k] >= best[0]:
                    break
                if clear(fe, c2[k]):
                    best = (float(ca2[k]), fe, c2[k])
                    break
    if best[1] is None:
        return {"hand": hand, "band": band, "found": False}
    pose = dict(zip(THUMB_JOINTS, map(float, np.round(best[2], 6))))
    out = thumb_in_plane(hand, pose, best[1], bd["pip_deg"], hm=hm, middle=middle)
    out.update({"found": True, "band": band, "band_rule": bd, "rule": rule, "grid_candidates": len(cands)})
    if verbose:
        print(f"  FE {best[1]:.2f}, thumb {pose}: opposition 3D {out['opposition_3d_deg']:.1f}, in-plane "
              f"{out['opposition_deg']:.1f}, tilt {out['thumb_normal_tilt_deg']:.1f} deg, gap {out['gap_mm']:.1f} mm, "
              f"tangential {out['tangential_mm']:+.1f} mm, lateral "
              f"{out['thumb_face_y_mm'] - (out['mid_plane_y_mm'] if middle else out['index_pad_y_mm']):+.1f} mm, k {out['k_thumb_n_m']:.0f} N/m, clearance {out['clearance_mm']:.2f} mm", flush=True)
    return out


def cmd_pose(a) -> int:
    res = {}
    for hand in a.hand:
        hm = HandModel(hand)
        res[hand] = {band: search_pose(hand, band, hm) for band in BANDS}
    if "O" in a.hand:  # the check: thumb_in_plane equals pinch3_pose's on O's historical pose
        import pinch3_pose as PP

        mine, ref = thumb_in_plane("O", HIST_POSE_O, 72.5), PP.thumb_in_plane(72.5)
        diff = max(abs(mine["opposition_deg"] - ref["opposition_deg"]), abs(mine["k_thumb_n_m"] - ref["k_thumb_n_m"]),
                   float(np.max(np.abs(np.subtract(mine["thumb_face_mm"], ref["thumb_face_mm"])))))
        print(f"check vs pinch3_pose on O's historical pose: max |diff| {diff:.2e}; that pose: 3D opposition "
              f"{mine['opposition_3d_deg']:.1f}, in-plane {mine['opposition_deg']:.1f} deg, gap {mine['gap_mm']:.1f} mm, "
              f"tangential {mine['tangential_mm']:+.1f} mm, clearance {mine['clearance_mm']:.2f}")
        res["O_hist"] = {"33b_small": mine, "33b_large": thumb_in_plane("O", HIST_POSE_O, 70.0),
                         "36": mine, "check_max_diff": diff}
    if a.out:
        old = json.loads(a.out.read_text()) if a.out.exists() else {}
        a.out.write_text(json.dumps({**old, **res}, indent=1))
    return 0


def _patched_pp(hand: str, pose_file: Path, task: str):
    """pinch3_pose with thumb_in_plane replaced by the hand's rule pose (from the pose file): for 33b the task's FE
    label 72.5 takes the 16-20 mm band's pose and 70 the 20-24 mm band's; for 36 the 6-9 mm band's."""
    import pinch3_pose as PP

    poses = json.loads(pose_file.read_text())[hand]
    if task == "33b":
        pick = {72.5: poses["33b_small"], 70.0: poses["33b_large"]}
    else:
        pick = {72.5: poses["36"]}
    PP.thumb_in_plane = lambda fe_deg=72.5, pip_deg=0.0: pick[float(fe_deg)]
    return PP, poses


def cmd_draws33b(a) -> int:
    import pinch3 as P3
    import two_pad_drake as T

    _, poses = _patched_pp(a.hand, a.poses, "33b")
    draws, ps = P3.make_draws(64, 34, P3.RANDOM)
    k_full = ps[72.5]["k_thumb_n_m"]
    config = {"approach_s": 1.0, "settle_s": 0.5, "release_s": 0.5, "sweep_s": 2.0, "hold_s": 0.5,
              "k_thumb": 0.5 * k_full, "thumb_contact_k": 0.5 * T.POINT_STIFFNESS, "share": 0.5}
    a.out.write_text(json.dumps({"seed": 34, "n": 64, "rng": "numpy.random.default_rng([34, 34])",
                                 "ranges": P3.RANDOM, "config": config, "k_thumb_full_n_m": k_full, "hand": a.hand,
                                 "pose": {str(k): v for k, v in ps.items()}, "draws": draws}, indent=1))
    opp = [x["opposition_deg"] for x in draws]
    print(f"{a.hand}: 64 draws -> {a.out}; thumb k {k_full:.0f} N/m; opposition {min(opp):.1f}..{max(opp):.1f} deg")
    return 0


def cmd_draws36(a) -> int:
    import peg_pinch as PG

    PP, poses = _patched_pp(a.hand, a.poses, "36")
    draws, pose = PG.make_draws(64, 36, PG.RANDOM, 72.5)
    config = {**PG.DEFAULTS, "k_thumb": pose["k_thumb_n_m"]}
    a.out.write_text(json.dumps({"seed": 36, "n": 64, "rng": "numpy.random.default_rng([36, 36])",
                                 "ranges": PG.RANDOM, "config": config, "hand": a.hand, "pose": pose,
                                 "draws": draws}, indent=1))
    opp = [x["opposition_deg"] for x in draws]
    ends = sum(x["index_end"] == "front" for x in draws)
    print(f"{a.hand}: 64 draws -> {a.out}; thumb k {pose['k_thumb_n_m']:.0f} N/m; index-thumb opposition "
          f"{min(opp):.1f}..{max(opp):.1f} deg; index on the peg's front end in {ends}/64")
    return 0


# ------------------------------------------------------------------------------------- reach (pinch_thumb_ik copy)
REACH_JOINTS = ("thumb_ro", "thumb_raa", "thumb_mcp", "thumb_ip", "index_fe", "index_aa")
FACE_O_MM = 6.0
POS_TOL_MM, ANG_TOL_DEG = 1.0, 5.0


class Reach:
    """pinch_thumb_ik.Hand (committed version) for any hand: the thumb pad face from the URDF box."""

    def __init__(self, hand: str):
        from pydrake.math import RigidTransform
        import render3d as R

        self.hm = HandModel(hand)
        self.plant, self.sg, self.model, self.desc = self.hm.plant, self.hm.sg, self.hm.model, self.hm.desc
        self.pctx, self.sctx = self.hm.pctx, self.hm.sctx
        self.joints = [self.plant.GetJointByName(n, self.model) for n in REACH_JOINTS]
        self.lo = np.array([j.position_lower_limits()[0] for j in self.joints])
        self.hi = np.array([j.position_upper_limits()[0] for j in self.joints])
        self.distal = self.plant.GetBodyByName("thumb_distal", self.model)
        self.proximal = self.plant.GetBodyByName("index_proximal", self.model)
        _, _, _, self.origin, _ = R.hand_frame("O", "index")  # fingers identical on both hands
        self.set_joints(np.zeros(len(REACH_JOINTS)))
        X0 = self.plant.EvalBodyPoseInWorld(self.pctx, self.proximal)
        self.X_P_plane = X0.inverse().multiply(RigidTransform(1e-3 * self.origin))
        self.body_of = self.hm.body_of

    def pose_fingers(self, q_index_deg):
        import render3d as R

        LH.set_pose(self.plant, self.pctx, self.model, self.desc, 0.0, 0.0)
        for digit in ("middle", "ring"):
            for name, value in LH.table(self.desc, digit, R.CURL[1]).items():
                self.plant.GetJointByName(name, self.model).set_angle(self.pctx, value)
            self.plant.GetJointByName(f"{digit}_fe", self.model).set_angle(self.pctx, math.radians(R.CURL[0]))
        for name, value in LH.table(self.desc, "index", q_index_deg).items():
            self.plant.GetJointByName(name, self.model).set_angle(self.pctx, value)

    def set_joints(self, x):
        for j, v in zip(self.joints, x):
            j.set_angle(self.pctx, float(v))

    def plane(self):
        return self.plant.EvalBodyPoseInWorld(self.pctx, self.proximal).multiply(self.X_P_plane)

    def pad(self, x):
        self.set_joints(x)
        X = self.plane().inverse().multiply(self.plant.EvalBodyPoseInWorld(self.pctx, self.distal))
        return 1e3 * (X @ self.hm.pad_face), X.rotation().matrix() @ self.hm.pad_normal

    def solve(self, p_mm, n_dir, starts=24, seed=0):
        from scipy.optimize import least_squares

        def res(x):
            p, nv = self.pad(x)
            ang = math.degrees(math.acos(float(np.clip(nv @ n_dir, -1, 1))))
            return np.concatenate([p - p_mm, [ang]])

        rng = np.random.default_rng(seed)
        best = None
        for _ in range(starts):
            x0 = self.lo + rng.uniform(0.05, 0.95, len(REACH_JOINTS)) * (self.hi - self.lo)
            r = least_squares(res, x0, bounds=(self.lo, self.hi), xtol=1e-10, ftol=1e-10)
            if best is None or r.cost < best.cost:
                best = r
        out = res(best.x)
        return best.x, float(np.linalg.norm(out[:3])), float(out[3])

    def clearance(self, sphere_c_plane_mm, r_mm):
        qo = self.sg.get_query_output_port().Eval(self.sctx)
        sphere_c_mm = 1e3 * (self.plane() @ (1e-3 * np.asarray(sphere_c_plane_mm)))
        hand = np.inf
        for pr in qo.ComputeSignedDistancePairwiseClosestPoints(0.05):
            a, b = self.body_of.get(pr.id_A), self.body_of.get(pr.id_B)
            if a is None or b is None:
                continue
            ta, tb = a[0].startswith("thumb"), b[0].startswith("thumb")
            if ta == tb:
                continue
            th, other = (a, b) if ta else (b, a)
            if th[0] == "thumb_cmc" or (th[0] == "thumb_metacarpal" and other[0] == "palm"):
                continue
            hand = min(hand, 1e3 * pr.distance)
        sph = np.inf
        insp = self.sg.model_inspector()
        for gid, (body, name) in self.body_of.items():
            if not body.startswith("thumb") or type(insp.GetShape(gid)).__name__ == "Box":
                continue
            shape = insp.GetShape(gid)
            X = qo.GetPoseInWorld(gid)
            p = 1e3 * X.translation()
            rad = 1e3 * getattr(shape, "radius", lambda: 0.0)()
            if hasattr(shape, "length"):
                ax = X.rotation().matrix()[:, 2]
                L = 1e3 * shape.length()
                t = float(np.clip((sphere_c_mm - p) @ ax, -L / 2, L / 2))
                d = np.linalg.norm(sphere_c_mm - (p + t * ax))
            else:
                d = np.linalg.norm(sphere_c_mm - p)
            sph = min(sph, d - rad - r_mm)
        return float(hand), float(sph)


def _reach_one(args):
    hand, g, run, seed = args
    hd = _reach_one.cache.get(hand) if hasattr(_reach_one, "cache") else None
    if hd is None:
        _reach_one.cache = {hand: Reach(hand)}
        hd = _reach_one.cache[hand]
    n2 = np.array(g["n"])
    n3 = np.array([n2[0], 0.0, n2[1]])
    to_hand = lambda xz: np.array([xz[0], 0.0, xz[1] - FACE_O_MM])
    shift = np.array(run["sphere_shift_mm"])
    travel = float(shift @ n2)
    row = {"draw": g["draw"], "q_s_deg": g["q_s_deg"], "r_mm": g["r_mm"], "travel_mm": travel}
    for phase, q_idx, off in (("start", g["q_s_deg"], 0.0), ("end", g["q_s_deg"] + g["sweep_deg"], travel)):
        tp = to_hand(np.array(g["thumb_mm"]) + off * n2)
        c = to_hand(np.array(g["c_mm"]) + shift if phase == "end" else np.array(g["c_mm"]))
        hd.pose_fingers(q_idx)
        if phase == "end":
            hd.lo[4:], hd.hi[4:] = row["start"]["x"][4:], np.array(row["start"]["x"][4:]) + 1e-9
        else:
            hd.lo[4:] = [j.position_lower_limits()[0] for j in hd.joints[4:]]
            hd.hi[4:] = [j.position_upper_limits()[0] for j in hd.joints[4:]]
        x, pos, ang = hd.solve(tp, -n3, seed=seed)
        hd.pad(x)
        ch, cs = hd.clearance(c, g["r_mm"])
        ok = pos < POS_TOL_MM and ang < ANG_TOL_DEG and ch > 0 and cs > -0.5
        row[phase] = {"x": x.tolist(), "joints_deg": dict(zip(REACH_JOINTS, np.degrees(x).round(3).tolist())),
                      "pos_mm": pos, "ang_deg": ang, "clear_hand_mm": ch, "clear_sphere_mm": cs, "ok": bool(ok)}
    return row


def cmd_reach(a) -> int:
    from multiprocessing import Pool

    draws = json.loads(a.draws.read_text())["draws"][: a.only]
    run = {r["draw"]: r["1.3"] for r in json.loads(a.run.read_text())["draws"]}
    with Pool(a.workers) as pool:
        rows = pool.map(_reach_one, [(a.hand, g, run[g["draw"]], a.seed) for g in draws], chunksize=1)
    both = sum(r["start"]["ok"] and r["end"]["ok"] for r in rows)
    pos = np.array([[r["start"]["pos_mm"], r["end"]["pos_mm"]] for r in rows])
    ang = np.array([[r["start"]["ang_deg"], r["end"]["ang_deg"]] for r in rows])
    summ = {"hand": a.hand, "seed": a.seed, "start_ok": sum(r["start"]["ok"] for r in rows),
            "end_ok": sum(r["end"]["ok"] for r in rows), "both_ok": both, "n": len(rows),
            "pos_median_mm": float(np.median(pos)), "ang_median_deg": float(np.median(ang)),
            "ang_start_median_deg": float(np.median(ang[:, 0])),
            "within_tol_ignoring_collision": int(sum((r["start"]["pos_mm"] < POS_TOL_MM) and
                                                     (r["start"]["ang_deg"] < ANG_TOL_DEG) for r in rows))}
    print(json.dumps(summ))
    a.out.write_text(json.dumps({"tolerance": {"pos_mm": POS_TOL_MM, "ang_deg": ANG_TOL_DEG}, "summary": summ,
                                 "rows": rows}, indent=1))
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("pose")
    s.add_argument("--hand", nargs="+", default=["O", "O_thumbopt"])
    s.add_argument("--out", type=Path, default=LOGS / "thumbopt_pose.json")
    for name in ("draws33b", "draws36"):
        s = sub.add_parser(name)
        s.add_argument("--hand", required=True)
        s.add_argument("--poses", type=Path, default=LOGS / "thumbopt_pose.json")
        s.add_argument("--out", type=Path, required=True)
    s = sub.add_parser("reach")
    s.add_argument("--hand", required=True)
    s.add_argument("--draws", type=Path, default=LOGS / "pinch_e33.json")
    s.add_argument("--run", type=Path, default=LOGS / "pinch_O_e33.json")
    s.add_argument("--out", type=Path, required=True)
    s.add_argument("--only", type=int, default=None)
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--workers", type=int, default=5)
    a = p.parse_args()
    return {"pose": cmd_pose, "draws33b": cmd_draws33b, "draws36": cmd_draws36, "reach": cmd_reach}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
