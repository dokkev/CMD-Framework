"""Experiment 33's opposing pad against the hand's real thumb: can hand O's thumb (thumb_ro, thumb_raa, thumb_mcp,
thumb_ip, within their limits) put its distal pad (the URDF's thumb_distal pad box, face at x = 13 mm in the link
frame, outward normal +x) where the simulation's thumb pad is, with its face toward the sphere (normal -n)?

For every e33 draw, two poses: the sweep's start (the nominal thumb face: the sphere's far side, c + r n) and the end
(the face moved back along n by the sphere's travel along n over the sweep and hold in O's mu 1.3 run). The index
finger is at the frame's flexion (q_s, q_s + sweep), middle and ring curled as in the render (render3d.CURL), the
thumb solved by least squares (position mm, normal deg, equal weight) from 24 starts. Collision: the signed distance
of the thumb's proximal and distal links (and the metacarpal) to the palm and the index, middle and ring links (the
URDF's collision geometry, no filters), and the thumb's links (pad excepted) to the sphere.

    uv run python pinch_thumb_ik.py [--draws logs/pinch_e33.json] [--out logs/pinch_thumb_ik.json]
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import load_hand as LH
import render3d as R

HERE = Path(__file__).resolve().parent
THUMB = ("thumb_ro", "thumb_raa", "thumb_mcp", "thumb_ip", "index_fe", "index_aa")
"""The solved joints: the thumb's four and the index MCP's two. The simulation holds the index MCP fixed with gravity
off (the proximal phalanx is its world), so any fixed MCP pose leaves the task's physics unchanged; the index MCP is
posed to meet the thumb, as a pinch does."""
PAD_FACE = np.array([0.013, 0.041, 0.0])  # the thumb pad box's face centre in thumb_distal (m); its normal +x
FACE_O_MM = 6.0  # O's proximal face depth below the link line (mount 3 + pad 3 mm): world z = z_face - 6
POS_TOL_MM, ANG_TOL_DEG = 1.0, 5.0


class Hand:
    def __init__(self):
        from pydrake.math import RigidTransform
        from pydrake.multibody.parsing import Parser
        from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
        from pydrake.systems.framework import DiagramBuilder

        urdf, description = LH.paths("O", visuals=False)
        self.desc = json.loads(description.read_text())
        b = DiagramBuilder()
        self.plant, self.sg = AddMultibodyPlantSceneGraph(b, 0.0)
        self.model = Parser(self.plant).AddModels(str(urdf))[0]
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("palm", self.model), RigidTransform())
        self.plant.Finalize()  # no collision filters: palm-thumb pairs are checked here
        self.diagram = b.Build()
        self.ctx = self.diagram.CreateDefaultContext()
        self.pctx = self.plant.GetMyMutableContextFromRoot(self.ctx)
        self.sctx = self.sg.GetMyContextFromRoot(self.ctx)
        self.joints = [self.plant.GetJointByName(n, self.model) for n in THUMB]
        self.lo = np.array([j.position_lower_limits()[0] for j in self.joints])
        self.hi = np.array([j.position_upper_limits()[0] for j in self.joints])
        self.distal = self.plant.GetBodyByName("thumb_distal", self.model)
        self.proximal = self.plant.GetBodyByName("index_proximal", self.model)
        _, _, _, self.origin, _ = R.hand_frame("O", "index")  # the index planar frame's origin (mm, hand frame)
        self.set_joints(np.zeros(len(THUMB)))
        X0 = self.plant.EvalBodyPoseInWorld(self.pctx, self.proximal)
        # the planar frame in index_proximal's frame (at MCP 0 it is the hand frame moved to the FE anchor)
        self.X_P_plane = X0.inverse().multiply(__import__("pydrake.math", fromlist=["RigidTransform"]).RigidTransform(
            1e-3 * self.origin))
        insp = self.sg.model_inspector()
        self.body_of = {}
        for gid in insp.GetAllGeometryIds():
            fid = insp.GetFrameId(gid)
            body = self.plant.GetBodyFromFrameId(fid)
            if body is not None:
                self.body_of[gid] = (body.name(), insp.GetName(gid))

    def pose_fingers(self, q_index_deg):
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
        """The index planar frame in the world at the current MCP pose."""
        return self.plant.EvalBodyPoseInWorld(self.pctx, self.proximal).multiply(self.X_P_plane)

    def pad(self, x):
        """The thumb pad's face centre (mm) and outward normal, in the index planar frame."""
        self.set_joints(x)
        Xp = self.plane().inverse()
        X = Xp.multiply(self.plant.EvalBodyPoseInWorld(self.pctx, self.distal))
        return 1e3 * (X @ PAD_FACE), X.rotation().matrix()[:, 0]

    def solve(self, p_mm, n_dir, starts=24, seed=0):
        from scipy.optimize import least_squares

        def res(x):
            p, nv = self.pad(x)
            ang = math.degrees(math.acos(float(np.clip(nv @ n_dir, -1, 1))))
            return np.concatenate([p - p_mm, [ang]])

        rng = np.random.default_rng(seed)
        best = None
        for k in range(starts):
            x0 = self.lo + rng.uniform(0.05, 0.95, len(THUMB)) * (self.hi - self.lo)
            r = least_squares(res, x0, bounds=(self.lo, self.hi), xtol=1e-10, ftol=1e-10)
            if best is None or r.cost < best.cost:
                best = r
        out = res(best.x)
        return best.x, float(np.linalg.norm(out[:3])), float(out[3]), (out[:3]).tolist()

    def clearance(self, sphere_c_plane_mm, r_mm):
        """Smallest signed distances (mm): thumb links (metacarpal, proximal, distal) to the palm and the fingers;
        thumb links except the pad to the sphere."""
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
                continue  # the thumb's base sits in the palm
            hand = min(hand, 1e3 * pr.distance)
        # the sphere against the thumb's capsules (pad box excepted): segment distances
        sph = np.inf
        insp = self.sg.model_inspector()
        for gid, (body, name) in self.body_of.items():
            if not body.startswith("thumb") or name.endswith("_c3"):
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


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--draws", type=Path, default=HERE / "logs" / "pinch_e33.json")
    p.add_argument("--run", type=Path, default=HERE / "logs" / "pinch_O_e33.json")
    p.add_argument("--out", type=Path, default=HERE / "logs" / "pinch_thumb_ik.json")
    p.add_argument("--only", type=int, default=None)
    a = p.parse_args()
    draws = json.loads(a.draws.read_text())["draws"][: a.only]
    run = {r["draw"]: r for r in json.loads(a.run.read_text())["draws"]}
    hand = Hand()
    rows = []
    for g in draws:
        n2 = np.array(g["n"])
        n3 = np.array([n2[0], 0.0, n2[1]])
        to_hand = lambda xz_face: np.array([xz_face[0], 0.0, xz_face[1] - FACE_O_MM])  # planar frame, mm
        x_o = run[g["draw"]]["1.3"]
        shift = np.array(x_o["sphere_shift_mm"])
        travel = float(shift @ n2)
        row = {"draw": g["draw"], "q_s_deg": g["q_s_deg"], "r_mm": g["r_mm"], "back_mm": g["back_mm"],
               "travel_mm": travel}
        for phase, q_idx, off in (("start", g["q_s_deg"], 0.0), ("end", g["q_s_deg"] + g["sweep_deg"], travel)):
            tp = to_hand(np.array(g["thumb_mm"]) + off * n2)
            c = to_hand(np.array(g["c_mm"]) + shift if phase == "end" else np.array(g["c_mm"]))
            hand.pose_fingers(q_idx)
            if phase == "end":  # the MCP stays where the start put it; only the thumb follows its spring
                hand.lo[4:], hand.hi[4:] = row["start"]["x"][4:], row["start"]["x"][4:]
                hand.hi[4:] += 1e-9
            else:
                hand.lo[4:] = [j.position_lower_limits()[0] for j in hand.joints[4:]]
                hand.hi[4:] = [j.position_upper_limits()[0] for j in hand.joints[4:]]
            x, pos, ang, dp = hand.solve(tp, -n3)
            hand.pad(x)
            clr_hand, clr_sph = hand.clearance(c, g["r_mm"])
            ok = pos < POS_TOL_MM and ang < ANG_TOL_DEG and clr_hand > 0 and clr_sph > -0.5
            row[phase] = {"x": x.tolist(), "joints_deg": dict(zip(THUMB, np.degrees(x).round(3).tolist())), "pos_mm": pos, "ang_deg": ang,
                          "dp_mm": dp, "clear_hand_mm": clr_hand, "clear_sphere_mm": clr_sph, "ok": bool(ok),
                          "target_mm": tp.tolist()}
        rows.append(row)
        s, e = row["start"], row["end"]
        print(f"draw {g['draw']:2d} q_s {g['q_s_deg']:5.1f} r {g['r_mm']:4.1f} | start pos {s['pos_mm']:6.2f} mm ang "
              f"{s['ang_deg']:5.1f} clr {s['clear_hand_mm']:6.1f}/{s['clear_sphere_mm']:5.1f} {'OK' if s['ok'] else '--'}"
              f" | end pos {e['pos_mm']:6.2f} ang {e['ang_deg']:5.1f} clr {e['clear_hand_mm']:6.1f}/"
              f"{e['clear_sphere_mm']:5.1f} {'OK' if e['ok'] else '--'}", flush=True)
    both = sum(r["start"]["ok"] and r["end"]["ok"] for r in rows)
    pos = np.array([[r["start"]["pos_mm"], r["end"]["pos_mm"]] for r in rows])
    ang = np.array([[r["start"]["ang_deg"], r["end"]["ang_deg"]] for r in rows])
    print(f"reachable (pos < {POS_TOL_MM} mm, normal < {ANG_TOL_DEG} deg, no collision) at start "
          f"{sum(r['start']['ok'] for r in rows)}, end {sum(r['end']['ok'] for r in rows)}, both {both} / {len(rows)}; "
          f"pos median {np.median(pos):.2f} mm (max {pos.max():.2f}), normal median {np.median(ang):.2f} deg "
          f"(max {ang.max():.2f})")
    a.out.write_text(json.dumps({"tolerance": {"pos_mm": POS_TOL_MM, "ang_deg": ANG_TOL_DEG}, "rows": rows}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
