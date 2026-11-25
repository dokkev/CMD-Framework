"""Experiment 33b: the three-finger pinch's thumb, from hand O's URDF at the chosen pose, in the index finger's planar
frame (docs/experimental/drake_pinch.md, 33b).

The pose (URDF joint values, found by a joint-space FK grid): thumb_ro -90, thumb_raa 10 (CMC), thumb_mcp 45,
thumb_ip -7.5 deg; index_fe = middle_fe = FE (72.5 deg; 70 for the larger spheres), index_aa = middle_aa = 0; index and
middle PIP 0; ring curled. Pads: index distal pad face centre (-6.5, 0, -6) mm in index_distal, outward normal -z;
thumb pad face centre (13, 41, 0) mm in thumb_distal, outward normal +x.

Returns, in the planar frame of the index (two_pad_drake's: x along the proximal phalanx from the MCP-FE axis, z
dorsal, the proximal phalanx is the world): the thumb pad's face centre projected on the plane midway between the
index and the middle finger (mm), its normal's in-plane direction and its out-of-plane tilt (ignored by the planar
model), the index-middle pad spacing, and the thumb's equivalent linear stiffness along its pad normal: every thumb
servo held by the XM335 position loop (desc["servo"]["derived"]["stiffness_nm_per_rad"]), in series through its lever
arm, k = 1 / sum_i (a_i . n x (p - o_i))^2 / k_theta, with o_i, a_i the joint's anchor and axis and p the pad centre.

    uv run python pinch3_pose.py [--fe 72.5]
"""
from __future__ import annotations

import argparse
import json
import math

import numpy as np

import load_hand as LH

POSE = {"thumb_ro": -90.0, "thumb_raa": 10.0, "thumb_mcp": 45.0, "thumb_ip": -7.5, "index_aa": 0.0, "middle_aa": 0.0}
THUMB_JOINTS = ("thumb_ro", "thumb_raa", "thumb_mcp", "thumb_ip")
P_I, N_I = np.array([-0.0065, 0.0, -0.006]), np.array([0.0, 0.0, -1.0])
P_T, N_T = np.array([0.013, 0.041, 0.0]), np.array([1.0, 0.0, 0.0])


def thumb_in_plane(fe_deg: float = 72.5, pip_deg: float = 0.0) -> dict:
    from pydrake.math import RigidTransform

    urdf, description = LH.paths("O", visuals=False)
    _, plant, sg, model, desc, _ = LH.load_hand(urdf, description, gravity=False, pose=(0, 0, 0, 0, 0, 0))
    ctx = plant.CreateDefaultContext()
    J = lambda n: plant.GetJointByName(n, model)
    B = lambda n: plant.GetBodyByName(n, model)
    LH.set_pose(plant, ctx, model, desc, pip_deg)
    for k, v in LH.table(desc, "ring", 100).items():
        J(k).set_angle(ctx, v)
    J("ring_fe").set_angle(ctx, math.radians(95))
    for k, v in POSE.items():
        J(k).set_angle(ctx, math.radians(v))
    for d in ("index", "middle"):
        J(f"{d}_fe").set_angle(ctx, math.radians(fe_deg))
    # the planar frame: the index FE anchor, axes of index_proximal (at FE 0 the hand's); the mid-plane is offset in y
    fe_anchor = np.asarray(next(j for j in desc["joints"] if j["name"] == "index_fe")["anchor"], float)
    Xprox = plant.EvalBodyPoseInWorld(ctx, B("index_proximal"))
    origin_w = Xprox.multiply(fe_anchor - np.asarray(next(b for b in desc["bodies"]
                                                          if b["name"] == "index_proximal")["origin"], float))
    Xplane = RigidTransform(Xprox.rotation(), origin_w)
    inv = Xplane.inverse()
    Xt = plant.EvalBodyPoseInWorld(ctx, B("thumb_distal"))
    pt_w, nt_w = Xt.multiply(P_T), Xt.rotation().multiply(N_T)
    pt, nt = 1e3 * inv.multiply(pt_w), inv.rotation().multiply(nt_w)
    Xi, Xm = plant.EvalBodyPoseInWorld(ctx, B("index_distal")), plant.EvalBodyPoseInWorld(ctx, B("middle_distal"))
    pi, pm = 1e3 * inv.multiply(Xi.multiply(P_I)), 1e3 * inv.multiply(Xm.multiply(P_I))
    ni = inv.rotation().multiply(Xi.rotation().multiply(N_I))
    y_mid = 0.5 * (pi[1] + pm[1])
    tilt = math.degrees(math.asin(abs(float(nt[1]))))
    n_in = np.array([nt[0], nt[2]]) / math.hypot(nt[0], nt[2])
    # the thumb's compliance along its pad normal
    k_theta = desc["servo"]["derived"]["stiffness_nm_per_rad"]
    comp, arms = 0.0, {}
    for name in THUMB_JOINTS:
        j = J(name)
        Xc = plant.EvalBodyPoseInWorld(ctx, j.child_body())
        jd = next(x for x in desc["joints"] if x["name"] == name)
        cb = next(b for b in desc["bodies"] if b["name"] == jd["child"])
        o_w = Xc.multiply(np.asarray(jd["anchor"], float) - np.asarray(cb["origin"], float))
        a_w = Xc.rotation().multiply(np.asarray(jd["axis"], float))
        arm = float(a_w @ np.cross(pt_w - o_w, nt_w))  # m: torque about the joint per unit force along n
        arms[name] = 1e3 * arm
        comp += arm ** 2 / k_theta
    k_lin = 1.0 / comp
    comp_mi = sum((arms[n] * 1e-3) ** 2 / k_theta for n in ("thumb_mcp", "thumb_ip"))
    return {"fe_deg": fe_deg, "pip_deg": pip_deg,
            "thumb_face_mm": [float(pt[0]), float(pt[2])], "thumb_face_y_mm": float(pt[1]), "mid_plane_y_mm": y_mid,
            "thumb_normal_in_plane": n_in.tolist(), "thumb_normal_tilt_deg": tilt,
            "index_pad_mm": [float(pi[0]), float(pi[2])], "middle_pad_y_mm": float(pm[1]), "index_pad_y_mm": float(pi[1]),
            "index_normal": [float(ni[0]), float(ni[2])],
            "opposition_deg": math.degrees(math.acos(float(np.clip(-(n_in @ np.array([ni[0], ni[2]]) /
                                                                     math.hypot(ni[0], ni[2])), -1, 1)))),
            "k_theta_nm_rad": k_theta, "lever_arms_mm": arms, "k_thumb_n_m": k_lin,
            "k_thumb_mcp_ip_only_n_m": 1.0 / comp_mi if comp_mi > 0 else None}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--fe", type=float, nargs="+", default=[72.5, 70.0])
    a = p.parse_args()
    for fe in a.fe:
        print(json.dumps(thumb_in_plane(fe), indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
