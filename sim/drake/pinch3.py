"""Experiment 33b: three-finger pinch rotate (thumb opposing index and middle), the real thumb at its URDF pose
(docs/experimental/drake_pinch.md, 33b). Draws and calibration only; the simulation is pinch_rotate.py's, configured by
the draws file's 'config':

- Planar and symmetric: index and middle carry the same redesign and the same PIP command, so the sphere stays in
  their mid-plane; one finger is modelled carrying half of everything the thumb and sphere see ('share' 0.5: the
  thumb's spring and its pad's contact stiffness, and the sphere's torsional spring and damper are halved; the thumb
  pad's penetration is its force over the halved stiffness, i.e. the real thumb force over 2.5 kN/m). The thumb
  normal's out-of-plane tilt (7.9 deg at the pose) is ignored.
- The thumb is the real thumb at the pose of pinch3_pose.py (thumb_ro -90, raa 10, mcp 45, ip -7.5 deg; index and
  middle FE 72.5 deg, 70 for spheres of 20 mm and more): its pad's in-plane normal (20.4 / 17.9 deg from direct
  opposition) and its stiffness along that normal, every thumb servo's XM335 position loop through its lever arm
  (k_thumb_full N/m; per finger half). It moves only along its normal (the servo compliance), with its rest position
  preload / k into the sphere.
- Placement (proximal-face frame, the same on every hand, as every Study 3 task): the index distal pad's centre point
  (distal-frame x -6.5 mm, the URDF pad centre) shifted by jitter_mm along the pad, at O's start flexion q_s; the sphere
  touches it (centre r along O's pad normal); the thumb pad touches the sphere's far side with the FK normal turned by
  jitter_deg. The thumb face then sits thumb_shift_mm (along its normal, tangential) from the FK pose: the small
  thumb re-command that closes the pinch on this sphere (reported).

    uv run python pinch3.py draws --n 64 --seed 34 --out logs/pinch3_e33b.json [--ranges JSON]
    uv run python pinch_rotate.py run --hands O --draws logs/pinch3_e33b.json --tag e33b
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import pinch3_pose as PP
import two_pad_drake as T

HERE = Path(__file__).resolve().parent
# experiment 33b's distributions (fixed on O before any redesigned-hand data): independent uniforms drawn as vectors of
# N in this order from numpy.random.default_rng([seed, 34])
RANDOM = {"q_s_deg": (0.0, 10.0), "r_mm": (8.0, 12.0), "jitter_mm": (-1.0, 1.0), "jitter_deg": (-3.0, 3.0),
          "preload_n": (0.2, 0.5), "sweep_deg": (4.0, 10.0), "k_r": (0.6, 0.9)}
PAD_CENTRE_X = -6.5  # the index distal pad's centre, distal frame (mm): the URDF pad centre


def make_draws(n, seed, ranges=RANDOM):
    rng = np.random.default_rng([seed, 34])
    cols = {k: rng.uniform(lo, hi, n) for k, (lo, hi) in ranges.items()}
    g = T.finger_geometry("O")
    qs, th = T.servo_table(g)
    face = g["pads"]["mount_offset_mm"] + g["pads"]["thickness_mm"]
    poses = {fe: PP.thumb_in_plane(fe) for fe in (72.5, 70.0)}
    out = []
    for i in range(n):
        d = {k: float(v[i]) for k, v in cols.items()}
        fe = 72.5 if 2 * d["r_mm"] < 20.0 else 70.0
        pose = poses[fe]
        q = math.radians(d["q_s_deg"])
        p = T.reference_point(g, qs, th, q, PAD_CENTRE_X + d["jitter_mm"])  # face frame
        n_i = T.rot(q) @ np.array([0.0, -1.0])
        c = p + d["r_mm"] * n_i
        a = math.radians(d["jitter_deg"])
        nt = np.array(pose["thumb_normal_in_plane"])  # the thumb pad's outward normal, toward the sphere
        nt = np.array([math.cos(a) * nt[0] - math.sin(a) * nt[1], math.sin(a) * nt[0] + math.cos(a) * nt[1]])
        tp = c - d["r_mm"] * nt  # the thumb face's contact point (face frame)
        fk = np.array(pose["thumb_face_mm"]) + np.array([0.0, face])  # FK thumb face centre, face frame
        dv = tp - fk
        out.append({"draw": i, **d, "fe_deg": fe, "c_mm": c.tolist(), "n": (-nt).tolist(), "thumb_mm": tp.tolist(),
                    "thumb_shift_mm": {"along_normal": float(dv @ nt), "tangential": float(dv @ np.array([-nt[1], nt[0]]))},
                    "opposition_deg": float(math.degrees(math.acos(float(np.clip(-(nt @ n_i), -1, 1)))))})
    return out, poses


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pd_ = sub.add_parser("draws")
    pd_.add_argument("--n", type=int, default=64)
    pd_.add_argument("--seed", type=int, default=34)
    pd_.add_argument("--out", type=Path, required=True)
    pd_.add_argument("--ranges", default=None, help="JSON overriding RANDOM's entries (calibration)")
    pd_.add_argument("--config", default=None, help="JSON overriding the config (calibration)")
    a = p.parse_args()
    ranges = {**RANDOM, **(json.loads(a.ranges) if a.ranges else {})}
    draws, poses = make_draws(a.n, a.seed, ranges)
    k_full = poses[72.5]["k_thumb_n_m"]
    config = {"approach_s": 1.0, "settle_s": 0.5, "release_s": 0.5, "sweep_s": 2.0, "hold_s": 0.5,
              "k_thumb": 0.5 * k_full, "thumb_contact_k": 0.5 * T.POINT_STIFFNESS, "share": 0.5,
              **(json.loads(a.config) if a.config else {})}
    sh = np.array([[x["thumb_shift_mm"]["along_normal"], x["thumb_shift_mm"]["tangential"]] for x in draws])
    a.out.write_text(json.dumps({"seed": a.seed, "n": a.n, "rng": f"numpy.random.default_rng([{a.seed}, 34])",
                                 "ranges": ranges, "config": config, "k_thumb_full_n_m": k_full,
                                 "pose": {str(k): v for k, v in poses.items()}, "draws": draws}, indent=1))
    print(f"{len(draws)} draws -> {a.out}; thumb k {k_full:.0f} N/m (per finger {0.5 * k_full:.0f}); thumb shift from "
          f"FK along normal {sh[:, 0].min():+.1f}..{sh[:, 0].max():+.1f} mm, tangential {sh[:, 1].min():+.1f}.."
          f"{sh[:, 1].max():+.1f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
