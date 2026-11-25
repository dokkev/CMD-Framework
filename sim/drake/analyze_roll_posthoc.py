"""Experiment 22 (in-hand rolling), post hoc and not used for any judgement: why does the added error E not follow
Delta_contact? Two hypotheses, each a Spearman rho over the scored hands with a hand-bootstrap 95% interval, from the
logs alone (no new simulation).

1. The contact point moves along the pads during the sweep, so the one-point Delta_contact (the point under the
   fingertip, over 0-120 deg) misses the change where the cylinder actually touches. Per pair, the distal contact
   location is found geometrically: the cylinder centre (pre.centre_mm at the grasp, pre.centre_mm + (along_mm,
   -off_face_mm) at the end) projected onto the distal face at the measured flexion (pre.q_deg, roll.q_end_deg), the
   distal pose closed from the servo table. The location is kept as its distance from the pad's distal end and taken
   linear in q over the commanded sweep (q_cmd to q_cmd + s). The local Delta_contact is the largest distance, over the
   sweep, between H's and O's distal-face point at that same distance from the tip, each in its own proximal-face frame
   (two_pad_drake.reference_point), at O's contact locations (primary) and at H's (variant). A one-point control: the
   tip point over the swept range only.
2. The friction effect comes from damping-induced partial slip that depends on the preload. Per pair, the preload
   deviation |N_H / N_O - 1| (distal, and the sum of both pads) at the grasp and at the end, the mean over the two mu.
   Slip against pure rolling: on the proximal pad (fixed) pure rolling gives along = rho * rotation; on the distal pad
   the contact's travel along the pad is rho * (rotation + delta q). Slip is the measured travel minus that.

Per hand: medians over the scored pairs (pairing as in analyze_roll.py: Gate 0 at both mu on O and H, |D(O)| >= 0.5
deg, H holding at both mu).

    uv run python analyze_roll_posthoc.py [--out logs/e22_posthoc.json]
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

from analyze import DESIGN
from two_pad_drake import _closure_gap, finger_geometry, reference_point, rot, servo_table

HERE = Path(__file__).resolve().parent
RADII, MU, SWEEPS, MIN_D = (15.0, 20.0), (1.3, 2.0), (8, 15), 0.5
N_BOOT = 10_000


def load(hand, mu, s, r):
    p = HERE / "logs" / f"dkroll_{hand}_mu{mu:g}_point-e22-s{s}_r{r:g}.json"
    return {(e["q0_deg"], round(e["x_c_mm"], 3)): e for e in json.loads(p.read_text())["envs"]} if p.exists() else None


class Hand:
    def __init__(self, name):
        self.g = finger_geometry(name)
        self.qs, self.th = servo_table(self.g)
        p = self.g["pads"]
        self.face = p["mount_offset_mm"] + p["thickness_mm"]
        self.t0, self.t1 = p["distal_start_mm"], p["distal_end_mm"]

    def b(self, q):
        """The distal frame's origin B (world mm) at flexion q (rad), the four-bar closed exactly."""
        t0 = float(np.interp(q, self.qs, self.th))
        lo, hi = t0 - 0.01, t0 + 0.01
        while _closure_gap(self.g, lo, q) * _closure_gap(self.g, hi, q) > 0:
            lo, hi = lo - 0.01, hi + 0.01
        theta = brentq(lambda t: _closure_gap(self.g, t, q), lo, hi, xtol=1e-13)
        return self.g["ap"] + rot(theta) @ (self.g["b"] - self.g["ap"])

    def contact_x(self, centre, q):
        """The distal-frame x of the foot of the cylinder centre on the distal face, and the centre's depth below
        the face (mm; rho minus penetration)."""
        r = rot(q).T @ (np.asarray(centre) - self.b(q))
        return float(r[0]), float(-self.face - r[1])

    def point(self, q, from_tip):
        return reference_point(self.g, self.qs, self.th, q, self.t1 - from_tip)


def ranks(v):
    return np.argsort(np.argsort(v)).astype(float)


def spearman(x, y):
    return float(np.corrcoef(ranks(np.asarray(x, float)), ranks(np.asarray(y, float)))[0, 1])


def partial_spearman(y, x, z):
    ry, rx, rz = (ranks(np.asarray(v, float)) for v in (y, x, z))
    A = np.c_[rz, np.ones(len(rz))]
    a, b = (v - A @ np.linalg.lstsq(A, v, rcond=None)[0] for v in (ry, rx))
    den = math.sqrt((a @ a) * (b @ b))
    return float(a @ b / den) if den > 0 else float("nan")


def boot(f, n, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(N_BOOT):
        i = rng.integers(0, n, n)
        if len(set(i.tolist())) < 4:
            continue
        v = f(i)
        if np.isfinite(v):
            out.append(v)
    return [float(np.percentile(out, 2.5)), float(np.percentile(out, 97.5))]


def run_quantities(hd: Hand, e, rho):
    """Per run: distal contact at grasp and end (distance from the tip, mm), slips (mm), forces."""
    pre, ro = e["pre"], e["roll"]
    q0, q1 = math.radians(pre["q_deg"]), math.radians(ro["q_end_deg"])
    c0 = np.array(pre["centre_mm"])
    c1 = c0 + np.array([ro["along_mm"], -ro["off_face_mm"]])
    x0, d0 = hd.contact_x(c0, q0)
    x1, d1 = hd.contact_x(c1, q1)
    r = math.radians(ro["rotation_deg"])
    return {"tip0": hd.t1 - x0, "tip1": hd.t1 - x1, "travel_d": x1 - x0, "depth0": d0, "depth1": d1,
            "on_pad": bool(hd.t0 <= x0 <= hd.t1 and hd.t0 <= x1 <= hd.t1),
            "slip_p": ro["along_mm"] - rho * r, "slip_d": (x1 - x0) - rho * (r + (q1 - q0)),
            "roll_p": rho * r, "along": ro["along_mm"], "rot": ro["rotation_deg"],
            "n_pre": np.array(pre["normal_n"]), "n_end": np.array(ro["normal_end_n"]), "q_cmd": pre["q_cmd_deg"]}


def local_dc(o: Hand, h: Hand, q_cmd, sweep, tips, tips_h=None, n=31):
    """max over the commanded sweep of |c_H(q) - c_O(q)| at the given distance from the tip (linear in q); with
    tips_h, H's point is taken at its own distance (each hand at its own actual contact)."""
    tips_h = tips if tips_h is None else tips_h
    qa, qb = math.radians(q_cmd), min(math.radians(q_cmd + sweep), math.radians(120.0))
    best = 0.0
    for k, q in enumerate(np.linspace(qa, qb, n)):
        u = k / (n - 1)
        fo, fh = tips[0] + (tips[1] - tips[0]) * u, tips_h[0] + (tips_h[1] - tips_h[0]) * u
        best = max(best, float(np.linalg.norm(h.point(q, fh) - o.point(q, fo))))
    return best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=HERE / "logs" / "e22_posthoc.json")
    args = ap.parse_args()
    scored = {x["hand"]: x for x in json.loads(
        (HERE.parents[1] / "report" / "study3" / "data" / "drake_e22_point.json").read_text())}
    o = Hand("O")
    rows, o_pairs = [], {}
    for hand in DESIGN:
        if hand not in scored:
            continue
        h = Hand(hand)
        pairs = []
        for s in SWEEPS:
            for r in RADII:
                runs = {(n, m): load(n, m, s, r) for n in ("O", hand) for m in MU}
                for k in runs[("O", MU[0])]:
                    es = {nm: runs[nm].get(k) for nm in runs}
                    if any(v is None or not v["gate0"]["pass"] or v.get("roll") is None for v in es.values()):
                        continue
                    d_o = es[("O", MU[1])]["roll"]["rotation_deg"] - es[("O", MU[0])]["roll"]["rotation_deg"]
                    if abs(d_o) < MIN_D or not all(es[(hand, m)]["roll"]["held"] for m in MU):
                        continue
                    d_h = es[(hand, MU[1])]["roll"]["rotation_deg"] - es[(hand, MU[0])]["roll"]["rotation_deg"]
                    qo = {m: run_quantities(o, es[("O", m)], r) for m in MU}
                    qh = {m: run_quantities(h, es[(hand, m)], r) for m in MU}
                    q_cmd = qo[MU[0]]["q_cmd"]
                    tips_o = [np.mean([qo[m]["tip0"] for m in MU]), np.mean([qo[m]["tip1"] for m in MU])]
                    tips_h = [np.mean([qh[m]["tip0"] for m in MU]), np.mean([qh[m]["tip1"] for m in MU])]
                    dev = lambda a, b: abs(a / b - 1.0) if b > 1e-9 else float("nan")  # noqa: E731
                    pairs.append({
                        "s": s, "r": r, "key": list(k), "rel": abs(d_h - d_o) / abs(d_o),
                        "local_dc_at_O": local_dc(o, h, q_cmd, s, tips_o),
                        "local_dc_at_H": local_dc(o, h, q_cmd, s, tips_h),
                        "tip_dc_swept": local_dc(o, h, q_cmd, s, [0.0, 0.0]),
                        "local_dc_own": local_dc(o, h, q_cmd, s, tips_o, tips_h),
                        "travel_d_H": float(np.mean([qh[m]["travel_d"] for m in MU])),
                        "travel_d_O": float(np.mean([qo[m]["travel_d"] for m in MU])),
                        "tip_H": tips_h, "tip_O": tips_o,
                        "off_pad": not all(q["on_pad"] for q in (*qo.values(), *qh.values())),
                        "pre_dev_distal": float(np.mean([dev(qh[m]["n_pre"][1], qo[m]["n_pre"][1]) for m in MU])),
                        "pre_dev_sum": float(np.mean([dev(qh[m]["n_pre"].sum(), qo[m]["n_pre"].sum()) for m in MU])),
                        "end_dev_distal": float(np.mean([dev(qh[m]["n_end"][1], qo[m]["n_end"][1]) for m in MU])),
                        "end_dev_sum": float(np.mean([dev(qh[m]["n_end"].sum(), qo[m]["n_end"].sum()) for m in MU])),
                        "slip_p": {f"{n}_mu{m:g}": q[m]["slip_p"] for n, q in (("H", qh), ("O", qo)) for m in MU},
                        "slip_d": {f"{n}_mu{m:g}": q[m]["slip_d"] for n, q in (("H", qh), ("O", qo)) for m in MU},
                        "roll_p": {f"{n}_mu{m:g}": q[m]["roll_p"] for n, q in (("H", qh), ("O", qo)) for m in MU},
                        "along": {f"{n}_mu{m:g}": q[m]["along"] for n, q in (("H", qh), ("O", qo)) for m in MU},
                        # the friction effect on slip: slip(mu 2) - slip(mu 1.3), H's minus O's
                        "d_slip_p_dev": abs((qh[2.0]["slip_p"] - qh[1.3]["slip_p"]) - (qo[2.0]["slip_p"] - qo[1.3]["slip_p"])),
                        "depth_range": [min(q[m][d] for q in (qo, qh) for m in MU for d in ("depth0", "depth1")),
                                        max(q[m][d] for q in (qo, qh) for m in MU for d in ("depth0", "depth1"))],
                    })
                    o_pairs[(s, r, k)] = qo
        if not pairs:
            continue
        med = lambda key: float(np.median([p[key] for p in pairs]))  # noqa: E731
        med_abs = lambda key, sub: float(np.median([abs(p[key][sub]) for p in pairs]))  # noqa: E731
        row = {"hand": hand, "E": scored[hand]["E"], "E_check": med("rel"), "pairs": len(pairs),
               "d_contact": DESIGN[hand][0], "d_skel": DESIGN[hand][1],
               **{k: med(k) for k in ("local_dc_at_O", "local_dc_at_H", "tip_dc_swept", "local_dc_own", "pre_dev_distal",
                                       "pre_dev_sum", "end_dev_distal", "end_dev_sum", "d_slip_p_dev")},
               "travel_d_H_mm": float(np.median([abs(p["travel_d_H"]) for p in pairs])),
               "travel_d_O_mm": float(np.median([abs(p["travel_d_O"]) for p in pairs])),
               "travel_d_H_signed_mm": med("travel_d_H"),
               "off_pad_pairs": sum(p["off_pad"] for p in pairs),
               **{f"slip_p_{k}_mm": med_abs("slip_p", k) for k in ("H_mu1.3", "H_mu2", "O_mu1.3", "O_mu2")},
               **{f"slip_d_{k}_mm": med_abs("slip_d", k) for k in ("H_mu1.3", "H_mu2", "O_mu1.3", "O_mu2")},
               **{f"roll_p_{k}_mm": med_abs("roll_p", k) for k in ("H_mu1.3", "H_mu2")},
               **{f"along_{k}_mm": med_abs("along", k) for k in ("H_mu1.3", "H_mu2")},
               "depth_range_mm": [min(p["depth_range"][0] for p in pairs), max(p["depth_range"][1] for p in pairs)],
               "pair_detail": pairs}
        rows.append(row)
        print(f"{hand:22s} E {100 * row['E']:5.1f}%/{100 * row['E_check']:5.1f} ({row['pairs']}) "
              f"depth {row['depth_range_mm'][0]:.1f}-{row['depth_range_mm'][1]:.1f} off-pad {row['off_pad_pairs']} "
              f"along {row['along_H_mu1.3_mm']:.2f}/{row['along_H_mu2_mm']:.2f} | Dc {row['d_contact']:4.2f} local@O "
              f"{row['local_dc_at_O']:5.2f} own {row['local_dc_own']:5.2f} tip-swept {row['tip_dc_swept']:5.2f} | "
              f"travel H {row['travel_d_H_mm']:5.2f} O {row['travel_d_O_mm']:5.2f} mm | pre dev d "
              f"{row['pre_dev_distal']:.2f} sum {row['pre_dev_sum']:.2f} end d {row['end_dev_distal']:.2f} | slip_p H "
              f"{row['slip_p_H_mu1.3_mm']:.2f}/{row['slip_p_H_mu2_mm']:.2f} slip_d H {row['slip_d_H_mu1.3_mm']:.2f}/"
              f"{row['slip_d_H_mu2_mm']:.2f} roll {row['roll_p_H_mu1.3_mm']:.2f} mm", flush=True)

    n = len(rows)
    E = np.array([x["E"] for x in rows])
    col = lambda k: np.array([x[k] for x in rows])  # noqa: E731
    stats = {}
    for k in ("d_contact", "d_skel", "local_dc_at_O", "local_dc_at_H", "tip_dc_swept", "local_dc_own", "travel_d_H_mm",
              "pre_dev_distal", "pre_dev_sum", "end_dev_distal", "end_dev_sum", "d_slip_p_dev",
              "slip_p_H_mu1.3_mm", "slip_d_H_mu1.3_mm"):
        v = col(k)
        stats[f"rho_E_{k}"] = {"rho": spearman(v, E), "ci95": boot(lambda i, v=v: spearman(v[i], E[i]), n)}
    for z in ("pre_dev_distal", "pre_dev_sum", "end_dev_sum"):
        zv, dc = col(z), col("d_contact")
        stats[f"partial_E_d_contact_given_{z}"] = {
            "rho": partial_spearman(E, dc, zv), "ci95": boot(lambda i: partial_spearman(E[i], dc[i], zv[i]), n)}
        stats[f"partial_E_{z}_given_d_contact"] = {
            "rho": partial_spearman(E, zv, dc), "ci95": boot(lambda i: partial_spearman(E[i], zv[i], dc[i]), n)}
    stats["rho_d_contact_pre_dev_distal"] = spearman(col("d_contact"), col("pre_dev_distal"))
    stats["rho_d_contact_end_dev_sum"] = spearman(col("d_contact"), col("end_dev_sum"))
    stats["rho_d_skel_end_dev_sum"] = spearman(col("d_skel"), col("end_dev_sum"))
    stats["rho_pre_dev_sum_end_dev_sum"] = spearman(col("pre_dev_sum"), col("end_dev_sum"))
    stats["rho_d_contact_local_dc_at_O"] = spearman(col("d_contact"), col("local_dc_at_O"))
    o_slip_p = [abs(q[m]["slip_p"]) for q in o_pairs.values() for m in MU]
    o_roll_p = [abs(q[m]["roll_p"]) for q in o_pairs.values() for m in MU]
    o_travel = [abs(q[m]["travel_d"]) for q in o_pairs.values() for m in MU]
    stats["O"] = {"pairs": len(o_pairs), "slip_p_median_mm": float(np.median(o_slip_p)),
                  "roll_p_median_mm": float(np.median(o_roll_p)), "travel_d_median_mm": float(np.median(o_travel)),
                  "travel_d_range_mm": [float(min(o_travel)), float(max(o_travel))]}
    for k, v in stats.items():
        if isinstance(v, dict) and "rho" in v:
            print(f"{k:45s} {v['rho']:+.2f} [{v['ci95'][0]:+.2f}, {v['ci95'][1]:+.2f}]")
        else:
            print(k, v)
    args.out.write_text(json.dumps({"note": "experiment 22 post hoc, not used for any judgement. d_slip_p_dev is partly E by construction (slip_p = "
                                    "along - rho * rotation), so its correlation is descriptive only. At equal flexion "
                                    "H's and O's distal phalanges have the same orientation, so their difference is one "
                                    "translation: local_dc_at_O = local_dc_at_H = tip_dc_swept identically.", "hands": n,
                                    "stats": stats, "rows": rows}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
