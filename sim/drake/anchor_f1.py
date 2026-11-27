"""The Drake fingertip tasks anchored at F_1 (experimental; docs/experimental/pr_mr.md, "Pre-registration: Drake fingertip
tasks anchored at F_1").

The paper's fingertip dial (experiment 29, randomized starts, seed 29) and block pivot (experiment 35, seed 35) place
the object in each hand's proximal-face frame (anchor 'face'). With --anchor f1 the same objects sit at O's
coordinates in F_1 (the link-line frame, the world here, the MCP held) on every hand. Commands, gates, the friction
swap 1.3 -> 2.0 and E are the experiments' own (analyze_roll.tip_error, box_pivot.hand_error).

Per hand, from its parameter file over q in [0, 120 deg], the distal contact face's point under the fingertip c(q):
- Delta_contact: max |c_H - c_O| with c from the proximal face's line (the paper's);
- Delta_MCP: the same in F_1 (c without the proximal face's depth), the path the distal pad takes around the MCP axis;
- Delta_skel: the same for the fingertip C on the distal link line.
'design' uses the parameter file's distal mount offset where it has one (the study's values); 'sim' the depth the
Drake model gives the distal pad (it mounts both pads at mount_offset_mm).

    uv run python anchor_f1.py deltas
    uv run python anchor_f1.py analyze --task tipdial_y --tag e29y_f1 [--compare e29y] [--out logs/e29y_f1.json]
    uv run python anchor_f1.py analyze --task pivot --tag e35_f1 [--compare e35] [--out logs/e35_f1.json]
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import analyze_roll as AR
import box_pivot as BP
import two_pad_drake as T
from analyze import DESIGN, spearman_perm

LOGS = Path(__file__).resolve().parent / "logs"
Q_DEG = np.arange(0.0, 120.0 + 1e-9, 0.25)


MR = ["ms_e22", "mrs_e22", "lowm_s4_e21", "highm_m3_e21", "highm_m4p5_e21", "mr_bz", "mrmid_bz", "mr_free", "mrmid_free", "h3to5_mr_free", "h3to5_mrmid_free", "h3to9_mr_free",
      "h3to9_mrmid_free"]  # e20's picks (B, or A and B, off the link line)
HANDS = [*DESIGN, *MR]


def points(hand: str, distal: str = "design", q_deg=Q_DEG) -> dict:
    """The fingertip C and the distal-face point under it (from the link line, F_1) over q_deg, mm."""
    g = T.finger_geometry(hand)
    qs, th = T.servo_table(g)
    pads = g["pads"]
    face = pads["mount_offset_mm"] + pads["thickness_mm"]
    d_d = (pads.get("distal_mount_offset_mm", pads["mount_offset_mm"]) if distal == "design"
           else pads["distal_mount_mm"]) + pads["thickness_mm"]
    tip, f1 = [], []
    for q in np.radians(np.asarray(q_deg, dtype=float)):
        # reference_point(x) = B + R(q) (x, -distal face) + (0, face): B from x = 0
        b = (T.reference_point(g, qs, th, q, 0.0) - T.rot(q) @ np.array([0.0, -pads["distal_face_mm"]])
             - np.array([0.0, face]))
        c = b + T.rot(q) @ np.array([pads["distal_end_mm"], 0.0])
        tip.append(c)
        f1.append(c + T.rot(q) @ np.array([0.0, -d_d]))
    f1 = np.array(f1)
    return {"tip": np.array(tip), "f1": f1, "face": f1 + np.array([0.0, face]), "face_depth": face, "distal_depth": d_d}


def deltas(hands, distal: str = "design") -> dict:
    o = points("O", distal)
    out = {}
    for h in hands:
        p = points(h, distal)
        d = {k: float(np.max(np.linalg.norm(p[k] - o[k], axis=1))) for k in ("face", "f1", "tip")}
        out[h] = {"d_contact": d["face"], "d_mcp": d["f1"], "d_skel": d["tip"], "dt": p["face_depth"] - o["face_depth"],
                  "d_mcp_q0": float(np.linalg.norm(p["f1"][0] - o["f1"][0]))}
    return out


def cmd_deltas(a) -> int:
    hands = [h for h in HANDS if T.hand_file(h).exists()]
    des, sim = deltas(hands, "design"), deltas(hands, "sim")
    print(f"{'hand':22s} {'dt':>4s} | {'Dc':>5s} {'Dmcp':>5s} {'Dskel':>5s} (paper Dc, Ds) | sim: Dc Dmcp")
    for h in hands:
        d, s = des[h], sim[h]
        print(f"{h:22s} {d['dt']:4.1f} | {d['d_contact']:5.2f} {d['d_mcp']:5.2f} {d['d_skel']:5.2f} "
              f"({'(%.2f, %.2f)' % DESIGN[h] if h in DESIGN else 'new':>12s}) | {s['d_contact']:5.2f} {s['d_mcp']:5.2f}")
    path = LOGS / "anchor_f1_deltas.json"
    path.write_text(json.dumps({"design": des, "sim": sim}, indent=1))
    print(path)
    return 0


def errors(task: str, tag: str) -> dict:
    """E per redesigned hand (None: no run), with the experiment's own rules."""
    if task == "pivot":
        keep, excl = BP.o_keep(tag)
        rows = {h: BP.hand_error(h, tag, keep) for h in HANDS}
        return {"o_kept": len(keep), "o_excluded": excl, "rows": {h: r for h, r in rows.items() if r is not None}}
    AR.TASK, AR.RANDOM = task, True
    o = AR.tip_error("O", "point", tag)
    rows = {h: AR.tip_error(h, "point", tag) for h in HANDS}
    return {"o_kept": o["candidates"], "o_excluded": o["o_excluded"],
            "rows": {h: r for h, r in rows.items() if r is not None}}


def cmd_analyze(a) -> int:
    dl = json.loads((LOGS / "anchor_f1_deltas.json").read_text())
    res = {"task": a.task, "tag": a.tag}
    for label, tag in (("this", a.tag), ("compare", a.compare)):
        if tag is None:
            continue
        r = errors(a.task, tag)
        scored = [h for h, v in r["rows"].items() if v["E"] is not None]
        st = {}
        for set_, hands in (("paper", [h for h in scored if h in DESIGN]), ("paper+MR", scored)):
            for key in ("d_mcp", "d_contact", "d_skel"):
                for src in ("design", "sim"):
                    x = [dl[src][h][key] for h in hands]
                    rho, p = spearman_perm(x, [r["rows"][h]["E"] for h in hands])
                    st[f"{set_}:{key}_{src}"] = {"rho": rho, "p": p, "hands": len(hands)}
        res[label] = {"tag": tag, **r, "stats": st}
        print(f"{label} {tag}: O kept {r['o_kept']}, hands scored {len(scored)}")
        for key, v in st.items():
            print(f"  rho(E, {key:27s}) {v['rho']:+.2f}  one-sided p {v['p']:.4f}  ({v['hands']} hands)")
    print(f"{'hand':22s} {'Dc':>5s} {'Dmcp':>5s} {'Dskel':>5s} | " +
          " | ".join(f"E {res[k]['tag']} (pairs, over, lost)" for k in ("this", "compare") if k in res))
    for h in HANDS:
        if h not in dl["design"]:
            continue
        d = dl["design"][h]
        cells = []
        for k in ("this", "compare"):
            if k in res:
                v = res[k]["rows"].get(h)
                cells.append("-" if v is None or v["E"] is None else
                             f"{100 * v['E']:6.1f}% ({v['pairs']}, {v.get('over_pressed', 0)}, {v['lost']})")
        print(f"{h:22s} {d['d_contact']:5.2f} {d['d_mcp']:5.2f} {d['d_skel']:5.2f} | " + " | ".join(cells))
    if a.out:
        a.out.write_text(json.dumps(res, indent=1, default=float))
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("deltas")
    pa = sub.add_parser("analyze")
    pa.add_argument("--task", choices=("tipdial_y", "tipdial_z", "pivot"), required=True)
    pa.add_argument("--tag", required=True)
    pa.add_argument("--compare", default=None, help="the face-anchored tag to compare with")
    pa.add_argument("--out", type=Path, default=None)
    a = p.parse_args()
    return cmd_deltas(a) if a.cmd == "deltas" else cmd_analyze(a)


if __name__ == "__main__":
    raise SystemExit(main())
