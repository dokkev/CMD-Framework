"""Experiment 22 (in-hand rolling) analysis, as pre-registered in report/study3/experiments.md: per hand H, over
placements and both sweeps where O and H pass Gate 0 at both mu and |D(O)| >= 0.5 deg, D = rotation(2.0) -
rotation(1.3), E(H) = median |D(H) - D(O)| / |D(O)|; grasps lost by the end are counted apart. Spearman rho(E,
D_contact) and rho(E, D_skel) with exact one-sided permutation p over the redesigned hands present.

    uv run python analyze_roll.py --contact point --tag e22 [--out file.json]

Experiment 28 (the fingertip dial, task tipdial; --task tipdial_y or tipdial_z, as pre-registered): a pair is a
placement where O touches the dial (distal force > 0.2 N) at both mu, |D(O)| >= 0.3 deg, O's largest distal-pad
penetration (largest distal force / 2.5 kN/m) is <= 1.5 mm at both mu and O's proximal pad never touches the dial.
H not touching at both mu: lost (counted apart); H over 1.5 mm at either mu: over-pressed (counted apart, not scored).

    uv run python analyze_roll.py --task tipdial_y --tag e28y --noise_tag e28yn [--out file.json]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from analyze import DESIGN, spearman_perm

HERE = Path(__file__).resolve().parent
RADII, MU = (15.0, 20.0), (1.3, 2.0)
TASKS = {  # task: (file stem, variant tag prefix, variants, result key, metric, minimum |D(O)|, O must hold)
    "roll": ("dkroll", "s", (8, 15), "roll", "rotation_deg", 0.5, False),
    "pullin": ("dkpullin", "a", (3, 6), "pullin", "drawn_mm", 0.3, True),
    "dial": ("dkdial", "s", (8, 15), "dial", "rotation_deg", 0.3, True),
    "dial30": ("dkdial", "s", (45,), "dial", "rotation_deg", 0.3, True),  # experiment 26
}
TASK_RADII = {"dial30": (10.0, 12.0), "tipdial_y": (10.0, 12.0), "tipdial_z": (15.0,)}
TIP_PEN_MM, POINT_K = 1.5, 2.5  # experiment 28's penetration gate (mm) and the pad's point stiffness (N/mm)
PROX_TOUCH_N = 1e-3
TASK = "roll"


def load(hand, mu, contact, tag, s, r):
    stem, pre = TASKS[TASK][0], TASKS[TASK][1]
    p = HERE / "logs" / f"{stem}_{hand}_mu{mu:g}_{contact}-{tag}-{pre}{s}_r{r:g}.json"
    return {(e["q0_deg"], round(e["x_c_mm"], 3)): e for e in json.loads(p.read_text())["envs"]} if p.exists() else None


def error(hand, contact, tag):
    _, _, variants, rk, metric, min_effect, o_holds = TASKS[TASK]
    rel, lost, n = [], 0, 0
    for s in variants:
        for r in TASK_RADII.get(TASK, RADII):
            runs = {(h, m): load(h, m, contact, tag, s, r) for h in ("O", hand) for m in MU}
            if any(v is None for v in runs.values()):
                return None
            for k in runs[("O", MU[0])]:
                es = {hm: runs[hm].get(k) for hm in runs}
                if any(e is None or not e["gate0"]["pass"] or e.get(rk) is None for e in es.values()):
                    continue
                if o_holds and not all(es[("O", m)][rk]["held"] for m in MU):
                    continue
                d_o = es[("O", MU[1])][rk][metric] - es[("O", MU[0])][rk][metric]
                if abs(d_o) < min_effect:
                    continue
                n += 1
                if not all(es[(hand, m)][rk]["held"] for m in MU):
                    lost += 1
                    continue
                d_h = es[(hand, MU[1])][rk][metric] - es[(hand, MU[0])][rk][metric]
                rel.append(abs(d_h - d_o) / abs(d_o))
    return {"E": float(np.median(rel)) if rel else None, "pairs": len(rel), "lost": lost, "candidates": n}


def tip_load(hand, mu, contact, tag, r):
    p = HERE / "logs" / f"dktipdial_{hand}_mu{mu:g}_{contact}-{tag}_r{r:g}.json"
    if not p.exists():
        return None
    return {(e["rho_mm"], e["q0_deg"], round(e["x_c_mm"], 3), e["placement"].get("y_mm", 0.0)): e["dial"]
            for e in json.loads(p.read_text())["envs"]}


def tip_error(hand, contact, tag):
    """Experiment 28: E(H) and the validity record. hand may be 'O@tag' (O under another tag: the noise floor)."""
    h, _, htag = hand.partition("@")
    rel, lost, over, n, prox_touch, pen_h, clear_h, pen_o = [], 0, 0, 0, 0, [], [], []
    for r in TASK_RADII[TASK]:
        o = {m: tip_load("O", m, contact, tag, r) for m in MU}
        hh = {m: tip_load(h, m, contact, htag or tag, r) for m in MU}
        if any(v is None for v in (*o.values(), *hh.values())):
            return None
        for k in o[MU[0]]:
            os_ = [o[m][k] for m in MU]
            if not all(d["touched"] for d in os_):
                continue
            if any(d["distal_force_max_n"] / POINT_K > TIP_PEN_MM or d["proximal_force_max_n"] > PROX_TOUCH_N
                   for d in os_):
                continue
            d_o = os_[1]["rotation_deg"] - os_[0]["rotation_deg"]
            if abs(d_o) < 0.3:
                continue
            n += 1
            pen_o.append(max(d["distal_force_max_n"] for d in os_) / POINT_K)
            hs = [hh[m].get(k) for m in MU]
            if any(d is None or not d["touched"] for d in hs):
                lost += 1
                continue
            pen = max(d["distal_force_max_n"] for d in hs) / POINT_K
            pen_h.append(pen)
            clear_h.append(min(d["proximal_clearance_min_mm"] for d in hs))
            prox_touch += any(d["proximal_force_max_n"] > PROX_TOUCH_N for d in hs)
            if pen > TIP_PEN_MM:
                over += 1
                continue
            d_h = hs[1]["rotation_deg"] - hs[0]["rotation_deg"]
            rel.append(abs(d_h - d_o) / abs(d_o))
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "lost": lost, "over_pressed": over, "candidates": n,
            "proximal_touched": prox_touch, "pen_max_mm": float(max(pen_h)) if pen_h else None,
            "pen_median_mm": float(np.median(pen_h)) if pen_h else None,
            "proximal_clearance_min_mm": float(min(clear_h)) if clear_h else None,
            "o_pen_max_mm": float(max(pen_o)) if pen_o else None}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--task", choices=tuple(TASKS) + ("tipdial_y", "tipdial_z"), default="roll")
    p.add_argument("--contact", default="point")
    p.add_argument("--tag", default="e22")
    p.add_argument("--noise_tag", default=None, help="tipdial: O's run with a +0.05 deg command under this tag")
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()
    global TASK
    TASK = args.task
    if TASK.startswith("tipdial"):
        return tip_main(args)
    rows = []
    for hand, (dc, ds) in DESIGN.items():
        r = error(hand, args.contact, args.tag)
        if r is None or r["E"] is None:
            continue
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        print(f"{hand:22s} D_c {dc:4.2f} D_s {ds:5.2f} | {args.task} ({args.contact}): E {100 * r['E']:6.1f}% "
              f"({r['pairs']} pairs, {r['lost']} lost of {r['candidates']})", flush=True)
    if len(rows) >= 3:
        rho, pv = spearman_perm([x["d_contact"] for x in rows], [x["E"] for x in rows])
        rho_s, pv_s = spearman_perm([x["d_skel"] for x in rows], [x["E"] for x in rows])
        print(f"rho(E, D_contact) {rho:.2f} (one-sided p {pv:.4f}); rho(E, D_skel) {rho_s:.2f} (p {pv_s:.4f}); "
              f"hands {len(rows)}")
    if args.out:
        args.out.write_text(json.dumps(rows, indent=1))
    return 0


def tip_main(args) -> int:
    rows = []
    for hand, (dc, ds) in {"O": (0.0, 0.0), **DESIGN}.items():
        r = tip_error(hand, args.contact, args.tag)
        if r is None or (r["E"] is None and hand != "O"):
            continue
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        print(f"{hand:22s} D_c {dc:4.2f} D_s {ds:5.2f} | {args.task}: E "
              f"{'-' if r['E'] is None else f'{100 * r['E']:6.1f}%'} ({r['pairs']} pairs of {r['candidates']}; lost "
              f"{r['lost']}, over-pressed {r['over_pressed']}, proximal touched {r['proximal_touched']}) | pen max "
              f"{r['pen_max_mm']:.2f} mm, proximal clearance min {r['proximal_clearance_min_mm']:.1f} mm", flush=True)
    red = [x for x in rows if x["hand"] != "O"]
    out = {"rows": red, "O": next((x for x in rows if x["hand"] == "O"), None)}
    if args.noise_tag:
        nz = tip_error(f"O@{args.noise_tag}", args.contact, args.tag)
        out["noise_floor"] = {"tag": args.noise_tag, **nz}
        print(f"noise floor (O, command +0.05 deg, tag {args.noise_tag}): E {100 * nz['E']:.2f}% (75th percentile "
              f"{100 * nz['E_p75']:.2f}%, {nz['pairs']} pairs)")
    if len(red) >= 3:
        rho, pv = spearman_perm([x["d_contact"] for x in red], [x["E"] for x in red])
        rho_s, pv_s = spearman_perm([x["d_skel"] for x in red], [x["E"] for x in red])
        out.update(rho_contact=rho, p_contact=pv, rho_skel=rho_s, p_skel=pv_s)
        print(f"rho(E, D_contact) {rho:.2f} (one-sided p {pv:.4f}); rho(E, D_skel) {rho_s:.2f} (p {pv_s:.4f}); "
              f"hands {len(red)}")
    if args.out:
        args.out.write_text(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
