#!/usr/bin/env python3
"""The transfer study's long fingers: UL (coverage alone) and RL (the contact path of O kept), L_f in [120, 150] mm.

    OPTIMIZE_WORKERS=3 python scripts/optimize/run.py scripts/study/study3_trajectory/cfl_long.yaml --tag q120_h7
    python scripts/study/study3_trajectory/long_fingers.py [--workers 3] [--resume]

The reference O is Study 1's CFL for the 3 mm pad at q_req 120 deg (90 mm, ``cfl_q120_h3to7_h0``). Both long fingers
carry 7 mm pads under the studies' CFL goal with the finger length searched in [120, 150] mm (``cfl_long.yaml``):

- UL: Study 1's objective (Eq. opt1, the most C_pad), the balanced design of the first portfolio of ``cfl_long_q120_h7``
  (as U was chosen from ``cfl_q120_h7``);
- UL120: UL's search with L_f held at 120 mm (``cfl_long_q120_h7_L120``), RL's coverage-only partner at its length
  (UL at 150 mm is kept as a secondary hand);
- RL60: the same front's least change with C_pad >= 0.60 (``--extra-floors``), the contact-preserving partner of
  UL120 (RL itself is barely apart from UL120 in Delta_J);
- RL: Study 2's redesign (Eq. opt2) against O, NSGA-II for the least change and the most C_pad, Study 2's search
  (one_finger.yaml ``redesign.search``), warm-started from O and UL; the least change with C_pad at least O's.

A longer finger's fingertip lies |L_f - 90| further out at q = 0, so Delta_contact against O is at least about that
whatever the joint: with Study 2's Delta_contact (``--objective literal``) the front only trades finger length for
coverage (Delta_contact = L_f - 90 along it) and its most-coverage end is UL. The default objective is the change of the
contact path from a common start, ``Delta_J = max_q |dc(q) - dc(0)|`` (the contact Jacobian's share, the integral term
of Delta_contact in problem_formulation.md; ``contact_kinematics.anchored``). Its most-coverage end is UL too (the
coverage rises with L_f to the range's top), so RL keeps O's coverage instead (``--rl-pick``). The table gives
Delta_contact, its offset ``|dc(0)|`` and Delta_J, in mm against O.

Writes ``output/study/study3_long_fingers/`` (``fronts_<objective>.csv``, ``front_<objective>.csv``, ``designs.csv``)
and the hands
``output/sim/study3/hands/cfl_q120_L<L_f>_h3to7_{hu,hr}.yaml`` (make_hands.py's decoding and checks).
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import make_hands  # noqa: E402  (puts the design root and the study folders on the path)

from core.morphology.code_hand.parameters import PROTOTYPE  # noqa: E402
from core.morphology.mechanism.linkage.design import decode  # noqa: E402
from core.optimization.analysis.contact_kinematics import front, path_change, paths, redesign  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.optimizer import OptimizerSettings  # noqa: E402
from core.optimization.study import load_config  # noqa: E402

ROOT = make_hands.ROOT
e15 = make_hands.e15
UL_FOLDER = ROOT / "output" / "study" / "linkage" / "cfl_long_q120_h7"
UL120_FOLDER = ROOT / "output" / "study" / "linkage" / "cfl_long_q120_h7_L120"
"""UL120: UL's search with L_f held at 120 mm (``--set "goal.link.finger_length_mm=[119.95, 120.05]" --tag
q120_h7_L120``), the coverage-only partner of RL at RL's length."""
OUT = ROOT / "output" / "study" / "study3_long_fingers"
FLEXION, THIN, THICK = 120.0, 3.0, 7.0


def read_rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def write_rows(path: Path, rows: list[dict]) -> None:
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, keys)
        writer.writeheader()
        writer.writerows(rows)


def _p(point) -> tuple[float, float]:
    return round(point.x_mm, 2), round(point.z_mm, 2)


def _d(p, q) -> float:
    return math.hypot(q.x_mm - p.x_mm, q.z_mm - p.z_mm)


def measures(base, x, config, reference, scale: float) -> dict:
    """One design against O: geometry, coverage, Delta_contact / Delta_skel (mm) and the transmission."""

    evaluation = base.evaluate(x, config)
    report = base.report(evaluation)
    own = paths(base, x, config, evaluation, len(reference["contact"]))
    dc = own["contact"] - reference["contact"]
    finger = decode(x, config.link_length_resolution_mm)
    j = finger.joint
    return {
        "feasible": bool(evaluation.feasible), "L_f_mm": float(report["finger_length_mm"]),
        "L1_mm": math.hypot(j.b.x_mm, j.b.z_mm), "L2_mm": float(finger.distal.end_mm),
        "A": _p(j.a), "Ap": _p(j.ap), "B": _p(j.b), "Bp": _p(j.bp),  # proximal frame, extended
        "AAp_mm": _d(j.a, j.ap), "ApB_mm": _d(j.ap, j.b), "ABp_mm": _d(j.a, j.bp), "BBp_mm": _d(j.b, j.bp),
        "tilt_deg": float(x[7]),  # the design vector's servo tilt (make_hands.py)
        "c_pad_mean": float(report["c_pad_mean"]),
        "pads_mm": (float(report["proximal_support_length_mm"]), float(report["distal_support_length_mm"])),
        "pad_share": (float(report["proximal_support_length_mm"]) + float(report["distal_support_length_mm"]))
        / float(report["finger_length_mm"]),
        "delta_contact_mm": float(np.max(np.linalg.norm(dc, axis=1))),
        "delta_contact_q0_mm": float(np.linalg.norm(dc[0])),
        "delta_contact_rest_mm": float(np.max(np.linalg.norm(dc - dc[0], axis=1))),
        "delta_skel_mm": path_change(own["tip"], reference["tip"], 1.0),
        "q_max_deg": float(report["q_max_deg"]), "ma_wc": float(report["ma_wc"]),
        "gamma_deg": (float(report.get("gamma_min_deg", math.nan)), float(report.get("gamma_max_deg", math.nan))),
        "bound_hits": report.get("bound_hits", ""),
        "x": ";".join(f"{v:.6g}" for v in x),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--resume", action="store_true", help="reuse fronts_<objective>.csv's seeds")
    parser.add_argument("--objective", choices=("anchored", "literal"), default="anchored",
                        help="RL's change: of the contact path from a common start (the contact Jacobian's, default) "
                             "or of the path itself (Study 2's Delta_contact)")
    parser.add_argument("--rl-pick", choices=("floor", "most"), default="floor",
                        help="RL from the front: the least change with C_pad >= the floor (default), or the most C_pad")
    parser.add_argument("--coverage-floor", type=float, default=None, help="default: O's C_pad")
    parser.add_argument("--extra-floors", type=float, nargs="*", default=[0.60],
                        help="also RL<100 c> (hand ..._hr<100 c>): the least change with C_pad >= c, for each c")
    args = parser.parse_args(argv)
    study = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    spec = study["architectures"]["cfl"]
    designs = e15.study1.study1_designs("cfl", spec, FLEXION, study)
    o, goal7 = designs[THIN], designs[THICK]
    base = mechanism(o.mechanism)
    reference = paths(base, o.x, o.config)
    scale = float(base.report(base.evaluate(o.x, o.config))["finger_length_mm"])

    ul_portfolio = sorted(UL_FOLDER.glob("portfolio_*"))[0]
    ul_config = load_config(sorted(UL_FOLDER.glob("run_*"))[0])
    x_ul = e15.balanced_x(ul_portfolio)
    ul120 = None
    if sorted(UL120_FOLDER.glob("portfolio_*")):
        ul120 = (e15.balanced_x(sorted(UL120_FOLDER.glob("portfolio_*"))[0]),
                 load_config(sorted(UL120_FOLDER.glob("run_*"))[0]))

    OUT.mkdir(parents=True, exist_ok=True)
    search = study["redesign"]["search"]
    fronts, front_file = OUT / f"fronts_{args.objective}.csv", OUT / f"front_{args.objective}.csv"
    kept = read_rows(fronts) if args.resume else []
    done = {int(r["seed"]) for r in kept}
    for seed in search["seeds"]:
        if int(seed) in done:
            continue
        start = time.time()
        found, count = redesign(base, ul_config, reference["contact"], scale, OptimizerSettings(
            search["algorithm"], int(search["population"]), int(search["generations"]), int(seed)),
            [o.x, x_ul], args.workers, anchor=args.objective == "anchored")
        print(f"RL seed {seed}: {count} evaluations in {time.time() - start:.0f} s, {len(found)} on its front; "
              f"least change {found[0][1].contact_change * scale:.2f} mm at C_pad {found[0][1].c_pad_mean:.3f}, "
              f"most C_pad {found[-1][1].c_pad_mean:.3f} at {found[-1][1].contact_change * scale:.2f} mm", flush=True)
        kept += [{"seed": seed, "contact_change_mm": e.contact_change * scale, "c_pad_mean": e.c_pad_mean,
                  "finger_length_mm": base.report(e.base)["finger_length_mm"], "x": ";".join(f"{v:.6g}" for v in x)}
                 for x, e in found]
        write_rows(fronts, kept)
    merged = [kept[i] for i in front([(float(r["contact_change_mm"]), float(r["c_pad_mean"])) for r in kept])]
    write_rows(front_file, merged)
    # RL: the least change that keeps O's coverage. The front's most-coverage end (R's rule) is UL itself here: the
    # coverage rises with L_f to the range's top, where one design holds it.
    floor = float(base.report(base.evaluate(o.x, o.config))["c_pad_mean"]) if args.coverage_floor is None \
        else args.coverage_floor
    pick = merged[-1] if args.rl_pick == "most" else next(r for r in merged if float(r["c_pad_mean"]) >= floor)
    print(f"RL: {args.rl_pick} (C_pad floor {floor:.3f}): change {float(pick['contact_change_mm']):.2f} mm, "
          f"C_pad {float(pick['c_pad_mean']):.4f}, L_f {float(pick['finger_length_mm']):.1f} mm")
    x_rl = tuple(float(v) for v in pick["x"].split(";"))
    extra = []  # RL60: a lower floor, for a real contrast in Delta_J to UL120 at the same length
    for c in args.extra_floors:
        row = next(r for r in merged if float(r["c_pad_mean"]) >= c)
        extra.append((f"hr{round(100 * c):d}", f"RL{round(100 * c):d}", tuple(float(v) for v in row["x"].split(";"))))

    # The 7 mm R/U of the 90 mm pair, for comparison (make_hands.py's choice).
    with (e15.STUDY2 / "front.csv").open(encoding="utf-8", newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r["architecture"] == "cfl"
                and float(r["required_flexion_deg"]) == FLEXION and float(r["to_mm"]) == THICK]
    x_r7 = tuple(float(v) for v in max(rows, key=lambda r: float(r["c_pad_mean"]))["x"].split(";"))
    x_u7 = e15.balanced_x(sorted(e15.study1.condition_folder(
        "cfl", spec, FLEXION, THICK, float(study["native_thickness_mm"])).glob("portfolio_*"))[0])

    table = []
    for name, x, cfg in (("O", o.x, o.config), ("R7", x_r7, goal7.config), ("U7", x_u7, goal7.config),
                         ("RL", x_rl, ul_config), *((label, x, ul_config) for _, label, x in extra),
                         ("UL", x_ul, ul_config),
                         *((("UL120", *ul120),) if ul120 else ())):
        table.append({"hand": name, **measures(base, x, cfg, reference, scale)})
    write_rows(OUT / "designs.csv", table)
    print(f"\nRL front ({args.objective}): {len(merged)} points, change {float(merged[0]['contact_change_mm']):.2f}.."
          f"{float(merged[-1]['contact_change_mm']):.2f} mm, C_pad {float(merged[0]['c_pad_mean']):.3f}.."
          f"{float(merged[-1]['c_pad_mean']):.3f}")
    print(f"{'hand':4s} {'L_f':>6s} {'L1':>6s} {'L2':>6s} {'tilt':>5s} {'C_pad':>6s} {'pads P+D':>11s} {'Dc':>6s} "
          f"{'Dc(0)':>6s} {'Dc-rest':>7s} {'Dskel':>6s} {'q_max':>6s} {'MA_wc':>6s} feas")
    for r in table:
        print(f"{r['hand']:4s} {r['L_f_mm']:6.1f} {r['L1_mm']:6.1f} {r['L2_mm']:6.1f} {r['tilt_deg']:5.0f} "
              f"{r['c_pad_mean']:6.3f} {r['pads_mm'][0]:5.1f}+{r['pads_mm'][1]:5.1f} {r['delta_contact_mm']:6.2f} "
              f"{r['delta_contact_q0_mm']:6.2f} {r['delta_contact_rest_mm']:7.2f} {r['delta_skel_mm']:6.2f} "
              f"{r['q_max_deg']:6.1f} {r['ma_wc']:6.3f} {r['feasible']}")
        print(f"     A {r['A']}, A' {r['Ap']}, B {r['B']}, B' {r['Bp']}; |AA'| {r['AAp_mm']:.1f}, "
              f"|A'B| {r['ApB_mm']:.1f}, |AB'| {r['ABp_mm']:.1f}, |BB'| {r['BBp_mm']:.1f}; "
              f"gamma {r['gamma_deg'][0]:.0f}..{r['gamma_deg'][1]:.0f}; bounds {r['bound_hits'] or '-'}")

    # The hands, as make_hands.py writes them (decoded, then checked against the study's kinematics).
    o_base = mechanism(o.mechanism)
    ok = True
    for suffix, label, x, cfg in (("hu", "UL", x_ul, ul_config), ("hr", "RL", x_rl, ul_config),
                                  *((suffix, label, x, ul_config) for suffix, label, x in extra),
                                  *((("hu", "UL120", *ul120),) if ul120 else ())):
        finger_params, pads = make_hands.hand_parameters(x, cfg, THICK)
        tip_err, contact_err = make_hands.check(o_base, x, cfg, finger_params, pads, math.radians(FLEXION))
        length = next(r["L_f_mm"] for r in table if r["hand"] == label)
        name = f"cfl_q120_L{round(length):d}_h3to7_{suffix}"
        print(f"{label} -> {name}: tip {tip_err:.2e} mm, contact {contact_err:.2e} mm from the study's")
        if max(tip_err, contact_err) > 1e-6:
            ok = False
            continue
        rel = Path("../../../../core/morphology/code_hand") / PROTOTYPE.name
        doc = {"base": str(rel), "finger": finger_params, "pads": pads,
               "source": {"study": "Study 3 transfer, long fingers (scripts/study/study3_trajectory/long_fingers.py)",
                          "hand": label, "design": [round(v, 6) for v in x], "required_flexion_deg": FLEXION,
                          "pad_thickness_mm": THICK, "finger_length_mm": round(length, 3)}}
        path = make_hands.OUT / f"{name}.yaml"
        make_hands.OUT.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(doc, sort_keys=False))
        print(f"  -> {path.relative_to(ROOT)}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
