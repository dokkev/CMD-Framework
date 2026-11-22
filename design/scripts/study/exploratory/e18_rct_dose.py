#!/usr/bin/env python3
"""Experimental (playground): CFL redesigns for a thicker pad at chosen sizes of relative-contact-trajectory change.

    python scripts/study/exploratory/e18_rct_dose.py [--flexion 120] [--to 7] [--targets 1.5 2.25 3.0 4.5]
        [--seeds 0] [--generations 200] [--tolerance 0.1]

Study 2's front (least change D_c, most coverage C_pad) keeps only the redesigns no other beats, so at 120 deg,
3 -> 7 mm it holds D_c 0.84-0.92 mm alone: a redesign with a larger change and no more coverage is dominated. A
dose-response test (does an interface measurement's error follow D_c, and not the skeleton's change D_s?) needs
redesigns spread in D_c at near-best coverage. For each target t this searches (NSGA-II, Study 2's settings) for
the least |D_c - t| and the most C_pad under the thicker pad's Study 1 goal, warm-started from H_R, H_U and the
previous targets' picks, and keeps the most-coverage design with |D_c - t| <= tolerance.
With --skeleton min/max a third objective drives D_s down or up and the pick is the least/most D_s among
the front's designs within the tolerance at C_pad >= --coverage_floor: designs that vary D_c and D_s apart.

Also evaluates the naive swap: H0's own design carrying the thicker pads, unchanged (the usual practice: same
hand, new pads), and lists H_R and H_U. Writes ``output/study/exploratory/e18_rct_dose/designs.csv`` with Study 2's
columns (D_c, D_s = skeleton_change, C_pad, MA_wc, x).
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
for path in (ROOT, ROOT / "scripts" / "study", ROOT / "scripts" / "study" / "study2_morphology_adaptation", HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
import e15_transmission_change as e15  # noqa: E402
import redesign as study2  # noqa: E402
import shared  # noqa: E402
from core.optimization.analysis.contact_kinematics import INVALID, ContactMechanism, Redesigned, _picklable, path_change, paths  # noqa: E402
from core.optimization.mechanism import Objective, mechanism, read_config_file  # noqa: E402
from core.optimization.optimizer import OptimizerSettings  # noqa: E402
from core.optimization.study import run_study  # noqa: E402

OUT = ROOT / "output" / "study" / "exploratory" / "e18_rct_dose"


@dataclasses.dataclass(frozen=True)
class WithSkeleton(Redesigned):
    skeleton_change: float = INVALID


class TargetMechanism(ContactMechanism):
    """The contact redesign with its first objective the distance of D_c (mm) from ``target_mm``; with ``skeleton``
    "min" or "max" a third objective drives the skeleton's change D_s (the fingertip path's change) down or up."""

    def __init__(self, base, reference, scale_mm: float, target_mm: float, skeleton: str = "none",
                 reference_tip=None) -> None:
        super().__init__(base, reference, scale_mm)
        self.target_mm, self.skeleton = float(target_mm), skeleton
        self.reference_tip = None if reference_tip is None else np.asarray(reference_tip, dtype=float)
        self.name, self.formulation = f"target:{base.name}", f"{base.formulation}+contact-target-v1"
        scale, target = self.scale_mm, self.target_mm
        self.objectives = {
            "contact_change": Objective(
                lambda e: INVALID if e.contact_change >= INVALID else abs(e.contact_change * scale - target),
                "contact_change", "|D_c - t|", "on_target", False),
            "c_pad_mean": Objective(lambda e: -e.c_pad_mean, "c_pad_mean", "C_pad", "most_coverage", True)}
        if skeleton != "none":
            sign = 1.0 if skeleton == "min" else -1.0
            self.objectives["skeleton_change"] = Objective(
                lambda e: INVALID if e.skeleton_change >= INVALID else sign * e.skeleton_change,
                "skeleton_change", "D_s", f"{skeleton}_skeleton", skeleton == "max")

    def evaluate(self, x, config):
        if self.skeleton == "none":
            return super().evaluate(x, config)
        try:
            base = self.base.evaluate(x, config)
        except ValueError:
            return None
        if base is None:
            return None
        base = _picklable(base)
        change, coverage, skel = INVALID, 0.0, INVALID
        if base.feasible:
            found = paths(self.base, x, config, base, len(self.reference))
            if found is not None:
                change = path_change(found["contact"], self.reference, self.scale_mm)
                skel = path_change(found["tip"], self.reference_tip, self.scale_mm)
            value = self.base.report(base).get("c_pad_mean")
            coverage = float(value) if value not in (None, "") and math.isfinite(float(value)) else 0.0
        return WithSkeleton(base, change, coverage, dict(base.constraints), skel)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexion", type=float, default=120.0)
    parser.add_argument("--to", type=float, default=7.0)
    parser.add_argument("--targets", type=float, nargs="+", default=(1.5, 2.25, 3.0, 4.5))
    parser.add_argument("--seeds", type=int, nargs="+", default=(0,))
    parser.add_argument("--population", type=int, default=200)
    parser.add_argument("--generations", type=int, default=200)
    parser.add_argument("--tolerance", type=float, default=0.1, help="mm")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--skeleton", nargs="+", default=("none",), choices=("none", "min", "max"),
                        help="per target, one search per entry: none (D_c and C_pad), min/max (also D_s)")
    parser.add_argument("--coverage_floor", type=float, default=0.70, help="picks need C_pad at least this")
    parser.add_argument("--out", type=Path, default=OUT / "designs.csv")
    args = parser.parse_args(argv)

    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    spec = config["architectures"]["cfl"]
    designs = e15.study1.study1_designs("cfl", spec, args.flexion, config)
    h0, goal = designs[3.0], designs[args.to]
    base = mechanism(h0.mechanism)
    reference = paths(base, h0.x, h0.config)
    scale = float(base.report(base.evaluate(h0.x, h0.config))["finger_length_mm"])
    with (e15.STUDY2 / "front.csv").open(encoding="utf-8", newline="") as stream:
        front = [r for r in csv.DictReader(stream) if r["architecture"] == "cfl"
                 and float(r["required_flexion_deg"]) == args.flexion and float(r["to_mm"]) == args.to]
    x_r = tuple(float(v) for v in max(front, key=lambda r: float(r["c_pad_mean"]))["x"].split(";"))
    folder = e15.study1.condition_folder("cfl", spec, args.flexion, args.to, float(config["native_thickness_mm"]))
    x_u = e15.balanced_x(sorted(folder.glob("portfolio_*"))[0])
    head = {"architecture": "cfl", "required_flexion_deg": args.flexion, "from_mm": 3.0, "to_mm": args.to}
    plain = ContactMechanism(base, reference["contact"], scale)
    rows = []

    def add(label: str, x, cfg, seed="") -> dict | None:
        e = plain.evaluate(x, cfg)
        if e is None or not e.feasible or e.contact_change >= INVALID:
            print(f"{label}: not feasible with the {args.to:g} mm pads"
                  + ("" if e is None else f" (constraints {dict(e.constraints)})"))
            rows.append({**head, "label": label, "feasible": False, "seed": seed})
            return None
        row = {**head, "label": label, "feasible": True, **study2._row({}, seed, x, e, base, cfg, reference, scale)}
        rows.append(row)
        print(f"{label}: D_c {row['contact_change_mm']:.2f} mm, D_s {100 * row['skeleton_change']:.2f}% L_f "
              f"({row['skeleton_change'] * scale:.2f} mm), C_pad {row['c_pad_mean']:.4f}, MA_wc {row['ma_wc']:.3f}")
        return row

    add("H_R", x_r, goal.config)
    add("H_U", x_u, goal.config)
    add("naive swap (H0 x, thicker pads)", h0.x, goal.config)
    warm = [x_r, x_u]
    for target in args.targets:
        for seed, mode in ((s, m) for s in args.seeds for m in args.skeleton):
            t0 = time.time()
            mech = TargetMechanism(base, reference["contact"], scale, target, mode, reference["tip"])
            result = run_study(mech, goal.config, OptimizerSettings("nsga2", args.population, args.generations, seed),
                               initial_designs=warm, workers=args.workers)
            near = [(tuple(c.x), c.evaluation) for c in result.pareto if c.evaluation.feasible
                    and c.evaluation.contact_change < INVALID
                    and abs(c.evaluation.contact_change * scale - target) <= args.tolerance
                    and c.evaluation.c_pad_mean >= args.coverage_floor]
            label = f"target {target:g}" + ("" if mode == "none" else f" {mode}-Ds")
            print(f"{label}, seed {seed}: {len(result.pareto)} on the front, {len(near)} within "
                  f"{args.tolerance:g} mm at C_pad >= {args.coverage_floor:g} ({time.time() - t0:.0f} s)", flush=True)
            if not near:
                rows.append({**head, "label": label, "feasible": False, "seed": seed})
                continue
            if mode == "none":
                x, _ = max(near, key=lambda f: f[1].c_pad_mean)
            else:
                pick = min if mode == "min" else max
                x, _ = pick(near, key=lambda f: f[1].skeleton_change)
            add(label, x, goal.config, seed)
            warm.append(x)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    shared.write_csv(args.out, rows)
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
