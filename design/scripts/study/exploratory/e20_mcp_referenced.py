#!/usr/bin/env python3
"""Experimental (playground): the MCP-referenced redesign MR, against the proximal-face-referenced one (PR = H_R).

    python scripts/study/exploratory/e20_mcp_referenced.py [--flexion 120] [--to 7] [--seeds 0 1]
        [--b-z -2 8] [--generations 200] [--coverage_floor 0.70]

Both redesigns keep the contact Jacobian: the rate at which the distal contact face's point under the fingertip moves
with the flexion. They differ in which reference the point's initial position is anchored to (docs/experimental/pr_mr.md):
- PR matches the point's path seen from the proximal contact face (Delta_contact, the paper's).
- MR matches it seen from the MCP axis, the proximal link frame F_1 (Delta_MCP).
An MCP rotation turns F_1 rigidly, so Delta_MCP holds at every MCP angle. The two references differ by the proximal
face's depth change dt, so Delta_contact + Delta_MCP >= |dt| at every flexion.

A thicker pad moves the distal face dt toward the object. Matching it from the MCP axis needs the distal link line raised
by about dt, so B's height above the link line (b_z, 0 in the studies) is searched in --b-z. NSGA-II minimises Delta_MCP,
Delta_contact and -C_pad together, warm-started from H_R and H_U. The study's goal is otherwise unchanged.

Picks, among the front's designs with C_pad >= --coverage_floor:
- MR: the least Delta_MCP;
- MR-mid: the least max(Delta_MCP, Delta_contact).

Writes ``output/study/exploratory/e20_mcp_referenced/``:
- ``front.csv``: every front design, with Delta_MCP, Delta_contact, Delta_skel, C_pad, MA_wc, b_z and x;
- ``picks.csv``: O (3 mm), PR (H_R), U (H_U), MR and MR-mid, with the same columns.
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
import shared  # noqa: E402
from core.optimization.analysis.contact_kinematics import INVALID, ContactMechanism, Redesigned, _picklable, path_change, paths  # noqa: E402
from core.optimization.mechanism import Objective, mechanism, read_config_file  # noqa: E402
from core.optimization.optimizer import OptimizerSettings  # noqa: E402
from core.optimization.study import run_study  # noqa: E402

OUT = ROOT / "output" / "study" / "exploratory" / "e20_mcp_referenced"


@dataclasses.dataclass(frozen=True)
class Referenced(Redesigned):
    mcp_change: float = INVALID
    skeleton_change: float = INVALID


def in_f1(contact: np.ndarray, proximal_depth_mm: float) -> np.ndarray:
    """A contact path from the proximal face's line (:func:`contact_path`) seen from the MCP axis's link line instead."""
    return np.asarray(contact, dtype=float) - np.array((0.0, proximal_depth_mm))


class MCPReferenced(ContactMechanism):
    """Objectives Delta_MCP, Delta_contact (both mm) and -C_pad."""

    def __init__(self, base, reference: dict, reference_depth_mm: float, scale_mm: float) -> None:
        super().__init__(base, reference["contact"], scale_mm)
        self.reference_f1 = in_f1(reference["contact"], reference_depth_mm)
        self.reference_tip = np.asarray(reference["tip"], dtype=float)
        self.name, self.formulation = f"mcp_referenced:{base.name}", f"{base.formulation}+mcp-referenced-v1"
        s = self.scale_mm
        self.objectives = {
            "mcp_change": Objective(lambda e: e.mcp_change * s if e.mcp_change < INVALID else INVALID,
                                    "mcp_change", "D_MCP", "least_mcp_change", False),
            "contact_change": Objective(lambda e: e.contact_change * s if e.contact_change < INVALID else INVALID,
                                        "contact_change", "D_c", "least_contact_change", False),
            "c_pad_mean": Objective(lambda e: -e.c_pad_mean, "c_pad_mean", "C_pad", "most_coverage", True)}

    def evaluate(self, x, config):
        try:
            base = self.base.evaluate(x, config)
        except ValueError:
            return None
        if base is None:
            return None
        base = _picklable(base)
        change, mcp, skel, coverage = INVALID, INVALID, INVALID, 0.0
        if base.feasible:
            found = paths(self.base, x, config, base, len(self.reference))
            if found is not None:
                depth = float(self.base.report(base)["proximal_depth_mm"])
                change = path_change(found["contact"], self.reference, self.scale_mm)
                mcp = path_change(in_f1(found["contact"], depth), self.reference_f1, self.scale_mm)
                skel = path_change(found["tip"], self.reference_tip, self.scale_mm)
            value = self.base.report(base).get("c_pad_mean")
            coverage = float(value) if value not in (None, "") and math.isfinite(float(value)) else 0.0
        return Referenced(base, change, coverage, dict(base.constraints), mcp, skel)


def with_b_z(config, low: float, high: float, a_z=None):
    """``config`` with B's height above the link line searched in [low, high] (0 in the studies); with ``a_z``
    (low, high) A's height too, and the joint-axes alignment (A and B on the link line) dropped."""
    bounds = {**dict(config.bounds), "b_z_mm": (float(low), float(high))}
    if a_z is None:
        return dataclasses.replace(config, bounds=bounds)
    bounds["a_z_mm"] = (float(a_z[0]), float(a_z[1]))
    return dataclasses.replace(config, bounds=bounds, align_joint_axes=False)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--flexion", type=float, default=120.0)
    parser.add_argument("--to", type=float, default=7.0)
    parser.add_argument("--seeds", type=int, nargs="+", default=(0, 1))
    parser.add_argument("--b-z", type=float, nargs=2, default=(-2.0, 8.0), metavar=("LO", "HI"))
    parser.add_argument("--a-z", type=float, nargs=2, default=None, metavar=("LO", "HI"),
                        help="also search A's height (and drop the joint-axes alignment); default: A on the link line")
    parser.add_argument("--population", type=int, default=200)
    parser.add_argument("--generations", type=int, default=200)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--coverage_floor", type=float, default=0.70)
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args(argv)

    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    spec = config["architectures"]["cfl"]
    designs = e15.study1.study1_designs("cfl", spec, args.flexion, config)
    h0, goal = designs[3.0], designs[args.to]
    base = mechanism(h0.mechanism)
    reference = paths(base, h0.x, h0.config)
    h0_report = base.report(base.evaluate(h0.x, h0.config))
    scale = float(h0_report["finger_length_mm"])
    mech = MCPReferenced(base, reference, float(h0_report["proximal_depth_mm"]), scale)
    goal_config = with_b_z(goal.config, *args.b_z, a_z=args.a_z)

    with (e15.STUDY2 / "front.csv").open(encoding="utf-8", newline="") as stream:
        study2_front = [r for r in csv.DictReader(stream) if r["architecture"] == "cfl"
                        and float(r["required_flexion_deg"]) == args.flexion and float(r["to_mm"]) == args.to]
    x_r = tuple(float(v) for v in max(study2_front, key=lambda r: float(r["c_pad_mean"]))["x"].split(";"))
    folder = e15.study1.condition_folder("cfl", spec, args.flexion, args.to, float(config["native_thickness_mm"]))
    x_u = e15.balanced_x(sorted(folder.glob("portfolio_*"))[0])

    def row(label, x, e, cfg, seed="") -> dict:
        report = base.report(e.base)
        return {"label": label, "seed": seed, "to_mm": args.to, "required_flexion_deg": args.flexion,
                "mcp_change_mm": e.mcp_change * scale, "contact_change_mm": e.contact_change * scale,
                "skeleton_change_mm": e.skeleton_change * scale, "c_pad_mean": e.c_pad_mean,
                "ma_wc": report.get("ma_wc"), "b_z_mm": float(x[2]),
                "face_depth_proximal_mm": report.get("proximal_depth_mm"),
                "face_depth_distal_mm": report.get("distal_depth_mm"), "x": ";".join(f"{v:.6g}" for v in x)}

    picks = []
    for label, x, cfg in (("O", h0.x, h0.config), ("PR", x_r, goal_config), ("U", x_u, goal_config)):
        e = mech.evaluate(x, cfg)
        picks.append(row(label, x, e, cfg))
        print(f"{label}: D_MCP {picks[-1]['mcp_change_mm']:.2f} D_c {picks[-1]['contact_change_mm']:.2f} "
              f"D_s {picks[-1]['skeleton_change_mm']:.2f} C_pad {e.c_pad_mean:.4f}", flush=True)

    found = []
    for seed in args.seeds:
        t0 = time.time()
        result = run_study(mech, goal_config, OptimizerSettings("nsga2", args.population, args.generations, seed),
                           initial_designs=[x_r, x_u], workers=args.workers)
        new = [(tuple(c.x), c.evaluation, seed) for c in result.pareto
               if c.evaluation.feasible and c.evaluation.mcp_change < INVALID and c.evaluation.contact_change < INVALID]
        found += new
        print(f"seed {seed}: {len(new)} front designs, {result.evaluation_count} evaluations, {time.time() - t0:.0f} s",
              flush=True)
    args.out.mkdir(parents=True, exist_ok=True)
    rows = [row("front", x, e, goal_config, seed) for x, e, seed in found]
    shared.write_csv(args.out / "front.csv", rows)
    ok = [(x, e, s) for x, e, s in found if e.c_pad_mean >= args.coverage_floor]
    for label, key in (("MR", lambda f: f[1].mcp_change),
                       ("MR-mid", lambda f: max(f[1].mcp_change, f[1].contact_change))):
        if not ok:
            print(f"{label}: no front design with C_pad >= {args.coverage_floor}")
            continue
        x, e, s = min(ok, key=key)
        picks.append(row(label, x, e, goal_config, s))
        print(f"{label}: D_MCP {picks[-1]['mcp_change_mm']:.2f} D_c {picks[-1]['contact_change_mm']:.2f} "
              f"D_s {picks[-1]['skeleton_change_mm']:.2f} C_pad {e.c_pad_mean:.4f} MA {picks[-1]['ma_wc']} "
              f"b_z {picks[-1]['b_z_mm']:.1f}", flush=True)
    shared.write_csv(args.out / "picks.csv", picks)
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
