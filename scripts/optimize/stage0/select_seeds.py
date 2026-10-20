#!/usr/bin/env python3
"""Verify Stage-0 Pareto candidates on a fine PIP grid and mark the seeds.

    python scripts/optimize/stage0/select_seeds.py output/stage0/run_001 output/stage0/run_002

Each candidate is re-evaluated with ``--step-deg`` PIP servo samples (the optimizer
uses a coarse grid, so a condition can fail between samples). The verified,
mutually non-dominated candidates are written with a ``seed_role``
(``docs/optimization/stage0.md``), for the objectives the runs optimized
(``core/optimization/stage0/objectives.py``):

- the best at each objective: ``best_transmission`` (smallest ``k_max``),
  ``best_length`` (longest pads), ``best_curl`` (largest ``phi_safe_max``);
- ``balanced``: the closest to the ideal point with each objective scaled to
  ``[0, 1]`` over the verified front (the compromise of them all);
- ``pareto``: every other verified non-dominated candidate.

A candidate that holds several roles lists them all, joined by ``;``. The runs
must share their study settings (all but the seed and the GA size) and use the
current formulation; earlier runs (``OA + BC``) can only warm-start a new one.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import math
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.stage0 import VARIABLE_NAMES, decode, evaluate, read_design  # noqa: E402
from core.optimization.stage0.run import (  # noqa: E402
    FORMULATION,
    LINK_COLUMNS,
    link_values,
    load_config,
    load_formulation,
    load_objectives,
    objectives,
)
from core.optimization.stage0.objectives import role_order, seed_roles  # noqa: E402


DEFAULT_OUTPUT = REPOSITORY_ROOT / "output" / "stage0" / "seeds.csv"
RUN_SETTINGS = ("seed", "population_size", "generations")
"""Config entries that may differ between the runs being merged."""


def study_settings(run: Path) -> dict:
    """The run's study settings as loaded (so a setting added since, at its
    default, matches), less the per-run ones."""

    config = load_config(run)
    return {f.name: getattr(config, f.name) for f in dataclasses.fields(config)
            if f.name not in RUN_SETTINGS}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path, help="Stage-0 run directories")
    parser.add_argument("--step-deg", type=float, default=0.05)
    parser.add_argument("--dip-tolerance-deg", type=float, default=0.1,
                        help="DIP and driven-link backoff allowed in verification")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)

    for run in args.runs:
        if (formulation := load_formulation(run)) != FORMULATION:
            parser.error(f"{run} used the {formulation!r} formulation, not {FORMULATION!r}; "
                         "warm-start a new run from it instead")
    names = load_objectives(args.runs[0])
    for run in args.runs[1:]:
        if load_objectives(run) != names:
            parser.error(f"{run} optimized {load_objectives(run)}, not {names}")
    roles_order = role_order(names)
    reference = study_settings(args.runs[0])
    for run in args.runs[1:]:
        settings = study_settings(run)
        differing = sorted(key for key in reference.keys() | settings.keys()
                           if reference.get(key) != settings.get(key))
        if differing:
            parser.error(f"{run} differs from {args.runs[0]} in {', '.join(differing)}")

    rows = []
    print("run                 pareto  verified")
    for run in args.runs:
        config = dataclasses.replace(
            load_config(run), servo_step_rad=math.radians(args.step_deg),
            dip_backoff_tolerance_rad=math.radians(args.dip_tolerance_deg),
        )
        with (run / "pareto.csv").open(encoding="utf-8", newline="") as stream:
            candidates = list(csv.DictReader(stream))
        verified = 0
        for row in candidates:
            x = read_design(row)
            e = evaluate(decode(x, config.link_length_resolution_mm), config)
            if not e.feasible:
                continue
            verified += 1
            rows.append({
                "run": f"{run.parent.name}/{run.name}", "candidate": row["candidate"],
                **dict(zip(VARIABLE_NAMES, x)),
                "objectives": objectives(e, names),
                "torque_ratio_max": e.torque_ratio_max,
                "torque_ratio_min": e.torque_ratio_min,
                "torque_ratio_spread": e.torque_ratio_spread,
                "usable_support_length_mm": e.usable_support_length_mm,
                "proximal_usable_length_mm": e.proximal_usable_length_mm,
                "distal_usable_length_mm": e.distal_usable_length_mm,
                "support_length_mm": e.support_length_mm,
                "proximal_support_length_mm": e.proximal_support_length_mm,
                "distal_support_length_mm": e.distal_support_length_mm,
                "phi_safe_deg": math.degrees(e.phi_safe_rad),
                "usable_pip_deg": math.degrees(e.usable_pip_rad),
                "stop_reason": e.stop_reason,
                "collision_pair": e.collision_pair,
                "pad_clearance_mm": e.pad_clearance_mm,
                "min_margin_deg": math.degrees(e.min_margin_rad),
                "toggle_margin_deg": math.degrees(e.toggle_margin_rad),
                "hyperextension_phi_deg": math.degrees(e.hyperextension_phi_rad),
                "servo_travel_deg": math.degrees(e.servo_travel_rad),
                "lever_max_mm": e.lever_max_mm,
                "ab_mm": e.ab_mm,
                "curl_tip_x_mm": e.curl_tip_x_mm,
                "servo_lowest_z_mm": e.servo_lowest_z_mm,
                **dict(zip(LINK_COLUMNS, link_values(e.parameters))),
            })
        print(f"{run.parent.name + '/' + run.name:19s} {len(candidates):6d}  {verified:8d}")

    # Keep the verified candidates no other verified candidate dominates.
    def dominates(first, second):
        return all(a <= b for a, b in zip(first, second)) and first != second

    rows = [row for row in rows
            if not any(dominates(other["objectives"], row["objectives"]) for other in rows)]
    for row in rows:
        row["roles"] = []
    for role, index in seed_roles([row["objectives"] for row in rows], rows, names).items():
        rows[index]["roles"].append(role)
    for row in rows:
        row["roles"].sort(key=roles_order.index)
    rows.sort(key=lambda row: (roles_order.index(row["roles"][0]) if row["roles"] else len(roles_order),
                               row["torque_ratio_max"]))
    for row in rows:
        row.pop("objectives")
        row["seed_role"] = ";".join(row.pop("roles")) or "pareto"

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        fields = ["seed_role", *[key for key in (rows[0] if rows else {"run": 0}) if key != "seed_role"]]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n{len(rows)} verified non-dominated seeds -> {args.output}")
    print(f"  {'role':28s} {'run':14s} {'#':>4s} {'k_max':>6s} {'k ratio':>7s} {'pad length (P + D) mm':>22s} "
          f"{'phi_safe':>9s} {'pad clr':>8s}  stop")
    for row in rows:
        if row["seed_role"] == "pareto":
            break
        pads = (f"{row['support_length_mm']:.1f} ({row['proximal_support_length_mm']:.1f} + "
                f"{row['distal_support_length_mm']:.1f})")
        stop = row["stop_reason"] + (f" ({row['collision_pair']})" if row["collision_pair"] else "")
        print(f"  {row['seed_role']:28s} {row['run']:14s} {row['candidate']:>4s} "
              f"{row['torque_ratio_max']:6.2f} {row['torque_ratio_spread']:7.2f} {pads:>22s} {row['phi_safe_deg']:7.1f}° "
              f"{row['pad_clearance_mm']:6.2f}mm  {stop}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
