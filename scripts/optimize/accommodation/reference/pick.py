#!/usr/bin/env python3
"""Pick each mechanism's reference layout from its search (reference/*.yaml)
and record it for the fixed-layout benchmark.

    python scripts/optimize/accommodation/reference/pick.py [output folder]

Per mechanism, from the latest verified front in
<output>/<mechanism>/<mechanism>/ (default output/accommodation_reference):
the design with the largest min(S_p - 25, S_d - 25), the larger total
support S_p + S_d breaking ties (S: each support's length, the longest
module it can carry). Checked again with the benchmark's own geometry
(core.optimization.analysis.contact_envelope): the reference modules clear
through 150 deg, and the most it flexes with them. Writes
<output>/reference_layouts.json (read by visualize.comparison.contact_envelope)
and prints the audit table: actuator and pivots, supports, reach.
"""

import json
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.analysis.contact_envelope import evaluate_fit, layout_summary, max_flexion_deg  # noqa: E402
from core.optimization.analysis.tradeoff import Modules, load  # noqa: E402
from core.optimization.study import load_run  # noqa: E402

HERE = Path(__file__).resolve().parent
MECHANISMS = ("revolute", "linkage")
REFERENCE = Modules(25.0, 5.0, 25.0, 5.0)
REQUIRED_FLEXION_DEG = 150.0


def best(rows: list[dict]) -> dict:
    """The largest balance, then the largest total support."""

    return max(rows, key=lambda r: (round(float(r["support_balance_mm"]), 6),
                                    float(r["proximal_room_mm"]) + float(r["distal_room_mm"])))


def main(argv: list[str]) -> int:
    import csv

    output = REPOSITORY_ROOT / (argv[0] if argv else "output/accommodation_reference")
    chosen = {}
    for name in MECHANISMS:
        study = output / name / name
        fronts = sorted(study.glob("portfolio_*/seeds.csv"))
        if not fronts:
            print(f"{name}: no verified front in {study.relative_to(REPOSITORY_ROOT)} (run reference/run.sh)")
            return 1
        rows = list(csv.DictReader(fronts[-1].open(encoding="utf-8")))
        row = best(rows)
        run = load_run(study / row["run"].split("/")[-1])
        x = tuple(run.mechanism.read_design(row))
        _, config = load(HERE / f"{name}.yaml")
        fit = evaluate_fit(name, x, config, REFERENCE, REQUIRED_FLEXION_DEG)
        reach, stop = max_flexion_deg(name, x, config, REFERENCE)
        chosen[name] = {"config": str((HERE / f"{name}.yaml").relative_to(REPOSITORY_ROOT)),
                        "front": str(fronts[-1].relative_to(REPOSITORY_ROOT)), "run": row["run"],
                        "candidate": row["candidate"], "support_balance_mm": float(row["support_balance_mm"]),
                        "reference_clear_to_required": fit.feasible, "required_flexion_deg": REQUIRED_FLEXION_DEG,
                        "max_flexion_deg": reach, "max_flexion_stop": stop.reason,
                        "max_flexion_pair": stop.collision_pair, **layout_summary(name, x, config, REFERENCE)}
        if not fit.feasible:
            print(f"{name}: the chosen layout does not carry the reference modules through "
                  f"{REQUIRED_FLEXION_DEG:g} deg in the benchmark ({fit.reason} at {fit.first_failure_deg}); "
                  "check the search and the benchmark agree before using it")
    path = output / "reference_layouts.json"
    path.write_text(json.dumps({"reference_modules": REFERENCE.__dict__, "layouts": chosen}, indent=2),
                    encoding="utf-8")
    print(f"{'':10s} {'|OC|':>7s} {'S_p':>6s} {'S_d':>6s} {'balance':>8s} {'max phi':>8s}  joints")
    for name, info in chosen.items():
        joints = ", ".join(f"{k} ({v[0]:.1f}, {v[1]:.1f})" for k, v in {**info["active_joints"],
                                                                         **info["passive_joints"]}.items())
        print(f"{name:10s} {info['finger_length_mm']:7.1f} {info['support_room_mm']['proximal']:6.1f} "
              f"{info['support_room_mm']['distal']:6.1f} {info['support_balance_mm']:8.1f} "
              f"{info['max_flexion_deg']:8.1f}  {joints}; tilt {info['pip_servo_tilt_deg']:.0f} deg")
    print(f"-> {path.relative_to(REPOSITORY_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
