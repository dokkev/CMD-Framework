#!/usr/bin/env python3
"""Pick each mechanism's reference layout from its search (reference/*.yaml)
and record it for the fixed-layout benchmark.

    python scripts/optimize/accommodation/reference/pick.py [output folder]

Per mechanism, from the latest verified front in
<output>/<mechanism>/<mechanism>/ (default output/accommodation_reference):
the designs ranked by min(S_p - 25, S_d - 25), the larger total support
S_p + S_d breaking ties (S: each support's length, the longest module it can
carry); the first the benchmark's own geometry
(core.optimization.analysis.contact_envelope) also passes is chosen: a
buildable layout carrying the reference modules clear through 150 deg.
Refused (nothing written, exit 2): a run of another formulation or config
than reference/<mechanism>.yaml now gives (search again), or no candidate
passing the benchmark. Writes <output>/reference_layouts.json (read by
visualize.comparison.contact_envelope) and prints the audit table: actuator
and pivots, supports, reach.
"""

import json
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.analysis.contact_envelope import evaluate_fit, layout_summary, max_flexion_deg  # noqa: E402
from core.optimization.analysis.tradeoff import Modules, load  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.study import load_run  # noqa: E402

HERE = Path(__file__).resolve().parent
MECHANISMS = ("revolute", "linkage")
REFERENCE = Modules(25.0, 5.0, 25.0, 5.0)
REQUIRED_FLEXION_DEG = 150.0


def ranked(rows: list[dict]) -> list[dict]:
    """The largest balance first, then the largest total support."""

    return sorted(rows, key=lambda r: (round(float(r["support_balance_mm"]), 6),
                                       float(r["proximal_room_mm"]) + float(r["distal_room_mm"])), reverse=True)


def best(rows: list[dict]) -> dict:
    return ranked(rows)[0]


def choose(rows: list[dict], passes) -> dict | None:
    """The best-ranked row ``passes(row)`` accepts (``None``: none)."""

    return next((row for row in ranked(rows) if passes(row)), None)


def config_mismatch(found, recorded, current) -> list[str]:
    """The config entries a run recorded that differ from the config file's now
    (its seeds and warm start aside: they change the search, not the design's rules)."""

    def flat(values, prefix=""):
        if isinstance(values, dict):
            return {k: v for key, value in values.items() for k, v in flat(value, f"{prefix}{key}.").items()}
        return {prefix[:-1]: json.dumps(values, default=str, sort_keys=True)}

    old, new = flat(found.config_to_json(recorded)), flat(found.config_to_json(current))
    return sorted(key for key in old.keys() | new.keys() if old.get(key) != new.get(key)
                  and key.split(".")[-1] not in ("seeds", "warm_start"))


def searched_config(path: Path):
    """The mechanism and config the search ran with (the file as is, no benchmark overrides)."""

    values = read_config_file(path)
    found = mechanism(values.pop("mechanism"))
    values.pop("output")
    return found, found.config_from_mapping(found.split_settings(values)[0])


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
        found, searched = searched_config(HERE / f"{name}.yaml")
        for run_name in sorted({row["run"].split("/")[-1] for row in rows}):
            run = load_run(study / run_name)
            differ = config_mismatch(found, run.config, searched)
            if run.formulation != found.formulation or differ:
                print(f"{name}: {run_name} was searched with {run.formulation} and another config "
                      f"({', '.join(differ[:8]) or 'formulation only'}) than {name}.yaml gives now: search again")
                return 2
        _, config = load(HERE / f"{name}.yaml")
        design = {}

        def passes(row) -> bool:
            run = load_run(study / row["run"].split("/")[-1])
            design[id(row)] = tuple(run.mechanism.read_design(row))
            return evaluate_fit(name, design[id(row)], config, REFERENCE, REQUIRED_FLEXION_DEG).feasible

        row = choose(rows, passes)
        if row is None:
            print(f"{name}: no candidate of {fronts[-1].relative_to(REPOSITORY_ROOT)} carries the reference modules "
                  f"through {REQUIRED_FLEXION_DEG:g} deg in the benchmark: nothing recorded")
            return 2
        x = design[id(row)]
        if row is not best(rows):
            print(f"{name}: the front's best ({best(rows)['support_balance_mm']} mm) fails the benchmark; "
                  f"took the best that passes ({row['support_balance_mm']} mm)")
        reach, stop = max_flexion_deg(name, x, config, REFERENCE)
        chosen[name] = {"config": str((HERE / f"{name}.yaml").relative_to(REPOSITORY_ROOT)),
                        "front": str(fronts[-1].relative_to(REPOSITORY_ROOT)), "run": row["run"],
                        "candidate": row["candidate"], "support_balance_mm": float(row["support_balance_mm"]),
                        "reference_clear_to_required": True, "required_flexion_deg": REQUIRED_FLEXION_DEG,
                        "max_flexion_deg": reach, "max_flexion_stop": stop.reason,
                        "max_flexion_pair": stop.collision_pair, **layout_summary(name, x, config, REFERENCE)}
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
