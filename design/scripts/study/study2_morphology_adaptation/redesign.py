#!/usr/bin/env python3
"""Study 2: a finger redesigned for a thicker interface, its contact-centric kinematics kept (A.6.3).

    python scripts/study/study2_morphology_adaptation/redesign.py [one_finger.yaml] [--resume]
        [--architectures cfl,2r_remote] [--flexions 120] [--output dir]

For each architecture and required flexion, the reference finger is Study 1's
design for the thinnest pads. For each thicker pad, the finger is searched
again for two objectives at once: the least change of the contact-centric
kinematics from the reference (the distal contact face's path seen from the
proximal one) and the most pad coverage (Study 1's objective, under Study 1's
goal for that thickness). No coverage floor is set: the front says what
coverage is kept at each change.

Writes ``fronts.csv`` (every seed's front; reused by ``--resume``),
``front.csv`` (the seeds merged), ``profiles.csv`` (how far the most-coverage
redesign's contact path is from the reference's at each flexion) and ``summary.csv`` per architecture, flexion
and thickness: the least change found and its coverage, the coverage of Study
1's own design for that thickness and its change, and what a joint turning
about a fixed axis must show, ``2 sin(q_req/2) dh`` (A.6.0, P4).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
for path in (REPOSITORY_ROOT, REPOSITORY_ROOT / "scripts" / "study", Path(__file__).resolve().parent):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import one_finger as study1  # noqa: E402
import numpy as np  # noqa: E402
import shared  # noqa: E402
from core.optimization.analysis.contact_kinematics import front, path_change, paths, redesign  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.optimizer import OptimizerSettings  # noqa: E402


def read_rows(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _row(head: dict, seed, x, evaluation, base, config, reference, scale: float) -> dict[str, Any]:
    report = base.report(evaluation.base)
    own = paths(base, x, config, evaluation.base, len(reference["tip"]))
    length = float(report["finger_length_mm"])
    pads = float(report["proximal_support_length_mm"]) + float(report["distal_support_length_mm"])
    return {**head, "seed": seed, "contact_change": evaluation.contact_change,
            "contact_change_mm": evaluation.contact_change * scale,
            "skeleton_change": path_change(own["tip"], reference["tip"], scale) if own else math.nan,
            "c_pad_mean": evaluation.c_pad_mean, "pad_coverage": pads / length,
            "face_depth_proximal_mm": report.get("proximal_depth_mm"),
            "face_depth_distal_mm": report.get("distal_depth_mm"), "q_max_deg": report.get("q_max_deg"),
            "ma_wc": report.get("ma_wc"), "x": ";".join(f"{v:.6g}" for v in x)}


def run(path: Path, resume: bool = False, only_architectures=None, only_flexions=None,
        output: str | None = None) -> Path:
    config = read_config_file(path)
    settings = config["redesign"]
    out = REPOSITORY_ROOT / (output or settings["output"])
    out.mkdir(parents=True, exist_ok=True)
    kept = read_rows(out / "fronts.csv") if resume else []
    done = {(r["architecture"], float(r["required_flexion_deg"]), float(r["to_mm"]), int(r["seed"])) for r in kept}
    thin = float(settings["reference_thickness_mm"])
    search = settings["search"]
    merged, summary, profiles, skipped = [], [], [], []
    for name, spec in config["architectures"].items():
        if only_architectures and name not in only_architectures:
            continue
        for flexion in (float(q) for q in config["required_flexion_deg"]):
            if only_flexions and flexion not in only_flexions:
                continue
            designs = study1.study1_designs(name, spec, flexion, config)
            if thin not in designs or len(designs) < 2:
                skipped.append({"architecture": name, "required_flexion_deg": flexion})
                print(f"skip {name} q{flexion:g}: Study 1 not run for it", flush=True)
                continue
            first = designs[thin]
            base = mechanism(first.mechanism)
            reference = paths(base, first.x, first.config)
            scale = float(base.report(base.evaluate(first.x, first.config))["finger_length_mm"])
            warm_front: list = []
            for target in sorted(h for h in designs if h > thin):
                goal = designs[target]
                head = {"architecture": name, "mechanism": first.mechanism, "upper_bound": first.upper_bound,
                        "required_flexion_deg": flexion, "from_mm": thin, "to_mm": target}
                for seed in search["seeds"]:
                    if (name, flexion, target, int(seed)) in done:
                        continue
                    print(f"{name} / q_req {flexion:g} / {thin:g} -> {target:g} mm / seed {seed} ...", flush=True)
                    found, _ = redesign(base, goal.config, reference["contact"], scale, OptimizerSettings(
                        search["algorithm"], int(search["population"]), int(search["generations"]), int(seed)),
                        [first.x, goal.x, *warm_front], int(settings.get("workers", 1)))
                    kept += [_row(head, seed, x, e, base, goal.config, reference, scale) for x, e in found]
                    done.add((name, flexion, target, int(seed)))
                    shared.write_csv(out / "fronts.csv", kept)
                    if found:
                        print(f"  least change {100 * found[0][1].contact_change:.2f}% of L_f at C_pad "
                              f"{found[0][1].c_pad_mean:.3f}; most C_pad {found[-1][1].c_pad_mean:.3f} at "
                              f"{100 * found[-1][1].contact_change:.2f}%", flush=True)
                own = [r for r in kept if (r["architecture"], float(r["required_flexion_deg"]), float(r["to_mm"]))
                       == (name, flexion, target)]
                keep = front([(float(r["contact_change"]), float(r["c_pad_mean"])) for r in own])
                best = [own[i] for i in keep]
                merged += best
                warm_front = [tuple(float(v) for v in str(r["x"]).split(";")) for r in best[:5]]
                study1_paths = paths(base, goal.x, goal.config)
                study1_coverage = study1.coverage(base, base.evaluate(goal.x, goal.config))
                dh = target - thin
                row = {**head, "fixed_axis_change": 2 * math.sin(math.radians(flexion) / 2) * dh / scale,
                       "study1_c_pad": study1_coverage,
                       "study1_change": path_change(study1_paths["contact"], reference["contact"], scale)
                       if study1_paths else math.nan, "front_points": len(best)}
                if best:
                    least, most = best[0], best[-1]
                    row |= {"least_change": float(least["contact_change"]), "c_pad_at_least_change": float(least["c_pad_mean"]),
                            "most_c_pad": float(most["c_pad_mean"]), "change_at_most_c_pad": float(most["contact_change"])}
                    # Where over the flexion the two contact paths part: the most-coverage redesign's.
                    x = tuple(float(v) for v in str(most["x"]).split(";"))
                    own_paths = paths(base, x, goal.config)
                    if own_paths is not None:
                        apart = np.linalg.norm(own_paths["contact"] - reference["contact"], axis=1)
                        profiles += [{**head, "q_deg": math.degrees(q), "contact_apart_mm": float(d)}
                                     for q, d in zip(reference["q"], apart)]
                summary.append(row)
                shared.write_csv(out / "front.csv", merged)
                shared.write_csv(out / "summary.csv", summary)
                shared.write_csv(out / "profiles.csv", profiles)
    (out / "manifest.json").write_text(json.dumps({
        "config": str(path), "formulation": "problem_formulation.md A.6.3 (2026-11-20: contact-centric redesign)",
        "redesign": settings, "skipped": skipped, **shared.git_commit(out)}, indent=2))
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("config", nargs="?", default=str(study1.DEFAULT_CONFIG))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--architectures")
    parser.add_argument("--flexions")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    out = run(Path(args.config), args.resume, args.architectures.split(",") if args.architectures else None,
              [float(q) for q in args.flexions.split(",")] if args.flexions else None, args.output)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
