#!/usr/bin/env python3
"""Study 2: a finger redesigned for a thicker interface, its relative contact trajectory kept (A.6.3).

    python scripts/study/study2_morphology_adaptation/redesign.py [one_finger.yaml] [--resume]
        [--architectures cfl,2r_remote] [--flexions 120] [--output dir] [--free-heights] [--objective eq9]

For each architecture and required flexion, the reference finger is Study 1's
design for the thinnest pads. For each thicker pad, the finger is searched
again for two objectives at once: the least change of the contact-centric
kinematics from the reference (the distal contact face's path seen from the
proximal one) and the most pad coverage (Study 1's objective, under Study 1's
goal for that thickness). No coverage floor is set: the front says what
coverage is kept at each change.

With ``--free-heights`` (experimental, docs/experimental/pr_mr.md), the pivots' heights above the link lines are
searched as well (``FREE_HEIGHTS``): the CFL's A and B (the joint-axes alignment dropped), the 2R's joint A above
the proximal and the distal link line, the RCJ's roller centres A and B above theirs. Each row also gives the change
of the same contact path seen from the MCP axis (``mcp_change_mm``, Delta_MCP).

With ``--objective eq9`` (the paper's Eq. 9), the search has three objectives: the least change of the contact path
seen from the proximal pad (Delta^prox_distal, ``contact_change``), the least change of the same path seen from the
finger base F_1 (Delta^base_distal, ``mcp_change``) and the most coverage, with the pivots' heights free (as
``--free-heights``). The fronts are then the three objectives' (``front_of``). The summary picks each condition's
redesign by the default's rule (:func:`pick`): the most coverage on the front, ties to the least Delta^prox_distal, then
the least Delta^base_distal; it also gives the least Delta^prox_distal within ``NEAR_C_PAD`` of that coverage.

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
import dataclasses
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
from core.optimization.analysis.contact_kinematics import front, front_of, path_change, paths, redesign  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.optimizer import OptimizerSettings  # noqa: E402


FREE_HEIGHTS = {"a_z_mm": (-3.0, 8.0), "b_z_mm": (-3.0, 10.0)}
"""Search bounds of the pivots' heights under ``--free-heights`` (the e20-e22 hands' bounds): the proximal one's
(the CFL's A, the 2R's A above the proximal link line, the RCJ's A) and the distal one's (the CFL's B, the 2R's A above
the distal link line, the RCJ's B)."""
OBJECTIVES = ("paper", "eq9")
NEAR_C_PAD = 0.005
"""The coverage band of the summary's second pick: the least Delta^prox_distal at no less than the front's most
coverage minus this."""


def free_heights(config):
    """``config`` with the pivots' heights searched (``FREE_HEIGHTS``): the linkage's A and B (the joint-axes
    alignment dropped), the 2R's and the RCJ's joint (``heights``; the belt 2R's in its joint's config)."""

    heights = (FREE_HEIGHTS["a_z_mm"], FREE_HEIGHTS["b_z_mm"])
    if hasattr(config, "align_joint_axes"):  # the linkage
        return dataclasses.replace(config, bounds={**dict(config.bounds), **FREE_HEIGHTS}, align_joint_axes=False)
    if hasattr(config, "heights"):  # the 2R, the RCJ
        return dataclasses.replace(config, heights=heights)
    if hasattr(config, "base") and hasattr(config.base, "heights"):  # the belt 2R
        return dataclasses.replace(config, base=dataclasses.replace(config.base, heights=heights))
    raise ValueError(f"no free heights for {type(config).__name__}")


def heights_of(base, x) -> tuple[float, float]:
    """A design's two pivot heights (``FREE_HEIGHTS``' order; 0 where the vector has none)."""

    names = tuple(base.variable_names)
    if "a_z_mm" in names and "b_z_mm" in names:  # the linkage's own variables
        return float(x[names.index("a_z_mm")]), float(x[names.index("b_z_mm")])
    extra = [float(v) for v in x[len(names):len(names) + 2]]
    return tuple(extra + [0.0] * (2 - len(extra)))


def with_heights(base, x, config) -> tuple[float, ...]:
    """``x`` with the pivots' heights at 0 appended where ``config`` searches them and the vector has none (a 2R's
    or an RCJ's Study 1 design as a start)."""

    searched = getattr(config, "heights", None) or getattr(getattr(config, "base", None), "heights", None)
    names = tuple(base.variable_names)
    if searched is None or len(x) > len(names):
        return tuple(x)
    return (*x, 0.0, 0.0)


def pick(rows: list[dict], near: float = 0.0) -> dict | None:
    """A condition's redesign from its front: the least Delta^prox_distal among the designs with at least the most
    coverage minus ``near`` (ties to the least Delta^base_distal). ``near`` 0 is the default summary's rule: the most
    coverage, ties to the least change."""

    if not rows:
        return None
    best = max(float(r["c_pad_mean"]) for r in rows)
    within = [r for r in rows if float(r["c_pad_mean"]) >= best - near - 1e-12]
    if near == 0.0:
        within = [r for r in within if float(r["c_pad_mean"]) >= best - 1e-12]
    mcp = lambda r: float(r["mcp_change_mm"]) if str(r.get("mcp_change_mm", "")) not in ("", "nan") else math.inf  # noqa: E731
    return min(within, key=lambda r: (float(r["contact_change"]), mcp(r)))


def read_rows(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _row(head: dict, seed, x, evaluation, base, config, reference, scale: float) -> dict[str, Any]:
    heights = heights_of(base, x)
    report = base.report(evaluation.base)
    own = paths(base, x, config, evaluation.base, len(reference["tip"]))
    length = float(report["finger_length_mm"])
    pads = float(report["proximal_support_length_mm"]) + float(report["distal_support_length_mm"])
    mcp = math.nan
    if own and "depth_mm" in reference:
        shift = np.array((0.0, float(report["proximal_depth_mm"]) - reference["depth_mm"]))
        mcp = path_change(np.asarray(own["contact"]) - shift, reference["contact"], scale) * scale
    return {**head, "seed": seed, "contact_change": evaluation.contact_change,
            "contact_change_mm": evaluation.contact_change * scale,
            "skeleton_change": path_change(own["tip"], reference["tip"], scale) if own else math.nan,
            "mcp_change_mm": mcp, "height_proximal_mm": heights[0], "height_distal_mm": heights[1],
            "c_pad_mean": evaluation.c_pad_mean, "pad_coverage": pads / length,
            "face_depth_proximal_mm": report.get("proximal_depth_mm"),
            "face_depth_distal_mm": report.get("distal_depth_mm"), "q_max_deg": report.get("q_max_deg"),
            "ma_wc": report.get("ma_wc"), "x": ";".join(f"{v:.6g}" for v in x)}


def run(path: Path, resume: bool = False, only_architectures=None, only_flexions=None,
        output: str | None = None, heights: bool = False, objective: str = "paper") -> Path:
    if objective not in OBJECTIVES:
        raise ValueError(f"objective must be one of {OBJECTIVES}")
    eq9 = objective == "eq9"
    heights = heights or eq9  # Eq. 9's F' frees the pivots' heights
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
            first_report = base.report(base.evaluate(first.x, first.config))
            scale = float(first_report["finger_length_mm"])
            reference = {**reference, "depth_mm": float(first_report.get("proximal_depth_mm") or math.nan)}
            warm_front: list = []
            for target in sorted(h for h in designs if h > thin):
                goal = designs[target]
                if heights:
                    goal = dataclasses.replace(goal, config=free_heights(goal.config))
                head = {"architecture": name, "mechanism": first.mechanism, "upper_bound": first.upper_bound,
                        "required_flexion_deg": flexion, "from_mm": thin, "to_mm": target}
                for seed in search["seeds"]:
                    if (name, flexion, target, int(seed)) in done:
                        continue
                    print(f"{name} / q_req {flexion:g} / {thin:g} -> {target:g} mm / seed {seed} ...", flush=True)
                    starts = [with_heights(base, x, goal.config) for x in (first.x, goal.x)]
                    found, _ = redesign(base, goal.config, reference["contact"], scale, OptimizerSettings(
                        search["algorithm"], int(search["population"]), int(search["generations"]), int(seed)),
                        [*starts, *warm_front], int(settings.get("workers", 1)),
                        reference_depth_mm=reference["depth_mm"] if eq9 else None)
                    kept += [_row(head, seed, x, e, base, goal.config, reference, scale) for x, e in found]
                    done.add((name, flexion, target, int(seed)))
                    shared.write_csv(out / "fronts.csv", kept)
                    if found:
                        most = max(found, key=lambda f: f[1].c_pad_mean)
                        print(f"  {len(found)} on the front; least change {100 * found[0][1].contact_change:.2f}% of "
                              f"L_f at C_pad {found[0][1].c_pad_mean:.3f}; most C_pad {most[1].c_pad_mean:.3f} at "
                              f"{100 * most[1].contact_change:.2f}%", flush=True)
                own = [r for r in kept if (r["architecture"], float(r["required_flexion_deg"]), float(r["to_mm"]))
                       == (name, flexion, target)]
                if eq9:
                    keep = front_of([(float(r["contact_change"]), float(r["mcp_change_mm"]), -float(r["c_pad_mean"]))
                                     for r in own])
                else:
                    keep = front([(float(r["contact_change"]), float(r["c_pad_mean"])) for r in own])
                best = [own[i] for i in keep]
                merged += best
                # The warm starts: the least-change end of the front (and, for three objectives, its coverage end).
                warm = best[:5] + (sorted(best, key=lambda r: -float(r["c_pad_mean"]))[:5] if eq9 else [])
                warm_front = list(dict.fromkeys(tuple(float(v) for v in str(r["x"]).split(";")) for r in warm))
                study1_paths = paths(base, goal.x, goal.config)
                study1_coverage = study1.coverage(base, base.evaluate(goal.x, goal.config))
                dh = target - thin
                row = {**head, "fixed_axis_change": 2 * math.sin(math.radians(flexion) / 2) * dh / scale,
                       "study1_c_pad": study1_coverage,
                       "study1_change": path_change(study1_paths["contact"], reference["contact"], scale)
                       if study1_paths else math.nan, "front_points": len(best)}
                if best:
                    least, most = best[0], pick(best)
                    near = pick(best, NEAR_C_PAD)
                    row |= {"least_change": float(least["contact_change"]), "c_pad_at_least_change": float(least["c_pad_mean"]),
                            "most_c_pad": float(most["c_pad_mean"]), "change_at_most_c_pad": float(most["contact_change"]),
                            "change_at_most_c_pad_mm": float(most["contact_change"]) * scale,
                            "mcp_change_at_most_c_pad_mm": float(most["mcp_change_mm"]),
                            "heights_at_most_c_pad_mm": f"{float(most.get('height_proximal_mm') or 0):g};"
                                                        f"{float(most.get('height_distal_mm') or 0):g}",
                            "face_depth_proximal_at_most_c_pad_mm": most["face_depth_proximal_mm"],
                            "near_c_pad": float(near["c_pad_mean"]), "change_near_c_pad": float(near["contact_change"]),
                            "mcp_change_near_c_pad_mm": float(near["mcp_change_mm"]),
                            "least_mcp_change_mm": min(float(r["mcp_change_mm"]) for r in best),
                            "finger_length_mm": scale}
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
        "redesign": settings, "objective": objective, "free_heights": FREE_HEIGHTS if heights else None,
        "near_c_pad": NEAR_C_PAD, "skipped": skipped, **shared.git_commit(out)}, indent=2))
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("config", nargs="?", default=str(study1.DEFAULT_CONFIG))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--architectures")
    parser.add_argument("--flexions")
    parser.add_argument("--output")
    parser.add_argument("--free-heights", action="store_true", help="also search the pivots' heights")
    parser.add_argument("--objective", choices=OBJECTIVES, default="paper",
                        help="paper: Delta_contact and C_pad (default); eq9: also Delta_MCP, the heights free")
    args = parser.parse_args(argv)
    out = run(Path(args.config), args.resume, args.architectures.split(",") if args.architectures else None,
              [float(q) for q in args.flexions.split(",")] if args.flexions else None, args.output, args.free_heights,
              args.objective)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
