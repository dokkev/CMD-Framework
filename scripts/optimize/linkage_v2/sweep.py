#!/usr/bin/env python3
"""Exploratory one-at-a-time sweeps of linkage formulation v2 (docs/linkage_v2_sweeps.md).

    python scripts/optimize/linkage_v2/sweep.py run --conditions baseline --seeds 0 1 2 --generations 200
    python scripts/optimize/linkage_v2/sweep.py run --seeds 0 1 2 3 4 --generations 200
    python scripts/optimize/linkage_v2/sweep.py analyze --generations 200

Conditions: the ROM sweep (phi_req 120-180 deg at L_p 30, L_d 40) and the
L_p x L_d grid at phi_req 150 deg. ``run`` optimizes each with single NSGA-II
runs (the development protocol), one per seed, into ``output/linkage_v2/sweeps/<condition>/run_NNN``.
``analyze`` verifies each condition's runs on the 0.05 deg grid and merges
them (``core/optimization/seeds.py``), then writes per-condition metrics,
the knee designs (with the A'' post-extension), the monotonicity check and
the figures to ``output/linkage_v2/sweeps/``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

import dataclasses  # noqa: E402

from core.optimization.hypervolume import front_hypervolume  # noqa: E402
from core.optimization.linkage.extend import extend_pads  # noqa: E402
from core.optimization.linkage_v2 import MECHANISM  # noqa: E402
from core.optimization.seeds import select_seeds  # noqa: E402
from core.optimization.study import load_run, load_yaml, run_study, save_result, write_rows  # noqa: E402

CONFIG = Path(__file__).with_name("config.yaml")
ROOT = REPOSITORY_ROOT / "output" / "linkage_v2" / "sweeps"
BASELINE = {"phi_req": 150.0, "L_p": 30.0, "L_d": 40.0}
KEYS = {"phi_req": "requirements.required_rom_deg", "L_p": "requirements.module_max_length_p",
        "L_d": "requirements.module_max_length_d"}
ROM = (120.0, 135.0, 150.0, 165.0, 180.0)
PROXIMAL = (20.0, 30.0, 40.0, 50.0)
DISTAL = (30.0, 40.0, 50.0, 60.0)
CAPS = (1.6, 1.8, 2.0)
NAMES = ("transmission", "finger_length")


def condition_name(values: dict) -> str:
    return f"rom{values['phi_req']:g}_lp{values['L_p']:g}_ld{values['L_d']:g}"


def conditions() -> dict[str, dict]:
    """Every condition, by name: the ROM sweep at the baseline pads and the
    L_p x L_d grid at the baseline ROM (the baseline once)."""

    found = {}
    for phi in ROM:
        each = dict(BASELINE, phi_req=phi)
        found[condition_name(each)] = each
    for proximal in PROXIMAL:
        for distal in DISTAL:
            each = dict(BASELINE, L_p=proximal, L_d=distal)
            found[condition_name(each)] = each
    return found


def harder(values: dict) -> list[dict]:
    """The neighbouring conditions one step harder (larger phi_req, L_p or
    L_d) in the sweep: every design feasible there is feasible here, with the
    same |OC| and at most the same k_max."""

    found = []
    if values["L_p"] == BASELINE["L_p"] and values["L_d"] == BASELINE["L_d"] and values["phi_req"] in ROM:
        i = ROM.index(values["phi_req"])
        if i + 1 < len(ROM):
            found.append(dict(values, phi_req=ROM[i + 1]))
    if values["phi_req"] == BASELINE["phi_req"]:
        for key, axis in (("L_p", PROXIMAL), ("L_d", DISTAL)):
            i = axis.index(values[key])
            if i + 1 < len(axis):
                found.append(dict(values, **{key: axis[i + 1]}))
    return found


def warm_designs(values: dict, most: int) -> list[list[float]]:
    """At most ``most`` designs of the harder neighbours' verified fronts,
    spread along |OC| (the ends of each front always among them)."""

    rows = []
    for neighbour in harder(values):
        path = ROOT / condition_name(neighbour) / "seeds.csv"
        if path.exists():
            rows += list(csv.DictReader(path.open(encoding="utf-8", newline="")))
    rows = _non_dominated(rows)
    rows.sort(key=lambda r: float(r["finger_length_mm"]))
    if len(rows) > most:
        picks = sorted({round(i * (len(rows) - 1) / (most - 1)) for i in range(most)})
        rows = [rows[i] for i in picks]
    return [MECHANISM.read_design(row) for row in rows]


def run(args) -> None:
    every = conditions()
    names = ([condition_name(BASELINE)] if args.conditions == ["baseline"]
             else args.conditions or list(every))
    for name in names:
        overrides = [f"{KEYS[key]}={value}" for key, value in every[name].items()]
        overrides.append(f"optimizer.generations={args.generations}")
        config, settings = load_yaml(MECHANISM, CONFIG, overrides)
        initial = warm_designs(every[name], settings.population_size // 2) if args.warm_from_harder else []
        if args.warm_from_harder and not initial:
            print(f"{name}: no harder neighbour front to warm-start from, skipped")
            continue
        for seed in args.seeds:
            settings = dataclasses.replace(settings, seed=seed)
            result = run_study(MECHANISM, config, settings, initial_designs=initial, workers=args.workers)
            root = ROOT / name
            index = 1
            while (root / f"run_{index:03d}").exists():
                index += 1
            output = save_result(result, root / f"run_{index:03d}")
            if initial:  # record where it started
                (output / "warm_start.json").write_text(json.dumps({
                    "from": [condition_name(n) for n in harder(every[name])], "designs": len(initial)}) + "\n")
            print(f"{name} seed {seed} gen {args.generations}{' warm' if initial else ''}: {len(result.pareto)} feasible Pareto, "
                  f"{result.feasible_evaluation_count}/{result.evaluation_count} feasible evaluations -> {output}")


def condition_runs(name: str, generations: int | None) -> list:
    runs = [load_run(path) for path in sorted((ROOT / name).glob("run_*"))]
    if generations is not None:
        runs = [run for run in runs if run.settings.generations == generations]
    return runs


def inherited(name: str, config) -> list[dict]:
    """The harder neighbours' verified fronts, re-verified here on the fine
    grid: designs every one of which this condition admits (nested
    constraints), so its front can only be as good or better."""

    fine = MECHANISM.refine(config, math.radians(0.05))
    rows = []
    for neighbour in harder(conditions()[name]):
        path = ROOT / condition_name(neighbour) / "seeds.csv"
        if not path.exists():
            continue
        for row in csv.DictReader(path.open(encoding="utf-8", newline="")):
            x = MECHANISM.read_design(row)
            e = MECHANISM.evaluate(x, fine)
            if e is not None and e.feasible:
                rows.append({"run": f"inherited:{condition_name(neighbour)}", "candidate": row["candidate"],
                             "algorithm": row["algorithm"], "seed": row["seed"],
                             **dict(zip(MECHANISM.variable_names, x)), **MECHANISM.report(e)})
    return rows


def merged_front(own: list[dict], extra: list[dict], names=NAMES) -> list[dict]:
    """Non-dominated rows of both, with the representative roles marked."""

    rows = _non_dominated([{k: v for k, v in r.items() if k != "seed_role"} for r in own + extra])
    if not rows:
        return rows
    scores = [[MECHANISM.objectives[n].minimized(float(r[MECHANISM.objectives[n].column])) for n in names]
              for r in rows]
    roles = MECHANISM.seed_roles(scores, rows, names)
    order = MECHANISM.role_order(names)
    for row in rows:
        row["seed_role"] = ";".join(role for role in order if roles.get(role) is not None
                                    and rows[roles[role]] is row) or "pareto"
    rows.sort(key=lambda r: (min((order.index(x) for x in r["seed_role"].split(";") if x in order),
                                 default=len(order)), float(r["torque_ratio_max"])))
    return rows


def metrics(name: str, runs: list, extend: bool) -> dict:
    """Verify and merge a condition's runs, with the harder neighbours'
    verified fronts as further candidates; its front's figures and knee."""

    config = runs[0].config
    own = select_seeds(runs).rows
    extra = inherited(name, config)
    rows = merged_front(own, extra)
    write_rows(ROOT / name / "seeds.csv", rows, ("seed_role", "run", "candidate", "algorithm", "seed"))
    evaluations = sum(run.metadata["evaluation_count"] for run in runs)
    feasible = sum(run.metadata.get("feasible_evaluation_count") or 0 for run in runs)
    out = {"condition": name, "runs": [run.directory.name for run in runs],
           "seeds": [run.settings.seed for run in runs], "generations": runs[0].settings.generations,
           "front": len(rows), "feasible_fraction": feasible / evaluations if evaluations else 0.0,
           "inherited_candidates": len(extra),
           "inherited_on_front": sum(1 for r in rows if str(r["run"]).startswith("inherited:"))}
    if not rows:
        out.update(feasible=False)
        return out
    k = [float(r["torque_ratio_max"]) for r in rows]
    length = [float(r["finger_length_mm"]) for r in rows]
    out.update(
        feasible=True,
        min_k_max=min(k), min_finger_length_mm=min(length),
        hypervolume=front_hypervolume(rows, MECHANISM, NAMES, config.hypervolume_reference),
        **{f"min_finger_length_at_k{cap}": min((l for kk, l in zip(k, length) if kk <= cap), default=None)
           for cap in CAPS},
        bound_hits={},
        front_points=sorted(zip(length, k)),
    )
    for row in rows:
        for hit in filter(None, row["bound_hits"].split(";")):
            out["bound_hits"][hit] = out["bound_hits"].get(hit, 0) + 1
    # Leave one run out: how much of the merged front the other runs reach.
    if len(runs) > 1:
        per_run = [select_seeds([each]).rows for each in runs]
        out["single_run_hypervolumes"] = [
            front_hypervolume(rows_, MECHANISM, NAMES, config.hypervolume_reference) for rows_ in per_run]
        out["leave_one_out_share"] = min(
            front_hypervolume(_non_dominated([r for j, rows_ in enumerate(per_run) if j != i for r in rows_]),
                              MECHANISM, NAMES, config.hypervolume_reference) / out["hypervolume"]
            for i in range(len(runs)))
    knee = next(r for r in rows if "balanced" in r["seed_role"].split(";"))
    out["knee"] = {key: knee[key] for key in (*MECHANISM.variable_names, "run", "candidate",
                                              "torque_ratio_max", "finger_length_mm", "phi_max_deg",
                                              "required_proximal_mm", "required_distal_mm",
                                              "proximal_support_length_mm", "distal_support_length_mm",
                                              "proximal_face_mm", "distal_face_mm", "pad_clearance_mm",
                                              "min_margin_deg", "torque_ratio_min", "bound_hits",
                                              *[c for c in knee if c.startswith(("b_built", "bp_from", "apb", "abp"))])}
    if extend:
        fine = MECHANISM.refine(config, math.radians(0.05))
        extension = extend_pads(MECHANISM.read_design(knee), fine, mechanism=MECHANISM,
                                priority=config.extension_priority, extra_clearance_mm=0.5)
        e = extension.evaluation
        out["knee"].update(
            post_app_offset_mm=e.parameters.app_offset_mm, post_bpp_offset_mm=e.parameters.bpp_offset_mm,
            post_proximal_support_length_mm=e.proximal_support_length_mm,
            post_distal_support_length_mm=e.distal_support_length_mm,
            post_proximal_face_mm=e.proximal_face_mm, post_distal_face_mm=e.distal_face_mm,
            post_pad_clearance_mm=e.pad_clearance_mm, post_feasible_fine=e.feasible,
            post_torque_ratio_max=e.torque_ratio_max)
    return out


def _non_dominated(rows: list) -> list:
    points = [(float(r["torque_ratio_max"]), float(r["finger_length_mm"])) for r in rows]
    return [row for row, p in zip(rows, points)
            if not any(q[0] <= p[0] and q[1] <= p[1] and q != p for q in points)]


def monotonicity(results: dict) -> list[str]:
    """Constraints are nested: min |OC| (overall and at each cap) and min k_max
    may not fall as phi_req, L_p or L_d rises. Every violation, named."""

    keys = ["min_k_max", "min_finger_length_mm", *[f"min_finger_length_at_k{cap}" for cap in CAPS]]
    lines = [("phi_req", [dict(BASELINE, phi_req=phi) for phi in ROM])]
    lines += [(f"L_p at L_d {d:g}", [dict(BASELINE, L_p=p, L_d=d) for p in PROXIMAL]) for d in DISTAL]
    lines += [(f"L_d at L_p {p:g}", [dict(BASELINE, L_p=p, L_d=d) for d in DISTAL]) for p in PROXIMAL]
    problems = []
    for label, series in lines:
        swept = label.split(" ")[0]
        for metric in keys:
            previous = None
            for values in series:
                result = results.get(condition_name(values), {})
                current = result.get(metric) if result.get("feasible") else math.inf
                current = math.inf if current is None else current
                if previous is not None and current < previous[1] - 1e-9:
                    problems.append(f"{label}: {metric} falls from {previous[1]:.4g} at {previous[0]:g} "
                                    f"to {current:.4g} at {values[swept]:g} ({condition_name(values)})")
                previous = (values[swept], current)
    return problems


def hardest_first() -> list[str]:
    """Every condition after all of its harder neighbours."""

    every = conditions()
    done: list[str] = []
    while len(done) < len(every):
        for name, values in every.items():
            if name not in done and all(condition_name(h) in done for h in harder(values)):
                done.append(name)
    return done


def analyze(args) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    results = {}
    names = args.conditions or hardest_first()
    for name in names:
        runs = condition_runs(name, args.generations)
        if not runs:
            print(f"{name}: no runs")
            continue
        results[name] = metrics(name, runs, extend=not args.no_extend)
        r = results[name]
        print(f"{name}: {len(runs)} runs, front {r['front']}, feasible evaluations {r['feasible_fraction']:.1%}"
              + (f", min k {r['min_k_max']:.3f}, min |OC| {r['min_finger_length_mm']:.1f}, "
                 f"HV {r['hypervolume']:.2f}, leave-one-out {r.get('leave_one_out_share', 1.0):.1%}"
                 if r["feasible"] else ", INFEASIBLE"))
    problems = monotonicity(results) if len(results) == len(conditions()) else ["incomplete sweep"]
    (ROOT / "summary.json").write_text(json.dumps({"results": results, "monotonicity": problems}, indent=2) + "\n")
    print("monotonicity:", "ok" if not problems else "\n  " + "\n  ".join(problems))
    if len(results) != len(conditions()):
        return

    figures = args.figures or ROOT
    figures.mkdir(parents=True, exist_ok=True)
    get = lambda **values: results.get(condition_name(dict(BASELINE, **values)), {})  # noqa: E731

    # 1. Fronts of the ROM sweep.
    figure, axis = plt.subplots(figsize=(6.6, 4.6), constrained_layout=True)
    for phi in ROM:
        r = get(phi_req=phi)
        if r.get("feasible"):
            length, k = zip(*r["front_points"])
            axis.plot(length, k, ".-", label=f"phi_req = {phi:g}°")
        else:
            axis.plot([], [], "x", label=f"phi_req = {phi:g}°: infeasible")
    axis.set(xlabel="finger length |OC| [mm]", ylabel="k_max on [0, phi_req]", title="Fronts, ROM sweep")
    axis.grid(True, color="#e6e8eb")
    axis.legend(fontsize=8)
    figure.savefig(figures / "fronts_rom.png", dpi=130)
    plt.close(figure)

    # 2. min |OC| (per cap) and min k_max against phi_req.
    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.0), constrained_layout=True)
    for cap in (None, *CAPS):
        key = "min_finger_length_mm" if cap is None else f"min_finger_length_at_k{cap}"
        y = [get(phi_req=phi).get(key) if get(phi_req=phi).get("feasible") else None for phi in ROM]
        axes[0].plot(ROM, [math.nan if v is None else v for v in y], "o-" if cap else "k--",
                     label="no cap" if cap is None else f"k_max <= {cap}")
    axes[0].set(xlabel="phi_req [deg]", ylabel="min |OC| [mm]", xticks=ROM)
    axes[0].legend(fontsize=8)
    axes[1].plot(ROM, [get(phi_req=phi).get("min_k_max", math.nan) for phi in ROM], "o-")
    axes[1].set(xlabel="phi_req [deg]", ylabel="min k_max", xticks=ROM)
    for axis in axes:
        axis.grid(True, color="#e6e8eb")
    figure.savefig(figures / "rom_sensitivity.png", dpi=130)
    plt.close(figure)

    # 3. min |OC| over the L_p x L_d grid, per cap.
    caps = (None, *CAPS)
    figure, axes = plt.subplots(1, len(caps), figsize=(4.0 * len(caps), 3.8), constrained_layout=True)
    grids = []
    for cap in caps:
        key = "min_finger_length_mm" if cap is None else f"min_finger_length_at_k{cap}"
        grid = np.array([[get(L_p=p, L_d=d).get(key) if get(L_p=p, L_d=d).get("feasible") else None
                          for p in PROXIMAL] for d in DISTAL], dtype=float)
        grids.append(grid)
    finite = np.concatenate([g[np.isfinite(g)] for g in grids])
    low, high = (float(finite.min()), float(finite.max())) if finite.size else (0.0, 1.0)
    extent = (PROXIMAL[0] - 5, PROXIMAL[-1] + 5, DISTAL[0] - 5, DISTAL[-1] + 5)
    for axis, cap, grid in zip(axes, caps, grids):
        shown = axis.imshow(grid, origin="lower", cmap="viridis", aspect="auto", extent=extent,
                            vmin=low, vmax=high)
        for i, d in enumerate(DISTAL):
            for j, p in enumerate(PROXIMAL):
                value = grid[i, j]
                axis.text(p, d, "infeasible" if not np.isfinite(value) else f"{value:.1f}",
                          ha="center", va="center", fontsize=8,
                          color="black" if np.isfinite(value) and value > (low + high) / 2 else "white")
        axis.set(xlabel="L_p [mm]", ylabel="L_d [mm]", xticks=PROXIMAL, yticks=DISTAL,
                 title="min |OC|, no cap" if cap is None else f"min |OC|, k_max <= {cap}")
    figure.colorbar(shown, ax=axes, label="min |OC| [mm]")
    figure.savefig(figures / "pad_grid_min_length.png", dpi=130)
    plt.close(figure)

    # 4. The knee design's A'' / B'' extensions across the grid.
    figure, axes = plt.subplots(1, 4, figsize=(15, 3.8), constrained_layout=True)
    panels = (("app_offset_mm", "knee A'' offset (optimized)"), ("bpp_offset_mm", "knee B'' offset (optimized)"),
              ("post_app_offset_mm", "knee A'' offset (post-processed)"),
              ("post_bpp_offset_mm", "knee B'' offset (post-processed)"))
    for axis, (key, title) in zip(axes, panels):
        grid = np.array([[float(get(L_p=p, L_d=d)["knee"][key]) if get(L_p=p, L_d=d).get("feasible")
                          and key in get(L_p=p, L_d=d)["knee"] else math.nan
                          for p in PROXIMAL] for d in DISTAL])
        image = axis.imshow(grid, origin="lower", cmap="magma", aspect="auto",
                            extent=(PROXIMAL[0] - 5, PROXIMAL[-1] + 5, DISTAL[0] - 5, DISTAL[-1] + 5))
        for i, d in enumerate(DISTAL):
            for j, p in enumerate(PROXIMAL):
                axis.text(p, d, "x" if not np.isfinite(grid[i, j]) else f"{grid[i, j]:.1f}",
                          ha="center", va="center", fontsize=8, color="white")
        axis.set(xlabel="L_p [mm]", ylabel="L_d [mm]", xticks=PROXIMAL, yticks=DISTAL, title=title)
        figure.colorbar(image, ax=axis, label="mm")
    figure.savefig(figures / "pad_grid_knee_extensions.png", dpi=130)
    plt.close(figure)
    print(f"figures -> {figures}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    runner = commands.add_parser("run")
    runner.add_argument("--conditions", nargs="+", help="condition names, or 'baseline' (default: every one)")
    runner.add_argument("--seeds", nargs="+", type=int, default=[0])
    runner.add_argument("--generations", type=int, default=400)
    runner.add_argument("--workers", type=int, default=8)
    runner.add_argument("--warm-from-harder", action="store_true",
                        help="start half the population from the harder neighbours' verified fronts")
    analysis = commands.add_parser("analyze")
    analysis.add_argument("--generations", type=int, help="only runs of this length")
    analysis.add_argument("--conditions", nargs="+", help="condition names (default: every one)")
    analysis.add_argument("--figures", type=Path, help="default: output/linkage_v2/sweeps")
    analysis.add_argument("--no-extend", action="store_true", help="skip the knee designs' A'' extension")
    args = parser.parse_args(argv)
    {"run": run, "analyze": analyze}[args.command](args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
