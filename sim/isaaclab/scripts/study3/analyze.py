# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""Study 3: figures and tables from the pilot's training logs and evaluations (no simulation).

    uv run python scripts/study3/analyze.py [--eval_root logs/study3_pilot/eval]

Reads every training run with a ``study3_manifest.json`` (smoke runs excluded) and every evaluation summary,
and writes, under ``report/study3/``:

- ``figures/study3_learning_curves.png``: success rate, episode length and reward against environment
  transitions, per variant and seed;
- ``figures/study3a_transfer_matrix.png`` (and ``study3b_...`` per backend): first-goal success of each
  training variant on each test variant, seed mean with each seed's value;
- ``figures/study3_trajectories.png``: the cube's yaw against its goal for a few matched episodes;
- ``tables/cells.csv``: one row per evaluation (training variant, seed, test variant, backend) with success,
  goals, drops, validity, contact summaries; ``tables/gaps.csv``: transfer and solver gaps per seed;
- ``tables/episodes.csv.gz``: every evaluated episode.

Each evaluation's latest run per cell is used. Gaps are differences of first-goal success rates (fractions).
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

REPORT = common.REPO_ROOT / "report" / "study3"
COLORS = {"flat": "#2a78d6", "crowned": "#eb6834"}
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
VARIANTS = ("flat", "crowned")
PHYSICS = ("newton_mjwarp", "newton_vbd")
HELD_STEPS = 30
"""A first goal counts as held when the cube is not dropped in the 30 policy steps (1 s) after it."""
PHYSICS_LABEL = {"newton_mjwarp": "MuJoCo-Warp", "newton_vbd": "VBD"}


def style(ax) -> None:
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.tick_params(labelsize=7, colors=MUTED)


def training_runs() -> list[dict]:
    runs = []
    for manifest in sorted((common.SIM_ROOT / "logs" / "rsl_rl" / "study3_code_reorient").glob("*/study3_manifest.json")):
        data = json.loads(manifest.read_text())
        if "smoke" in Path(data["log_dir"]).name or data.get("status") != "passed" or data.get("overrides"):
            continue
        data["dir"] = manifest.parent
        runs.append(data)
    return runs


def curves(run: dict) -> dict[str, np.ndarray]:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

    events = next(run["dir"].glob("events.out.tfevents.*"))
    acc = EventAccumulator(str(events), size_guidance={"scalars": 0})
    acc.Reload()
    per_iteration = run["steps_per_env"] * run["num_envs"]
    out = {}
    for tag in ("Metrics/success_rate", "Train/mean_episode_length", "Train/mean_reward",
                "Episode_Termination/object_out_of_reach"):
        values = acc.Scalars(tag)
        out[tag] = np.array([[(v.step + 1) * per_iteration, v.value] for v in values])
    return out


def smooth(y: np.ndarray, window: int = 15) -> np.ndarray:
    if len(y) < window:
        return y
    kernel = np.ones(window) / window
    padded = np.concatenate([np.full(window // 2, y[0]), y, np.full(window - 1 - window // 2, y[-1])])
    return np.convolve(padded, kernel, mode="valid")


def learning_figure(runs: list[dict]) -> Path:
    panels = [("Metrics/success_rate", "success rate\n(goals / (goals + 1))"),
              ("Train/mean_episode_length", "episode length\n[policy steps]"),
              ("Train/mean_reward", "mean reward")]
    fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.3))
    for run in runs:
        data = curves(run)
        for ax, (tag, _) in zip(axes, panels):
            x, y = data[tag][:, 0] / 1e6, data[tag][:, 1]
            ax.plot(x, smooth(y), color=COLORS[run["variant"]], linewidth=1.0, alpha=0.75)
    for variant in VARIANTS:
        seeds = sorted(r["seed"] for r in runs if r["variant"] == variant)
        axes[0].plot([], [], color=COLORS[variant], linewidth=1.4, label=f"{variant} ({len(seeds)} seeds)")
    for ax, (_, label) in zip(axes, panels):
        style(ax)
        ax.set_xlabel("environment transitions [M]", fontsize=7, color=INK)
        ax.set_ylabel(label, fontsize=7, color=INK)
    axes[0].legend(frameon=False, fontsize=6.5, loc="upper left")
    fig.tight_layout()
    return save(fig, "study3_learning_curves.png")


def save(fig, name: str) -> Path:
    path = REPORT / "figures" / name
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


def evaluations(root: Path) -> dict[tuple, dict]:
    """The latest evaluation per (training variant, training seed, test variant, backend)."""
    found = {}
    for summary_path in sorted(root.glob("*/summary.json")):
        s = json.loads(summary_path.read_text())
        key = (s["train_variant"], s["train_seed"], s["test_variant"], s["physics"])
        s["dir"] = summary_path.parent
        found[key] = s  # sorted by run id (time): the last one wins
    return found


def episodes(summary: dict) -> list[dict]:
    with (summary["dir"] / "episodes.csv").open() as handle:
        return list(csv.DictReader(handle))


def cell_rows(evals: dict[tuple, dict]) -> list[dict]:
    rows = []
    for (tv, seed, xv, physics), s in sorted(evals.items()):
        eps = episodes(s)
        num = lambda k: [float(e[k]) for e in eps if e[k] != ""]  # noqa: E731
        valid = [e for e in eps if e["valid"] == "1"]
        success = np.array(num("success_first_goal"))
        # the first goal reached with the cube still held: not dropped within HELD_STEPS after it (or at all)
        held = np.array([
            float(e["success_first_goal"]) == 1.0 and (e["dropped"] == "0" or
                                                     int(e["length"]) - float(e["steps_to_first_goal"]) >= HELD_STEPS)
            for e in eps], dtype=float)
        rows.append({
            "train_variant": tv, "train_seed": seed, "test_variant": xv, "physics": physics,
            "episodes": len(eps),
            "success_first_goal": success.mean(),
            "success_se": success.std(ddof=1) / math.sqrt(len(success)) if len(success) > 1 else float("nan"),
            "success_held": held.mean(),
            "success_first_goal_valid_only": np.mean([float(e["success_first_goal"]) for e in valid]) if valid else "",
            "goals_reached_mean": np.mean(num("goals_reached")),
            "drop_rate": np.mean(num("dropped")),
            "valid_fraction": len(valid) / len(eps),
            "length_mean": np.mean(num("length")),
            "steps_to_first_goal_median": float(np.median(num("steps_to_first_goal"))) if num("steps_to_first_goal") else "",
            "min_error_first_goal_median": float(np.median(num("min_error_first_goal_rad"))) if num("min_error_first_goal_rad") else "",
            "action_abs_mean": np.mean(num("action_abs_mean")),
            "tracking_rad_mean": np.mean(num("tracking_rad_mean")),
            "loop_residual_mm_max_median": float(np.median(num("loop_residual_mm_max"))),
            "pads_in_contact_mean": np.mean(num("pads_in_contact_mean")) if num("pads_in_contact_mean") else "",
            "contact_fraction": np.mean(num("contact_fraction")) if num("contact_fraction") else "",
            "nonpad_force_n_mean": np.mean(num("nonpad_force_n_mean")) if num("nonpad_force_n_mean") else "",
            "initial_state_sha256": s["initial_state_sha256"][:16],
            "checkpoint_sha256": s["checkpoint_sha256"][:16],
            "eval_run": s["run"],
        })
    return rows


def gap_rows(cells: list[dict], metric: str = "success_first_goal") -> list[dict]:
    index = {(c["train_variant"], c["train_seed"], c["test_variant"], c["physics"]): c[metric] for c in cells}
    rows = []
    for (tv, seed, xv, physics), value in sorted(index.items()):
        if xv != tv:
            continue
        other = "crowned" if tv == "flat" else "flat"
        row = {"train_variant": tv, "train_seed": seed, "physics": physics, "in_domain": value}
        if (tv, seed, other, physics) in index:
            row["cross"] = index[(tv, seed, other, physics)]
            row["transfer_gap"] = value - index[(tv, seed, other, physics)]
        rows.append(row)
    solver = []
    for (tv, seed, xv, physics), value in sorted(index.items()):
        if physics != "newton_mjwarp" or (tv, seed, xv, "newton_vbd") not in index:
            continue
        solver.append({"train_variant": tv, "train_seed": seed, "test_variant": xv,
                       "mjwarp": value, "vbd": index[(tv, seed, xv, "newton_vbd")],
                       "solver_gap": value - index[(tv, seed, xv, "newton_vbd")]})
    return rows, solver


def matrix_figure(cells: list[dict], physics: str, name: str, metric: str = "success_first_goal",
                  label: str = "first-goal success") -> Path | None:
    data = defaultdict(list)
    for c in cells:
        if c["physics"] == physics:
            data[(c["train_variant"], c["test_variant"])].append((c["train_seed"], c[metric]))
    if not data:
        return None
    fig, ax = plt.subplots(figsize=(3.3, 2.7))
    grid = np.full((2, 2), np.nan)
    for i, tv in enumerate(VARIANTS):
        for j, xv in enumerate(VARIANTS):
            values = data.get((tv, xv), [])
            if values:
                grid[i, j] = np.mean([v for _, v in values])
    ax.imshow(grid, cmap="Blues", vmin=0.0, vmax=max(0.5, float(np.nanmax(grid)) * 1.15))
    for i, tv in enumerate(VARIANTS):
        for j, xv in enumerate(VARIANTS):
            values = sorted(data.get((tv, xv), []))
            if not values:
                continue
            mean = np.mean([v for _, v in values])
            parts = [f"{100 * v:.0f}" for _, v in values]
            seeds = ", ".join(parts[:3]) + (",\n" + ", ".join(parts[3:]) if len(parts) > 3 else "")
            ax.text(j, i - 0.08, f"{100 * mean:.0f}%", ha="center", va="center", fontsize=10, fontweight="bold",
                    color="white" if mean > 0.35 else INK)
            ax.text(j, i + 0.25, f"seeds: {seeds}", ha="center", va="center", fontsize=6,
                    color="white" if mean > 0.35 else MUTED)
    ax.set_xticks([0, 1], [f"test: {v}" for v in VARIANTS], fontsize=7)
    ax.set_yticks([0, 1], [f"train: {v}" for v in VARIANTS], fontsize=7)
    ax.set_title(f"{label}, {PHYSICS_LABEL[physics]}", fontsize=8, color=INK)
    for side in ax.spines.values():
        side.set_visible(False)
    return save(fig, name)


def trajectory_figure(evals: dict[tuple, dict], seed: int) -> Path | None:
    pairs = [(tv, xv) for tv in VARIANTS for xv in VARIANTS]
    if not all((tv, seed, xv, "newton_mjwarp") in evals for tv, xv in pairs):
        return None
    fig, axes = plt.subplots(2, 2, figsize=(7.16, 3.6), sharex=True, sharey=True)
    for ax, (tv, xv) in zip(axes.flat, pairs):
        npz = np.load(evals[(tv, seed, xv, "newton_mjwarp")]["dir"] / "trajectories.npz")
        yaw, goal, dt = npz["yaw"], npz["goal"], float(npz["dt"])
        t = np.arange(len(yaw)) * dt
        for k in range(min(4, yaw.shape[1])):
            ax.plot(t, np.degrees(yaw[:, k]), color=COLORS[xv], linewidth=1.0, alpha=0.9)
            ax.plot(t, np.degrees(goal[:, k]), color=MUTED, linewidth=0.8, linestyle=(0, (2, 2)))
        style(ax)
        ax.set_title(f"trained on {tv}, tested on {xv}", fontsize=7.5, color=INK)
    for ax in axes[1]:
        ax.set_xlabel("time [s]", fontsize=7)
    for ax in axes[:, 0]:
        ax.set_ylabel("cube yaw [deg]\n(dashed: goal)", fontsize=7)
    fig.tight_layout()
    return save(fig, f"study3_trajectories_seed{seed}.png")


def validation(root: Path = common.LOG_ROOT / "validate", tag: str = "gate0_implicit") -> dict[tuple, dict]:
    """The latest Gate 0 rollout per (variant, backend, policy)."""
    found = {}
    for path in sorted(root.glob(f"*_{tag}_*/summary.json")):
        s = json.loads(path.read_text())
        found[(s["variant"], s["physics"], s["policy"])] = s
    return found


def validation_rows(runs: dict[tuple, dict]) -> list[dict]:
    rows = []
    for (variant, physics, policy), s in sorted(runs.items()):
        mean, peak = s["mean"], s["max"]
        rows.append({
            "variant": variant, "physics": physics, "policy": policy, "num_envs": s["num_envs"], "steps": s["steps"],
            "all_finite": s["all_finite"], "drops": s["terminations"]["object_out_of_reach"],
            "time_outs": s["terminations"]["time_out"],
            "tracking_rad_mean": mean["tracking_rad"], "loop_residual_mm_max": peak["loop_residual_mm_max"],
            "passive_sync_rad_max": peak["passive_sync_rad_max"], "cube_yaw_abs_max_rad": peak["cube_rot_abs_z"],
            "pads_in_contact_mean": mean.get("pads_in_contact", ""), "stray_contact_n_max": peak.get("stray_contact_n_max", ""),
            "nonpad_force_on_cube_n_max": peak.get("nonpad_force_on_cube_n", ""),
            "env_steps_per_s": s["env_steps_per_s"], "servo": s["manifest"].get("overrides", ""), "run": s["run"],
        })
    return rows


def prescribed_figure(runs: dict[tuple, dict]) -> Path | None:
    """The cube's yaw under the same prescribed abduction on both backends (before any policy)."""
    keys = [(v, p, "abduct") for v in VARIANTS for p in PHYSICS]
    if not all(k in runs for k in keys):
        return None
    fig, ax = plt.subplots(figsize=(3.5, 2.4))
    for v, p, _ in keys:
        rows = runs[(v, p, "abduct")]["rows"]
        t = np.arange(len(rows)) / 30.0
        ax.plot(t, np.degrees([r["cube_rot_abs_z"] for r in rows]), color=COLORS[v],
                linestyle="-" if p == "newton_mjwarp" else (0, (3, 2)), linewidth=1.4,
                label=f"{v}, {PHYSICS_LABEL[p]}")
    style(ax)
    ax.set_xlabel("time [s]", fontsize=7)
    ax.set_ylabel("cube |yaw| [deg], env mean", fontsize=7)
    ax.set_title("same abduction command, before training", fontsize=7.5, color=INK)
    ax.legend(frameon=False, fontsize=6.5, loc="center left", bbox_to_anchor=(1.02, 0.5))
    fig.tight_layout()
    return save(fig, "study3b_prescribed_abduction.png")


SEED_METRICS = ("success_first_goal", "success_held", "drop_rate", "goals_reached_mean", "length_mean",
                "nonpad_force_n_mean", "pads_in_contact_mean", "action_abs_mean")


def permutation_p(a: list[float], b: list[float]) -> float:
    """Exact two-sided permutation p-value of the difference of means (seeds as the unit)."""
    import itertools

    pooled, n = list(a) + list(b), len(a)
    observed = abs(np.mean(a) - np.mean(b))
    count = total = 0
    for chosen in itertools.combinations(range(len(pooled)), n):
        rest = [pooled[i] for i in range(len(pooled)) if i not in chosen]
        total += 1
        count += abs(np.mean([pooled[i] for i in chosen]) - np.mean(rest)) >= observed - 1e-12
    return count / total


def seed_stats(cells: list[dict], physics: str = "newton_mjwarp") -> list[dict]:
    """Per metric: in-domain flat-trained against crowned-trained policies, and the transfer gap of each
    training variant, with seeds as the unit (mean, sd, exact permutation p, Hedges' g)."""
    rows = []
    def values(tv, xv, metric):
        return [float(c[metric]) for c in cells if c["physics"] == physics and c["train_variant"] == tv
                and c["test_variant"] == xv and c[metric] != ""]
    for metric in SEED_METRICS:
        flat, crowned = values("flat", "flat", metric), values("crowned", "crowned", metric)
        if len(flat) < 2 or len(crowned) < 2:
            continue
        pooled = math.sqrt(((len(flat) - 1) * np.var(flat, ddof=1) + (len(crowned) - 1) * np.var(crowned, ddof=1))
                           / (len(flat) + len(crowned) - 2))
        g = (np.mean(crowned) - np.mean(flat)) / pooled if pooled > 0 else float("inf")
        g *= 1 - 3 / (4 * (len(flat) + len(crowned)) - 9)
        rows.append({"comparison": "in-domain crowned - flat", "metric": metric, "n_flat": len(flat),
                     "n_crowned": len(crowned), "flat_mean": np.mean(flat), "flat_sd": np.std(flat, ddof=1),
                     "crowned_mean": np.mean(crowned), "crowned_sd": np.std(crowned, ddof=1),
                     "difference": np.mean(crowned) - np.mean(flat), "hedges_g": g,
                     "permutation_p": permutation_p(flat, crowned)})
    for tv, xv in (("flat", "crowned"), ("crowned", "flat")):
        for metric in ("success_first_goal", "success_held", "drop_rate"):
            index = {c["train_seed"]: float(c[metric]) for c in cells
                     if c["physics"] == physics and c["train_variant"] == tv and c["test_variant"] == tv}
            cross = {c["train_seed"]: float(c[metric]) for c in cells
                     if c["physics"] == physics and c["train_variant"] == tv and c["test_variant"] == xv}
            gaps = [index[k] - cross[k] for k in index if k in cross]
            if len(gaps) < 2:
                continue
            # exact sign-flip test of a zero mean gap over seeds
            import itertools
            flips = [abs(np.mean([g * s for g, s in zip(gaps, signs)])) for signs in itertools.product((1, -1), repeat=len(gaps))]
            p = np.mean([f >= abs(np.mean(gaps)) - 1e-12 for f in flips])
            rows.append({"comparison": f"transfer gap {tv} -> {xv}", "metric": metric, "n_flat": len(gaps),
                         "n_crowned": "", "flat_mean": np.mean(gaps), "flat_sd": np.std(gaps, ddof=1),
                         "crowned_mean": "", "crowned_sd": "", "difference": np.mean(gaps), "hedges_g": "",
                         "permutation_p": p})
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "wt", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--eval_root", type=Path, default=common.LOG_ROOT / "eval")
    args = parser.parse_args(argv)
    gate = validation()
    if gate:
        write_csv(REPORT / "tables" / "validation.csv", validation_rows(gate))
        print(prescribed_figure(gate))
    runs = training_runs()
    print(f"{len(runs)} training runs")
    if runs:
        print(learning_figure(runs))
        write_csv(REPORT / "tables" / "training.csv", [
            {k: r[k] for k in ("variant", "seed", "iterations", "num_envs", "transitions", "wall_s", "status")}
            | {"final_checkpoint_sha256": (r["final_checkpoint_sha256"] or "")[:16], "log_dir": Path(r["log_dir"]).name,
               "git_commit": r["git_commit"][:10], "git_dirty": r["git_dirty"],
               "final_success_rate": float(curves(r)["Metrics/success_rate"][-20:, 1].mean()),
               "final_episode_length": float(curves(r)["Train/mean_episode_length"][-20:, 1].mean())}
            for r in runs])
    evals = evaluations(args.eval_root)
    print(f"{len(evals)} evaluation cells")
    if evals:
        cells = cell_rows(evals)
        write_csv(REPORT / "tables" / "cells.csv", cells)
        write_csv(REPORT / "tables" / "seed_stats.csv", seed_stats(cells))
        for metric in ("success_first_goal", "success_held"):
            transfer, solver = gap_rows(cells, metric)
            write_csv(REPORT / "tables" / f"transfer_gaps_{metric}.csv", transfer)
            write_csv(REPORT / "tables" / f"solver_gaps_{metric}.csv", solver)
        print(matrix_figure(cells, "newton_mjwarp", "study3a_transfer_matrix_held.png", "success_held",
                            "first goal reached, cube held 1 s"))
        write_csv(REPORT / "tables" / "episodes.csv.gz", [e for s in evals.values() for e in episodes(s)])
        for physics, name in (("newton_mjwarp", "study3a_transfer_matrix.png"),
                              ("newton_vbd", "study3b_transfer_matrix_vbd.png")):
            print(matrix_figure(cells, physics, name))
        for seed in sorted({k[1] for k in evals}):
            print(trajectory_figure(evals, seed))
        hashes = {c["initial_state_sha256"] for c in cells}
        print(f"initial-state hashes across cells: {len(hashes)} distinct (1 = matched)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
