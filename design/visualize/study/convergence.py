"""Each run's best pad coverage against the generation: did the search converge?

    python -m visualize.study.convergence <config dir> ... [--output name]

``config dir``: a study config's output (``output/comparison/<mechanism>/<config>``),
its ``run_NNN`` each with a ``history.npz``. Writes each config's
``convergence.csv`` (run, optimizer, seed, generation, evaluations, best) and
one figure, a panel per config, a line per run, to
output/comparison/<name>.png (default ``convergence``). Prints per run the
final best, the generation it came within 0.1 %p of it and the gain over the
last 20 % of the generations.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from core.optimization.analysis.convergence import best_by_generation, load_history  # noqa: E402
from visualize.style import COLORS, GRID, INK, MUTED, axes_style, save_comparison, use  # noqa: E402

TOLERANCE = 1e-3
"""0.1 %p of coverage."""
LATE_SHARE = 0.8
SEED_DASHES = ("-", (0, (5, 2)), (0, (1.5, 1.5)), (0, (6, 2, 1, 2)))


def runs_of(config_dir: Path) -> list[tuple[Path, dict]]:
    """The config's runs with a history, and each one's optimizer settings."""

    found = []
    for run in sorted(config_dir.glob("run_*")):
        if (run / "history.npz").exists():
            found.append((run, json.loads((run / "run.json").read_text(encoding="utf-8"))))
    return found


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("configs", nargs="+", type=Path)
    parser.add_argument("--output", default="convergence")
    args = parser.parse_args(argv)
    use("report")
    columns = min(len(args.configs), 3)
    rows = -(-len(args.configs) // columns)
    figure, axes = plt.subplots(rows, columns, figsize=(4.4 * columns, 3.3 * rows), squeeze=False,
                                constrained_layout=True)
    for axis in axes.flat[len(args.configs):]:
        axis.axis("off")
    print(f"{'config':34s} {'run':8s} {'optimizer':10s} {'final %':>8s} {'within 0.1%p at':>16s} "
          f"{'late gain %p':>13s}")
    for axis, config_dir in zip(axes.flat, args.configs):
        axes_style(axis, GRID)
        table = []
        for index, (run, info) in enumerate(runs_of(config_dir)):
            history = load_history(run)
            curve = best_by_generation(history, "pad_coverage", larger_is_better=True)
            optimizer = info.get("optimizer", {})
            mechanism = info.get("mechanism", "linkage")
            generations = range(len(curve.best))
            table += [{"run": run.name, "algorithm": optimizer.get("algorithm"), "seed": optimizer.get("seed"),
                       "generation": g, "evaluations": int(curve.evaluations[g]),
                       "best_pad_coverage": float(curve.best[g])} for g in generations]
            axis.plot([g + 1 for g in generations], 100.0 * curve.best, color=COLORS.get(mechanism, INK),
                      linestyle=SEED_DASHES[index % len(SEED_DASHES)], linewidth=1.4,
                      label=f"{optimizer.get('algorithm')}/{optimizer.get('seed')}")
            within = curve.generation_within(TOLERANCE)
            print(f"{config_dir.name:34s} {run.name:8s} {optimizer.get('algorithm', '')!s:10s} "
                  f"{100 * curve.final:8.2f} {('-' if within is None else within + 1):>16} "
                  f"{100 * curve.gain_after(LATE_SHARE):13.2f}")
        if table:
            with (config_dir / "convergence.csv").open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(table[0]))
                writer.writeheader()
                writer.writerows(table)
        axis.set_xscale("log")
        axis.set_xlabel("generation", color=INK)
        axis.set_ylabel("best pad coverage [%]", color=INK)
        axis.set_title(f"{config_dir.parent.name}/{config_dir.name}", color=INK, fontsize=9, loc="left")
        best = [v for line in axis.get_lines() for v in line.get_ydata() if math.isfinite(v)]
        if best:
            top = max(best)
            axis.set_ylim(max(min(best), top - 15.0), top + 1.0)  # the last 15 %p: where it settles
        axis.legend(frameon=False, fontsize=7, loc="lower right")
    figure.suptitle("Best pad coverage found up to each generation, per run (feasible designs; optimizer grid)",
                    color=MUTED, fontsize=9, x=0.01, ha="left")
    print(f"saved {save_comparison(figure, args.output)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
