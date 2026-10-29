"""How a portfolio's runs converged, generation by generation.

    python -m visualize.paper.fig_convergence <portfolio dir> [k cap]

``portfolio dir``: a study's ``portfolio_NNN`` (its ``portfolio.json`` names
the runs); each run needs its ``history.npz`` (runs saved since it was
recorded). The front after a generation is the non-dominated set of the
run's feasible evaluations up to it (its archive then), on the optimizer's
grid. Left: its hypervolume, the objectives normalized between the ideal and
the nadir of all the runs' last fronts together (reference point 1.1 in
each), as a share of the hypervolume of those fronts merged. Right: the most
pad coverage with k_max at most ``k cap`` (default 2.0). One colour per
optimizer, one line per seed. Saves
output/paper/fig_convergence_<study>_<preset>.png / .pdf.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from pymoo.indicators.hv import HV  # noqa: E402

from core.optimization.study import load_run, non_dominated  # noqa: E402
from visualize.style import INK, MUTED, axes_style, save, use  # noqa: E402

ALGORITHM_COLORS = {"nsga2": "#2a78d6", "smsemoa": "#eb6834", "agemoea": "#1f9e74"}
SEED_DASHES = ("-", (0, (5, 2)), (0, (1.5, 1.5)))
CHECKPOINTS = 60
"""Generations the fronts are measured at (log spaced, plus the first and last)."""


def fronts(history, checkpoints: np.ndarray):
    """The run's front (minimized objectives) after each checkpoint generation."""

    generation, objectives = history["generation"], history["objectives"]
    front = np.empty((0, objectives.shape[1]))
    start = 0
    for last in checkpoints:
        stop = int(np.searchsorted(generation, last, side="right"))
        if stop > start:
            pool = np.vstack((front, objectives[start:stop]))
            front = pool[non_dominated(pool)]
            start = stop
        yield front


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__)
        return 2
    use("paper")
    portfolio = Path(argv[0]).resolve()
    k_cap = float(argv[1]) if len(argv) > 1 else 2.0
    names = json.loads((portfolio / "portfolio.json").read_text())["runs"]
    runs = []
    for name in names:
        path = portfolio.parent / name / "history.npz"
        if not path.exists():
            print(f"{name}: no history.npz (saved before it was recorded), skipped")
            continue
        history = dict(np.load(path))
        settings = load_run(portfolio.parent / name).settings
        runs.append((name, settings, history))
    if not runs:
        return 1
    objective_names = [str(name) for name in runs[0][2]["objective_names"]]
    k_index, pad_index = objective_names.index("transmission"), objective_names.index("pad_coverage")

    finals = [history["objectives"][non_dominated(history["objectives"])] for _, _, history in runs]
    merged = np.vstack(finals)
    merged = merged[non_dominated(merged)]
    ideal, nadir = merged.min(axis=0), merged.max(axis=0)
    span = np.where(nadir > ideal, nadir - ideal, 1.0)
    indicator = HV(ref_point=np.full(len(ideal), 1.1))
    hv = lambda front: indicator((front - ideal) / span) if len(front) else 0.0  # noqa: E731
    reference = hv(merged)

    figure, (hv_axis, pad_axis) = plt.subplots(1, 2, figsize=(9.0, 3.4), constrained_layout=True)
    seeds: dict[str, int] = {}
    print(f"{'run':10s} {'optimizer':12s} HV % at 30% / last of the generations;  "
          f"pad % (k <= {k_cap:g}) at 30% / last")
    for name, settings, history in runs:
        generations = len(history["evaluations"])
        checkpoints = np.unique(np.concatenate((
            [0], np.round(np.geomspace(1, generations - 1, CHECKPOINTS)), [generations - 1]))).astype(int)
        shares, pads = [], []
        for front in fronts(history, checkpoints):
            shares.append(100.0 * hv(front) / reference)
            within = front[front[:, k_index] <= k_cap + 1e-12]
            pads.append(100.0 * -within[:, pad_index].min() if len(within) else np.nan)
        shares, pads = np.array(shares), np.array(pads)
        color = ALGORITHM_COLORS.get(settings.algorithm, INK)
        dash = SEED_DASHES[seeds.setdefault(settings.algorithm, 0) % len(SEED_DASHES)]
        seeds[settings.algorithm] += 1
        hv_axis.plot(checkpoints + 1, shares, color=color, linestyle=dash, linewidth=1.4)
        pad_axis.plot(checkpoints + 1, pads, color=color, linestyle=dash, linewidth=1.4)
        third = int(np.searchsorted(checkpoints, 0.3 * generations))
        print(f"{name:10s} {settings.algorithm + '/' + str(settings.seed):12s} {shares[third]:6.1f} / "
              f"{shares[-1]:6.1f}{'':34s}{pads[third]:6.1f} / {pads[-1]:6.1f}")

    for axis in (hv_axis, pad_axis):
        axes_style(axis)
        axis.set_xscale("log")
        axis.set_xlabel("generation", color=INK)
    hv_axis.set_ylabel("hypervolume, % of the merged front's", color=INK)
    pad_axis.set_ylabel(f"most pad coverage with k_max ≤ {k_cap:g} [%]", color=INK)
    algorithms = [Line2D([], [], color=ALGORITHM_COLORS.get(a, INK), linewidth=1.6, label=a) for a in seeds]
    seed_lines = [Line2D([], [], color=MUTED, linestyle=SEED_DASHES[i], linewidth=1.4, label=f"seed {i + 1}")
                  for i in range(max(seeds.values()))]
    hv_axis.legend(handles=algorithms + seed_lines, frameon=False, fontsize=7.5, loc="lower right")
    study = portfolio.parent
    figure.suptitle(f"{study.parent.name}/{study.name}: convergence of {len(runs)} runs "
                    "(each run's front up to the generation, optimizer grid)",
                    color=MUTED, fontsize=8.5, x=0.01, ha="left")
    print(f"saved {save(figure, f'fig_convergence_{study.parent.name}_{study.name}')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
