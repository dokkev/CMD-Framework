"""The 2R against CoDe carrying the same contact modules: each finger held
fixed, drawn through its motion, and how far it flexes as its proximal
module grows toward the joint (``core.optimization.analysis.tradeoff``).

    python -m visualize.comparison.modules

Rows: the 2R (servo on the joint) and CoDe, both with the drawn modules
(DRAWN), at extension, 90 deg, 120 deg and each one's own most flexion.
Right: ``phi_max`` against the proximal module's length, for each distal
module thickness of DISTAL_HEIGHTS_MM (the distal length DISTAL_LENGTH_MM,
the proximal thickness PROXIMAL_HEIGHT_MM). Draft: CoDe is the production
design (made for 5 mm modules), the 2R a grid pick for the 10 mm distal
module, its supports as long as the housing allows; each finger fixed (its supports too), only the modules change; the motion on a 1 deg grid. Saves
output/comparison/modules.png and the curves as .csv.
"""

from __future__ import annotations

import csv
import dataclasses
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from core.morphology.linkage import FingerLinkage  # noqa: E402
from core.optimization.analysis.tradeoff import Modules, load, phi_max_deg, proximal_curve, with_modules  # noqa: E402
from core.optimization.linkage.design import decode  # noqa: E402
from core.optimization.revolute.evaluate import geometry  # noqa: E402
from visualize import sketch  # noqa: E402
from visualize.finger import Finger  # noqa: E402
from visualize.fronts import read_rows, run_of, with_role  # noqa: E402
from visualize.style import COLLIDE, COLORS, INK, MUTED, REPOSITORY_ROOT, save_comparison  # noqa: E402

STUDY = REPOSITORY_ROOT / "scripts/optimize/accommodation"
CODE_SEEDS = REPOSITORY_ROOT / "output/linkage/config/paper/portfolio_001/seeds.csv"
REVOLUTE_X = (47.5, 52.5, 38.0, 9.5, 0.0, 0.0, 90.0)
"""The 2R: A at 47.5 of |OC| 100, the pads on its link lines (drop 0), the
servo upright, its supports up to the joint's housing (A'' and B'' 9.5 from
A): on the link line a support cannot run into the housing."""
OVERRIDES = ("goal.link.dorsal_space_mm=34", "goal.link.proximal_length_ratio=[0.30, 0.70]",
             "goal.link.distal_length_ratio=[0.25, 0.70]")
PROXIMAL_HEIGHT_MM = 5.0
DISTAL_LENGTH_MM = 25.0
DISTAL_HEIGHTS_MM = (5.0, 10.0)
DRAWN = Modules(28.0, PROXIMAL_HEIGHT_MM, DISTAL_LENGTH_MM, 10.0)
LENGTHS_MM = np.arange(1.0, 50.0 + 1e-9, 1.0)
POSES_DEG = (0.0, 90.0, 120.0)
NAMES = {"revolute": "2R (servo on the joint)", "linkage": "CoDe (production)"}


def code_design() -> tuple[float, ...]:
    row = with_role(read_rows(CODE_SEEDS), "balanced")
    x = list(run_of(CODE_SEEDS, row).mechanism.read_design(row))
    x[-2] = x[-1] = 3.0  # its pads' drops: the packaging's (A's and B's pin radius)
    return tuple(x)


def fingers(modules: Modules):
    """Each mechanism: its mechanism, config, fixed design and the design with ``modules``."""

    out = {}
    for name, x in (("revolute", REVOLUTE_X), ("linkage", code_design())):
        found, config = load(STUDY / f"{name}_P25x5_D25x5_flex150.yaml", *OVERRIDES)
        out[name] = (found, config, x, with_modules(config, modules))
    return out


def finger(name: str, config, x) -> Finger:
    state = FingerLinkage(decode(x, config.link_length_resolution_mm)) if name == "linkage" else geometry(x, config)
    return Finger(name, {}, config, tuple(x), state)


def main() -> int:
    built = fingers(DRAWN)
    figure = plt.figure(figsize=(14.5, 6.2), constrained_layout=True)
    grid = figure.add_gridspec(2, 5, width_ratios=(1, 1, 1, 1, 1.5))
    rows = []
    for r, (name, (found, config, x, carried)) in enumerate(built.items()):
        reach = phi_max_deg(found, config, x, DRAWN)
        drawn = finger(name, carried, x)
        for c, pose in enumerate((*POSES_DEG, reach)):
            axis = figure.add_subplot(grid[r, c])
            sketch.frame(axis, (-14.0, 106.0), (-64.0, 34.0))
            shown = min(pose, reach)
            drawn.draw(axis, shown)
            short = pose > reach + 1e-9
            title = f"φ = {shown:.0f}°" + (" (most)" if c == 3 else "") + (f": stops at {reach:.0f}°" if short else "")
            axis.set_title(title, color=COLLIDE if short else INK, fontsize=8, loc="left")
            if c == 0:
                axis.text(-12.0, 31.0, NAMES[name], color=COLORS[name], fontsize=9, fontweight="bold", va="top")
    curves = figure.add_subplot(grid[:, 4])
    for name in ("revolute", "linkage"):
        for height, alpha in zip(DISTAL_HEIGHTS_MM, (0.45, 1.0)):
            modules = Modules(1.0, PROXIMAL_HEIGHT_MM, DISTAL_LENGTH_MM, height)
            found, config, x, _ = fingers(modules)[name]
            reach = proximal_curve(found, config, x, modules, LENGTHS_MM)
            curves.plot(LENGTHS_MM, reach, color=COLORS[name], alpha=alpha, linewidth=1.8,
                        label=f"{NAMES[name]}, distal {DISTAL_LENGTH_MM:g} x {height:g} mm")
            rows += [{"mechanism": name, "proximal_length_mm": length, "proximal_height_mm": PROXIMAL_HEIGHT_MM,
                      "distal_length_mm": DISTAL_LENGTH_MM, "distal_height_mm": height,
                      "phi_max_deg": "" if math.isnan(value) else f"{value:.2f}"}
                     for length, value in zip(LENGTHS_MM, reach)]
    curves.axhline(150.0, color=MUTED, linestyle=(0, (4, 2)), linewidth=1.0)
    curves.text(1.0, 151.0, "φ_req 150°", color=MUTED, fontsize=7.5)
    curves.set_xlabel(f"proximal module length ℓp [mm] (thickness {PROXIMAL_HEIGHT_MM:g} mm)", color=INK)
    curves.set_ylabel("continuous flexion φ_max [deg]", color=INK)
    curves.set_ylim(90.0, 185.0)
    curves.grid(True, color="#e3e2dd", linewidth=0.6)
    curves.legend(fontsize=7, frameon=False, loc="lower left")
    curves.set_title("fixed fingers: flexion vs proximal module\n(no line: the module does not mount)",
                     color=INK, fontsize=8.5, loc="left")
    figure.suptitle(f"DRAFT: 2R vs CoDe with the same modules (drawn: proximal {DRAWN.proximal_length_mm:g} x "
                    f"{DRAWN.proximal_height_mm:g} mm, distal {DRAWN.distal_length_mm:g} x {DRAWN.distal_height_mm:g} mm); "
                    "CoDe = production design made for 5 mm modules; dorsal space 34 mm, clearance 0, MCP 0°",
                    color=COLLIDE, fontsize=8.5, x=0.01, ha="left")
    out = save_comparison(figure, "modules")
    with out.with_suffix(".csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
