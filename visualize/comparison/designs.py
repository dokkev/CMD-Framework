"""The two designs the comparison figures put side by side: a representative
linkage of a verified front, and the serial 2R with the same finger and pads."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from core.morphology.linkage import FingerLinkage
from core.optimization.linkage.design import decode
from core.optimization.mechanism import mechanism
from core.optimization.study import load_yaml
from visualize.fronts import read_rows, run_of, with_role
from visualize.style import OUTPUT, REPOSITORY_ROOT

DEFAULT_SEEDS = OUTPUT / "linkage/config/paper/portfolio_001/seeds.csv"


def linkage_seed(seeds: Path, role: str):
    """A portfolio's representative: its row, its run's config and its linkage."""

    row = with_role(read_rows(seeds), role)
    if row is None:
        raise ValueError(f"no {role} in {seeds}")
    run = run_of(seeds, row)
    linkage = FingerLinkage(decode(run.mechanism.read_design(row), run.config.link_length_resolution_mm))
    return row, run.config, linkage


def matched_revolute(finger_mm: float, proximal_mm: float, distal_mm: float):
    """The 2R of at most ``finger_mm`` with these pads that flexes the most:
    its mechanism, config and ``(x, evaluation)``."""

    found = mechanism("revolute")
    config, _ = load_yaml(found, REPOSITORY_ROOT / "scripts/optimize/linkage/config.yaml", [
        f"goal.pads.proximal.length_mm=[{proximal_mm}, {proximal_mm}]",
        f"goal.pads.distal.length_mm=[{distal_mm}, {distal_mm}]",
        "goal.motion.dip_flexion_deg=10", f"goal.link.finger_length_mm=[40, {finger_mm}]"])
    start = config.goal.packaging.proximal_pad_start_mm
    low, high = config.bounds["distal_drop_mm"]
    tilt = config.bounds["servo_tilt_deg"][0]
    best = None

    def scan(lengths, oas, drops) -> None:
        nonlocal best
        for length in lengths:
            for a in oas:
                ac = length - a
                if ac < 25.0:
                    continue
                for drop in drops:
                    x = tuple(round(float(v), 1) for v in (a, ac, start + proximal_mm, ac - distal_mm, drop,
                                                            config.joint_radius_mm, tilt))
                    e = found.evaluate(x, config)
                    if e.feasible and (best is None or (e.phi_max_rad, -e.finger_length_mm)
                                       > (best[1].phi_max_rad, -best[1].finger_length_mm)):
                        best = (x, e)

    # Coarse over |OC|, |OA| and the distal pad's drop, then fine about the best.
    scan(np.arange(finger_mm - 10.0, finger_mm + 1e-9, 2.0), np.arange(20.0, finger_mm, 2.0),
         np.arange(low, high + 1e-9, 2.0))
    if best is not None:
        a, ac, _, _, drop = best[0][:5]
        scan(np.arange(max(a + ac - 2.0, finger_mm - 10.0), min(a + ac + 2.0, finger_mm) + 1e-9, 0.5),
             np.arange(a - 2.0, a + 2.0 + 1e-9, 0.5), np.arange(max(drop - 2.0, low), min(drop + 2.0, high) + 1e-9, 0.5))
    return found, config, best
