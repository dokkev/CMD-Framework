"""k against the servo angle: the optimized balanced linkages against the
symmetric crossed four-bar (``fig_antiparallelogram``) of the same g / r.

    python -m visualize.paper.fig_k_balanced

The linkages are the ``balanced`` seeds of the PORTFOLIOS (the free
four-bar's and the optimized symmetric one's), swept by their servo from
extension to their safe reach. The ideal symmetric four-bar has the first
one's g = |A'A| over the mean of its bars, its best 150 deg window shifted to start at 0 (its
extension). Left: k against the servo angle; right: k against the distal
flexion phi. Saves output/paper/fig_k_balanced.png / .pdf.
"""

from __future__ import annotations

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from core.morphology.linkage import FingerLinkage  # noqa: E402
from core.optimization.linkage.design import VARIABLE_NAMES, decode  # noqa: E402
from core.optimization.study import load_run  # noqa: E402
from visualize.fronts import read_rows, with_role  # noqa: E402
from visualize.motion import servo_sweep  # noqa: E402
from visualize.paper import fig_antiparallelogram  # noqa: E402
from visualize.style import COLORS, INK, MUTED, OUTPUT, axes_style, save, use  # noqa: E402

LINKAGE = OUTPUT / "linkage"
PORTFOLIOS = (("config/portfolio_003", "balanced, current rules", COLORS["linkage"], "-"),
              ("symmetric/portfolio_001", "balanced, symmetric four-bar (optimized)", COLORS["rcj"], "-"),
              ("config/portfolio_001", "balanced, 2026-10-26 (no A-servo, pads from A, B)", MUTED, (0, (4, 2))))
STEP_DEG = 0.05


def balanced(portfolio: str) -> tuple[dict[str, str], FingerLinkage]:
    row = with_role(read_rows(LINKAGE / portfolio / "seeds.csv"), "balanced")
    config = load_run(LINKAGE / row["run"]).config
    return row, FingerLinkage(decode([float(row[n]) for n in VARIABLE_NAMES], config.link_length_resolution_mm))


def linkage_k(row: dict[str, str], linkage: FingerLinkage) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Servo angle and phi (deg) from extension up to the safe reach, and k."""

    servo, sweep = servo_sweep(linkage, STEP_DEG)
    phi = np.degrees(sweep.total_flexion_rad)
    theta = np.degrees(np.abs(servo[:len(phi)]))
    keep = phi <= float(row["safe_reach_deg"]) + 1e-9
    theta, phi = theta[keep], phi[keep]
    return theta, phi, np.gradient(phi, theta)


def main() -> int:
    use("paper")
    figure, (servo_axis, phi_axis) = plt.subplots(1, 2, figsize=(9.0, 3.4), constrained_layout=True)
    for axis in (servo_axis, phi_axis):
        axes_style(axis)
        axis.set_ylabel("k = dφ / dθ", color=INK)
    servo_axis.set_xlabel("servo angle θ from extension [deg]", color=INK)
    phi_axis.set_xlabel("distal flexion φ [deg]", color=INK)
    ratio = None
    for portfolio, label, color, dash in PORTFOLIOS:
        row, linkage = balanced(portfolio)
        theta, phi, k = linkage_k(row, linkage)
        servo_axis.plot(theta, k, color=color, linestyle=dash, linewidth=2.0, label=label)
        phi_axis.plot(phi, k, color=color, linestyle=dash, linewidth=2.0, label=label)
        within = phi <= 150.0
        print(f"{portfolio} {label}: k {k[within].min():.2f} .. {k[within].max():.2f} on [0, 150] deg, "
              f"servo {theta[within][-1]:.1f} deg for 150 deg, reach {phi[-1]:.1f} deg")
        if ratio is None:
            g = float(np.linalg.norm(np.subtract((linkage.a.x_mm, linkage.a.z_mm), (linkage.ap.x_mm, linkage.ap.z_mm))))
            bars = (math.dist((linkage.ap.x_mm, linkage.ap.z_mm), (linkage.b.x_mm, linkage.b.z_mm))
                    + math.dist((linkage.a.x_mm, linkage.a.z_mm), (linkage.bp.x_mm, linkage.bp.z_mm))) / 2.0
            ratio = round(g / bars, 2)
    theta, phi, k = fig_antiparallelogram.sweep(ratio)
    i, j = fig_antiparallelogram.best_window(phi, k)
    theta, phi, k = theta[i:j + 1] - theta[i], phi[i:j + 1] - phi[i], k[i:j + 1]
    label = f"symmetric, ideal (links only), g / r = {ratio:.2f}"
    servo_axis.plot(theta, k, color=INK, linewidth=1.6, linestyle=(0, (1, 1.5)), label=label)
    phi_axis.plot(phi, k, color=INK, linewidth=1.6, linestyle=(0, (1, 1.5)), label=label)
    print(f"symmetric g/r {ratio:.2f}: k {k.min():.2f} .. {k.max():.2f} on 150 deg, servo {theta[-1]:.1f} deg")
    phi_axis.axvline(150.0, color=MUTED, linewidth=0.8, linestyle=(0, (3, 2)))
    phi_axis.text(151, 0.15, "φ_req", color=MUTED, fontsize=8)
    for axis in (servo_axis, phi_axis):
        axis.set_ylim(0, 3.2)
    servo_axis.legend(frameon=False, fontsize=7.5, loc="lower left")
    figure.suptitle("k of the optimized balanced linkage against the symmetric crossed four-bar",
                    color=MUTED, fontsize=8.5, x=0.01, ha="left")
    print(f"saved {save(figure, 'fig_k_balanced')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
