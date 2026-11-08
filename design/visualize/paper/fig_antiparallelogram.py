"""The symmetric crossed four-bar (antiparallelogram: |A'A| = |B'B| = g,
|A'B| = |AB'| = r): k = dphi/dtheta against the servo angle, per g / r.

    python -m visualize.paper.fig_antiparallelogram

Only the ratio g / r matters (the kinematics is scale-free). The servo
turns the bar A'B by theta from the symmetric pose (the coupler B'B parallel
to the ground A'A, the bars crossed), both ways; phi is the coupler's turn
relative to the ground, positive as theta. Each curve spans the servo angles
where the transmission angle at B' (between B'->A and B'->B) stays within
[30, 150] deg, the most a finger on this four-bar can sweep; drawn bold, the
150 deg window of phi within it whose k_max is least. Saves
output/paper/fig_antiparallelogram.png / .pdf.
"""

from __future__ import annotations

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from visualize.style import INK, MUTED, axes_style, save, use  # noqa: E402

RATIOS = (0.2, 0.3, 0.4, 0.5, 0.6)
TRANSMISSION_DEG = (30.0, 150.0)
STEP_DEG = 0.05
WINDOW_DEG = 150.0
"""The required flexion, placed where it costs the least k_max."""
SHADES = ("#b9d3f2", "#86b2e8", "#4f8fdc", "#2a6fc4", "#164c8f")
"""One hue, light to dark with g / r (a magnitude)."""


def _half(ratio: float, sense: float) -> tuple[list[float], list[float]]:
    """Servo angle and phi (deg) from the symmetric pose one way, while the
    transmission angle stays in range."""

    g, r = ratio, 1.0
    a_prime, a = np.array((-g / 2, 0.0)), np.array((g / 2, 0.0))
    h = math.sqrt(r * r - g * g)
    b, b_prime = np.array((g / 2, h)), np.array((-g / 2, h))
    theta0 = math.atan2(b[1] - a_prime[1], b[0] - a_prime[0])
    coupler0 = math.atan2(b[1] - b_prime[1], b[0] - b_prime[0])
    thetas, phis, coupler = [0.0], [0.0], coupler0
    for step in range(1, int(360.0 / STEP_DEG)):
        theta = sense * math.radians(step * STEP_DEG)
        b = a_prime + r * np.array((math.cos(theta0 + theta), math.sin(theta0 + theta)))
        d = np.linalg.norm(b - a)
        if not abs(r - g) < d < r + g:
            break
        # B' on |AB'| = r and |BB'| = g: the branch nearest the last pose.
        along = (r * r - g * g + d * d) / (2 * d)
        unit = (b - a) / d
        off = math.sqrt(max(r * r - along * along, 0.0)) * np.array((-unit[1], unit[0]))
        b_prime = min((a + along * unit + off, a + along * unit - off), key=lambda p: np.linalg.norm(p - b_prime))
        u, v = a - b_prime, b - b_prime
        mu = math.degrees(math.acos(np.clip(u @ v / (np.linalg.norm(u) * np.linalg.norm(v)), -1.0, 1.0)))
        if not TRANSMISSION_DEG[0] <= mu <= TRANSMISSION_DEG[1]:
            break
        angle = math.atan2(b[1] - b_prime[1], b[0] - b_prime[0])
        coupler += (angle - coupler + math.pi) % (2 * math.pi) - math.pi  # unwrapped
        thetas.append(math.degrees(theta))
        phis.append(math.degrees(coupler - coupler0))
    return thetas, phis


def sweep(ratio: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Servo angle, phi (both signed as theta, deg) and k over the usable range."""

    forward, back = _half(ratio, 1.0), _half(ratio, -1.0)
    theta = np.array(back[0][:0:-1] + forward[0])
    phi = np.array(back[1][:0:-1] + forward[1])
    if phi[-1] < phi[0]:  # phi as theta
        phi = -phi
    return theta, phi, np.gradient(phi, theta)


def best_window(phi: np.ndarray, k: np.ndarray) -> tuple[int, int] | None:
    """Indices of the WINDOW_DEG span of phi with the least k_max (``None``: none fits)."""

    best, found = math.inf, None
    end = 0
    for start in range(len(phi)):
        while end < len(phi) and phi[end] - phi[start] < WINDOW_DEG:
            end += 1
        if end == len(phi):
            break
        worst = float(k[start:end + 1].max())
        if worst < best:
            best, found = worst, (start, end)
    return found


def main() -> int:
    use("paper")
    figure, (k_axis, phi_axis) = plt.subplots(1, 2, figsize=(9.0, 3.4), constrained_layout=True)
    for axis in (k_axis, phi_axis):
        axes_style(axis)
        axis.set_xlabel("servo angle θ from the symmetric pose [deg]", color=INK)
    for ratio, shade in zip(RATIOS, SHADES):
        theta, phi, k = sweep(ratio)
        label = f"g / r = {ratio:.1f}"
        window = best_window(phi, k)
        for axis, values in ((k_axis, k), (phi_axis, phi)):
            axis.plot(theta, values, color=shade, linewidth=1.0, alpha=0.7)
            if window:
                i, j = window
                axis.plot(theta[i:j + 1], values[i:j + 1], color=shade, linewidth=2.6, label=label)
        span = phi[-1] - phi[0]
        line = (f"g/r {ratio:.1f}: theta {theta[0]:6.1f} .. {theta[-1]:5.1f} deg, phi span {span:5.1f} deg, "
                f"k {k.min():.2f} .. {k.max():.2f}")
        if window:
            i, j = window
            line += (f"; best 150 deg: theta {theta[i]:.1f} .. {theta[j]:.1f} (servo {theta[j] - theta[i]:.1f} deg), "
                     f"k {k[i:j + 1].min():.2f} .. {k[i:j + 1].max():.2f}")
        print(line)
    k_axis.set_ylabel("k = dφ / dθ", color=INK)
    phi_axis.set_ylabel("distal turn φ from the symmetric pose [deg]", color=INK)
    k_axis.set_ylim(bottom=0)
    phi_axis.legend(frameon=False, fontsize=8, loc="lower right")
    figure.suptitle("Symmetric crossed four-bar, |A'A| = |B'B| = g, |A'B| = |AB'| = r; transmission at B' in "
                    "[30, 150]°; bold: the 150° of φ with the least k_max",
                    color=MUTED, fontsize=8.5, x=0.01, ha="left")
    print(f"saved {save(figure, 'fig_antiparallelogram')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
