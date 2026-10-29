"""A first look at the rolling contact joint (RCJ) on the pad study's terms,
before it is a study of its own: the RCJ with the given finger and pads that
flexes the most, sketched as ``visualize.comparison.clearance``'s.

    python -m visualize.comparison.rcj [finger_mm] [proximal_mm] [distal_mm]

The kinematics are ``core.morphology.rolling_contact_joint``'s (the servo at
the proximal roller's centre ``A`` turns the carrier ``A-B`` by ``theta``,
the distal phalanx flexes ``phi = 2 theta``, so ``k = 2``). Provisional
packaging, in one plane: the servo stands on ``A`` (as the retention
study's), the proximal pad hangs from the MCP servo toward ``A`` with its top
a roller radius below ``O-A`` (clear of its own roller), the distal pad from
``C`` toward ``B`` a searched drop below ``B-C``. Checked from 0 to 180 deg:
the distal pad and the distal roller against the proximal pad, the proximal
roller, the servo and the MCP servo (MCP at 0). The carrier and links are
in other layers, the cables are not modelled. Searched: ``|OA|`` in 40-60
mm, ``r`` from the servo's radius to 20 mm, the drop 0-25 mm. Saves
``output/comparison/rcj.png``.
"""

from __future__ import annotations

import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import core.morphology.linkage.packaging as packaging_module  # noqa: E402
from core.morphology.linkage.packaging import (  # noqa: E402
    ServoFootprint,
    _polygon_to_disk,
    footprint_gaps,
    mcp_servo_gaps,
    polygon_gaps,
)
from core.optimization.mechanism import mechanism  # noqa: E402
from core.optimization.study import load_yaml  # noqa: E402
from visualize import gaps, sketch  # noqa: E402
from visualize.motion import collisions  # noqa: E402
from visualize.style import COLORS, INK, REPOSITORY_ROOT, save_comparison  # noqa: E402

STEP_DEG = 0.5
XLIM, YLIM = (-16.0, 105.0), (-60.0, 32.0)


class Rcj:
    """One RCJ design's solids over the distal flexion ``phi``."""

    def __init__(self, a_x: float, r: float, bc: float, proximal: float, distal: float, drop: float, packaging):
        self.a_x, self.r, self.bc, self.drop, self.packaging = a_x, r, bc, drop, packaging
        height = packaging.proximal_pad.height_mm
        start = packaging.proximal_pad_start_mm
        top = -r
        self.proximal_pad = np.array(((start, top - height), (start + proximal, top - height),
                                      (start + proximal, top), (start, top)))
        u0 = max(bc - distal, 0.0)
        self.distal_local = np.array(((u0, -drop - height), (bc, -drop - height), (bc, -drop), (u0, -drop)))
        servo = packaging.pip_servo
        self.servo = ServoFootprint((a_x, 0.0), (0.0, 1.0), servo.radius_mm, servo.rear_extent_mm)

    @property
    def finger_mm(self) -> float:
        return self.a_x + 2.0 * self.r + self.bc

    def b(self, phi: np.ndarray) -> np.ndarray:
        theta = np.asarray(phi) / 2.0
        return np.stack((self.a_x + 2.0 * self.r * np.cos(-theta), 2.0 * self.r * np.sin(-theta)), axis=1)

    def posed(self, local: np.ndarray, phi: np.ndarray) -> np.ndarray:
        phi = np.atleast_1d(phi)
        e = np.stack((np.cos(phi), -np.sin(phi)), axis=1)
        n = np.stack((-e[:, 1], e[:, 0]), axis=1)
        return self.b(phi)[:, None] + local[None, :, :1] * e[:, None] + local[None, :, 1:] * n[:, None]

    def distal_pad(self, phi: np.ndarray) -> np.ndarray:
        return self.posed(self.distal_local, phi)

    def tip(self, phi: np.ndarray) -> np.ndarray:
        return self.posed(np.array(((self.bc, 0.0),)), phi)[:, 0]

    def margins(self, phi: np.ndarray) -> dict[tuple[str, str], np.ndarray]:
        p = self.packaging
        pad = self.distal_pad(phi)
        count = len(pad)
        proximal = np.broadcast_to(self.proximal_pad, (count, 4, 2))
        a = np.broadcast_to((self.a_x, 0.0), (count, 2))
        b = self.b(phi)
        return {
            ("distal_pad", "proximal_pad"): polygon_gaps(pad, proximal, p),
            ("distal_pad", "proximal_roller"): _polygon_to_disk(pad, a, self.r),
            ("distal_pad", "pip_servo"): footprint_gaps(pad, self.servo, p),
            ("distal_pad", "mcp_servo"): mcp_servo_gaps(pad, p, (0.0,)),
            ("distal_roller", "proximal_pad"): _polygon_to_disk(proximal, b, self.r),
        }

    def reach_deg(self, step_deg: float = STEP_DEG) -> float:
        phi = np.radians(np.arange(0.0, 180.0 + 1e-9, step_deg))
        least = np.min(np.stack(list(self.margins(phi).values())), axis=0)
        least = np.where(self.tip(phi)[:, 0] < 0.0, -1.0, least)
        bad = np.flatnonzero(least < -1e-9)
        return float(np.degrees(phi[bad[0] - 1])) if bad.size else 180.0 if bad.size == 0 else 0.0


def search(finger: float, proximal: float, distal: float, packaging) -> Rcj | None:
    best, best_reach = None, -1.0
    for a_x in np.arange(40.0, 60.0 + 1e-9, 2.0):
        for r in np.arange(packaging.pip_servo.radius_mm, 20.0 + 1e-9, 1.0):
            bc = finger - a_x - 2.0 * r
            if bc < max(distal, 25.0):
                continue
            for drop in np.arange(0.0, 25.0 + 1e-9, 2.5):
                design = Rcj(a_x, r, bc, proximal, distal, drop, packaging)
                at_rest = np.min(np.stack(list(design.margins(np.zeros(1)).values())))
                if design.proximal_pad[:, 0].max() > a_x + 1e-9 or at_rest < -1e-9:
                    continue  # the pad stops at A; nothing overlaps extended
                reach = design.reach_deg(2.0)
                if reach > best_reach + 1e-9 or (abs(reach - best_reach) < 1e-9 and r < best.r):
                    best, best_reach = design, reach
    return best


def draw(axis, design: Rcj, phi_deg: float, hit: set[str]) -> None:
    sketch.palm(axis, design.packaging)
    sketch.servo(axis, design.servo.outline)
    phi = np.radians([phi_deg])
    b = design.b(phi)[0]
    for center, name in (((design.a_x, 0.0), "proximal_roller"), (b, "distal_roller")):
        sketch.roller(axis, center, design.r, name in hit)
    sketch.pad(axis, design.proximal_pad, "proximal_pad" in hit)
    sketch.pad(axis, design.distal_pad(phi)[0], "distal_pad" in hit)
    a = np.array((design.a_x, 0.0))
    near, far = design.posed(np.array(((design.distal_local[:, 0].min(), -design.drop),
                                       (design.bc, -design.drop))), phi)[0]
    sketch.links(axis, ((0.0, 0.0), a), (a, b), (b, near, far, b))
    sketch.nodes(axis, b, near, far)
    sketch.active(axis, (0.0, 0.0), a)


def main(argv: list[str]) -> int:
    packaging_module.EXACT_BAND_MM = 1.0e6  # every gap exact
    finger = float(argv[0]) if argv else 100.0
    proximal = float(argv[1]) if len(argv) > 1 else 36.0
    distal = float(argv[2]) if len(argv) > 2 else 28.0
    config, _ = load_yaml(mechanism("revolute"), REPOSITORY_ROOT / "scripts/optimize/revolute/config.yaml", [])
    packaging = config.goal.packaging
    design = search(finger, proximal, distal, packaging)
    if design is None:
        print("no RCJ fits")
        return 1
    reach = design.reach_deg()
    phi = np.arange(0.0, 180.0 + 1e-9, gaps.STEP_DEG)
    margins = design.margins(np.radians(phi))
    poses = [0.0, 90.0, 135.0, 180.0]

    figure = plt.figure(figsize=(4.0 * (len(poses) + 1), 8.2), facecolor="white", constrained_layout=True)
    grid = figure.add_gridspec(2, len(poses) + 1)
    for c, pose in enumerate(poses):
        axis = figure.add_subplot(grid[0, c])
        k = int(np.argmin(np.abs(phi - pose)))
        draw(axis, design, pose, collisions(margins, k))
        sketch.frame(axis, XLIM, YLIM)
        axis.set_title(f"{pose:.0f}°", color=INK, fontsize=10, loc="left")
    axis = figure.add_subplot(grid[0, len(poses)])
    draw(axis, design, 0.0, set())
    trail = np.arange(0.0, reach + 1e-9, 10.0)
    sketch.pad_trail(axis, [design.distal_pad(np.radians([pose]))[0] for pose in trail],
                     0.25 + 0.75 * trail / 180.0, COLORS["rcj"], alpha=0.85, linewidth=0.5, zorder=2.5)
    sketch.frame(axis, XLIM, YLIM)
    axis.set_title(f"distal pad every 10°, 0–{reach:.0f}°", color=INK, fontsize=10, loc="left")
    gaps.clearance_axes(figure.add_subplot(grid[1, :]), phi, margins, reach, {},
                          f"gap of each checked pair; safe to {reach:.0f}°")
    figure.suptitle(
        f"Rolling contact joint (provisional packaging, k = 2): |OC| {design.finger_mm:.0f} mm "
        f"(|OA| {design.a_x:.0f}, rollers r {design.r:.1f}, BC {design.bc:.1f}), pads {proximal:.0f} + {distal:.0f} mm, "
        f"distal pad {design.drop:.1f} mm below B-C; finger {2 * design.r + 5:.0f}+ mm deep at the joint",
        color=INK, fontsize=11, x=0.01, ha="left")
    output = save_comparison(figure, "rcj")
    print(f"RCJ |OA| {design.a_x}, r {design.r}, BC {design.bc}, drop {design.drop}: safe to {reach:.1f} deg")
    print(f"saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
