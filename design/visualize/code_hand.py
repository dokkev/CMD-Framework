"""Draw the CoDe parametric model (core/morphology/code_hand): servos, joint axes and linkages, no CAD.

    python -m visualize.code_hand                         # the CAD pose stored in prototype.yaml
    python -m visualize.code_hand --pose zero             # every joint at 0
    python -m visualize.code_hand --pose zero --set index.pip_deg=60 thumb.mcp_deg=30
    python -m visualize.code_hand --params my_hand.yaml --out output/code_hand/my_hand.png

Each servo is the XM335 body (``core.hardware.xm335``: a D profile, rounded at the shaft, extruded along it)
at the model's servo frame; a dark dot marks the output axis on the horn side. Joint axes are dashed lines; a black cross marks each distal point, and the thumb
end effector (``Thumb EE``) carries a small RGB triad. Each finger shows its four-bar in the servo layer:
the proximal segment O-A-A', the driver A'-B, the coupler A-B' and the distal segment B'-B-C. The thumb
shows its chain through the servo origins to its distal point. Four views: isometric, top (dorsal), front (distal) and side.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from core.hardware.xm335 import XM335Actuator
from core.morphology.code_hand import (
    FINGERS,
    CoDeHand,
    HandConfiguration,
    load,
)
from core.morphology.code_hand.parameters import PROTOTYPE

ROOT = Path(__file__).resolve().parents[1]
BODY = XM335Actuator().geometry  # D-shaped body in the servo (centre) frame: shaft at the origin, cap +Y
COLORS = {"index": "#3b6fb6", "middle": "#3a9a6a", "ring": "#c07a2c", "thumb": "#9b4d9b"}
VIEWS = (("isometric", 25, -135), ("top (dorsal)", 90, -90), ("front (from distal)", 0, 0), ("side (from +Y)", 0, 90))


def _profile(n_arc: int = 16) -> np.ndarray:
    """The D outline (x, y): the rear rectangle and the rounded cap about the shaft, counter-clockwise."""
    r, rear = BODY.radius_mm, BODY.rear_extent_mm
    arc = [(r * np.cos(t), r * np.sin(t)) for t in np.linspace(0.0, np.pi, n_arc)]
    return np.array([(-r, -rear), (r, -rear), *arc])


PROFILE = _profile()


def body_faces(frame: np.ndarray) -> list[np.ndarray]:
    """The servo body's faces in H: the D profile extruded +-depth/2 along the shaft axis."""
    half = BODY.depth_mm / 2.0
    n = len(PROFILE)
    local = np.vstack([np.c_[PROFILE, np.full(n, z), np.ones(n)] for z in (-half, half)])
    world = (frame @ local.T).T[:, :3]
    bottom, top = world[:n], world[n:]
    sides = [np.array([bottom[i], bottom[(i + 1) % n], top[(i + 1) % n], top[i]]) for i in range(n)]
    return [bottom, top, *sides]


def configuration(name: str, stored: HandConfiguration, overrides: list[str]) -> HandConfiguration:
    q = stored if name == "cad" else HandConfiguration()
    for item in overrides:
        key, value = item.split("=")
        digit, joint = key.split(".")
        q = replace(q, **{digit: replace(getattr(q, digit), **{joint: float(value)})})
    return q


def draw(ax, frames) -> None:
    for key, frame in frames.servos.items():
        color = COLORS[key.split("/")[0]]
        faces = body_faces(frame)
        ax.add_collection3d(Poly3DCollection(faces[:2], facecolor=color, edgecolor="#222222", linewidth=0.4,
                                             alpha=0.28))
        ax.add_collection3d(Poly3DCollection(faces[2:], facecolor=color, edgecolor="none", alpha=0.18))
        horn = frame @ np.array([0.0, 0.0, BODY.depth_mm / 2.0, 1.0])
        ax.scatter(*horn[:3], color="#222222", s=6)
    for key, (point, direction) in frames.axes.items():
        if key.split("/")[1] in ("aa", "fe", "ro", "raa", "mcp", "ip"):
            ends = np.array([point - 14.0 * direction, point + 14.0 * direction])
            ax.plot(*ends.T, color=COLORS[key.split("/")[0]], linestyle="--", linewidth=0.9)
    for digit in FINGERS:
        o = frames.servos[f"{digit}/fe"][:3, 3]
        a, ap, b, bp = (frames.axes[f"{digit}/{n}"][0] for n in ("A", "Ap", "B", "Bp"))
        c = frames.points[f"{digit}/C"]
        color = COLORS[digit]
        for chain, width in (((o, a, ap, o), 2.2), ((ap, b), 1.4), ((a, bp), 1.4), ((bp, b, c, bp), 2.2)):
            ax.plot(*np.array(chain).T, color=color, linewidth=width)
        ax.scatter(*np.array([o, a, ap, b, bp]).T, color=color, s=10)
        ax.scatter(*c, color="black", s=14, marker="x")
    thumb = np.array([frames.servos[f"thumb/{j}"][:3, 3] for j in ("ro", "raa", "mcp", "ip")]
                     + [frames.points["thumb/tip"]])
    ax.plot(*thumb.T, color=COLORS["thumb"], linewidth=2.2)
    ax.scatter(*thumb[:-1].T, color=COLORS["thumb"], s=10)
    ax.scatter(*thumb[-1], color="black", s=14, marker="x")
    for ee in frames.ends.values():  # end-effector frames
        for axis, color in zip(ee[:3, :3].T, ("red", "green", "blue")):
            ax.plot(*np.array([ee[:3, 3], ee[:3, 3] + 8.0 * axis]).T, color=color, linewidth=1.2)
    for axis, color in zip(np.eye(3), ("red", "green", "blue")):  # hand frame
        ax.plot(*np.array([[0, 0, 0], 15.0 * axis]).T, color=color, linewidth=1.5)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--params", type=Path, default=PROTOTYPE)
    parser.add_argument("--pose", choices=("cad", "zero"), default="cad")
    parser.add_argument("--set", nargs="*", default=[], metavar="DIGIT.JOINT=DEG",
                        help="override joints, e.g. index.pip_deg=60 thumb.ro_deg=-30")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)

    parameters, stored = load(args.params)
    q = configuration(args.pose, stored, args.set)
    frames = CoDeHand(parameters).frames(q)

    points = np.vstack([np.vstack(body_faces(f)) for f in frames.servos.values()]
                       + [np.array(list(frames.points.values()))])
    lo, hi = points.min(axis=0) - 5.0, points.max(axis=0) + 5.0
    fig = plt.figure(figsize=(14, 11))
    for i, (title, elev, azim) in enumerate(VIEWS):
        ax = fig.add_subplot(2, 2, i + 1, projection="3d")
        draw(ax, frames)
        ax.set_xlim(lo[0], hi[0]), ax.set_ylim(lo[1], hi[1]), ax.set_zlim(lo[2], hi[2])
        ax.set_box_aspect(hi - lo)
        ax.view_init(elev=elev, azim=azim)
        ax.set_proj_type("ortho")
        hidden = {"top (dorsal)": "z", "front (from distal)": "x", "side (from +Y)": "y"}.get(title)
        if hidden:  # the axis along the view direction carries no information
            getattr(ax, f"set_{hidden}ticks")([])
            getattr(ax, f"set_{hidden}label")("")
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("X distal [mm]", fontsize=7), ax.set_ylabel("Y thumb [mm]", fontsize=7)
        ax.set_zlabel("Z dorsal [mm]", fontsize=7)
        ax.tick_params(labelsize=6)
    handles = [plt.Line2D([], [], color=c, linewidth=3, label=d) for d, c in COLORS.items()]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False)
    pose = ", ".join(f"{d}: " + " ".join(f"{k.removesuffix('_deg')}={v:g}" for k, v in j.items())
                     for d, j in q.to_dict().items())
    fig.suptitle(f"CoDe parametric model ({args.params.name}, pose: {args.pose})\n{pose} [deg]", fontsize=9, y=0.995)
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    out = args.out or ROOT / "output" / "code_hand" / f"code_hand_{args.pose}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
