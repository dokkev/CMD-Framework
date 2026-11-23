"""O, R and U side by side: each finger rendered in 3D (render3d.add_finger, Drake VTK) at flexions 0, 60 and 120 deg,
and a 2D comparison of what a redesign keeps: the distal contact point c(q) in the proximal pad-face frame
(contact-surface motion) and the fingertip path r_C(q) from the link line (skeleton motion).

    uv run python render_hands.py   ->  logs/hands_ORU_3d.png, logs/hands_ORU_paths.png
"""
import math

import numpy as np

import render3d as R3
import two_pad_drake as T

HANDS = (("O", "O (3 mm)"), ("R", "R (7 mm, keeps contact motion)"), ("U", "U (7 mm, coverage only)"))
QS = (0.0, 60.0, 120.0)


def distal_pose(hand, q_deg):
    g = T.finger_geometry(hand)
    qs, th = T.servo_table(g)
    q = math.radians(q_deg)
    theta = float(np.interp(q, qs, th))
    B = g["ap"] + T.rot(theta) @ (g["b"] - g["ap"])
    return B, q


def render_3d():
    from PIL import Image
    paths = {}
    for hand, _ in HANDS:
        for qd in QS:
            sc = R3.Scene(T.OUT / "render3d" / f"hand_{hand}_{int(qd)}")
            B, q = distal_pose(hand, qd)
            R3.add_finger(sc, hand, B, q)
            out = T.OUT / f"hand_{hand}_{int(qd)}.png"
            R3.render(sc, out, target_xz=(40.0, -12.0), dist=0.24, az_deg=-20.0, el_deg=14.0, fov_deg=30.0)
            paths[(hand, qd)] = out
    ims = {k: np.asarray(Image.open(v).convert("RGB")) for k, v in paths.items()}
    ink = np.any(np.stack([im.min(axis=2) < 245 for im in ims.values()]), axis=0)
    r, c = np.where(ink)
    box = (max(r.min() - 10, 0), r.max() + 10, max(c.min() - 10, 0), c.max() + 10)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 3, figsize=(9, 7.2))
    for i, (hand, label) in enumerate(HANDS):
        for j, qd in enumerate(QS):
            ax = axes[i, j]
            ax.imshow(ims[(hand, qd)][box[0]:box[1], box[2]:box[3]])
            ax.axis("off")
            if i == 0:
                ax.set_title(f"q = {int(qd)}°", fontsize=10)
        axes[i, 0].text(-0.02, 0.5, label, transform=axes[i, 0].transAxes, rotation=90, ha="right", va="center",
                        fontsize=9)
    fig.tight_layout()
    out = T.OUT / "hands_ORU_3d.png"
    fig.savefig(out, dpi=160)
    print(out)


def render_paths():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colours = {"O": "#333333", "R": "#2a78d6", "U": "#eb6834"}
    fig, axes = plt.subplots(1, 2, figsize=(9, 4.0))
    qd = np.linspace(0.0, 120.0, 121)
    for hand, label in HANDS:
        g = T.finger_geometry(hand)
        p = g["pads"]
        t = p["mount_offset_mm"] + p["thickness_mm"]
        cx = p["distal_end_mm"]
        cs, rs = [], []
        for d in qd:
            B, q = distal_pose(hand, d)
            cs.append(B + T.rot(q) @ np.array([cx, -t]) - np.array([0.0, -t]))  # in the pad-face frame
            rs.append(B + T.rot(q) @ np.array([cx, 0.0]))  # from the link line
        cs, rs = np.array(cs), np.array(rs)
        for ax, path in ((axes[0], cs), (axes[1], rs)):
            ax.plot(path[:, 0], path[:, 1], color=colours[hand], lw=1.6, label=label)
            for k in (0, 60, 120):
                ax.plot(*path[k], "o", color=colours[hand], ms=3.5)
    axes[0].set_title("contact-surface motion c(q)\n(distal face point, proximal pad-face frame)", fontsize=9)
    axes[1].set_title("skeleton motion r_C(q)\n(fingertip, from the link line)", fontsize=9)
    for ax in axes:
        ax.set_aspect("equal")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("z (mm, dorsal)")
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8, loc="upper left")
    fig.text(0.5, 0.01, "R: Δ_contact 0.86 mm, Δ_skel 7.67 mm   U: Δ_contact 3.91 mm, Δ_skel 3.38 mm   (dots: q = 0, 60, 120°)",
             ha="center", fontsize=8)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    out = T.OUT / "hands_ORU_paths.png"
    fig.savefig(out, dpi=160)
    print(out)


if __name__ == "__main__":
    render_paths()
    render_3d()
