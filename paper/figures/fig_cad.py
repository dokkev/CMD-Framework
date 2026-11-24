"""Fig. 1 of the RSS 2027 extended abstract (PNG only, single column, print size). Every render is of the visual
USD used in Isaac Lab, drawn by Newton's GL viewer (sim/isaaclab/scripts/code_hand/render_visuals.py; never the Drake
renders): the hand palm up (--view palm_up -> cad/hand_usd_palm_up.png) and the fingers of O, R and U from the side
(logs/urdf/cfl_q120_h3to7_{h0,hr,hu}_thumbpad_vis_ball_colliders.usda, --view side --pip 0 60 120 ->
cad/usd_finger_<H>_q<q>.png). The dark viewer background is blended to white here.

    conda run -n cmd python paper/figures/fig_cad.py [dest_dir]

Contact-surface paths: design/output/paper/fig1_paths_ORU.json (paper/figures/fig1_paths.py); the reported interface
effects: report/study3/data/isaac_conclusion_study3.json.
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CAD = ROOT / "paper" / "figures" / "cad"
DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
INK, INK2, PAD = "#0b0b0b", "#52514e", "#2a5db0"


def usd_white(path, paper=244.0):
    """The Newton render with its uniform dark background blended to a light grey (white parts stay visible) and the
    mask of the hand (not cropped)."""
    from PIL import Image
    im = np.asarray(Image.open(path).convert("RGB")).astype(float)
    bg = np.median(np.concatenate([im[:20, :20].reshape(-1, 3), im[-20:, -20:].reshape(-1, 3)]), axis=0)
    alpha = np.clip((np.linalg.norm(im - bg, axis=2) - 1.5) / 6.0, 0.0, 1.0)[..., None]
    return (alpha * im + (1.0 - alpha) * paper).astype(np.uint8), alpha[..., 0]


def usd_render(path):
    """The Newton render with its uniform dark background blended to white and cropped to the hand."""
    from PIL import Image
    im = np.asarray(Image.open(path).convert("RGB")).astype(float)
    bg = np.median(np.concatenate([im[:20, :20].reshape(-1, 3), im[-20:, -20:].reshape(-1, 3)]), axis=0)
    alpha = np.clip((np.linalg.norm(im - bg, axis=2) - 8.0) / 40.0, 0.0, 1.0)[..., None]
    out = alpha * im + (1.0 - alpha) * 255.0
    rows, cols = np.where(alpha[..., 0] > 0.05)
    out = out[max(rows.min() - 8, 0):rows.max() + 8, max(cols.min() - 8, 0):cols.max() + 8]
    return out.astype(np.uint8)


def icon(ax, kind):
    """A cross-section sketch of one contact interface on its module base (illustrative, not to scale)."""
    from matplotlib.patches import Rectangle, Polygon, FancyBboxPatch
    ax.set_xlim(0, 10); ax.set_ylim(-5.2, 1.6); ax.set_aspect("equal"); ax.axis("off")
    ax.add_patch(Rectangle((0.5, 0.0), 9.0, 1.0, fc="#c9cdd2", ec=INK2, lw=0.5))  # module base on the phalanx
    if kind == "thin":
        ax.add_patch(Rectangle((0.5, -0.8), 9.0, 0.8, fc="#5b7fc7", ec=PAD, lw=0.5))
    elif kind == "thick":
        ax.add_patch(FancyBboxPatch((0.7, -3.6), 8.6, 3.5, boxstyle="round,pad=0.1,rounding_size=1.2",
                                    fc="#b9c9ee", ec=PAD, lw=0.5))
    elif kind == "textured":
        ax.add_patch(Rectangle((0.5, -1.6), 9.0, 1.6, fc="#8fa7dc", ec=PAD, lw=0.5))
        for x in np.arange(0.9, 9.4, 0.9):
            ax.add_patch(Polygon([(x - 0.4, -1.6), (x, -2.3), (x + 0.4, -1.6)], fc="#8fa7dc", ec=PAD, lw=0.4))
    elif kind == "tactile":
        ax.add_patch(Rectangle((0.5, -3.4), 9.0, 3.4, fc="#eef0f3", ec=INK2, lw=0.5))   # housing
        ax.add_patch(Rectangle((3.6, -1.4), 2.8, 1.2, fc="#6b6f75", ec=INK2, lw=0.4))   # camera
        ax.add_patch(Polygon([(4.4, -1.4), (5.6, -1.4), (6.4, -3.0), (3.6, -3.0)], fc="#fff6cc", ec="none",
                             alpha=0.9))                                                    # field of view
        ax.add_patch(Rectangle((0.5, -4.6), 9.0, 1.2, fc="#d6c9ee", ec="#7a5fb0", lw=0.5))  # gel
    import numpy as _np  # noqa: F401


HAND_C = {"O": "#3a3a3a", "R": "#1f6fd1", "U": "#d9622b"}


def crop_white(im, pad=6):
    rows, cols = np.where(im.min(axis=2) < 245)
    return im[max(rows.min() - pad, 0):rows.max() + pad, max(cols.min() - pad, 0):cols.max() + pad]


def main() -> int:
    """Fig. 1 (money figure, 2026-11-24): keeping the contact-surface motion lets an interface be evaluated without a
    bias from the reused controller. (a) the hand (Isaac Lab USD); (b) O and two 7 mm redesigns at the same joint
    command; (c) how far each redesign's distal contact surface strays from O's over flexion; (d) the effect of a
    softer 7 mm pad on pull-out load as each hand reports it under the same impedance setpoint."""
    import json
    from PIL import Image
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix", "font.size": 7})
    width, height = 3.45, 2.40
    fig = plt.figure(figsize=(width, height))
    lab = dict(fontsize=7, fontweight="bold", va="top")
    # (a) the hand
    iso = usd_render(CAD / "hand_usd_palm_up.png")
    aw = 0.88
    ah = aw * iso.shape[0] / iso.shape[1]
    a = fig.add_axes((0.02 / width, (2.27 - ah) / height, aw / width, ah / height))
    a.imshow(iso); a.axis("off")
    fig.text(0.005, 0.995, "(a)", **lab)
    fig.text(0.46 / width, (2.27 - ah - 0.03) / height, "CoDe hand\n(pads: blue)", fontsize=6, color=INK2,
             ha="center", va="top", linespacing=1.0)
    # (b) O, U, R at the same joint command
    fig.text(0.98 / width, 0.995, "(b)", **lab)
    fig.text(2.2 / width, 2.37 / height, "same joint command, $q=120^\\circ$", fontsize=6.5, ha="center", va="top")
    names = (("O", "O: 3 mm pads"), ("U", "U: 7 mm,\ncoverage only"), ("R", "R: 7 mm, keeps\ncontact motion"))
    keyed = [usd_white(CAD / f"usd_finger_{h}_q120.png") for h, _ in names]
    ims = [k[0] for k in keyed]
    ink = np.any(np.stack([k[1] > 0.5 for k in keyed]), axis=0)
    ink &= ink.sum(axis=1, keepdims=True) > 20  # ignore stray thin rows
    r, c = np.where(ink)
    c0 = c.min() + int(0.12 * (c.max() - c.min()))  # drop the palm block at the left edge
    ims = [im[max(r.min() - 4, 0):r.max() + 4, c0:c.max() + 4] for im in ims]
    fw = 0.78
    fh = fw * ims[0].shape[0] / ims[0].shape[1]
    for i, ((h, name), im) in enumerate(zip(names, ims)):
        x0 = 1.0 + i * (fw + 0.03)
        ax = fig.add_axes((x0 / width, (2.21 - fh) / height, fw / width, fh / height))
        ax.imshow(im); ax.axis("off")
        fig.text((x0 + fw / 2) / width, (2.21 - fh - 0.02) / height, name, fontsize=6, color=HAND_C[h],
                 ha="center", va="top", linespacing=1.0, fontweight="bold" if h != "O" else "normal")
    # (c) contact-surface deviation from O
    paths = json.load(open(ROOT / "design" / "output" / "paper" / "fig1_paths_ORU.json"))
    ref = np.array(paths["O"]["contact"])
    q = np.linspace(0, 120, len(ref))
    ax = fig.add_axes((0.36 / width, 0.30 / height, 1.25 / width, 0.80 / height))
    ax.axhspan(0, 1.0, color="#1f6fd1", alpha=0.08, lw=0)
    for h in "UR":
        dev = np.linalg.norm(np.array(paths[h]["contact"]) - ref, axis=1)
        ax.plot(q, dev, color=HAND_C[h], lw=1.4)
        ax.text(122, dev[-1], h, color=HAND_C[h], fontsize=6.5, fontweight="bold", va="center")
    ax.set_xlim(0, 120); ax.set_ylim(0, 4.5); ax.set_xticks([0, 60, 120]); ax.set_yticks([0, 2, 4])
    ax.set_xlabel("flexion $q$ (deg)", labelpad=1)
    ax.set_ylabel("contact surface\noff O's path (mm)", labelpad=1, linespacing=1.0)
    ax.text(64, 1.22, "kept: $<1$ mm", fontsize=5.8, color="#1f6fd1", va="bottom")
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.tick_params(length=2, pad=1, labelsize=6)
    fig.text(0.005, 1.22 / height, "(c)", **lab)
    # (d) the interface effect each hand reports under the same impedance setpoint
    d = json.load(open(ROOT / "report" / "study3" / "data" / "isaac_conclusion_study3.json"))["hands"]
    vals = (("true\n(on O)", d["R"]["true_change_pct"], "#9a9a9a"),
            ("on R", d["R"]["measured_change_pct"], HAND_C["R"]),
            ("on U", d["U"]["measured_change_pct"], HAND_C["U"]))
    bx = fig.add_axes((2.30 / width, 0.30 / height, 1.10 / width, 0.66 / height))
    for i, (name, v, col) in enumerate(vals):
        bx.bar(i, v, width=0.62, color=col, edgecolor="none")
        bx.text(i, v + (5 if v > 0 else -5), f"{v:+.0f}%".replace("-", "\u2212"), ha="center", va="bottom" if v > 0 else "top",
                fontsize=6.2, color=INK)
    bx.axhline(0, color=INK, lw=0.6)
    bx.set_xticks(range(3)); bx.set_xticklabels([v[0] for v in vals], fontsize=6, linespacing=1.0)
    bx.set_ylim(-82, 66); bx.set_yticks([-50, 0, 50]); bx.set_yticklabels(["$-50$", "0", "$+50$"])
    bx.set_ylabel("pull-out load change (%)", labelpad=1, fontsize=6)
    bx.text(2.0, -8, "sign\nreversed", ha="center", va="top", fontsize=6, color=HAND_C["U"], style="italic", linespacing=1.0)
    for sp in ("top", "right", "bottom"):
        bx.spines[sp].set_visible(False)
    bx.tick_params(length=2, pad=1, labelsize=6)
    bx.tick_params(axis="x", length=0)
    fig.text(1.80 / width, 1.22 / height, "(d)", **lab)
    fig.text(2.80 / width, 1.22 / height, "stiff 3 mm $\\to$ soft 7 mm pad,\nsame setpoint", fontsize=6.3, ha="center", va="top", linespacing=1.0)
    DEST.mkdir(parents=True, exist_ok=True)
    out = DEST / "fig_cad.png"
    fig.savefig(out, dpi=400)
    Image.open(out).convert("RGB").save(out)  # opaque RGB, no SMask (PaperPlaza)
    print(out, f"{width} x {height} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
