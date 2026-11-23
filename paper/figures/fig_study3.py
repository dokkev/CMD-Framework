"""Fig. 5 of the RSS 2027 extended abstract, Study 3 in one double-column figure (PNG only, print size).

    cd sim/drake && uv run python ../../paper/figures/fig_study3.py [dest_dir]

All panels: the error a redesign adds to a measured interface effect, E (median over placements, log scale), against
its change of the contact Jacobian, Delta_contact, one marker per redesigned hand, under the reused impedance setpoint
(condition A, open loop).
(a) Isaac Lab 3 (Newton, MuJoCo-Warp), point contact: pad friction at O's contacts (filled, colour by thickness),
    pad stiffness (squares), friction on the hand-independent grid (grey rings); dotted: the CFL redesigns of Study 2;
    dashed: E = 20%. Inputs: sim/isaaclab/logs/experimental/two_pad/dose_A_all.json, comp.json,
    dose_A-grid_expanded.json (as make_figures.fig_study3 before 2026-11-23).
(b) Contact model, pad friction, holding: Isaac Lab point (as (a), filled), the Drake replica with point contact, and
    Drake with hydroelastic pads (E = 0.1 MPa over 3 mm on every hand). Inputs: sim/drake/logs/e21_point.json,
    e21_hydro.json (analyze.py --out).
(c) The three tasks on hand O (radius 15 mm, q0 = 100 deg, mu 1.3), rendered in 3D by Drake's VTK renderer at the
    simulated end states (sim/drake/render_tasks.py -> logs/task_snapshots.json; sim/drake/render3d.py ->
    logs/render_<task>.png): O's linkage, pads and servos; red arrows: the load.
(d) Task, Drake, point, pad friction: holding against a load, grasp acquisition, turning a sprung dial. Inputs:
    e21_point.json, e23_point.json, e24_point.json (analyze_roll.py --out). In-hand rolling (experiment 22, which
    failed its test) is left out of the figure; the paper states it as a limitation.
Legends give Spearman rho(E, Delta_contact) and its exact or Monte Carlo one-sided permutation p (sim/drake/analyze.py).
Experiments 21-24: docs/experimental/separability.md.
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
TWO_PAD = ROOT / "sim" / "isaaclab" / "logs" / "experimental" / "two_pad"
DRAKE = ROOT / "sim" / "drake" / "logs"
sys.path.insert(0, str(ROOT / "sim" / "drake"))
from analyze import spearman_perm  # noqa: E402

DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
WIDTH, HEIGHT = 7.16, 1.52
THICK = {5: "#93bde9", 7: "#2a78d6", 9: "#0d3c78"}  # as the other figures: pad thickness, light to dark
GREY, INK2 = "#9a9a9a", "#555555"
YLIM = (2, 600)


def label(name, x, y):
    rho, p = spearman_perm(x, y)
    return f"{name}: $\\rho$ {rho:.2f}, " + ("$p$<0.001" if p < 0.001 else f"$p$ {p:.3f}")


def series(ax, rows, name, colour, marker, filled, ms=3.2):
    rows = [r for r in rows if r.get("E") is not None]
    x = np.array([r["d_contact"] for r in rows])
    y = 100 * np.array([r["E"] for r in rows])
    ax.plot(x, y, ls="none", marker=marker, ms=ms, mec=colour, mfc=colour if filled else "white", mew=0.8,
            label=label(name, x, y), zorder=3)


def style(ax, title):
    ax.set_yscale("log")
    ax.set_ylim(*YLIM)
    ax.set_xlim(0, 5.6)
    ax.set_xlabel(r"$\Delta_{\mathrm{contact}}$ (mm)", fontsize=7, labelpad=1)
    ax.tick_params(labelsize=6, length=2)
    ax.set_yticks([3, 10, 30, 100, 300])
    ax.set_yticklabels(["3", "10", "30", "100", "300"])
    ax.minorticks_off()
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.text(0.03, 0.97, title, transform=ax.transAxes, fontsize=7, fontweight="bold", va="top", ha="left")
    ax.legend(fontsize=5.6, frameon=False, loc="lower left", bbox_to_anchor=(-0.04, 1.0), handletextpad=0.2,
              borderaxespad=0.0, labelspacing=0.15, handlelength=1.0)


TASK = {"hold": ("hold", "#2a78d6"), "acquire": ("grasp acquisition", "#9b5de5"), "dial": ("dial turning", "#eda100")}


def rot(q, p):
    """Flexion by q about +y in (x, z dorsal): +x toward -z."""
    c, s_ = np.cos(q), np.sin(q)
    return np.array([c * p[0] + s_ * p[1], -s_ * p[0] + c * p[1]])


def renders(fig, rect):
    """(c): the three tasks on O, rendered in 3D by Drake (sim/drake/render3d.py) at their simulated states, side by
    side in rect (x0, y0, w, h, figure fractions), cropped to one common box so the three share a scale."""
    from PIL import Image
    ims = [np.asarray(Image.open(DRAKE / f"render_{k}.png").convert("RGB")) for k in TASK]
    ink = np.any(np.stack([im.min(axis=2) < 245 for im in ims]), axis=0)
    rows, cols = np.where(ink[:, :])
    left = int(0.20 * ims[0].shape[1])  # from the MCP joint on; the MCP servo is left out
    box = (max(rows.min() - 8, 0), rows.max() + 8, max(cols.min(), left), cols.max() + 8)
    x0, y0, w, h = rect
    for i, (key, im) in enumerate(zip(TASK, ims)):
        ax = fig.add_axes((x0 + i * w / 3, y0 + 0.13, w / 3, h - 0.13))
        ax.imshow(im[box[0]:box[1], box[2]:box[3]], interpolation="lanczos")
        ax.set_anchor("C")
        ax.axis("off")
        fig.text(x0 + (i + 0.5) * w / 3, y0 + 0.06, TASK[key][0], fontsize=6, color=TASK[key][1], ha="center",
                 va="center")


def main() -> int:
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix", "font.size": 7})
    fig = plt.figure(figsize=(WIDTH, HEIGHT))
    gs = fig.add_gridspec(1, 4, width_ratios=[1, 1, 1.75, 1], wspace=0.22, left=0.05, right=0.995, bottom=0.2,
                          top=0.61)
    axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), None, fig.add_subplot(gs[0, 3])]
    box = gs[0, 2].get_position(fig)
    renders(fig, (box.x0 - 0.015, 0.0, box.width + 0.03, 0.9))
    fig.text(box.x0, 0.985, "(c) tasks on $O$ (Drake)", fontsize=7, fontweight="bold", ha="left", va="top")

    # (a) Isaac Lab, three settings at the reused setpoint
    ax = axes[0]
    grid = json.loads((TWO_PAD / "dose_A-grid_expanded.json").read_text())["hands"]
    fr = json.loads((TWO_PAD / "dose_A_all.json").read_text())["hands"]
    comp = json.loads((TWO_PAD / "comp.json").read_text())["hands"]
    gx = [r["D_c_mm"] for r in grid.values()]
    gy = [100 * r["E_rel_median"] for r in grid.values()]
    ax.plot(gx, gy, ls="none", marker="o", ms=2.6, mec=GREY, mfc="none", mew=0.6, zorder=2,
            label=label("friction, grid (45)", gx, gy))
    fx = [r["D_c_mm"] for r in fr.values()]
    fy = [100 * r["E_rel_median"] for r in fr.values()]
    for r in fr.values():
        c = THICK[int(r["thickness_mm"])]
        ax.plot(r["D_c_mm"], 100 * r["E_rel_median"], ls="none", marker="o", ms=3.3, mec=c, mfc=c, mew=0.8, zorder=3)
    ax.plot([], [], ls="none", marker="o", ms=3.3, mec=THICK[7], mfc=THICK[7], label=label("friction (15)", fx, fy))
    sx = [r["D_c_mm"] for r in comp.values()]
    sy = [100 * r["E_median"] for r in comp.values()]
    for hand, r in comp.items():
        c = THICK[7 if not hand.startswith("h3to") else int(hand[4])]
        ax.plot(r["D_c_mm"], 100 * r["E_median"], ls="none", marker="s", ms=3.0, mec=c, mfc="white", mew=0.8,
                zorder=3)
    ax.plot([], [], ls="none", marker="s", ms=3.0, mec=THICK[7], mfc="white", mew=0.8,
            label=label("stiffness (14)", sx, sy))
    ax.axhline(20, color=INK2, lw=0.6, ls=(0, (3, 2)), zorder=1)
    ax.text(5.55, 21, "20%", fontsize=5.6, ha="right", va="bottom", color=INK2)
    for x, t in ((0.35, 5), (0.86, 7), (2.32, 9)):  # the CFL redesigns of Study 2 (q_req 120 deg)
        ax.axvline(x, color=THICK[t], lw=0.7, ls=":", zorder=1)
        ax.text(x + 0.06, 2.3, f"{t} mm", fontsize=5.4, color=THICK[t], va="bottom", rotation=90)
    style(ax, "(a) reused setpoint")
    ax.set_ylabel("error $E$ (%)", fontsize=7, labelpad=1)

    # (b) contact model
    ax = axes[1]
    point = json.loads((DRAKE / "e21_point.json").read_text())
    hydro = json.loads((DRAKE / "e21_hydro.json").read_text())
    series(ax, [{"d_contact": r["d_contact"], "E": r.get("E_isaac")} for r in point], "Isaac Lab, point",
           "#52514e", "o", True)
    series(ax, point, "Drake, point", "#2a78d6", "^", False)
    series(ax, hydro, "Drake, hydroelastic", "#eb6834", "D", True, ms=3.0)
    style(ax, "(b) contact model")

    # (d) task
    ax = axes[3]
    series(ax, point, "hold", "#2a78d6", "^", False)
    series(ax, json.loads((DRAKE / "e23_point.json").read_text()), "grasp acquisition", "#9b5de5", "v", True)
    series(ax, json.loads((DRAKE / "e24_point.json").read_text()), "dial turning", "#eda100", "P", True)
    style(ax, "(d) task (Drake, point)")

    DEST.mkdir(parents=True, exist_ok=True)
    out = DEST / "fig_study3.png"
    fig.savefig(out, dpi=400)
    from PIL import Image
    Image.open(out).convert("RGB").save(out)  # opaque RGB, no SMask (PaperPlaza)
    print(out, f"{WIDTH} x {HEIGHT} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
