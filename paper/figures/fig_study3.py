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
(c) The three tasks on hand O (radius 15 mm, q0 = 100 deg, mu 1.3), drawn from Drake states (sim/drake/
    render_tasks.py -> logs/task_snapshots.json): pads, phalanx lines and the object, no linkage; red arrows: the load;
    grasp acquisition shows its start faintly.
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
WIDTH, HEIGHT = 7.16, 1.6
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
PAD_FC, PAD_EC, LINK = "#c4daf4", "#2a78d6", "#9a9a9a"


def rot(q, p):
    """Flexion by q about +y in (x, z dorsal): +x toward -z."""
    c, s_ = np.cos(q), np.sin(q)
    return np.array([c * p[0] + s_ * p[1], -s_ * p[0] + c * p[1]])


def scene(ax, snap, key, state, faint=False):
    """Pads, phalanx lines and the object at one Drake state (mm); the linkage is not drawn."""
    from matplotlib.patches import Circle, Polygon
    pads = snap["pads"]
    m, d = pads["mount_offset_mm"], pads["mount_offset_mm"] + pads["thickness_mm"]
    a = 0.3 if faint else 1.0
    prox = [(pads["proximal_start_mm"], -m), (pads["proximal_end_mm"], -m), (pads["proximal_end_mm"], -d),
            (pads["proximal_start_mm"], -d)]
    bx, bz, q = state["distal"]
    B = np.array([bx, bz])
    dist = [B + rot(q, np.array(p)) for p in ((pads["distal_start_mm"], -m), (pads["distal_end_mm"], -m),
                                               (pads["distal_end_mm"], -d), (pads["distal_start_mm"], -d))]
    if not faint:
        ax.add_patch(Polygon(prox, closed=True, fc=PAD_FC, ec=PAD_EC, lw=0.6, zorder=2))
        ax.plot([pads["proximal_start_mm"] - 4, B[0]], [0, 0], color=LINK, lw=1.4, solid_capstyle="round", zorder=1)
    ax.add_patch(Polygon(dist, closed=True, fc=PAD_FC, ec=PAD_EC, lw=0.6, alpha=a, zorder=2))
    tip = B + rot(q, np.array([pads["distal_end_mm"] + 2, 0.0]))
    ax.plot([B[0], tip[0]], [B[1], tip[1]], color=LINK, lw=1.4, alpha=a, solid_capstyle="round", zorder=1)
    c = np.array(state["centre"])
    colour = TASK[key][1]
    ax.add_patch(Circle(c, snap["rho_mm"], fc="white" if key != "dial" else "#fff3d6", ec=colour, lw=0.8, alpha=a,
                        zorder=3))
    spin = state["spin"]
    ax.plot([c[0], c[0] + snap["rho_mm"] * np.cos(-np.pi / 2 - spin)],
            [c[1], c[1] + snap["rho_mm"] * np.sin(-np.pi / 2 - spin)], color=colour, lw=0.6, alpha=a, zorder=4)


def scenes(fig, rect, snap):
    """(c): the three tasks, stacked over the figure's height in rect (x0, y0, w, h, figure fractions), each at its
    Drake state; red arrows give the load."""
    x0, y0, w, h = rect
    for i, key in enumerate(("hold", "acquire", "dial")):
        ax = fig.add_axes((x0, y0 + (2 - i) * h / 3, w, h / 3))
        t = snap[key]
        if key == "acquire":
            scene(ax, snap, key, t["start"], faint=True)
        scene(ax, snap, key, t["end"])
        c = np.array(t["end"]["centre"])
        if key == "hold":
            ax.annotate("", xy=c + [11, 0], xytext=c, arrowprops=dict(arrowstyle="-|>", color="#b8322a", lw=0.7,
                                                                    mutation_scale=5), zorder=5)
        elif key == "acquire":
            s0 = np.array(t["start"]["centre"])
            ax.annotate("", xy=c + [-0.5, 0], xytext=s0, arrowprops=dict(arrowstyle="-|>", color=INK2, lw=0.6,
                                                                        mutation_scale=5, ls=(0, (2, 1))), zorder=5)
            ax.annotate("", xy=c + [8, 8], xytext=c, arrowprops=dict(arrowstyle="-|>", color="#b8322a", lw=0.7,
                                                                    mutation_scale=5), zorder=5)
        else:
            axis = np.array(t["axis"])
            zz = np.linspace(axis[1] - 16, axis[1], 9)
            xx = axis[0] + 2.2 * np.array([0, 1, -1, 1, -1, 1, -1, 1, 0])
            ax.plot(xx, zz - 4, color=INK2, lw=0.5, zorder=1)
            ax.plot(axis[0], axis[1], marker="+", color=INK2, ms=4, mew=0.6, zorder=5)
        ax.set_xlim(4, 62); ax.set_ylim(-47, 4); ax.set_aspect("equal", adjustable="datalim"); ax.axis("off")
        ax.text(1.0, 0.92, TASK[key][0], transform=ax.transAxes, fontsize=5.8, color=TASK[key][1], ha="right",
                va="top")


def main() -> int:
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix", "font.size": 7})
    fig = plt.figure(figsize=(WIDTH, HEIGHT))
    gs = fig.add_gridspec(1, 4, width_ratios=[1, 1, 0.62, 1], wspace=0.22, left=0.055, right=0.995, bottom=0.2,
                          top=0.61)
    axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), None, fig.add_subplot(gs[0, 3])]
    box = gs[0, 2].get_position(fig)
    scenes(fig, (box.x0 - 0.01, 0.02, box.width + 0.02, 0.9), json.loads((DRAKE / "task_snapshots.json").read_text()))
    fig.text(box.x0 - 0.01, 0.985, "(c) tasks on $O$", fontsize=7, fontweight="bold", ha="left", va="top")

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
    print(out, f"{WIDTH} x {HEIGHT} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
