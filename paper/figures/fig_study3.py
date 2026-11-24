"""Fig. 5 of the RSS 2027 extended abstract, Study 3 in one double-column figure (PNG only, print size).

    cd sim/drake && uv run python ../../paper/figures/fig_study3.py [dest_dir]

All panels: the error a redesign adds to a measured interface effect, E (median over placements, log scale), against
its change of the contact Jacobian, Delta_contact, one marker per redesigned hand, under the reused impedance setpoint
(condition A, open loop).
(b) Isaac Lab 3 (Newton, MuJoCo-Warp), point contact: pad friction at O's contacts (filled, colour by thickness),
    pad stiffness (squares), friction on the hand-independent grid (grey rings); dotted: the CFL redesigns of Study 2;
    dashed: E = 20%. Inputs: sim/isaaclab/logs/study3/two_pad/dose_A_all.json, comp.json,
    dose_A-grid_expanded.json (as make_figures.fig_study3 before 2026-11-23).
(c) Contact model, pad friction, grasp pull-out: Isaac Lab point (as (b), filled), the Drake replica with point contact, and
    Drake with hydroelastic pads (E = 0.1 MPa over 3 mm on every hand). Inputs: sim/drake/logs/e21_point.json,
    e21_hydro.json (analyze.py --out); and with each pad as soft as it is thick (0.1 MPa over its own thickness),
    against O carrying the same pad (experiment 25; e25_hydro_own.json, analyze_e25.py --out).
(a) The three tasks on hand O (radius 15 mm, q0 = 100 deg, mu 1.3), three frames each in order, rendered in 3D by
    Drake's VTK renderer at simulated states (sim/drake/render_tasks.py -> logs/task_snapshots.json; render3d.py ->
    logs/render_<task><frame>.png): O's linkage (links drawn bowed, pins exact), pads and servos; red arrows: load.
Panels (b)-(d) below were (a), (b), (d) before 2026-11-23.
(d) Task, Drake, point, pad friction: grasp pull-out (the load rising until the grasp gives way), grasp acquisition, turning a sprung dial. Inputs:
    e21_point.json, e23_point.json, e26_point.json (analyze_roll.py --out; the dial of experiment 26, turned about 30
    degrees, replaced experiment 24's 5-10 degree dial on 2026-11-23). In-hand rolling (experiment 22, which
    failed its test) is left out of the figure; the paper states it as a limitation.
Legends give Spearman rho(E, Delta_contact) and its exact or Monte Carlo one-sided permutation p (sim/drake/analyze.py).
Experiments 21-26: report/study3/experiments.md.
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "report" / "study3" / "data"  # paper data, committed (copied from sim/*/logs)
sys.path.insert(0, str(ROOT / "sim" / "drake"))
from analyze import spearman_perm  # noqa: E402

DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
STRIP_IN = 0.63  # (a), the task frames
PLOT_IN, LEGEND_IN, BOTTOM_IN = 0.78, 0.53, 0.28  # each plot, its title and legend above it, the x label
WIDTH, HEIGHT = 7.16, 3.2
THICK = {5: "#93bde9", 7: "#2a78d6", 9: "#0d3c78"}  # as the other figures: pad thickness, light to dark
GREY, INK2, INK = "#9a9a9a", "#555555", "#0b0b0b"
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
    # the panel title heads the legend above the axes, so neither covers data
    leg = ax.legend(title=title, fontsize=5.8, frameon=False, loc="lower left", bbox_to_anchor=(-0.02, 1.01),
                    handletextpad=0.2, borderaxespad=0.0, labelspacing=0.15, handlelength=1.0, alignment="left")
    leg.get_title().set_fontsize(7)
    leg.get_title().set_fontweight("bold")


TASK = {"hold": ("grasp pull-out", "#2a78d6"), "acquire": ("grasp acquisition", "#9b5de5"), "dial": ("dial turning", "#eda100")}


def rot(q, p):
    """Flexion by q about +y in (x, z dorsal): +x toward -z."""
    c, s_ = np.cos(q), np.sin(q)
    return np.array([c * p[0] + s_ * p[1], -s_ * p[0] + c * p[1]])


def strip(fig, y0, y1):
    """(a): each task on O in three frames, in order, rendered in 3D by Drake (sim/drake/render3d.py) at simulated
    states, across the figure between heights y0 and y1 (figure fractions); one crop box per task, so its frames
    share a scale within each task."""
    from PIL import Image
    ims = {k: [np.asarray(Image.open(DATA / "renders" / f"render_{k}{i}.png").convert("RGB")) for i in range(3)] for k in TASK}
    boxes = {}
    for k, v in ims.items():  # one crop box per task, so its three frames share a scale
        ink = np.any(np.stack([im.min(axis=2) < 245 for im in v]), axis=0)
        rows, cols = np.where(ink)
        left = int(0.22 * ink.shape[1])  # from the MCP joint on; the MCP servo is left out
        boxes[k] = (max(rows.min() - 6, 0), rows.max() + 6, max(cols.min(), left), cols.max() + 6)
    x0, x1, gap = 0.012, 0.995, 0.02
    gw = (x1 - x0 - 2 * gap) / 3
    label_h = 0.13 / HEIGHT
    for g, key in enumerate(TASK):
        gx = x0 + g * (gw + gap)
        fig.text(gx + gw / 2, y1 - 0.01, TASK[key][0], fontsize=6.5, color=TASK[key][1], ha="center", va="top")
        for i, im in enumerate(ims[key]):
            fw = 0.92 * gw / 3  # frame width; the rest of a third is the gap with its arrow
            ax = fig.add_axes((gx + i * gw / 3 + (gw / 3 - fw) / 2, y0, fw, y1 - y0 - label_h))
            box = boxes[key]
            ax.imshow(im[box[0]:box[1], box[2]:box[3]], interpolation="lanczos")
            ax.set_anchor("N")
            ax.axis("off")
            if i < 2:
                fig.text(gx + (i + 1) * gw / 3, y0 + 0.45 * (y1 - y0 - label_h), r"$\rightarrow$", fontsize=5,
                         color=INK2, ha="center", va="center")


def strip_one(fig, key, x0, x1, y0, y1, steps):
    """One task on O in three frames between figure fractions (x0, x1) x (y0, y1), a caption under each frame."""
    from PIL import Image
    ims = [np.asarray(Image.open(DATA / "renders" / f"render_{key}{i}.png").convert("RGB")) for i in range(3)]
    ink = np.any(np.stack([im.min(axis=2) < 245 for im in ims]), axis=0)
    rows, cols = np.where(ink)
    left = int(0.22 * ink.shape[1])
    box = (max(rows.min() - 6, 0), rows.max() + 6, max(cols.min(), left), cols.max() + 6)
    gw = (x1 - x0) / 3
    cap = 0.2 / HEIGHT
    for i, im in enumerate(ims):
        ax = fig.add_axes((x0 + i * gw + 0.04 * gw, y0 + cap, 0.92 * gw, y1 - y0 - cap))
        ax.imshow(im[box[0]:box[1], box[2]:box[3]], interpolation="lanczos")
        ax.set_anchor("S")
        ax.axis("off")
        fig.text(x0 + (i + 0.5) * gw, y0 + cap - 0.01, steps[i], fontsize=5.9, color=INK2, ha="center", va="top",
                 linespacing=1.05)
        if i < 2:
            fig.text(x0 + (i + 1) * gw, y0 + cap + 0.45 * (y1 - y0 - cap), r"$\rightarrow$", fontsize=6,
                     color=INK2, ha="center", va="center")


def scatter_isaac(ax, key):
    """E against key ('D_c_mm' or 'D_s_mm') for the Isaac Lab sets at the reused setpoint."""
    grid = json.loads((DATA / "isaac_dose_A-grid_expanded.json").read_text())["hands"]
    fr = json.loads((DATA / "isaac_dose_A_all.json").read_text())["hands"]
    comp = json.loads((DATA / "isaac_comp.json").read_text())["hands"]
    gx = [r[key] for r in grid.values()]
    gy = [100 * r["E_rel_median"] for r in grid.values()]
    ax.plot(gx, gy, ls="none", marker="o", ms=2.6, mec=GREY, mfc="none", mew=0.6, zorder=2,
            label=label("friction, grid (45)", gx, gy))
    fx = [r[key] for r in fr.values()]
    fy = [100 * r["E_rel_median"] for r in fr.values()]
    for r in fr.values():
        c = THICK[int(r["thickness_mm"])]
        ax.plot(r[key], 100 * r["E_rel_median"], ls="none", marker="o", ms=3.3, mec=c, mfc=c, mew=0.8, zorder=3)
    ax.plot([], [], ls="none", marker="o", ms=3.3, mec=THICK[7], mfc=THICK[7], label=label("friction (15)", fx, fy))
    sx = [r[key] for r in comp.values()]
    sy = [100 * r["E_median"] for r in comp.values()]
    for hand, r in comp.items():
        c = THICK[7 if not hand.startswith("h3to") else int(hand[4])]
        ax.plot(r[key], 100 * r["E_median"], ls="none", marker="s", ms=3.0, mec=c, mfc="white", mew=0.8, zorder=3)
    ax.plot([], [], ls="none", marker="s", ms=3.0, mec=THICK[7], mfc="white", mew=0.8,
            label=label("stiffness (14)", sx, sy))
    ax.axhline(20, color=INK2, lw=0.6, ls=(0, (3, 2)), zorder=1)


def main() -> int:
    """Fig. 4 (2026-11-24 figure plan), double column, about 80 mm tall: (a) the protocol on O, grasp pull-out in
    three Drake frames; (b) a reversed conclusion: the softer, thicker pad's effect measured on each 7 mm redesign
    at O's placements, against its true effect on O; (c) the error E against Delta_contact and (d) against
    Delta_skel, Isaac Lab at the reused setpoint. The other simulator, contact models and tasks go to Table I."""
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix", "font.size": 7})
    fig = plt.figure(figsize=(WIDTH, HEIGHT))
    lab = dict(fontsize=7.2, fontweight="bold", ha="left", va="top")
    # (a) protocol
    fig.text(0.004, 0.995, "(a) protocol on $O$: grasp pull-out", **lab)
    strip_one(fig, "hold", 0.0, 0.47, 0.585, 0.95,
              ("close to $q_0+5^\\circ$\nat stiffness $K_0$", "release the\ncylinder", "load until\nthe grasp gives way"))
    # (b) a reversed conclusion
    fig.text(0.525, 0.995, "(b) a reversed conclusion ($7$ mm, $O$'s placements)", **lab)
    bx = fig.add_axes((0.585, 0.62, 0.405, 0.31))
    con = json.loads((DATA / "isaac_conclusion_study3.json").read_text())["hands"]
    dc = {"R": 0.86, "U": 3.91}
    hands = [(k, v) for k, v in con.items() if int(v["thickness_mm"]) == 7]
    def dmm(k):
        if k in dc:
            return dc[k]
        return float(k.split("_")[0][1:].replace("p", "."))
    hands.sort(key=lambda kv: (dmm(kv[0]), kv[0]))
    xs = np.arange(len(hands))
    for i, (k, v) in enumerate(hands):
        m = v["measured_change_pct"]
        col = "#2a78d6" if k == "R" else ("#c0392b" if m > 0 else "#9a9a9a")
        bx.bar(i, m, width=0.62, color=col, zorder=3, hatch="////" if "maxDs" in k else None, edgecolor="white",
               lw=0)
        bx.plot([i - 0.38, i + 0.38], [v["true_change_pct"]] * 2, color=INK, lw=1.2, zorder=5)
        bx.text(i, m + (3 if m > 0 else -3), f"{m:+.0f}", fontsize=5.6, ha="center", va="bottom" if m > 0 else "top",
                color=INK2)
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    bx.legend(handles=[Line2D([], [], color=INK, lw=1.2, label="true effect, on $O$"),
                       Patch(fc="#c0392b", label="sign reversed"),
                       Patch(fc="#9a9a9a", hatch="////", ec="white", label="largest $\\Delta_{\\mathrm{skel}}$")],
              fontsize=5.8, frameon=False, loc="upper left", handlelength=1.4, borderaxespad=0.1, labelspacing=0.2)
    bx.axhline(0, color=INK2, lw=0.5, zorder=2)
    names = [("$R$\n" if k == "R" else "$U$\n" if k == "U" else "") + f"{dmm(k):.3g}" for k, _ in hands]
    bx.set_xticks(xs); bx.set_xticklabels(names, fontsize=5.8)
    bx.set_xlabel(r"7 mm redesign, by $\Delta_{\mathrm{contact}}$ (mm)", fontsize=6.6, labelpad=1)
    bx.set_ylabel("measured effect (%)", fontsize=6.6, labelpad=1)
    bx.set_ylim(-85, 65)
    bx.tick_params(labelsize=6, length=2)
    for side in ("top", "right"):
        bx.spines[side].set_visible(False)
    # (c), (d) the error against the contact-surface and the skeleton motion change
    for x0, key, title, xl in ((0.055, "D_c_mm", "(c) error vs. contact-surface motion change", r"$\Delta_{\mathrm{contact}}$ (mm)"),
                               (0.585, "D_s_mm", "(d) error vs. skeleton motion change", r"$\Delta_{\mathrm{skel}}$ (mm)")):
        ax = fig.add_axes((x0, 0.10, 0.405, 0.245))
        scatter_isaac(ax, key)
        ax.set_yscale("log"); ax.set_ylim(*YLIM)
        ax.set_yticks([3, 10, 30, 100, 300]); ax.set_yticklabels(["3", "10", "30", "100", "300"]); ax.minorticks_off()
        ax.set_xlabel(xl, fontsize=6.8, labelpad=1)
        ax.set_ylabel("error $E$ (%)", fontsize=6.8, labelpad=1)
        ax.tick_params(labelsize=6, length=2)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.set_xlim(left=0)
        leg = ax.legend(title=title, fontsize=5.8, frameon=False, loc="lower left", bbox_to_anchor=(-0.02, 1.01),
                        handletextpad=0.2, borderaxespad=0.0, labelspacing=0.15, handlelength=1.0, alignment="left",
                        ncol=2, columnspacing=0.8)
        leg.get_title().set_fontsize(7.2)
        leg.get_title().set_fontweight("bold")
    DEST.mkdir(parents=True, exist_ok=True)
    out = DEST / "fig_study3.png"
    fig.savefig(out, dpi=400)
    from PIL import Image
    Image.open(out).convert("RGB").save(out)  # opaque RGB, no SMask (PaperPlaza)
    print(out, f"{WIDTH} x {HEIGHT} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
