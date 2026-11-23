"""Fig. 2 of the RSS 2027 extended abstract: the four PIP joints compared, and how far each one's pole sits from
the contact face (PNG only).

    conda run -n cmd python paper/figures/fig_mechanisms.py [dest_dir]

(a-d) Sketches at 120 deg of flexion, one scale, in the style of Study 1's design figures
(design/scripts/study/study1_thickness_coverage/figure_designs.py), with pivots dotted and no palm hatching.
2R-DD, 2R-remote, RCJ: each one's Study 1 design for q_req = 120 deg and 7 mm pads. CFL: the CoDe prototype's
finger (design/output/linkage/config/paper_single_objective/portfolio_001, balanced), drawn qualitatively: pads
at 7 mm like the others, the distal pad laid on its link, and A'' placed at A so the proximal triangle closes
there.
(e) Where each joint's finite pole sits at q_req = 120 deg, relative to the proximal pad: along the finger from
its joint-side end, and above its mounting surface. A pin or roller centre must stay above the mount. Dots: the
feasible CFL designs of output/linkage/config/samples/seeds.csv under the prototype config; markers: each
architecture's Study 1 designs for 3-9 mm pads (the CFL's descend into the pad as the pads thicken).
Drawn at print size (full text width, 7.16 in).
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DESIGN = ROOT / "design"
for path in (DESIGN, DESIGN / "scripts" / "study", DESIGN / "scripts" / "study" / "study2_morphology_adaptation"):
    sys.path.insert(0, str(path))

import one_finger  # noqa: E402
from core.optimization.mechanism import read_config_file  # noqa: E402
from visualize import sketch  # noqa: E402
from visualize.finger import view_of  # noqa: E402
from dataclasses import replace  # noqa: E402

from visualize.finger.view import load_run  # noqa: E402
from visualize.fronts import read_rows, with_role  # noqa: E402
from visualize.style import use  # noqa: E402

DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
FLEXION, THICKNESS = 120.0, 7.0
POLE_LAW = DESIGN / "output" / "study" / "study2_pole_law" / "designs.csv"
MARK = {"2r_dd": "^", "2r_remote": "s", "rcj": "D", "cfl": "o"}
PROTOTYPE = DESIGN / "output" / "linkage" / "config" / "paper_single_objective" / "portfolio_001" / "seeds.csv"
WIDTH = 7.16
INK, INK2 = "#0b0b0b", "#52514e"
# (key, title, colour as in the paper's other figures, one-line description, ours)
ARCH = [("2r_dd", "2R-DD", "#1baf7a", "servo at the joint", False),
        ("2r_remote", "2R-remote", "#eb6834", "pin joint, belt from a proximal servo", False),
        ("rcj", "RCJ", "#eda100", "rolling contact joint", False),
        ("cfl", "CFL (ours)", "#2a78d6", "crossed four-bar, proximal servo", True)]


SAMPLES = DESIGN / "output" / "linkage" / "config" / "samples" / "seeds.csv"


def finite_pole(view, q_deg: float) -> np.ndarray:
    """The point the distal phalanx turns about from extension to q (Lemma 1), from two distal-pad corners."""
    p0, p1 = view.distal_pad(0.0)[:2], view.distal_pad(q_deg)[:2]
    d0, d1 = p0[1] - p0[0], p1[1] - p1[0]
    th = np.arctan2(d1[1], d1[0]) - np.arctan2(d0[1], d0[0])
    rot = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return np.linalg.solve(np.eye(2) - rot, p1[0] - rot @ p0[0])


def pole_on_pad(view, q_deg: float) -> tuple[float, float]:
    """The pole relative to the proximal pad: along the finger from its joint-side end, and height above its
    mounting surface (negative: inside the pad, toward the contact face)."""
    pole, pad = finite_pole(view, q_deg), np.asarray(view.proximal_pad())
    return float(pole[0] - pad[:, 0].max()), float(pole[1] - pad[:, 1].max())


def main() -> int:
    use("paper")
    sketch.LINK_WIDTH = 2.0  # thinner links at print size
    sketch.SHOW_GROUND = False  # no palm hatching beside the MCP servo
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix"})
    config = read_config_file(one_finger.DEFAULT_CONFIG)
    views = {}
    for key, *_ in ARCH:
        if key == "cfl":  # the CoDe prototype's finger, its pads drawn as thick as the others' (qualitative)
            row = with_role(read_rows(PROTOTYPE), "balanced")
            run = load_run(PROTOTYPE.parent.parent / row["run"].split("/")[-1])
            pk = run.config.packaging
            config_cfl = replace(run.config, packaging=replace(
                pk, proximal_pad=replace(pk.proximal_pad, height_mm=THICKNESS),
                distal_pad=replace(pk.distal_pad, height_mm=THICKNESS)))
            x = list(run.mechanism.read_design(row))
            names = list(run.mechanism.variable_names)
            x[names.index("distal_mount_offset_mm")] = 0.0  # pad on its link, no pivot clearance (qualitative)
            # A'' (= A'_x + offset) at A: the red triangle O-A'-A'' closes at the pivot
            x[names.index("app_offset_mm")] = x[names.index("a_x_mm")] - x[names.index("ap_x_mm")]
            views[key] = view_of(run.mechanism.name, config_cfl, x)
            continue
        designs = one_finger.study1_designs(key, config["architectures"][key], FLEXION, config)
        ref = designs[THICKNESS]
        views[key] = view_of(ref.mechanism, ref.config, ref.x)
    probe_fig, probe = plt.subplots()
    for view in views.values():
        view.draw(probe, FLEXION, hit=set(), style="sketch")
    box = probe.dataLim
    plt.close(probe_fig)
    xlim, ylim = (box.x0 - 1.5, box.x1 + 1.5), (box.y0 - 1.5, box.y1 + 1.5)
    panel_w = 1.26  # in, per sketch; a plot panel follows
    panel_h = panel_w * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    title_h, foot_h = 0.2, 0.0
    height = title_h + panel_h + foot_h
    fig = plt.figure(figsize=(WIDTH, height))
    for i, (key, title, colour, desc, ours) in enumerate(ARCH):
        x0 = 0.005 + i * panel_w / WIDTH
        ax = fig.add_axes((x0, foot_h / height, panel_w / WIDTH, panel_h / height))
        sketch.frame(ax, xlim, ylim)
        view = views[key]
        view.draw(ax, FLEXION, hit=set(), style="sketch")
        fig.text(x0 + panel_w / WIDTH / 2, 1 - 0.02 / height, title, ha="center", va="top", fontsize=8.5,
                 color=colour, fontweight="bold" if ours else "normal")
    for i, lab in enumerate("abcd"):
        fig.text(0.005 + i * panel_w / WIDTH + 0.004, 1 - 0.02 / height, f"({lab})", ha="left", va="top", fontsize=7.5,
                 fontweight="bold")
    # (e) where each joint's pole can sit at q_req, relative to the proximal pad
    left = 0.005 + len(ARCH) * panel_w / WIDTH + 0.07
    pax = fig.add_axes((left, 0.25, 0.995 - left - 0.005, 0.6))
    pax.add_patch(plt.Rectangle((-14, 0), 40, 14, fc="#e9e8e4", ec="none", zorder=0))
    pax.text(25, 12.8, "pins, rollers", ha="right", va="top", fontsize=6.3, color=INK2)
    pax.add_patch(plt.Rectangle((-14, -THICKNESS), 14, THICKNESS, fc="#f6c4c3", ec="#e34948", lw=0.6, zorder=0.5))
    pax.text(-7, -THICKNESS / 2, "pad", ha="center", va="center", fontsize=6.3, color=INK2)
    pax.axhline(0, color=INK2, lw=0.5, zorder=1)
    run = load_run(PROTOTYPE.parent.parent / "run_003")
    names = list(run.mechanism.variable_names)
    cloud = []
    for r in csv.DictReader(open(SAMPLES)):
        try:
            cloud.append(pole_on_pad(view_of("linkage", run.config, [float(r[n]) for n in names]), FLEXION))
        except Exception:  # noqa: BLE001 -- a sample that does not reach the flexion is skipped
            continue
    cloud = np.array(cloud)
    pax.scatter(cloud[:, 0], cloud[:, 1], s=1.2, color="#2a78d6", alpha=0.25, lw=0, zorder=2)
    for key, title, colour, _, ours in ARCH:
        pts = []
        for h, ref in one_finger.study1_designs(key, config["architectures"][key], FLEXION, config).items():
            try:
                pts.append(pole_on_pad(view_of(ref.mechanism, ref.config, ref.x), FLEXION))
            except Exception:  # noqa: BLE001
                continue
        pts = np.array(pts)
        pax.scatter(pts[:, 0], pts[:, 1], s=14, marker=MARK[key], color=colour, edgecolor="white", lw=0.4,
                    zorder=4 if ours else 3)
        if ours:  # the CFL's pole descends with the pad thickness
            pax.annotate("", xy=(pts[-1, 0] + 2.6, pts[-1, 1]), xytext=(pts[0, 0] + 2.6, pts[0, 1]),
                         arrowprops=dict(arrowstyle="->", color=colour, lw=0.7))
            pax.text(pts[-1, 0] + 3.4, (pts[0, 1] + pts[-1, 1]) / 2, "3$\\to$9 mm", color=colour, fontsize=6.3,
                     ha="left", va="center")
    pax.set_xlim(-14, 26); pax.set_ylim(-THICKNESS - 1.5, 14)
    pax.set_aspect("equal")
    pax.set_xlabel("from the pad's end (mm)", fontsize=7, labelpad=1)
    pax.set_ylabel("above the mount (mm)", fontsize=7, labelpad=1)
    pax.tick_params(labelsize=6.5, length=2)
    for side in ("top", "right"):
        pax.spines[side].set_visible(False)
    pax.grid(False)
    fig.text(left - 0.04, 1 - 0.02 / height, "(e) where the pole can sit", ha="left", va="top", fontsize=7.5,
             fontweight="bold")
    DEST.mkdir(parents=True, exist_ok=True)
    path = DEST / "fig_mechanisms.png"
    fig.savefig(path, dpi=400)
    plt.close(fig)
    print(path, f"{WIDTH:.2f} x {height:.2f} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
