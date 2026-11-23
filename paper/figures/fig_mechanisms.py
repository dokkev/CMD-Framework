"""Fig. 2 of the RSS 2027 extended abstract: the four PIP architectures compared (PNG only).

    conda run -n cmd python paper/figures/fig_mechanisms.py [dest_dir]

2R-DD, 2R-remote and RCJ: each one's Study 1 design for q_req = 120 deg and 7 mm pads. CFL: the CoDe prototype's
finger (design/output/linkage/config/paper_single_objective/portfolio_001, balanced; prototype track, drawn only).
All drawn at 120 deg of flexion in the
sketch style of Study 1's design figures (design/scripts/study/study1_thickness_coverage/figure_designs.py):
grey servo bodies, red proximal and blue distal links, pads shaded, dotted the distal segment's centre of
rotation from extension. One scale for every panel. Drawn at print size (full text width, 7.16 in).
"""
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
from visualize.finger.view import view_from_row  # noqa: E402
from visualize.fronts import read_rows, with_role  # noqa: E402
from visualize.style import use  # noqa: E402

DEST = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "ContactCentricDesign-RSS2027" / "figures"
FLEXION, THICKNESS = 120.0, 7.0
PROTOTYPE = DESIGN / "output" / "linkage" / "config" / "paper_single_objective" / "portfolio_001" / "seeds.csv"
WIDTH = 7.16
INK, INK2 = "#0b0b0b", "#52514e"
# (key, title, colour as in the paper's other figures, one-line description, ours)
ARCH = [("2r_dd", "2R-DD", "#1baf7a", "servo at the joint", False),
        ("2r_remote", "2R-remote", "#eb6834", "pin joint, belt from a proximal servo", False),
        ("rcj", "RCJ", "#eda100", "rolling contact joint", False),
        ("cfl", "CFL (ours)", "#2a78d6", "crossed four-bar, proximal servo", True)]


def main() -> int:
    use("paper")
    plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "Times New Roman", "DejaVu Serif"],
                         "mathtext.fontset": "stix"})
    config = read_config_file(one_finger.DEFAULT_CONFIG)
    views = {}
    for key, *_ in ARCH:
        if key == "cfl":  # the CoDe prototype's finger
            views[key] = view_from_row(with_role(read_rows(PROTOTYPE), "balanced"), PROTOTYPE.parent.parent)
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
    panel_w = 1.46  # in, per panel; the row is centred
    panel_h = panel_w * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])
    title_h, foot_h = 0.2, 0.0
    height = title_h + panel_h + foot_h
    fig = plt.figure(figsize=(WIDTH, height))
    for i, (key, title, colour, desc, ours) in enumerate(ARCH):
        x0 = 0.5 - len(ARCH) * panel_w / WIDTH / 2 + i * panel_w / WIDTH
        ax = fig.add_axes((x0, foot_h / height, panel_w / WIDTH, panel_h / height))
        sketch.frame(ax, xlim, ylim)
        view = views[key]
        view.draw(ax, FLEXION, hit=set(), style="sketch")
        sketch.icr_path(ax, view.icr(np.linspace(0.0, FLEXION, 61)), size=5, linewidth=0.6)
        fig.text(x0 + panel_w / WIDTH / 2, 1 - 0.02 / height, title, ha="center", va="top", fontsize=8.5,
                 color=colour, fontweight="bold" if ours else "normal")
    DEST.mkdir(parents=True, exist_ok=True)
    path = DEST / "fig_mechanisms.png"
    fig.savefig(path, dpi=400)
    plt.close(fig)
    print(path, f"{WIDTH:.2f} x {height:.2f} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
