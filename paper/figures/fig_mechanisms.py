"""Fig. 2 of the RSS 2027 extended abstract: the four PIP joints compared, and how far each one's pole sits from
the contact face (PNG only).

    conda run -n cmd python paper/figures/fig_mechanisms.py [dest_dir]

(a-d) Sketches at 120 deg of flexion, one scale, in the style of Study 1's design figures
(design/scripts/study/study1_thickness_coverage/figure_designs.py), with pivots dotted and no palm hatching.
2R-DD, 2R-remote, RCJ: each one's Study 1 design for q_req = 120 deg and 7 mm pads. CFL: the CoDe prototype's
finger (design/output/linkage/config/paper_single_objective/portfolio_001, balanced), drawn qualitatively: pads
at 7 mm like the others, the distal pad laid on its link, and A'' placed at A so the proximal triangle closes
there.
(e) Face depth minus pole depth at q_req = 120 deg for Study 1's designs (design/output/study/study2_pole_law/
designs.csv): the gap of Eq. (2) is 2 (t - z) tan(q/2), so a smaller value leaves more room for the pads.
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
    # (e) the pole is designable: how far the pole sits above the contact face, against pad thickness
    left = 0.005 + len(ARCH) * panel_w / WIDTH + 0.075
    pax = fig.add_axes((left, 0.27, 0.995 - left - 0.005, 0.56))
    rows = [r for r in csv.DictReader(open(POLE_LAW)) if float(r["required_flexion_deg"]) == FLEXION]
    for key, title, colour, _, ours in ARCH:
        pts = sorted((float(r["thickness_mm"]), float(r["face_depth_proximal_mm"]) - float(r["pole_depth_at_req_mm"]))
                     for r in rows if r["architecture"] == key)
        h, gap = map(np.array, zip(*pts))
        pax.plot(h, gap, color=colour, marker=MARK[key], ms=3.6, lw=1.4 if ours else 1.1, mec="white", mew=0.4,
                 zorder=3 if ours else 2)
    pax.set_xticks([3, 5, 7, 9]); pax.set_xlim(2.5, 9.5); pax.set_ylim(0, 14)
    pax.set_xlabel("pad thickness $h$ (mm)", fontsize=7, labelpad=1)
    pax.set_ylabel("face above pole (mm)", fontsize=7, labelpad=1)
    pax.tick_params(labelsize=6.5, length=2)
    for side in ("top", "right"):
        pax.spines[side].set_visible(False)
    pax.grid(axis="y", color="#e4e3df", lw=0.5)
    fig.text(left - 0.045, 1 - 0.02 / height, "(e) pole vs face, $120^\\circ$", ha="left", va="top", fontsize=7.5,
             fontweight="bold")
    DEST.mkdir(parents=True, exist_ok=True)
    path = DEST / "fig_mechanisms.png"
    fig.savefig(path, dpi=400)
    plt.close(fig)
    print(path, f"{WIDTH:.2f} x {height:.2f} in")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
