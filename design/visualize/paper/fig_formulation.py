"""Paper figures: the prototype and the study formulations as typeset equations
(``design/SSoT/problem_formulation.md`` A.5.4 and A.6), rendered with
Matplotlib's mathtext in Computer Modern (no LaTeX installation needed).

    python -m visualize.paper.fig_formulation [out_dir]

Saves ``formulation_prototype.png`` and ``formulation_study.png`` (300 dpi) to
``out_dir`` (default: ``output/paper``). Each block is a list of rows: a
keyword column, right-aligned, and the math, left-aligned at one tab.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

from visualize.style import PAPER  # noqa: E402

FONT = 10.5  # pt
LINE = 0.30  # in, between rows
GAP = 0.12  # in, between a row's keyword and its math
INK = "#111111"

# The prototype formulation (A.5.4): its problem, then its terms.
PROTOTYPE = [
    ("box", [
        (r"$\underset{M}{\mathrm{maximize}}$", r"$(\bar C_{\mathrm{pad}}(M),\;MA_{\mathrm{wc}}(M))$"),
        (r"$\mathrm{subject\ to}$", r"$K=K_{\mathrm{CFL}},$"),
        ("", r"$\mathbf{m}_{\min}\leq\mathbf{m}(M)\leq\mathbf{m}_{\max},$"),
        ("", r"$\mathcal{F}_{\mathrm{static}}(M),\quad\mathcal{F}_{\mathrm{motion}}(M),$"),
        ("", r"$\gamma_{\min}\leq\gamma(M,\mathbf{q})\leq\gamma_{\max}\quad\forall\,\mathbf{q}\in\mathcal{Q}_{\mathrm{req}},$"),
        ("", r"$MA_{\mathrm{wc}}(M)\geq MA_{\min}$"),
    ]),
    ("rows", [
        (r"$\mathrm{where}$", r"$\bar C_{\mathrm{pad}}(M)=\dfrac{1}{|\mathcal{Q}_{\mathrm{eval}}|}"
                              r"\sum_{q\in\mathcal{Q}_{\mathrm{eval}}}"
                              r"\dfrac{2\,|T_1(M,q)\cap T_2(M,q)|}{L_f(q)},$"),
        ("", r"$\mathcal{Q}_{\mathrm{eval}}=\{90\degree,\;\frac{1}{2}(90\degree+q_{\mathrm{req}}),\;q_{\mathrm{req}}\},$"),
        ("", r"$MA_{\mathrm{wc}}(M)=\min_{\mathbf{q}\in\mathcal{Q}_{\mathrm{req}}}"
             r"\left|\dfrac{d\theta_{\mathrm{pip}}}{dq_2}\right|.$"),
    ]),
]

# The study formulation (A.6): the native references, Study 1 and Study 2.
STUDY = [
    ("head", "Native references"),
    ("rows", [
        ("", r"$M_0^K(q_{\mathrm{req}})=\underset{M\in\mathcal{M}_K}{\mathrm{arg\,max}}\;\bar C_{\mathrm{pad}}(M)$"),
        (r"$\mathrm{s.t.}$", r"$L_f(M)=90\ \mathrm{mm},\quad\mathbf{m}_{\min}\leq\mathbf{m}(M)\leq\mathbf{m}_{\max},"
                             r"\quad\mathcal{F}_{\mathrm{static}}(M),\quad\mathcal{F}_{\mathrm{motion}}(M),$"),
        ("", r"$K\in\{K_{\mathrm{2R‐DD}},\,K_{\mathrm{2R‐remote}},\,K_{\mathrm{CFL}},\,K_{\mathrm{RCJ}}\},"
             r"\quad q_{\mathrm{req}}\in\{90\degree,120\degree,150\degree\},$"),
        ("", r"$\gamma_{\min}\leq\gamma(M,\mathbf{q})\leq\gamma_{\max}\ \ \forall\,\mathbf{q}\in\mathcal{Q}_{\mathrm{req}}"
             r"\quad(K_{\mathrm{CFL}}\ \mathrm{only}),$"),
        None,
        ("", r"$\mathbf{d}(M)=(L_f,\,L_1,\,L_2,\,x_{\mathrm{art}}),\qquad"
             r"\mathbf{d}_0^K=\mathbf{d}(M_0^K(q_{\mathrm{req}}))$"),
    ]),
    ("head", "Study 1: contact modules carried by a fixed morphology"),
    ("rows", [
        ("", r"$M^K(c)=(K,\;C(c),\;A_0^K,\;S_0^K),\qquad c=(\ell_1,h_1,\ell_2,h_2)\in\mathcal{C}_{\mathrm{cat}}$"),
        ("", r"$\mathcal{C}_{\mathrm{feas}}(K;q_{\mathrm{req}})=\{\,c\in\mathcal{C}_{\mathrm{cat}}\;\mid\;"
             r"M^K(c)\ \mathrm{mounts}\ c,\;\;\mathcal{F}_{\mathrm{static}}(M^K(c)),\;\;"
             r"q_{\max}(M^K(c))\geq q_{\mathrm{req}}\,\}$"),
    ]),
    ("head", "Study 2: least change of morphology to carry a module"),
    ("rows", [
        ("", r"$\varepsilon_{d,K}^{\star}(c;\,q_{\mathrm{req}})=\min_{M}\;"
             r"\dfrac{\Vert\mathbf{d}(M)-\mathbf{d}_0^K\Vert_\infty}{L_{f,0}^K}$"),
        (r"$\mathrm{s.t.}$", r"$A=A_0^K,\quad M\ \mathrm{carries}\ c\ \mathrm{at}\ q_{\mathrm{req}}$"),
        None,
        ("", r"$\mathcal{R}_K(\tau;\,q_{\mathrm{req}})=\{\,(\ell,h)\;\mid\;"
             r"\varepsilon_{d,K}^{\star}((\ell,h,\ell,h);\,q_{\mathrm{req}})\leq\tau\,\}$"),
    ]),
]


def layout(figure, blocks, top: float, keyword_in: float) -> None:
    """Typeset ``blocks`` on ``figure`` from ``top`` (in) down: ``box``/``rows`` are rows
    of a keyword and its math (the math of every row at one tab), ``head`` a section title."""

    inches = figure.dpi_scale_trans
    y = top
    for kind, content in blocks:
        if kind == "head":
            figure.text(0.05, y, content, transform=inches, fontsize=FONT - 0.5, fontweight="bold",
                        fontfamily="DejaVu Sans", color=INK, va="baseline")
            y -= LINE * 1.05
            continue
        first = len(figure.texts)
        for row in content:
            if row is None:  # a gap between a problem and a definition
                y -= LINE * 0.45
                continue
            keyword, math = row
            if keyword:
                figure.text(keyword_in, y, keyword, transform=inches, fontsize=FONT, color=INK, ha="right",
                            va="baseline")
            figure.text(keyword_in + GAP, y, math, transform=inches, fontsize=FONT, color=INK, va="baseline")
            y -= LINE * (1.55 if "frac" in math else 1.35 if r"\underset" in math else 1.0)
        if kind == "box":  # a frame around the block's rows
            figure.canvas.draw()
            boxes = [t.get_window_extent() for t in figure.texts[first:]]
            x1 = max(b.x1 for b in boxes) / figure.dpi
            y0, y1 = min(b.y0 for b in boxes) / figure.dpi, max(b.y1 for b in boxes) / figure.dpi
            figure.patches.append(FancyBboxPatch((0.1, y0 - 0.06), x1 + 0.08 - 0.1, y1 - y0 + 0.12,
                                                 boxstyle="square,pad=0", transform=inches, fill=False,
                                                 edgecolor=INK, linewidth=0.8, figure=figure))
        y -= LINE * 0.45


def render(blocks, path: Path, keyword_in: float = 1.05, margin: float = 0.08) -> Path:
    """``blocks`` typeset once to measure them, then again on a page cut to them."""

    plt.rcParams.update({"mathtext.fontset": "cm", "font.family": "serif", "font.serif": ["DejaVu Serif"]})
    probe = plt.figure(figsize=(12.0, 20.0))
    layout(probe, blocks, 19.0, keyword_in)
    probe.canvas.draw()
    boxes = [t.get_window_extent() for t in probe.texts] + [p.get_window_extent() for p in probe.patches]
    x1 = max(b.x1 for b in boxes) / probe.dpi
    y0, y1 = min(b.y0 for b in boxes) / probe.dpi, max(b.y1 for b in boxes) / probe.dpi
    plt.close(probe)
    figure = plt.figure(figsize=(x1 + margin, y1 - y0 + 2 * margin))
    layout(figure, blocks, 19.0 - y0 + margin, keyword_in)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=300, facecolor="white")
    plt.close(figure)
    return path


def main(argv: list[str]) -> int:
    out = Path(argv[0]) if argv else PAPER
    print(render(PROTOTYPE, out / "formulation_prototype.png"))
    print(render(STUDY, out / "formulation_study.png", keyword_in=0.55))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
