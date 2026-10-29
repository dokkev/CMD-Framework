"""Rebuild the comparison's paper figures from one source of runs.

    python -m visualize.paper.build [source] [height_mm] [flexion_deg]

``source``: the comparison study's output folder (default
``output/comparison_quick``: drafts). Collects the table
(``visualize.paper.collect``), then draws the Visual and the Scientific
figures at the given condition (default 5 mm, 150 deg) and the Visual one at
every condition of the grid.
"""

from __future__ import annotations

import sys

from visualize.paper import collect, fig_scientific, fig_visual


def main(argv: list[str]) -> int:
    source = argv[0] if argv else "output/comparison_quick"
    height, flexion = (argv[1], argv[2]) if len(argv) > 2 else ("5", "150")
    name = source.rstrip("/").split("/")[-1]
    collect.main([source])
    fig_scientific.main([height, flexion, name])
    for h in collect.STUDY.THICKNESSES_MM:
        for f in collect.STUDY.FLEXIONS_DEG:
            fig_visual.main([str(h), str(f), name])
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
