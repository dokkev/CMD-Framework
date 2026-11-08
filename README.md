# CoDe Hand

A research platform for Contact Morphology Design: a robotic hand designed around its contact interfaces. The project
goal and claim discipline are in [GOAL.md](GOAL.md).

## Layout

```text
cmd_framework/
├── GOAL.md            project north star
├── docs/              decisions, task notes
├── report/            study reports and paper figures (report/figures/)
├── design/            CMD-Framework: mechanism geometry, kinematics, optimization, studies
│   ├── core/          morphology, mechanisms (crossed four-bar FK), packaging, evaluators, optimizer
│   ├── scripts/       optimization runs and studies (scripts/study/…)
│   ├── visualize/     plotting
│   ├── tests/
│   └── output/        run outputs (not tracked)
└── sim/
    └── code_isaaclab/ Isaac Lab external project: physics simulation, contact experiments, later learning
```

The two subprojects keep separate environments and exchange data only through versioned exports.

| | `design/` | `sim/code_isaaclab/` |
|---|---|---|
| Environment | conda `cmd` (Python 3.11) | uv (Python 3.12), Isaac Lab 3.0 checkout at `../IsaacLab` beside this repository |
| Run | from `design/`: `python scripts/study/...` | from `sim/code_isaaclab/`: `uv run ...` |
| Tests | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest` (repository root or `design/`) | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest` |

Studies write figures and reports to the repository-level `report/`. Their run records stay under
`design/output/`.

The simulation side consumes geometry exports that carry their source commit. It does not reimplement or import
`design/core`. See `sim/code_isaaclab/README.md`.
