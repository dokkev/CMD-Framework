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
    └── isaaclab/      Isaac Lab external project (package code_isaaclab): physics simulation, contact experiments, later learning
```

The two subprojects keep separate environments and exchange data only through versioned exports.

| | `design/` | `sim/isaaclab/` |
|---|---|---|
| Environment | conda `cmd` (Python 3.11) | uv (Python 3.12), Isaac Lab 3.0 checkout at `../IsaacLab` beside this repository |
| Run | from `design/`: `python scripts/study/...` | from `sim/isaaclab/`: `uv run ...` |
| Tests | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest` (repository root or `design/`) | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest` |

Studies write figures and reports to the repository-level `report/`. Their run records stay under
`design/output/`.

**Figures: PNG only, no PDF.** Figure scripts save `.png` (plus `.csv` for the numbers). No script writes a `.pdf`,
to `report/`, `design/output/` or anywhere else, and no PDF is committed. Do not add PDF output to a new script.

## Two optimizations: prototype and study

`design/` runs the same optimizer (`scripts/optimize/run.py`) for two different jobs. They have separate configs,
separate packaging assumptions and separate outputs. **Never mix them.**

| | Prototype optimization | Study optimization |
|---|---|---|
| Purpose | Design the CoDe finger that gets built (hardware workstream, `report/prototype_hardware.md`) | Answer GOAL.md's research questions: mechanism comparison, Studies 1-4 (`report/kinematic_study_freeze.md`) |
| Configs | `design/scripts/optimize/linkage/config.yaml` and the files extending it (`config_pivot8.yaml`, ...) | `design/scripts/study/base.yaml` → `baseline.yaml` (a frozen copy of the production config at 6ccab53) and the files under `design/scripts/study/` |
| Packaging | Follows the hardware as it is chosen (e.g. pivots A, A', B, B' 8 mm in `config_pivot8.yaml`) | Fixed and shared by every mechanism: pivots A 6, A' 10, B 6, B' 6 mm, idealized planar envelopes |
| Outputs | `design/output/linkage/<config>/...`, hand variants such as `core/morphology/code_hand/pivot8_balanced.yaml`, sim exports | `design/output/study/...`, `report/` study reports |
| Code name | "production" | "study" |

Rules:
- **A study config never takes a prototype condition.** Do not change a packaging, bound or rule under
  `scripts/study/` because the prototype changed (for example, no pivot-8 study variant). The study's assumptions
  change only by a decision about the study itself, made in `base.yaml`/`baseline.yaml` for every mechanism at once.
- **The prototype config never reads study rules or outputs.** A change to `baseline.yaml` does not reach
  `scripts/optimize/linkage/config.yaml`, and the reverse is also true.
- **Results are not compared across the two.** A prototype run's numbers (k_max, φ, pad lengths) and a study's numbers
  come from different packaging and goals; do not put them in one table or draw one conclusion from both.
- The one allowed crossing: a study may take a prototype **design vector** as a frozen reference input, copied into a
  tracked study file with its provenance (`scripts/study/study2_morphology_adaptation/references_nominal15.yaml`). It is
  then re-evaluated under the study's own rules and packaging, not the prototype's.
- Each report and each output says which of the two it belongs to.

The simulation side consumes geometry exports that carry their source commit. It does not reimplement or import
`design/core`. See `sim/isaaclab/README.md`.

The parametric hand model lives in `design/core/morphology/code_hand`. Every joint coordinate is positive
by the right-hand rule about its local +Z axis (CCW when viewed from +Z toward the origin), and flexion is
positive on every digit: the thumb MCP and IP servos are mounted with their output on the side that makes it
so (the prototype CAD still has them the other way round, a declared deviation in `core/cad/code_hand.py`).
Joint zero is defined geometrically: fingers fully extended, and the thumb's CMC-RAA (CMC-FE), MCP and IP
centres collinear with MCP = IP = 0. The nominal axis-mounting transforms remain part of the geometry.
CAD observations are stored separately as `cad_configuration`; they do not calibrate the model's zero.
Actuator coordinates (`CoDeHand.actuators`) are each servo's output angle from the CAD revolute mate's zero
(`*_actuator_zero_deg`); the PIP servo's is the direction of its link A'-B, not the distal segment's angle.
From `design/`, `python -m visualize.code_hand` draws the geometric zero; `--pose cad` selects the saved CAD pose.

The production PIP servo tilt is fixed at 15 degrees (`finger.pip_servo_tilt_deg` in the hand model,
`goal.hardware.pip_servo.tilt_angle_deg` in the production optimizer). Active Studies 1-2 use a separate
0-90 degree tilt sweep with a 15-degree nominal; earlier fixed-tilt references/results remain historical.
Planar collision checks use actual hardware envelopes with no universal body-clearance allowance.
