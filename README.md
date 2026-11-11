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
