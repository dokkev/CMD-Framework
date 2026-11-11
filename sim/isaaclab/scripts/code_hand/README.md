# CoDe hand loop closure in Newton and Isaac Lab

Can the CoDe hand's closed four-bar run in Newton and Isaac Lab, built from the parametric model, and on which
asset path? These are feasibility checks of the kinematic closure. They are not dynamics or contact validation:
masses, inertias and the direct-drive joint limits are placeholders, and there are no colliders.

## Pipeline

```text
design/core/morphology/code_hand  (parametric model)
   │  design/scripts/export/code_hand_sim.py      → design/output/sim/code_hand.json
   ▼
code_hand.json  (bodies, tree joints, loop cuts, four-bar tables, actuator zeros; m / deg / kg; source commit)
   │  scripts/code_hand/make_assets.py           → code_hand.urdf, code_hand_{ball,revolute}.usda
   ▼
Newton ModelBuilder / add_urdf / add_usd   →   Isaac Lab Articulation (UsdFileCfg, Newton MuJoCo-Warp)
```

Each finger is a spanning tree plus one cut:

```text
palm ─aa→ mcp ─fe→ proximal ─pip_servo (A', actuated)→ servo_link ─b (B)→ distal
                   proximal ─a (A)→ driver_link
                   driver_link ⇢ distal : loop cut at B' (outside the articulation)
```

The tree has no PIP joint: the PIP angle is the distal segment's rotation relative to the proximal one,
`pip_servo + b`, because all four-bar axes are parallel. The policy-facing actuators are the 13 servos (3 per
finger and 4 on the thumb). `a` and `b` are passive.

## Run

```bash
cd design && python scripts/export/code_hand_sim.py                       # cmd env
cd sim/isaaclab
uv run python scripts/code_hand/make_assets.py && uv run python scripts/code_hand/make_assets.py --loop revolute
uv run python scripts/code_hand/e1_newton_loop.py [--loop ball|revolute|none] [--gravity] [--eq-timeconst T --eq-solimp D0 D1]
uv run python scripts/code_hand/e2_import_paths.py --source builder|urdf|usd-ball|usd-revolute [--armature 1e-5] [...]
uv run python scripts/code_hand/e3_isaaclab.py [--usd ...] [--num-envs 64] [--gravity] [--eq-timeconst T --eq-solimp D0 D1]
```

Each experiment ramps every PIP servo target from 0 to 45 deg (PIP 0 to about 107 deg) and lets it settle 0.4 s
per step. It reports the worst B' residual (the distance between the two bodies' copies of the cut point), and the
PIP measured from the bodies against the exported closure table at the measured servo angle. The table has a 1 deg
step, so about 1e-3 deg of the PIP error is interpolation.

## Results (2026-11-11, export from 43ee22c, Newton 1.6.1, Isaac Lab 3.0, MuJoCo-Warp)

| Path | Loop | Gravity | Equality (solref, solimp) | Armature | B' residual | PIP error |
|---|---|---|---|---|---|---|
| E1 builder | ball (1 CONNECT) | off | default (0.02, 0.9/0.95) | 1e-5 | 0.17 µm | 0.001° |
| E1 builder | revolute (2 CONNECT) | off | default | 1e-5 | 0.02 µm | 0.0001° |
| E1 builder | none | on | – | 1e-5 | 28.7 mm | 69.6° (distal falls: loop needed) |
| E1 builder | ball | on | default | 1e-5 | 0.33 mm | 2.30° |
| E1 builder | ball | on | 0.004, 0.99/0.999 | 1e-5 | 1.4 µm | 0.010° |
| E1 builder | revolute | on | 0.004, 0.99/0.999 | 1e-5 | 0.7 µm | 0.005° |
| E2 add_urdf + loader cut | ball | off | default | 0 | 0.87 mm | 4.35° |
| E2 add_urdf + loader cut | ball | off | default | 1e-5 | 0.16 µm | 0.001° |
| E2 add_usd | ball | off | default | 0 | 0.87 mm | 4.37° |
| E2 add_usd | ball | off | default | 1e-5 | 0.13 µm | 0.001° |
| E2 add_usd | revolute | off | default | 1e-5 | 0.10 µm | 0.0002° |
| E3 Isaac Lab, 64 envs | ball | off | default | 1e-5 | 0.31 µm | 0.002° |
| E3 Isaac Lab, 64 envs | ball | on | default | 1e-5 | 0.33 mm | 2.30° |
| E3 Isaac Lab, 64 envs | ball | on | 0.0084, 0.99/0.999 | 1e-5 | 6.0 µm | 0.043° |
| E3 Isaac Lab, 64 envs | revolute | on | 0.0084, 0.99/0.999 | 1e-5 | 2.9 µm | 0.021° |

Isaac Lab sees 19 joints (the cuts do not leak into `joint_names`) and 20 bodies. MuJoCo has nv 19 and one CONNECT
per ball cut (two per revolute cut), replicated per environment. The servo angles agree across environments to
4e-5 deg.

## Findings

- **The open-tree-plus-cut approach (as in lumohand) works directly.** Newton turns any joint outside the
  articulation into MuJoCo equality constraints. USD marks it with `physics:excludeFromArticulation`, and Isaac Lab's
  Newton backend loads that unchanged. No custom closure solver is needed in the simulation loop.
- **Isaac Lab can only take USD here.** Its URDF and MJCF converters need Isaac Sim (`isaacsim`), which is not
  installed. Newton's own `add_urdf` loads the tree, and a loader can close the cuts from the URDF marker frames
  (E2), but that path stays outside Isaac Lab. USD is written directly with `pxr`.
- **Armature is required.** Without it the light links (3 g, inertia about 1e-8 kg m²) let the soft constraint open
  by 0.9 mm in transients. In Isaac Lab, give every tree joint, passive ones included, an `armature` through
  `ImplicitActuatorCfg` (here a stiffness-0 "passive" group).
- **The default equality is too soft under load.** B-B' is short (9.3 mm), so 0.3 mm of constraint stretch is
  2.3 deg of PIP. A stiffer `solref`/`solimp` brings this to hundredths of a degree. In Isaac Lab it is set on
  `solver.mjw_model.eq_solref/eq_solimp` after `sim.reset()`. This uses the solver's internals (no config field
  found yet), and `solref[0]` must stay at least 2x the solver step (here 2x(1/480) s).
- **Revolute vs ball cut.** The revolute cut (two CONNECTs, redundant for a planar loop) was 2-8x tighter than
  the ball cut in every condition, at twice the equality rows. Both are usable once the equality is stiffened.

## Not tested yet

Colliders and self-collision filtering between the loop's links, contact with an object, the thumb's and
fingers' real masses and inertias, actuator effort and velocity limits (X330), mesh visuals, and a second
simulator (Study D).
