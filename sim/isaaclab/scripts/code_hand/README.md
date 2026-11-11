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

## Backends (E4: standalone Newton, E5: Isaac Lab)

```bash
uv run python scripts/code_hand/e4_backends.py --solver mujoco|kamino|xpbd|featherstone|semi_implicit|vbd [--gravity] [...]
uv run python scripts/code_hand/e5_isaaclab_backends.py --physics mjwarp|mjwarp_stiff|kamino|kamino_dvi|xpbd|vbd|featherstone \
    [--gravity] [--actuator implicit|explicit] [--servo-armature 5e-3 --kd 0.2]
```

The same USD (ball cut) and the same body-pose measurements on every backend. E5 adds `joint_pos` (what a policy
observes) against the body poses, a reset to PIP 60 deg written either as the whole closed tree or as the servo
joint only, and throughput (256 environments, RTX 4070 Ti SUPER, 1/240 s with 2 substeps).

E5, gravity on, implicit PD, servo armature 1e-5:

| Backend | Loop | PIP err (ramp) | Servo track | joint_pos vs bodies | Reset, closed tree | Reset, servo only (1 step) | env-steps/s |
|---|---|---|---|---|---|---|---|
| MuJoCo-Warp, default equality | CONNECT, soft | 2.30° | 0.071° | 0.000° | 0.011°, settles to 1.05° | 33.9° | 225k |
| MuJoCo-Warp, stiff equality | CONNECT | 0.011° | 0.072° | 0.000° | 0.012° → 0.004° | 23.3° | 223k |
| Kamino PADMM (`use_fk_solver`) | hard bilateral | 0.045° | 0.072° | 0.000° | 0.001° → 0.023° | 28.9° | 15k |
| Kamino DVI, defaults | hard bilateral | 17.6° (fails) | 35.7° | 0.000° | – | – | 4.5k |
| XPBD, 30 iterations | position-based | 0.53° | 1.80° | **62° (stale)** | 0.09° | 4.0° | 176k |
| VBD + joint sync manager | – | 0.20° | 0.29° | 0.000° | 0.005° → 0.12° | 30.4° | 95k |
| Featherstone | not supported | non-finite | – | – | – | – | – |

Standalone (E4) agrees: Featherstone ignores the cut (17 mm residual with substeps), semi-implicit diverges, VBD
needs `ModelBuilder.color()`.

Servo model (E5, gravity on, servo armature 5e-3, kd 0.2):

| Backend | Implicit PD: servo track / PIP err | Explicit PD (`IdealPDActuatorCfg`): servo track / PIP err |
|---|---|---|
| MuJoCo-Warp, stiff | 0.0720° / 0.0024° | 0.0719° / 0.0025° |
| Kamino PADMM | 0.0720° / 0.048° | 0.0722° / 0.049° |
| XPBD | 5.94° / 0.46° | diverges |
| VBD + sync | 0.27° / 0.31° | diverges |

With armature 1e-5 the explicit PD diverges on every backend: 5 N m/rad on about 1e-5 kg m^2 is roughly
700 rad/s, too fast for an explicit update at 1/240 s. A geared X330 (323:1) has a much larger reflected rotor
inertia, so a realistic armature is part of the servo model, not a numerical tweak. Its value is a placeholder
until identified.

Findings:
- One asset serves every backend that supports loops (MuJoCo-Warp, Kamino, XPBD, VBD). Featherstone cannot close
  the loop and must be rejected, not silently run open.
- What differs per backend is the loop's numerics and the joint-state plumbing, not the asset. MuJoCo-Warp needs its
  loop equality stiffened (no config field: set on the solver after it is built). Kamino closes the loop hard and
  has its own loop-aware FK on reset, at about 15x the cost. XPBD leaves `joint_pos` stale (as VBD did before the
  sync manager).
- Only MuJoCo-Warp and Kamino give the same servo response, with either actuator model. Between them the explicit
  PD makes the actuator law identical by construction. XPBD and VBD implement the drive differently and do not
  follow an explicit torque, so a cross-backend difference there mixes actuator and contact effects.
- A reset must write the whole closed tree. Writing only the servo joints leaves A and B open, and every backend,
  Kamino's FK included (the passive joints are still read as inputs), starts with a 2-10 mm gap that snaps shut.

## XM335 servo model (E6)

`design/core/hardware/xm335.py` holds the XM335-T323-T datasheet (ROBOTIS e-manual; source commits in the module)
and the drive law the export's `servo` block carries:

```text
tau = tau_stall(V) * (clip(K * e, -1, 1) - qd / w_noload(V))        at 11.1 V: tau_stall 1.03 N m, w_noload 5.55 rad/s
K   = (KPP(TBL) / 128) pulse->PWM / 885, default KPP 2000           -> kp 11.85 N m/rad, saturates at a 4.98 deg error
                                                                       kd (back-EMF) 0.186 N m s/rad
```

The vendor's three voltage points are linear in voltage to within 2 %, which supports the DC-motor line. The law
is exactly Isaac Lab's `DCMotorCfg` (PD with kp, kd, clipped to the four-quadrant N-T envelope; the sim test pins
this to 1e-6 N m), which Isaac Lab runs as a Newton-native actuator. `code_isaaclab.actuators.xm335_actuator_cfg`
builds it from the export (`"dc_motor"`) or as its implicit linearisation (`"implicit"`).

```bash
uv run python scripts/code_hand/e6_servo_steps.py --physics mjwarp_stiff|kamino --servo dc_motor|implicit --gravity
```

E6, gravity on, armature 5e-3 (the earlier placeholder):

| Backend / servo | FE +2 deg: peak speed, t90, overshoot | FE +60 deg: peak speed (% no-load), t90 | PIP servo +40 deg: peak speed, t90 |
|---|---|---|---|
| MuJoCo-Warp / dc_motor | 1.10 rad/s, 33 ms, 35 % | 5.63 rad/s (101 %), 188 ms | 5.54 rad/s (100 %), 133 ms |
| Kamino / dc_motor | 1.10 rad/s, 33 ms, 35 % | 5.63 rad/s (101 %), 188 ms | 5.54 rad/s (100 %), 133 ms |
| MuJoCo-Warp / implicit | 1.02 rad/s, 33 ms, 34 % | 16.8 rad/s (303 %), 96 ms | 13.1 rad/s (236 %), 79 ms |
| Kamino / implicit | 0.97 rad/s, 38 ms, 29 % | 24.2 rad/s (436 %), 67 ms | non-finite |

- The dc_motor servo is backend-independent: MuJoCo-Warp and Kamino agree to the reported digits. Saturated moves
  run at the no-load speed, as the datasheet's N-T curve requires (101 % on the FE step, which gravity assists).
- The implicit linearisation agrees for small moves but lets large moves run 2-4x faster than the motor can, and it
  differs between backends. Use it only for small-motion tasks, and say so.
- The table above used a 5e-3 placeholder. The armature is now back-estimated from the datasheet and the case
  size as 3.1e-3 (range 2.9e-3 to 4.0e-3; a maxon DCX 12 S frame, method in `core/hardware/xm335.py`
  `ARMATURE_ESTIMATE`). At 3.1e-3 / 4.0e-3 (MuJoCo-Warp, dc_motor, gravity on): FE +2 deg overshoot 24 % / 30 %,
  t90 29 ms; FE +60 deg t90 179 / 183 ms; PIP +40 deg t90 125 / 129 ms; saturated moves at the no-load speed.
- The armature and the PID units remain to be identified on a real XM335 (a pendulum or no-load step, and a
  static stiffness test) before small-motion timing is interpreted.

## Not tested yet

Colliders and self-collision filtering between the loop's links, contact with an object, the thumb's and
fingers' real masses and inertias, the XM335's identified armature, PID units and friction, mesh visuals, Kamino's
constraint-stabilisation and FK actuation flags, and a simulator outside Newton (Study D).
