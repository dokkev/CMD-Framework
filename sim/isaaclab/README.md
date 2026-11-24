# code_isaaclab

Physics-simulation workspace of the **CoDe Hand** (Contact Morphology Design) project. It will hold contact
experiments, task environments, and later policy learning and evaluation on interchangeable contact interfaces. It is
an Isaac Lab *external project* (an installable package with its own `pyproject.toml` and `uv.lock`). It lives in the
CMD-Framework monorepo at `sim/isaaclab/`. It does not modify Isaac Lab.

## CoDe setup (verified 2026-11-07)

### Repositories and responsibilities

| Repository | Role |
|---|---|
| CMD-Framework (this monorepo, outside `sim/`) | Mechanism geometry and kinematics (crossed four-bar FK), morphology definitions, contact-interface design and analysis; conda `cmd` environment |
| `sim/isaaclab` (this project) | Physics simulation, contact experiments, task environments, later learning and policy evaluation; uv environment |
| `IsaacLab/` beside the monorepo (`../../../IsaacLab` from here) | Upstream NVIDIA Isaac Lab checkout, unmodified, not part of the monorepo |

Integration boundary (`design/scripts/export/code_hand_sim.py` → `scripts/code_hand/make_assets.py`; see
[scripts/code_hand/README.md](scripts/code_hand/README.md)):

```text
CMD-Framework (design/)
    |
    | versioned morphology / FK / servo exports (code_hand_sim/1 JSON, source commit recorded)
    v
code_isaaclab (sim/isaaclab/)
    |
    | URDF / USD robot assets and interchangeable contact interfaces
    v
simulation environments
```

Linkage kinematics stay in CMD-Framework. This project consumes their versioned exports (with the source commit
recorded) and neither reimplements them nor imports CMD-Framework's `core/` directly, which keeps the boundary when both
live in one repository.

### Environment

| Item | Value |
|---|---|
| Isaac Lab | `release/3.0.0`, commit `44044b0963001962abe5701133427e37b003fffe` (`VERSION` 3.0.0; `isaaclab` package 28.0.1) |
| Installation | Official uv workflow: `git clone --branch release/3.0.0`, then `uv run` / `uv sync`. No conda, no Isaac Sim |
| Python | 3.12.12 (uv-managed) |
| PyTorch | 2.11.0+cu130 (CUDA available) |
| Physics backend | Newton 1.6.1, `physics=newton_mjwarp`. Isaac Sim / PhysX not installed (deferred) |
| OS | Ubuntu 24.04.3 LTS, x86_64, kernel 6.17 |
| GPU / driver | NVIDIA GeForce RTX 4070 Ti SUPER (16 GB, compute capability 8.9) / driver 590.48.01 (CUDA 13.1). Requirement: ≥ 580.65.06, Turing or newer |
| RAM | 78 GB |

### Install

Workspace layout: `IsaacLab/` and the monorepo (`cmd_framework/`) side by side.

```bash
git clone --branch release/3.0.0 https://github.com/isaac-sim/IsaacLab.git   # in the workspace, if not present
git -C IsaacLab checkout 44044b0963001962abe5701133427e37b003fffe              # the verified commit (the branch moves)
cd cmd_framework/sim/isaaclab
uv sync --locked   # editable ../../../IsaacLab sources from [tool.uv.sources]; uv.lock pins everything else
```

Run everything with `uv run ...`; no manual activation is needed.

The lock depends on `IsaacLab/` sitting beside the monorepo (`../../../IsaacLab`). `uv.lock` also lists Isaac Sim (`isaacsim*`) packages,
because uv resolves every optional extra. They are installed only with `--extra isaacsim`, never by a default sync.

### Verified smoke tests (2026-11-07)

| Check | Command | Result |
|---|---|---|
| Isaac Lab CLI and task registry | `cd <workspace>/IsaacLab && uv run isaaclab list_envs --show_presets` | 45 unique built-in tasks (139 rows with presets), including `Isaac-Cartpole-Direct` with `physics=newton_mjwarp` as default |
| Newton physics stepping | `cd <workspace>/IsaacLab && uv run isaaclab benchmark runtime --task Isaac-Cartpole-Direct --num_envs 4 --num_steps 128 --benchmark_formatter summary physics=newton_mjwarp` | Completed: 128 iterations × 4 envs, ~4.1k steps/s, 1.28 GB GPU memory, environment setup ~23 s. Headless, no GUI |
| Package import and source paths | `uv run python -c "import code_isaaclab, isaaclab; print(code_isaaclab.__file__, isaaclab.__file__)"` | `src/code_isaaclab` and `<workspace>/IsaacLab/source/isaaclab` (editable) |
| Task discovery from this project | `uv run isaaclab list_envs --show_presets` | Lists `CoDe-Reorient-Cube-Allegro` with `physics=newton_mjwarp` (default) and `physics=newton_vbd` |
| Lock reproducibility | `uv sync --locked` | Lock is consistent with `pyproject.toml` |
| Generated tests | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q` | 1 passed |

GUI rendering, Isaac Sim and PhysX were not tested.

### Troubleshooting (issues seen on this workstation)

- `uv run pytest` fails at collection with `ModuleNotFoundError: No module named 'lark'` from
  `/opt/ros/jazzy/.../launch`. ROS Jazzy's pytest plugins load automatically from the system Python path. Run tests
  with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.

---

## In-hand manipulation example (2026-11-07)

This is a modular in-hand cube reorientation task, built to take a custom hand later. It reuses Isaac Lab's
manager-based reorientation MDP (`isaaclab_tasks.core.reorient`): observations, actions, goal command, rewards,
terminations and reset events. Three parts can be swapped independently:

| Part | Where | Swapped how |
|---|---|---|
| Export & assets | `src/code_isaaclab/assets/`: `Description` (`code_hand_sim/1` JSON reader), `write_urdf`, `write_usd`, `build_newton`; CLI in `scripts/code_hand/make_assets.py` | Export a new JSON from `design/` and run `make_assets.py` (`--colliders`, `--passive-limit-margin`) |
| Closed-loop hooks | `src/code_isaaclab/loops.py`: `stiffen_mujoco_loops` / `stiffen_env_mujoco_loops` (startup) and `FourBarTables` / `sync_four_bar_joints_on_reset` (reset) | Enabled automatically when a `HandSpec` comes from a closed-loop `Description`; no-op for open-chain hands or non-MuJoCo backends |
| Hand | `src/code_isaaclab/hands/`: `HandSpec` and registry (`allegro.py`), plus `code_hand.py` (`code_hand_articulation_cfg`, `make_code_hand_spec`) | Build a `HandSpec` from an exported `Description` + USD via `make_code_hand_spec(...)`, register it, and subclass `HandReorientEnvCfg`. No MDP code changes |
| Physics backend | `src/code_isaaclab/physics.py`: `HandPhysicsCfg` presets `newton_mjwarp` (default, training), `newton_mjwarp_hydro` (hydroelastic contact) and `newton_vbd`; `coupled_vbd_cfg()` for MJWarp + VBD deformable pads (experimental); `solver_physics_cfg()` for single-hand solver sweeps | `physics=<name>` on any command; a new backend is one more field |
| Task | `src/code_isaaclab/tasks/reorient/`: `HandReorientEnvCfg` (hand-agnostic, wires closed-loop events when present), `AllegroReorientEnvCfg`, RSL-RL runner cfg, task id `CoDe-Reorient-Cube-Allegro` | — |
| Verification | `src/code_isaaclab/verification.py` (`RolloutRecorder`, `compare`) and `scripts/rollout_metrics.py` | The metrics read only the `robot` and `object` entities and the `object_pose` command, so they work for any hand |

Train (MuJoCo-Warp) and play:

```bash
uv run --extra rsl-rl isaaclab train --rl_library rsl_rl --task CoDe-Reorient-Cube-Allegro physics=newton_mjwarp
uv run --extra rsl-rl isaaclab play  --rl_library rsl_rl --task CoDe-Reorient-Cube-Allegro physics=newton_mjwarp
```

Backend comparison: the same seeded actions, one process per backend.

```bash
uv run python scripts/rollout_metrics.py --task CoDe-Reorient-Cube-Allegro --policy random --num_envs 8 \
    --max_steps 200 --out logs/verify/random_newton_mjwarp.json physics=newton_mjwarp
uv run python scripts/rollout_metrics.py ... --out logs/verify/random_newton_vbd.json physics=newton_vbd
uv run python scripts/rollout_metrics.py --compare logs/verify/*.json
```

### Morphology O continuous cube rotation

`CoDe-Rotate-Cube-O` uses the O hand, a 40 mm cube, cached stable grasps, and
relative joint targets. The task rewards rotation about world +z. Its actor
uses simulated cube pose and angular velocity alongside joint history.

From `sim/isaaclab`, generate the O grasp cache if one does not exist, then
train a checkpoint:

```bash
uv run python scripts/rotate_cube/gen_grasps.py --hand O physics=newton_mjwarp
uv run python scripts/reorient_o/train.py --task CoDe-Rotate-Cube-O \
    --iterations 1000 --num_envs 4096 --no_video --logger tensorboard
```

To fine-tune a checkpoint, add `--checkpoint <path>`. Each `--iterations` value
counts additional PPO updates. Use `--override env.rewards.<term>.weight=<weight>`
for a reward experiment and `--fresh_optimizer` when changing the optimizer
learning rate. The run saves the resolved environment and agent configs under
`params/` alongside the checkpoints.

The optional `milestones` reward credits new 30° increments while the
cube stays within the configured displacement and tilt bounds at the
crossing. Both rotation measures must reach the increment. Re-crossing a
credited angle pays zero. A weight of 36 gives 1.2 reward per increment
at the default 30 Hz control rate.
With milestones enabled, `timeout_progress` can also reward the total
retained rotation at a clean timeout. This term stays at zero if the
cube crossed either grasp bound at any step.
Its optional `second_half_weight` adds credit for forward rotation in
the second half of the episode, capped by total net forward rotation.

`joint_target_limit_regrasp` is an optional cost on normalized joint targets
near selected actuator limits. `min_progress_rad` and `ramp_progress_rad`
defer and smooth this cost until the policy has made some rotation, so early
grasp acquisition is not penalized. Setting `max_distance_m` and `min_up_dot`
also gates the cost to states where the cube remains inside a chosen grasp
envelope. A gated cost requires the milestones term to be enabled.

For a separately generated evaluation cache, set a new destination and a seed
range disjoint from the training cache:

```bash
CODE_ROTATE_CUBE_GRASPS="$PWD/logs/rotate_cube/heldout_grasps" \
    uv run python scripts/rotate_cube/gen_grasps.py --hand O --target 256 \
    --num_envs 128 --max_rounds 14 --seed_start 10000 physics=newton_mjwarp

uv run --extra rsl-rl python scripts/rotate_cube/eval_policy.py \
    --checkpoint <checkpoint-a.pt> --checkpoint <checkpoint-b.pt> \
    --num_envs 96 --eval_seed 21000 physics=newton_mjwarp \
    env.events.reset_grasp.params.cache="$PWD/logs/rotate_cube/heldout_grasps/O.pt" \
    env.terminations.object_off_grasp.params.max_distance=0.07 \
    env.terminations.object_tilted.params.min_dot=0.0
```

The evaluator records the first episode in each environment, including the
terminal state before reset. It uses the actor mean with the configured
observation noise and reports signed world-z rotation, projected heading
change, and timeout, tilt, and displacement metrics. The stricter 90° and
360° metrics require agreement of both rotation measures, a clean timeout,
at most 40 mm of displacement and at most 60° of tilt from the reset pose.
The command above uses the `tilt90` comparison termination thresholds.
Set the same physics, observation and action settings across compared runs.
To remove observation noise from a paired comparison, add
`env.observations.policy.enable_corruption=false`.
If changing the starting MCP/PIP angles or cube size through the
`CODE_REORIENT_O_*` settings, generate a new cache under a separate
`CODE_ROTATE_CUBE_GRASPS` path and use those same settings at train and eval.

Set `CODE_ROTATE_CUBE_CROWN_MM=1` to lower the ends of each pad's contact
face by 1 mm along its length, with the same center height and mounting
face. For a width-wise crown on the three distal finger pads, set
`CODE_ROTATE_CUBE_CROWN_LAYOUT=distal_width` and a sagitta such as
`CODE_ROTATE_CUBE_CROWN_MM=0.5`. The flat setting is `0`.
For a fixed-policy pad transfer comparison,
reuse the same checkpoint, grasp cache, and evaluation seed in both
conditions. For training on a crowned profile, generate a new cache in a
separate directory with that profile.

The evaluator's `--contact_metrics` option reports contact with the
pad-carrying links. To seed a training curriculum with post-turn states,
evaluate one checkpoint on a training cache and use `--export_grasps`.
The export selects clean timeouts with at least 45° of forward rotation,
the 40 mm and 60° limits respected, low linear speed, and simultaneous
thumb-link and finger-link contact at the final step. Keep the held-out
cache out of the curriculum.

`CODE_ROTATE_CUBE_PHASE_PERIOD_S` optionally appends the sine and cosine
of an episode clock to the policy observation. Use the training period
again at evaluation time. This variant has two extra input features and
requires a compatible checkpoint.

`CODE_ROTATE_CUBE_GOAL_OBS=1` also appends two errors to the next rotation
milestone, using the projected heading and the integrated world-z
rotation. The next target advances when the milestone reward credits the
current target. To warm-start from an otherwise compatible checkpoint,
use `scripts/rotate_cube/extend_observations.py --features 2` and load
the resulting checkpoint with `--fresh_optimizer`.

Pass `--trace path.npz` when evaluating one checkpoint to record net
rotation, heading, displacement, tilt, action activity and normalized
joint targets before reset.

### Verified (2026-11-07)
- **Upstream reference.** `Isaac-Reorient-Cube-Allegro` runs on `newton_mjwarp`: 4 envs, 100 random steps.
- **Project task, stepping.** `CoDe-Reorient-Cube-Allegro` initializes and steps on both presets: 4 envs, 100
  random steps each. VBD solver initialization takes about 16 s; MJWarp takes about 1 s.
- **Training.** Two RSL-RL iterations on `newton_mjwarp` (64 envs) complete and save checkpoints under
  `logs/rsl_rl/code_reorient_allegro/`.
- **Configuration tests.** `uv run pytest`: 4 passed. They cover registration, the hand spec applied to the env
  cfg, both presets present, and MJWarp buffers sized for the hand.
- **Backend comparison.** 8 envs, 200 steps, seed 42:

| metric | zero / mjwarp | zero / vbd | random / mjwarp | random / vbd |
|---|---|---|---|---|
| resets | 0 | 0 | 58 | 0 |
| joint tracking error [rad] | 0.025 | 0.473 | 0.380 | 0.585 |
| object height [m] | 0.554 | 0.558 | 0.552 | 0.558 |
| orientation error [rad] | 1.897 | 1.912 | 2.080 | 1.912 |

**VBD finding and fix (2026-11-08).**
- *The symptom.* The first VBD runs above showed the hand not tracking its targets, with identical metrics
  under zero and random actions.
- *The cause.* It was not VBD. A pure-Newton probe (one revolute joint, drive target 0.8 rad) shows that
  `SolverVBD` reaches the target: `joint_q` reads 0.8000 after `newton.eval_ik`, against 0.0000 as stored.
  VBD integrates body poses (maximal coordinates), and Isaac Lab 3.0 never recomputes joint coordinates
  after a VBD step. Observations and joint-space actions therefore saw reset values.
- *The fix.* `code_isaaclab/vbd_manager.py` (`NewtonVBDJointSyncManager`, selected through
  `VBDSolverCfg.class_type`) runs `eval_ik` after each step. Isaac Lab itself is unchanged.
- *Convergence.* With the fix, joint tracking depends on how far VBD converges:

| `newton_vbd` | Zero-action joint tracking [rad] | Notes |
|---|---|---|
| Before the fix | 0.475 (stale) | Actions had no visible effect |
| 10 iterations × 2 substeps | 0.176 | Zero and random actions now differ |
| 30 iterations × 2 substeps | 0.119 | |
| **30 iterations × 4 substeps (preset)** | **0.057** | MJWarp: 0.029. The cube still drops occasionally under zero action (1 reset in 100 steps on 4 envs; 0 on MJWarp), a real sim-to-sim difference to examine |

Running everything on VBD, rigid and deformable in one solver, is therefore the native path for compliant
pads. It also avoids the coupled-solver problems below. Not yet tested: deformable pads attached to the
fingers inside VBD.

## Contact-model experiments (2026-11-08)

`scripts/contact_experiments.py`: a 4 cm, 0.1 kg rigid block is dropped 6 cm onto an 8×8×2 cm pad, and
the run lasts 1 s at 240 Hz. Only primitive shapes are used. One experiment runs per process:

```bash
uv run python scripts/contact_experiments.py point | pipeline | hydro
uv run --extra tetrahedralization python scripts/contact_experiments.py vbd_coupled [--coupling '{"mass_scale": 10.0}']
```

| Experiment | Physics | Lowest block centre | Peak penetration | Final | Verdict |
|---|---|---|---|---|---|
| `point` | MJWarp point contact (training preset) | 0.0331 m | 6.86 mm | 0.0399 m | Reference |
| `pipeline` | MJWarp + Newton collision pipeline, no hydroelastic (control) | 0.0331 m | 6.86 mm | 0.0399 m | Same as `point` |
| `hydro` | MJWarp + hydroelastic (`physics=newton_mjwarp_hydro`) | 0.0394 m | 0.57 mm | 0.0400 m | **Works**; the change comes from the hydroelastic model, not the pipeline |
| `vbd_coupled` | MJWarp block + VBD deformable pad, proxy coupler (`coupled_vbd_cfg`) | — | — | — | **Not working yet** (below) |

**`vbd_coupled`.** The coupled step runs and stays finite, but the block–pad contact is wrong.

| Setting | Outcome |
|---|---|
| Default | The block is launched upward (z = 3.8 m after 1 s) from about 3 cm above the pad |
| `staggered` mode, Aitken relaxation, `proxy_relaxation` 0.3, 3 coupling iterations, softer `soft_contact_ke` | Same ejection |
| `mass_scale` 10 | No ejection, but the block sinks about 14 mm into the 20 mm pad. The material's elastic estimate is about 0.2 mm, so this is interpenetration, not compression |

The settings follow Isaac Lab's `franka_soft` coupled task, where the proxied bodies are PD-driven robot
links. A free, light rigid body (here the block; in a hand task the cube) is the case that fails.
`coupled_vbd_cfg` is kept as an experimental hook and is **not** a task preset.

**Hydroelastic on the Allegro task.** `physics=newton_mjwarp_hydro` fails at scene build on the Allegro asset.
Its colliders are authored as convex-hull approximations, which Newton replaces with non-mesh geometry, so
SDF/hydroelastic state cannot be kept. Assets built for this project (the CoDe hand from CMD-Framework
exports) should author their colliders hydro-ready: `NewtonSDFCollisionCfg(hydroelastic_enabled=True, ...)`
with no convex-hull approximation. The preset then applies unchanged.

## Study 3 measurement (2026-11-22)

`scripts/study3_measurement/` holds the simulation measurement behind contribution C2.3. It is defined in
`design/SSoT/problem_formulation.md` A.6.4, and its code map is in `design/SSoT/code_formulation.md`. A cylinder is
held by both pads of each CFL redesign, loaded until the grasp gives way, and the change of that limit load under an
interface change is compared with the reference hand O.

```bash
uv run python scripts/study3_measurement/two_pad_wrench.py --hand O --mu 1.3 --condition A --radius_mm 15
python3 scripts/study3_measurement/dose_analysis.py --condition A --set all
uv run python scripts/study3_measurement/invariants.py
```

Every run and analysis is written to `logs/experimental/two_pad/`, where the first runs were written. Analyses pick
the latest run per hand, mu, condition and radius, so a smoke run written there takes part in them.
`scripts/study3/` is the archived learning programme and is not evidence for C2.3. `scripts/experimental/` keeps the
first two separability experiments.

## Generated template notes

An installable downstream Isaac Lab task package with a standard uv `src` layout.

The template's original note ("no tasks registered yet") is superseded by the in-hand example above.


## Installation

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then create the project environment. The default
environment uses the Newton backend and does not install Isaac Sim:

```bash
uv sync
```

This project's `pyproject.toml` uses editable relative paths to an Isaac Lab source checkout. Update
`[tool.uv.sources]` if either directory moves.

The project forwards every optional extra declared by the Isaac Lab package used to create it, including physics,
rendering, visualizer, RL, teleoperation, and development features. Inspect `pyproject.toml` for the complete list and
pass each required extra to `uv run`:

```bash
# Standalone OV PhysX
uv run --extra ovphysx isaaclab random_agent --task <TASK_NAME> physics=ovphysx

# Isaac Sim with PhysX and RTX rendering
uv run --extra isaacsim isaaclab random_agent --task <TASK_NAME> physics=isaacsim_physx

# Isaac Sim with the Rerun visualizer
uv run --extra isaacsim --extra rerun isaaclab random_agent --task <TASK_NAME> --viz rerun
```

The `all` extra forwards Isaac Lab's `all` selection. The `ov` extra installs both the `ovphysx` and `ovrtx` runtimes;
you can also select either independently. Commit `pyproject.toml` and `uv.lock` so collaborators use the same
environment.

## Run the tasks

After registering a task, list it with:

```bash
uv run isaaclab list_envs --show_presets
```

## Project structure

Add task packages under `src/code_isaaclab/tasks`. Import each task package from that directory so its Gymnasium
registrations load through the project's `isaaclab.tasks` entry point.

## Project assets

Keep project-owned USD files and related data under `src/code_isaaclab/assets/data`. The asset module exposes a
stable path that works from an editable checkout and an installed wheel:

```python
from code_isaaclab.assets import CODE_ISAACLAB_ASSETS_DIR

robot_usd_path = CODE_ISAACLAB_ASSETS_DIR / "robots" / "my_robot.usd"
```

Keep asset configuration in the `code_isaaclab.assets` package and pass `str(robot_usd_path)` to configuration fields that
expect a string. USD files are configured for Git LFS by the project's `.gitattributes`; install Git LFS before adding
large binary assets.

## Development

Run the registration test and code-quality checks through the project environment:

```bash
uv run pytest
uv run pre-commit run --all-files
```

The test helpers under `source/isaaclab_tasks/test` in the Isaac Lab repository are not part of the installed
`isaaclab_tasks` package. Keep test fixtures in this project and use public Isaac Lab APIs. If you copy
`env_test_utils.py`, it becomes vendored code whose upstream changes you must track.

To configure VS Code or Cursor, run the `setup_python_env` task or invoke its command directly:

```bash
uv run isaaclab --editor
```

The setup command selects the active interpreter and writes a git-ignored `pyrightconfig.json`. The resulting
configuration inherits the project's checked-in Pyright settings and adds the Isaac Sim extensions, project `src` root,
and any Isaac Lab packages discovered in the active Python environment. This supports both Pylance in VS Code and
basedpyright in Cursor.

In VS Code, use Pylance and select the interpreter that ran the setup command. In Cursor, install the
[basedpyright extension](https://marketplace.visualstudio.com/items?itemName=detachhead.basedpyright) instead of
Pylance, select the same interpreter, and reload the window. Both language servers read `pyrightconfig.json`.

When using the `isaacsim` extra, include it while generating the editor configuration so the command can discover the
Isaac Sim installation:

```bash
uv run --extra isaacsim isaaclab --editor
```

For an Isaac Sim binaries installation that is not available in the project environment, provide its path explicitly:

```bash
# Linux
uv run isaaclab --editor --isaac_path <isaac-sim-path>

# Windows
uv run isaaclab --editor --isaac_path <isaac-sim-path>
```

## Troubleshooting

If Pylance or basedpyright cannot resolve modules, confirm that the selected interpreter matches the one used to run the
setup command, then reload the editor window. To add a missing extension or reduce indexing memory, edit the `extraPaths`
array in the root `pyrightconfig.json`; remove simulator extension directories that the project does not use.
