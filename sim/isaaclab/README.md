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

Future integration boundary. This is architecture only; there are no exporters or importers yet:

```text
CMD-Framework
    |
    | versioned morphology / FK / geometry exports
    v
code_isaaclab
    |
    | robot assets and interchangeable contact interfaces
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
| Task discovery from this project | `uv run isaaclab list_envs` | Runs; empty table as expected (Blank template, no CoDe task registered yet) |
| Lock reproducibility | `uv sync --locked` | Lock is consistent with `pyproject.toml` |
| Generated tests | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q` | 1 passed |

GUI rendering, Isaac Sim and PhysX were not tested.

### Troubleshooting (issues seen on this workstation)

- `uv run pytest` fails at collection with `ModuleNotFoundError: No module named 'lark'` from
  `/opt/ros/jazzy/.../launch`. ROS Jazzy's pytest plugins load automatically from the system Python path. Run tests
  with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.

---

## Generated template notes

An installable downstream Isaac Lab task package with a standard uv `src` layout.

No tasks are registered yet. Add task packages under `src/code_isaaclab/tasks` and register them with Gymnasium.


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
