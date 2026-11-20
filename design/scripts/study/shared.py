"""What Study 1 and Study 2 share: the reference hands, the contact module
catalog, the comparison rules every reference must share, and the run record.

Both studies take their references from one file
(``study2_morphology_adaptation/references.yaml``) and their modules from one
catalog (``study1_fixed_morphology/modules.yaml``), so a reference or a module
means the same in both.
"""

from __future__ import annotations

import csv
import importlib.util
import math
import subprocess
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.analysis.tradeoff import Modules, goal_of, with_goal, with_modules  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

HERE = Path(__file__).resolve().parent
SIDES = ("proximal", "distal")
REFERENCES = "scripts/study/study2_morphology_adaptation/references.yaml"
CATALOG = "scripts/study/study1_fixed_morphology/modules.yaml"
NOMINAL_REFERENCES = "scripts/study/study2_morphology_adaptation/references_nominal15.yaml"


def load_module(name: str, path: Path):
    """A script module by path (the study folders are not packages)."""

    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # before running it: its dataclasses look themselves up there
    spec.loader.exec_module(module)
    return module


partitions = load_module("study_partitions", HERE / "partitions.py")


# The contact modules ---------------------------------------------------------

@dataclass(frozen=True)
class ModulePair:
    module_id: str
    group: str
    proximal_length_mm: float
    proximal_thickness_mm: float
    distal_length_mm: float
    distal_thickness_mm: float

    @property
    def modules(self) -> Modules:
        return Modules(self.proximal_length_mm, self.proximal_thickness_mm, self.distal_length_mm,
                       self.distal_thickness_mm)


def module_catalog(values: dict[str, Any]) -> list[ModulePair]:
    """The catalog's pairs (``modules.yaml``), named ones first, then each sweep's grid."""

    baseline = values["baseline"]

    def pair(module_id: str, group: str, given: dict[str, Any]) -> ModulePair:
        sides = {side: {**baseline[side], **(given.get(side) or {})} for side in SIDES}
        return ModulePair(module_id, group, *(float(sides[side][key]) for side in SIDES
                                              for key in ("length_mm", "thickness_mm")))

    pairs = [pair(str(name), "named", given or {}) for name, given in (values.get("modules") or {}).items()]
    for sweep, grid in (values.get("sweeps") or {}).items():
        for length in grid["length_mm"]:
            for thickness in grid["thickness_mm"]:
                given = {side: {"length_mm": length, "thickness_mm": thickness} for side in grid["sides"]}
                pairs.append(pair(f"{sweep}_L{length:g}_T{thickness:g}", str(sweep), given))
    ids = [p.module_id for p in pairs]
    if len(set(ids)) != len(ids):
        raise ValueError("module ids must be unique")
    for p in pairs:
        if min(p.proximal_length_mm, p.distal_length_mm, p.proximal_thickness_mm, p.distal_thickness_mm) <= 0.0:
            raise ValueError(f"module {p.module_id}: its lengths and thicknesses must be positive")
    return pairs


def catalog(root: Path = REPOSITORY_ROOT) -> dict[str, ModulePair]:
    return {pair.module_id: pair for pair in module_catalog(read_config_file(root / CATALOG))}


# The reference hands -------------------------------------------------------

@dataclass(frozen=True)
class Reference:
    """A reference hand: its design (its depths the baseline module's), mechanism and study config."""

    reference_id: str
    kind: str
    """``matched`` or ``native``."""
    mechanism: str
    x: tuple[float, ...]
    config: Any
    config_path: str
    source: str
    upper_bound: bool = False
    """The idealized RCJ: no roller wall, bearing or cable retention modelled; reported apart."""
    base_reference_id: str = ""
    servo_tilt_deg: float | None = None
    design_flexion_deg: float | None = None
    """The required flexion the reference was designed for (a native one: its
    search's ``q_req``); evaluated at it alone. ``None``: at every one asked."""


def flexions_for(reference: Reference, flexions) -> list[float]:
    """The required flexions ``reference`` is evaluated at: its own design's
    alone (a native reference, problem formulation A.6.1), else ``flexions``."""

    flexions = [float(v) for v in flexions]
    if reference.design_flexion_deg is None:
        return flexions
    return [v for v in flexions if math.isclose(v, reference.design_flexion_deg)]


def load_config(path: str, root: Path = REPOSITORY_ROOT, tilt_deg: float | None = None, overrides=()):
    """A study config file's mechanism and config (its optimizer settings
    aside), its servo's tilt held at ``tilt_deg`` if given (a mechanism file
    searches it; a reference may fix it), and ``overrides`` (``key=value``, a
    reference's ``set``) applied."""

    from core.optimization.linkage.mechanism import resolve_preset

    overrides = tuple(overrides) + (() if tilt_deg is None else
                                    (f"goal.hardware.pip_servo.tilt_angle_deg=[{tilt_deg}, {tilt_deg}]",))
    values = read_config_file(root / path, overrides)
    found = mechanism(values.pop("mechanism"))
    values.pop("output", None)
    resolve_preset(values)
    values, _ = found.split_settings(values)
    return found, found.config_from_mapping(values)


def _native(spec: dict[str, Any], root: Path):
    """A native reference's design: its portfolio's ``seeds.csv`` row by role."""

    from core.optimization.study import load_run

    portfolio = root / spec["portfolio"]
    with (portfolio / "seeds.csv").open(encoding="utf-8", newline="") as stream:
        row = next((r for r in csv.DictReader(stream) if spec["role"] in (r.get("seed_role") or "").split(";")),
                   None)
    if row is None:
        raise ValueError(f"{portfolio}: no seed with role {spec['role']}")
    run = load_run(portfolio.parent.parent / row["run"])
    return run.mechanism.name, tuple(float(v) for v in run.mechanism.read_design(row)), \
        f"{spec['portfolio']}/seeds.csv#{spec['role']}"


def references(values: dict[str, Any], kinds, baseline: Modules, root: Path = REPOSITORY_ROOT) -> list[Reference]:
    """The references of ``kinds``: matched ones built to the tracked
    morphology (``values['morphology']``), native ones as their portfolios hold them."""

    built = []
    if "matched" in kinds:
        morphology = values["morphology"]
        if "x" not in morphology:
            raise ValueError("references.yaml: the matched morphology is a design vector (x), not an output")
        source_mechanism, _ = load_config(morphology["config"], root)
        source_x = tuple(float(v) for v in morphology["x"])
        if source_mechanism.name != morphology["mechanism"]:
            raise ValueError(f"references.yaml: a {morphology['mechanism']} design under a "
                             f"{source_mechanism.name} config")
        if len(source_x) != len(source_mechanism.variable_names):
            raise ValueError(f"references.yaml: {source_mechanism.name} has {len(source_mechanism.variable_names)}"
                             f" design variables, the morphology gives {len(source_x)}")
        for spec in values["matched"]:
            found, config = load_config(spec["config"], root, spec.get("tilt_deg"), spec.get("set", ()))
            part = partitions.partition(found.name)
            carried = with_modules(config, baseline)
            external = partitions.partition(source_mechanism.name).external(source_x, carried)
            coordinates, template = part.template(external, source_mechanism.name, source_x, carried)
            x = part.compose(external, coordinates, template, carried)
            if x is None:
                raise ValueError(f"reference {spec['id']}: no {found.name} design matches references.yaml")
            built.append(Reference(spec["id"], "matched", found.name, x, config, spec["config"],
                                   f"matched to references.yaml#{morphology['id']}",
                                   bool(spec.get("upper_bound", False))))
    if "native" in kinds:
        for spec in values.get("native") or ():
            found, config = load_config(spec["config"], root, spec.get("tilt_deg"), spec.get("set", ()))
            flexion = spec.get("required_flexion_deg")
            if flexion is not None:  # designed for it: its goal asks it
                config = with_required_flexion(config, float(flexion))
            name, x, source = _native(spec, root)
            if name != found.name:
                raise ValueError(f"reference {spec['id']}: a {name} design under a {found.name} config")
            if spec.get("design"):  # the found design with some coordinates set (an actuation fixed after the search)
                names = found.variable_names
                unknown = set(spec["design"]) - set(names)
                if unknown:
                    raise ValueError(f"reference {spec['id']}: no design variables {sorted(unknown)}")
                x = tuple(float(spec["design"].get(n, v)) for n, v in zip(names, x))
                source = f"{source} with {spec['design']}"
            built.append(Reference(spec["id"], "native", name, x, config, spec["config"], source,
                                   bool(spec.get("upper_bound", False)),
                                   design_flexion_deg=None if flexion is None else float(flexion)))
    return built


def load_references(kinds=("matched",), root: Path = REPOSITORY_ROOT,
                    path: str = REFERENCES, tilt_sweep=None) -> list[Reference]:
    found = references(read_config_file(root / path), kinds, catalog(root)["baseline"].modules, root)
    return sweep_references(found, tilt_sweep)


def sweep_references(found: list[Reference], sweep=None) -> list[Reference]:
    """Each tilt is a separate fixed-actuation reference; E/Q are unchanged.

    Explicit base ids avoid sweeping the same architecture twice via its old 0/90-degree references.
    References outside the requested list (e.g. the ideal RCJ) remain separate nominal conditions.
    """
    if not sweep:
        return found
    selected = set(sweep["references"])
    unknown = selected - {r.reference_id for r in found}
    if unknown:
        raise ValueError(f"unknown tilt-sweep references: {sorted(unknown)}")
    angles = tuple(float(v) for v in sweep["angles_deg"])
    if not angles or len(set(angles)) != len(angles) or not all(math.isfinite(v) for v in angles):
        raise ValueError("tilt sweep needs unique finite angles")
    expanded = []
    for ref in found:
        if ref.reference_id not in selected:
            expanded.append(ref)
            continue
        if sweep.get("include_reference"):  # the design at its own tilt too
            expanded.append(ref)
        names = mechanism(ref.mechanism).variable_names
        slot = names.index("servo_tilt_deg")
        for tilt in angles:
            goal = goal_of(ref.config)
            config = with_goal(ref.config, replace(goal, pip_servo_tilt_rad=math.radians(tilt),
                               bounds={**goal.bounds, "servo_tilt_deg": (tilt, tilt)}))
            x = list(ref.x)
            x[slot] = tilt
            expanded.append(replace(ref, reference_id=f"{ref.reference_id}__tilt{tilt:g}",
                                    base_reference_id=ref.reference_id, servo_tilt_deg=tilt,
                                    x=tuple(x), config=config))
    return expanded


# The comparison rules ------------------------------------------------------

ACTUATION_FIELDS = ("pip_servo_tilt_rad", "pulley_radius_mm")
"""Goal fields that are each reference's own actuation (A), not a shared rule:
the servo's tilt and the belt's pulleys."""


SERVO_PLACEMENT_BOUNDS = ("ap_x_mm", "ap_z_mm", "ap_from_a_x_mm", "ap_from_a_z_mm")
"""Search bounds on where the PIP servo's output shaft ``A'`` sits: actuation
(A.6.1), each architecture's own (the 2R-remote keeps it proximal of its joint)."""


PIP_SERVO_FIELDS = ("pip_servo_body", "a_pin_servo_check", "pip_servo_pad_check",
                    "allow_collision_pip_servo_to_joint_a", "allow_collision_pip_servo_to_joint_bp")
"""The linkage packaging's fields that only say whether the PIP servo's body is
there and checked (``linkage_idealized`` turns them off): actuation, not rules."""


def comparison_rules(config) -> dict[str, Any]:
    """What every reference compared must share: its whole goal (the required
    flexion, the finger's length range, the mounting policy and range, the pins,
    servos and pads, the face rules, the constraints, the resolution) but for
    its actuation (:data:`ACTUATION_FIELDS`, and the servo tilt's bounds)."""

    import dataclasses

    goal = goal_of(config)
    rules = {field.name: getattr(goal, field.name) for field in dataclasses.fields(goal)
             if field.name not in ACTUATION_FIELDS}
    # The PIP servo's presence (the idealized CFL) is actuation too.
    packaging = rules["packaging"]
    if any(field.name == "pip_servo_body" for field in dataclasses.fields(packaging)):
        rules["packaging"] = {field.name: getattr(packaging, field.name) for field in dataclasses.fields(packaging)
                              if field.name not in PIP_SERVO_FIELDS}
    rules["bounds"] = {name: tuple(value) for name, value in goal.bounds.items()
                       if name != "servo_tilt_deg" and name not in SERVO_PLACEMENT_BOUNDS}
    if rules.get("relative_bounds"):
        rules["relative_bounds"] = {name: tuple(value) for name, value in rules["relative_bounds"].items()
                                    if name not in SERVO_PLACEMENT_BOUNDS}
    return {name: (dict(value) if hasattr(value, "items") else value) for name, value in rules.items()}


def check_comparable(found: list[Reference]) -> dict[str, Any]:
    """The rules the references share; they must all share them, or nothing is
    compared. References designed for different required flexions are compared
    only among their own (they share every rule but it)."""

    rules = {r.reference_id: comparison_rules(r.config) for r in found}
    if any(r.design_flexion_deg is not None for r in found):
        for rule in rules.values():
            rule.pop("required_rom_rad", None)
    first = next(iter(rules.values()), {})
    differing = {rid: sorted(k for k, v in rule.items() if first.get(k) != v) for rid, rule in rules.items()}
    differing = {rid: d for rid, d in differing.items() if d}
    if differing:
        raise ValueError(f"references under different comparison rules: {differing}")
    return {name: repr(value) if not isinstance(value, (int, float, str, bool, type(None), list, tuple, dict))
            else value for name, value in first.items()}


def with_goal_overrides(found: list[Reference], overrides: dict[str, Any] | None) -> list[Reference]:
    """Every reference with the same goal fields set (a sensitivity run's
    ``goal_overrides``, e.g. ``include_linkage_occlusion: true``); unchanged without
    any. A mapping sets fields of a nested dataclass (``packaging:
    {a_pivot_diameter_mm: 8}``). Check the result with :func:`check_comparable`:
    the overrides must leave every reference under one rule set."""

    if not overrides:
        return list(found)
    import dataclasses

    from core.optimization.analysis.tradeoff import with_goal

    def updated(value, changes):
        fields = {}
        for name, change in changes.items():
            current = getattr(value, name)
            fields[name] = (updated(current, change) if isinstance(change, dict) and dataclasses.is_dataclass(current)
                            else change)
        return dataclasses.replace(value, **fields)

    return [dataclasses.replace(r, config=with_goal(r.config, updated(goal_of(r.config), overrides))) for r in found]


def span_floor(reference_external, modules: Modules, config) -> float:
    """The least ``eps_d`` a module forces on a reference (normalized by its
    finger's length): the spans it needs past the reference's, the proximal one
    from the pad's start (``|OA''| >= start + l_p``), the distal one from the
    tip (``|B''C| >= l_d``). No mechanism adapts below it; the differences
    between mechanisms are above it."""

    start = goal_of(config).packaging.proximal_pad_start_mm
    need = max(0.0, start + modules.proximal_length_mm - reference_external["proximal_span_mm"],
               modules.distal_length_mm - reference_external["distal_span_mm"])
    return need / reference_external["finger_length_mm"]


def with_required_flexion(config, flexion_deg: float):
    """``config`` asking ``flexion_deg`` (where the module must be usable and the faces are counted)."""

    import dataclasses

    return with_goal(config, dataclasses.replace(goal_of(config), required_rom_rad=math.radians(flexion_deg)))


# The run record ------------------------------------------------------------

SOURCE_PATHS = ("core", "scripts/study", "visualize")
"""The code and study files a run depends on."""


def git_commit(output: Path | None = None) -> dict[str, Any]:
    """The source commit; with ``output`` and a dirty tree, the uncommitted
    changes saved with the results (``source.patch`` against the commit, the
    untracked files under ``source_untracked/``), so the run's code is recoverable."""

    import shutil

    def git(*args: str) -> str:
        return subprocess.run(("git", *args), cwd=REPOSITORY_ROOT, capture_output=True, text=True).stdout

    record: dict[str, Any] = {"commit": git("rev-parse", "HEAD").strip() or None,
                              "dirty": bool(git("status", "--porcelain", "--", *SOURCE_PATHS).strip())}
    if output is not None and record["dirty"]:
        (output / "source.patch").write_text(git("diff", "HEAD", "--binary", "--", *SOURCE_PATHS), encoding="utf-8")
        untracked = [line for line in git("ls-files", "--others", "--exclude-standard", "--", *SOURCE_PATHS)
                     .splitlines() if line]
        for name in untracked:
            target = output / "source_untracked" / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(REPOSITORY_ROOT / name, target)
        record.update(patch="source.patch", untracked=untracked)
    return record


def formulations(found: list[Reference]) -> dict[str, str]:
    return {r.mechanism: mechanism(r.mechanism).formulation for r in found}


def write_csv(path: Path, rows: list[dict]) -> None:
    columns = list(dict.fromkeys(name for row in rows for name in row))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


__all__ = ["CATALOG", "REFERENCES", "ModulePair", "Reference", "catalog", "check_comparable", "comparison_rules",
           "flexions_for", "formulations", "git_commit", "span_floor", "load_config", "load_module", "load_references", "module_catalog",
           "partitions", "references", "with_required_flexion", "write_csv"]
