#!/usr/bin/env python3
"""Contact accommodation study: frozen cores carrying contact module pairs.

    python scripts/study/study1_fixed_morphology/run.py [config.yaml]

Stage A: the cores (``cores.yaml``), each a finished design held as it is.
Stage B: every module pair of the catalog (``modules.yaml``) on every core,
only the modules' lengths and thicknesses changing
(``tradeoff.with_modules``; the design vector, so the skeleton, joints,
actuator, supports and mounting depths, untouched).
Stage C: each assembly through each required flexion
(``contact_envelope.evaluate_fit``), its maximum safe flexion
(``contact_envelope.max_flexion_deg``) and, when it fits, its effective
contact coverage at that flexion (the mechanism's own evaluation).

A failed assembly is reported, never redesigned. Writes ``manifest.json``,
``core_designs.csv``, ``module_catalog.csv`` and ``assembly_results.csv``
to the config's ``output``.
"""

from __future__ import annotations

import copy
import csv
import dataclasses
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.analysis.contact_envelope import (  # noqa: E402
    COLLISION_POLICY,
    FitResult,
    evaluate_fit,
    layout_summary,
    max_flexion_deg,
    motion,
)
from core.optimization.analysis.tradeoff import Modules, goal_of, with_modules  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from core.optimization.mounting import TOLERANCE_MM, mounting  # noqa: E402
from core.optimization.study import load_run, load_yaml  # noqa: E402

DEFAULT_CONFIG = Path(__file__).resolve().parent / "config.yaml"
DIGITS = 6
"""Results are rounded to this many decimals (equal inputs, equal files)."""
SIDES = ("proximal", "distal")
UNMOUNTED = ("support_exceeded", "invalid_fixed_layout", "mount_incompatible")
"""Failures before any motion: the module does not mount, the core cannot be
built, or the core's mounting surfaces cannot carry it (:func:`compatible`)."""


@dataclass(frozen=True)
class CoreDesign:
    """A frozen core: one design and the config it was found under."""

    core_id: str
    mechanism: str
    x: tuple[float, ...]
    config: Any
    formulation: str | None
    source: str
    role: str | None = None
    source_commit: str | None = None
    source_run: str | None = None
    """The run the design came from (from the repository root), and its row there."""
    candidate: str | None = None


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


# Stage A: the cores ---------------------------------------------------------

def _row(path: Path, column: str, value: str, joined: bool = False) -> dict[str, str]:
    """The first row whose ``column`` is ``value`` (``joined``: one of its
    ``;``-joined values, as a seed picked for two roles, ``most_pad;balanced``)."""

    with path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            found = row.get(column) or ""
            if value in (found.split(";") if joined else (found,)):
                return row
    raise ValueError(f"{path}: no row with {column} = {value}")


def load_core(spec: dict[str, Any], root: Path = REPOSITORY_ROOT) -> CoreDesign:
    """A core from its ``cores.yaml`` entry (see there)."""

    core_id = str(spec["id"])
    if "portfolio" in spec or "run" in spec:
        if "portfolio" in spec:
            portfolio = root / spec["portfolio"]
            row = _row(portfolio / "seeds.csv", "seed_role", str(spec["role"]), joined=True)
            run = load_run(portfolio.parent.parent / row["run"])
            source = f"{spec['portfolio']}/seeds.csv#{spec['role']}"
        else:
            run = load_run(root / spec["run"])
            row = _row(run.directory / "pareto.csv", "candidate", str(spec["candidate"]))
            source = f"{spec['run']}/pareto.csv#{spec['candidate']}"
        found = run.mechanism
        directory = run.directory.resolve()
        source_run = str(directory.relative_to(root.resolve())) if directory.is_relative_to(root.resolve()) \
            else str(directory)
        core = CoreDesign(core_id, found.name, tuple(float(v) for v in found.read_design(row)), run.config,
                          run.formulation, source, spec.get("role"), run.metadata.get("source_git_commit"),
                          source_run, str(row["candidate"]))
    elif "x" in spec:
        found = mechanism(spec["mechanism"])
        config, _ = load_yaml(found, root / spec["config"], tuple(spec.get("overrides", ())))
        core = CoreDesign(core_id, found.name, tuple(float(v) for v in spec["x"]), config, found.formulation,
                          f"{spec['config']} (design vector given)", spec.get("role"))
    else:
        raise ValueError(f"core {core_id}: give portfolio + role, run + candidate, or config + x")
    if spec.get("mechanism") not in (None, core.mechanism):
        raise ValueError(f"core {core_id}: its source is a {core.mechanism} design, not {spec['mechanism']}")
    return core


def distinct(cores: list[CoreDesign]) -> tuple[list[CoreDesign], dict[str, str]]:
    """The cores without repeats (two roles picking one design, say): the first
    kept, each repeat's id mapped to it."""

    kept, seen, duplicates = [], {}, {}
    for core in cores:
        key = (core.mechanism, core.x)
        if key in seen:
            duplicates[core.core_id] = seen[key]
        else:
            seen[key] = core.core_id
            kept.append(core)
    return kept, duplicates


def physical_rules(core: CoreDesign) -> dict[str, Any]:
    """What every core must share: the packaging (servos, pins, clearances,
    which pairs are checked), less the pads the modules set."""

    goal = goal_of(core.config)
    rules = dataclasses.asdict(goal.packaging)
    for side in SIDES:
        rules.pop(f"{side}_pad", None)
    rules["mount_policy"] = goal.mount_policy
    rules["align_mount_offsets"] = goal.align_mount_offsets
    rules["mount_offset_range_mm"] = [list(goal.bounds[f"{side}_mount_offset_mm"]) for side in SIDES]
    rules["resolution_mm"] = goal.resolution_mm
    # What the collision checks include: the bars as capsules, the MCP servo's poses.
    rules["link_radius_mm"] = goal.link_radius_mm
    rules["mcp_samples_rad"] = [float(value) for value in goal.mcp_samples_rad]
    return json.loads(json.dumps(rules, default=str))


def collision_policy(link_radius_mm: float | None) -> dict[str, Any]:
    """``COLLISION_POLICY`` as these cores are checked: the links and bars
    only as capsules when ``link_radius_mm`` is set (else not checked)."""

    policy = copy.deepcopy(COLLISION_POLICY)
    links = next(item for item in policy["checked_in_xz_projection"] if "link_radius_mm" in item)
    if link_radius_mm is None:
        policy["checked_in_xz_projection"].remove(links)
        policy["not_checked"]["module - links and bars of the other body"] = (
            "goal.collision.param.link_radius_mm unset, for CoDe and the 2R-DD alike")
    else:
        policy["checked_in_xz_projection"][policy["checked_in_xz_projection"].index(links)] = \
            links.replace("link_radius_mm", f"link_radius_mm = {link_radius_mm:g}")
    return policy


def check_equivalent(cores: list[CoreDesign]) -> dict[str, Any]:
    """The physical rules every core is checked under; ``ValueError`` when two differ."""

    reference = physical_rules(cores[0])
    for core in cores[1:]:
        rules = physical_rules(core)
        differing = sorted(name for name in reference.keys() | rules.keys() if reference.get(name) != rules.get(name))
        if differing:
            raise ValueError(f"cores {cores[0].core_id} and {core.core_id} are checked under different physical "
                             f"rules ({', '.join(differing)}): not comparable")
    return reference


def summary(core: CoreDesign, reference: Modules) -> dict[str, Any]:
    """The core as it is audited (``layout_summary``): its joints, actuator,
    supports and mounting depths, which no module changes (``reference`` is
    any pair, the summary needs one)."""

    return layout_summary(core.mechanism, core.x, core.config, reference)


def mounting_depths(core: CoreDesign, reference: Modules) -> tuple[float, float]:
    """The core's mounting depths below its reference lines (mm): its own, whatever the module."""

    drops = summary(core, reference)["pad_drops_mm"]
    return float(drops["proximal"]), float(drops["distal"])


# Stage B: the module catalog ------------------------------------------------

def module_catalog(values: dict[str, Any]) -> list[ModulePair]:
    """The catalog's pairs, named ones first, then each sweep's grid."""

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


# Stage C: the assemblies ----------------------------------------------------

def with_required_flexion(config, flexion_deg: float):
    """``config`` with its required flexion ``flexion_deg`` (where the faces' crossing is counted)."""

    goal = dataclasses.replace(goal_of(config), required_rom_rad=math.radians(flexion_deg))
    return dataclasses.replace(config, goal=goal) if hasattr(config, "goal") else goal


def coverage(core: CoreDesign, pair: ModulePair, flexion_deg: float) -> tuple[float, float, float]:
    """The pads' effective lengths at ``flexion_deg`` and their share of the
    extended finger, as the mechanism's evaluation counts them."""

    found = mechanism(core.mechanism)
    evaluation = found.evaluate(core.x, with_modules(with_required_flexion(core.config, flexion_deg), pair.modules))
    if evaluation is None:
        return math.nan, math.nan, math.nan
    proximal, distal = evaluation.proximal_effective_mm, evaluation.distal_effective_mm
    return proximal, distal, (proximal + distal) / evaluation.finger_length_mm


def _round(value):
    if isinstance(value, float):
        return "" if math.isnan(value) else round(value, DIGITS)
    return "" if value is None else value


def assemblies(cores: list[CoreDesign], catalog: list[ModulePair], flexions_deg, limit_deg: float) -> list[dict]:
    """One record per core, module pair and required flexion."""

    records, reference = [], catalog[0].modules
    for core in cores:
        depths = mounting_depths(core, reference)
        finger_length = summary(core, reference)["finger_length_mm"]
        try:  # the core's motion, which no module changes, once per flexion
            motions = {deg: motion(core.mechanism, core.x, core.config, deg) for deg in (*flexions_deg, limit_deg)}
        except ValueError:  # no closed four-bar: evaluate_fit reports it
            motions = {}
        for pair in catalog:
            # The module on the frozen core's mounting surfaces: each deep enough for
            # it, and still on the hardware that carries it there. One that does not
            # mount at all says so first.
            found = _mounting(core, pair)
            incompatible = found is not None and not compatible(found, depths)

            def remounted(fit: FitResult) -> FitResult:
                return FitResult(False, "mount_incompatible") if incompatible and fit.reason not in UNMOUNTED else fit

            reach, reach_fit = max_flexion_deg(core.mechanism, core.x, core.config, pair.modules, limit_deg,
                                               motions.get(limit_deg))
            reach_fit = remounted(reach_fit)
            if reach_fit.reason in UNMOUNTED:  # no motion to speak of: not 0 deg
                reach = math.nan
            for deg in flexions_deg:
                fit = remounted(evaluate_fit(core.mechanism, core.x, core.config, pair.modules, deg,
                                             motions.get(deg)))
                proximal, distal, share = coverage(core, pair, deg) if fit.feasible else (math.nan,) * 3
                records.append({
                    "assembly_id": f"{core.core_id}__{pair.module_id}__phi{deg:g}",
                    "core_id": core.core_id, "mechanism": core.mechanism, "role": core.role,
                    "source": core.source, "formulation": core.formulation,
                    "module_id": pair.module_id, "module_group": pair.group,
                    "proximal_length_mm": pair.proximal_length_mm,
                    "proximal_thickness_mm": pair.proximal_thickness_mm,
                    "distal_length_mm": pair.distal_length_mm, "distal_thickness_mm": pair.distal_thickness_mm,
                    "proximal_mount_depth_mm": depths[0], "distal_mount_depth_mm": depths[1],
                    "proximal_mount_case": "" if found is None else found.cases[0],
                    "distal_mount_case": "" if found is None else found.cases[1],
                    "proximal_required_depth_mm": None if found is None else found.depths_mm[0],
                    "distal_required_depth_mm": None if found is None else found.depths_mm[1],
                    "proximal_native_support_mm": None if found is None else found.native_mm[0],
                    "distal_native_support_mm": None if found is None else found.native_mm[1],
                    "proximal_contact_depth_mm": depths[0] + pair.proximal_thickness_mm,
                    "distal_contact_depth_mm": depths[1] + pair.distal_thickness_mm,
                    "finger_length_mm": float(finger_length),
                    "required_flexion_deg": float(deg),
                    "mounted": fit.reason not in UNMOUNTED,
                    "feasible": fit.feasible, "rom_preserved": fit.feasible, "reason": fit.reason,
                    "collision_pair": fit.collision_pair, "first_failure_deg": fit.first_failure_deg,
                    "max_safe_flexion_deg": float(reach), "max_flexion_reason": reach_fit.reason,
                    "proximal_effective_mm": proximal, "distal_effective_mm": distal, "contact_coverage": share,
                })
    return [{key: _round(value) for key, value in record.items()} for record in records]


def compatible(found, depths: tuple[float, float]) -> bool:
    """Whether a module goes on the core's frozen mounting surfaces: for each
    pad, the least depth it needs at most the core's (``d_req <= d_core``),
    and the core's within what the hardware it runs under supports there
    (``d_core <= d_native``; a surface on the line needs none). The collisions
    at the core's depths are then the motion's to check."""

    for required, native, depth in zip(found.depths_mm, found.native_mm, depths):
        if required is None or required > depth + TOLERANCE_MM:
            return False  # it needs to hang deeper than the core's surface
        if depth > TOLERANCE_MM and depth > native + TOLERANCE_MM:
            return False  # the surface, off the line, over nothing that carries it there
    return True


def _mounting(core: CoreDesign, pair: ModulePair):
    """The mounting policy's depths for ``pair`` on ``core`` (``None``: the
    core's depths are searched, not set by a policy)."""

    config = with_modules(core.config, pair.modules)
    if goal_of(config).mount_policy == "searched":
        return None
    if core.mechanism == "linkage":
        from core.morphology.mechanism.linkage.design import decode

        try:
            decode(core.x, config.link_length_resolution_mm)
        except ValueError:  # no closed four-bar: evaluate_fit reports it
            return None
    return mounting(core.mechanism, core.x, config)  # another mechanism: no policy, an error


# Output -----------------------------------------------------------------------

def core_rows(cores: list[CoreDesign], reference: Modules) -> list[dict]:
    rows = []
    for core in cores:
        layout = summary(core, reference)
        depths = mounting_depths(core, reference)
        rows.append({"core_id": core.core_id, "mechanism": core.mechanism, "role": core.role or "",
                     "formulation": core.formulation or "", "source": core.source,
                     "source_commit": core.source_commit or "",
                     "source_run": core.source_run or "", "candidate": core.candidate or "",
                     "finger_length_mm": _round(float(layout["finger_length_mm"])),
                     "proximal_mount_depth_mm": _round(depths[0]), "distal_mount_depth_mm": _round(depths[1]),
                     "design_vector": json.dumps([_round(v) for v in core.x]),
                     "layout": json.dumps(layout, sort_keys=True, default=float)})
    return rows


def _write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _git_commit() -> dict[str, Any]:
    def git(*args: str) -> str:
        return subprocess.run(("git", *args), cwd=REPOSITORY_ROOT, capture_output=True, text=True).stdout.strip()

    # The code and every study file (configs, cores, catalog) count.
    return {"commit": git("rev-parse", "HEAD") or None,
            "dirty": bool(git("status", "--porcelain", "--", "core", "scripts/study"))}


def run(config_path: Path, root: Path = REPOSITORY_ROOT, output: Path | None = None) -> Path:
    """The whole study as ``config_path`` says; returns the output folder."""

    config = read_config_file(config_path)
    cores_values = read_config_file(root / config["cores"])
    modules_values = read_config_file(root / config["modules"])
    cores, duplicates = distinct([load_core(spec, root) for spec in cores_values["cores"]])
    rules = check_equivalent(cores)
    catalog = module_catalog(modules_values)
    flexions = [float(deg) for deg in config["required_flexion_deg"]]
    limit = float(config.get("max_flexion_limit_deg", 180.0))
    output = output or root / config["output"]
    output.mkdir(parents=True, exist_ok=True)
    _write_csv(output / "core_designs.csv", core_rows(cores, catalog[0].modules))
    _write_csv(output / "module_catalog.csv", [dataclasses.asdict(pair) for pair in catalog])
    _write_csv(output / "assembly_results.csv", assemblies(cores, catalog, flexions, limit))
    (output / "manifest.json").write_text(json.dumps({
        "study": "study1_fixed_morphology",
        "config": {"path": str(config_path), "values": config},
        "cores": cores_values, "modules": modules_values,
        "required_flexion_deg": flexions, "max_flexion_limit_deg": limit,
        "physical_rules": rules, "collision_policy": collision_policy(rules["link_radius_mm"]),
        "mounting_depth": "each core's own (its design vector), below its reference lines (CoDe: OA, BC; "
                          "2R-DD: its segments' lines), the module's thickness below it, held for every module",
        "mounting_policy": {
            "policy": sorted({goal_of(core.config).mount_policy for core in cores}),
            "rule": "shallowest_supported: each pad, independently, at the least depth on the resolution grid "
                    "clearing its own joint hardware (case A: 0, the pad short of it; case B: under it, no deeper "
                    "than that hardware reaches: CoDe's pin housings and, for the proximal pad, its PIP servo; the "
                    "2R-DD's servo for its proximal pad; a servo by its footprint over the pad); deeper (case C, "
                    "a spacer) is no design (core.optimization.mounting)",
            "module_replacement": "on the core's frozen depths: each pad's least depth at most the core's, and "
                                  "the core's within the native support under that module "
                                  "(d_req <= d_core <= d_native); else mount_incompatible, not remounted",
        },
        "source": _git_commit(),
        "duplicate_cores": duplicates,
        "counts": {"cores": len(cores), "modules": len(catalog), "flexions": len(flexions)},
    }, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8")
    return output


def main(argv: list[str]) -> int:
    if argv and argv[0].startswith("-"):
        print(__doc__.strip())
        return 2
    print(f"wrote {run(Path(argv[0]) if argv else DEFAULT_CONFIG)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
