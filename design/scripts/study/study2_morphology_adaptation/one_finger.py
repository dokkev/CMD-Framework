#!/usr/bin/env python3
"""Study 1's designs, and each of them with pads of every thickness (problem formulation A.6.0).

    python scripts/study/study2_morphology_adaptation/one_finger.py [one_finger.yaml]
        [--architectures cfl,2r_dd] [--flexions 120] [--output dir]

Reads Study 1's designs (the finger optimized for each pad thickness and
required flexion, A.6.2) and, with no search, mounts pads of every thickness
on each of them as it is: ``cross.csv`` (designed for ``h_a``, carrying
``h_b``: its coverage, its reach, why it stops). ``pole_law.py`` checks
Study 0's relations against these. An architecture and flexion whose Study 1
runs are not all there yet is skipped (and said so).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))
STUDY = REPOSITORY_ROOT / "scripts" / "study"
if str(STUDY) not in sys.path:
    sys.path.insert(0, str(STUDY))

import shared  # noqa: E402
from core.optimization.analysis.one_finger import coverage, fitted, mount  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "one_finger.yaml"


def condition_folder(name: str, spec: dict[str, Any], flexion: float, thickness: float, native: float) -> Path:
    tag = f"{name}_q{flexion:g}" + ("" if thickness == native else f"_h{thickness:g}")
    return REPOSITORY_ROOT / "output" / "study" / spec["mechanism"] / tag


def study1_designs(name: str, spec: dict[str, Any], flexion: float, config: dict[str, Any]) -> dict[float, Any]:
    """Study 1's design of ``name`` for each thickness at ``flexion`` (a
    reference: its design vector and the config it was searched under); a
    thickness whose run is not there is left out."""

    native = float(config["native_thickness_mm"])
    specs = []
    for thickness in (float(h) for h in config["thickness_mm"]):
        portfolios = sorted(condition_folder(name, spec, flexion, thickness, native).glob("portfolio_*"))
        if not portfolios:
            continue
        entry = {"id": f"{name}_q{flexion:g}_h{thickness:g}", "config": spec["config"],
                 "required_flexion_deg": flexion, "role": "balanced",
                 "portfolio": str(portfolios[-1].relative_to(REPOSITORY_ROOT)),
                 "upper_bound": bool(spec.get("upper_bound", False)),
                 "set": [f"goal.pads.proximal.height_mm={thickness:g}", f"goal.pads.distal.height_mm={thickness:g}"]}
        if thickness == native and spec.get("native_design"):
            entry["design"] = spec["native_design"]
        specs.append((thickness, entry))
    found = shared.references({"native": [entry for _, entry in specs]}, ("native",),
                              shared.catalog(REPOSITORY_ROOT)["baseline"].modules)
    return {thickness: reference for (thickness, _), reference in zip(specs, found)}


class Study:
    """One architecture at one required flexion."""

    def __init__(self, name: str, flexion: float, designs: dict[float, Any]) -> None:
        self.name, self.flexion, self.designs = name, float(flexion), designs
        self.thicknesses = sorted(designs)
        first = designs[self.thicknesses[0]]
        self.base = mechanism(first.mechanism)
        self.part = shared.partitions.partition(first.mechanism)
        self.frame = first.config  # the thickness in it is replaced at every mounting
        self.upper_bound = bool(first.upper_bound)

    def head(self) -> dict[str, Any]:
        return {"architecture": self.name, "mechanism": self.base.name, "upper_bound": self.upper_bound,
                "required_flexion_deg": self.flexion}

    def carried(self, x) -> list[dict[str, Any]]:
        """The finger ``x`` with the pads of each thickness: its coverage, pads and reach."""

        found = []
        for thickness in self.thicknesses:
            composed, evaluation = mount(self.base, self.part, x, self.frame, thickness)
            modules = fitted(self.part, x, self.frame, thickness)
            row = {"thickness_mm": thickness, "carried": bool(evaluation is not None and evaluation.feasible),
                   "c_pad_mean": coverage(self.base, evaluation), "failure": ""}
            if modules is not None:
                row |= {"proximal_pad_mm": modules.proximal_length_mm, "distal_pad_mm": modules.distal_length_mm}
            if composed is None or evaluation is None:
                row["failure"] = "not_mounted"
            else:
                report = self.base.report(evaluation)
                failed = sorted((n for n, v in evaluation.constraints.items() if v > 0.0),
                                key=lambda n: -evaluation.constraints[n])
                row |= {"failure": ";".join(failed[:2]), "q_max_deg": report.get("q_max_deg"),
                        "ma_wc": report.get("ma_wc"), "finger_length_mm": report.get("finger_length_mm")}
            found.append(row)
        return found

    def cross(self) -> list[dict[str, Any]]:
        """A: each Study 1 design carrying the pads of every thickness."""

        return [{**self.head(), "designed_for_mm": designed, **row}
                for designed in self.thicknesses for row in self.carried(self.designs[designed].x)]


def run(path: Path, only_architectures=None, only_flexions=None, output: str | None = None) -> Path:
    config = read_config_file(path)
    out = REPOSITORY_ROOT / (output or config["output"])
    out.mkdir(parents=True, exist_ok=True)
    flexions = [float(q) for q in config["required_flexion_deg"] if not only_flexions or float(q) in only_flexions]
    cross, skipped = [], []
    for name, spec in config["architectures"].items():
        if only_architectures and name not in only_architectures:
            continue
        for flexion in flexions:
            designs = study1_designs(name, spec, flexion, config)
            if len(designs) < len(config["thickness_mm"]):
                missing = sorted(set(float(h) for h in config["thickness_mm"]) - set(designs))
                skipped.append({"architecture": name, "required_flexion_deg": flexion, "missing_thickness_mm": missing})
                print(f"skip {name} q{flexion:g}: Study 1 not run for {missing} mm", flush=True)
                continue
            cross += Study(name, flexion, designs).cross()
            shared.write_csv(out / "cross.csv", cross)
    (out / "manifest.json").write_text(json.dumps({
        "config": str(path), "formulation": "problem_formulation.md A.6.0 (2026-11-20: a fixed finger, other thicknesses)",
        "thickness_mm": config["thickness_mm"], "skipped": skipped, **shared.git_commit(out)}, indent=2))
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("config", nargs="?", default=str(DEFAULT_CONFIG))
    parser.add_argument("--architectures")
    parser.add_argument("--flexions")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    out = run(Path(args.config), args.architectures.split(",") if args.architectures else None,
              [float(q) for q in args.flexions.split(",")] if args.flexions else None, args.output)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
