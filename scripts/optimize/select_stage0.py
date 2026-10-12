#!/usr/bin/env python3
"""Screen and verify Stage-0 Pareto candidates into a shortlist.

    python scripts/optimize/select_stage0.py outputs/stage0/run_004 outputs/stage0/run_005

Each run's Pareto candidates are screened on their grasp MCP angle and facing
error, then re-checked on a fine MCP grid (see
:func:`cmd.optimization.stage0.verify.verify_candidate`). Candidates are
reported best transmission quality first, with the window between the safe
limit and the first near-singular pose where a mechanical stopper may go.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from cmd.morphology.linkage import LinkageWorkspaceCriteria  # noqa: E402
from cmd.optimization.stage0 import VARIABLE_NAMES, Stage0Config, decode  # noqa: E402
from cmd.optimization.stage0.verify import verify_candidate  # noqa: E402


DEFAULT_OUTPUT = REPOSITORY_ROOT / "outputs" / "stage0" / "shortlist.csv"


def load_config(run_directory: Path) -> Stage0Config:
    """Rebuild the study configuration recorded in a run's ``run.json``."""

    values = json.loads((run_directory / "run.json").read_text(encoding="utf-8"))["config"]
    values["bounds"] = {name: tuple(bounds) for name, bounds in values["bounds"].items()}
    values["criteria"] = LinkageWorkspaceCriteria(**values["criteria"])
    values["extended_length_bounds_mm"] = tuple(values["extended_length_bounds_mm"])
    return Stage0Config(**values)


def shortlist_run(
    run_directory: Path, max_grasp_mcp_deg: float, max_facing_error_deg: float
) -> tuple[int, list[dict[str, object]]]:
    """Return the number screened in and the verified rows of one run."""

    config = load_config(run_directory)
    with (run_directory / "pareto.csv").open(encoding="utf-8", newline="") as stream:
        candidates = list(csv.DictReader(stream))
    screened = [
        row for row in candidates
        if float(row["grasp_mcp_deg"]) <= max_grasp_mcp_deg
        and float(row["facing_error_deg"]) <= max_facing_error_deg
    ]
    rows = []
    for row in screened:
        x = [float(row[name]) for name in VARIABLE_NAMES]
        check = verify_candidate(decode(x), config, math.radians(float(row["grasp_mcp_deg"])))
        rows.append({
            "run": run_directory.name,
            "candidate": row["candidate"],
            **dict(zip(VARIABLE_NAMES, x)),
            "worst_transmission_quality": float(row["worst_transmission_quality"]),
            "facing_error_deg": float(row["facing_error_deg"]),
            "grasp_mcp_deg": float(row["grasp_mcp_deg"]),
            "safe_limit_deg": math.degrees(check.safe_limit_rad),
            "stopper_before_deg": math.degrees(check.first_singular_rad),
            "reversal_after_deg": math.degrees(check.first_reversal_rad),
            "closes_to_deg": math.degrees(check.closes_to_rad),
            "min_margin_deg": math.degrees(check.min_margin_rad),
            "max_backoff_deg": math.degrees(check.max_backoff_rad),
            "verified": check.passed,
        })
    return len(screened), rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path, help="Stage-0 run directories")
    parser.add_argument("--max-grasp-mcp-deg", type=float, default=60.0)
    parser.add_argument("--max-facing-error-deg", type=float, default=10.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)

    shortlist = []
    print("run       screened  verified  best quality")
    for run_directory in args.runs:
        screened, rows = shortlist_run(
            run_directory, args.max_grasp_mcp_deg, args.max_facing_error_deg
        )
        verified = [row for row in rows if row["verified"]]
        best = max((row["worst_transmission_quality"] for row in verified), default=float("nan"))
        print(f"{run_directory.name:9s} {screened:8d}  {len(verified):8d}  {best:12.3f}")
        shortlist += verified
    shortlist.sort(key=lambda row: -row["worst_transmission_quality"])

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        fields = list(shortlist[0]) if shortlist else ["run", "candidate"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(shortlist)
    print(f"\n{len(shortlist)} verified candidates -> {args.output}")
    for row in shortlist[:10]:
        print(
            f"  {row['run']} #{row['candidate']:>3s}: quality {row['worst_transmission_quality']:.3f}, "
            f"facing {row['facing_error_deg']:.1f} deg, grasp MCP {row['grasp_mcp_deg']:.0f} deg, "
            f"safe to {row['safe_limit_deg']:.0f} deg, stopper before {row['stopper_before_deg']:.1f} deg, "
            f"min margin {row['min_margin_deg']:.0f} deg"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
