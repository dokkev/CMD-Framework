#!/usr/bin/env python3
"""E03: proximal-distal module independence.

A grid of proximal length x distal length (15, 20, 25, 30 mm; thickness 3 or
8 mm, the same on both sides) is evaluated on every matched reference at
120 deg, through Study 1's ``assembly``. Coupling is read two ways:
- **verdict interaction**: the joint verdict at (p, d) differs from
  verdict(p, 20) AND verdict(20, d);
- **usable-length coupling**: how much one side's effective length changes
  when only the other side's module changes.
Only pairs that mount are counted.
"""
from __future__ import annotations

import itertools
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

LENGTHS = (15.0, 20.0, 25.0, 30.0)
THICKNESS = (3.0, 8.0)
FLEXION = 120.0


def main():
    refs = common.references()
    out = common.output("e03_module_independence")
    rows = []
    for rid, ref in refs.items():
        for t in THICKNESS:
            for p, d in itertools.product(LENGTHS, LENGTHS):
                pair = common.shared.ModulePair(f"p{p:g}_d{d:g}_t{t:g}", "e03", p, t, d, t)
                row = common.study1.assembly(ref, pair, [FLEXION])[0]
                rows.append({"reference_id": rid, "thickness_mm": t, "proximal_length_mm": p, "distal_length_mm": d,
                             **{k: row.get(k) for k in ("mounted", "feasible", "failure", "proximal_effective_mm",
                                                        "distal_effective_mm", "phi_max_deg")}})
    summary = []
    for rid in refs:
        for t in THICKNESS:
            grid = {(r["proximal_length_mm"], r["distal_length_mm"]): r for r in rows
                    if r["reference_id"] == rid and r["thickness_mm"] == t}
            pairs = interactions = 0
            prox_coupling = dist_coupling = 0.0
            for (p, d), r in grid.items():
                one_p, one_d = grid[(p, 20.0)], grid[(20.0, d)]
                if not (r["mounted"] and one_p["mounted"] and one_d["mounted"]):
                    continue
                pairs += 1
                interactions += int(bool(r["feasible"]) != (bool(one_p["feasible"]) and bool(one_d["feasible"])))
                # change of distal usable length caused only by the proximal module (and vice versa)
                if r.get("distal_effective_mm") is not None and one_d.get("distal_effective_mm") is not None:
                    dist_coupling = max(dist_coupling, abs(r["distal_effective_mm"] - one_d["distal_effective_mm"]))
                if r.get("proximal_effective_mm") is not None and one_p.get("proximal_effective_mm") is not None:
                    prox_coupling = max(prox_coupling, abs(r["proximal_effective_mm"] - one_p["proximal_effective_mm"]))
            summary.append({"reference_id": rid, "thickness_mm": t, "mounted_pairs": pairs,
                            "verdict_interactions": interactions,
                            "interaction_rate": interactions / pairs if pairs else None,
                            "max_distal_change_from_proximal_mm": dist_coupling,
                            "max_proximal_change_from_distal_mm": prox_coupling})
    common.shared.write_csv(out / "summary.csv", summary)
    common.finish(out, rows, {"lengths_mm": LENGTHS, "thickness_mm": THICKNESS, "flexion_deg": FLEXION})
    for s in summary:
        print(s)


if __name__ == "__main__":
    main()
