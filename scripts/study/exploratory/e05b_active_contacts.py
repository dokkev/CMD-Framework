#!/usr/bin/env python3
"""E05 follow-up: which static contact is exactly active in each reference.

Writes ``nominal_margins.csv`` (every constraint of each prepared nominal
design) and ``one_at_a_time.csv`` (each free variable moved by +/-0.1 mm, or
+/-0.5 deg for the tilt, and the pads re-seated by the policy; the
violations listed) under output/study/exploratory/e05_tolerance/. Run after
``e05_tolerance.py``.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
import e05_tolerance as e5  # noqa: E402
from core.optimization.mounting import mounting  # noqa: E402

OUT = common.OUTPUT / "e05_tolerance"


def main():
    refs, cat = common.references(), common.catalog()
    margins, scan = [], []
    for rid, ref in refs.items():
        found = common.mechanism(ref.mechanism)
        part = common.shared.partitions.partition(ref.mechanism)
        for module in ("baseline", "thick"):
            x, config = e5.prepared(ref, cat[module].modules)
            e = found.evaluate(x, config)
            margins.append({"reference_id": rid, "module": module, "feasible": e.feasible,
                            **{f"g_{k}": v for k, v in e.constraints.items()}})
            if module != "baseline":
                continue
            names = found.variable_names
            bounds = getattr(config, "bounds", {}) or {}
            for i, name in enumerate(names):
                if name in part.depths or (name in bounds and name != "servo_tilt_deg"
                                           and bounds[name][0] == bounds[name][1]):
                    continue
                for delta in (-0.1, 0.1):
                    y = list(x)
                    y[i] += delta * (5 if name == "servo_tilt_deg" else 1)
                    seated = mounting(ref.mechanism, tuple(y), config).depths_mm
                    if seated is None:
                        scan.append({"reference_id": rid, "variable": name, "delta": delta, "feasible": False,
                                     "violations": "no_depth"})
                        continue
                    for slot, depth in zip(part.depths, seated):
                        y[names.index(slot)] = depth
                    ev = found.evaluate(tuple(y), config)
                    bad = {k: round(v, 3) for k, v in ev.constraints.items() if v > 0}
                    scan.append({"reference_id": rid, "variable": name, "delta": delta, "feasible": ev.feasible,
                                 "violations": ";".join(f"{k}={v}" for k, v in bad.items())})
    common.shared.write_csv(OUT / "nominal_margins.csv", margins)
    common.shared.write_csv(OUT / "one_at_a_time.csv", scan)
    for row in scan:
        if row["violations"]:
            print(row)


if __name__ == "__main__":
    main()
