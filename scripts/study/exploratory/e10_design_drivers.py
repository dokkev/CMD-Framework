#!/usr/bin/env python3
"""E10: what predicts accommodation capacity inside CoDe's design space?

A deterministic farthest-point cover (normalized design space; the Study 3
``cover``) picks 150 designs from the archived linkage-v12 production fronts
(output/servo_close/linkage/proto_tilt10_align/run_001-009, about 22k
designs). Each design is frozen under the Study 1 rules (code_tilt10 study
config, policy depths) and judged on the Study 1 catalog at 120 deg. Features
per design:
- |AB|, the segment spans and the finger length;
- the extended ICR share;
- ICR travel from 0 to 120 deg (and its palmar drop);
- the PIP servo shaft position.
Spearman correlations of the feature with the pairs carried, and partial
correlations controlling for both spans, follow. The archive was optimized
for other objectives, so it is a biased sample: these are correlations,
not causes (E02 is the causal test).
"""
from __future__ import annotations

import csv
import dataclasses
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from core.optimization.analysis.transmission import Transmission  # noqa: E402
from core.optimization.study import load_run  # noqa: E402

ARCHIVE = common.ROOT / "output/servo_close/linkage/proto_tilt10_align"
COUNT = 150
FLEXION = 120.0
_STATE = {}


def load_archive():
    designs, sources = [], []
    for run_dir in sorted(ARCHIVE.glob("run_00*")):
        run = load_run(run_dir)
        with (run_dir / "pareto.csv").open() as stream:
            for i, row in enumerate(csv.DictReader(stream)):
                designs.append(tuple(float(v) for v in run.mechanism.read_design(row)))
                sources.append(f"{run_dir.name}#{i}")
    return np.array(designs), sources


def icr(finger, phi_deg):
    s = finger.sweep_pip([np.radians(phi_deg)])
    if len(s.q_pip_rad) != 1:
        return None
    a = np.array([finger.joint.a.x_mm, finger.joint.a.z_mm])
    ap = np.array([finger.joint.ap.x_mm, finger.joint.ap.z_mm])
    d1, d2 = s.b[0] - ap, s.bp[0] - a
    t = np.linalg.solve(np.array([d1, -d2]).T, a - ap)
    return ap + t[0] * d1


def _init():
    _STATE["code"] = common.references()["code_matched"]
    _STATE["catalog"] = list(common.catalog().values())


def _job(args):
    source, x = args
    code = _STATE["code"]
    ref = dataclasses.replace(code, reference_id=source, x=tuple(x), source=source)
    part = common.shared.partitions.partition("linkage")
    external = part.external(ref.x, ref.config)
    row = {"source": source, **{f"e_{k}": v for k, v in external.items()}}
    names = common.mechanism("linkage").variable_names
    v = dict(zip(names, x))
    row.update(ab_mm=v["b_x_mm"] - v["a_x_mm"], ap_x_mm=v["ap_x_mm"], ap_z_mm=v["ap_z_mm"],
               bc_mm=v["bc_length_mm"], icr0_share=external["articulation_mm"] / external["finger_length_mm"])
    try:
        finger = Transmission("linkage", ref.x, ref.config).finger
        p0, p120 = icr(finger, 0.0), icr(finger, 120.0)
    except Exception:  # noqa: BLE001 - a design the current decoder rejects
        p0 = p120 = None
    if p0 is not None and p120 is not None:
        row.update(icr_travel_mm=float(np.linalg.norm(p120 - p0)), icr_drop_mm=float(p0[1] - p120[1]),
                   icr120_z_mm=float(p120[1]))
    carried = mounted = 0
    for pair in _STATE["catalog"]:
        r = common.study1.assembly(ref, pair, [FLEXION])[0]
        carried += bool(r["feasible"])
        mounted += bool(r["mounted"])
        if pair.module_id == "baseline":
            row["baseline_feasible"] = bool(r["feasible"])
    row.update(carried=carried, mounted=mounted)
    return row


def spearman(a, b):
    from scipy.stats import spearmanr
    ok = np.isfinite(a) & np.isfinite(b)
    return float(spearmanr(a[ok], b[ok]).statistic) if ok.sum() > 5 else float("nan")


def partial(a, b, controls):
    """Spearman partial correlation: rank-transform, regress out controls, correlate residuals."""
    from scipy.stats import rankdata
    ok = np.isfinite(a) & np.isfinite(b) & np.all(np.isfinite(controls), axis=1)
    ra, rb = rankdata(a[ok]), rankdata(b[ok])
    C = np.column_stack([np.ones(ok.sum())] + [rankdata(c) for c in controls[ok].T])
    res = lambda y: y - C @ np.linalg.lstsq(C, y, rcond=None)[0]  # noqa: E731
    ea, eb = res(ra), res(rb)
    return float(ea @ eb / np.sqrt((ea @ ea) * (eb @ eb)))


def main():
    out = common.output("e10_design_drivers")
    designs, sources = load_archive()
    study3 = common.shared.load_module("study3_run", common.ROOT / "scripts/study/study3_transmission/run.py")
    picks = study3.cover(designs, COUNT, designs)
    jobs = [(sources[i], designs[i]) for i in picks]
    with ProcessPoolExecutor(8, initializer=_init) as pool:
        rows = list(pool.map(_job, jobs))
    features = ("ab_mm", "icr_travel_mm", "icr_drop_mm", "icr120_z_mm", "icr0_share", "e_proximal_span_mm",
                "e_distal_span_mm", "e_finger_length_mm", "ap_x_mm", "ap_z_mm", "bc_mm")
    y = np.array([r["carried"] for r in rows], dtype=float)
    spans = np.array([[r["e_proximal_span_mm"], r["e_distal_span_mm"]] for r in rows], dtype=float)
    stats = []
    for f in features:
        a = np.array([r.get(f, np.nan) for r in rows], dtype=float)
        stats.append({"feature": f, "spearman": spearman(a, y),
                      "partial_given_spans": partial(a, y, spans) if not f.startswith("e_") or "finger" in f else None,
                      "n": int(np.isfinite(a).sum())})
    common.shared.write_csv(out / "correlations.csv", stats)
    common.finish(out, rows, {"archive": str(ARCHIVE.relative_to(common.ROOT)), "count": COUNT,
                              "archive_size": len(designs), "selection": "farthest point, normalized design space",
                              "flexion_deg": FLEXION})
    print("archive", len(designs), "selected", len(rows), "carried: min/median/max",
          y.min(), np.median(y), y.max(), "baseline feasible", sum(bool(r.get("baseline_feasible")) for r in rows))
    for s in stats:
        print(s)


if __name__ == "__main__":
    main()
