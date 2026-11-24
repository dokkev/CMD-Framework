"""Find a CFL design whose pads, each lengthened toward the joint as far as it stays clear of the other over
0..150 deg, come closest at 150 deg (Fig. 2(a)). Candidates: Study 1's CFL designs for 150 deg and Study 2's CFL
front points for 150 deg (and 120 deg designs that reach 150). Writes design/output/paper/fig2_pad_search.json.

    conda run -n cmd python paper/figures/fig2_pad_search.py
"""
import csv
import json
import sys
from dataclasses import replace
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "paper" / "figures"), str(ROOT / "design"), str(ROOT / "design" / "scripts" / "study"),
                str(ROOT / "design" / "scripts" / "study" / "study2_morphology_adaptation")]
import one_finger  # noqa: E402
from core.optimization.mechanism import read_config_file, mechanism  # noqa: E402
from visualize.finger import view_of  # noqa: E402
from shapely.geometry import Polygon as SPoly  # noqa: E402

Q = 150.0
cfg = read_config_file(one_finger.DEFAULT_CONFIG)
spec = cfg["architectures"]["cfl"]


def with_pads(cf, h):
    pk = cf.packaging
    return replace(cf, packaging=replace(pk, proximal_pad=replace(pk.proximal_pad, height_mm=h),
                                         distal_pad=replace(pk.distal_pad, height_mm=h)))


def evaluate(mech, config, x, h=7.0):
    names = list(mechanism(mech).variable_names)
    x = list(x)
    for k in ("proximal_mount_offset_mm", "distal_mount_offset_mm"):
        x[names.index(k)] = 0.0
    try:
        v = view_of(mech, with_pads(config, h), x)
        if any(v.collisions(q) for q in np.linspace(0, Q, 16)):
            return None
        d0 = v.distal_pad(0.0)
        poses = []
        for qq in np.linspace(0.0, Q, 31):
            dq = v.distal_pad(qq)
            ang = np.arctan2(*(dq[1] - dq[0])[::-1]) - np.arctan2(*(d0[1] - d0[0])[::-1])
            poses.append((np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]]), dq[1]))
    except Exception:
        return None

    def padded(ep, ed):
        pp = v.proximal_pad().copy(); pp[[1, 2], 0] += ep
        dd = d0.copy(); dd[[0, 3], 0] -= ed
        return pp, dd

    def clear(ep, ed):
        pp, dd = padded(ep, ed)
        return all(SPoly(pp).buffer(0.3).intersection(SPoly((dd - d0[1]) @ R.T + t)).area < 1e-4 for R, t in poses)
    if not clear(0, 0):
        return None
    ep = ed = 0.0
    for _ in range(3):
        while ep < 30 and clear(ep + 0.5, ed):
            ep += 0.5
        while ed < 30 and clear(ep, ed + 0.5):
            ed += 0.5
    pp, dd = padded(ep, ed)
    R, t = poses[-1]
    gap = SPoly(pp).distance(SPoly((dd - d0[1]) @ R.T + t))
    lp = float(pp[:, 0].max() - pp[:, 0].min()); ld = float(dd[:, 0].max() - dd[:, 0].min())
    return {"gap150_mm": gap, "ext": [ep, ed], "len": [lp, ld]}


def main():
    out = []
    for q_req in (150.0, 120.0):
        ds = one_finger.study1_designs("cfl", spec, q_req, cfg)
        for h, d in ds.items():
            r = evaluate(d.mechanism, d.config, d.x)
            if r:
                out.append({"source": f"study1 q{q_req:g} h{h:g}", "x": list(map(float, d.x)), "cfg": [q_req, h], **r})
        for row in csv.DictReader(open(ROOT / "design/output/study/study2_redesign_2026-11-21/front.csv")):
            if row["architecture"] != "cfl" or float(row["required_flexion_deg"]) != q_req:
                continue
            hh = float(row["to_mm"])
            if hh not in ds:
                continue
            x = [float(v) for v in row["x"].split(";")]
            r = evaluate(ds[hh].mechanism, ds[hh].config, x)
            if r:
                out.append({"source": f"study2 q{q_req:g} to{hh:g}", "x": x, "cfg": [q_req, hh], **r})
    out.sort(key=lambda r: r["gap150_mm"])
    for r in out[:10]:
        print(round(r["gap150_mm"], 2), r["source"], r["ext"], [round(v, 1) for v in r["len"]])
    print(len(out), "candidates")
    (ROOT / "design/output/paper/fig2_pad_search.json").write_text(json.dumps(out[:20], indent=1))


if __name__ == "__main__":
    main()
