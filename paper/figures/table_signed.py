"""Table I data: the signed bias in the measured pad effect, (D(H) - D(O)) / D(O) per placement-direction pair (+: the
effect overstated in its own direction, -: understated or reversed), its
median for CoDe (R) and Conventional (U), and O's effect D(O) (median, N), for the seven Study 3B settings. Same pairs
and gates as each setting's E (median |bias|, checked here against report/study3/data). No simulation: the raw logs
(sim/isaaclab/logs/study3/two_pad, sim/drake/logs). Writes report/study3/data/table_signed.json.

    conda run -n cmd python paper/figures/table_signed.py
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
S3 = ROOT / "report" / "study3" / "data"
sys.path.insert(0, str(ROOT / "sim" / "isaaclab" / "scripts" / "study3_measurement"))
sys.path.insert(0, str(ROOT / "sim" / "drake"))
import dose_analysis as da  # noqa: E402
import robust_analysis as ra  # noqa: E402
from padk_analysis import effects  # noqa: E402
import analyze as dk  # noqa: E402
import analyze_e25 as e25  # noqa: E402


def isaac_friction(hand, cond="A"):
    out = {}
    for r in (15.0, 20.0):
        runs = {(h, m): da.latest(h, m, cond, r) for h in ("O", hand) for m in (1.3, 2.0)}
        if any(v is None for v in runs.values()):
            continue
        by = {hm: {da.env_key(e): e for e in d["envs"]} for hm, d in runs.items()}
        for k in by[("O", 1.3)]:
            es = {hm: by[hm].get(k) for hm in by}
            if any(e is None or not e["gate0"]["pass"] or e["limit"]["f_limit_n"] is None for e in es.values()):
                continue
            d_o = es[("O", 2.0)]["limit"]["f_limit_n"] - es[("O", 1.3)]["limit"]["f_limit_n"]
            d_h = es[(hand, 2.0)]["limit"]["f_limit_n"] - es[(hand, 1.3)]["limit"]["f_limit_n"]
            if abs(d_o) >= 1e-9:
                out[(r,) + tuple(k)] = (d_h, d_o)
    return out


def pairs_from(get_h, get_o):
    eh, eo = get_h, get_o
    return {k: (eh[k], d_o) for k, d_o in eo.items() if k in eh and abs(d_o) > 1e-9}


def drake(hand, contact, tag, ref_tag=None):
    out = {}
    for r in dk.RADII:
        o = {m: (e25.load("O", m, ref_tag, r) if ref_tag else dk.load_drake("O", m, contact, tag, r)) for m in dk.MU}
        hh = {m: (e25.load(hand, m, tag, r) if ref_tag else dk.load_drake(hand, m, contact, tag, r)) for m in dk.MU}
        if any(v is None for v in (*o.values(), *hh.values())):
            continue
        by = {**{("O", m): {dk.key(e): e for e in o[m]} for m in dk.MU},
              **{("H", m): {dk.key(e): e for e in hh[m]} for m in dk.MU}}
        for k in by[("O", dk.MU[0])]:
            es = [by[hm].get(k) for hm in by]
            if any(e is None or not e["gate0"]["pass"] or e["limit"].get("f_limit_n") is None for e in es):
                continue
            d_o = by[("O", 2.0)][k]["limit"]["f_limit_n"] - by[("O", 1.3)][k]["limit"]["f_limit_n"]
            d_h = by[("H", 2.0)][k]["limit"]["f_limit_n"] - by[("H", 1.3)][k]["limit"]["f_limit_n"]
            if abs(d_o) >= 1e-9:
                out[k] = (d_h, d_o)
    return out


def ref_e(name, key):
    d = json.load(open(S3 / name))
    if isinstance(d, dict) and "hands" in d:
        return {h: d["hands"][h][key] for h in ("R", "U")}
    rows = d["rows"] if isinstance(d, dict) else d
    return {r["hand"]: r["E"] for r in rows if r["hand"] in ("R", "U")}


SETTINGS = (
    ("friction, Isaac point", lambda h: isaac_friction(h), ("isaac_dose_A_all.json", "E_rel_median")),
    ("stiffness, Isaac point", lambda h: pairs_from(ra.comp_effect(h), ra.comp_effect("O")),
     ("isaac_comp.json", "E_median")),
    ("crowned, Isaac point", lambda h: pairs_from(ra.shape_effect(h), ra.shape_effect("O")),
     ("isaac_shape.json", "E_median")),
    ("friction softer pad, Isaac point",
     lambda h: pairs_from(effects(h, "A-padk"), effects("O", f"A-padk-ref{int(da.thickness(h))}")),
     ("isaac_padk_matched.json", "E_median")),
    ("friction, Drake point", lambda h: drake(h, "point", "e21"), ("drake_e21_point.json", "E")),
    ("friction, Drake hydro", lambda h: drake(h, "hydro", "e21"), ("drake_e21_hydro.json", "E")),
    ("friction softer pad, Drake hydro", lambda h: drake(h, "hydro", "e25", "e25-ref7"),
     ("drake_e25_hydro_own.json", "E")),
)


# the command protocols (pull-out, friction 1.3 -> 2.0, 7 mm): what is reused across redesigns, ordered by the sensing
PROTOCOLS = (
    ("setpoint", lambda h: isaac_friction(h, "A"), ("isaac_dose_A_all.json", "E_rel_median")),
    ("contact setpoint", lambda h: drake(h, "point", "e27"), ("drake_e27_point.json", "E")),
    ("torque", lambda h: isaac_friction(h, "C"), ("isaac_dose_C_all.json", "E_rel_median")),
    ("touch", lambda h: isaac_friction(h, "E"), ("isaac_dose_E_all.json", "E_rel_median")),
    ("force", lambda h: isaac_friction(h, "B"), ("isaac_dose_B.json", "E_rel_median")),
)


def main():
    out = {"note": __doc__.splitlines()[0], "settings": {}, "protocols": {}}
    for group, items in (("settings", SETTINGS), ("protocols", PROTOCOLS)):
        for name, fn, ref in items:
            out[group][name] = one(name, fn, ref)
    (S3 / "table_signed.json").write_text(json.dumps(out, indent=1))


def one(name, fn, ref_):
    refname, refkey = ref_
    if True:
        ref = ref_e(refname, refkey)
        row = {}
        for h in ("R", "U"):
            pr = fn(h)
            b = np.array([(dh - do) / do for dh, do in pr.values()])  # +: overstates O's effect, -: understates it
            e = float(np.median(np.abs(b)))
            assert abs(e - ref[h]) < 1e-6, (name, h, e, ref[h])
            row[h] = {"bias_median": float(np.median(b)), "bias_q25": float(np.percentile(b, 25)),
                      "bias_q75": float(np.percentile(b, 75)), "E": e, "pairs": int(len(b)),
                      "O_effect_median_n": float(np.median([do for _, do in pr.values()]))}
        print(f"{name:34s} O {row['R']['O_effect_median_n']:+.2f} N | CoDe {100 * row['R']['bias_median']:+5.0f}% "
              f"(E {100 * row['R']['E']:.0f}) | Conv {100 * row['U']['bias_median']:+5.0f}% (E {100 * row['U']['E']:.0f})")
        return row


if __name__ == "__main__":
    main()
