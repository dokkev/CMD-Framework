"""Raw data of Studies 1-3 as flat CSV tables, one row per measured design or simulated run, with a metadata JSON per
study (columns, units, definitions, sources, git commit).

    python3 report/make_raw_tables.py

Writes report/study1/data/raw/, report/study2/data/raw/ and report/study3/data/raw/. Study 1 and 2 come from the
committed summaries in report/; Study 3 from the simulation logs (sim/drake/logs, sim/isaaclab/logs/study3/two_pad),
which are untracked: the CSVs written here are the committed copy.
"""
from __future__ import annotations

import csv
import datetime
import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REP = ROOT / "report"
DRAKE = ROOT / "sim/drake/logs"
ISAAC = ROOT / "sim/isaaclab/logs/study3/two_pad"
L_F_MM = 90.0

DRAKE_NAME = re.compile(r"^(dk[a-z]*)_(.+)_mu([0-9.]+)_(point|hydro)-(.+?)(?:_r([0-9.]+|any))?\.json$")
ISAAC_NAME = re.compile(r"^tp_(.+)_mu([0-9.]+)_(A|A-grid)_r([0-9.]+)_\d{8}-\d{6}_\d+\.json$")

# Drake run sets kept (tag -> experiment, what it is); calibration, noise-floor and replaced runs are left out
DRAKE_SETS = {
    ("dk", "point", "e21"): ("21", "pull-out", "point contact 2.5 kN/m"),
    ("dk", "hydro", "e21"): ("21", "pull-out", "hydroelastic 0.1 MPa over 3 mm on every hand"),
    ("dk", "hydro", "e25"): ("25", "pull-out", "hydroelastic 0.1 MPa over each pad's own thickness"),
    ("dk", "hydro", "e25-ref5"): ("25", "pull-out", "reference: O carrying the soft 5 mm pad"),
    ("dk", "hydro", "e25-ref7"): ("25", "pull-out", "reference: O carrying the soft 7 mm pad"),
    ("dk", "hydro", "e25-ref9"): ("25", "pull-out", "reference: O carrying the soft 9 mm pad"),
    ("dk", "point", "e27"): ("27", "pull-out", "joint setpoint re-chosen per hand so the fingertip contact point matches O's"),
    ("dk", "point", "e27m"): ("27", "pull-out", "auxiliary: the same with the distal pad's midpoint matched"),
    ("dkpullin", "point", "e23-a3"): ("23", "grasp acquisition", "cylinder starts 3 mm past O's contact"),
    ("dkpullin", "point", "e23-a6"): ("23", "grasp acquisition", "cylinder starts 6 mm past O's contact"),
    ("dkdial", "point", "e26-s45"): ("26", "dial", "superseded by experiment 28z (distal pad pressed 2-11 mm, beyond the pad model): sprung dial, 45 deg sweep"),
    ("dktipdial", "point", "e28z"): ("28", "fingertip dial", "z-axis disk turned by the distal pad on its top face (Study 3's dial task)"),
    ("dktipdial", "point", "e28y"): ("28", "fingertip dial", "auxiliary: y-axis dial driven tangentially by the distal pad"),
    ("dktipdial", "point", "e29z"): ("29", "fingertip dial", "z-axis disk, 64 randomized starts (seed 29), common to all hands"),
    ("dkcoin", "point", "e30b"): ("30b", "fingertip coin drag", "the distal pad alone drags a tethered coin toward the palm on a sprung table, 64 randomized starts (seed 31)"),
    ("dktipdial", "point", "e29y"): ("29", "fingertip dial", "y-axis dial, 64 randomized starts (seed 29), common to all hands"),
    ("dkdial", "point", "e24-s8"): ("24", "dial", "superseded by experiment 26: dial turned 5-10 deg, 8 deg sweep"),
    ("dkdial", "point", "e24-s15"): ("24", "dial", "superseded by experiment 26: dial turned 5-10 deg, 15 deg sweep"),
    ("dkroll", "point", "e22-s8"): ("22", "in-hand rolling", "flexion sweep of 8 deg"),
    ("dkroll", "point", "e22-s15"): ("22", "in-hand rolling", "flexion sweep of 15 deg"),
}


def rel(p: Path) -> str:
    return str(p.relative_to(ROOT))


def git_commit() -> str:
    return subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def write_csv(path: Path, rows: list[dict]) -> list[str]:
    path.parent.mkdir(parents=True, exist_ok=True)
    cols = list(rows[0].keys())
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    return cols


def r4(v):
    return None if v is None else round(float(v), 4)


def meta(path: Path, body: dict) -> None:
    body = {"created": datetime.date.today().isoformat(), "git_commit": git_commit(), "generator": rel(Path(__file__)),
            **body}
    path.write_text(json.dumps(body, ensure_ascii=False, indent=1))


def study1() -> None:
    src = REP / "study1/data/thickness_coverage_summary.csv"
    rows = [{"architecture": r["architecture"].replace("*", ""), "idealized": r["architecture"].endswith("*"),
             "required_flexion_deg": int(float(r["required_flexion_deg"])), "pad_thickness_mm": float(r["thickness_mm"]),
             "c_pad": r4(r["c_pad_mean"]), "c_pad_worst_run": r4(r["c_pad_run_low"]), "runs": int(r["runs"]),
             "design_found": float(r["c_pad_mean"]) > 0, "stop_reason": r["stop_reason"],
             "proximal_pad_mm": r4(r["proximal_pad_mm"]), "distal_pad_mm": r4(r["distal_pad_mm"]),
             "finger_length_mm": r4(r["finger_length_mm"]), "q_max_deg": r4(r["q_max_deg"]), "ma_wc": r4(r["ma_wc"])}
            for r in csv.DictReader(src.open())]
    out = REP / "study1/data/raw/coverage.csv"
    cols = write_csv(out, rows)
    meta(REP / "study1/data/raw/metadata.json", {
        "study": "Study 1: best pad coverage per architecture, pad thickness and required flexion (2D side model, "
                 "finger length 90 mm)",
        "files": {rel(out): {"rows": "one per (architecture, required flexion, pad thickness): the best design found",
                             "columns": cols}},
        "sources": [rel(src)],
        "definitions": {
            "c_pad": "pad coverage C̄_pad (fraction 0-1) of the best design over the search runs",
            "c_pad_worst_run": "the worst of the runs (GA and NSGA-II, three seeds each)",
            "idealized": "upper bound without the joint actuator (CFL ideal, RCJ ideal)",
            "design_found": "false where no design meets the requirement",
            "q_max_deg": "largest collision-free flexion found (evaluator cap 180)",
            "ma_wc": "worst-case mechanical advantage of the transmission",
            "stop_reason": "the constraint that ended the search's improvement",
        },
        "units": {"lengths": "mm", "angles": "deg", "c_pad": "fraction"},
    })


def study2() -> None:
    src = REP / "study2/data/redesign_summary.csv"
    names = {"cfl": "CFL", "2r_remote": "2R-remote", "2r_dd": "2R-DD", "rcj": "RCJ", "cfl_ideal": "CFL", "rcj_ideal": "RCJ"}
    rows = [{"architecture": names[r["architecture"]], "idealized": r["upper_bound"] == "True",
             "required_flexion_deg": int(float(r["required_flexion_deg"])), "from_mm": float(r["from_mm"]),
             "to_mm": float(r["to_mm"]), "fixed_axis_bound_mm": r4(float(r["fixed_axis_change"]) * L_F_MM),
             "study1_c_pad": r4(r["study1_c_pad"]) if r["study1_c_pad"] else None,
             "front_points": int(r["front_points"]) if r["front_points"] else 0,
             "d_contact_at_most_c_pad_mm": r4(float(r["change_at_most_c_pad"]) * L_F_MM) if r["change_at_most_c_pad"] else None,
             "most_c_pad": r4(r["most_c_pad"]) if r["most_c_pad"] else None,
             "least_d_contact_mm": r4(float(r["least_change"]) * L_F_MM) if r["least_change"] else None,
             "c_pad_at_least_d_contact": r4(r["c_pad_at_least_change"]) if r["c_pad_at_least_change"] else None}
            for r in csv.DictReader(src.open())]
    out = REP / "study2/data/raw/redesign.csv"
    cols = write_csv(out, rows)
    fronts = REP / "study2/data/redesign_fronts.csv"
    meta(REP / "study2/data/raw/metadata.json", {
        "study": "Study 2: redesign of each architecture's 3 mm design for 5, 7 and 9 mm pads (2D side model)",
        "files": {rel(out): {"rows": "one per (architecture, required flexion, target thickness): the two ends of the "
                                     "Pareto front of coverage against contact-surface change", "columns": cols},
                  rel(fronts): {"rows": "every point of every front (already committed; not copied)"}},
        "sources": [rel(src), rel(fronts)],
        "definitions": {
            "d_contact": "Δ_contact: largest change over flexion of the distal contact point's path, seen from the "
                         "proximal pad face, between the 3 mm design and the redesign; stored as a fraction of the "
                         "finger length, converted here with L_f = 90 mm",
            "fixed_axis_bound_mm": "lowest Δ_contact a fixed-axis joint can reach for this thickness change, "
                                   "2 sin(q_req/2) Δh",
            "d_contact_at_most_c_pad_mm / most_c_pad": "the coverage end of the front (the design reported)",
            "least_d_contact_mm / c_pad_at_least_d_contact": "the other end of the front",
            "study1_c_pad": "Study 1's best coverage at the target thickness",
        },
        "units": {"lengths": "mm", "c_pad": "fraction"},
    })


def drake_rows() -> tuple[list[dict], list[str]]:
    rows, srcs = [], []
    for p in sorted(DRAKE.glob("dk*.json")):
        m = DRAKE_NAME.match(p.name)
        if not m:
            continue
        kind, hand, mu, contact, tag, rho = m.groups()
        info = DRAKE_SETS.get((kind, contact, tag))
        if not info:
            continue
        exp, task, setting = info
        srcs.append(rel(p))
        d = json.loads(p.read_text())
        for e in d["envs"]:
            g = e.get("gate0") or {}
            pl = e.get("placement") or {}
            e = {"q0_deg": pl.get("q_t_deg", pl.get("q0_deg")), "x_c_mm": pl.get("x_c_mm"), "direction": "none", **e}
            radius = float(e["rho_mm"]) if "rho_mm" in e else (float(rho) if rho not in (None, "any") else
                                                                (pl["diam_mm"] / 2 if "diam_mm" in pl else None))
            row = {"engine": "Drake", "experiment": exp, "task": task, "setting": setting, "tag": tag,
                   "contact_model": contact, "hand": hand, "mu": float(mu), "cylinder_radius_mm": r4(radius),
                   "draw": (e.get("placement") or {}).get("draw"), "tip_spring_nm_rad": r4((e.get("placement") or {}).get("tip_kr")),
                   "sideways_offset_mm": r4((e.get("placement") or {}).get("y_mm")),
                   "q0_deg": e["q0_deg"], "x_c_mm": r4(e["x_c_mm"]), "load_direction": e["direction"],
                   "gate0_pass": g.get("pass"), "measure": None, "measure_unit": None, "event": None, "held_at_end": None}
            lim = e.get("limit") or {}
            if task == "pull-out":
                row.update(measure=r4(lim.get("f_limit_n")), measure_unit="N (pull-out limit load)", event=lim.get("event"))
                sp = e.get("setpoint")
                if sp:
                    row["setpoint_o_deg"] = r4(sp.get("q_cmd_o_deg"))
                    row["setpoint_deg"] = r4(sp.get("q_cmd_deg"))
                    row["setpoint_residual_mm"] = r4(sp.get("residual_mm"))
            elif task == "grasp acquisition":
                x = e.get("pullin") or {}
                row.update(measure=r4(x.get("drawn_mm")), measure_unit="mm (drawn in toward the MCP)", held_at_end=x.get("held"))
            elif task in ("dial", "fingertip dial"):
                x = e.get("dial") or {}
                row.update(measure=r4(x.get("rotation_deg")), measure_unit="deg (dial rotation)", held_at_end=x.get("held"),
                           event=None if x.get("touched", True) else "not touched")
                if "distal_force_max_n" in x:
                    row["distal_force_max_n"] = r4(x.get("distal_force_max_n"))
                    row["proximal_force_max_n"] = r4(x.get("proximal_force_max_n"))
                elif "distal_normal_end_n" in x:
                    row["distal_force_end_n"] = r4(x.get("distal_normal_end_n"))
            elif task == "fingertip coin drag":
                x = e.get("drag") or {}
                row.update(measure=r4(x.get("travel_end_mm")), measure_unit="mm (coin travel toward the MCP)",
                           event=None if x.get("touched", True) else "not touched",
                           distal_force_max_n=r4(x.get("distal_force_max_n")), proximal_force_max_n=r4(x.get("proximal_force_max_n")))
            elif task == "in-hand rolling":
                x = e.get("roll") or {}
                row.update(measure=r4(x.get("rotation_deg")), measure_unit="deg (cylinder rotation)", held_at_end=x.get("held"),
                           event=None if x.get("lost_at_step") is None else f"lost at step {x['lost_at_step']}")
            rows.append(row)
    return rows, srcs


def isaac_rows() -> tuple[list[dict], list[str]]:
    latest = {}
    for p in sorted(ISAAC.glob("tp_*.json")):
        m = ISAAC_NAME.match(p.name)
        if m:
            latest[m.groups()] = p  # the last run of each (hand, mu, condition, radius), as sim/drake/analyze.py
    rows, srcs = [], []
    for (hand, mu, cond, rho), p in sorted(latest.items()):
        srcs.append(rel(p))
        d = json.loads(p.read_text())
        for e in d["envs"]:
            lim = e.get("limit") or {}
            rows.append({"engine": "Isaac Lab", "experiment": "5-13", "task": "pull-out",
                         "setting": "point contact; O's placements" if cond == "A" else "point contact; hand-independent grid placements",
                         "tag": cond, "contact_model": "point", "hand": hand, "mu": float(mu),
                         "cylinder_radius_mm": float(e.get("rho_mm", rho)), "q0_deg": e["q0_deg"], "x_c_mm": r4(e["x_c_mm"]),
                         "load_direction": e["direction"], "gate0_pass": (e.get("gate0") or {}).get("pass"),
                         "measure": r4(lim.get("f_limit_n")), "measure_unit": "N (pull-out limit load)",
                         "event": lim.get("event"), "held_at_end": None})
    return rows, srcs


def study3() -> None:
    out_dir = REP / "study3/data/raw"
    drows, dsrc = drake_rows()
    irows, isrc = isaac_rows()
    files = {}
    keys = ["engine", "experiment", "task", "setting", "tag", "contact_model", "hand", "mu", "cylinder_radius_mm", "q0_deg",
            "x_c_mm", "load_direction", "gate0_pass", "measure", "measure_unit", "event", "held_at_end",
            "distal_force_max_n", "proximal_force_max_n", "distal_force_end_n",
            "draw", "tip_spring_nm_rad", "sideways_offset_mm",
            "setpoint_o_deg", "setpoint_deg", "setpoint_residual_mm"]
    groups = {}
    for r in drows + irows:
        name = {"pull-out": "pullout", "grasp acquisition": "acquisition", "dial": "dial", "fingertip dial": "tipdial",
                "fingertip coin drag": "coin",
                "in-hand rolling": "rolling"}[r["task"]]
        eng = "isaac" if r["engine"] == "Isaac Lab" else "drake"
        groups.setdefault(f"runs_{eng}_{name}.csv", []).append({k: r.get(k) for k in keys})
    for fname, rows in sorted(groups.items()):
        cols = [k for k in keys if any(r.get(k) is not None for r in rows)]
        write_csv(out_dir / fname, [{k: r[k] for k in cols} for r in rows])
        files[rel(out_dir / fname)] = {"rows": "one per simulated run: hand x friction x cylinder radius x placement x "
                                               "load direction (or trajectory)", "count": len(rows), "columns": cols,
                                       "hands": sorted({r["hand"] for r in rows})}

    # per-hand errors (already computed by sim/drake/analyze.py and the Isaac Lab dose analysis)
    per = []
    sets = [("isaac_dose_A_all.json", "Isaac Lab", "pull-out", "point contact; O's placements"),
            ("isaac_dose_A-grid_expanded.json", "Isaac Lab", "pull-out", "point contact; hand-independent grid placements"),
            ("drake_e21_point.json", "Drake", "pull-out", "point contact"),
            ("drake_e21_hydro.json", "Drake", "pull-out", "hydroelastic 3 mm"),
            ("drake_e25_hydro_own.json", "Drake", "pull-out", "hydroelastic, each pad as soft as it is thick"),
            ("drake_e27_point.json", "Drake", "pull-out", "contact-referenced joint setpoint"),
            ("drake_e27m_point.json", "Drake", "pull-out", "contact-referenced setpoint, pad midpoint (auxiliary)"),
            ("drake_e23_point.json", "Drake", "grasp acquisition", "point contact"),
            ("drake_e26_point.json", "Drake", "dial", "superseded by experiment 28z (pad pressed beyond the model)"),
            ("drake_e28z_point.json", "Drake", "fingertip dial", "z-axis disk (Study 3's dial task)"),
            ("drake_e28y_point.json", "Drake", "fingertip dial", "auxiliary: y-axis fingertip dial"),
            ("drake_e29z_point.json", "Drake", "fingertip dial", "z-axis disk, randomized starts (confirmation)"),
            ("drake_e29y_point.json", "Drake", "fingertip dial", "y-axis dial, randomized starts (confirmation)"),
            ("drake_e30b_coin.json", "Drake", "fingertip coin drag", "long drag, randomized starts"),
            ("drake_e24_point.json", "Drake", "dial", "superseded by experiment 26: dial turned 5-10 deg"),
            ("drake_e22_point.json", "Drake", "in-hand rolling", "point contact")]
    for fname, eng, task, setting in sets:
        d = json.loads((REP / "study3/data" / fname).read_text())
        if isinstance(d, dict) and "hands" in d:
            items = [(k, v["D_c_mm"], v["D_s_mm"], v["E_rel_median"], v["pairs"], v["thickness_mm"]) for k, v in d["hands"].items()]
        else:
            rows = d["rows"] if isinstance(d, dict) else d
            items = [(r["hand"], r["d_contact"], r["d_skel"], r.get("E"), r.get("pairs"), None) for r in rows]
        for hand, dc, ds, e, n, h in items:
            if hand == "O":
                continue
            per.append({"engine": eng, "task": task, "setting": setting, "source": f"report/study3/data/{fname}",
                        "hand": hand, "pad_thickness_mm": h, "d_contact_mm": dc, "d_skel_mm": ds, "E": r4(e), "pairs": n})
    th = {r["hand"]: r["pad_thickness_mm"] for r in per if r["pad_thickness_mm"]}
    for r in per:
        r["pad_thickness_mm"] = th.get(r["hand"], r["pad_thickness_mm"])
    out = out_dir / "errors_by_hand.csv"
    cols = write_csv(out, per)
    files[rel(out)] = {"rows": "one per (setting, redesigned hand): the error E of that hand", "count": len(per), "columns": cols}

    meta(out_dir / "metadata.json", {
        "study": "Study 3: the error a redesign adds to a measured interface effect (simulation; CFL fingers, q_req 120 deg)",
        "files": files,
        "sources": {"drake_logs": dsrc, "isaac_logs": isrc,
                    "per_hand": [f"report/study3/data/{s[0]}" for s in sets]},
        "protocol": {
            "grasp": "the cylinder is placed where O's proximal and distal pads both touch it at flexion q0 and the "
                     "finger is closed to O's joint setpoint q0 + 5 deg at stiffness K0 (joint impedance control); the "
                     "same setpoint is given to every hand unless the setting says otherwise",
            "gate0": "the grasp is accepted when both pads hold the cylinder still after release (gate0_pass)",
            "pull-out": "a load in one direction (+x toward the fingertip, -x toward the palm, -z out of the grasp, "
                        "+x-z diagonal) rises until the grasp gives way; measure = that load",
            "grasp acquisition": "from the open finger, closing to O's setpoint draws the cylinder in against a load; "
                                 "measure = distance drawn in",
            "dial": "one closing turns a sprung dial pressed between both pads; measure = rotation (experiments 24, 26; superseded)",
            "fingertip dial": "the distal pad alone drives a dial against a torsional spring as the finger sweeps open-loop "
                              "from O's touch angle; z: a disk whose top face the pad drags, y: a rim pushed tangentially; "
                              "the proximal pad never touches; measure = rotation (experiment 28)",
            "fingertip coin drag": "the distal pad alone drags a 2 mm coin (tethered at 45 N/m, coin-table mu 0.2) toward the "
                                   "MCP on a table sprung at 30 N/m, sweeping 45-55 deg open-loop from O's touch angle; "
                                   "measure = the coin's travel at the end (experiment 30b; time step 0.25 ms)",
            "in-hand rolling": "after the grasp, an open-loop flexion sweep of 8 or 15 deg rolls the cylinder; "
                               "measure = rotation",
        },
        "events": {"slip": "a pad's tangential speed exceeds 10 mm/s", "loss": "a pad's normal force falls below the "
                   "threshold (the pad leaves the cylinder)", "collapse": "the finger is pushed open (the joint "
                   "yields) before the pads slip", "roll": "the cylinder rolls past the threshold"},
        "error": {
            "D": "effect of the friction change in a run pair: measure(mu 2.0) - measure(mu 1.3) at the same "
                 "placement, direction and radius",
            "E (per pair)": "|D(hand) - D(O)| / |D(O)|, over the pairs where O and the hand both pass gate 0 at both "
                            "frictions and the measure exists",
            "E (per hand, errors_by_hand.csv)": "median over that hand's pairs (sim/drake/analyze.py error()); tasks "
                                                "drop pairs with a small effect on O (|D(O)| < 0.3 mm or deg, rolling "
                                                "< 0.5 deg); acquisition scores a hand that loses the cylinder as the "
                                                "largest error",
            "experiment 25": "each redesigned hand is compared with O carrying the same soft pad (tags e25-ref5/7/9)",
        },
        "hands": {"O": "3 mm reference", "R": "7 mm redesign keeping the contact-surface motion (Δ_contact 0.86 mm)",
                  "U": "7 mm redesign for coverage only (Δ_contact 3.91 mm)",
                  "naming": "dN (p = decimal point): target Δ_contact in mm; h3toN: 3 -> N mm; maxDs/minDs: largest or "
                            "smallest skeleton change for that target; _hr/_hu: the R- and U-type designs at another "
                            "thickness; _s0/_s1: search seed"},
        "units": {"lengths": "mm", "loads": "N", "angles": "deg", "E": "fraction"},
    })


def main() -> int:
    study1()
    study2()
    study3()
    for p in sorted((REP).glob("study[123]/data/raw/*")):
        print(f"{p.stat().st_size / 1024:8.0f} KB  {rel(p)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
