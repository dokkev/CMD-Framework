#!/usr/bin/env python3
"""Grasp-set benchmark runner (``docs/experimental/grasp_set.md``).

    # run hands (tag=hand file@thumb; thumb: "prototype" = the file's own thumb, or
    #   "x,z,e,orientation[,yaw,pitch,roll[,proximal bend[,IP lower limit]]]")
    python scripts/optimize/thumb_layout/grasp_set.py run --workers 6 --seed 0 --rerun-misses \
        --hand O=output/sim/study3/hands/cfl_q120_h3to7_h0.yaml@prototype \
        --hand O_thumbopt=output/sim/study3/hands/cfl_q120_h3to7_h0.yaml@-50.5,-10.5,34,proximal,-14,-16,14
    # compare (seed 0 with its miss reruns is reported; seed 1 gives the run-to-run spread)
    python scripts/optimize/thumb_layout/grasp_set.py compare O O_thumbopt O_pitch

Writes ``output/thumb_layout/grasp_set/<tag>/seed<N>/``: ``pairs.json`` (one entry per object-grasp pair),
``rerun.json`` (the misses rerun with 4x the seeds), ``hand.json``; ``compare`` writes
``output/thumb_layout/grasp_set/compare_<tags>.json`` and prints the tables.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.morphology.code_hand import load  # noqa: E402
from core.optimization.thumb_layout import Evaluator, ThumbDesign  # noqa: E402
from core.optimization.thumb_layout.evaluator import LIMITS_DEG  # noqa: E402
from core.optimization.thumb_layout.grasp_set import GRASPS, MUS, SEEDS, GraspHand, object_set, run_pair  # noqa: E402

OUT = ROOT / "output" / "thumb_layout" / "grasp_set"
RERUN_FACTOR = 4
RERUN_SEED_OFFSET = 1000

_HANDS: dict[str, GraspHand] = {}
_SPECS: dict[str, str] = {}


def parse_hand(spec: str) -> tuple[str, Path, str]:
    tag, rest = spec.split("=", 1)
    path, _, thumb = rest.partition("@")
    path = Path(path) if Path(path).is_absolute() else ROOT / path
    return tag, path, thumb or "prototype"


def thumb_design(parameters, thumb: str) -> ThumbDesign:
    if thumb == "prototype":
        return ThumbDesign.prototype(parameters)
    parts = thumb.split(",")
    x, z, e = (float(v) for v in parts[:3])
    angles = [float(v) for v in parts[4:7]] + [0.0] * (3 - len(parts[4:7]))
    bend = float(parts[7]) if len(parts) > 7 and parts[7] else 0.0
    return ThumbDesign(x, z, e, parts[3], *angles, proximal_bend_deg=bend)


def thumb_limits(thumb: str):
    """The 9th field of a thumb spec, if given: the IP lower limit [deg] (hyperextension)."""
    parts = thumb.split(",")
    return {"ip": (float(parts[8]), LIMITS_DEG["ip"][1])} if len(parts) > 8 and parts[8] else None


def _init(specs: dict[str, str]) -> None:
    _SPECS.update(specs)


def _hand(tag: str) -> GraspHand:
    if tag not in _HANDS:
        _, path, thumb = parse_hand(_SPECS[tag])
        parameters, _ = load(path)
        _HANDS[tag] = GraspHand(parameters, thumb_design(parameters, thumb),
                                Evaluator(parameters, thumb_limits_deg=thumb_limits(thumb)))
    return _HANDS[tag]


def _job(args) -> dict:
    tag, obj_index, grasp, n_seeds, rng_seed = args
    shape = object_set()[obj_index]
    t = time.time()
    r = run_pair(_hand(tag), shape, grasp, n_seeds, rng_seed, tag).as_dict()
    r["seconds"] = round(time.time() - t, 1)
    r["rng_seed"] = rng_seed
    return r


def cmd_run(args) -> int:
    specs = dict((parse_hand(s)[0], s) for s in args.hand)
    objects = object_set()
    grasps = args.grasp or list(GRASPS)
    jobs = [(tag, i, g, args.seeds, args.seed) for tag in specs for i in range(len(objects)) for g in grasps]
    print(f"{len(jobs)} pairs, {args.workers} workers", flush=True)
    t0 = time.time()
    with Pool(args.workers, initializer=_init, initargs=(specs,)) as pool:
        results = pool.map(_job, jobs, chunksize=1)
        by_tag = {tag: [r for r in results if r["hand"] == tag] for tag in specs}
        for tag, rows in by_tag.items():
            d = OUT / tag / f"seed{args.seed}"
            d.mkdir(parents=True, exist_ok=True)
            tag_, path, thumb = parse_hand(specs[tag])
            (d / "hand.json").write_text(json.dumps({"tag": tag, "hand_file": str(path), "thumb": thumb,
                                                     "seeds": args.seeds, "seed": args.seed}, indent=1))
            (d / "pairs.json").write_text(json.dumps(rows, indent=1))
            n = sum(r["found"][str(MUS[0])] for r in rows)
            print(f"{tag} seed {args.seed}: {n}/{len(rows)} found at mu {MUS[0]} ({time.time() - t0:.0f} s)", flush=True)
        if args.rerun_misses:
            seed = args.seed + RERUN_SEED_OFFSET
            misses = [(r["hand"], [o.label for o in objects].index(r["object"]), r["grasp"], args.seeds * RERUN_FACTOR,
                       seed) for r in results if not all(r["found"].values())]
            print(f"rerunning {len(misses)} misses with {args.seeds * RERUN_FACTOR} seeds", flush=True)
            reruns = pool.map(_job, misses, chunksize=1)
            for tag in specs:
                rows = [r for r in reruns if r["hand"] == tag]
                (OUT / tag / f"seed{args.seed}" / "rerun.json").write_text(json.dumps(rows, indent=1))
                flips = {str(mu): sum(r["found"][str(mu)] for r in rows) for mu in MUS}
                print(f"{tag}: {len(rows)} reruns, flips to found {flips}", flush=True)
    print(f"done in {time.time() - t0:.0f} s")
    return 0


# ---------------------------------------------------------------- compare


def load_pairs(tag: str, seed: int, with_rerun: bool = True) -> dict[tuple[str, str], dict]:
    d = OUT / tag / f"seed{seed}"
    rows = {(r["object"], r["grasp"]): r for r in json.loads((d / "pairs.json").read_text())}
    if with_rerun and (d / "rerun.json").exists():
        for r in json.loads((d / "rerun.json").read_text()):
            key = (r["object"], r["grasp"])
            base = rows[key]
            merged = dict(base)
            merged["found"], merged["eps"], merged["best"] = dict(base["found"]), dict(base["eps"]), dict(base["best"])
            merged["rerun_flip"] = {}
            for mu in map(str, MUS):
                merged["rerun_flip"][mu] = r["found"][mu] and not base["found"][mu]
                if r["found"][mu] and r["eps"][mu] >= base["eps"][mu] or not base["found"][mu] and r["found"][mu]:
                    merged["eps"][mu] = r["eps"][mu]
                    merged["best"][mu] = r["best"][mu]
                merged["found"][mu] = base["found"][mu] or r["found"][mu]
            rows[key] = merged
    return rows


def size_ranges(rows: dict, mu: str) -> dict:
    out = {}
    for grasp in GRASPS:
        out[grasp] = {}
        for kind in ("sphere", "cylinder", "box"):
            sizes = [o.size_mm for o in object_set(False) if o.kind == kind
                     and rows[(o.label, grasp)]["found"][mu]]
            out[grasp][kind] = (min(sizes), max(sizes), len(sizes)) if sizes else None
    return out


def _med(v):
    return float(np.median(v)) if len(v) else float("nan")


def cmd_compare(args) -> int:
    tags = args.tags
    final = {t: load_pairs(t, args.seed) for t in tags}
    runs = {t: [load_pairs(t, s, with_rerun=False) for s in (args.seed, args.spread_seed)
                if (OUT / t / f"seed{s}" / "pairs.json").exists()] for t in tags}
    keys = list(next(iter(final.values())).keys())
    core = {o.label for o in object_set(False)}
    out = {"hands": {}, "pairs": len(keys)}
    for mu in map(str, MUS):
        common = [k for k in keys if all(final[t][k]["found"][mu] for t in tags)]
        spreads = []
        for t in tags:
            if len(runs[t]) == 2:
                m = [_med([r[k]["eps"][mu] if r[k]["found"][mu] else 0.0 for k in common]) for r in runs[t]]
                spreads.append(abs(m[0] - m[1]))
        spread = max(spreads) if spreads else float("nan")
        out[f"mu{mu}"] = {"common_pairs": len(common), "spread": spread}
        for t in tags:
            rows = final[t]
            feas = [k for k in keys if rows[k]["found"][mu]]
            eps_feas = [rows[k]["eps"][mu] for k in feas]
            entry = {
                "feasible": len(feas), "feasible_core": sum(k[0] in core for k in feas),
                "median_eps_common": _med([rows[k]["eps"][mu] for k in common]),
                "median_eps_all": _med([rows[k]["eps"][mu] if rows[k]["found"][mu] else 0.0 for k in keys]),
                "median_eps_feasible": _med(eps_feas), "worst_eps_feasible": min(eps_feas) if eps_feas else float("nan"),
                "median_eps_common_pinch": _med([rows[k]["eps"][mu] for k in common if k[1].startswith("pinch")]),
                "median_eps_common_multi": _med([rows[k]["eps"][mu] for k in common if not k[1].startswith("pinch")]),
                "by_grasp": {g: sum(rows[k]["found"][mu] for k in keys if k[1] == g) for g in GRASPS},
                "size_ranges": size_ranges(rows, mu),
                "rerun_flips": sum(bool(rows[k].get("rerun_flip", {}).get(mu)) for k in keys),
                "reruns": sum("rerun_flip" in rows[k] for k in keys),
                "spread_run": spreads[tags.index(t)] if len(spreads) == len(tags) else None,
            }
            out["hands"].setdefault(t, {})[f"mu{mu}"] = entry
    # the registered rule at mu 0.5
    mu = str(MUS[0])
    spread = out[f"mu{mu}"]["spread"]
    verdicts = []
    for i, a in enumerate(tags):
        for b in tags[i + 1:]:
            ea, eb = out["hands"][a][f"mu{mu}"], out["hands"][b][f"mu{mu}"]
            dn = ea["feasible"] - eb["feasible"]
            de = ea["median_eps_common"] - eb["median_eps_common"]
            by_n = a if dn >= 2 else b if dn <= -2 else None
            by_e = a if de > spread else b if de < -spread else None
            if by_n and by_e and by_n != by_e:
                verdict = f"{by_n} better (feasible pairs decide; {by_e} has the higher median eps)"
            elif by_n or by_e:
                verdict = f"{by_n or by_e} better"
            else:
                verdict = "not distinguished"
            verdicts.append({"a": a, "b": b, "d_feasible": dn, "d_median_eps": round(de, 4),
                             "spread": round(spread, 4), "verdict": verdict})
    out["verdicts"] = verdicts
    out["per_pair"] = {t: [{"object": k[0], "grasp": k[1], **{f"found_{m}": final[t][k]["found"][m] for m in map(str, MUS)},
                            **{f"eps_{m}": round(final[t][k]["eps"][m], 4) for m in map(str, MUS)},
                            "facing_min_deg": final[t][k].get("facing_min_deg"),
                            "pads": final[t][k]["best"].get(mu, {}).get("pads")} for k in keys] for t in tags}
    path = OUT / f"compare_{'_'.join(tags)}.json"
    path.write_text(json.dumps(out, indent=1))
    _print(out, tags)
    print(f"wrote {path}")
    return 0


def _print(out: dict, tags) -> None:
    for mu in map(str, MUS):
        print(f"\nmu {mu}: common pairs {out[f'mu{mu}']['common_pairs']}, spread {out[f'mu{mu}']['spread']:.4f}")
        print("| hand | feasible (core) | " + " | ".join(GRASPS) + " | median eps common (pinch/multi) | median all | worst |")
        for t in tags:
            e = out["hands"][t][f"mu{mu}"]
            print(f"| {t} | {e['feasible']} ({e['feasible_core']}) | " + " | ".join(str(e["by_grasp"][g]) for g in GRASPS)
                  + f" | {e['median_eps_common']:.4f} ({e['median_eps_common_pinch']:.4f}/{e['median_eps_common_multi']:.4f})"
                  f" | {e['median_eps_all']:.4f} | {e['worst_eps_feasible']:.4f} |")
    print()
    for t in tags:
        e = out["hands"][t][f"mu{MUS[0]}"]
        ranges = {g: {k: (f"{v[0]:g}-{v[1]:g} ({v[2]})" if v else "-") for k, v in r.items()}
                  for g, r in e["size_ranges"].items()}
        print(t, json.dumps(ranges), f"reruns {e['reruns']}, flips {e['rerun_flips']}")
    for v in out["verdicts"]:
        print(v)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--hand", action="append", required=True, help="tag=hand.yaml@thumb (repeatable)")
    r.add_argument("--grasp", action="append", choices=list(GRASPS))
    r.add_argument("--seed", type=int, default=0)
    r.add_argument("--seeds", type=int, default=SEEDS)
    r.add_argument("--workers", type=int, default=1)
    r.add_argument("--rerun-misses", action="store_true")
    c = sub.add_parser("compare")
    c.add_argument("tags", nargs="+")
    c.add_argument("--seed", type=int, default=0)
    c.add_argument("--spread-seed", type=int, default=1)
    args = ap.parse_args(argv)
    return cmd_run(args) if args.cmd == "run" else cmd_compare(args)


if __name__ == "__main__":
    raise SystemExit(main())
