"""E(H) for the Drake replica, defined exactly as Isaac Lab's dose analysis (sim/isaaclab/scripts/study3_measurement/
dose_analysis.py): over placement-direction pairs where O and H both pass Gate 0 at both mu and have a limit,
D = F_limit(2.0) - F_limit(1.3), E(H) = median |D(H) - D(O)| / |D(O)|, pooled over directions and radii. Prints the
Isaac Lab value (condition A, point contact) beside it, and Spearman rho(E, D_contact) with an exact one-sided
permutation p over the redesigned hands present.

    uv run python analyze.py --contact point --tag rep [--isaac]
"""
from __future__ import annotations

import argparse
import glob
import itertools
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parent / "isaaclab" / "logs" / "experimental" / "two_pad"
# D_contact, D_skel (mm) of the hands, from dose_analysis.py
DESIGN = {"R": (0.86, 7.67), "d1p5_minDs_s0": (1.60, 5.37), "d2p25_minDs_s0": (2.33, 4.62),
          "d2p25_maxDs_s0": (2.30, 9.17), "d3_minDs_s0": (3.09, 3.83), "U": (3.91, 3.38), "d4p5_s0": (4.41, 11.33),
          "d5p5_s0": (5.40, 12.17), "d1p5_maxDs_s0": (1.46, 9.67), "d3_maxDs_s0": (3.04, 11.55),
          "h3to5_hr": (0.35, 3.55), "h3to5_hu": (1.58, 2.02), "h3to5_d3_s0": (2.92, 6.38), "h3to5_d3_minDs_s0": (3.09, 0.38),
          "h3to9_hr": (2.32, 9.37), "h3to9_hu": (3.89, 7.17), "h3to9_d3p5_maxDs_s0": (3.52, 12.72),
          "h3to9_d5_minDs_s0": (5.08, 5.32)}
RADII, MU = (15.0, 20.0), (1.3, 2.0)


def key(e: dict) -> tuple:
    return (e["rho_mm"], e["q0_deg"], round(float(e["x_c_mm"]), 3), e["direction"])


def load_drake(hand: str, mu: float, contact: str, tag: str, r: float):
    p = HERE / "logs" / f"dk_{hand}_mu{mu:g}_{contact}{('-' + tag) if tag else ''}_r{r:g}.json"
    return json.loads(p.read_text())["envs"] if p.exists() else None


def load_isaac(hand: str, mu: float, r: float):
    fs = sorted(glob.glob(str(ISAAC / f"tp_{hand}_mu{mu:g}_A_r{r:g}_*.json")))
    if not fs:
        return None
    envs = json.loads(Path(fs[-1]).read_text())["envs"]
    for e in envs:
        e.setdefault("rho_mm", r)
    return envs


def error(loader, hand: str) -> tuple[float | None, int]:
    rel = []
    for r in RADII:
        runs = {(h, m): loader(h, m, r) for h in ("O", hand) for m in MU}
        if any(v is None for v in runs.values()):
            return None, 0
        by = {hm: {key(e): e for e in v} for hm, v in runs.items()}
        for k in by[("O", MU[0])]:
            es = [by[hm].get(k) for hm in by]
            if any(e is None or not e["gate0"]["pass"] or e["limit"]["f_limit_n"] is None for e in es):
                continue
            o_lo, o_hi, h_lo, h_hi = (by[hm][k]["limit"]["f_limit_n"] for hm in
                                      (("O", MU[0]), ("O", MU[1]), (hand, MU[0]), (hand, MU[1])))
            d_o, d_h = o_hi - o_lo, h_hi - h_lo
            if abs(d_o) < 1e-9:
                continue
            rel.append(abs(d_h - d_o) / abs(d_o))
    return (float(np.median(rel)) if rel else None), len(rel)


def spearman_perm(x, y) -> tuple[float, float]:
    def rank(v):
        return np.argsort(np.argsort(v)).astype(float)
    x, y = np.asarray(x), np.asarray(y)
    rho = float(np.corrcoef(rank(x), rank(y))[0, 1])
    if len(x) <= 9:
        perms = [float(np.corrcoef(rank(x), rank(np.array(p)))[0, 1]) for p in itertools.permutations(y)]
    else:
        rng = np.random.default_rng(0)
        perms = [float(np.corrcoef(rank(x), rank(rng.permutation(y)))[0, 1]) for _ in range(20000)]
    return rho, float(np.mean(np.array(perms) >= rho - 1e-12))


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--contact", default="point")
    p.add_argument("--tag", default="")
    p.add_argument("--isaac", action="store_true", help="also Isaac Lab's condition A (point contact)")
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()
    rows = []
    for hand, (dc, ds) in DESIGN.items():
        e, n = error(lambda h, m, r: load_drake(h, m, args.contact, args.tag, r), hand)
        if e is None:
            continue
        ei, ni = error(load_isaac, hand) if args.isaac else (None, 0)
        rows.append({"hand": hand, "d_contact": dc, "d_skel": ds, "E": e, "pairs": n, "E_isaac": ei, "pairs_isaac": ni})
        print(f"{hand:22s} D_c {dc:4.2f} D_s {ds:5.2f} | Drake {args.contact}: E {100 * e:6.1f}% ({n:3d} pairs)"
              + (f" | Isaac point: E {100 * ei:6.1f}% ({ni} pairs)" if ei is not None else ""))
    if len(rows) >= 3:
        rho, pv = spearman_perm([r["d_contact"] for r in rows], [r["E"] for r in rows])
        rho_s, pv_s = spearman_perm([r["d_skel"] for r in rows], [r["E"] for r in rows])
        print(f"Drake {args.contact}: rho(E, D_contact) {rho:.2f} (one-sided p {pv:.4f}); rho(E, D_skel) {rho_s:.2f} "
              f"(p {pv_s:.4f}); hands {len(rows)}")
    if args.out:
        args.out.write_text(json.dumps(rows, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
