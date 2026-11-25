"""Experiment 34: the pad-shape interface (flat -> 2 mm crowned) in the Drake pull-out, each shape with its own
placements (docs/experimental/drake_shape.md).

Isaac Lab's experiment 8 placed the cylinder where O's FLAT pads touch it and ran crowned pads there; Gate 0 collapsed
(O 56 -> 17 of 132). Here each shape is placed by O's own contact on that shape, and the pair is (radius, q0,
direction): the same cylinder radius and the same commanded setpoint q0 + 5 deg, the cylinder where O's two pads of
that shape both touch it at q0.

- Pads: every pad (both, every hand) is a convex prism with its box's envelope (length, width, thickness, mounting
  face); 'flat' is the box itself as a convex mesh, 'crowned' has its contact face a circular arc along the pad whose
  middle keeps the full thickness and whose ends lie 2 mm higher (sagitta 2 mm, 24 chords), the profile of
  sim/isaaclab contact_interface.CROWNED. Both shapes go through the same convex-mesh collision path. The pads are
  made by substituting pydrake.geometry.Box inside two_pad_drake._build (two_pad_drake is not edited).
- Placement (made on O in the planar world frame, given to every hand in its own proximal-face frame, as experiment 21): flat, the rule of two_pad_grasps.json
  (centre at depth face + rho, rho from the distal face line at q0); crowned, the centre at distance R + rho from both
  arcs' centres (R = (L/2)^2 / (2 c) + c / 2), the palmar intersection. A placement is kept only if both contacts lie
  on the pads (within their length).
- Run: condition A as experiment 21 (point contact, K_Q 5, the setpoint q0 + 5 deg, the cylinder held while the finger
  closes 40 + 30 steps, released, settled 120 steps, Gate 0, the load ramp in four directions, each branched from the
  settled state), at mu 1.3. Gate 0: both pads >= 0.2 N, still, and each contact on its pad's face (its point within
  the pad's length; the flat rule's normal-angle check is replaced by this, which also holds for a crowned pad).
- Measure: F_limit, and the load-displacement curve (load, displacement along the load) for the deviation figure.
  D(H) = F_limit(crowned) - F_limit(flat) per (rho, q0, direction); E(H) = median |D(H) - D(O)| / |D(O)|.

    uv run python pad_shape.py place [--radii 10 12.5 15 17.5 20]
    uv run python pad_shape.py run --hands O --tag e34 [--only 6]
    uv run python pad_shape.py analyze --tag e34 --noise_tag e34n --out logs/e34_shape.json
"""
from __future__ import annotations

import argparse
import json
import math
import time
from functools import lru_cache
from multiprocessing import Pool
from pathlib import Path

import numpy as np

import two_pad_drake as T

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
MESH = LOGS / "pad_shape_meshes"
CROWN_MM, SEGMENTS = 2.0, 24
MU = 1.3
SHAPES = ("flat", "crowned")
PLACES = LOGS / "pad_shape_places.json"
FACE_O_MM = 6.0  # O's proximal face depth below the link line (mount 3 + pad 3 mm)


# ----------------------------------------------------------------------------------------------------- pads
def pad_obj(length, width, height, crown) -> Path:
    """A convex prism (m) centred at the origin like Box(length, width, height); its contact face (-z) crowned."""
    MESH.mkdir(parents=True, exist_ok=True)
    p = MESH / f"pad_{length * 1e3:.3f}_{width * 1e3:.3f}_{height * 1e3:.3f}_{crown * 1e3:.2f}.obj"
    if p.exists():
        return p
    hx, hz = length / 2, height / 2
    if crown > 0:
        R = hx ** 2 / (2 * crown) + crown / 2
        xs = np.linspace(-hx, hx, SEGMENTS + 1)
        zc = -hz + (R - np.sqrt(R ** 2 - xs ** 2))  # contact face: -hz at the middle, -hz + crown at the ends
    else:
        xs, zc = np.array([-hx, hx]), np.array([-hz, -hz])
    prof = [(x, z) for x, z in zip(xs, zc)] + [(hx, hz), (-hx, hz)]  # contact arc, then the mounting face
    v = [(x, y, z) for x, z in prof for y in (-width / 2, width / 2)]
    with open(p, "w") as f:
        for a in v:
            f.write(f"v {a[0]:.9f} {a[1]:.9f} {a[2]:.9f}\n")
        n = len(prof)
        for i in range(n):  # side quads (the hull is what Drake uses; faces only need to exist)
            j = (i + 1) % n
            f.write(f"f {2 * i + 1} {2 * j + 1} {2 * j + 2}\nf {2 * i + 1} {2 * j + 2} {2 * i + 2}\n")
        for k in range(1, n - 1):  # end caps
            f.write(f"f 1 {2 * k + 1} {2 * k + 3}\nf 2 {2 * k + 4} {2 * k + 2}\n")
    return p


def use_shape(shape: str) -> None:
    """Make two_pad_drake._build's pad boxes convex prisms of this shape (the roller is a Cylinder, untouched)."""
    import pydrake.geometry as G

    if not hasattr(G, "_Box_orig"):
        G._Box_orig = G.Box
    crown = CROWN_MM * 1e-3 if shape == "crowned" else 0.0
    G.Box = lambda l, w, h: G.Convex(str(pad_obj(l, w, h, crown)), 1.0)


# ----------------------------------------------------------------------------------------------------- placements
def place(hand, shape, rho, q0_deg):
    """The cylinder centre (world x, z mm) where the hand's two pads of this shape touch it at flexion q0, its
    contact points and whether both lie on the pads; None if no such placement."""
    g = T.finger_geometry(hand)
    qs, th = T.servo_table(g)
    pads = g["pads"]
    m, h = pads["mount_offset_mm"], pads["thickness_mm"]
    face = m + h
    s0, s1, t0, t1 = pads["proximal_start_mm"], pads["proximal_end_mm"], pads["distal_start_mm"], pads["distal_end_mm"]
    q = math.radians(q0_deg)
    theta = float(np.interp(q, qs, th))
    B = g["ap"] + T.rot(theta) @ (g["b"] - g["ap"])
    Rq = T.rot(q)
    to_w = lambda pd: B + Rq @ pd  # distal frame -> world
    if shape == "flat":
        from dial_placements import x_contact

        xc = x_contact(g, qs, th, q0_deg, rho)
        if xc is None:
            return None
        c = np.array([xc, -(face + rho)])
        cp = np.array([xc, -face])
        nd = Rq @ np.array([0.0, -1.0])
        cd = c - rho * nd
    else:
        c_ = CROWN_MM

        def circ(L):
            return (L / 2) ** 2 / (2 * c_) + c_ / 2
        Rp, Rd = circ(s1 - s0), circ(t1 - t0)
        Cp = np.array([(s0 + s1) / 2, -face + Rp])
        Cd = to_w(np.array([(t0 + t1) / 2, -face + Rd]))
        r1, r2 = Rp + rho, Rd + rho
        d = np.linalg.norm(Cd - Cp)
        if d > r1 + r2 or d < abs(r1 - r2):
            return None
        a = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
        hh = math.sqrt(max(r1 ** 2 - a ** 2, 0.0))
        u = (Cd - Cp) / d
        perp = np.array([-u[1], u[0]])
        cands = [Cp + a * u + hh * perp, Cp + a * u - hh * perp]
        c = min(cands, key=lambda v: v[1])  # the palmar one (below both pads)
        cp = Cp + Rp * (c - Cp) / np.linalg.norm(c - Cp)
        cd = Cd + Rd * (c - Cd) / np.linalg.norm(c - Cd)
    # contacts on the pads: proximal x within [s0, s1]; distal, its distal-frame x within [t0, t1]
    xd = float((Rq.T @ (cd - B))[0])
    on = bool(s0 <= cp[0] <= s1 and t0 <= xd <= t1)
    return {"c_mm": c.tolist(), "contact_p_mm": cp.tolist(), "contact_d_mm": cd.tolist(), "on_pads": on,
            "contact_p_x": float(cp[0]), "contact_d_x": xd}


def cmd_place(a) -> int:
    rows = []
    for rho in a.radii:
        for q0 in np.arange(a.q_lo, a.q_hi + 1e-9, a.q_step):
            r = {"rho_mm": float(rho), "q0_deg": float(q0)}
            for shape in SHAPES:
                r[shape] = place("O", shape, rho, q0)
            ok = all(r[s] is not None and r[s]["on_pads"] for s in SHAPES)
            if ok:
                rows.append(r)
    PLACES.write_text(json.dumps({"radii": a.radii, "q": [a.q_lo, a.q_hi, a.q_step], "crown_mm": CROWN_MM,
                                  "places": rows}, indent=1))
    for rho in a.radii:
        n = [r for r in rows if r["rho_mm"] == rho]
        print(f"rho {rho}: {len(n)} placements with both shapes' contacts on O's pads (q0 "
              f"{min((r['q0_deg'] for r in n), default=0):.0f}-{max((r['q0_deg'] for r in n), default=0):.0f}); "
              f"crowned - flat centre shift (mm): " +
              " ".join(f"{np.subtract(r['crowned']['c_mm'], r['flat']['c_mm']).round(1).tolist()}" for r in n[:3]))
    print(f"{len(rows)} placements -> {PLACES.name}")
    return 0


# ----------------------------------------------------------------------------------------------------- run
def settle(hand, shape, rho, c_mm, q_cmd_deg, time_step=1.0e-3):
    use_shape(shape)
    g = T.finger_geometry(hand)
    qs, th = T.servo_table(g, 140.0)
    q_cmd = math.radians(q_cmd_deg)
    theta_cmd = float(np.interp(q_cmd, qs, th))
    ratio = float(np.interp(q_cmd, qs, np.gradient(th, qs)))
    k_theta, d_theta = T.K_Q / ratio ** 2, T.D_Q / ratio ** 2
    # placements are made on O in the world (link-line) frame; every hand gets them in its own proximal-face frame,
    # as experiment 21 (centre depth face_H + rho): z moves by O's face depth minus the hand's
    pads = g["pads"]
    face_h = pads["mount_offset_mm"] + pads["thickness_mm"]
    x0 = np.array([c_mm[0], c_mm[1] + FACE_O_MM - face_h]) * 1e-3
    s1 = T.Stage(hand, MU, rho, "point", time_step, True, k_theta, d_theta)
    s1.plant.GetJointByName("slide_x").set_translation(s1.pctx, x0[0])
    s1.plant.GetJointByName("slide_z").set_translation(s1.pctx, x0[1])
    s1.ctrl.hold_at = np.array([x0[0], x0[1], 0.0])
    s1.sim.Initialize()
    for i in range(40):
        s1.ctrl.theta_cmd = theta_cmd * (i + 1) / 40
        s1.step()
    for _ in range(30):
        s1.step()
    state = s1.plant.GetPositionsAndVelocities(s1.pctx).copy()
    s = T.Stage(hand, MU, rho, "point", time_step, False, k_theta, d_theta)
    s.plant.SetPositionsAndVelocities(s.pctx, state)
    s.ctrl.theta_cmd = theta_cmd
    s.sim.Initialize()
    speed = spin = 0.0
    for k in range(120):
        s.step()
        if k >= 110:
            m_ = s.measure()
            speed = max(speed, 1e3 * float(np.linalg.norm(m_["v"][[0, 2]])))
            spin = max(spin, abs(m_["w"]))
    m_ = s.measure()
    pads = g["pads"]
    on = [None, None]
    if np.all(np.isfinite(m_["point"][0])):
        on[0] = bool(pads["proximal_start_mm"] - 0.5 <= 1e3 * m_["point"][0][0] <= pads["proximal_end_mm"] + 0.5)
    if np.all(np.isfinite(m_["point"][1])):
        Xd = s.plant.EvalBodyPoseInWorld(s.pctx, s.bodies["distal"])
        xd = 1e3 * float((Xd.inverse() @ m_["point"][1])[0])
        on[1] = bool(pads["distal_start_mm"] - 0.5 <= xd <= pads["distal_end_mm"] + 0.5)
    pads_ok = bool((m_["normal"] >= T.GATE_N).all())
    still = speed < T.GATE_SPEED_MM_S and spin < T.GATE_SPIN_RAD_S
    gate = {"pass": bool(pads_ok and still and on[0] and on[1]), "pads": pads_ok, "still": bool(still),
            "on_face": on, "speed_mm_s": float(speed)}
    pre = {"normal_n": m_["normal"].tolist(), "q_deg": math.degrees(m_["q"]), "pen_mm": float(m_["normal"].max()) / 2.5}
    return s, m_, gate, pre, (k_theta, d_theta, theta_cmd)


def ramp(s, m0, direction):
    d = np.array(T.DIRECTIONS[direction])
    c0 = m0["c"].copy()
    limit = {"event": None, "f_limit_n": None}
    curve = []
    steps = int(T.MAX_FORCE_N / (T.RATE_N_S * T.DT_ENV)) + 1
    for i in range(steps):
        f = T.RATE_N_S * T.DT_ENV * (i + 1)
        s.ctrl.load = f * d
        s.step()
        m_ = s.measure()
        dv = 1e3 * (m_["c"] - c0)[[0, 2]]
        disp = float(np.linalg.norm(dv))
        curve.append([round(f * T.FINGERS, 4), round(float(dv @ d), 4), round(disp, 4)])
        event = None
        k = int(np.argmax(m_["slip"]))
        if 1e3 * m_["slip"][k] > T.SLIP_MM_S:
            event = "slip"
        elif (m_["normal"] < T.LOSS_N).any():
            event = "loss"
        elif disp > T.COLLAPSE_MM:
            event = "collapse"
        if event:
            limit = {"event": event, "f_limit_n": float(f * T.FINGERS)}
            break
    return limit, curve


def run_task(task):
    hand, shape, pl, delta = task
    t0 = time.time()
    rho, q0 = pl["rho_mm"], pl["q0_deg"]
    rec = {"hand": hand, "shape": shape, "rho_mm": rho, "q0_deg": q0}
    try:
        s, m_, gate, pre, (kt, dt, th) = settle(hand, shape, rho, pl[shape]["c_mm"], q0 + 5.0 + delta)
        rec.update(gate0=gate, pre=pre, dirs={})
        if gate["pass"]:
            state = s.plant.GetPositionsAndVelocities(s.pctx).copy()
            for direction in T.DIRECTIONS:
                b = T.Stage(hand, MU, rho, "point", 1.0e-3, False, kt, dt)
                b.plant.SetPositionsAndVelocities(b.pctx, state)
                b.ctrl.theta_cmd = th
                b.sim.Initialize()
                lim, curve = ramp(b, b.measure(), direction)
                rec["dirs"][direction] = {"limit": lim, "curve": curve}
    except RuntimeError as err:
        rec["solver_fail"] = str(err)[:200]
    rec["wall_s"] = time.time() - t0
    return rec


def cmd_run(a) -> int:
    places = json.loads(PLACES.read_text())["places"][: a.only]
    tasks = [(h, sh, pl, a.delta_deg) for h in a.hands for pl in places for sh in SHAPES]
    t0 = time.time()
    with Pool(a.workers) as pool:
        recs = pool.map(run_task, tasks, chunksize=1)
    for h in a.hands:
        rr = [r for r in recs if r["hand"] == h]
        (LOGS / f"shape_{h}_{a.tag}.json").write_text(json.dumps({"hand": h, "tag": a.tag, "mu": MU,
                                                                   "delta_deg": a.delta_deg, "runs": rr}))
        g0 = {sh: sum(r.get("gate0", {}).get("pass", False) for r in rr if r["shape"] == sh) for sh in SHAPES}
        print(f"{h} {a.tag}: {len(places)} placements, Gate 0 {g0}, cpu {sum(r['wall_s'] for r in rr):.0f} s",
              flush=True)
    print(f"done in {time.time() - t0:.0f} s (workers {a.workers})", flush=True)
    return 0


# ----------------------------------------------------------------------------------------------------- analysis
PEN_MM = 1.5
D_MIN_N = 0.6  # nine load steps (0.0667 N each, three fingers); 0.2 and 1.0 N as sensitivity


def _runs(hand, tag):
    p = LOGS / f"shape_{hand}_{tag}.json"
    if not p.exists():
        return None
    out = {}
    for r in json.loads(p.read_text())["runs"]:
        out[(r["rho_mm"], r["q0_deg"], r["shape"])] = r
    return out


def _flim(r, d):
    if r is None or r.get("solver_fail") or not r["gate0"]["pass"]:
        return None
    return r["dirs"][d]["limit"]["f_limit_n"]


def o_pairs(tag):
    o = _runs("O", tag)
    keys = sorted({k[:2] for k in o})
    keep, excl = {}, {"o_solver": 0, "o_gate0_flat": 0, "o_gate0_crowned": 0, "o_pen": 0, "o_no_limit": 0,
                      "small_d": 0}
    for k in keys:
        rf, rc = o.get(k + ("flat",)), o.get(k + ("crowned",))
        for d in T.DIRECTIONS:
            if any(r is None or r.get("solver_fail") for r in (rf, rc)):
                excl["o_solver"] += 1
            elif not rf["gate0"]["pass"]:
                excl["o_gate0_flat"] += 1
            elif not rc["gate0"]["pass"]:
                excl["o_gate0_crowned"] += 1
            elif max(rf["pre"]["pen_mm"], rc["pre"]["pen_mm"]) > PEN_MM:
                excl["o_pen"] += 1
            elif _flim(rf, d) is None or _flim(rc, d) is None:
                excl["o_no_limit"] += 1
            elif abs(_flim(rc, d) - _flim(rf, d)) < D_MIN_N - 1e-9:
                excl["small_d"] += 1
            else:
                keep[k + (d,)] = (_flim(rc, d) - _flim(rf, d), rf, rc)
    return keep, excl


def hand_error(hand, tag, keep):
    h = _runs(hand, tag)
    if h is None:
        return None
    rel, sens, pens = [], [], []
    cnt = {"solver": 0, "gate0": 0, "over_pressed": 0, "no_limit": 0}
    for k, (d_o, _, _) in keep.items():
        rf, rc = h.get(k[:2] + ("flat",)), h.get(k[:2] + ("crowned",))
        if any(r is None or r.get("solver_fail") for r in (rf, rc)):
            cnt["solver"] += 1
            sens.append(math.inf)
            continue
        if not (rf["gate0"]["pass"] and rc["gate0"]["pass"]):
            cnt["gate0"] += 1
            sens.append(math.inf)
            continue
        pen = max(rf["pre"]["pen_mm"], rc["pre"]["pen_mm"])
        pens.append(pen)
        if pen > PEN_MM:
            cnt["over_pressed"] += 1
            sens.append(math.inf)
            continue
        ff, fc = _flim(rf, k[2]), _flim(rc, k[2])
        if ff is None or fc is None:
            cnt["no_limit"] += 1
            continue
        e = abs((fc - ff) - d_o) / abs(d_o)
        rel.append(e)
        sens.append(e)
    return {"E": float(np.median(rel)) if rel else None, "E_p75": float(np.percentile(rel, 75)) if rel else None,
            "pairs": len(rel), "E_sens": float(np.median(sens)) if sens else None, **cnt,
            "pen_max_mm": float(max(pens)) if pens else None}


def deviation(tag, keep):
    """R and U minus O, held load (the largest load sustained up to each displacement along the load) on a 0-5 mm
    grid, per shape, over O's kept pairs (fig4_sync's pull-out 'deviation' with shapes in place of frictions)."""
    grid = np.linspace(0.0, 5.0, 101)

    def held(curve):
        c = np.asarray(curve)
        dd = np.maximum.accumulate(np.maximum(c[:, 1], 0.0))
        return np.interp(grid, dd, c[:, 0], right=c[-1, 0])
    o = _runs("O", tag)
    out = {"x": grid.round(3).tolist()}
    for hand in ("R", "U"):
        h = _runs(hand, tag)
        for shape in SHAPES:
            ys = []
            for k in keep:
                ro, rh = o[k[:2] + (shape,)], h.get(k[:2] + (shape,))
                if rh is None or rh.get("solver_fail") or not rh["gate0"]["pass"]:
                    continue
                if max(rh["pre"]["pen_mm"], 0) > PEN_MM:
                    continue
                ys.append(held(rh["dirs"][k[2]]["curve"]) - held(ro["dirs"][k[2]]["curve"]))
            y = np.array(ys)
            out[f"{hand}_{shape}"] = {"median": np.median(y, 0).round(4).tolist(),
                                     "q25": np.percentile(y, 25, 0).round(4).tolist(),
                                     "q75": np.percentile(y, 75, 0).round(4).tolist(), "n": len(ys)}
    return out


def cmd_analyze(a) -> int:
    from analyze import DESIGN, spearman_perm
    global D_MIN_N
    if a.d_min is not None:
        D_MIN_N = a.d_min
    keep, excl = o_pairs(a.tag)
    d_o = np.array([v[0] for v in keep.values()])
    places = {k[:2] for k in keep}
    print(f"{a.tag}: O pairs {len(keep)} ({len(places)} placements), excluded {excl}; D(O) "
          f"{d_o.min():+.2f}/{np.median(d_o):+.2f}/{d_o.max():+.2f} N (negative {int((d_o < 0).sum())})")
    out = {"tag": a.tag, "o_pairs": len(keep), "o_excluded": excl, "d_o_n": d_o.tolist(), "rows": []}
    if a.noise_tag:
        nz = hand_error("O", a.noise_tag, keep)
        out["noise_floor"] = nz
        print(f"noise floor ({a.noise_tag}): E {100 * nz['E']:.2f}% (p75 {100 * nz['E_p75']:.2f}%), pairs {nz['pairs']}")
    for hand, (dc, ds) in DESIGN.items():
        r = hand_error(hand, a.tag, keep)
        if r is None:
            continue
        out["rows"].append({"hand": hand, "d_contact": dc, "d_skel": ds, **r})
        print(f"{hand:22s} Dc {dc:4.2f} Ds {ds:5.2f} | E {'-' if r['E'] is None else f'{100 * r[chr(69)]:5.1f}%'} | "
              f"pairs {r['pairs']}/{len(keep)} gate0 {r['gate0']} over {r['over_pressed']} no_limit "
              f"{r['no_limit']} | pen max {r['pen_max_mm'] or 0:.2f}", flush=True)
    rows = [r for r in out["rows"] if r["E"] is not None]
    if len(rows) >= 3:
        st = {}
        for key in ("E", "E_sens"):
            for cand in ("d_contact", "d_skel"):
                rho, p = spearman_perm([r[cand] for r in rows], [r[key] for r in rows])
                st[f"rho_{key}_{cand}"], st[f"p_{key}_{cand}"] = rho, p
            print(f"rho({key}, D_contact) {st[f'rho_{key}_d_contact']:.2f} (permutation p "
                  f"{st[f'p_{key}_d_contact']:.4f}); rho({key}, D_skel) {st[f'rho_{key}_d_skel']:.2f} "
                  f"(p {st[f'p_{key}_d_skel']:.4f}); hands {len(rows)}")
        out["stats"] = st
    if _runs("R", a.tag) and _runs("U", a.tag):
        out["deviation"] = deviation(a.tag, keep)
        for k in ("R_flat", "U_flat", "R_crowned", "U_crowned"):
            m = np.array(out["deviation"][k]["median"])
            i = int(np.argmax(np.abs(m)))
            print(f"deviation {k}: peak {m[i]:+.3f} N at {out['deviation']['x'][i]:.2f} mm, end {m[-1]:+.3f} N, "
                  f"n {out['deviation'][k]['n']}")
    if a.out:
        a.out.write_text(json.dumps(out, indent=1, default=float))
    return 0


def cmd_check(a) -> int:
    """Gates only (no D, no E): Gate 0 per shape, penetration, solver."""
    for hand in a.hands:
        h = _runs(hand, a.tag)
        for shape in SHAPES:
            rr = [r for k, r in h.items() if k[2] == shape]
            g0 = sum(r.get("gate0", {}).get("pass", False) for r in rr)
            pens = [r["pre"]["pen_mm"] for r in rr if "pre" in r and r["gate0"]["pass"]]
            fail = sum("solver_fail" in r for r in rr)
            off = sum(1 for r in rr if "gate0" in r and r["gate0"]["pads"] and not all(r["gate0"]["on_face"]))
            print(f"{hand} {shape:8s}: Gate 0 {g0}/{len(rr)} (contact off the face {off}), solver {fail}, pen max "
                  f"{max(pens) if pens else 0:.2f} mm (> 1.5: {sum(p > PEN_MM for p in pens)})")
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    pp = sub.add_parser("place")
    pp.add_argument("--radii", type=float, nargs="+", default=[10.0, 12.5, 15.0, 17.5, 20.0])
    pp.add_argument("--q_lo", type=float, default=40.0)
    pp.add_argument("--q_hi", type=float, default=115.0)
    pp.add_argument("--q_step", type=float, default=5.0)
    pr = sub.add_parser("run")
    pr.add_argument("--hands", nargs="+", default=["O"])
    pr.add_argument("--tag", required=True)
    pr.add_argument("--only", type=int, default=None)
    pr.add_argument("--delta_deg", type=float, default=0.0)
    pr.add_argument("--workers", type=int, default=6)
    pa = sub.add_parser("analyze")
    pa.add_argument("--tag", required=True)
    pa.add_argument("--noise_tag", default=None)
    pa.add_argument("--out", type=Path, default=None)
    pa.add_argument("--d_min", type=float, default=None, help="|D(O)| floor (N), default D_MIN_N")
    pc = sub.add_parser("check")
    pc.add_argument("--tag", required=True)
    pc.add_argument("--hands", nargs="+", default=["O"])
    a = p.parse_args()
    return {"place": cmd_place, "run": cmd_run, "analyze": cmd_analyze, "check": cmd_check}[a.cmd](a)


if __name__ == "__main__":
    raise SystemExit(main())
