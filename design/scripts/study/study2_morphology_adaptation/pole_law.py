#!/usr/bin/env python3
"""Study 0 and its checks (problem formulation A.6.0, A.6.3): what the joint takes from the pads.

    python scripts/study/study2_morphology_adaptation/pole_law.py [--tag name]

For every Study 1 design at hand (the finger optimized for a pad thickness and
a required flexion, A.6.2), with no search:

- the finite rotation pole ``P(q)`` of the distal phalanx's motion (the point
  about which the pose at flexion ``q`` is the extended pose turned by ``q``),
  from the fingertip's path alone, the same for every mechanism;
- the first flexion at which the two pads, as the rectangles they are, meet
  when the distal one is turned about ``P(q)`` (exact, no corner assumption);
- the joint gap ``G`` (the finger's length between the two pads) the law
  asks for the required flexion, in closed form (corners) and exactly (the
  least gap about the pole with no contact up to ``q_req``).

- between the designs for adjacent thicknesses, how far the motion moved: the
  skeleton's (the fingertip's path) and the contact's (one contact face seen
  from the other).

- each design as it is with pads of the other thicknesses: where the law says
  the pads first meet, against the evaluator's reach.

Writes ``output/study/study2_pole_law/designs.csv``, ``poles.csv``, ``changes.csv`` (Study 2)
and ``fixed_finger.csv``, and
``report/study2/figures/study2_pole_law_<tag>.png``.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
for path in (REPOSITORY_ROOT, REPOSITORY_ROOT / "scripts" / "study", Path(__file__).resolve().parent):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import one_finger as study2  # noqa: E402
from core.optimization.analysis.contact_kinematics import tip_path  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402
from visualize.style import COLORS, GRID, INK, MARKERS, use  # noqa: E402

FIGURES = REPOSITORY_ROOT.parent / "report" / "study2" / "figures"
OUTPUT = REPOSITORY_ROOT / "output" / "study" / "study2_pole_law"
NAMES = {"cfl": "CFL", "2r_remote": "2R-remote", "2r_dd": "2R-DD", "rcj": "RCJ", "cfl_ideal": "CFL ideal*",
         "rcj_ideal": "RCJ ideal*"}
STEP_DEG = 0.05


# The motion and its pole ----------------------------------------------------------

def poles(tips: np.ndarray, q_rad: np.ndarray) -> np.ndarray:
    """The pole ``P(q)`` of each pose against the extended one: the distal
    phalanx turns by ``-q`` (flexion is clockwise, toward the palm), so
    ``tip(q) - P = R(-q) (tip(0) - P)``. ``q = 0`` has none (NaN)."""

    found = np.full((len(q_rad), 2), np.nan)
    for k, q in enumerate(q_rad):
        if q <= 1e-9:
            continue
        c, s = math.cos(-q), math.sin(-q)
        rotation = np.array([[c, -s], [s, c]])
        found[k] = np.linalg.solve(np.eye(2) - rotation, tips[k] - rotation @ tips[0])
    return found


# The pads as rectangles -----------------------------------------------------------

def rectangle(x0: float, x1: float, top: float, bottom: float) -> np.ndarray:
    return np.array([(x0, bottom), (x1, bottom), (x1, top), (x0, top)], dtype=float)


def overlap(first: np.ndarray, second: np.ndarray) -> float:
    """How deep two convex polygons overlap (separating axes; <= 0: apart by that much)."""

    best = -math.inf
    for polygon in (first, second):
        edges = np.roll(polygon, -1, axis=0) - polygon
        for ex, ey in edges:
            length = math.hypot(ex, ey)
            if length <= 0.0:
                continue
            normal = np.array((ey, -ex)) / length
            a, b = first @ normal, second @ normal
            best = max(best, max(b.min() - a.max(), a.min() - b.max()))
    return -best


def turned(polygon: np.ndarray, pole: np.ndarray, q: float) -> np.ndarray:
    c, s = math.cos(-q), math.sin(-q)
    return (polygon - pole) @ np.array([[c, -s], [s, c]]).T + pole


def first_contact_deg(proximal: np.ndarray, distal: np.ndarray, pole_path: np.ndarray, q_rad: np.ndarray) -> float:
    """The first flexion at which the pads overlap (NaN: none on the path)."""

    for q, pole in zip(q_rad[1:], pole_path[1:]):
        if overlap(proximal, turned(distal, pole, q)) > 1e-9:
            return math.degrees(q)
    return math.nan


def least_gap(pad: dict, pole_path: np.ndarray, q_rad: np.ndarray, required_rad: float) -> float:
    """The least gap between the pads with no contact up to the required
    flexion, the pads' depths and the pole's path kept: both pads moved off
    the joint by the same length (bisection)."""

    use_q = q_rad <= required_rad + 1e-9
    centre = 0.5 * (pad["proximal_end"] + pad["distal_start"])

    def clear(gap: float) -> bool:
        proximal = rectangle(centre - gap / 2 - 200.0, centre - gap / 2, pad["proximal_top"], -pad["t1"])
        distal = rectangle(centre + gap / 2, centre + gap / 2 + 200.0, pad["distal_top"], -pad["t2"])
        return not math.isfinite(first_contact_deg(proximal, distal, pole_path[use_q], q_rad[use_q]))

    low, high = -60.0, 120.0
    if clear(low):
        return low
    for _ in range(40):
        middle = 0.5 * (low + high)
        low, high = (low, middle) if clear(middle) else (middle, high)
    return high


def corner_gap(pad: dict, pole_path: np.ndarray, q_rad: np.ndarray, required_rad: float) -> tuple[float, float]:
    """The closed form: ``G = max_q (tau1(q) + tau2(q)) tan(q/2)`` with
    ``tau_i = t_i - z(q)`` the face's depth below the pole (0 where the pole is
    below the face), and the flexion that sets it."""

    best, at = 0.0, 0.0
    for q, pole in zip(q_rad[1:], pole_path[1:]):
        if q > required_rad + 1e-9:
            break
        depth = -pole[1]
        value = (max(pad["t1"] - depth, 0.0) + max(pad["t2"] - depth, 0.0)) * math.tan(q / 2)
        if value > best:
            best, at = value, math.degrees(q)
    return best, at


def coverage_law(pad: dict, start: float, length: float, pole_path: np.ndarray, q_rad: np.ndarray,
                 required_rad: float) -> float:
    """The cylinder-based pad coverage (A.5.1) from the pads and the pole's path
    alone: at each evaluation pose the two face lines meet at ``X``; the pads
    share the distances from ``X`` both cover; the posed length is the two
    phalanges' and the straight gap between their ends (on the link lines)."""

    values = []
    for pose in (math.pi / 2, (math.pi / 2 + required_rad) / 2, required_rad):
        index = int(np.argmin(np.abs(q_rad - pose)))
        pole, q = pole_path[index], q_rad[index]
        outer1, inner1 = np.array((start, -pad["t1"])), np.array((pad["proximal_end"], -pad["t1"]))
        inner2, outer2 = turned(np.array([(pad["distal_start"], -pad["t2"]), (length, -pad["t2"])]), pole, q)
        d1, d2 = outer1 - inner1, outer2 - inner2
        along = np.linalg.solve(np.column_stack((d1, -d2)), inner2 - inner1)
        x = inner1 + along[0] * d1  # where the face lines meet
        spans = []
        for inner, outer in ((inner1, outer1), (inner2, outer2)):
            unit = (outer - inner) / np.linalg.norm(outer - inner)
            spans.append((max(float((inner - x) @ unit), 0.0), float((outer - x) @ unit)))
        common = max(min(spans[0][1], spans[1][1]) - max(spans[0][0], spans[1][0]), 0.0)
        end1 = np.array((pad["proximal_end"], 0.0))
        end2 = turned(np.array([(pad["distal_start"], 0.0)]), pole, q)[0]
        posed = pad["proximal_end"] + float(np.linalg.norm(end2 - end1)) + (length - pad["distal_start"])
        values.append(2.0 * common / posed)
    return float(np.mean(values))


# Every Study 1 design ---------------------------------------------------------------

def designs(config: dict) -> list[dict]:
    found = []
    for name, spec in config["architectures"].items():
        for flexion in (float(q) for q in config["required_flexion_deg"]):
            for thickness, reference in sorted(study2.study1_designs(name, spec, flexion, config).items()):
                found.append({"architecture": name, "required_flexion_deg": flexion, "thickness_mm": thickness,
                              "reference": reference})
    return found


def analyse(entry: dict) -> tuple[dict, list[dict]]:
    reference, flexion, h = entry["reference"], entry["required_flexion_deg"], entry["thickness_mm"]
    base = mechanism(reference.mechanism)
    report = base.report(base.evaluate(reference.x, reference.config))
    length = float(report["finger_length_mm"])
    from core.optimization.analysis.tradeoff import goal_of

    packaging = goal_of(reference.config).packaging
    start = packaging.proximal_pad_start_mm - packaging.proximal_pad.offset_mm
    pad = {"proximal_end": start + float(report["proximal_support_length_mm"]),
           "distal_start": length - float(report["distal_support_length_mm"]),
           "t1": float(report["proximal_depth_mm"]), "t2": float(report["distal_depth_mm"])}
    pad |= {"proximal_top": -pad["t1"] + h, "distal_top": -pad["t2"] + h}
    q = np.radians(np.arange(0.0, 180.0 + STEP_DEG / 2, STEP_DEG))
    try:
        tips = tip_path(reference.mechanism, reference.x, reference.config, q)
    except (ValueError, ZeroDivisionError):
        tips = np.full((len(q), 2), np.nan)
    q = q[:len(tips)]  # a mechanism gives the poses it reaches
    valid = np.all(np.isfinite(tips), axis=1)
    last = int(np.argmin(valid)) if not valid.all() else len(q)
    q, tips = q[:last], tips[:last]
    path = poles(tips, q)
    proximal = rectangle(start, pad["proximal_end"], pad["proximal_top"], -pad["t1"])
    distal = rectangle(pad["distal_start"], length, pad["distal_top"], -pad["t2"])
    required = math.radians(flexion)
    contact = first_contact_deg(proximal, distal, path, q)
    closed, at = corner_gap(pad, path, q, required)
    index = min(int(round(flexion / STEP_DEG)), len(q) - 1)
    row = {"architecture": entry["architecture"], "mechanism": reference.mechanism, "required_flexion_deg": flexion,
           "thickness_mm": h, "c_pad_mean": float(report["c_pad_mean"]),
           "pad_coverage": (float(report["proximal_support_length_mm"])
                            + float(report["distal_support_length_mm"])) / length,
           "c_pad_law": coverage_law(pad, start, length, path, q, required),
           "gap_mm": pad["distal_start"] - pad["proximal_end"], "gap_law_exact_mm": least_gap(pad, path, q, required),
           "gap_law_corner_mm": closed, "critical_flexion_deg": at,
           "face_depth_proximal_mm": pad["t1"], "face_depth_distal_mm": pad["t2"],
           "mount_depth_proximal_mm": pad["t1"] - h, "mount_depth_distal_mm": pad["t2"] - h,
           "pole_x_at_req_mm": float(path[index, 0]), "pole_depth_at_req_mm": float(-path[index, 1]),
           "first_contact_deg": contact, "q_max_deg": float(report["q_max_deg"]),
           "stop_reason": report.get("stop_reason", ""), "collision_pair": report.get("collision_pair", ""),
           "pad_clearance_mm": report.get("pad_clearance_mm", "")}
    use_q = q <= required + 1e-9
    # The contact face seen from the other contact face: the distal face's point under the fingertip,
    # from the proximal face's line (the faces turn by q whatever the design, so one point is the pose).
    contact = np.array([turned(np.array([(length, -pad["t2"])]), pole, angle)[0] if angle > 0
                        else np.array((length, -pad["t2"])) for angle, pole in zip(q, path)])
    contact = contact - np.array((0.0, -pad["t1"]))
    row["_paths"] = {"q": q[use_q], "tip": tips[use_q], "contact": contact[use_q], "length": length,
                     "pole": (path - np.array((0.0, -pad["t1"])))[use_q], "design": entry["reference"]}
    samples = [{"architecture": entry["architecture"], "required_flexion_deg": flexion, "thickness_mm": h,
                "q_deg": round(math.degrees(angle), 3), "pole_x_mm": float(p[0]), "pole_depth_mm": float(-p[1])}
               for angle, p in zip(q, path) if angle > 0 and abs(math.degrees(angle) % 5.0) < STEP_DEG / 2
               or abs(math.degrees(angle) % 5.0 - 5.0) < STEP_DEG / 2]
    return row, samples


def changes(rows: list[dict]) -> list[dict]:
    """Between Study 1's designs for adjacent thicknesses (one architecture, one
    required flexion): how far the motion moved, of the skeleton (the
    fingertip's path) and of the contact (the distal contact face's path seen
    from the proximal one), each the largest distance over ``[0, q_req]`` per
    finger length; and what a joint turning about a fixed axis must show (its
    faces each ``dh`` farther from the axis: ``2 sin(q_req/2) dh``, the
    skeleton unmoved)."""

    found = []
    keys = sorted({(r["architecture"], r["required_flexion_deg"]) for r in rows})
    for architecture, flexion in keys:
        own = sorted((r for r in rows if (r["architecture"], r["required_flexion_deg"]) == (architecture, flexion)),
                     key=lambda r: r["thickness_mm"])
        for thin, thick in zip(own, own[1:]):
            a, b = thin["_paths"], thick["_paths"]
            n = min(len(a["q"]), len(b["q"]))
            scale = 0.5 * (a["length"] + b["length"])
            skeleton = float(np.max(np.linalg.norm(a["tip"][:n] - b["tip"][:n], axis=1))) / scale
            contact = float(np.max(np.linalg.norm(a["contact"][:n] - b["contact"][:n], axis=1))) / scale
            # Proposition 3: the same, from the pole's shift seen from the contact face.
            identity = float(np.nanmax(2 * np.sin(a["q"][1:n] / 2)
                                       * np.linalg.norm(a["pole"][1:n] - b["pole"][1:n], axis=1))) / scale
            dh = thick["thickness_mm"] - thin["thickness_mm"]
            found.append({"architecture": architecture, "mechanism": thin["mechanism"],
                          "required_flexion_deg": flexion, "from_mm": thin["thickness_mm"],
                          "to_mm": thick["thickness_mm"], "skeleton_change": skeleton, "contact_change": contact,
                          "contact_change_identity": identity,
                          "mount_depth_change_mm": thick["mount_depth_distal_mm"] - thin["mount_depth_distal_mm"],
                          "fixed_axis_contact_change": 2 * math.sin(math.radians(flexion) / 2) * dh / scale,
                          "gap_change_mm": thick["gap_mm"] - thin["gap_mm"],
                          "fixed_axis_gap_change_mm": 2 * math.tan(math.radians(flexion) / 2) * dh,
                          "pole_depth_change_mm": thick["pole_depth_at_req_mm"] - thin["pole_depth_at_req_mm"],
                          "c_pad_change": thick["c_pad_mean"] - thin["c_pad_mean"]})
    return found


def fixed_fingers(rows: list[dict], thicknesses) -> list[dict]:
    """Each design as it is, with pads of every thickness (``one_finger.mount``):
    the flexion at which the law says the pads first meet, against the reach
    the study's evaluator finds (on its own 1 deg grid)."""

    from core.optimization.analysis.one_finger import fitted, mount
    from core.optimization.analysis.tradeoff import goal_of

    found = []
    for row in rows:
        reference, paths = row["_paths"]["design"], row["_paths"]
        base = mechanism(reference.mechanism)
        part = study2.shared.partitions.partition(reference.mechanism)
        q = np.radians(np.arange(0.0, 180.0 + STEP_DEG / 2, STEP_DEG))
        try:
            tips = tip_path(reference.mechanism, reference.x, reference.config, q)
        except (ValueError, ZeroDivisionError):
            continue
        q = q[:len(tips)]
        path = poles(tips, q)
        packaging = goal_of(reference.config).packaging
        start = packaging.proximal_pad_start_mm - packaging.proximal_pad.offset_mm
        for h in (float(v) for v in thicknesses):
            _, evaluation = mount(base, part, reference.x, reference.config, h)
            if evaluation is None or fitted(part, reference.x, reference.config, h) is None:
                continue
            report = base.report(evaluation)
            length = float(report["finger_length_mm"])
            t1, t2 = float(report["proximal_depth_mm"]), float(report["distal_depth_mm"])
            proximal = rectangle(start, start + float(report["proximal_support_length_mm"]), -t1 + h, -t1)
            distal = rectangle(length - float(report["distal_support_length_mm"]), length, -t2 + h, -t2)
            found.append({"architecture": row["architecture"], "required_flexion_deg": row["required_flexion_deg"],
                          "designed_for_mm": row["thickness_mm"], "thickness_mm": h,
                          "first_contact_law_deg": first_contact_deg(proximal, distal, path, q),
                          "q_max_deg": float(report["q_max_deg"]), "stop_reason": report.get("stop_reason", ""),
                          "collision_pair": report.get("collision_pair", "")})
    return found


def figure(rows: list[dict], samples: list[dict], tag: str) -> Path:
    fig, axes = plt.subplots(1, 3, figsize=(10.2, 3.1))
    key = {v: k for k, v in NAMES.items()}
    # (a) the pole's depth over the flexion: the designs for 5 mm pads, q_req 120.
    axis = axes[0]
    for name, label in NAMES.items():
        own = [s for s in samples if s["architecture"] == name and s["required_flexion_deg"] == 120.0
               and s["thickness_mm"] == 5.0 and s["q_deg"] <= 120.0]
        if not own:
            continue
        mech = next(r["mechanism"] for r in rows if r["architecture"] == name)
        ideal = label.endswith("*")
        axis.plot([s["q_deg"] for s in own], [s["pole_depth_mm"] for s in own], color=COLORS[mech],
                  lw=1.0 if ideal else 1.8, ls=(0, (3, 2)) if ideal else "-", label=label)
    axis.axhline(0.0, color=INK, lw=0.5)
    axis.set_xlabel("flexion q (deg)", fontsize=7.5)
    axis.set_ylabel("pole depth z(q) below the link line (mm)", fontsize=7.5)
    axis.set_title("(a) where the phalanx turns about\n($q_{req}$ 120°, 5 mm pads)", fontsize=8)
    axis.legend(fontsize=6.5, frameon=False)
    # (b) the law against the optimizer: the gap it asks and the gap the design has.
    axis = axes[1]
    for row in rows:
        mech = row["mechanism"]
        binding = row["stop_reason"] == "collision" or float(row["pad_clearance_mm"] or 1.0) < 0.1
        axis.plot(row["gap_law_exact_mm"], row["gap_mm"], MARKERS.get(mech, "o"), color=COLORS[mech], ms=4,
                  mfc=COLORS[mech] if binding else "white")
    limit = max([r["gap_mm"] for r in rows] + [1.0]) * 1.08
    axis.plot([0, limit], [0, limit], color=INK, lw=0.6)
    axis.set_xlabel("joint gap the law asks (mm)", fontsize=7.5)
    axis.set_ylabel("joint gap of the optimized design (mm)", fontsize=7.5)
    axis.set_title("(b) the law against every Study 1 design\n(hollow: the pads are not what stops it)", fontsize=8)
    # (c) the gap against the thickness, per architecture, q_req 120.
    axis = axes[2]
    for name, label in NAMES.items():
        own = sorted((r for r in rows if r["architecture"] == name and r["required_flexion_deg"] == 120.0),
                     key=lambda r: r["thickness_mm"])
        if not own:
            continue
        mech, ideal = own[0]["mechanism"], label.endswith("*")
        axis.plot([r["thickness_mm"] for r in own], [r["gap_mm"] for r in own], color=COLORS[mech],
                  marker=MARKERS.get(mech, "o"), ms=4, lw=1.0 if ideal else 1.8, ls=(0, (3, 2)) if ideal else "-")
    axis.set_xlabel("pad thickness h (mm)", fontsize=7.5)
    axis.set_ylabel("joint gap G (mm)", fontsize=7.5)
    axis.set_title("(c) what a thicker pad takes\n($q_{req}$ 120°)", fontsize=8)
    for axis in axes:
        axis.grid(color=GRID, lw=0.6)
        for side in ("top", "right"):
            axis.spines[side].set_visible(False)
    fig.tight_layout()
    FIGURES.mkdir(parents=True, exist_ok=True)
    path = FIGURES / f"study2_pole_law_{tag}.png"
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)
    return path


def write(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tag", default="native")
    args = parser.parse_args(argv)
    config = read_config_file(study2.DEFAULT_CONFIG)
    rows, samples = [], []
    for entry in designs(config):
        row, own = analyse(entry)
        rows.append(row)
        samples += own
    OUTPUT.mkdir(parents=True, exist_ok=True)
    write(OUTPUT / "changes.csv", changes(rows))
    write(OUTPUT / "fixed_finger.csv", fixed_fingers(rows, config["thickness_mm"]))
    for row in rows:
        row.pop("_paths")
    write(OUTPUT / "designs.csv", rows)
    write(OUTPUT / "poles.csv", samples)
    use("paper")
    print(figure(rows, samples, args.tag), f"({len(rows)} designs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
