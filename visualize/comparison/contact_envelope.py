"""Contact accommodation of two fixed finger layouts, the 2R (servo on the
joint) and CoDe: which proximal and distal modules each carries through the
required flexion (``core.optimization.analysis.contact_envelope``).

    python -m visualize.comparison.contact_envelope

The layouts, the module grid, the reference modules and the flexion are set
below (edit them here; nothing is optimized). Writes to
output/comparison/contact_envelope/: layouts.png (each layout with the
reference modules, extended and at the required flexion or its first
failure, one scale), accommodation.png (2 x 2: per mechanism, experiment A,
the proximal module over length x thickness with the distal at the
reference, and B, the reverse; filled where the pair fits, letters naming
the failure), accommodation.csv (one record per mechanism, experiment and
module pair, as computed) and metadata.json (the layouts, the grid and the
assumptions, the source commit).
"""

from __future__ import annotations

import csv
import json
import math
import subprocess
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

from core.morphology.linkage import FingerLinkage  # noqa: E402
from core.optimization.analysis import contact_envelope as envelope  # noqa: E402
from core.optimization.analysis.loss import scene  # noqa: E402
from core.optimization.analysis.tradeoff import Modules, load, with_modules  # noqa: E402
from core.optimization.linkage.design import decode  # noqa: E402
from core.optimization.revolute.evaluate import geometry  # noqa: E402
from visualize import sketch  # noqa: E402
from visualize.finger import Finger  # noqa: E402
from visualize.style import COLLIDE, COLORS, INK, MUTED, OUTPUT, REPOSITORY_ROOT, use  # noqa: E402

# --- The fixed layouts (edit here). Complete design vectors, never changed by the sweep.
CONFIGS = REPOSITORY_ROOT / "scripts/optimize/accommodation"
LAYOUTS = {
    # A at 47.5 of |OC| 100, the servo on A upright (90 deg), the pads on the link lines;
    # supports: A'' at 36 (26.5 mm of room), B'' 15 past A (37.5 mm).
    "revolute": ((47.5, 52.5, 36.0, 15.0, 0.0, 0.0, 90.0), CONFIGS / "revolute_P25x5_D25x5_flex120.yaml"),
    # The production CoDe (paper, balanced): A'' at A, B'' at B, the pads 3 mm below the links.
    "linkage": ((40.2, 72.8, 0.0, 44.9, 11.6, 63.8, 10.0, 25.0, 27.2, 0.0, -4.7, -9.0, 0.0, 0.0, 3.0, 3.0),
                CONFIGS / "linkage_P25x5_D25x5_flex120.yaml"),
}
REFERENCE = Modules(proximal_length_mm=25.0, proximal_height_mm=5.0, distal_length_mm=25.0, distal_height_mm=5.0)
LENGTHS_MM = np.arange(15.0, 35.0 + 1e-9, 2.0)
HEIGHTS_MM = np.arange(3.0, 10.0 + 1e-9, 1.0)
REQUIRED_FLEXION_DEG = 120.0
# ---

OUT = OUTPUT / "comparison" / "contact_envelope"
NAMES = {"revolute": "2R (servo on the joint)", "linkage": "CoDe"}
CODES = {"support_exceeded": "S", "kinematic_unreachable": "K", "pad_pad_collision": "P", "pad_servo_collision": "V",
         "pad_joint_collision": "J", "pad_linkage_collision": "L", "pad_support_collision": "U",
         "structure_collision": "X", "invalid_fixed_layout": "!"}


def layouts() -> dict:
    return {name: (x, load(path)[1]) for name, (x, path) in LAYOUTS.items()}


def finger(name: str, config, x) -> Finger:
    state = FingerLinkage(decode(x, config.link_length_resolution_mm)) if name == "linkage" else geometry(x, config)
    return Finger(name, {}, config, tuple(x), state)


def describe(name: str, x, config) -> dict:
    """The layout's structure, as the metadata reports it."""

    carried = with_modules(config, REFERENCE)
    goal = getattr(carried, "goal", carried)
    rooms = envelope.support_rooms(name, x, carried)
    v = dict(zip(load(LAYOUTS[name][1])[0].variable_names, x))
    info = {"design_vector": list(x), "variables": v, "support_room_mm": {"proximal": rooms[0], "distal": rooms[1]},
            "proximal_module_start_mm": goal.packaging.proximal_pad_start_mm}
    if name == "linkage":
        p = decode(x, config.link_length_resolution_mm)
        info.update({"finger_length_mm": math.hypot(p.b_x_mm + p.bc_length_mm, p.b_z_mm),
                     "active_joints": {"O": [0.0, 0.0], "A'": [p.ap.x_mm, p.ap.z_mm]},
                     "passive_joints": {"A": [p.a_x_mm, p.a_z_mm], "B": [p.b_x_mm, p.b_z_mm], "B'": [p.bp.x_mm, p.bp.z_mm]},
                     "supports_x_mm": {"A''": p.app_x_mm, "B''": p.bpp_x_mm},
                     "pad_drops_mm": {"proximal": goal.packaging.proximal_drop(p), "distal": goal.packaging.distal_drop(p)}})
    else:
        info.update({"finger_length_mm": v["a_x_mm"] + v["ac_length_mm"],
                     "active_joints": {"O": [0.0, 0.0], "A": [v["a_x_mm"], 0.0]},
                     "supports_x_mm": {"A''": v["app_x_mm"], "B''_from_A": v["bpp_from_a_mm"]},
                     "pad_drops_mm": {"proximal": v["proximal_drop_mm"], "distal": v["distal_drop_mm"]},
                     "servo_tilt_deg": v["servo_tilt_deg"]})
    return info


def draw_supports(axis, name: str, config, x, pose: float) -> None:
    for polygon in scene(name, config, x, [pose]).solids["distal_pad"]["carrier"].polygons:
        polygon = np.asarray(polygon)
        polygon = polygon[0] if polygon.ndim == 3 else polygon
        axis.add_patch(Polygon(polygon, closed=True, facecolor="#cfcac0", edgecolor=MUTED, linewidth=0.6, zorder=2))


def layouts_figure(found: dict) -> Path:
    figure, axes = plt.subplots(len(found), 2, figsize=(10.0, 3.4 * len(found)), constrained_layout=True,
                                squeeze=False)
    for row, (name, (x, config)) in zip(axes, found.items()):
        carried = with_modules(config, REFERENCE)
        fit = envelope.evaluate_fit(name, x, config, REFERENCE, REQUIRED_FLEXION_DEG)
        shown = fit.first_failure_deg if fit.first_failure_deg is not None else REQUIRED_FLEXION_DEG
        drawn = finger(name, carried, x)
        info = describe(name, x, config)
        for axis, pose in zip(row, (0.0, shown)):
            sketch.frame(axis, (-14.0, 124.0), (-70.0, 40.0))
            draw_supports(axis, name, carried, x, pose)
            drawn.draw(axis, pose)
            ok = pose == 0.0 or fit.feasible
            label = (f"φ = {pose:.1f}°" if pose == 0.0 else
                     f"φ = {pose:.1f}° (required)" if fit.feasible else
                     f"first failure at φ = {pose:.1f}° ({fit.reason}: {fit.collision_pair or ''})")
            axis.set_title(label, color=INK if ok else COLLIDE, fontsize=8, loc="left")
        row[0].text(-12.0, 37.0, f"{NAMES[name]}\n|OC| {info['finger_length_mm']:.1f} mm; supports: proximal "
                    f"{info['support_room_mm']['proximal']:.1f}, distal {info['support_room_mm']['distal']:.1f} mm",
                    color=COLORS[name], fontsize=8, fontweight="bold", va="top")
    figure.suptitle(f"Fixed layouts with the reference modules (proximal {REFERENCE.proximal_length_mm:g} x "
                    f"{REFERENCE.proximal_height_mm:g}, distal {REFERENCE.distal_length_mm:g} x "
                    f"{REFERENCE.distal_height_mm:g} mm); grey: the fixed supports", color=MUTED, fontsize=9,
                    x=0.01, ha="left")
    out = OUT / "layouts.png"
    figure.savefig(out, dpi=150)
    return out


def heatmaps(records: list[dict]) -> Path:
    mechanisms = list(LAYOUTS)
    figure, axes = plt.subplots(len(mechanisms), 2, figsize=(10.5, 3.6 * len(mechanisms)), constrained_layout=True,
                                squeeze=False)
    step_l, step_h = np.diff(LENGTHS_MM[:2])[0], np.diff(HEIGHTS_MM[:2])[0]
    for row, name in zip(axes, mechanisms):
        for axis, experiment, side in zip(row, ("A", "B"), ("proximal", "distal")):
            for record in (r for r in records if r["mechanism"] == name and r["experiment"] == experiment):
                length, height = record[f"{side}_length_mm"], record[f"{side}_height_mm"]
                axis.add_patch(Rectangle((length - step_l / 2, height - step_h / 2), step_l, step_h,
                                         facecolor=COLORS[name] if record["feasible"] else "#f1efe9",
                                         edgecolor="white", linewidth=0.8))
                if not record["feasible"]:
                    axis.text(length, height, CODES.get(record["reason"], "?"), ha="center", va="center",
                              fontsize=6.5, color=MUTED)
            reference = (getattr(REFERENCE, f"{side}_length_mm"), getattr(REFERENCE, f"{side}_height_mm"))
            axis.add_patch(Rectangle((reference[0] - step_l / 2, reference[1] - step_h / 2), step_l, step_h,
                                     fill=False, edgecolor=INK, linewidth=1.6))
            axis.set_xlim(LENGTHS_MM[0] - step_l / 2, LENGTHS_MM[-1] + step_l / 2)
            axis.set_ylim(HEIGHTS_MM[0] - step_h / 2, HEIGHTS_MM[-1] + step_h / 2)
            axis.set_xticks(LENGTHS_MM)
            axis.set_yticks(HEIGHTS_MM)
            axis.set_xlabel(f"{side} module length [mm]", color=INK)
            axis.set_ylabel(f"{side} module thickness [mm]", color=INK)
            fixed = "distal" if side == "proximal" else "proximal"
            axis.set_title(f"{NAMES[name]}: experiment {experiment}, {side} module varied\n"
                           f"({fixed} {getattr(REFERENCE, f'{fixed}_length_mm'):g} x "
                           f"{getattr(REFERENCE, f'{fixed}_height_mm'):g} mm)", color=COLORS[name], fontsize=8.5,
                           loc="left")
    legend = "  ".join(f"{code} {reason}" for reason, code in CODES.items())
    figure.suptitle(f"Contact accommodation of fixed layouts through {REQUIRED_FLEXION_DEG:g}°: filled = both modules "
                    f"fit, clear all through; framed = the reference pair\n{legend}", color=MUTED, fontsize=7.5,
                    x=0.01, ha="left")
    out = OUT / "accommodation.png"
    figure.savefig(out, dpi=150)
    return out


def main() -> int:
    use("paper")
    OUT.mkdir(parents=True, exist_ok=True)
    found = layouts()
    records = envelope.sweep(found, REFERENCE, LENGTHS_MM, HEIGHTS_MM, REQUIRED_FLEXION_DEG)
    with (OUT / "accommodation.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPOSITORY_ROOT, capture_output=True, text=True).stdout
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "core", "visualize"], cwd=REPOSITORY_ROOT,
                                capture_output=True, text=True).stdout.strip())
    metadata = {
        "source_commit": commit.strip(), "uncommitted_changes": dirty,
        "layouts": {name: {**describe(name, x, config), "config": str(LAYOUTS[name][1].relative_to(REPOSITORY_ROOT))}
                    for name, (x, config) in found.items()},
        "reference_modules": REFERENCE.__dict__, "lengths_mm": LENGTHS_MM.tolist(), "heights_mm": HEIGHTS_MM.tolist(),
        "required_flexion_deg": REQUIRED_FLEXION_DEG,
        "sampling": {"revolute_flexion_step_deg": envelope.REVOLUTE_STEP_DEG,
                     "linkage_servo_step_deg": envelope.LINKAGE_SERVO_STEP_DEG,
                     "linkage_endpoint": "the first sample at or past the required flexion"},
        "assumptions": {
            "clearance_mm": 0.0, "numerical_tolerance_mm": envelope.TOLERANCE_MM,
            "signed_overlap": "overlap negative by its depth (packaging.overlap_depth)",
            "link_capsule_radius_mm": found["linkage"][1].link_radius_mm,
            "support_plate_height_mm": found["linkage"][1].proximal_carrier_height_mm,
            "mcp_flexion_deg": 0.0, "view": "2D side view; every modelled solid occupies the projected plane",
            "excluded": ["manufacturing tolerances", "bearings, shafts, brackets", "torque and transmission limits",
                         "dorsal-space requirement", "contact-face loss or accessibility"]},
    }
    (OUT / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"saved {layouts_figure(found)}\nsaved {heatmaps(records)}\nsaved {OUT / 'accommodation.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
