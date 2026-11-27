#!/usr/bin/env python3
"""Experimental: what the proximal face shift dt changes (docs/experimental/pr_mr.md), drawn from the design values.

    python scripts/study/exploratory/e23_face_shift_figure.py

Hands at 120 deg, 3 -> 7 mm: Original (3 mm), Pad-Pad (Study 2's PR, dt 4) and MO-Pad (e20's MR-mid, dt 1).
- Top row: each finger at q = 0 in F_1 (the MCP frame: MCP axis at the origin, link line on y = 0), with its proximal
  pad (mount and thickness), its distal pad, and a cube resting on the proximal face. Original's face is dashed in every
  panel; the cube moves away from the palm by dt.
- Bottom row: the distal pad's contact point over q in [0, 120 deg], seen from the proximal face (Delta_contact) and
  from the MCP frame (Delta_MCP). The two views differ by the face depth, so a hand that matches Original in one view
  is off by dt in the other unless dt is small.

Writes ``report/experimental/figures/face_shift_dt.png``.
"""
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
for p in (ROOT, ROOT / "scripts" / "study", HERE):
    sys.path.insert(0, str(p))
import e20_mcp_referenced as E  # noqa: E402
from core.optimization.analysis.contact_kinematics import paths  # noqa: E402
from core.optimization.mechanism import mechanism, read_config_file  # noqa: E402

OUT = ROOT.parent / "report" / "experimental" / "figures" / "face_shift_dt.png"
COLORS = {"Original": "#6b6b6b", "Pad-Pad": "#2a78d6", "MO-Pad": "#eb6834"}
INK, MUTED = "#222222", "#8a8a8a"


def hands():
    config = read_config_file(ROOT / "scripts" / "study" / "study2_morphology_adaptation" / "one_finger.yaml")
    designs = E.e15.study1.study1_designs("cfl", config["architectures"]["cfl"], 120.0, config)
    h0, goal = designs[3.0], designs[7.0]
    base = mechanism(h0.mechanism)
    with (E.e15.STUDY2 / "front.csv").open(encoding="utf-8", newline="") as stream:
        front = [r for r in csv.DictReader(stream) if r["architecture"] == "cfl"
                 and float(r["required_flexion_deg"]) == 120 and float(r["to_mm"]) == 7]
    x_pr = tuple(float(v) for v in max(front, key=lambda r: float(r["c_pad_mean"]))["x"].split(";"))
    picks = ROOT / "output" / "study" / "exploratory" / "e20_mcp_referenced_free" / "picks.csv"
    x_mo = next(tuple(float(v) for v in r["x"].split(";")) for r in csv.DictReader(picks.open()) if r["label"] == "MR-mid")
    free = E.with_b_z(goal.config, -3, 10, a_z=(-3, 8))
    out = {}
    for name, x, cfg, t in (("Original", h0.x, h0.config, 3.0), ("Pad-Pad", x_pr, goal.config, 7.0),
                            ("MO-Pad", x_mo, free, 7.0)):
        e = base.evaluate(x, cfg)
        r = base.report(e)
        p = paths(base, x, cfg, e)
        depth = float(r["proximal_depth_mm"])
        out[name] = {"t": t, "depth": depth, "distal_depth": float(r["distal_depth_mm"]),
                     "prox_len": float(r["proximal_length_mm"]), "prox_pad": float(r["proximal_support_length_mm"]),
                     "dist_pad": float(r["distal_support_length_mm"]), "length": float(r["finger_length_mm"]),
                     "q": np.degrees(p["q"]), "face": np.asarray(p["contact"]),
                     "f1": np.asarray(p["contact"]) - np.array((0.0, depth))}
    return out


def finger(ax, name, h, ref):
    """The proximal phalanx at q = 0 in F_1, zoomed on the proximal pad, with a cube resting on the face and a
    palm-fixed opposing contact (the thumb, say) placed where Original's cube ends."""
    c = COLORS[name]
    mount = h["depth"] - h["t"]
    side, x0 = 24.0, 3.0
    ax.plot([-4, 64], [0, 0], color=INK, lw=1.4, solid_capstyle="round")
    ax.plot(0, 0, "o", color=INK, ms=6, zorder=3)
    ax.text(-1, 1.6, "MCP axis", ha="left", va="bottom", fontsize=8, color=INK)
    ax.text(63, 1.6, "link line", ha="right", va="bottom", fontsize=7.5, color=MUTED)
    ax.add_patch(Rectangle((x0, -h["depth"]), h["prox_pad"], h["t"], fc=c, ec="none", alpha=0.35))
    ax.plot([x0, x0 + h["prox_pad"]], [-h["depth"]] * 2, color=c, lw=2.2)
    ax.text(x0 + h["prox_pad"] + 1.5, -h["depth"] + h["t"] / 2, f"mount {mount:.0f}\npad {h['t']:.0f}",
            va="center", fontsize=7.5, color=MUTED, linespacing=1.2)
    ax.plot([x0 - 3, x0 + ref["prox_pad"] + 3], [-ref["depth"]] * 2, color=MUTED, lw=1, ls=(0, (3, 2)))
    cx = x0 + 6
    ax.add_patch(Rectangle((cx, -h["depth"] - side), side, side, fc="white", ec=c, lw=1.6))
    ax.text(cx + side / 2, -h["depth"] - side / 2, "object", ha="center", va="center", fontsize=8, color=c)
    fixed = -ref["depth"] - side
    ax.plot([cx - 6, cx + side + 6], [fixed, fixed], color=INK, lw=2.4)
    ax.text(cx + side + 7, fixed, "palm-fixed contact\n(e.g. thumb)", va="center", fontsize=7.5, color=INK, linespacing=1.2)
    dt = h["depth"] - ref["depth"]
    if abs(dt) > 1e-6:
        ax.add_patch(Rectangle((cx, fixed - dt), side, dt, fc="#d62728", ec="none", alpha=0.45))
        xa = cx - 3
        ax.annotate("", xy=(xa, -h["depth"]), xytext=(xa, -ref["depth"]),
                    arrowprops=dict(arrowstyle="<->", color=INK, lw=1, shrinkA=0, shrinkB=0))
        ax.text(xa - 1, -(h["depth"] + ref["depth"]) / 2 - 3.5, f"Δt {dt:.0f}", ha="right", va="center", fontsize=8.5,
                color=INK)
        ax.text(cx + side / 2, fixed - dt - 1.2, f"overlap {dt:.0f} mm", ha="center", va="top", fontsize=7.5,
                color="#b01c1c")
    ax.set_title(f"{name}: face {h['depth']:.0f} mm below the link line", fontsize=10,
                 color=c if name != "Original" else INK, loc="left")
    ax.set_xlim(-14, 74)
    ax.set_ylim(-43, 5)
    ax.set_aspect("equal")
    ax.axis("off")


def path_panel(ax, data, key, title, label):
    ref = data["Original"][key]
    for name, h in data.items():
        p = h[key]
        dev = np.max(np.linalg.norm(p - ref, axis=1))
        ax.plot(p[:, 0], p[:, 1], color=COLORS[name], lw=2 if name != "Original" else 1.4,
                ls="-" if name != "Original" else (0, (3, 2)),
                label=name if name == "Original" else f"{name}: {label} {dev:.2f} mm")
        ax.plot(p[0, 0], p[0, 1], "o", color=COLORS[name], ms=4)
    ax.set_title(title, fontsize=10, loc="left", color=INK)
    ax.set_aspect("equal")
    ax.set_ylim(-48, 4)
    ax.grid(color="#e6e6e6", lw=0.6)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.set_xlabel("x [mm]", fontsize=8.5)
    ax.set_ylabel("y [mm]", fontsize=8.5)
    ax.tick_params(labelsize=8)
    ax.legend(fontsize=8, frameon=False, loc="upper left")


def main() -> int:
    data = hands()
    fig = plt.figure(figsize=(13, 8.6), dpi=200)
    gs = fig.add_gridspec(2, 3, height_ratios=(0.62, 1.15), hspace=0.08, wspace=0.18)
    for i, name in enumerate(data):
        finger(fig.add_subplot(gs[0, i]), name, data[name], data["Original"])
    path_panel(fig.add_subplot(gs[1, 0]), data, "face", "Distal contact path, from the proximal face", "Δ_contact")
    ax = fig.add_subplot(gs[1, 1])
    path_panel(ax, data, "f1", "The same path, from the palm (F₁)", "Δ_MCP")
    note = fig.add_subplot(gs[1, 2])
    note.axis("off")
    note.text(0, 0.95, "What Δt changes", fontsize=10.5, weight="bold", color=INK, va="top")
    lines = [
        "Δt = how far the proximal pad face moves",
        "away from the MCP axis (palm side).",
        "",
        "• An object resting on the proximal pad",
        "  sits Δt farther from the palm. Against a",
        "  palm-fixed contact (the thumb, another",
        "  finger, the palm) the grasp is off by Δt:",
        "  the policy's thumb motion no longer meets",
        "  the object where it did (red).",
        "• The two views of the distal contact path",
        "  differ by exactly the face depth, so",
        "  Δ_contact + Δ_MCP ≥ |Δt|.",
        "• Pad-Pad keeps the path from the face",
        "  (0.86 mm) but is ~4 mm off from the palm.",
        "• MO-Pad mounts the pad deeper (mount 0),",
        "  so Δt = 1 and both views stay ≈ 1 mm.",
    ]
    note.text(0, 0.86, "\n".join(lines), fontsize=8.8, color=INK, va="top", linespacing=1.45)
    fig.suptitle("Proximal face shift Δt (CFL, q_req 120°, 3 → 7 mm pads)", fontsize=12, x=0.125, ha="left", y=0.97)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, bbox_inches="tight", facecolor="white")
    for name, h in data.items():
        print(name, f"depth {h['depth']:.2f} t {h['t']} dc {np.max(np.linalg.norm(h['face'] - data['Original']['face'], axis=1)):.2f}"
              f" dmcp {np.max(np.linalg.norm(h['f1'] - data['Original']['f1'], axis=1)):.2f}")
    print(OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
