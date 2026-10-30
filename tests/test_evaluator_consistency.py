"""The optimization evaluators and the fixed-layout benchmark
(core.optimization.analysis.contact_envelope) agree on the geometry they share.

Two independent paths: the mechanism's full evaluator (``Mechanism.evaluate``,
the study's constraints, on the verification grid) and ``evaluate_fit``; and,
for the signs of the margins, an independent overlap test here (matplotlib's
point-in-polygon and exact segment crossings on the scene's solids), not the
packaging's distance kernels.

Intentional differences, not tested for equality:

- one way only: a design the full evaluator accepts the benchmark accepts;
  not the reverse (the benchmark leaves out the transmission range, torque,
  the dorsal space, the fingertip behind O, the link shares, the least pad);
- where a motion fails: the evaluator samples the 0.05 deg verification grid
  (the 2R) or the servo's (CoDe), the benchmark 0.25 deg of flexion (the 2R)
  or 0.1 deg of servo (CoDe) and reports the first failing sample, so the two
  angles agree to within a sample;
- which pair names a failure: the benchmark the deepest overlap at the first
  failing pose, the evaluator the first pair failing there; where several
  solids overlap at once (two bars from one pin), the names may differ;
- static failures: the evaluator returns one constraint (``pad_collision_mm``,
  ``pivot_pin_mm``, ``mcp_servo_mm``, ``distal_support_mm``) and stops, the
  benchmark names the pair (``invalid_fixed_layout`` for the structure);
  CoDe's pin A against the servo is ``pivot_pin_mm`` (|AA'| less both pins'
  envelopes) in one, the pin against the servo's outline in the other: both
  catch a servo on A, by different measures.
"""

import math
from pathlib import Path

import numpy as np
import pytest
from matplotlib.path import Path as Outline

from core.optimization.analysis.contact_envelope import evaluate_fit, layout_violations
from core.optimization.analysis.loss import scene
from core.optimization.analysis.tradeoff import Modules, load, with_modules
from core.optimization.mechanism import mechanism, read_config_file

STUDY = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "accommodation"
LINKAGE = (40.2, 72.8, 0.0, 44.9, 11.6, 63.8, 10.0, 25.0, 27.2, 0.0, -4.7, -9.0, 0.0, 0.0, 3.0, 3.0)
REVOLUTE = (47.5, 52.5, 36.0, 15.0, 0.0, 0.0, 90.0)
REFERENCE = Modules(25.0, 5.0, 25.0, 5.0)


def optimization(name: str, modules: Modules, flexion: float):
    """The study's config with these modules and flexion, on its verification grid."""

    values = read_config_file(STUDY / f"{name}_P25x5_D25x5_flex150.yaml", (
        f"goal.motion.dip_flexion_deg={flexion}",
        f"goal.pads.proximal.length_mm=[{modules.proximal_length_mm}, {modules.proximal_length_mm}]",
        f"goal.pads.proximal.height_mm={modules.proximal_height_mm}",
        f"goal.pads.distal.length_mm=[{modules.distal_length_mm}, {modules.distal_length_mm}]",
        f"goal.pads.distal.height_mm={modules.distal_height_mm}"))
    found = mechanism(values.pop("mechanism"))
    values.pop("output")
    config = found.config_from_mapping(found.split_settings(values)[0])
    return found, found.refine(config, math.radians(0.05))


def benchmark(name: str):
    return load(STUDY / f"{name}_P25x5_D25x5_flex150.yaml")[1]


# Independent geometry: exact overlap of convex polygons, segments and disks.
def _crosses(p0, p1, q0, q1) -> bool:
    def side(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    d1, d2, d3, d4 = side(p0, p1, q0), side(p0, p1, q1), side(q0, q1, p0), side(q0, q1, p1)
    return d1 * d2 < 0.0 and d3 * d4 < 0.0  # a proper crossing (touching is not overlap)


def _edges(polygon):
    polygon = np.asarray(polygon, dtype=float)
    return list(zip(polygon, np.roll(polygon, -1, axis=0)))


def overlap(first, second) -> bool:
    """Whether two polygons' interiors meet (touching is not overlap)."""

    shrink = -1e-7  # matplotlib: a negative radius keeps the boundary out
    if Outline(second).contains_points(first, radius=shrink).any() or \
            Outline(first).contains_points(second, radius=shrink).any():
        return True
    return any(_crosses(*e, *f) for e in _edges(first) for f in _edges(second))


def disk_overlaps(polygon, centre, radius) -> bool:
    centre = np.asarray(centre, dtype=float)
    if Outline(polygon).contains_point(centre):
        return True
    points = np.concatenate([np.linspace(a, b, 2001) for a, b in _edges(polygon)])
    return float(np.linalg.norm(points - centre, axis=1).min()) < radius - 1e-7


def solids_at(name: str, modules: Modules, x, phi: float):
    built = scene(name, with_modules(benchmark(name), modules), x, [phi])
    first = lambda polygon: np.asarray(polygon)[0] if np.ndim(polygon) == 3 else np.asarray(polygon)  # noqa: E731
    return {"proximal_pad": first(built.solids["distal_pad"]["other_pad"].polygons[0]),
            "distal_pad": first(built.solids["proximal_pad"]["other_pad"].polygons[0]),
            "mcp_servo": first(built.solids["distal_pad"]["mcp_servo"].polygons[0]),
            "carriers": [first(p) for p in built.solids["distal_pad"]["carrier"].polygons],
            "servo": [first(p) for p in built.solids["distal_pad"]["servo"].polygons],
            "pins": [(np.asarray(c)[0] if np.ndim(c) == 2 else np.asarray(c), r)
                     for c, r in built.solids["distal_pad"]["pin"].disks]}


@pytest.mark.parametrize("name, x, flexion", [("linkage", LINKAGE, 120.0), ("linkage", LINKAGE, 150.0),
                                              ("revolute", REVOLUTE, 120.0)])
def test_a_design_the_optimization_accepts_the_benchmark_accepts(name, x, flexion):
    found, config = optimization(name, REFERENCE, flexion)
    assert found.evaluate(x, config).feasible
    assert evaluate_fit(name, x, benchmark(name), REFERENCE, flexion).feasible


MOTION_CASES = [
    # name, design, modules, flexion, the pair both name, the benchmark's sample step (deg of flexion)
    ("revolute", REVOLUTE, Modules(15.0, 10.0, 25.0, 5.0), 150.0, "distal_pad/proximal_pad", 0.25),
    ("revolute", REVOLUTE, Modules(15.0, 3.0, 25.0, 5.0), 170.0, "distal_pad/mcp_servo", 0.25),
    ("linkage", LINKAGE, Modules(25.0, 10.0, 25.0, 5.0), 150.0, "distal_pad/proximal_pad", 0.4),
]


@pytest.mark.parametrize("name, x, modules, flexion, pair, step", MOTION_CASES)
def test_a_collision_in_the_motion_fails_both_at_the_same_pose(name, x, modules, flexion, pair, step):
    found, config = optimization(name, modules, flexion)
    evaluation = found.evaluate(x, config)
    fit = evaluate_fit(name, x, benchmark(name), modules, flexion)
    assert not evaluation.feasible and not fit.feasible
    assert evaluation.collision_pair == fit.collision_pair == pair
    safe = math.degrees(evaluation.phi_safe_rad)
    assert safe - 1e-6 <= fit.first_failure_deg <= safe + step + 0.05
    # Independently: clear where the evaluator is still safe, overlapping where the benchmark fails.
    first, second = pair.split("/")
    for phi, colliding in ((safe, False), (fit.first_failure_deg, True)):
        solids = solids_at(name, modules, x, phi)
        assert overlap(solids[first], solids[second]) == colliding, (phi, colliding)


def test_a_support_through_the_servo_fails_both():
    x = (47.5, 52.5, 52.5, 15.0, 0.0, 0.0, 90.0)  # the 2R's support on the link line, 5 mm past A
    found, config = optimization("revolute", REFERENCE, 120.0)
    evaluation = found.evaluate(x, config)
    fit = evaluate_fit("revolute", x, benchmark("revolute"), REFERENCE, 120.0)
    assert evaluation.constraints["pad_collision_mm"] > 0.0  # at rest, the support's pair among them
    assert not fit.feasible and fit.collision_pair == "proximal_carrier/pip_servo" and fit.first_failure_deg == 0.0
    solids = solids_at("revolute", REFERENCE, x, 0.0)
    support = max(solids["carriers"], key=lambda p: -np.asarray(p)[:, 0].min())  # the proximal plate
    assert overlap(support, solids["servo"][0])
    # The layout's own support stops short of the housing: clear in all three.
    clear = solids_at("revolute", REFERENCE, REVOLUTE, 0.0)
    support = max(clear["carriers"], key=lambda p: -np.asarray(p)[:, 0].min())
    assert not overlap(support, clear["servo"][0])


def test_an_unbuildable_structure_fails_both():
    # The 2R's servo 15 mm from O, into the MCP servo.
    x = (15.0, 85.0, 30.0, 15.0, 0.0, 0.0, 90.0)
    found, config = optimization("revolute", REFERENCE, 120.0)
    assert found.evaluate(x, config).constraints["mcp_servo_mm"] > 0.0
    assert evaluate_fit("revolute", x, benchmark("revolute"), REFERENCE, 120.0).reason == "invalid_fixed_layout"
    solids = solids_at("revolute", REFERENCE, x, 0.0)
    assert overlap(solids["servo"][0], solids["mcp_servo"])
    # CoDe's servo on A' 6 mm above A, on A's pin.
    x = LINKAGE[:3] + (42.0, 6.0) + LINKAGE[5:]
    found, config = optimization("linkage", REFERENCE, 120.0)
    assert found.evaluate(x, config).constraints["pivot_pin_mm"] > 0.0
    fit = evaluate_fit("linkage", x, benchmark("linkage"), REFERENCE, 120.0)
    assert fit.reason == "invalid_fixed_layout" and fit.collision_pair == "a_pin/pip_servo"
    solids = solids_at("linkage", REFERENCE, x, 0.0)
    (a, radius) = solids["pins"][0]  # A's housing
    assert disk_overlaps(solids["servo"][0], a, radius)
    assert layout_violations("linkage", x, with_modules(benchmark("linkage"), REFERENCE))["a_pin/pip_servo"] < 0.0


def test_a_pad_on_its_own_pin_fails_both():
    # CoDe's distal pad on the B-C line (no drop) from B: its corner in B's housing.
    x = LINKAGE[:-1] + (0.0,)
    found, config = optimization("linkage", REFERENCE, 120.0)
    assert found.evaluate(x, config).constraints["distal_support_mm"] > 0.0
    fit = evaluate_fit("linkage", x, benchmark("linkage"), REFERENCE, 120.0)
    assert not fit.feasible and fit.first_failure_deg == pytest.approx(0.0, abs=1e-9)
    solids = solids_at("linkage", REFERENCE, x, 0.0)
    b, radius = solids["pins"][2]  # B's housing (A, A', B, B')
    assert disk_overlaps(solids["distal_pad"], b, radius)
    # With its drop, clear.
    clear = solids_at("linkage", REFERENCE, LINKAGE, 0.0)
    assert not disk_overlaps(clear["distal_pad"], *clear["pins"][2])
