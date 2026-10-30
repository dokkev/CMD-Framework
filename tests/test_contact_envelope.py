"""The fixed-layout contact accommodation benchmark
(core.optimization.analysis.contact_envelope)."""

from pathlib import Path

import numpy as np
import pytest

from core.optimization.analysis import contact_envelope as envelope
from core.optimization.analysis.contact_envelope import Motion, evaluate_fit, motion, support_rooms
from core.optimization.analysis.loss import scene
from core.optimization.analysis.tradeoff import Modules, load, with_modules

STUDY = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "accommodation"
LAYOUTS = {
    "revolute": (47.5, 52.5, 36.0, 15.0, 0.0, 0.0, 90.0),
    "linkage": (40.2, 72.8, 0.0, 44.9, 11.6, 63.8, 10.0, 25.0, 27.2, 0.0, -4.7, -9.0, 0.0, 0.0, 3.0, 3.0),
}
REFERENCE = Modules(25.0, 5.0, 25.0, 5.0)


def base(name):
    return load(STUDY / f"{name}_P25x5_D25x5_flex120.yaml")[1]


def solids(name, modules):
    """The fixed structure's solids, extended, with ``modules`` mounted."""

    built = scene(name, with_modules(base(name), modules), LAYOUTS[name], [0.0])
    fixed = {}
    for category in ("carrier", "link", "pin", "servo", "mcp_servo"):
        shapes = built.solids["distal_pad"][category]
        fixed[category] = ([np.asarray(p) for p in shapes.polygons], [np.asarray(s) for s in shapes.segments],
                           [(np.asarray(c), r) for c, r in shapes.disks])
    return fixed, built


@pytest.mark.parametrize("name", list(LAYOUTS))
def test_the_mechanism_and_its_supports_stay_whatever_the_modules(name):
    other = Modules(15.0, 10.0, 21.0, 3.0)
    (first, built), (second, _) = solids(name, REFERENCE), solids(name, other)
    for category in first:
        for a, b in zip(first[category][0] + first[category][1], second[category][0] + second[category][1]):
            assert np.allclose(a, b), category
        for (c1, r1), (c2, r2) in zip(first[category][2], second[category][2]):
            assert np.allclose(c1, c2) and r1 == r2
    # Both supports explicitly: the proximal plate to A'' and the distal one from B''.
    assert len(first["carrier"][0]) == 2
    assert support_rooms(name, LAYOUTS[name], with_modules(base(name), REFERENCE)) == \
        support_rooms(name, LAYOUTS[name], with_modules(base(name), other))


@pytest.mark.parametrize("name", list(LAYOUTS))
def test_the_two_modules_are_independent(name):
    modules = Modules(25.0, 5.0, 25.0, 10.0)
    _, built = solids(name, modules)
    proximal = np.asarray(built.solids["distal_pad"]["other_pad"].polygons[0])
    distal = np.asarray(built.solids["proximal_pad"]["other_pad"].polygons[0])[0]
    assert np.ptp(proximal[:, 1]) == pytest.approx(5.0) and np.ptp(distal[:, 1]) == pytest.approx(10.0)
    assert np.ptp(proximal[:, 0]) == pytest.approx(25.0, abs=0.11) and np.ptp(distal[:, 0]) == pytest.approx(25.0, abs=0.11)
    # Their faces need not line up extended.
    assert proximal[:, 1].min() != pytest.approx(distal[:, 1].min())
    assert evaluate_fit(name, LAYOUTS[name], base(name), modules, 90.0).reason != "invalid_fixed_layout"


@pytest.mark.parametrize("name", list(LAYOUTS))
def test_a_module_longer_than_its_support_is_rejected(name):
    rooms = support_rooms(name, LAYOUTS[name], with_modules(base(name), REFERENCE))
    for modules in (Modules(rooms[0] + 0.5, 5.0, 20.0, 5.0), Modules(20.0, 5.0, rooms[1] + 0.5, 5.0)):
        assert evaluate_fit(name, LAYOUTS[name], base(name), modules, 90.0).reason == "support_exceeded"


def test_touching_is_allowed_and_penetration_is_not():
    # The 2R's proximal module and its support on the link line, ending at the
    # servo's housing (radius 9.5 about A): their top corners touch; 0.1 mm
    # longer, both penetrate it.
    for end, feasible in ((47.5 - 9.5, True), (47.5 - 9.4, False)):
        x = (47.5, 52.5, end, 15.0, 0.0, 0.0, 90.0)
        config = base("revolute")
        length = support_rooms("revolute", x, with_modules(config, REFERENCE))[0]
        fit = evaluate_fit("revolute", x, config, Modules(length, 5.0, 25.0, 5.0), 60.0)
        assert fit.feasible == feasible
        if not feasible:
            assert fit.first_failure_deg == 0.0 and "pip_servo" in fit.collision_pair
            assert fit.reason in ("pad_servo_collision", "structure_collision")


def test_each_mechanism_moves_as_it_is_built():
    import math

    from core.morphology.linkage import FingerLinkage
    from core.optimization.linkage.design import decode

    # The 2R: its flexion itself, to the required angle exactly.
    revolute = motion("revolute", LAYOUTS["revolute"], with_modules(base("revolute"), REFERENCE), 120.0)
    assert revolute.phi_deg[0] == 0.0 and revolute.phi_deg[-1] == 120.0
    assert np.diff(revolute.phi_deg).max() <= envelope.REVOLUTE_STEP_DEG + 1e-9
    # CoDe: the four-bar solved, phi = q_pip + q_dip, to the first pose at or past it.
    config = with_modules(base("linkage"), REFERENCE)
    code = motion("linkage", LAYOUTS["linkage"], config, 120.0)
    linkage = FingerLinkage(decode(LAYOUTS["linkage"], config.link_length_resolution_mm))
    step = math.radians(envelope.LINKAGE_SERVO_STEP_DEG)
    sweep = linkage.sweep_servo(np.arange(len(code.phi_deg)) * step * (1 if code.phi_deg[1] > 0 else -1))
    assert code.phi_deg == pytest.approx(np.degrees(sweep.q_pip_rad + sweep.q_dip_rad))
    assert code.phi_deg[-1] >= 120.0 and code.phi_deg[-2] < 120.0 and code.reached
    # Not a fixed pivot: C's distance from A changes as it flexes.
    a = np.array((linkage.a.x_mm, linkage.a.z_mm))
    assert np.ptp(np.linalg.norm(code.c - a, axis=1)) > 1.0


def test_a_collision_on_the_way_fails_even_if_the_end_is_clear():
    name, x = "revolute", LAYOUTS["revolute"]
    config = base("revolute")
    modules = Modules(20.0, 5.0, 25.0, 5.0)
    assert evaluate_fit(name, x, config, modules, 120.0).feasible
    late = evaluate_fit(name, x, config, modules, 175.0)
    assert not late.feasible
    # The colliding pose between clear ones (the end at 120 deg clear): still infeasible.
    detour = Motion(np.array([0.0, late.first_failure_deg + 1.0, 120.0]), True)
    fit = evaluate_fit(name, x, config, modules, 120.0, detour)
    assert not fit.feasible and fit.first_failure_deg == pytest.approx(late.first_failure_deg + 1.0)


def test_the_sweep_needs_no_optimizer_and_writes_every_pair(monkeypatch):
    import importlib

    import core.optimization.study as study

    # The modules (each package's evaluate() function shadows its module's name).
    linkage_evaluate = importlib.import_module("core.optimization.linkage.evaluate")
    revolute_evaluate = importlib.import_module("core.optimization.revolute.evaluate")

    def forbidden(*args, **kwargs):
        raise AssertionError("the benchmark must not run the optimizer or the design evaluators")

    monkeypatch.setattr(study, "run_study", forbidden)
    monkeypatch.setattr(linkage_evaluate, "evaluate", forbidden)
    monkeypatch.setattr(revolute_evaluate, "evaluate", forbidden)
    layouts = {name: (x, base(name)) for name, x in LAYOUTS.items()}
    before = {name: tuple(x) for name, x in LAYOUTS.items()}
    records = envelope.sweep(layouts, REFERENCE, [15.0, 25.0], [3.0, 10.0], 90.0)
    assert len(records) == 2 * 2 * 4
    keys = {(r["mechanism"], r["experiment"], r["proximal_length_mm"], r["proximal_height_mm"],
             r["distal_length_mm"], r["distal_height_mm"]) for r in records}
    assert len(keys) == len(records)
    assert all(r["reason"] in envelope.REASONS and r["feasible"] == (r["reason"] == "feasible") for r in records)
    assert {name: tuple(x) for name, (x, _) in layouts.items()} == before


def test_a_support_may_not_run_through_its_own_actuator():
    # The 2R's support on the link line, run 5 mm past A, goes through the servo
    # on A (ending on its own pin would be its mounting); the layout's own clears it.
    config = base("revolute")
    assert evaluate_fit("revolute", LAYOUTS["revolute"], config, REFERENCE, 90.0).feasible
    x = (47.5, 52.5, 47.5 + 5.0, 15.0, 0.0, 0.0, 90.0)
    fit = evaluate_fit("revolute", x, config, REFERENCE, 90.0)
    assert not fit.feasible and fit.collision_pair == "proximal_carrier/pip_servo"


def test_the_reference_layouts_are_picked_by_balanced_spare_support():
    import importlib.util

    from core.optimization.mechanism import mechanism, read_config_file

    # The objective: the shorter spare support, the total only breaking ties
    # (the search's own config: the reference modules 25 mm long).
    values = read_config_file(STUDY / "reference" / "revolute.yaml")
    found = mechanism(values.pop("mechanism"))
    values.pop("output")
    config = found.config_from_mapping(found.split_settings(values)[0])
    evaluation = found.evaluate(LAYOUTS["revolute"], config)
    report = found.report(evaluation)
    assert report["support_balance_mm"] == pytest.approx(min(report["proximal_room_mm"] - 25.0,
                                                             report["distal_room_mm"] - 25.0))
    value = found.objective_values(evaluation, ("support_balance",))[0]
    assert -value == pytest.approx(report["support_balance_mm"], abs=0.05)
    spec = importlib.util.spec_from_file_location("pick", STUDY / "reference" / "pick.py")
    pick = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pick)
    rows = [{"support_balance_mm": "5.0", "proximal_room_mm": "40", "distal_room_mm": "30"},
            {"support_balance_mm": "5.0", "proximal_room_mm": "30", "distal_room_mm": "45"},
            {"support_balance_mm": "1.0", "proximal_room_mm": "26", "distal_room_mm": "80"}]
    assert pick.best(rows) is rows[1]  # the balance first, then the total


def test_an_unbuildable_layout_is_invalid_whatever_the_modules():
    # The 2R's servo on A 15 mm from O runs into the MCP servo.
    fit = evaluate_fit("revolute", (15.0, 85.0, 30.0, 15.0, 0.0, 0.0, 90.0), base("revolute"), REFERENCE, 90.0)
    assert fit.reason == "invalid_fixed_layout" and fit.collision_pair == "pip_servo/mcp_servo"
    # CoDe's servo on A' 6 mm above A sits on A's pin.
    x = list(LAYOUTS["linkage"])
    x[3], x[4] = 42.0, 6.0
    assert evaluate_fit("linkage", tuple(x), base("linkage"), REFERENCE, 90.0).reason == "invalid_fixed_layout"
    # The illustrative layouts are buildable.
    for name, layout in LAYOUTS.items():
        assert min(envelope.layout_violations(name, layout, with_modules(base(name), REFERENCE)).values()) >= -1e-6


def test_a_dip_between_samples_is_found():
    # Margin |p - 50| - 1: clear at both samples 0 and 100, colliding in between.
    params, least = np.array([0.0, 100.0]), np.array([49.0, 49.0])
    least_at = lambda p: np.abs(p - 50.0) - 1.0  # noqa: E731
    dip = envelope.first_dip(params, least, np.array([100.0]), least_at, refine_mm=0.5)
    assert dip is not None and 49.0 <= dip <= 51.0
    # Margins that cannot meet across the interval are not resampled.
    assert envelope.first_dip(params, least, np.array([50.0]), least_at) is None


def test_the_pick_takes_the_best_candidate_the_benchmark_passes_and_checks_the_config():
    import importlib.util

    spec = importlib.util.spec_from_file_location("pick", STUDY / "reference" / "pick.py")
    pick = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pick)
    rows = [{"support_balance_mm": "5.0", "proximal_room_mm": "40", "distal_room_mm": "30", "id": 0},
            {"support_balance_mm": "8.0", "proximal_room_mm": "33", "distal_room_mm": "33", "id": 1}]
    assert pick.choose(rows, lambda row: row["id"] == 0) is rows[0]  # the better one fails the benchmark
    assert pick.choose(rows, lambda row: False) is None
    found, config = pick.searched_config(STUDY / "reference" / "revolute.yaml")
    assert pick.config_mismatch(found, config, config) == []
    import dataclasses

    other = dataclasses.replace(config, goal=dataclasses.replace(config.goal, required_rom_rad=2.0,
                                                                 seeds=(5,)))
    assert pick.config_mismatch(found, other, config) == ["goal.required_rom_rad"]  # not the seeds
