"""Contact-interface retention: the 2R and RCJ evaluators."""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from core.morphology.linkage.packaging import ContactPad, Packaging
from core.optimization.mechanism import mechanism, read_config_file
from core.optimization.optimizer import OptimizerSettings
from core.optimization.retention import RetentionConfig, front_retention
from core.optimization.retention.geometry import Module
from core.optimization.study import load_run, load_yaml, run_study, save_result

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts" / "optimize"
NAMES = ("open_chain_idealized", "open_chain_packaged", "rolling_contact_joint")


def study(name: str, *overrides: str) -> tuple:
    found = mechanism(name)
    config, _ = load_yaml(found, SCRIPTS / name / "config.yaml", overrides)
    return found, config


def test_retention_is_the_kept_trapezoid_over_the_nominal_block() -> None:
    proximal = Module(10.0, 1.0, 5.0, 40.0, line_mm=30.0, palmar_mm=20.0)
    distal = Module(45.0, -1.0, 5.0, 40.0, line_mm=40.0, palmar_mm=40.0)

    assert proximal.retention == pytest.approx(25.0 / 40.0)
    assert distal.retention == 1.0
    # The outer edge a-c stays; only the joint-facing vertices b and d move.
    assert {tuple(p) for p in proximal.outline()} == {(10.0, -5.0), (30.0, -5.0), (40.0, 0.0), (10.0, 0.0)}
    assert {tuple(p) for p in distal.outline()} == {(5.0, -5.0), (45.0, -5.0), (45.0, 0.0), (5.0, 0.0)}
    assert Module(10.0, 1.0, 5.0, 40.0, 0.0, 0.0).empty


def test_every_mechanism_shares_the_nominal_modules_and_the_rom() -> None:
    configs = [study(name)[1] for name in NAMES]
    for config in configs[1:]:
        assert config.packaging == configs[0].packaging
        assert config.required_rom_rad == configs[0].required_rom_rad
        assert config.extended_length_bounds_mm == configs[0].extended_length_bounds_mm
    pad = configs[0].packaging.proximal_pad
    assert (pad.height_mm, pad.chamfer_mm, pad.length_mm) == (5.0, 0.0, 45.0)
    # One study per required ROM, set from the command line.
    assert study(NAMES[0], "motion.required_rom_deg=150")[1].required_rom_rad == pytest.approx(math.radians(150))
    values = read_config_file(SCRIPTS / NAMES[0] / "config.yaml", ["pad.distal.length_mm=50"])
    assert values["pad"]["distal"]["length_mm"] == 50 and values["pad"]["proximal"]["length_mm"] == 45.0


def test_the_module_is_the_unchamfered_block_of_a_set_nominal_length() -> None:
    with pytest.raises(ValueError, match="chamfer_mm must be 0"):
        RetentionConfig(bounds={}, packaging=Packaging(proximal_pad=ContactPad(length_mm=40.0)))
    with pytest.raises(ValueError, match="nominal length_mm"):
        RetentionConfig(bounds={}, packaging=Packaging(proximal_pad=ContactPad(chamfer_mm=0.0)))


def test_the_rom_is_swept_continuously_to_the_required_pose() -> None:
    found, config = study("open_chain_idealized", "motion.required_rom_deg=90")
    assert config.rom_samples_rad[0] == 0.0 and config.rom_samples_rad[-1] == pytest.approx(math.pi / 2)
    design = (50.0, 45.0, 30.0, 28.0, 30.0, 28.0)

    at_90 = found.evaluate(design, config)
    assert at_90.feasible and at_90.rom_reached_rad == pytest.approx(math.pi / 2)
    # Folding flat, the segments' modules overlap: the 2R stops short of 180 deg.
    at_180 = found.evaluate(design, dataclasses.replace(config, required_rom_rad=math.pi))
    assert not at_180.feasible
    assert math.pi / 2 <= at_180.rom_reached_rad < math.pi
    assert at_180.first_collision


def test_packaging_adds_the_housings_and_nothing_else() -> None:
    idealized, config = study("open_chain_idealized", "motion.required_rom_deg=90")
    packaged = mechanism("open_chain_packaged")
    assert set(idealized.pairs) < set(packaged.pairs)
    assert {p.first for p in set(packaged.pairs) - set(idealized.pairs)} | {
        p.second for p in set(packaged.pairs) - set(idealized.pairs)} >= {"a_servo", "mcp_servo"}

    # The proximal module run up to A: clear of the pins, inside the housing at A.
    design = (50.0, 45.0, 34.0, 34.0, 30.0, 28.0)
    assert idealized.evaluate(design, config).feasible
    e = packaged.evaluate(design, config)
    assert not e.feasible and e.margins_mm["proximal_module-a_servo"] < 0.0
    # Retention itself does not depend on the packaging.
    assert e.proximal_retention == idealized.evaluate(design, config).proximal_retention


def test_transmission_is_a_diagnostic_over_the_required_rom() -> None:
    two_r, config = study("open_chain_packaged", "motion.required_rom_deg=90")
    rcj = mechanism("rolling_contact_joint")
    e = two_r.evaluate((50.0, 45.0, 20.0, 20.0, 20.0, 20.0), config)
    assert (e.torque_ratio_min, e.torque_ratio_max) == (1.0, 1.0)
    assert math.degrees(e.servo_travel_rad) == pytest.approx(90.0)
    assert "transmission" not in two_r.objectives and "transmission" not in rcj.objectives
    assert not any("torque" in name for name in two_r.constraint_names + rcj.constraint_names)


def test_the_rcj_rolls_two_to_one_and_is_provisional() -> None:
    rcj, config = study("rolling_contact_joint", "motion.required_rom_deg=180")
    allowance = dataclasses.replace(config, cable_allowance_mm=2.0)
    lower, _ = rcj.search_box(allowance)
    assert dict(zip(rcj.variable_names, lower))["radius_mm"] == pytest.approx(11.5)

    geometry = dict(zip(rcj.geometry_names, (40.0, 0.0, 12.0, 0.0, 40.0)))
    modules = rcj.module_outlines((*geometry.values(), 20.0, 20.0, 20.0, 20.0), geometry, config)
    solids = rcj.solids(geometry, modules, config).solids
    # At 180 deg of flexion the carrier has turned 90 deg: B straight palmar of A.
    b = solids["distal_roller"].centers[-1]
    assert b == pytest.approx((40.0, -24.0))
    link = solids["distal_link"]
    assert (link.end[-1] - link.start[-1]) == pytest.approx((-40.0, 0.0), abs=1e-9)

    e = rcj.evaluate((*geometry.values(), 20.0, 20.0, 20.0, 20.0), config)
    assert e.provisional == "cable routing unverified"
    assert (e.torque_ratio_min, e.torque_ratio_max) == (2.0, 2.0)
    assert rcj.report(e)["provisional"]


def test_the_front_figure_is_the_best_least_retention() -> None:
    found = mechanism("open_chain_packaged")
    rows = [{"proximal_retention": 1.0, "distal_retention": 0.2},
            {"proximal_retention": 0.7, "distal_retention": 0.6},
            {"proximal_retention": 0.5, "distal_retention": 0.9}]
    objectives = [(-r["proximal_retention"], -r["distal_retention"]) for r in rows]
    roles = found.seed_roles(objectives, rows, ("proximal_retention", "distal_retention"))

    assert roles["maximin"] == 1 and roles["best_proximal"] == 0 and roles["best_distal"] == 2
    assert front_retention(rows) == pytest.approx(0.6)
    assert found.role_order(("proximal_retention", "distal_retention"))[0] == "maximin"


@pytest.mark.parametrize("name", NAMES)
def test_each_mechanism_optimizes_saves_and_reloads(name: str, tmp_path) -> None:
    found, config = study(name, "motion.required_rom_deg=90")
    result = run_study(found, config, OptimizerSettings(population_size=12, generations=3))
    run = load_run(save_result(result, tmp_path / "run"))

    assert run.mechanism is found and run.current
    assert run.objectives == ("proximal_retention", "distal_retention")
    assert run.config == config
    for candidate in result.pareto:
        assert candidate.evaluation.feasible
        assert 0.0 <= candidate.evaluation.min_retention <= 1.0
    rows = run.candidates()
    if rows:
        assert {"proximal_retention", "distal_retention", "provisional", "torque_ratio_max"} <= set(rows[0])
        assert np.isfinite(float(rows[0]["extended_length_mm"]))
