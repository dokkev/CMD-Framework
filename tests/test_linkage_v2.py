"""Linkage formulation v2: the ROM interval, the C_x stopper, the objectives."""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import pytest
import yaml

from core.morphology.linkage.packaging import ContactPad, Packaging
from core.optimization.linkage import VARIABLE_NAMES, decode
from core.optimization.linkage_v2 import CONSTRAINT_NAMES, MECHANISM, LinkageV2Config, evaluate
from core.optimization.linkage_v2.evaluate import finger_length_mm
from core.optimization.mechanism import mechanism

CONFIG_FILE = Path(__file__).resolve().parents[1] / "scripts/optimize/linkage_v2/config.yaml"
BOUNDS = {name: (0.0, 200.0) for name in VARIABLE_NAMES}
THIN = Packaging(pivot_clearance_diameter_mm=6.5,
                 proximal_pad=ContactPad(height_mm=1.0, chamfer_mm=0.0),
                 distal_pad=ContactPad(height_mm=1.0, chamfer_mm=0.0))
# The v1 test design that curls to 180 deg safely (thin pads, its own servo tilt).
GOOD = (36.0, 79.3, 10.0, 41.6, 12.6, 63.3, 17.4, 41.9, 30.0, 0.0)
BASE = LinkageV2Config(bounds=BOUNDS, packaging=THIN, pip_servo_tilt_rad=None,
                       minimum_proximal_support_mm=0.0, minimum_distal_support_mm=0.0)


def at(degrees: float, config: LinkageV2Config = BASE, **changes) -> LinkageV2Config:
    return dataclasses.replace(config, required_rom_rad=math.radians(degrees), **changes)


def test_flexion_past_the_required_rom_does_not_matter() -> None:
    # A transmission margin of 50 deg is lost at phi = 158.5 deg.
    strict = dataclasses.replace(BASE, minimum_transmission_margin_rad=math.radians(50.0),
                                 enforce_curl_tip=False)
    reach = math.degrees(evaluate(decode(GOOD), at(180.0, strict)).phi_safe_rad)
    assert 150.0 < reach < 170.0
    assert evaluate(decode(GOOD), at(reach - 2.0, strict)).constraints["rom_deg"] <= 0.0
    beyond = evaluate(decode(GOOD), at(reach + 2.0, strict))
    assert beyond.constraints["rom_deg"] == pytest.approx(reach + 2.0 - reach, abs=0.6)
    assert beyond.stop_reason == "transmission"


def test_the_margin_extends_the_safe_interval_not_the_rom() -> None:
    strict = dataclasses.replace(BASE, minimum_transmission_margin_rad=math.radians(50.0),
                                 enforce_curl_tip=False)
    reach = math.degrees(evaluate(decode(GOOD), at(180.0, strict)).phi_safe_rad)
    margin = math.radians(5.0)
    assert evaluate(decode(GOOD), at(reach - 6.0, strict, rom_margin_rad=margin)).constraints["rom_deg"] <= 0.0
    tight = evaluate(decode(GOOD), at(reach - 4.0, strict, rom_margin_rad=margin))
    assert tight.constraints["rom_deg"] > 0.0
    # phi_req is still what k_max is measured to.
    assert tight.required_rom_rad == pytest.approx(math.radians(reach - 4.0))


def test_k_max_is_measured_on_zero_to_phi_req_only() -> None:
    values = [evaluate(decode(GOOD), at(degrees)).torque_ratio_max for degrees in (60.0, 90.0, 120.0, 150.0)]
    assert values == sorted(values)          # a longer interval never lowers the worst
    assert values[0] < values[-1]
    shallow = evaluate(decode(GOOD), at(60.0))
    # The interval ends at the first servo sample at or past phi_req.
    assert math.degrees(shallow.usable_pip_rad) < 60.0


def test_curling_behind_o_after_phi_req_is_stopped_not_rejected() -> None:
    longer = list(GOOD)
    longer[VARIABLE_NAMES.index("bc_length_mm")] = 60.0
    full = evaluate(decode(longer), at(180.0))
    # C passes behind O at about 137 deg, so the curl is limited to there.
    crossing = math.degrees(full.phi_max_rad)
    assert 130.0 < crossing < 145.0 and full.constraints["curl_tip_x_mm"] > 0.0
    before = evaluate(decode(longer), at(crossing - 3.0))
    assert before.feasible and before.least_tip_x_mm >= 0.0
    assert math.degrees(before.phi_safe_rad) > crossing  # it could curl further: no matter
    after = evaluate(decode(longer), at(crossing + 3.0))
    assert not after.feasible and after.constraints["curl_tip_x_mm"] > 0.0


def test_objectives_and_constraints_are_wired_as_specified() -> None:
    assert MECHANISM.active_objectives(BASE) == ("transmission", "finger_length")
    assert set(MECHANISM.objectives) == {"transmission", "finger_length"}  # no curl, no pad length
    e = MECHANISM.evaluate(GOOD, at(150.0))
    assert MECHANISM.objective_values(e, ("transmission", "finger_length")) == (
        e.torque_ratio_max, pytest.approx(math.hypot(79.3 + 30.0, 10.0)))
    assert e.finger_length_mm == finger_length_mm(decode(GOOD))
    assert "rom_deg" in CONSTRAINT_NAMES and "minimum_flexion_deg" not in CONSTRAINT_NAMES
    assert len(e.g) == len(CONSTRAINT_NAMES)
    # L_p is the proximal pad's mounting-line length with A'' included.
    need = e.proximal_support_length_mm + 5.0
    short = MECHANISM.evaluate(GOOD, at(150.0, minimum_proximal_support_mm=need))
    assert short.constraints["proximal_support_mm"] == pytest.approx(5.0)
    longer = MECHANISM.evaluate((*GOOD, 5.0), at(150.0, minimum_proximal_support_mm=need))
    assert longer.constraints["proximal_support_mm"] <= 1e-9
    # The palmar face is the a-b less the chamfer.
    chamfered = dataclasses.replace(THIN, proximal_pad=ContactPad(height_mm=1.0, chamfer_mm=1.0))
    face = MECHANISM.evaluate(GOOD, at(150.0, packaging=chamfered))
    assert face.proximal_face_mm == pytest.approx(face.proximal_support_length_mm - 1.0)


def test_bound_hits_name_the_variables_at_their_search_limits() -> None:
    bounds = dict(BOUNDS, bc_length_mm=(30.0, 60.0))
    e = MECHANISM.evaluate(GOOD, at(150.0, bounds=bounds))
    assert "bc_length_mm:low" in e.bound_hits
    assert MECHANISM.report(e)["bound_hits"].count("bc_length_mm:low") == 1


def test_the_v2_config_file_pins_everything_and_is_registered() -> None:
    raw = yaml.safe_load(CONFIG_FILE.read_text())
    assert "base" not in raw  # it does not inherit the v1 study config
    for key in ("height_mm", "chamfer_mm", "offset_mm"):
        assert key in raw["pad"]["proximal"] and key in raw["pad"]["distal"]
    for key in ("pip_servo", "mcp_servo", "pivot_clearance_diameter_mm", "body_clearance_mm",
                "ap_clearance_diameter_mm"):
        assert key in raw["packaging"]
    from core.optimization.study import load_yaml

    found = mechanism("linkage_v2")
    config, _ = load_yaml(found, CONFIG_FILE)
    assert found is MECHANISM and found.formulation != mechanism("linkage").formulation
    assert config.required_rom_rad == pytest.approx(math.radians(150.0))
    assert (config.minimum_proximal_support_mm, config.minimum_distal_support_mm) == (30.0, 40.0)
    assert config.packaging.proximal_pad == ContactPad(height_mm=10.0, offset_mm=0.0, chamfer_mm=2.5)
    assert config.search_bounds["servo_tilt_deg"] == (20.0, 20.0)
    assert config.hypervolume_reference == {"transmission": 2.3, "finger_length": 150.0}
    assert config.link_length_resolution_mm == 0.1
    with pytest.raises(ValueError, match="180 deg"):
        dataclasses.replace(config, required_rom_rad=math.pi, rom_margin_rad=0.1)
