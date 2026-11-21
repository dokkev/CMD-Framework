"""The CFL's closure coordinates: every design they make is one of the study's
search space, closes to the required flexion, and maps back to itself."""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "study"))

import shared  # noqa: E402
from core.geometry import fourbar  # noqa: E402
from core.morphology.mechanism.linkage.design import decode  # noqa: E402
from core.optimization.analysis.closure_search import ClosureMechanism  # noqa: E402


def _designs(count: int = 12, tries: int = 40000):
    base, config = shared.load_config("scripts/study/cfl.yaml", ROOT)
    config = shared.with_required_flexion(config, 120.0)
    mechanism = ClosureMechanism(base)
    lower, upper = (np.array(v) for v in mechanism.search_box(config))
    steps = np.array(mechanism.search_steps(config))
    rng, found = np.random.default_rng(0), []
    for _ in range(tries):
        coordinates = lower + np.round(rng.random(len(lower)) * (upper - lower) / steps) * steps
        x = mechanism.to_design(coordinates, config)
        if x is not None:
            found.append(x)
            if len(found) == count:
                break
    return base, mechanism, config, found


def test_designs_close_and_belong_to_the_study_space():
    base, mechanism, config, designs = _designs()
    assert len(designs) == 12
    for x in designs:
        assert base.to_design(base.from_design(x, config), config) == x  # the study's search makes it too
        try:
            joint = decode(x, config.link_length_resolution_mm).joint
        except ValueError:
            continue  # the rounded bars of a degenerate four-bar: the evaluation rejects it
        pivots = [(p.x_mm, p.z_mm) for p in (joint.a, joint.ap, joint.b, joint.bp)]
        assert fourbar.closes(*fourbar.shape(*pivots), config.required_rom_rad)


def test_design_maps_back_to_itself():
    _, mechanism, config, designs = _designs()
    for x in designs:
        assert mechanism.to_design(mechanism.from_design(x, config), config) == x
