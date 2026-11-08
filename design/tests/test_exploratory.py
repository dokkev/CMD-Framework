"""Focused checks of the exploratory-study helpers (scripts/study/exploratory)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

EXPLORATORY = Path(__file__).resolve().parents[1] / "scripts" / "study" / "exploratory"
sys.path.insert(0, str(EXPLORATORY))

import common  # noqa: E402
import e01_packaging as e01  # noqa: E402
import e02_polycentric_ablation as e02  # noqa: E402
import e02b_palmar_pivot as e02b  # noqa: E402
import e10_design_drivers as e10  # noqa: E402
from core.optimization.analysis.tradeoff import Modules  # noqa: E402

REFS = common.references()
CATALOG = common.catalog()


def test_codes_icr_at_extension_is_its_articulation_descriptor():
    # The crossed-link ICR traced by E02b/E10 matches the partition's extended ICR (E).
    code = REFS["code_matched"]
    path = e02b.icr_path(code, CATALOG["baseline"].modules)
    articulation = common.shared.partitions.partition("linkage").external(code.x, code.config)["articulation_mm"]
    assert path[0]["phi_deg"] == 0
    assert path[0]["icr_x_mm"] == pytest.approx(articulation, abs=0.05)


def test_packaging_raster_reads_the_pin_housings_as_codes_deepest_palmar_hardware():
    row, profile = e01.analyse(REFS["code_matched"], CATALOG["baseline"].modules)
    radius = common.goal_of(REFS["code_matched"].config).packaging.pivot_radius_mm("a")
    assert row["max_intrusion_mm"] == pytest.approx(radius, abs=e01.STEP)
    assert len(profile["x_mm"]) == len(profile["h_mm"])


def test_the_ideal_2r_ablation_shares_codes_external_morphology():
    ideal, config = e02.ideal_2r_config()
    code = REFS["code_matched"]
    modules = Modules(20.0, 3.0, 20.0, 3.0)
    x_code, carried = common.carried(code, modules)
    external = common.shared.partitions.partition("linkage").external(x_code, carried)
    part = common.shared.partitions.partition("revolute_idealized")
    config = common.with_modules(config, modules)
    coordinates, template = part.template(external, "linkage", x_code, config)
    x = part.compose(external, coordinates, template, config)
    rebuilt = part.external(x, config)
    for name in ("finger_length_mm", "proximal_span_mm", "distal_span_mm", "articulation_mm"):
        assert rebuilt[name] == pytest.approx(external[name], abs=1e-6)


def test_partial_correlation_removes_a_shared_driver():
    rng = np.random.default_rng(0)
    span = rng.normal(size=400)
    feature = span + 0.1 * rng.normal(size=400)  # tracks the span only
    target = span + 0.1 * rng.normal(size=400)
    assert e10.spearman(feature, target) > 0.8
    assert abs(e10.partial(feature, target, span[:, None])) < 0.2
