"""What a contact face loses in the fold (packaging.fold_free_lengths), and the
linkage options it goes with (pad drops, overhang)."""

import math
from pathlib import Path

import numpy as np
import pytest

from core.morphology.packaging.faces import fold_free_lengths
from core.optimization.mechanism import mechanism
from core.morphology.linkage.packaging import pip_footprint

CONFIG = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "linkage" / "config.yaml"
DROPS = ("goal.pads.on_links=false", "goal.pads.proximal.drop_mm=[0, null]", "goal.pads.distal.drop_mm=[0, null]")
PIVOT = np.array((50.0, 0.0))


def folded(phi_deg, proximal_depth=8.0, distal_depth=8.0, proximal=(10.0, 45.0), distal=(5.0, 45.0)):
    """Two faces about a pivot: the proximal one fixed, ``proximal_depth``
    below the axis over ``proximal`` (x); the distal one ``distal_depth``
    below its segment over ``distal`` (from the pivot), turned by ``phi``."""

    phi = np.radians(np.atleast_1d(phi_deg))
    e = np.stack((np.cos(phi), -np.sin(phi)), axis=1)
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)
    proximal_face = np.array(((proximal[0], -proximal_depth), (proximal[1], -proximal_depth)))
    outer = PIVOT + distal[1] * e - distal_depth * n
    inner = PIVOT + distal[0] * e - distal_depth * n
    return fold_free_lengths(proximal_face, (0.0, -1.0), np.stack((outer, inner), axis=1), -n)


def crossing(phi_deg, depth=8.0, proximal=(10.0, 45.0), distal=(5.0, 45.0)):
    """The v3 rule: each face up to where the two faces' lines cross."""

    phi = math.radians(phi_deg)
    e, n = np.array((math.cos(phi), -math.sin(phi))), np.array((math.sin(phi), math.cos(phi)))
    if abs(e[1]) < 1e-12:
        return proximal[1] - proximal[0], distal[1] - distal[0]
    u = (-depth - PIVOT[1] + depth * n[1]) / e[1]
    x = PIVOT[0] + u * e[0] - depth * n[0]
    return (float(np.clip(x - proximal[0], 0.0, proximal[1] - proximal[0])),
            float(np.clip(distal[1] - u, 0.0, distal[1] - distal[0])))


@pytest.mark.parametrize("phi_deg", [0.0, 30.0, 90.0, 120.0, 150.0])
def test_at_equal_depths_it_is_the_crossing_rule(phi_deg):
    free = folded(phi_deg)
    assert (free["proximal_pad"][0], free["distal_pad"][0]) == pytest.approx(crossing(phi_deg), abs=1e-9)


def faces_gap(phi_deg, proximal_depth, distal_depth, proximal=(10.0, 45.0), distal=(5.0, 45.0)):
    """Least distance between the two face segments (0: they touch or cross)."""

    phi = np.radians(np.atleast_1d(phi_deg))
    e = np.stack((np.cos(phi), -np.sin(phi)), axis=1)
    n = np.stack((-e[:, 1], e[:, 0]), axis=1)
    p0, p1 = np.array((proximal[0], -proximal_depth)), np.array((proximal[1], -proximal_depth))
    d0 = PIVOT + distal[1] * e - distal_depth * n
    d1 = PIVOT + distal[0] * e - distal_depth * n

    def to_segment(points, a, b):
        t = np.clip(np.sum((points - a) * (b - a), axis=-1) / np.sum((b - a) ** 2, axis=-1), 0.0, 1.0)
        return np.linalg.norm(points - (a + t[..., None] * (b - a)), axis=-1)

    s = np.linspace(0.0, 1.0, 41)[:, None, None]
    along_distal = d0[None] + s * (d1 - d0)[None]
    along_proximal = p0 + s[:, :, 0] * (p1 - p0)
    return np.minimum(to_segment(along_distal, p0, p1).min(axis=0),
                      to_segment(along_proximal[:, None, :], d0[None], d1[None]).min(axis=0))


def test_what_is_lost_grows_continuously_with_the_flexion():
    # Wherever the faces stay apart (touching, the pads are at their collision
    # boundary, which the checks exclude; there, perpendicular, the other face's
    # end projects onto a whole stretch of the face at once).
    phi = np.arange(0.0, 180.0, 0.01)
    for depths in ((8.0, 8.0), (8.0, 9.5), (5.0, 11.0)):
        free = folded(phi, *depths)
        apart = faces_gap(phi, *depths) > 0.5
        steps = apart[:-1] & apart[1:]
        for side in ("proximal_pad", "distal_pad"):
            assert np.abs(np.diff(free[side]))[steps].max() < 0.05  # mm per 0.01 deg


def test_barely_flexed_faces_at_different_depths_lose_nothing():
    # Their lines cross far away (at x ~ x_pivot + depth difference / phi),
    # through the middle of a pad at some small phi: the v3 rule's artifact.
    phi = np.arange(0.01, 3.0, 0.01)
    free = folded(phi, 8.0, 9.5)
    assert free["proximal_pad"] == pytest.approx(35.0) and free["distal_pad"] == pytest.approx(40.0)


def test_a_face_hides_only_where_it_is_opposite():
    # A distal pad far from the joint (u 30-45) covers none of the proximal
    # face's part near the joint: folded to 150 deg the crossing rule cuts it,
    # this one only where the distal face lies opposite.
    short = folded(150.0, distal=(30.0, 45.0))
    long = folded(150.0, distal=(5.0, 45.0))
    assert short["proximal_pad"][0] > long["proximal_pad"][0]


def load(*overrides):
    from core.optimization.study import load_yaml

    found = mechanism("linkage")
    return found, load_yaml(found, CONFIG, overrides)[0]


def test_a_linkage_with_drop_bounds_searches_its_pads_drops():
    found, config = load(*DROPS)
    assert config.searched_drops == ("proximal_drop_mm", "distal_drop_mm")
    assert config.bounds["proximal_drop_mm"] == (0.0, config.packaging.pivot_radius_mm("a"))
    assert config.bounds["distal_drop_mm"] == (0.0, config.packaging.pivot_radius_mm("b"))
    names = found.search_names(config)
    assert names[-2:] == ("proximal_drop_mm", "distal_drop_mm")
    # The production study holds them at its packaging's (NaN in the design).
    from core.optimization.study import load_yaml

    production = load_yaml(found, CONFIG, ())[0]
    assert production.searched_drops == ()
    assert production.bounds["proximal_drop_mm"] == (production.packaging.proximal_pad_drop_mm,) * 2


def test_an_overhanging_pad_is_limited():
    from core.morphology.linkage import FingerLinkage
    from core.morphology.linkage.design import decode
    from core.optimization.linkage.evaluate import overhang_violations

    found, config = load("goal.pads.proximal.overhang_max_mm=10", "goal.pads.distal.overhang_max_mm=10")
    _, strict = load("goal.pads.proximal.overhang_max_mm=0", "goal.pads.distal.overhang_max_mm=0")
    x = [36.0, 75.0, 0.0, 40.0, 12.0, 72.0, 8.0, 20.0, 25.0, 0.0, 15.0, 5.0, 0.0, 0.0, 3.0, 3.0]
    linkage = FingerLinkage(decode(x, config.link_length_resolution_mm))
    p = linkage.parameters
    front = max(p.a_x_mm + 3.0, float(pip_footprint(config.packaging, linkage).outline[:, 0].max()))
    over = p.app_x_mm - front
    assert over > 0.0
    assert overhang_violations(linkage, strict)[0] == pytest.approx(over)
    assert overhang_violations(linkage, config)[0] == pytest.approx(max(over - 10.0, 0.0))
