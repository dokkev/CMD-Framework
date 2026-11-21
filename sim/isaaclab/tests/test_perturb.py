# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""The P and D contact-face perturbations on a minimal one-finger description."""

import numpy as np
import pytest

from code_isaaclab.assets.perturb import offset_distal_faces, offset_fingers

Q = [0.707106781, -0.707106781, 0.0, 0.0]  # local +y -> hand -z (the export's pad orientation)


def _desc():
    def body(name, x, boxes=None):
        return {"name": name, "origin": [x, 0.0, -0.006], "com": [x, 0.0, -0.006],
                "capsule": {"start": [x, 0.0, -0.006], "end": [x + 0.01, 0.0, -0.006], "radius": 0.003},
                "boxes": boxes}
    pad = lambda n, x: {"name": n, "center": [x, 0.0, -0.0105], "quat_wxyz": Q, "half_extents": [0.02, 0.0015, 0.0095]}  # noqa: E731
    return {
        "bodies": [body("index_mcp", 0.014), body("index_proximal", 0.038, [pad("index_proximal_pad", 0.06)]),
                   body("index_servo_link", 0.08), body("index_driver_link", 0.064),
                   body("index_distal", 0.117, [pad("index_distal_pad", 0.11)])],
        "joints": [{"name": "index_fe", "parent": "index_mcp", "child": "index_proximal", "anchor": [0.038, 0.0, -0.006]},
                   {"name": "index_pip_servo", "parent": "index_proximal", "child": "index_servo_link", "anchor": [0.08, 0.0, 0.002]},
                   {"name": "index_b", "parent": "index_servo_link", "child": "index_distal", "anchor": [0.12, 0.0, -0.006]}],
        "loops": [{"name": "index_bp", "parent": "index_driver_link", "child": "index_distal", "anchor": [0.097, 0.0, 0.003]}],
        "tips": {"index": [0.128, 0.0, -0.006]},
    }


def test_offset_fingers_moves_the_carried_bodies_not_the_mcp_axis():
    d, out = _desc(), offset_fingers(_desc(), 0.004, digits=("index",))
    J = {j["name"]: j for j in out["joints"]}
    assert J["index_fe"]["anchor"] == d["joints"][0]["anchor"]
    assert J["index_pip_servo"]["anchor"][2] == pytest.approx(0.002 - 0.004)
    B = {b["name"]: b for b in out["bodies"]}
    assert B["index_mcp"]["origin"][2] == pytest.approx(-0.006)
    for name in ("index_proximal", "index_distal"):
        assert B[name]["boxes"][0]["center"][2] == pytest.approx(-0.0105 - 0.004)
    assert out["tips"]["index"][2] == pytest.approx(-0.010) and out["loops"][0]["anchor"][2] == pytest.approx(-0.001)
    assert d["bodies"][1]["origin"][2] == pytest.approx(-0.006)  # input untouched


def test_offset_distal_faces_moves_only_the_distal_contact_face():
    out = offset_distal_faces(_desc(), 0.004, digits=("index",))
    B = {b["name"]: b for b in out["bodies"]}
    prox, dist = B["index_proximal"]["boxes"][0], B["index_distal"]["boxes"][0]
    assert prox["center"][2] == pytest.approx(-0.0105) and prox["half_extents"][1] == pytest.approx(0.0015)
    mount = dist["center"][2] + dist["half_extents"][1]   # local -y side, toward the link (hand +z)
    face = dist["center"][2] - dist["half_extents"][1]    # local +y side, the contact face (hand -z)
    assert mount == pytest.approx(-0.0105 + 0.0015)
    assert face == pytest.approx(-0.0105 - 0.0015 - 0.004)
    assert np.allclose(B["index_distal"]["origin"], _desc()["bodies"][4]["origin"])
