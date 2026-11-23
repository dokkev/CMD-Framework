"""The simulator description export (``scripts/export/code_hand_sim.py``): servo visuals and their bodies."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

DESIGN = Path(__file__).resolve().parents[1]
STUDY3_O = DESIGN / "output" / "sim" / "study3" / "hands" / "cfl_q120_h3to7_h0.yaml"


@pytest.fixture(scope="module")
def export():
    spec = importlib.util.spec_from_file_location("code_hand_sim", DESIGN / "scripts" / "export" / "code_hand_sim.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def data(export):
    return export.export(STUDY3_O if STUDY3_O.exists() else export.PROTOTYPE)


def test_every_servo_is_drawn_on_a_body_of_the_tree(data):
    bodies = {b["name"] for b in data["bodies"]}
    visuals = {v["servo"]: v for v in data["visuals"]}
    assert set(visuals) == {f"{d}/{j}" for d in ("index", "middle", "ring") for j in ("aa", "fe", "pip")} | {
        f"thumb/{j}" for j in ("ro", "raa", "mcp", "ip")}
    assert all(v["body"] in bodies and v["shape"] == "servo" for v in visuals.values())
    shape = data["servo"]["shape"]
    assert shape["type"] == "d_extrusion" and shape["depth"] > 0 and shape["rear_extent"] >= shape["radius"]


def test_servo_bodies_ride_on_the_segment_their_frame_moves_with(data):
    visuals = {v["servo"]: v for v in data["visuals"]}
    assert visuals["index/aa"]["body"] == "palm"
    assert visuals["index/fe"]["body"] == "index_mcp"
    assert visuals["index/pip"]["body"] == "index_proximal"
    assert visuals["thumb/mcp"]["body"] == "thumb_proximal"
    # the IP servo's body lies beyond the IP axis, in the distal segment: it turns with the IP joint
    assert visuals["thumb/ip"]["body"] == "thumb_distal"


def test_servo_shafts_lie_on_their_joint_axes(data):
    joints = {j["name"]: j for j in data["joints"]}
    pairs = {"index/aa": "index_aa", "index/fe": "index_fe", "index/pip": "index_pip_servo", "thumb/ro": "thumb_ro",
             "thumb/raa": "thumb_raa", "thumb/mcp": "thumb_mcp", "thumb/ip": "thumb_ip"}
    for item in data["visuals"]:
        if item["servo"] not in pairs:
            continue
        joint = joints[pairs[item["servo"]]]
        w, x, y, z = item["quat_wxyz"]
        shaft = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        assert np.allclose(item["position"], joint["anchor"], atol=1e-9)
        assert abs(abs(shaft @ np.asarray(joint["axis"])) - 1.0) < 1e-6  # (quaternions exported to 9 digits)
