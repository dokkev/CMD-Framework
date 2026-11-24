"""The simulator description export (``scripts/export/code_hand_sim.py``): servo visuals and their bodies."""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

DESIGN = Path(__file__).resolve().parents[1]
STUDY3_O = DESIGN / "output" / "sim" / "study3" / "hands" / "cfl_q120_h3to7_h0.yaml"


@pytest.fixture(scope="module")
def export():
    spec = importlib.util.spec_from_file_location("code_hand_sim", DESIGN / "scripts" / "export" / "code_hand_sim.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["code_hand_sim"] = module  # (its dataclasses look their module up)
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


def test_palm_housing_encloses_the_palm_servos_and_clears_the_mcp_joints(export, data):
    params = data["palm_housing"]
    assert params == dict(export.dataclasses.asdict(export.PALM_HOUSING))
    palm = data["bodies"][0]
    assert palm["name"] == "palm" and "capsule" in palm  # the capsule stays for the placeholder inertia
    boxes = {b["name"]: b for b in palm["housing"]}
    assert set(boxes) == {"palm_hood", "palm_block", "palm_bridge", "palm_heel"}
    lo = {n: np.subtract(b["center"], b["half_extents"]) * 1e3 for n, b in boxes.items()}
    hi = {n: np.add(b["center"], b["half_extents"]) * 1e3 for n, b in boxes.items()}
    joints = {j["name"]: j for j in data["joints"]}
    fe_x = min(joints[f"{d}_fe"]["anchor"][0] for d in ("index", "middle", "ring")) * 1e3
    assert max(h[0] for h in hi.values()) == pytest.approx(fe_x - params["front_setback_mm"], abs=1e-6)
    visuals = {v["servo"]: v for v in data["visuals"]}
    depth = data["servo"]["shape"]["depth"] * 1e3
    for d in ("index", "middle", "ring"):  # the hood spans each abduction servo up to its wall
        p = np.asarray(visuals[f"{d}/aa"]["position"]) * 1e3
        assert lo["palm_hood"][1] < p[1] < hi["palm_hood"][1]
        assert hi["palm_hood"][2] >= p[2] + depth / 2 + params["wall_mm"] - 1e-6
    # the flat palmar face lies below the abduction servos by the palmar depth
    aa_bottom = min(np.asarray(visuals[f"{d}/aa"]["position"])[2] * 1e3 - depth / 2
                    for d in ("index", "middle", "ring"))
    assert lo["palm_block"][2] == pytest.approx(aa_bottom - params["palmar_depth_mm"], abs=1e-6)
    ro = np.asarray(visuals["thumb/ro"]["position"]) * 1e3  # the heel holds the thumb's rotation servo
    assert np.all(lo["palm_heel"] < ro) and np.all(ro < hi["palm_heel"])


def test_only_the_palm_has_a_housing(data):
    assert all("housing" not in body for body in data["bodies"][1:])
