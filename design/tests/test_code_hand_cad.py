"""The CoDe parametric model against the saved CAD snapshot, and the Onshape mate algebra (no network)."""

import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from core.cad.code_hand import (
    CAD_TURNED_SERVOS,
    CadMap,
    actuator_readings,
    actuator_zeros_from_snapshot,
    compare,
    configuration_from_snapshot,
    layout_from_snapshot,
    linkage_from_variables,
)
from core.cad.layout_mates import (
    MateParameters,
    face_frame_from_mate,
    face_query,
    frame_error,
    frame_from_axes,
    layout_mate,
    mate_parameters,
)
from core.morphology.code_hand import Pose, load

CAD_DIR = Path(__file__).resolve().parents[1] / "scripts" / "cad"
SNAPSHOT = json.loads((CAD_DIR / "cad_snapshot.json").read_text())
_SETTINGS = yaml.safe_load((CAD_DIR / "onshape.yaml").read_text())
CAD = CadMap(_SETTINGS["instances"], _SETTINGS["end_effectors"])
FACE = frame_from_axes([0.0, -14.125, -11.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0])


def test_prototype_reproduces_the_cad_ground_truth():
    parameters, configuration = load()
    errors = compare(parameters, configuration, SNAPSHOT, CAD)
    # 13 servo frames, 10 servo mate connectors, 18 revolute axes (the thumb EE connector is off the centre line)
    # and 9 actuator datums (finger AA and PIP, thumb RO, RAA and MCP)
    assert len(errors) == 50
    worst_mm = max(mm for mm, _ in errors.values())
    worst_deg = max(deg for _, deg in errors.values())
    assert worst_mm < 1e-3 and worst_deg < 1e-2, errors


def test_prototype_layout_and_pose_are_the_ones_fitted_from_the_snapshot():
    parameters, configuration = load()
    assert parameters.layout == layout_from_snapshot(SNAPSHOT, CAD)
    fitted = configuration_from_snapshot(SNAPSHOT, CAD, parameters)
    for digit, joints in configuration.to_dict().items():
        assert joints == pytest.approx(fitted.to_dict()[digit], abs=1e-5)


def test_prototype_actuator_zeros_are_the_cad_datums():
    parameters, configuration = load()
    zeros = actuator_zeros_from_snapshot(SNAPSHOT, CAD, parameters, configuration)
    assert set(zeros["finger"]) == {"aa_actuator_zero_deg", "pip_actuator_zero_deg"}
    assert set(zeros["thumb"]) == {"ro_actuator_zero_deg", "raa_actuator_zero_deg", "mcp_actuator_zero_deg"}
    for group, fields in zeros.items():
        model = getattr(parameters, group)
        for name, values in fields.items():
            # The three fingers' mates agree, so the PIP datum is a property of the shared finger.
            assert values == pytest.approx([getattr(model, name)] * len(values), abs=1e-3), name


def test_turned_servos_differ_from_the_cad_only_by_their_output_side():
    readings = actuator_readings(SNAPSHOT, CAD)
    assert CAD_TURNED_SERVOS == {"thumb/mcp", "thumb/ip"}
    # The thumb MCP's mate reads its angle with the model's sign: flexion (toward the pad) positive.
    _, configuration = load()
    assert configuration.thumb.mcp_deg > 0.0
    assert readings["thumb/mcp"] == pytest.approx(configuration.thumb.mcp_deg - 90.0, abs=1e-3)


def test_prototype_linkage_matches_the_onshape_variable_studio():
    parameters, _ = load()
    for field, value in linkage_from_variables(SNAPSHOT).items():
        assert getattr(parameters.finger, field) == pytest.approx(value)


POSES = [Pose(), Pose(14.125, 0.0, 16.5, -90.0), Pose(1.0, 2.0, 3.0, 10.0, 20.0, 30.0),
         Pose(-40.0, 10.0, -10.0, -60.0, 25.0, -70.0), Pose(5.0, -7.0, 9.0, -120.0, 85.0, 170.0),
         Pose(0.0, 0.0, 0.0, 30.0, 90.0, 0.0)]


@pytest.mark.parametrize("pose", POSES)
def test_mate_parameters_reproduce_the_base_pose(pose):
    base = pose.matrix()
    face = face_frame_from_mate(mate_parameters(base, FACE))
    assert frame_error(base, face @ np.linalg.inv(FACE)) == pytest.approx((0.0, 0.0), abs=1e-6)


def test_mate_composition_matches_the_onshape_measurement():
    # Measured on Onshape (2026-11-08): origin connector t=(10,20,30) mm, Rz 30°; mate Ry 20°; digit Rx 15°.
    measured = np.array([[0.8138, 0.4063, 0.4155], [0.4698, -0.8808, -0.0590], [0.3420, 0.2432, -0.9077]])
    face = face_frame_from_mate(MateParameters((10.0, 20.0, 30.0), 30.0, 20.0, 15.0))
    assert np.abs(face[:3, :3] - measured).max() < 1e-3
    assert face[:3, 3] == pytest.approx([10.0, 20.0, 30.0])


def test_layout_mate_links_its_connectors():
    mate = layout_mate("layout: index", face_query(["A", "B", "C"], "KF+J"), mate_parameters(np.eye(4), FACE))
    ids = [mc["featureId"] for mc in mate["mateConnectors"]]
    by_id = {p["parameterId"]: p for p in mate["parameters"]}
    assert [q["featureId"] for q in by_id["mateConnectorsQuery"]["queries"]] == ids and len(set(ids)) == 2
    assert by_id["mateType"]["value"] == "FASTENED"
    assert mate["mateConnectors"][0]["parameters"][1]["queries"][0]["path"] == ["A", "B", "C"]
