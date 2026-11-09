"""Onshape layout mates: fasten each digit's base servo to the assembly origin at a given pose.

Each digit is fastened to the assembly origin by one mate named ``layout: <digit>``. The mate joins:
- a connector on a face of the digit's base servo (its frame ``K`` in the servo part frame is known);
- a connector on the assembly origin.

A base pose is a 4x4 transform of the servo part frame in the hand frame [mm]; the parametric model
(``core.morphology.code_hand``) supplies it. How a mate realizes a pose was measured on Onshape (see
``tests/test_cad_layout_mates.py``):

    F_face = T(origin translation) Rz(origin rotation) Ry(-mate rotation) Rx(180°) Rx(-digit rotation)

So for a target face frame ``F = P K`` with intrinsic Z-Y-X angles (yaw, pitch, roll):
- origin rotation = yaw;
- mate rotation = -pitch;
- digit rotation = 180° - roll.
"""

from __future__ import annotations

import math
import secrets
from dataclasses import dataclass

import numpy as np

from core.morphology.code_hand.parameters import rx, ry, rz


def _wrap(deg: float) -> float:
    w = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if math.isclose(w, -180.0, abs_tol=1e-9) else w


def frame_from_axes(origin, x_axis, z_axis) -> np.ndarray:
    x, z = np.asarray(x_axis, float), np.asarray(z_axis, float)
    x, z = x / np.linalg.norm(x), z / np.linalg.norm(z)
    m = np.eye(4)
    m[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    m[:3, 3] = origin
    return m


def zyx_angles(rotation: np.ndarray) -> tuple[float, float, float]:
    """Intrinsic Z-Y-X angles (yaw, pitch, roll) in degrees. Gimbal lock at pitch ±90° puts it all in yaw."""
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -rotation[2, 0]))))
    if abs(rotation[2, 0]) < 1.0 - 1e-9:
        yaw = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
        roll = math.degrees(math.atan2(rotation[2, 1], rotation[2, 2]))
    else:
        yaw = math.degrees(math.atan2(-rotation[0, 1], rotation[1, 1]))
        roll = 0.0
    return _wrap(yaw), pitch, _wrap(roll)


def frame_error(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """Translation [mm] and rotation [deg] between two 4x4 frames."""
    rel = a[:3, :3].T @ b[:3, :3]
    axis = np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]])
    angle = math.degrees(math.atan2(np.linalg.norm(axis) / 2.0, (np.trace(rel) - 1.0) / 2.0))  # stable near 0
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), angle


@dataclass(frozen=True)
class MateParameters:
    translation_mm: tuple[float, float, float]
    origin_rotation_deg: float  # origin connector, about its Z
    mate_rotation_deg: float  # mate offset, about Y
    digit_rotation_deg: float  # digit connector, about its X


def mate_parameters(base: np.ndarray, face_in_servo: np.ndarray) -> MateParameters:
    face = base @ face_in_servo
    yaw, pitch, roll = zyx_angles(face[:3, :3])
    return MateParameters(tuple(float(v) for v in face[:3, 3]), yaw, _wrap(-pitch), _wrap(180.0 - roll))


def face_frame_from_mate(params: MateParameters) -> np.ndarray:
    """The face frame the mate produces (the measured Onshape composition)."""
    m = rz(params.origin_rotation_deg) @ ry(-params.mate_rotation_deg) @ rx(180.0) @ rx(-params.digit_rotation_deg)
    m[:3, 3] = params.translation_mm
    return m


# Onshape feature JSON
def _new_id() -> str:
    return "M" + secrets.token_urlsafe(12)[:16]


def _enum(pid: str, value: str, enum_name: str) -> dict:
    return {"btType": "BTMParameterEnum-145", "parameterId": pid, "value": value, "enumName": enum_name,
            "namespace": ""}


def _bool(pid: str, value: bool) -> dict:
    return {"btType": "BTMParameterBoolean-144", "parameterId": pid, "value": value}


def _qty(pid: str, expression: str) -> dict:
    return {"btType": "BTMParameterQuantity-147", "parameterId": pid, "expression": expression, "isInteger": False}


def _queries(pid: str, queries: list[dict]) -> dict:
    return {"btType": "BTMParameterQueryWithOccurrenceList-67", "parameterId": pid, "queries": queries}


def _fmt(value: float, unit: str) -> str:
    return f"{value:.6f} {unit}"


def _connector(origin_query: dict, translation_mm=(0.0, 0.0, 0.0), rotation_type="ABOUT_Z",
               rotation_deg=0.0) -> dict:
    return {
        "btType": "BTMMateConnector-66",
        "featureType": "mateConnector",
        "featureId": _new_id(),
        "name": "Mate connector",
        "isHidden": True,
        "isAuxiliaryTreeMateConnector": False,
        "parameters": [
            _enum("originType", "ON_ENTITY", "Origin type"),
            _queries("originQuery", [origin_query]),
            _bool("transform", True),
            *(_qty(f"translation{a}", _fmt(v, "mm")) for a, v in zip("XYZ", translation_mm)),
            _enum("rotationType", rotation_type, "Rotation axis"),
            _qty("rotation", _fmt(rotation_deg, "deg")),
            _bool("flipPrimary", False),
            _enum("secondaryAxisType", "PLUS_X", "Reorient secondary axis"),
        ],
    }


def face_query(occurrence_path: list[str], deterministic_id: str) -> dict:
    """Centroid of a face (by deterministic id) on the part at ``occurrence_path``."""
    return {"btType": "BTMInferenceQueryWithOccurrence-1083", "inferenceType": "CENTROID",
            "deterministicIds": [deterministic_id], "path": occurrence_path}


ORIGIN_QUERY = {"btType": "BTMFeatureQueryWithOccurrence-157", "featureId": "Origin", "queryData": "ORIGIN_Z",
                "path": []}


def layout_mate(name: str, digit_face_query: dict, params: MateParameters) -> dict:
    """A new Fastened mate from the digit face to the assembly origin, with inline mate connectors."""
    digit = _connector(digit_face_query, rotation_type="ABOUT_X", rotation_deg=params.digit_rotation_deg)
    origin = _connector(ORIGIN_QUERY, params.translation_mm, "ABOUT_Z", params.origin_rotation_deg)
    return {
        "btType": "BTMMate-64",
        "featureType": "mate",
        "name": name,
        "suppressed": False,
        "mateConnectors": [digit, origin],
        "parameters": [
            _enum("mateType", "FASTENED", "Mate type"),
            _queries("mateConnectorsQuery", [
                {"btType": "BTMFeatureQueryWithOccurrence-157", "featureId": digit["featureId"], "queryData": "",
                 "path": []},
                {"btType": "BTMFeatureQueryWithOccurrence-157", "featureId": origin["featureId"], "queryData": "",
                 "path": []},
            ]),
            _bool("mateOffsetEnabled", True),
            *(_qty(f"mateOffsetDistance{a}", "0 mm") for a in "XYZ"),
            _enum("rotationType", "ABOUT_Y", "Rotation axis"),
            _qty("rotation", _fmt(params.mate_rotation_deg, "deg")),
            _bool("primaryAxisAlignment", True),
            _enum("secondaryAxisAlignment", "PLUS_X", "Reorient secondary axis"),
        ],
    }
