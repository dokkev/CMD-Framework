"""Extract an observed assembly snapshot of the hand from Onshape for diagnostic comparison.

The CAD snapshot is **not** a source of truth (``core.morphology.code_hand`` is). It records the observed CAD state
for verification (``check``) or one-off parameter fitting (``fit``):
- ``servos``: the frame of every servo part (name prefix ``X330``) in the hand frame;
- ``revolutes``: every revolute mate's axis (origin and direction) in the hand frame, with the two parts it joins
  and its current angle ``rotation_deg`` (Onshape's ``rotationZ``: the first part's rotation relative to the
  second about ``axis``; 0 is the CAD's own datum, where the two mate connectors align);
- ``connectors``: every named mate connector's frame in the hand frame (e.g. ``(root):Thumb EE``);
- ``variables``: the expressions of every Variable Studio in the document.

Keys are readable occurrence paths, e.g. ``finger <2>/MCP-FE <1>/X330 <3>``. Lengths are millimetres. The
snapshot records the document microversion it was read from.
"""

from __future__ import annotations

import re

import numpy as np

from core.cad.layout_mates import frame_from_axes
from core.cad.onshape import ElementRef, OnshapeClient

SERVO_PREFIX = "X330"


def _mm(transform: list[float]) -> np.ndarray:
    m = np.array(transform, float).reshape(4, 4)
    m[:3, 3] *= 1000.0
    return m


def _frame(m: np.ndarray) -> dict:
    return {"origin_mm": m[:3, 3].tolist(), "x_axis": m[:3, 0].tolist(), "z_axis": m[:3, 2].tolist()}


class _Names:
    def __init__(self, definition: dict):
        self.names = {i["id"]: i["name"] for i in definition["rootAssembly"]["instances"]}
        for sub in definition.get("subAssemblies", []):
            self.names.update({i["id"]: i["name"] for i in sub["instances"]})

    def __call__(self, path) -> str:
        return "/".join(self.names.get(p, p) for p in path)


def _revolute(feature: dict, prefix: list[str], occurrences: dict, name) -> dict:
    data = feature["featureData"]
    first, second = data["matedEntities"]
    m = occurrences[tuple(prefix + first["matedOccurrence"])]
    cs = first["matedCS"]
    origin = m[:3, :3] @ (np.array(cs["origin"]) * 1000.0) + m[:3, 3]
    axis = m[:3, :3] @ np.array(cs["zAxis"])
    return {
        "origin_mm": origin.tolist(),
        "axis": axis.tolist(),
        "parts": [name(prefix + first["matedOccurrence"]), name(prefix + second["matedOccurrence"])],
    }


def extract_snapshot(client: OnshapeClient, assembly: ElementRef) -> dict:
    definition = client.assembly_definition(assembly)
    name = _Names(definition)
    root = definition["rootAssembly"]
    occurrences = {tuple(o["path"]): _mm(o["transform"]) for o in root["occurrences"]}
    subassemblies = {s["elementId"]: s for s in definition.get("subAssemblies", [])}

    servos = {name(path): _frame(m) for path, m in occurrences.items()
              if name([path[-1]]).startswith(SERVO_PREFIX)}

    revolutes: dict[str, dict] = {}
    connectors: dict[str, dict] = {}

    def collect(features: list[dict], prefix: list[str]) -> None:
        for f in features:
            if f["featureType"] == "mate" and f["featureData"]["mateType"] == "REVOLUTE" \
                    and not f["featureData"].get("suppressed"):
                key = f"{name(prefix) or '(root)'}:{f['featureData']['name']}"
                revolutes[key] = _revolute(f, prefix, occurrences, name)
            elif f["featureType"] == "mateConnector":
                data = f["featureData"]
                cs = data["mateConnectorCS"]
                local = frame_from_axes(np.array(cs["origin"]) * 1000.0, cs["xAxis"], cs["zAxis"])
                key = f"{name(prefix) or '(root)'}:{data['name']}"
                connectors[key] = {**_frame(occurrences[tuple(prefix + data["occurrence"])] @ local),
                                   "part": name(prefix + data["occurrence"])}

    def walk(instances: list[dict], prefix: list[str]) -> None:
        for inst in instances:
            if inst["type"] == "Assembly" and not inst.get("suppressed"):
                sub = subassemblies[inst["elementId"]]
                collect(sub.get("features", []), prefix + [inst["id"]])
                walk(sub["instances"], prefix + [inst["id"]])

    collect(root.get("features", []), [])
    walk(root["instances"], [])
    for value in client.mate_values(assembly):
        key = f"{name(value['ownerOccurrencePath']) or '(root)'}:{value['mateName']}"
        if key in revolutes and "rotationZ" in value:
            revolutes[key]["rotation_deg"] = float(np.degrees(value["rotationZ"]))

    variables = {}
    for element in client.elements(assembly.did, assembly.wid):
        if element["elementType"] == "VARIABLESTUDIO":
            studio = assembly.sibling(element["id"])
            variables[element["name"]] = {v["name"]: v["expression"] for v in client.variables(studio)}

    microversion = client.assembly_features(assembly).get("sourceMicroversion")
    return {
        "source": {"document": assembly.did, "workspace": assembly.wid, "element": assembly.eid,
                   "microversion": microversion},
        "units": "mm",
        "servos": dict(sorted(servos.items())),
        "revolutes": dict(sorted(revolutes.items())),
        "connectors": dict(sorted(connectors.items())),
        "variables": variables,
    }


def length_mm(expression: str) -> float | None:
    """Parse a simple Onshape length/angle expression such as ``35.7 mm`` or ``10 deg``; None if blank."""
    expression = (expression or "").strip()
    if not expression:
        return None
    match = re.fullmatch(r"(-?[\d.]+)\s*\*?\s*(mm|deg|m|in)?", expression)
    if not match:
        raise ValueError(f"unsupported expression {expression!r}")
    value, unit = float(match.group(1)), match.group(2)
    return value * {"m": 1000.0, "in": 25.4}.get(unit, 1.0)
