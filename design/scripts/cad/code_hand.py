#!/usr/bin/env python3
"""Keep the Onshape CAD in step with the CoDe parametric model (core/morphology/code_hand).

    python scripts/cad/code_hand.py snapshot [--ground-truth]   # CAD -> cad_snapshot.json
    python scripts/cad/code_hand.py fit                         # snapshot -> prototype.yaml layout + CAD pose
    python scripts/cad/code_hand.py check                       # live CAD vs the model
    python scripts/cad/code_hand.py push [--dry-run]            # model -> CAD (layout mates, linkage studio)
    python scripts/cad/code_hand.py view                        # Onshape shaded views -> output/cad/

The parameter file (``core/morphology/code_hand/prototype.yaml``) is the source of truth; ``push`` makes the
CAD follow it. ``fit`` exists to build the model from the CAD: it takes the layout and the joint angles off the
snapshot, and reports how far the model's fixed morphology is from the CAD.

``check`` fits the joint angles of the live CAD (the CAD may sit in any pose), then compares every servo
frame and revolute axis with the model. ``push`` deletes the mates under ``remove`` and replaces the layout
mates; make an Onshape version first. Settings: ``onshape.yaml``. Credentials: ``ONSHAPE_ACCESS_KEY``,
``ONSHAPE_SECRET_KEY``.
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.cad.code_hand import (
    LINKAGE_VARIABLES,
    CadMap,
    compare,
    configuration_from_snapshot,
    layout_from_snapshot,
    linkage_from_variables,
)
from core.cad.layout_mates import (
    face_query,
    frame_from_axes,
    layout_mate,
    mate_parameters,
)
from core.cad.onshape import ElementRef, OnshapeClient
from core.cad.snapshot import extract_snapshot, length_mm
from core.morphology.code_hand import (
    DIGITS,
    CoDeHand,
    HandParameters,
    load,
)
from core.morphology.code_hand.parameters import PROTOTYPE

SETTINGS = HERE / "onshape.yaml"
SNAPSHOT = HERE / "cad_snapshot.json"
MATE_PREFIX = "layout: "
TOL_MM, TOL_DEG = 0.01, 0.01


def report(errors: dict[str, tuple[float, float]]) -> bool:
    bad = {k: v for k, v in errors.items() if v[0] > TOL_MM or v[1] > TOL_DEG}
    worst_mm = max(errors.items(), key=lambda kv: kv[1][0])
    worst_deg = max(errors.items(), key=lambda kv: kv[1][1])
    print(f"{len(errors)} checks (servo frames and connectors, end effectors, revolute axes); "
          f"tolerance {TOL_MM} mm, {TOL_DEG} deg")
    print(f"  worst position {worst_mm[1][0]:.5f} mm ({worst_mm[0]}), worst angle {worst_deg[1][1]:.5f} deg "
          f"({worst_deg[0]})")
    for key, (mm, deg) in bad.items():
        print(f"  MISMATCH {key}: {mm:.4f} mm, {deg:.4f} deg")
    return not bad


def cmd_snapshot(client, ref) -> int:
    snapshot = extract_snapshot(client, ref)
    SNAPSHOT.write_text(json.dumps(snapshot, indent=1) + "\n")
    print(f"{SNAPSHOT.name}: {len(snapshot['servos'])} servos, {len(snapshot['revolutes'])} revolutes, "
          f"workspace {ref.wid}, microversion {snapshot['source']['microversion']}")
    return 0


def cmd_fit(cad: CadMap, thumb_zero: bool = False) -> int:
    snapshot = json.loads(SNAPSHOT.read_text())
    data = yaml.safe_load(PROTOTYPE.read_text())
    data["layout"] = HandParameters(layout_from_snapshot(snapshot, cad)).to_dict()["layout"]
    if thumb_zero:  # the CAD's thumb pose becomes q = 0
        for joint in ("ro", "raa", "mcp"):
            data["thumb"][f"{joint}_zero_deg"] = 0.0
        raw = configuration_from_snapshot(snapshot, cad, HandParameters.from_dict(data)).thumb
        for joint in ("ro", "raa", "mcp"):
            data["thumb"][f"{joint}_zero_deg"] = getattr(raw, f"{joint}_deg")
        print("thumb zero pose from the CAD:", {j: data["thumb"][f"{j}_zero_deg"] for j in ("ro", "raa", "mcp")})
    parameters = HandParameters.from_dict(data)
    configuration = configuration_from_snapshot(snapshot, cad, parameters)
    data["cad_configuration"] = configuration.to_dict()
    source = snapshot["source"]
    data["source"] = {"document": source["document"], "workspace": source["workspace"],
                      "microversion": source["microversion"]}
    text = PROTOTYPE.read_text()
    header = "".join(line for line in text.splitlines(keepends=True)[:20] if line.startswith("#"))
    PROTOTYPE.write_text(header + yaml.safe_dump(data, sort_keys=False))
    print(f"wrote layout and cad_configuration to {PROTOTYPE.relative_to(ROOT)}")

    studio = linkage_from_variables(snapshot)
    for field, value in studio.items():
        model = getattr(parameters.finger, field)
        if abs(model - value) > 1e-9:
            print(f"  linkage studio {field} = {value} differs from the model's {model}")
    tilt = length_mm(snapshot["variables"].get("Linkage Dimensions", {}).get("pip_servo_tilt", ""))
    if tilt is not None and abs((90.0 - parameters.finger.pip_servo_angle_deg) - tilt) > 1e-9:
        print(f"  note: Onshape pip_servo_tilt = {tilt} deg, but the assembled PIP servo sits "
              f"{90.0 - parameters.finger.pip_servo_angle_deg} deg from vertical (pip_servo_angle_deg = "
              f"{parameters.finger.pip_servo_angle_deg})")
    return 0 if report(compare(parameters, configuration, snapshot, cad)) else 1


def cmd_check(client, ref, cad: CadMap, params_file: Path = PROTOTYPE) -> int:
    snapshot = extract_snapshot(client, ref)
    parameters, _ = load(params_file)
    configuration = configuration_from_snapshot(snapshot, cad, parameters)
    print("CAD pose:", json.dumps(configuration.to_dict()))
    return 0 if report(compare(parameters, configuration, snapshot, cad)) else 1


def _write_linkage(client, settings, ref, parameters, dry_run) -> None:
    studios = {e["name"]: e["id"] for e in client.elements(ref.did, ref.wid) if e["elementType"] == "VARIABLESTUDIO"}
    studio = ref.sibling(studios[settings["linkage_studio"]])
    current = client.variables(studio)
    changed = False
    for v in current:
        if v["name"] in LINKAGE_VARIABLES:
            target = getattr(parameters.finger, LINKAGE_VARIABLES[v["name"]])
            if length_mm(v["expression"]) is None and target == 0.0:
                continue  # a blank expression already reads as 0
            if length_mm(v["expression"]) != target:
                print(f"linkage: {v['name']} {v['expression']!r} -> {target:g} mm")
                v["expression"] = f"{target:g} mm"
                changed = True
    if changed and not dry_run:
        # Onshape takes the whole studio back and rejects blank expressions: fill managed blanks with the model.
        for v in current:
            if not (v["expression"] or "").strip():
                if v["name"] not in LINKAGE_VARIABLES:
                    raise SystemExit(f"linkage studio variable {v['name']} is blank; set it in Onshape first")
                v["expression"] = f"{getattr(parameters.finger, LINKAGE_VARIABLES[v['name']]):g} mm"
                print(f"linkage: {v['name']} '' -> {v['expression']}")
        client.set_variables(studio, [{k: v[k] for k in ("name", "type", "expression", "description")}
                                      for v in current])
    if not changed:
        print("linkage: studio already matches")


def cmd_push(client, settings, ref, cad: CadMap, dry_run: bool, params_file: Path = PROTOTYPE) -> int:
    parameters, _ = load(params_file)
    _write_linkage(client, settings, ref, parameters, dry_run)

    definition = client.assembly_definition(ref)
    by_name = {i["name"]: i["id"] for i in definition["rootAssembly"]["instances"]}
    face = frame_from_axes(*(settings["servo_face"][k] for k in ("origin_mm", "x_axis", "z_axis")))
    face_id = settings["servo_face"]["deterministic_id"]
    features = {f["name"]: f for f in client.assembly_features(ref)["features"]}

    # Onshape breaks a mate whose inline connectors go through an API update (even one that only suppresses
    # it), so hand-built mates are deleted and layout mates are replaced, never updated.
    for name in settings.get("remove", []):
        if name in features:
            print(f"remove: {name}")
            if not dry_run:
                client.delete_assembly_feature(ref, features[name]["featureId"])

    servos = CoDeHand(parameters).frames().servos
    for digit in DIGITS:
        base = servos["thumb/ro" if digit == "thumb" else f"{digit}/aa"]
        params = mate_parameters(base, face)
        path = [by_name[cad.instances[digit]], *settings["base_servo_paths"][digit]]
        mate = layout_mate(MATE_PREFIX + digit, face_query(path, face_id), params)
        existing = features.get(MATE_PREFIX + digit)
        t = ", ".join(f"{v:.3f}" for v in params.translation_mm)
        print(f"{'replace' if existing else 'add'}: {MATE_PREFIX}{digit}  t=({t}) mm")
        if dry_run:
            continue
        if existing:
            client.delete_assembly_feature(ref, existing["featureId"])
        state = client.add_assembly_feature(ref, mate).get("featureState", {})
        if state.get("featureStatus") != "OK":
            raise SystemExit(f"{digit}: Onshape reported {state}")
    return 0 if dry_run else cmd_check(client, ref, cad, params_file)


def cmd_view(client, ref, views, out_dir: Path, size) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    for view in views:
        result = client.get(f"assemblies/{ref.path()}/shadedviews", viewMatrix=view, outputWidth=size[0],
                            outputHeight=size[1], pixelSize=0, useAntiAliasing="true", edges="show")
        path = out_dir / f"code_hand_{view}.png"
        path.write_bytes(base64.b64decode(result["images"][0]))
        print(path)
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("snapshot", "fit", "check", "push", "view"))
    parser.add_argument("--workspace", help="Onshape workspace id (default: onshape.yaml workspace)")
    parser.add_argument("--ground-truth", action="store_true", help="use onshape.yaml ground_truth_workspace")
    parser.add_argument("--dry-run", action="store_true", help="push: print the plan only")
    parser.add_argument("--thumb-zero", action="store_true", help="fit: take the CAD thumb pose as its zero")
    parser.add_argument("--params", type=Path, default=PROTOTYPE, help="check/push: parameter file")
    parser.add_argument("--views", nargs="+", default=["isometric", "top", "front", "right"])
    parser.add_argument("--out", type=Path, default=ROOT / "output" / "cad")
    parser.add_argument("--size", type=int, nargs=2, default=(1200, 900), metavar=("W", "H"))
    args = parser.parse_args(argv)

    settings = yaml.safe_load(SETTINGS.read_text())
    cad = CadMap(settings["instances"], settings.get("end_effectors", {}))
    if args.command == "fit":
        return cmd_fit(cad, args.thumb_zero)
    workspace = args.workspace or settings["ground_truth_workspace" if args.ground_truth else "workspace"]
    ref = ElementRef(settings["document"], workspace, settings["assembly"])
    client = OnshapeClient()
    if args.command == "snapshot":
        return cmd_snapshot(client, ref)
    if args.command == "check":
        return cmd_check(client, ref, cad, args.params)
    if args.command == "push":
        return cmd_push(client, settings, ref, cad, args.dry_run, args.params)
    return cmd_view(client, ref, args.views, args.out, tuple(args.size))


if __name__ == "__main__":
    raise SystemExit(main())
