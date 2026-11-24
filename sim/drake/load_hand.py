"""Load the CoDe hand into Drake from its plain URDF and close the four-bars from the hand description.

The description JSON (``design/output/sim/...``) is the single source: ``sim/isaaclab`` writes the USD (loops closed
inside it) and a plain URDF (spanning tree, collision, coloured visuals, transmissions; ``write_urdf(geometry=True)``,
``scripts/code_hand/make_assets.py --geometry --thumb-pad --no-usd``). Here the URDF is parsed into a MultibodyPlant
and the description's ``loops`` close each finger's four-bar with a ball constraint at its cut pin B' (SAP), the
``collision.filtered_pairs`` are applied, the actuators (from the URDF transmissions) get PD gains (the PIP servo's set
in flexion: K_q / (dtheta/dq)^2 and D_q / (dtheta/dq)^2 at the held flexion, through the hand's own ``four_bar``
table; the other servos the XM335 position loop's stiffness and back-EMF damping), and the hand starts closed at the
tables' joint angles.

    uv run python load_hand.py --hand O --check [--render 0 60 120] [--view side|side_driver|overview] [--contact point|hydroelastic]

``--check``: for O, R, U, (1) kinematics: the tree joints set from the ``four_bar`` table over q = 0..120 deg, the
two halves of each cut pin apart [mm] and the distal segment's angle to the proximal against q [deg]; (2) a held
pose: the PIP servos commanded to theta(q), 3 s simulated with gravity (palm up), the pin separation and the flexion
reached. ``--render``: Drake VTK images of the hand (palm up, at MCP 0 and these PIP angles) to ``logs/load_hand/``;
``--view side`` looks at the ring finger from its outer side (the coupler's) with ``render3d.py``'s camera (the look
of ``logs/hands_ORU_3d.png``), ``side_driver`` mirrored at the index finger from the other side (the driver's),
``overview`` from the Isaac Lab video camera. ``--hand-view [palm_up palm_down atlas]``: whole-hand views
(``HAND_VIEWS``) to ``logs/load_hand/drake_<hand>_<view>.png``.

The URDF is the one with the drawing-only geometry when it exists (``make_assets.py ... --visuals``:
``*_thumbpad_vis.urdf``), otherwise the plain ``*_thumbpad.urdf``; the visuals carry no mass or collision, so the
plant is the same.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
HAND_FILES = {"O": "h0", "R": "hr", "U": "hu"}
OUT = HERE / "logs" / "load_hand"
K_Q, D_Q = 5.0, 0.08
"""PIP impedance in flexion [N m/rad, N m s/rad] (the Study 3 two-pad measurement's and Isaac Lab's values)."""
HAND_POSE = (math.pi, 0.0, 0.0, 0.0, 0.0, 0.5)
"""Palm up, as in Isaac Lab: half a turn about x, the hand root 0.5 m up."""


SIDE_CAMERA = {"target_mm": (40.0, -12.0), "dist": 0.24, "az_deg": -20.0, "el_deg": 14.0, "fov_deg": 30.0}
"""``render_hands.py``'s camera for ``hands_ORU_3d.png`` (``render3d.render``), in a finger's linkage plane (x from
the FE axis along the proximal, z dorsal, the camera on the -y side)."""


def paths(hand: str, visuals: bool = True) -> tuple[Path, Path]:
    stem = f"cfl_q120_h3to7_{HAND_FILES[hand]}"
    urdf_dir = REPO / "sim" / "isaaclab" / "logs" / "urdf"
    urdf = urdf_dir / f"{stem}_thumbpad_vis.urdf"
    if not (visuals and urdf.exists()):
        urdf = urdf_dir / f"{stem}_thumbpad.urdf"
    return urdf, REPO / "design" / "output" / "sim" / "study3" / "hands" / f"{stem}.json"


def side_camera(desc: dict, digit: str = "ring", side: float = -1.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``SIDE_CAMERA`` for ``digit`` in the hand frame, from its ``-y`` side (``side=+1``: from ``+y``, mirrored):
    eye, target and the image's up direction (dorsal)."""
    joints = {j["name"]: j for j in desc["joints"]}
    o = np.asarray(joints[f"{digit}_fe"]["anchor"], float)
    x = np.asarray(joints[f"{digit}_a"]["anchor"], float) - o
    x /= np.linalg.norm(x)
    y = -side * np.asarray(joints[f"{digit}_fe"]["axis"], float)
    z = np.cross(x, -side * y)  # dorsal (the pads lie at -z)
    c = SIDE_CAMERA
    target = o + 1e-3 * (c["target_mm"][0] * x + c["target_mm"][1] * z)
    az, el = math.radians(c["az_deg"]), math.radians(c["el_deg"])
    eye = target + c["dist"] * (math.sin(az) * math.cos(el) * x - math.cos(az) * math.cos(el) * y + math.sin(el) * z)
    return eye, target, z


def table(desc: dict, digit: str, q_deg: float) -> dict[str, float]:
    """The four-bar's tree joint angles [rad] at flexion ``q_deg`` (the description's table)."""
    t = desc["four_bar"][digit]
    return {f"{digit}_{k}": math.radians(float(np.interp(q_deg, t["pip_deg"], t[f"{k2}_deg"])))
            for k, k2 in (("a", "a"), ("pip_servo", "ap"), ("b", "b"))}


def ratio(desc: dict, digit: str, q_deg: float) -> float:
    """dtheta/dq of the PIP servo at ``q_deg``."""
    t = desc["four_bar"][digit]
    return float(np.interp(q_deg, t["pip_deg"], np.gradient(t["ap_deg"], t["pip_deg"])))


def load_hand(urdf: Path, description: Path, *, time_step: float = 1e-3, contact: str = "point", pip_deg: float = 0.0,
              gravity: bool = True, renderer: bool = False, pose=HAND_POSE):
    """Build the plant. Returns ``(diagram, plant, scene_graph, model, desc, pins)``; ``pins`` lists each loop's
    ``(parent body, p_parent, child body, p_child)`` (the cut pin in both bodies' frames)."""
    from pydrake.geometry import CollisionFilterDeclaration, GeometrySet
    from pydrake.math import RigidTransform, RollPitchYaw
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
    from pydrake.multibody.tree import PdControllerGains
    from pydrake.systems.framework import DiagramBuilder

    desc = json.loads(Path(description).read_text())
    builder = DiagramBuilder()
    config = MultibodyPlantConfig(time_step=time_step, discrete_contact_approximation="sap",
                                  contact_model={"point": "point", "hydroelastic": "hydroelastic_with_fallback"}[contact])
    plant, sg = AddMultibodyPlant(config, builder)
    if renderer:
        from pydrake.geometry import LightParameter, MakeRenderEngineVtk, RenderEngineVtkParams

        params = RenderEngineVtkParams()
        params.default_clear_color = np.array([1.0, 1.0, 1.0])
        params.lights = [  # render3d's lights, in the camera frame (they follow the view)
            LightParameter(type="directional", direction=[0.35, 0.55, 0.75], intensity=0.80, frame="camera"),
            LightParameter(type="directional", direction=[-0.6, 0.3, 0.2], intensity=0.30, frame="camera"),
            LightParameter(type="directional", direction=[0.0, 0.0, 1.0], intensity=0.40, frame="camera"),
        ]
        params.cast_shadows = True
        params.shadow_map_size = 4096
        sg.AddRenderer("vtk", MakeRenderEngineVtk(params))
    model = Parser(plant).AddModels(str(urdf))[0]
    rpy, xyz = pose[:3], pose[3:]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("palm", model), RigidTransform(RollPitchYaw(*rpy), xyz))
    if not gravity:
        plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, 0.0])
    # link frames sit at the bodies' origins (the URDF puts each link at its parent joint's anchor)
    origin = {b["name"]: np.asarray(b["origin"], float) for b in desc["bodies"]}
    pins = []
    for cut in desc["loops"]:
        a, b = plant.GetBodyByName(cut["parent"], model), plant.GetBodyByName(cut["child"], model)
        anchor = np.asarray(cut["anchor"], float)
        p_a, p_b = anchor - origin[cut["parent"]], anchor - origin[cut["child"]]
        plant.AddBallConstraint(a, p_a, b, p_b)
        pins.append((a, p_a, b, p_b))
    manager = sg.collision_filter_manager()
    for x, y in desc["collision"]["filtered_pairs"]:
        gx = plant.GetCollisionGeometriesForBody(plant.GetBodyByName(x, model))
        gy = plant.GetCollisionGeometriesForBody(plant.GetBodyByName(y, model))
        if gx and gy:
            manager.Apply(CollisionFilterDeclaration().ExcludeBetween(GeometrySet(gx), GeometrySet(gy)))
    servo = desc["servo"]["derived"]
    from pydrake.multibody.tree import JointActuatorIndex

    for k in range(plant.num_actuators()):  # (the per-model index list is filled only by Finalize)
        actuator = plant.get_joint_actuator(JointActuatorIndex(k))
        name = actuator.joint().name()
        if name.endswith("_pip_servo"):
            r = ratio(desc, name.split("_")[0], pip_deg)
            gains = PdControllerGains(p=K_Q / r**2, d=D_Q / r**2)
        else:
            gains = PdControllerGains(p=servo["stiffness_nm_per_rad"], d=servo["back_emf_damping_nms_per_rad"])
        actuator.set_controller_gains(gains)
    plant.Finalize()
    diagram = builder.Build()
    return diagram, plant, sg, model, desc, pins


def set_pose(plant, plant_ctx, model, desc, pip_deg: float, mcp_deg: float = 0.0, thumb: dict | None = None):
    """Every joint at the pose: the four-bars closed at ``pip_deg`` from the tables, the MCPs at ``mcp_deg``."""
    for digit in desc["four_bar"]:
        for name, value in table(desc, digit, pip_deg).items():
            plant.GetJointByName(name, model).set_angle(plant_ctx, value)
        plant.GetJointByName(f"{digit}_fe", model).set_angle(plant_ctx, math.radians(mcp_deg))
    for name, deg in (thumb or {}).items():
        plant.GetJointByName(name, model).set_angle(plant_ctx, math.radians(deg))


def hold_inputs(plant, plant_ctx, model):
    """Fix the PD-controlled actuators' desired state at the current pose, no feedforward."""
    desired = [plant.get_joint_actuator(index).joint().get_angle(plant_ctx)
               for index in plant.GetJointActuatorIndices(model)]
    n = len(desired)
    plant.get_desired_state_input_port(model).FixValue(plant_ctx, np.concatenate([desired, np.zeros(n)]))
    plant.get_actuation_input_port(model).FixValue(plant_ctx, np.zeros(n))


def pin_gap_mm(plant, plant_ctx, pins) -> float:
    gaps = []
    for a, p_a, b, p_b in pins:
        wa = plant.EvalBodyPoseInWorld(plant_ctx, a).multiply(p_a)
        wb = plant.EvalBodyPoseInWorld(plant_ctx, b).multiply(p_b)
        gaps.append(np.linalg.norm(wa - wb))
    return 1e3 * float(max(gaps))


def flexion_deg(plant, plant_ctx, model, digit: str) -> float:
    """The distal segment's angle to the proximal about the PIP axis [deg] (from the bodies' poses)."""
    rp = plant.EvalBodyPoseInWorld(plant_ctx, plant.GetBodyByName(f"{digit}_proximal", model)).rotation()
    rd = plant.EvalBodyPoseInWorld(plant_ctx, plant.GetBodyByName(f"{digit}_distal", model)).rotation()
    rel = rp.inverse().multiply(rd).matrix()
    return math.degrees(math.atan2(rel[0, 2], rel[0, 0]))  # rotation about the body y (the flexion axes)


def check(hand: str, contact: str) -> dict:
    from pydrake.systems.analysis import Simulator

    urdf, description = paths(hand)
    result = {"hand": hand, "kinematics": {}, "held": {}}
    diagram, plant, sg, model, desc, pins = load_hand(urdf, description, contact=contact)
    ctx = diagram.CreateDefaultContext()
    pctx = plant.GetMyMutableContextFromRoot(ctx)
    gap, ang = 0.0, 0.0
    for q in np.arange(0.0, 120.0 + 1e-9, 5.0):
        set_pose(plant, pctx, model, desc, q)
        gap = max(gap, pin_gap_mm(plant, pctx, pins))
        ang = max(ang, max(abs(flexion_deg(plant, pctx, model, d) - q) for d in desc["four_bar"]))
    result["kinematics"] = {"pin_gap_mm_max": gap, "flexion_error_deg_max": ang, "q_deg": [0, 120, 5]}
    for q in (0.0, 30.0, 60.0, 90.0, 120.0):
        diagram, plant, sg, model, desc, pins = load_hand(urdf, description, contact=contact, pip_deg=q)
        ctx = diagram.CreateDefaultContext()
        pctx = plant.GetMyMutableContextFromRoot(ctx)
        set_pose(plant, pctx, model, desc, q)
        hold_inputs(plant, pctx, model)
        sim = Simulator(diagram, ctx)
        gap_max = 0.0
        for t in np.arange(0.1, 3.0 + 1e-9, 0.1):
            sim.AdvanceTo(t)
            gap_max = max(gap_max, pin_gap_mm(plant, pctx, pins))
        flex = {d: flexion_deg(plant, pctx, model, d) for d in desc["four_bar"]}
        result["held"][f"{q:g}"] = {"pin_gap_mm_max": gap_max, "pin_gap_mm_end": pin_gap_mm(plant, pctx, pins),
                                    "flexion_error_deg": max(abs(v - q) for v in flex.values())}
    return result


HAND_VIEWS = {
    "palm_up": {"pose": HAND_POSE, "dir": (0.55, -0.55, 0.63), "joints": {}},
    "palm_down": {"pose": (0.0, 0.0, 0.0, 0.0, 0.0, 0.5), "dir": (0.55, -0.55, 0.63), "joints": {}},
    "atlas": {"pose": (0.0, 0.0, 0.0, 0.0, 0.0, 0.5), "dir": (-0.62, -0.5, 0.6),
              "joints": {"fe": 25.0, "pip": 35.0, "thumb_mcp": 20.0, "thumb_ip": 20.0}},
}
"""Whole-hand views (the camera in the world, z up): ``palm_up`` and ``palm_down`` three-quarter views from the
fingers' front right, ``atlas`` from the dorsal back left with the fingers slightly flexed (the Atlas hand photo).
``sim/isaaclab/scripts/code_hand/render_visuals.py`` uses the same."""
HAND_VIEW_TARGET, HAND_VIEW_DIST, HAND_VIEW_FOV = (0.02, 0.01, 0.0), 0.44, 35.0
"""The point looked at (hand frame) [m], the camera's distance [m] and field of view [deg]."""


def render_hand_view(hand: str, view: str, size=(1000, 800)) -> Path:
    """A ``HAND_VIEWS`` image of the whole hand."""
    from PIL import Image
    from pydrake.geometry import ClippingRange, ColorRenderCamera, RenderCameraCore
    from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
    from pydrake.systems.sensors import CameraInfo

    spec = HAND_VIEWS[view]
    urdf, description = paths(hand)
    diagram, plant, sg, model, desc, _ = load_hand(urdf, description, renderer=True, pose=spec["pose"])
    ctx = diagram.CreateDefaultContext()
    pctx, sctx = plant.GetMyMutableContextFromRoot(ctx), sg.GetMyContextFromRoot(ctx)
    j = spec["joints"]
    set_pose(plant, pctx, model, desc, j.get("pip", 0.0), j.get("fe", 0.0),
             {k: v for k, v in j.items() if k.startswith("thumb_")})
    rot = RollPitchYaw(*spec["pose"][:3]).ToRotationMatrix().matrix()
    target = rot @ np.asarray(HAND_VIEW_TARGET) + np.asarray(spec["pose"][3:])
    d = np.asarray(spec["dir"], float)
    eye = target + HAND_VIEW_DIST * d / np.linalg.norm(d)
    fwd = (target - eye) / np.linalg.norm(target - eye)
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    camera = ColorRenderCamera(RenderCameraCore("vtk", CameraInfo(size[0], size[1], math.radians(HAND_VIEW_FOV)),
                                                ClippingRange(0.01, 3.0), RigidTransform()), False)
    img = sg.get_query_output_port().Eval(sctx).RenderColorImage(
        camera, sg.world_frame_id(), RigidTransform(RotationMatrix(np.column_stack([right, down, fwd])), eye))
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"drake_{hand}_{view}.png"
    Image.fromarray(np.array(img.data)[:, :, :3]).save(path)
    print(path)
    return path


def render(hand: str, pip_degs: list[float], eye=(0.17, -0.10, 0.63), lookat=(0.04, 0.02, 0.535), fov_deg=45.0,
           size=(960, 540), view: str = "overview") -> list[Path]:
    """Drake VTK images of the hand, palm up at MCP 0 and each PIP flexion, from the Isaac Lab video camera
    (``overview``) or ``render3d``'s side camera on the ring finger (``side``)."""
    from PIL import Image
    from pydrake.geometry import ClippingRange, ColorRenderCamera, RenderCameraCore
    from pydrake.math import RigidTransform, RotationMatrix
    from pydrake.systems.sensors import CameraInfo

    urdf, description = paths(hand)
    diagram, plant, sg, model, desc, pins = load_hand(urdf, description, renderer=True)
    ctx = diagram.CreateDefaultContext()
    pctx, sctx = plant.GetMyMutableContextFromRoot(ctx), sg.GetMyContextFromRoot(ctx)
    up = np.array([0.0, 0.0, 1.0])
    if view in ("side", "side_driver"):
        from pydrake.math import RollPitchYaw

        size, fov_deg = (960, 800), SIDE_CAMERA["fov_deg"]
        rot = RollPitchYaw(*HAND_POSE[:3]).ToRotationMatrix().matrix()
        eye_h, look_h, up_h = side_camera(desc) if view == "side" else side_camera(desc, "index", 1.0)
        eye, lookat, up = rot @ eye_h + HAND_POSE[3:], rot @ look_h + HAND_POSE[3:], rot @ up_h
    eye, lookat = np.asarray(eye), np.asarray(lookat)
    fwd = (lookat - eye) / np.linalg.norm(lookat - eye)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    camera = ColorRenderCamera(RenderCameraCore("vtk", CameraInfo(size[0], size[1], math.radians(fov_deg)),
                                                ClippingRange(0.01, 3.0), RigidTransform()), False)
    out = []
    OUT.mkdir(parents=True, exist_ok=True)
    for q in pip_degs:
        set_pose(plant, pctx, model, desc, q)
        img = sg.get_query_output_port().Eval(sctx).RenderColorImage(
            camera, sg.world_frame_id(), RigidTransform(RotationMatrix(np.column_stack([right, down, fwd])), eye))
        path = OUT / f"drake_{hand}_pip{q:g}{'' if view == 'overview' else '_' + view}.png"
        Image.fromarray(np.array(img.data)[:, :, :3]).save(path)
        out.append(path)
        print(path)
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hand", choices=tuple(HAND_FILES), nargs="+", default=["O"])
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--render", type=float, nargs="*", default=None, help="PIP flexions to render [deg]")
    parser.add_argument("--view", choices=("overview", "side", "side_driver"), default="overview")
    parser.add_argument("--hand-view", choices=tuple(HAND_VIEWS), nargs="*", default=None,
                        help="whole-hand views (HAND_VIEWS); none given: all")
    parser.add_argument("--contact", choices=("point", "hydroelastic"), default="point")
    args = parser.parse_args(argv)
    if args.check:
        results = [check(h, args.contact) for h in args.hand]
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / "check.json").write_text(json.dumps(results, indent=1) + "\n")
        for r in results:
            k = r["kinematics"]
            print(f"{r['hand']}: kinematics over q 0..120: pin gap max {k['pin_gap_mm_max']:.2e} mm, flexion error max "
                  f"{k['flexion_error_deg_max']:.2e} deg")
            for q, h in r["held"].items():
                print(f"   held q {q:>3s}: pin gap max {h['pin_gap_mm_max']:.3e} mm (end {h['pin_gap_mm_end']:.3e}), "
                      f"flexion error {h['flexion_error_deg']:.3f} deg")
    if args.render is not None:
        for h in args.hand:
            render(h, args.render or [0.0, 60.0, 120.0], view=args.view)
    if args.hand_view is not None:
        for h in args.hand:
            for view in args.hand_view or HAND_VIEWS:
                render_hand_view(h, view)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
