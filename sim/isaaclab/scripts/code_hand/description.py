"""Read the CoDe hand description exported by ``design/scripts/export/code_hand_sim.py`` and build it in Newton.

Shared by the loop-closure experiments in this folder. Nothing here imports ``design/core``.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[4]
DEFAULT = REPO / "design" / "output" / "sim" / "code_hand.json"


@dataclass
class Description:
    data: dict

    @classmethod
    def load(cls, path: Path = DEFAULT) -> Description:
        return cls(json.loads(Path(path).read_text()))

    @property
    def bodies(self) -> list[dict]:
        return self.data["bodies"]

    @property
    def joints(self) -> list[dict]:
        return self.data["joints"]

    @property
    def loops(self) -> list[dict]:
        return self.data["loops"]

    def body(self, name: str) -> dict:
        return next(b for b in self.bodies if b["name"] == name)

    def joint(self, name: str) -> dict:
        return next(j for j in self.joints if j["name"] == name)

    def actuated(self) -> list[str]:
        return [j["name"] for j in self.joints if j["actuator"] is not None]

    def pip_from_servo_joint(self, digit: str, ap_deg: float) -> float:
        table = self.data["four_bar"][digit]
        return float(np.interp(ap_deg, table["ap_deg"], table["pip_deg"]))

    def closed_pose(self, digit: str, pip_deg: float) -> dict[str, float]:
        """Tree joint angles [deg] of a finger's four-bar at ``pip_deg`` (from the exported table)."""
        table = self.data["four_bar"][digit]
        at = lambda key: float(np.interp(pip_deg, table["pip_deg"], table[key]))  # noqa: E731
        return {f"{digit}_a": at("a_deg"), f"{digit}_pip_servo": at("ap_deg"), f"{digit}_b": at("b_deg")}


def rod_inertia(body: dict) -> np.ndarray:
    """A placeholder inertia: a slender rod along the body's capsule, isotropic floor 1e-8 kg m^2."""
    start, end = np.array(body["capsule"]["start"]), np.array(body["capsule"]["end"])
    length = float(np.linalg.norm(end - start))
    value = max(body["mass"] * length**2 / 12.0, 1e-8)
    return np.eye(3) * value


def build_newton(desc: Description, loop: str = "ball", ke: float = 5.0, kd: float = 0.05,
                 armature: float = 1e-5, digits: tuple[str, ...] | None = None):
    """A Newton ModelBuilder: the tree as one articulation, each loop cut a joint outside it.

    ``loop``: ``"ball"`` (one CONNECT in MuJoCo), ``"revolute"`` (two CONNECTs) or ``"none"`` (left open).
    """
    import newton
    import warp as wp

    keep = (lambda name: True) if digits is None else (lambda name: name == "palm" or name.split("_")[0] in digits)
    builder = newton.ModelBuilder()
    index, origin = {}, {}
    for body in desc.bodies:
        if not keep(body["name"]):
            continue
        origin[body["name"]] = np.array(body["origin"])
        com = np.array(body["com"]) - origin[body["name"]]
        index[body["name"]] = builder.add_link(
            xform=wp.transform(wp.vec3(*origin[body["name"]]), wp.quat_identity()), com=wp.vec3(*com),
            inertia=wp.mat33(rod_inertia(body)), mass=body["mass"], label=body["name"])

    def xforms(joint):
        anchor = np.array(joint["anchor"])
        return (wp.transform(wp.vec3(*(anchor - origin[joint["parent"]])), wp.quat_identity()),
                wp.transform(wp.vec3(*(anchor - origin[joint["child"]])), wp.quat_identity()))

    tree = [builder.add_joint_fixed(-1, index["palm"], label="palm_fixed")]
    for joint in desc.joints:
        if not keep(joint["child"]):
            continue
        parent_xform, child_xform = xforms(joint)
        lower, upper = (math.radians(v) for v in joint["limits_deg"])
        driven = joint["actuator"] is not None
        tree.append(builder.add_joint_revolute(
            index[joint["parent"]], index[joint["child"]], parent_xform=parent_xform, child_xform=child_xform,
            axis=wp.vec3(*joint["axis"]), target_ke=ke if driven else 0.0, target_kd=kd if driven else 0.0,
            limit_lower=lower, limit_upper=upper, armature=armature, label=joint["name"]))
    builder.add_articulation(tree, label="code_hand")
    loops = []
    for cut in desc.loops:
        if loop == "none" or not keep(cut["child"]):
            continue
        parent_xform, child_xform = xforms(cut)
        if loop == "ball":
            j = builder.add_joint_ball(index[cut["parent"]], index[cut["child"]], parent_xform=parent_xform,
                                       child_xform=child_xform, label=cut["name"])
        else:
            j = builder.add_joint_revolute(index[cut["parent"]], index[cut["child"]], parent_xform=parent_xform,
                                           child_xform=child_xform, axis=wp.vec3(*cut["axis"]), target_ke=0.0,
                                           target_kd=0.0, label=cut["name"])
        builder.joint_articulation[j] = -1
        loops.append(j)
    return builder, index, loops


def rotation_about(r_parent: np.ndarray, r_child: np.ndarray, axis: np.ndarray) -> float:
    """The child's rotation [deg] relative to the parent about ``axis`` (given in the parent's frame)."""
    rel = r_parent.T @ r_child
    # Project: the angle of the rotation about axis, from the in-plane image of a perpendicular vector.
    axis = axis / np.linalg.norm(axis)
    ref = np.cross(axis, [1.0, 0.0, 0.0])
    if np.linalg.norm(ref) < 1e-6:
        ref = np.cross(axis, [0.0, 1.0, 0.0])
    ref /= np.linalg.norm(ref)
    moved = rel @ ref
    return math.degrees(math.atan2(axis @ np.cross(ref, moved), ref @ moved))


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Warp quaternion (x, y, z, w) to a rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
