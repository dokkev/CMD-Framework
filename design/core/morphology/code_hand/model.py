"""Derived geometry of the CoDe hand (the parametric model's L2).

``CoDeHand(parameters).frames(configuration)`` returns, in the hand frame H:
- ``servos``: every servo frame (4x4, mm; the actuator centre, see ``parameters``), keyed ``<digit>/<joint>``;
- ``axes``: every joint axis as ``(point, unit direction)``, keyed ``<digit>/<joint>``; a finger's four-bar
  pins are ``A``, ``Ap``, ``B`` and ``Bp``;
- ``points``: the fingertip ``C`` of each finger and the thumb's end-effector origin ``tip``;
- ``ends``: end-effector frames (4x4), so far the thumb's ``thumb/ee``.

Each servo's axis is its part frame's ``+Z``. The chains (``@`` composes left to right):

    finger:  base      = layout pose (MCP-AA servo)
             MCP-FE    = base Rz(90) Rz(aa) T(fe_x, 0, fe_z) Rx(-90)
             proximal  = MCP-FE Rz(fe)             (its X-Y plane is the linkage plane, Y palmar)
             PIP servo = proximal T(Ap_x, -Ap_z, 0) Rz(-pip_servo_angle)
    thumb:   RO        = layout pose
             RAA       = RO Rz(ro_zero + ro) Tz(ro_to_raa) Ry(-90)
             MCP       = RAA Rz(raa_zero + raa) Tz(raa_to_mcp) Ry(-90) Rz(mcp_zero - mcp)
             IP        = MCP Ty(-mcp_to_ip)
             EE        = IP Rz(-ip) T(ee_x, ee_y, ee_z)

The MCP servo body rides on the thumb's distal side, so the MCP angle turns the MCP servo frame itself. Thumb
MCP and IP turn about their servos' ``-Z``, so that positive is flexion (toward the pad), as for the fingers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from core.morphology.code_hand.parameters import (
    FINGERS,
    FingerConfiguration,
    FingerParameters,
    HandConfiguration,
    HandParameters,
    rx,
    ry,
    rz,
    trans,
)
from core.morphology.finger import PointXZ
from core.morphology.mechanism.linkage.model import FourBar


class ClosureError(ValueError):
    pass


def _rot2(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def _circle_intersections(c0, r0, c1, r1) -> list[np.ndarray]:
    d = float(np.linalg.norm(c1 - c0))
    if d < 1e-12 or d > r0 + r1 or d < abs(r0 - r1):
        return []
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = math.sqrt(max(r0**2 - a**2, 0.0))
    u = (c1 - c0) / d
    mid = c0 + a * u
    n = np.array([-u[1], u[0]])
    return [mid + h * n, mid - h * n]


@dataclass(frozen=True)
class FourBarPose:
    """Pins in the linkage plane ``(x, z)``: ``O`` at the origin, ``+X`` along ``O->A``, ``+Z`` dorsal."""

    a: np.ndarray
    ap: np.ndarray
    b: np.ndarray
    bp: np.ndarray
    c: np.ndarray


def _reference_b(finger: FingerParameters) -> np.ndarray:
    """``B`` in the extended pose. The driver ``A'-B`` and the coupler ``A-B'`` fix it: it lies on the circle
    about ``A'`` and on the circle about ``A - B'_D``. Of the two closures this takes the one counter-clockwise
    of ``A' -> centre``, the CoDe assembly branch (checked against the CAD)."""
    a = np.array([finger.a_x_mm, finger.a_z_mm])
    ap = np.array([finger.ap_x_mm, finger.ap_z_mm])
    centre = a - np.array([finger.bp_x_mm, finger.bp_z_mm])
    candidates = _circle_intersections(ap, finger.apb_mm, centre, finger.abp_mm)
    if not candidates:
        raise ClosureError("the four-bar does not close in the extended pose")
    u = centre - ap
    return max(candidates, key=lambda p: u[0] * (p - ap)[1] - u[1] * (p - ap)[0])


def reference_four_bar(finger: FingerParameters) -> FourBar:
    """The finger's four-bar as the kinematic study's :class:`FourBar` (extended pose), which poses it."""
    b = _reference_b(finger)
    bp = b + np.array([finger.bp_x_mm, finger.bp_z_mm])
    return FourBar(PointXZ(finger.a_x_mm, finger.a_z_mm), PointXZ(finger.ap_x_mm, finger.ap_z_mm),
                   PointXZ(*b), PointXZ(*bp))


def four_bar(finger: FingerParameters, pip_deg: float) -> FourBarPose:
    """The pins with the distal segment turned ``pip_deg`` into flexion (clockwise in ``x-z``)."""
    pose = reference_four_bar(finger).solve(math.radians(pip_deg))
    xz = lambda p: np.array([p.x_mm, p.z_mm])
    b = xz(pose.b)
    c = b + _rot2(-pip_deg) @ np.array([finger.c_x_mm, finger.c_z_mm])
    return FourBarPose(xz(pose.a), xz(pose.ap), b, xz(pose.bp), c)


def pip_from_pins(finger: FingerParameters, b: np.ndarray, bp: np.ndarray) -> float:
    """The PIP angle [deg] at which the distal segment carries ``B'`` at ``bp`` from ``B`` at ``b``."""
    ref = np.array([finger.bp_x_mm, finger.bp_z_mm])
    cur = bp - b
    return -math.degrees(math.atan2(ref[0] * cur[1] - ref[1] * cur[0], ref @ cur))


@dataclass(frozen=True)
class HandFrames:
    servos: dict[str, np.ndarray]
    axes: dict[str, tuple[np.ndarray, np.ndarray]]
    points: dict[str, np.ndarray]
    ends: dict[str, np.ndarray] = field(default_factory=dict)


def _axis(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return frame[:3, 3].copy(), frame[:3, 2].copy()


@dataclass(frozen=True)
class CoDeHand:
    parameters: HandParameters

    def finger_frames(self, digit: str, q: FingerConfiguration) -> HandFrames:
        p = self.parameters.finger
        base = self.parameters.layout.base(digit).matrix()
        fe_servo = (base @ rz(90.0) @ rz(q.aa_deg) @ trans(p.fe_axis_x_mm, 0.0, p.fe_axis_z_mm)
                    @ rx(-90.0))
        proximal = fe_servo @ rz(q.fe_deg)
        pip_servo = proximal @ trans(p.ap_x_mm, -p.ap_z_mm, 0.0) @ rz(-p.pip_servo_angle_deg)
        pins = four_bar(p, q.pip_deg)

        def plane(point) -> np.ndarray:  # linkage plane (x, z dorsal) -> H, through the servo centres
            return (proximal @ np.array([point[0], -point[1], 0.0, 1.0]))[:3]

        fe_dir = fe_servo[:3, 2].copy()
        axes = {f"{digit}/aa": _axis(base), f"{digit}/fe": _axis(fe_servo)}
        axes.update({f"{digit}/{n}": (plane(getattr(pins, n.lower())), fe_dir) for n in ("A", "Ap", "B", "Bp")})
        return HandFrames(
            servos={f"{digit}/aa": base, f"{digit}/fe": fe_servo, f"{digit}/pip": pip_servo},
            axes=axes,
            points={f"{digit}/C": plane(pins.c)},
        )

    def thumb_frames(self, q) -> HandFrames:
        p = self.parameters.thumb
        ro = self.parameters.layout.thumb.matrix()
        raa = ro @ rz(p.ro_zero_deg + q.ro_deg) @ trans(z=p.ro_to_raa_mm) @ ry(-90.0)
        mcp = raa @ rz(p.raa_zero_deg + q.raa_deg) @ trans(z=p.raa_to_mcp_mm) @ ry(-90.0) @ rz(p.mcp_zero_deg - q.mcp_deg)
        ip = mcp @ trans(y=-p.mcp_to_ip_mm)
        ee = ip @ rz(-q.ip_deg) @ trans(p.ee_x_mm, p.ee_y_mm, p.ee_z_mm)
        servos = {"thumb/ro": ro, "thumb/raa": raa, "thumb/mcp": mcp, "thumb/ip": ip}
        return HandFrames(servos=servos, axes={k: _axis(v) for k, v in servos.items()},
                          points={"thumb/tip": ee[:3, 3].copy()}, ends={"thumb/ee": ee})

    def frames(self, configuration: HandConfiguration | None = None) -> HandFrames:
        q = configuration or HandConfiguration()
        parts = [self.finger_frames(d, getattr(q, d)) for d in FINGERS] + [self.thumb_frames(q.thumb)]
        return HandFrames(
            servos={k: v for f in parts for k, v in f.servos.items()},
            axes={k: v for f in parts for k, v in f.axes.items()},
            points={k: v for f in parts for k, v in f.points.items()},
            ends={k: v for f in parts for k, v in f.ends.items()},
        )
