"""Derived geometry of the CoDe hand (the parametric model's L2).

``CoDeHand(parameters).frames(configuration)`` returns, in the hand frame H:
- ``servos``: every servo frame (4x4, mm; the actuator centre, see ``parameters``), keyed ``<digit>/<joint>``;
- ``axes``: every joint axis as ``(point, unit direction)``, keyed ``<digit>/<joint>``; a finger's four-bar
  pins are ``A``, ``Ap``, ``B`` and ``Bp``;
- ``points``: the fingertip ``C`` of each finger and the thumb's end-effector origin ``tip``;
- ``ends``: end-effector frames (4x4), so far the thumb's ``thumb/ee``;
- ``pads``: each finger pad's box (``PadBox``: centre frame and half extents, mm) when the parameters have pads,
  keyed ``<digit>/proximal_pad`` and ``<digit>/distal_pad``. The box frame's ``+X`` runs along its link toward
  the tip, ``+Y`` palmar (the proximal frame's ``Y``) and ``+Z`` along the FE axis.

Each servo's axis is its part frame's ``+Z``. The chains (``@`` composes left to right):

    finger:  base      = layout pose (MCP-AA servo)
             MCP-FE    = base Rz(90) Rz(aa) T(fe_x, 0, fe_z) Rx(-90)
             proximal  = MCP-FE Rz(fe)             (its X-Y plane is the linkage plane, Y palmar)
             PIP servo = proximal T(Ap_x, -Ap_z, 0) Rz(pip_servo_tilt - 90)
    thumb:   RO        = layout pose
             RAA       = RO Rz(ro) Tz(ro_to_raa) Ry(-90)
             MCP       = RAA Rz(raa + raa_to_mcp_clocking) Tz(raa_to_mcp) Ry(-90) MOUNT Rz(mcp)
             IP        = MCP Ty(-mcp_to_ip)
             EE        = IP Rz(ip) T(ee_x, ee_y, ee_z)

All joint coordinates are right-handed (CCW viewed from +Z toward the origin) about their local +Z axes, and
every flexion is positive: the finger FE and PIP, and the thumb MCP and IP, whose servos are mounted with
their output toward the side that makes it so (``THUMB_MCP_MOUNT``).
The MCP servo body rides on the thumb's distal side, so the MCP angle turns the MCP servo frame itself.
Joint zero is defined by these transforms and the extended four-bar, independently of the CAD pose. The
fixed ``THUMB_MCP_MOUNT`` = Rz(90) Ry(180) aligns the MCP servo's -Y with RAA +Z: at MCP/IP zero the RAA,
MCP, IP and tip centres lie on a straight line (for an on-centre tip, ee_x = ee_z = 0).

Actuator coordinates (``actuators`` / ``configuration_from_actuators``) are each servo's output angle, CCW
about its +Z, from the zero given by ``*_actuator_zero_deg`` (the CAD revolute mates' datum, see
``parameters``). They equal the joint coordinates up to that offset, except the PIP servo, which turns the
four-bar's servo link ``A'-B`` rather than the distal segment.
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
    ThumbConfiguration,
    rx,
    ry,
    rz,
    trans,
)
from core.morphology.finger import PointXZ
from core.morphology.mechanism.linkage.model import FourBar


class ClosureError(ValueError):
    pass


THUMB_MCP_MOUNT = rz(90.0) @ ry(180.0)
"""Fixed rotation from the RAA output (after ``Ry(-90)``) to the MCP servo frame at MCP = 0. ``Rz(90)`` puts
the MCP-IP line on the RAA axis; ``Ry(180)`` turns the servo output side over (the body is symmetric about
its mid-plane, so only the horn side moves) so that +Z rotation flexes the thumb toward its pad (+X)."""

FINGER_ACTUATORS = ("aa", "fe", "pip")
THUMB_ACTUATORS = ("ro", "raa", "mcp", "ip")


def _wrap(deg: float) -> float:
    return math.remainder(deg, 360.0)


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
    """Distal rotation about local +Z (CCW in the proximal X-Y plane).

    The linkage solver uses (x, z dorsal), mapped to (x, -z) in the proximal X-Y plane;
    its clockwise-positive flexion therefore already equals the model's CCW-positive coordinate.
    """
    pose = reference_four_bar(finger).solve(math.radians(pip_deg))
    xz = lambda p: np.array([p.x_mm, p.z_mm])
    b = xz(pose.b)
    c = b + _rot2(-pip_deg) @ np.array([finger.c_x_mm, finger.c_z_mm])
    return FourBarPose(xz(pose.a), xz(pose.ap), b, xz(pose.bp), c)


def pip_servo_deg(finger: FingerParameters, pip_deg: float) -> float:
    """The PIP servo link ``A'->B``'s direction [deg] in the PIP servo frame (CCW about its +Z, from +X)."""
    pins = four_bar(finger, pip_deg)
    v = pins.b - pins.ap
    return _wrap(math.degrees(math.atan2(-v[1], v[0])) - (finger.pip_servo_tilt_deg - 90.0))


def pip_from_servo(finger: FingerParameters, servo_deg: float) -> float:
    """The PIP angle [deg] at which the servo link ``A'->B`` points ``servo_deg`` in the PIP servo frame."""
    turn = math.radians(_wrap(servo_deg - pip_servo_deg(finger, 0.0)))
    sweep = reference_four_bar(finger).sweep_servo([turn])
    if not len(sweep.b):
        raise ClosureError(f"the four-bar does not close at PIP servo {servo_deg} deg")
    return pip_from_pins(finger, np.asarray(sweep.b[0]), np.asarray(sweep.bp[0]))


def pip_from_pins(finger: FingerParameters, b: np.ndarray, bp: np.ndarray) -> float:
    """The PIP angle [deg] at which the distal segment carries ``B'`` at ``bp`` from ``B`` at ``b``."""
    ref = np.array([finger.bp_x_mm, finger.bp_z_mm])
    cur = bp - b
    return -math.degrees(math.atan2(ref[0] * cur[1] - ref[1] * cur[0], ref @ cur))


@dataclass(frozen=True)
class PadBox:
    frame: np.ndarray
    """4x4 in H [mm]: the box centre; ``+X`` along the link, ``+Y`` palmar, ``+Z`` along the FE axis."""
    half_extents_mm: np.ndarray


@dataclass(frozen=True)
class HandFrames:
    servos: dict[str, np.ndarray]
    axes: dict[str, tuple[np.ndarray, np.ndarray]]
    points: dict[str, np.ndarray]
    ends: dict[str, np.ndarray] = field(default_factory=dict)
    pads: dict[str, PadBox] = field(default_factory=dict)


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
        pip_servo = proximal @ trans(p.ap_x_mm, -p.ap_z_mm, 0.0) @ rz(p.pip_servo_tilt_deg - 90.0)
        pins = four_bar(p, q.pip_deg)

        def plane(point) -> np.ndarray:  # linkage plane (x, z dorsal) -> H, through the servo centres
            return (proximal @ np.array([point[0], -point[1], 0.0, 1.0]))[:3]

        fe_dir = fe_servo[:3, 2].copy()
        axes = {f"{digit}/aa": _axis(base), f"{digit}/fe": _axis(fe_servo)}
        axes.update({f"{digit}/{n}": (plane(getattr(pins, n.lower())), fe_dir) for n in ("A", "Ap", "B", "Bp")})
        pads = {}
        if (pad := self.parameters.pads) is not None:
            depth_p, depth_d = (m + pad.thickness_mm / 2.0 for m in pad.mount_offsets_mm)
            half = lambda length: np.array([length / 2.0, pad.thickness_mm / 2.0, pad.width_mm / 2.0])
            mid = (pad.proximal_start_mm + pad.proximal_end_mm) / 2.0
            pads[f"{digit}/proximal_pad"] = PadBox(proximal @ trans(mid, depth_p, 0.0),
                                                  half(pad.proximal_end_mm - pad.proximal_start_mm))
            # The distal frame turns with the distal segment: rz(pip) in the proximal X-Y plane.
            mid = _rot2(-q.pip_deg) @ np.array([(pad.distal_start_mm + pad.distal_end_mm) / 2.0, -depth_d])
            centre = pins.b + mid
            pads[f"{digit}/distal_pad"] = PadBox(proximal @ trans(centre[0], -centre[1], 0.0) @ rz(q.pip_deg),
                                                half(pad.distal_end_mm - pad.distal_start_mm))
        return HandFrames(
            servos={f"{digit}/aa": base, f"{digit}/fe": fe_servo, f"{digit}/pip": pip_servo},
            axes=axes,
            points={f"{digit}/C": plane(pins.c)},
            pads=pads,
        )

    def thumb_frames(self, q) -> HandFrames:
        p = self.parameters.thumb
        ro = self.parameters.layout.thumb.matrix()
        raa = ro @ rz(q.ro_deg) @ trans(z=p.ro_to_raa_mm) @ ry(-90.0)
        mcp = (raa @ rz(q.raa_deg + p.raa_to_mcp_clocking_deg) @ trans(z=p.raa_to_mcp_mm)
               @ ry(-90.0) @ THUMB_MCP_MOUNT @ rz(q.mcp_deg))
        ip = mcp @ trans(y=-p.mcp_to_ip_mm)
        ee = ip @ rz(q.ip_deg) @ trans(p.ee_x_mm, p.ee_y_mm, p.ee_z_mm)
        servos = {"thumb/ro": ro, "thumb/raa": raa, "thumb/mcp": mcp, "thumb/ip": ip}
        return HandFrames(servos=servos, axes={k: _axis(v) for k, v in servos.items()},
                          points={"thumb/tip": ee[:3, 3].copy()}, ends={"thumb/ee": ee})

    def actuators(self, configuration: HandConfiguration | None = None) -> dict[str, float]:
        """Each servo's output angle [deg] from its actuator zero, keyed ``<digit>/<joint>``."""
        q = configuration or HandConfiguration()
        f, t = self.parameters.finger, self.parameters.thumb
        result = {}
        for digit in FINGERS:
            fq = getattr(q, digit)
            result[f"{digit}/aa"] = _wrap(fq.aa_deg - f.aa_actuator_zero_deg)
            result[f"{digit}/fe"] = _wrap(fq.fe_deg - f.fe_actuator_zero_deg)
            result[f"{digit}/pip"] = _wrap(pip_servo_deg(f, fq.pip_deg) - f.pip_actuator_zero_deg)
        for joint in THUMB_ACTUATORS:
            result[f"thumb/{joint}"] = _wrap(getattr(q.thumb, f"{joint}_deg") - getattr(t, f"{joint}_actuator_zero_deg"))
        return result

    def configuration_from_actuators(self, actuators: dict[str, float]) -> HandConfiguration:
        """The joint configuration at the servo angles ``actuators`` (keys as :meth:`actuators`)."""
        f, t = self.parameters.finger, self.parameters.thumb
        fingers = {
            digit: FingerConfiguration(
                aa_deg=_wrap(actuators[f"{digit}/aa"] + f.aa_actuator_zero_deg),
                fe_deg=_wrap(actuators[f"{digit}/fe"] + f.fe_actuator_zero_deg),
                pip_deg=pip_from_servo(f, actuators[f"{digit}/pip"] + f.pip_actuator_zero_deg),
            )
            for digit in FINGERS
        }
        thumb = ThumbConfiguration(**{f"{j}_deg": _wrap(actuators[f"thumb/{j}"] + getattr(t, f"{j}_actuator_zero_deg"))
                                      for j in THUMB_ACTUATORS})
        return HandConfiguration(**fingers, thumb=thumb)

    def frames(self, configuration: HandConfiguration | None = None) -> HandFrames:
        q = configuration or HandConfiguration()
        parts = [self.finger_frames(d, getattr(q, d)) for d in FINGERS] + [self.thumb_frames(q.thumb)]
        return HandFrames(
            servos={k: v for f in parts for k, v in f.servos.items()},
            axes={k: v for f in parts for k, v in f.axes.items()},
            points={k: v for f in parts for k, v in f.points.items()},
            ends={k: v for f in parts for k, v in f.ends.items()},
            pads={k: v for f in parts for k, v in f.pads.items()},
        )
