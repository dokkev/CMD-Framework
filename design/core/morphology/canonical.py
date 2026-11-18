"""Canonical 2D Finger Morphology SSoT (Layers 1, 2, and 3).

Realizes the mathematical formulation in ``design/SSoT/math_formulation.md``:

.. math::
    M = (\\mathbf p_{\\mathrm{mech}}, \\mathbf p_{\\mathrm{act}},
         \\mathbf p_{\\mathrm{struct}}, \\mathbf p_{\\mathrm{contact}}) \\in \\mathcal M(m),
    \\qquad m \\in \\{\\mathrm{CoDe}, \\mathrm{2R}, \\mathrm{2R\\text{-}remote}, \\mathrm{RCJ}\\}.

This module belongs strictly to Layers 1--3 and never imports ``core.optimization``.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Literal

import numpy as np

from core._validation import require_finite, require_nonnegative, require_positive
from core.geometry.planar import Transform2D
from core.hardware import DShapedExtrusion, XM335Actuator
from core.morphology.finger import Finger, Joint, PointXZ, Segment, ServoMount
from core.morphology.mechanism.linkage.model import FourBar
from core.morphology.mechanism.rcj.model import RollingJoint
from core.morphology.mechanism.revolute.model import Pin
from core.morphology.packaging.shapes import ServoFootprint

Architecture = Literal["CoDe", "2R", "2R-remote", "RCJ"]
ARCHITECTURES: tuple[Architecture, ...] = ("CoDe", "2R", "2R-remote", "RCJ")

MECHANISM_TO_ARCHITECTURE: Mapping[str, Architecture] = MappingProxyType(
    {
        "linkage": "CoDe",
        "revolute": "2R",
        "revolute_idealized": "2R",
        "revolute_parallel": "2R-remote",
        "rcj": "RCJ",
        "rcj_idealized": "RCJ",
    }
)
ARCHITECTURE_TO_MECHANISM: Mapping[Architecture, str] = MappingProxyType(
    {
        "CoDe": "linkage",
        "2R": "revolute",
        "2R-remote": "revolute_parallel",
        "RCJ": "rcj",
    }
)

_ANGLE_TOL = 1e-9
_POS_TOL_MM = 1e-6


class MorphologyError(ValueError):
    """Raised when a parameter tuple violates Layer 1/2 admissibility ``M in M(m)``."""


def _default_xm335() -> DShapedExtrusion:
    return XM335Actuator().geometry


def _require_zero_orientation(name: str, tf: Transform2D) -> None:
    if abs(tf.ori) > _ANGLE_TOL:
        raise MorphologyError(f"{name} must have zero local orientation (got {tf.ori!r} rad)")


def _circle_intersections(
    c0: np.ndarray, r0: float, c1: np.ndarray, r1: float
) -> list[np.ndarray]:
    delta = c1 - c0
    d = float(np.linalg.norm(delta))
    if d < 1e-12 or d > r0 + r1 + 1e-12 or d < abs(r0 - r1) - 1e-12:
        return []
    a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    u = delta / d
    mid = c0 + a * u
    n = np.array((-u[1], u[0]), dtype=float)
    return [mid + h * n, mid - h * n]


# ---------------------------------------------------------------------------
# Layer 3 Value Types
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CommonGeometry:
    """Derived Common Finger Geometry ``L(M) = [L_f, L_p, L_d]^T`` [mm] at ``q = 0``."""

    finger_length_mm: float
    proximal_length_mm: float
    distal_length_mm: float

    @property
    def vector(self) -> tuple[float, float, float]:
        return (self.finger_length_mm, self.proximal_length_mm, self.distal_length_mm)


# ---------------------------------------------------------------------------
# Group 1: Mechanism Parameterization p_mech in P_mech(m)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CoDeMechanismParams:
    """Canonical ``CoDe`` four-bar mechanism ``(T_A^{P}, T_{A'}^{P}, T_B^{P}(0), T_{B'}^{D})``.

    Independent canonical dimension: ``dim P_mech(CoDe) = 8``.
    """

    t_a: Transform2D
    t_ap: Transform2D
    t_b0: Transform2D
    t_bp_local: Transform2D

    def __post_init__(self) -> None:
        for label, tf in (
            ("t_a", self.t_a),
            ("t_ap", self.t_ap),
            ("t_b0", self.t_b0),
            ("t_bp_local", self.t_bp_local),
        ):
            if not isinstance(tf, Transform2D):
                raise TypeError(f"{label} must be a Transform2D")
            _require_zero_orientation(label, tf)

    @property
    def architecture(self) -> Architecture:
        return "CoDe"

    @property
    def dim(self) -> int:
        return 8

    @property
    def coordinates(self) -> tuple[float, ...]:
        """Minimal 8D chart ``(x_A, z_A, x_A', z_A', x_{B,0}, z_{B,0}, dx_{B'}, dz_{B'})``."""

        return (
            self.t_a.x_mm,
            self.t_a.z_mm,
            self.t_ap.x_mm,
            self.t_ap.z_mm,
            self.t_b0.x_mm,
            self.t_b0.z_mm,
            self.t_bp_local.x_mm,
            self.t_bp_local.z_mm,
        )

    @classmethod
    def from_coordinates(cls, q: Sequence[float]) -> CoDeMechanismParams:
        if len(q) != 8:
            raise ValueError(f"CoDe minimal coordinates require 8 values; got {len(q)}")
        xa, za, xap, zap, xb0, zb0, dxbp, dzbp = (float(v) for v in q)
        return cls(
            Transform2D.trans(xa, za),
            Transform2D.trans(xap, zap),
            Transform2D.trans(xb0, zb0),
            Transform2D.trans(dxbp, dzbp),
        )

    @classmethod
    def from_extended_pivots(
        cls,
        a_xz: Sequence[float],
        ap_xz: Sequence[float],
        b0_xz: Sequence[float],
        bp0_xz: Sequence[float],
    ) -> CoDeMechanismParams:
        """Construct from extended-pose pivot positions in ``{P}``."""

        xb0, zb0 = float(b0_xz[0]), float(b0_xz[1])
        xbp0, zbp0 = float(bp0_xz[0]), float(bp0_xz[1])
        return cls(
            Transform2D.trans(float(a_xz[0]), float(a_xz[1])),
            Transform2D.trans(float(ap_xz[0]), float(ap_xz[1])),
            Transform2D.trans(xb0, zb0),
            Transform2D.trans(xbp0 - xb0, zbp0 - zb0),
        )

    @classmethod
    def from_bar_lengths(
        cls,
        a_xz: Sequence[float],
        ap_xz: Sequence[float],
        bp_local_xz: Sequence[float],
        abp_length_mm: float,
        apb_length_mm: float,
        *,
        branch: int = 1,
    ) -> CoDeMechanismParams:
        """Construct from intrinsic machined bar lengths ``(|AB'|, |A'B|)`` and closure branch ``(+1 / -1)``."""

        require_positive("abp_length_mm", abp_length_mm)
        require_positive("apb_length_mm", apb_length_mm)
        if branch not in (-1, 1):
            raise ValueError("branch must be +1 or -1")
        a = np.array((float(a_xz[0]), float(a_xz[1])), dtype=float)
        ap = np.array((float(ap_xz[0]), float(ap_xz[1])), dtype=float)
        bp_local = np.array((float(bp_local_xz[0]), float(bp_local_xz[1])), dtype=float)
        centre = a - bp_local
        candidates = _circle_intersections(ap, float(apb_length_mm), centre, float(abp_length_mm))
        if not candidates:
            raise MorphologyError("intrinsic bar lengths do not close in the extended pose")
        u = centre - ap
        chosen = max(
            candidates,
            key=lambda p: branch * float(u[0] * (p[1] - ap[1]) - u[1] * (p[0] - ap[0])),
        )
        return cls(
            Transform2D.trans(float(a[0]), float(a[1])),
            Transform2D.trans(float(ap[0]), float(ap[1])),
            Transform2D.trans(float(chosen[0]), float(chosen[1])),
            Transform2D.trans(float(bp_local[0]), float(bp_local[1])),
        )

    @property
    def t_bp0(self) -> Transform2D:
        """Extended-pose transform of ``B'`` in ``{P}``: ``T_B^{P}(0) @ T_{B'}^{D}``."""

        return self.t_b0 @ self.t_bp_local

    @property
    def bar_lengths_mm(self) -> tuple[float, float]:
        """Derived rigid bar lengths ``(|AB'|, |A'B|)`` [mm]."""

        bp0 = self.t_bp0
        abp = math.hypot(bp0.x_mm - self.t_a.x_mm, bp0.z_mm - self.t_a.z_mm)
        apb = math.hypot(self.t_b0.x_mm - self.t_ap.x_mm, self.t_b0.z_mm - self.t_ap.z_mm)
        return (abp, apb)

    @property
    def four_bar(self) -> FourBar:
        """Underlying :class:`FourBar` joint solver."""

        bp0 = self.t_bp0
        return FourBar(
            a=PointXZ(*self.t_a.pos),
            ap=PointXZ(*self.t_ap.pos),
            b=PointXZ(*self.t_b0.pos),
            bp=PointXZ(*bp0.pos),
        )

    @property
    def joint(self) -> Joint:
        return self.four_bar

    @property
    def distal_origin_extended(self) -> tuple[float, float]:
        return self.t_b0.pos

    def validate(self) -> None:
        if math.hypot(self.t_a.x_mm - self.t_ap.x_mm, self.t_a.z_mm - self.t_ap.z_mm) <= _POS_TOL_MM:
            raise MorphologyError("CoDe proximal pivots A and A' must be distinct")
        if math.hypot(self.t_bp_local.x_mm, self.t_bp_local.z_mm) <= _POS_TOL_MM:
            raise MorphologyError("CoDe distal pivots B and B' must be distinct")
        abp, apb = self.bar_lengths_mm
        if abp <= _POS_TOL_MM or apb <= _POS_TOL_MM:
            raise MorphologyError("CoDe bar lengths |AB'| and |A'B| must be positive")
        try:
            fb = self.four_bar
            fb.check()
        except ValueError as exc:
            raise MorphologyError(f"CoDe four-bar invalid at reference pose: {exc}") from exc

    def distal_pose(self, phi_rad: float) -> Transform2D:
        pose = self.four_bar.solve(phi_rad)
        return Transform2D.trans(pose.b.x_mm, pose.b.z_mm) @ Transform2D.flexion(phi_rad)

    def icr(self, phi_rad: float = 0.0) -> PointXZ | None:
        if phi_rad == 0.0:
            return self.four_bar.icr
        return self.four_bar.solve(phi_rad).icr

    def transmission_ratio(self, phi_rad: float) -> float:
        """Exact analytical ``|d(phi)/d(theta_servo)|`` via implicit loop-closure differentiation."""

        pose = self.four_bar.solve(phi_rad)
        a = np.array(self.t_a.pos, dtype=float)
        ap = np.array(self.t_ap.pos, dtype=float)
        b = pose.b.array
        bp = pose.bp.array

        def perp(v: np.ndarray) -> np.ndarray:
            return np.array((-v[1], v[0]), dtype=float)

        db = -perp(b - ap)
        u = bp - a
        v = b - bp
        denom = float(np.dot(v, perp(u)))
        if abs(denom) < 1e-12:
            return math.inf
        dbp = perp(u) * (float(np.dot(v, db)) / denom)
        dv = db - dbp
        omega = float(v[0] * dv[1] - v[1] * dv[0]) / float(np.dot(v, v))
        return abs(omega)


@dataclass(frozen=True)
class RevoluteMechanismParams:
    """Canonical single-pivot mechanism ``(T_A^{P})`` shared by ``2R`` and ``2R-remote``.

    Independent canonical dimension: ``dim P_mech(2R) = 1``.
    """

    t_a: Transform2D

    def __post_init__(self) -> None:
        if not isinstance(self.t_a, Transform2D):
            raise TypeError("t_a must be a Transform2D")
        _require_zero_orientation("t_a", self.t_a)
        if abs(self.t_a.z_mm) > _POS_TOL_MM:
            raise MorphologyError("2R pivot A must lie on the +X_P reference axis (z_A = 0)")
        if self.t_a.x_mm <= 0.0:
            raise MorphologyError("2R pivot coordinate x_A must be positive")

    @property
    def dim(self) -> int:
        return 1

    @property
    def coordinates(self) -> tuple[float]:
        return (self.t_a.x_mm,)

    @classmethod
    def from_coordinates(cls, q: Sequence[float]) -> RevoluteMechanismParams:
        if len(q) != 1:
            raise ValueError(f"Revolute minimal coordinates require 1 value; got {len(q)}")
        return cls(Transform2D.trans_x(float(q[0])))

    @property
    def pin(self) -> Pin:
        return Pin(PointXZ(self.t_a.x_mm, 0.0))

    @property
    def joint(self) -> Joint:
        return self.pin

    @property
    def distal_origin_extended(self) -> tuple[float, float]:
        return (self.t_a.x_mm, 0.0)

    def validate(self) -> None:
        if self.t_a.x_mm <= 0.0:
            raise MorphologyError("x_A must be positive")

    def distal_pose(self, phi_rad: float) -> Transform2D:
        return Transform2D.trans_x(self.t_a.x_mm) @ Transform2D.flexion(phi_rad)

    def icr(self, phi_rad: float = 0.0) -> PointXZ:
        _ = require_finite("phi_rad", phi_rad)
        return PointXZ(self.t_a.x_mm, 0.0)

    def transmission_ratio(self, phi_rad: float) -> float:
        _ = require_finite("phi_rad", phi_rad)
        return 1.0


@dataclass(frozen=True)
class RcjMechanismParams:
    """Canonical symmetric 1:1 rolling-contact mechanism ``(T_A^{P}, r)``.

    Independent canonical dimension: ``dim P_mech(RCJ) = 2``.
    """

    t_a: Transform2D
    roller_radius_mm: float

    def __post_init__(self) -> None:
        if not isinstance(self.t_a, Transform2D):
            raise TypeError("t_a must be a Transform2D")
        _require_zero_orientation("t_a", self.t_a)
        if abs(self.t_a.z_mm) > _POS_TOL_MM:
            raise MorphologyError("RCJ proximal roller centre A must lie on +X_P (z_A = 0)")
        if self.t_a.x_mm <= 0.0:
            raise MorphologyError("RCJ x_A must be positive")
        r = require_finite("roller_radius_mm", self.roller_radius_mm)
        if r <= 0.0:
            raise MorphologyError("RCJ roller_radius_mm must be positive")
        object.__setattr__(self, "roller_radius_mm", float(r))

    @property
    def architecture(self) -> Architecture:
        return "RCJ"

    @property
    def dim(self) -> int:
        return 2

    @property
    def coordinates(self) -> tuple[float, float]:
        return (self.t_a.x_mm, self.roller_radius_mm)

    @classmethod
    def from_coordinates(cls, q: Sequence[float]) -> RcjMechanismParams:
        if len(q) != 2:
            raise ValueError(f"RCJ minimal coordinates require 2 values; got {len(q)}")
        return cls(Transform2D.trans_x(float(q[0])), float(q[1]))

    @property
    def rolling_joint(self) -> RollingJoint:
        return RollingJoint(PointXZ(self.t_a.x_mm, 0.0), self.roller_radius_mm)

    @property
    def joint(self) -> Joint:
        return self.rolling_joint

    @property
    def distal_origin_extended(self) -> tuple[float, float]:
        return (self.t_a.x_mm + 2.0 * self.roller_radius_mm, 0.0)

    def validate(self) -> None:
        if self.t_a.x_mm <= 0.0 or self.roller_radius_mm <= 0.0:
            raise MorphologyError("RCJ requires x_A > 0 and r > 0")

    def distal_pose(self, phi_rad: float) -> Transform2D:
        pose = self.rolling_joint.solve(phi_rad)
        return Transform2D.trans(pose.b.x_mm, pose.b.z_mm) @ Transform2D.flexion(phi_rad)

    def icr(self, phi_rad: float = 0.0) -> PointXZ:
        return self.rolling_joint.solve(phi_rad).icr

    def transmission_ratio(self, phi_rad: float) -> float:
        _ = require_finite("phi_rad", phi_rad)
        return 2.0


MechanismParams = CoDeMechanismParams | RevoluteMechanismParams | RcjMechanismParams


# ---------------------------------------------------------------------------
# Group 2: Actuation Parameterization p_act = ((G_1, T_1^{W}), (G_2, T_2^{P}))
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ActuatorSpec:
    """Single actuator ``(G_i, T_i)`` in its parent frame (``footprint=None`` for ``G_i = empty``)."""

    footprint: DShapedExtrusion | None
    pose: Transform2D

    def __post_init__(self) -> None:
        if self.footprint is not None and not isinstance(self.footprint, DShapedExtrusion):
            raise TypeError("footprint must be a DShapedExtrusion or None")
        if not isinstance(self.pose, Transform2D):
            raise TypeError("pose must be a Transform2D")

    @property
    def dorsal_tilt_rad(self) -> float:
        """Dorsal tilt ``psi`` from ``-X`` toward ``+Z`` (``ori(T) = -psi``)."""

        return -self.pose.ori

    @property
    def rear_direction(self) -> tuple[float, float]:
        """Unit direction of the rear housing body in the parent frame."""

        return self.pose.apply_vector((-1.0, 0.0))

    def to_servo_footprint(self) -> ServoFootprint | None:
        if self.footprint is None:
            return None
        return ServoFootprint(
            self.pose.pos,
            self.rear_direction,
            self.footprint.radius_mm,
            self.footprint.rear_extent_mm,
        )


@dataclass(frozen=True)
class ActuationParams:
    """Canonical actuation group ``p_act = ((G_1, T_1^{W}), (G_2, T_2^{P}))``."""

    mcp: ActuatorSpec
    pip: ActuatorSpec
    pulley_radius_mm: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.mcp, ActuatorSpec) or not isinstance(self.pip, ActuatorSpec):
            raise TypeError("mcp and pip must be ActuatorSpec instances")
        if self.pulley_radius_mm is not None:
            require_positive("pulley_radius_mm", self.pulley_radius_mm)

    @classmethod
    def from_minimal(
        cls,
        architecture: Architecture,
        mech: MechanismParams,
        *,
        pip_tilt_rad: float = 0.0,
        mcp_tilt_rad: float = 0.5 * math.pi,
        mcp_footprint: DShapedExtrusion | None = None,
        pip_footprint: DShapedExtrusion | None = None,
        idealized_pip: bool = False,
        remote_shaft_xz_mm: Sequence[float] | None = None,
        pulley_radius_mm: float | None = None,
    ) -> ActuationParams:
        """Construct ``p_act`` satisfying shaft-alignment dependencies by construction."""

        g1 = _default_xm335() if mcp_footprint is None else mcp_footprint
        g2 = None if idealized_pip else (_default_xm335() if pip_footprint is None else pip_footprint)
        t_mcp = Transform2D.rot(-require_finite("mcp_tilt_rad", mcp_tilt_rad))
        tilt = require_finite("pip_tilt_rad", pip_tilt_rad)
        if architecture == "CoDe":
            if not isinstance(mech, CoDeMechanismParams):
                raise MorphologyError("CoDe architecture requires CoDeMechanismParams")
            shaft = mech.t_ap.pos
        elif architecture in ("2R", "RCJ"):
            shaft = mech.t_a.pos
        elif architecture == "2R-remote":
            if remote_shaft_xz_mm is None or len(remote_shaft_xz_mm) != 2:
                raise MorphologyError("2R-remote requires explicit remote_shaft_xz_mm = (x_A', z_A')")
            shaft = (float(remote_shaft_xz_mm[0]), float(remote_shaft_xz_mm[1]))
            if pulley_radius_mm is None:
                raise MorphologyError("2R-remote requires positive pulley_radius_mm")
        else:
            raise MorphologyError(f"unknown architecture {architecture!r}")
        t_pip = Transform2D.trans(shaft[0], shaft[1]) @ Transform2D.rot(-tilt)
        return cls(
            mcp=ActuatorSpec(g1, t_mcp),
            pip=ActuatorSpec(g2, t_pip),
            pulley_radius_mm=pulley_radius_mm,
        )


# ---------------------------------------------------------------------------
# Group 3: Structural Admissible Space p_struct in P_struct <= SE(2)^3
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class StructureParams:
    """Canonical structural boundary frames ``(T_{p,end}^{P}, T_{d,start}^{D}, T_{d,end}^{D})``."""

    t_p_end: Transform2D
    t_d_start: Transform2D
    t_d_end: Transform2D

    def __post_init__(self) -> None:
        for label, tf in (
            ("t_p_end", self.t_p_end),
            ("t_d_start", self.t_d_start),
            ("t_d_end", self.t_d_end),
        ):
            if not isinstance(tf, Transform2D):
                raise TypeError(f"{label} must be a Transform2D")
            _require_zero_orientation(label, tf)
            if abs(tf.z_mm) > _POS_TOL_MM:
                raise MorphologyError(f"{label} must lie on its segment +X reference axis (z = 0)")
        if self.t_p_end.x_mm <= 0.0:
            raise MorphologyError("s_{p,end} must be positive")
        if self.t_d_end.x_mm <= 0.0:
            raise MorphologyError("s_{d,end} must be positive")
        if self.t_d_start.x_mm >= self.t_d_end.x_mm:
            raise MorphologyError("s_{d,start} must be strictly less than s_{d,end}")

    @classmethod
    def from_axial(
        cls, s_p_end_mm: float, s_d_start_mm: float, s_d_end_mm: float
    ) -> StructureParams:
        return cls(
            Transform2D.trans_x(float(s_p_end_mm)),
            Transform2D.trans_x(float(s_d_start_mm)),
            Transform2D.trans_x(float(s_d_end_mm)),
        )

    @property
    def dim(self) -> int:
        return 3

    @property
    def coordinates(self) -> tuple[float, float, float]:
        return (self.t_p_end.x_mm, self.t_d_start.x_mm, self.t_d_end.x_mm)

    @property
    def s_p_end_mm(self) -> float:
        return self.t_p_end.x_mm

    @property
    def s_d_start_mm(self) -> float:
        return self.t_d_start.x_mm

    @property
    def s_d_end_mm(self) -> float:
        return self.t_d_end.x_mm

    @property
    def proximal_length_mm(self) -> float:
        """Structural length ``L_p = s_{p,end}``."""

        return self.t_p_end.x_mm

    @property
    def distal_length_mm(self) -> float:
        """Structural length ``L_d = s_{d,end} - s_{d,start}``."""

        return self.t_d_end.x_mm - self.t_d_start.x_mm


# ---------------------------------------------------------------------------
# Group 4: Contact Parameterization p_contact = ((l_i, h_i, T_{c,i}^{b_i}))
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ContactModuleSpec:
    """Single contact interface module ``(l_i, h_i, T_{c,i}^{b_i})`` on body ``b_i in {P, D}``.

    In local contact frame ``{C_i}``, the rectangular block is ``[0, l_i] x [-h_i, 0]``.
    ``T_{c,i}^{b_i} = Trans(s_{c,i}, -d_i)`` with ``d_i >= 0``.
    """

    length_mm: float
    height_mm: float
    pose: Transform2D

    def __post_init__(self) -> None:
        length = require_positive("length_mm", self.length_mm)
        height = require_positive("height_mm", self.height_mm)
        if not isinstance(self.pose, Transform2D):
            raise TypeError("pose must be a Transform2D")
        _require_zero_orientation("contact.pose", self.pose)
        if self.pose.z_mm > _POS_TOL_MM:
            raise MorphologyError(
                f"contact mounting offset d_i = -z_mm must be non-negative (got z_mm={self.pose.z_mm})"
            )
        z_clamped = 0.0 if abs(self.pose.z_mm) <= _POS_TOL_MM else self.pose.z_mm
        if z_clamped != self.pose.z_mm:
            object.__setattr__(self, "pose", Transform2D.trans(self.pose.x_mm, z_clamped))
        object.__setattr__(self, "length_mm", float(length))
        object.__setattr__(self, "height_mm", float(height))

    @classmethod
    def parallel(
        cls, length_mm: float, height_mm: float, s_start_mm: float, mount_offset_mm: float = 0.0
    ) -> ContactModuleSpec:
        d = require_nonnegative("mount_offset_mm", mount_offset_mm)
        return cls(length_mm, height_mm, Transform2D.trans(float(s_start_mm), -float(d)))

    @property
    def s_start_mm(self) -> float:
        return self.pose.x_mm

    @property
    def s_end_mm(self) -> float:
        return self.pose.x_mm + self.length_mm

    @property
    def mount_offset_mm(self) -> float:
        return -self.pose.z_mm

    @property
    def face_depth_mm(self) -> float:
        return self.mount_offset_mm + self.height_mm

    @property
    def box_bounds_local(self) -> tuple[float, float, float, float]:
        """Axis-aligned box ``(u_min, u_max, v_min, v_max)`` in parent body frame ``{b_i}``."""

        u0, u1 = self.s_start_mm, self.s_end_mm
        v1 = -self.mount_offset_mm
        v0 = v1 - self.height_mm
        return (u0, u1, v0, v1)

    def body_polygon(self) -> np.ndarray:
        """Counter-clockwise ``(4, 2)`` outline in parent body frame ``{b_i}``."""

        u0, u1, v0, v1 = self.box_bounds_local
        return np.array(((u0, v0), (u1, v0), (u1, v1), (u0, v1)), dtype=float)

    def body_contact_face(self, side: Literal["proximal", "distal"]) -> tuple[np.ndarray, np.ndarray]:
        """Contact face segment ``(2, 2)`` in ``{b_i}`` ordered from outer end (``xi=0``) to inner end (``xi=l``),
        together with outward palmar unit normal ``(0, -1)``."""

        u0, u1, v0, _ = self.box_bounds_local
        if side == "proximal":
            face = np.array(((u0, v0), (u1, v0)), dtype=float)
        elif side == "distal":
            face = np.array(((u1, v0), (u0, v0)), dtype=float)
        else:
            raise ValueError(f"side must be 'proximal' or 'distal', got {side!r}")
        return face, np.array((0.0, -1.0), dtype=float)


@dataclass(frozen=True)
class ContactParams:
    """Canonical contact group ``p_contact = ((l_p, h_p, T_{c,p}^{P}), (l_d, h_d, T_{c,d}^{D}))``."""

    proximal: ContactModuleSpec
    distal: ContactModuleSpec

    def __post_init__(self) -> None:
        if not isinstance(self.proximal, ContactModuleSpec) or not isinstance(
            self.distal, ContactModuleSpec
        ):
            raise TypeError("proximal and distal must be ContactModuleSpec instances")

    @classmethod
    def outer_pinned(
        cls,
        struct: StructureParams,
        *,
        proximal_length_mm: float,
        proximal_height_mm: float,
        distal_length_mm: float,
        distal_height_mm: float,
        proximal_outer_start_mm: float = 0.0,
        distal_tip_offset_mm: float = 0.0,
        proximal_mount_offset_mm: float = 0.0,
        distal_mount_offset_mm: float = 0.0,
    ) -> ContactParams:
        """Construct ``ContactParams`` under the study/prototype outer-end pinning policy."""

        s_p = float(proximal_outer_start_mm)
        s_d = struct.s_d_end_mm - float(distal_tip_offset_mm) - float(distal_length_mm)
        return cls(
            proximal=ContactModuleSpec.parallel(
                proximal_length_mm, proximal_height_mm, s_p, proximal_mount_offset_mm
            ),
            distal=ContactModuleSpec.parallel(
                distal_length_mm, distal_height_mm, s_d, distal_mount_offset_mm
            ),
        )


# ---------------------------------------------------------------------------
# Complete Canonical Finger Morphology M in M(m)
# ---------------------------------------------------------------------------


DEFAULT_PIVOT_RADII_MM: Mapping[str, float] = MappingProxyType(
    {"a": 3.0, "ap": 5.0, "b": 3.0, "bp": 3.0}
)


@dataclass(frozen=True)
class FingerMorphology:
    """Canonical 2D finger morphology ``M = (p_mech, p_act, p_struct, p_contact) in M(m)``."""

    architecture: Architecture
    mech: MechanismParams
    act: ActuationParams
    struct: StructureParams
    contact: ContactParams
    pivot_radii_mm: Mapping[str, float] = field(default_factory=lambda: DEFAULT_PIVOT_RADII_MM)
    ap_shaft_clearance_radius_mm: float = 5.0

    def __post_init__(self) -> None:
        if self.architecture not in ARCHITECTURES:
            raise MorphologyError(f"unknown architecture {self.architecture!r}; expected one of {ARCHITECTURES}")
        radii = {k: float(require_nonnegative(f"pivot_radii_mm[{k}]", v)) for k, v in self.pivot_radii_mm.items()}
        object.__setattr__(self, "pivot_radii_mm", MappingProxyType(radii))
        require_nonnegative("ap_shaft_clearance_radius_mm", self.ap_shaft_clearance_radius_mm)

    def validate(self, *, require_support_containment: bool = True) -> None:
        """Verify Layer 2 admissibility conditions defining ``M in M(m)``."""

        # 1. Architecture -> Mechanism type & internal closure
        if self.architecture == "CoDe" and not isinstance(self.mech, CoDeMechanismParams):
            raise MorphologyError("architecture 'CoDe' requires CoDeMechanismParams")
        if self.architecture in ("2R", "2R-remote") and not isinstance(
            self.mech, RevoluteMechanismParams
        ):
            raise MorphologyError(f"architecture {self.architecture!r} requires RevoluteMechanismParams")
        if self.architecture == "RCJ" and not isinstance(self.mech, RcjMechanismParams):
            raise MorphologyError("architecture 'RCJ' requires RcjMechanismParams")
        self.mech.validate()

        # 2. Mechanism <-> Actuation shaft alignment
        if math.hypot(*self.act.mcp.pose.pos) > _POS_TOL_MM:
            raise MorphologyError("MCP actuator shaft must coincide with origin O = (0, 0)")
        pip_pos = self.act.pip.pose.pos
        if self.architecture == "CoDe":
            assert isinstance(self.mech, CoDeMechanismParams)
            target = self.mech.t_ap.pos
            if math.hypot(pip_pos[0] - target[0], pip_pos[1] - target[1]) > _POS_TOL_MM:
                raise MorphologyError("CoDe PIP actuator shaft must align with pivot A'")
        elif self.architecture in ("2R", "RCJ"):
            target = self.mech.t_a.pos
            if math.hypot(pip_pos[0] - target[0], pip_pos[1] - target[1]) > _POS_TOL_MM:
                raise MorphologyError(f"{self.architecture} PIP actuator shaft must align with pivot A")
        elif self.architecture == "2R-remote":
            target = self.mech.t_a.pos
            if math.hypot(pip_pos[0] - target[0], pip_pos[1] - target[1]) <= _POS_TOL_MM:
                raise MorphologyError("2R-remote actuator shaft A' must be distinct from joint A")
            if self.act.pulley_radius_mm is None or self.act.pulley_radius_mm <= 0.0:
                raise MorphologyError("2R-remote requires positive pulley_radius_mm")

        # 3. Structure <-> Contact axial support containment (when requested)
        if require_support_containment:
            p_mod, d_mod = self.contact.proximal, self.contact.distal
            if p_mod.s_start_mm < -_POS_TOL_MM or p_mod.s_end_mm > self.struct.s_p_end_mm + _POS_TOL_MM:
                raise MorphologyError(
                    f"proximal contact span [{p_mod.s_start_mm}, {p_mod.s_end_mm}] exceeds "
                    f"proximal structural span [0, {self.struct.s_p_end_mm}]"
                )
            if (
                d_mod.s_start_mm < self.struct.s_d_start_mm - _POS_TOL_MM
                or d_mod.s_end_mm > self.struct.s_d_end_mm + _POS_TOL_MM
            ):
                raise MorphologyError(
                    f"distal contact span [{d_mod.s_start_mm}, {d_mod.s_end_mm}] exceeds "
                    f"distal structural span [{self.struct.s_d_start_mm}, {self.struct.s_d_end_mm}]"
                )

    def is_admissible(self, *, require_support_containment: bool = True) -> bool:
        try:
            self.validate(require_support_containment=require_support_containment)
            return True
        except MorphologyError:
            return False

    # -----------------------------------------------------------------------
    # Layer 3: Derived Properties
    # -----------------------------------------------------------------------

    @property
    def common_geometry(self) -> CommonGeometry:
        """Derived ``L(M) = [L_f, L_p, L_d]^T`` at extended pose ``(q_MCP = 0, phi = 0)``."""

        d0 = self.mech.distal_origin_extended
        tip_x = d0[0] + self.struct.s_d_end_mm
        tip_z = d0[1]
        l_f = math.hypot(tip_x, tip_z)
        return CommonGeometry(
            finger_length_mm=l_f,
            proximal_length_mm=self.struct.proximal_length_mm,
            distal_length_mm=self.struct.distal_length_mm,
        )

    def distal_pose(self, phi_rad: float = 0.0) -> Transform2D:
        """Pose ``T_{PD}(phi; p_mech) in SE(2)`` of distal frame ``{D}`` in ``{P}``."""

        return self.mech.distal_pose(phi_rad)

    def frame_pose(
        self,
        body: Literal["W", "P", "D"],
        phi_rad: float = 0.0,
        q_mcp_rad: float = 0.0,
    ) -> Transform2D:
        """Pose ``T_{Wb}(q_MCP, phi)`` of rigid body ``b in {W, P, D}`` in world frame ``{W}``."""

        if body == "W":
            return Transform2D.identity()
        t_wp = Transform2D.flexion(q_mcp_rad)
        if body == "P":
            return t_wp
        if body == "D":
            return t_wp @ self.distal_pose(phi_rad)
        raise ValueError(f"body must be 'W', 'P', or 'D'; got {body!r}")

    def icr(self, phi_rad: float = 0.0) -> PointXZ | None:
        return self.mech.icr(phi_rad)

    @property
    def icr0_x_mm(self) -> float:
        icr0 = self.mech.icr(0.0)
        return math.nan if icr0 is None else icr0.x_mm

    @property
    def external_descriptor(self) -> dict[str, float]:
        """Study 2 external morphology descriptor ``E(M) = (L_f, L_p, L_d, x_{ICR,0})``."""

        cg = self.common_geometry
        return {
            "finger_length_mm": cg.finger_length_mm,
            "proximal_span_mm": cg.proximal_length_mm,
            "distal_span_mm": cg.distal_length_mm,
            "articulation_mm": self.icr0_x_mm,
        }

    def transmission_ratio(self, phi_rad: float = 0.0) -> float:
        """Instantaneous transmission ratio ``k(M, phi) = |d(phi)/d(theta_act)|``."""

        return self.mech.transmission_ratio(phi_rad)

    def support_rooms_mm(self, distal_tip_offset_mm: float = 0.0) -> tuple[float, float]:
        """Available axial mounting room ``(S_p, S_d)`` from outer datum to ``(A'', B'')``."""

        s_p = self.struct.s_p_end_mm - self.contact.proximal.s_start_mm
        s_d = (self.struct.s_d_end_mm - float(distal_tip_offset_mm)) - self.struct.s_d_start_mm
        return (s_p, s_d)

    def posed_contact_polygons(
        self, phi_rad: float = 0.0, q_mcp_rad: float = 0.0
    ) -> dict[str, np.ndarray]:
        """Posed ``(4, 2)`` contact polygons in ``{W}`` (``{P}`` when ``q_mcp_rad == 0``)."""

        t_p = self.frame_pose("P", phi_rad, q_mcp_rad)
        t_d = self.frame_pose("D", phi_rad, q_mcp_rad)
        return {
            "proximal_pad": t_p.apply_points(self.contact.proximal.body_polygon()),
            "distal_pad": t_d.apply_points(self.contact.distal.body_polygon()),
        }

    def posed_contact_faces(
        self, phi_rad: float = 0.0, q_mcp_rad: float = 0.0
    ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """Posed contact face ``((2, 2) outer->inner segment, (2,) outward unit normal)`` per pad."""

        t_p = self.frame_pose("P", phi_rad, q_mcp_rad)
        t_d = self.frame_pose("D", phi_rad, q_mcp_rad)
        p_face, p_norm = self.contact.proximal.body_contact_face("proximal")
        d_face, d_norm = self.contact.distal.body_contact_face("distal")
        return {
            "proximal_pad": (t_p.apply_points(p_face), np.asarray(t_p.apply_vector(p_norm), dtype=float)),
            "distal_pad": (t_d.apply_points(d_face), np.asarray(t_d.apply_vector(d_norm), dtype=float)),
        }

    def to_finger(self) -> Finger:
        """Project onto the kinematic :class:`~core.morphology.finger.Finger` chain."""

        return Finger(
            proximal=Segment(0.0, self.struct.s_p_end_mm),
            distal=Segment(self.struct.s_d_start_mm, self.struct.s_d_end_mm),
            joint=self.mech.joint,
            servo=ServoMount(self.act.pip.dorsal_tilt_rad),
            pad_offsets_mm=(
                self.contact.proximal.mount_offset_mm,
                self.contact.distal.mount_offset_mm,
            ),
        )


__all__ = [
    "ARCHITECTURES",
    "ARCHITECTURE_TO_MECHANISM",
    "DEFAULT_PIVOT_RADII_MM",
    "MECHANISM_TO_ARCHITECTURE",
    "ActuationParams",
    "ActuatorSpec",
    "Architecture",
    "CoDeMechanismParams",
    "CommonGeometry",
    "ContactModuleSpec",
    "ContactParams",
    "FingerMorphology",
    "MechanismParams",
    "MorphologyError",
    "RcjMechanismParams",
    "RevoluteMechanismParams",
    "StructureParams",
]
