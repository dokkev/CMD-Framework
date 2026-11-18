"""Explicit bidirectional mapping between flat optimizer vectors (Layer 5)
and Canonical Morphology ``FingerMorphology`` (Layers 1--3).

Every mechanism's ``VARIABLE_NAMES`` slot is classified as:
- **canonical**: independent parameter in ``p_mech``, ``p_act``, ``p_struct``, or ``p_contact``;
- **relative/encoded**: relative coordinate chart of a canonical parameter;
- **policy**: set by a Layer 4 design policy (e.g., ``shallowest_supported`` mounting depths).
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from core.morphology.canonical import (
    MECHANISM_TO_ARCHITECTURE,
    ActuationParams,
    CoDeMechanismParams,
    ContactModuleSpec,
    ContactParams,
    FingerMorphology,
    RcjMechanismParams,
    RevoluteMechanismParams,
    StructureParams,
)
from core.morphology.mechanism.linkage.design import VARIABLE_NAMES as LINKAGE_VARS
from core.morphology.mechanism.linkage.design import decode as decode_linkage
from core.morphology.mechanism.linkage.packaging import pad_offsets, pad_spans
from core.morphology.mechanism.rcj.solids import VARIABLE_NAMES as RCJ_VARS
from core.morphology.mechanism.revolute.solids import VARIABLE_NAMES as REVOLUTE_VARS
from core.morphology.mechanism.revolute_parallel.solids import VARIABLE_NAMES as PARALLEL_VARS
from core.optimization.analysis.tradeoff import goal_of


@dataclass(frozen=True)
class VariableSlotSpec:
    """Classification of one flat optimization vector slot against the SSoT."""

    name: str
    group: str
    """``"mech"``, ``"act"``, ``"struct"``, or ``"contact"``."""
    role: str
    """``"canonical"``, ``"relative_chart"``, or ``"policy_or_canonical"``."""
    ssot_expression: str


VARIABLE_SLOT_MAPPING: Mapping[str, tuple[VariableSlotSpec, ...]] = MappingProxyType(
    {
        "linkage": (
            VariableSlotSpec("a_x_mm", "mech", "canonical", "x_A in pos(T_A^{P})"),
            VariableSlotSpec("b_x_mm", "mech", "canonical", "x_{B,0} in pos(T_B^{P}(0))"),
            VariableSlotSpec("b_z_mm", "mech", "canonical", "z_{B,0} in pos(T_B^{P}(0))"),
            VariableSlotSpec("ap_x_mm", "mech", "canonical", "x_{A'} in pos(T_{A'}^{P}); induces pos(T_2^{P})"),
            VariableSlotSpec("ap_z_mm", "mech", "canonical", "z_{A'} in pos(T_{A'}^{P}); induces pos(T_2^{P})"),
            VariableSlotSpec("bp_x_mm", "mech", "relative_chart", "x_{B',0} = x_{B,0} + dx_{B'}"),
            VariableSlotSpec("bp_z_mm", "mech", "relative_chart", "z_{B',0} = z_{B,0} + dz_{B'}"),
            VariableSlotSpec("servo_tilt_deg", "act", "canonical", "psi_tilt = -ori(T_2^{P}) [deg]"),
            VariableSlotSpec("bc_length_mm", "struct", "canonical", "s_{d,end} in T_{d,end}^{D}"),
            VariableSlotSpec("a_z_mm", "mech", "canonical", "z_A in pos(T_A^{P}) (0 in axis-aligned study subspace)"),
            VariableSlotSpec("app_offset_mm", "struct", "relative_chart", "s_{p,end} - x_{A'}"),
            VariableSlotSpec("bpp_offset_mm", "struct", "relative_chart", "x_{B',0} - (x_{B,0} + s_{d,start})"),
            VariableSlotSpec("proximal_mount_offset_mm", "contact", "policy_or_canonical", "d_p = -z(T_{c,p}^{P})"),
            VariableSlotSpec("distal_mount_offset_mm", "contact", "policy_or_canonical", "d_d = -z(T_{c,d}^{D})"),
        ),
        "revolute": (
            VariableSlotSpec("a_x_mm", "mech", "canonical", "x_A in pos(T_A^{P}); induces pos(T_2^{P})"),
            VariableSlotSpec("ac_length_mm", "struct", "canonical", "s_{d,end} in T_{d,end}^{D}"),
            VariableSlotSpec("app_x_mm", "struct", "canonical", "s_{p,end} in T_{p,end}^{P}"),
            VariableSlotSpec("bpp_from_a_mm", "struct", "canonical", "s_{d,start} in T_{d,start}^{D}"),
            VariableSlotSpec("distal_drop_mm", "contact", "policy_or_canonical", "d_d = -z(T_{c,d}^{D})"),
            VariableSlotSpec("proximal_drop_mm", "contact", "policy_or_canonical", "d_p = -z(T_{c,p}^{P})"),
            VariableSlotSpec("servo_tilt_deg", "act", "canonical", "psi_tilt = -ori(T_2^{P}) [deg]"),
        ),
        "revolute_parallel": (
            VariableSlotSpec("a_x_mm", "mech", "canonical", "x_A in pos(T_A^{P})"),
            VariableSlotSpec("ac_length_mm", "struct", "canonical", "s_{d,end} in T_{d,end}^{D}"),
            VariableSlotSpec("app_x_mm", "struct", "canonical", "s_{p,end} in T_{p,end}^{P}"),
            VariableSlotSpec("bpp_from_a_mm", "struct", "canonical", "s_{d,start} in T_{d,start}^{D}"),
            VariableSlotSpec("distal_drop_mm", "contact", "policy_or_canonical", "d_d = -z(T_{c,d}^{D})"),
            VariableSlotSpec("proximal_drop_mm", "contact", "policy_or_canonical", "d_p = -z(T_{c,p}^{P})"),
            VariableSlotSpec("servo_tilt_deg", "act", "canonical", "psi_tilt = -ori(T_2^{P}) [deg]"),
            VariableSlotSpec("ap_from_a_x_mm", "act", "relative_chart", "x_{A'} - x_A in pos(T_2^{P})"),
            VariableSlotSpec("ap_from_a_z_mm", "act", "canonical", "z_{A'} in pos(T_2^{P})"),
        ),
        "rcj": (
            VariableSlotSpec("a_x_mm", "mech", "canonical", "x_A in pos(T_A^{P}); induces pos(T_2^{P})"),
            VariableSlotSpec("roller_radius_mm", "mech", "canonical", "r in p_mech^{RCJ}"),
            VariableSlotSpec("bc_length_mm", "struct", "canonical", "s_{d,end} in T_{d,end}^{D}"),
            VariableSlotSpec("app_x_mm", "struct", "canonical", "s_{p,end} in T_{p,end}^{P}"),
            VariableSlotSpec("bpp_from_b_mm", "struct", "canonical", "s_{d,start} in T_{d,start}^{D}"),
            VariableSlotSpec("distal_drop_mm", "contact", "policy_or_canonical", "d_d = -z(T_{c,d}^{D})"),
            VariableSlotSpec("proximal_drop_mm", "contact", "policy_or_canonical", "d_p = -z(T_{c,p}^{P})"),
            VariableSlotSpec("servo_tilt_deg", "act", "canonical", "psi_tilt = -ori(T_2^{P}) [deg]"),
        ),
    }
)


def _mcp_tilt_rad(packaging) -> float:
    rx, rz = packaging.mcp_servo_rear
    return math.atan2(rz, -rx)


def morphology_from_design(
    mechanism_name: str,
    x: Sequence[float],
    config: Any,
    *,
    apply_grid_assembly: bool = True,
) -> FingerMorphology:
    """Decode ``(mechanism_name, x, config)`` into a canonical :class:`FingerMorphology`."""

    if mechanism_name not in MECHANISM_TO_ARCHITECTURE:
        raise ValueError(f"unknown mechanism {mechanism_name!r}")
    arch = MECHANISM_TO_ARCHITECTURE[mechanism_name]
    goal = goal_of(config)
    packaging = goal.packaging
    radii = {p: packaging.pivot_radius_mm(p) for p in ("a", "ap", "b", "bp")}
    mcp_tilt = _mcp_tilt_rad(packaging)

    if mechanism_name == "linkage":
        res = goal.link_length_resolution_mm if apply_grid_assembly else None
        finger = decode_linkage(x, res)
        j = finger.joint
        mech = CoDeMechanismParams.from_extended_pivots(
            (j.a.x_mm, j.a.z_mm),
            (j.ap.x_mm, j.ap.z_mm),
            (j.b.x_mm, j.b.z_mm),
            (j.bp.x_mm, j.bp.z_mm),
        )
        act = ActuationParams.from_minimal(
            "CoDe",
            mech,
            pip_tilt_rad=finger.servo.tilt_rad,
            mcp_tilt_rad=mcp_tilt,
            mcp_footprint=packaging.mcp_servo,
            pip_footprint=packaging.pip_servo,
        )
        struct = StructureParams.from_axial(
            finger.proximal.end_mm, finger.distal.start_mm, finger.distal.end_mm
        )
        spans = pad_spans(finger, packaging)
        offs = pad_offsets(finger, packaging)
        p_span, d_span = spans["proximal_pad"], spans["distal_pad"]
        contact = ContactParams(
            proximal=ContactModuleSpec.parallel(
                max(p_span.length_mm, 1e-6),
                packaging.proximal_pad.height_mm,
                p_span.start_mm,
                offs["proximal_pad"],
            ),
            distal=ContactModuleSpec.parallel(
                max(d_span.length_mm, 1e-6),
                packaging.distal_pad.height_mm,
                d_span.start_mm,
                offs["distal_pad"],
            ),
        )
        return FingerMorphology(
            architecture="CoDe",
            mech=mech,
            act=act,
            struct=struct,
            contact=contact,
            pivot_radii_mm=radii,
            ap_shaft_clearance_radius_mm=packaging.ap_clearance_diameter_mm / 2.0,
        )

    if mechanism_name in ("revolute", "revolute_idealized", "revolute_parallel"):
        var_names = PARALLEL_VARS if mechanism_name == "revolute_parallel" else REVOLUTE_VARS
        v = dict(zip(var_names, (float(val) for val in x), strict=True))
        mech = RevoluteMechanismParams.from_coordinates((v["a_x_mm"],))
        joint_servo = getattr(config, "joint_servo", mechanism_name == "revolute")
        if mechanism_name == "revolute_parallel":
            act = ActuationParams.from_minimal(
                "2R-remote",
                mech,
                pip_tilt_rad=math.radians(v["servo_tilt_deg"]),
                mcp_tilt_rad=mcp_tilt,
                mcp_footprint=packaging.mcp_servo,
                pip_footprint=packaging.pip_servo,
                remote_shaft_xz_mm=(v["a_x_mm"] + v["ap_from_a_x_mm"], v["ap_from_a_z_mm"]),
                pulley_radius_mm=config.pulley_radius_mm,
            )
        else:
            act = ActuationParams.from_minimal(
                "2R",
                mech,
                pip_tilt_rad=math.radians(v["servo_tilt_deg"]),
                mcp_tilt_rad=mcp_tilt,
                mcp_footprint=packaging.mcp_servo,
                pip_footprint=packaging.pip_servo,
                idealized_pip=not joint_servo,
            )
        struct = StructureParams.from_axial(v["app_x_mm"], v["bpp_from_a_mm"], v["ac_length_mm"])
        start_p = packaging.proximal_pad_start_mm
        outer_d = v["ac_length_mm"] - packaging.distal_pad.offset_mm
        room_p = v["app_x_mm"] - start_p
        room_d = outer_d - v["bpp_from_a_mm"]
        len_p = packaging.proximal_pad.length_mm or min(room_p, packaging.proximal_pad.max_length_mm or math.inf)
        len_d = packaging.distal_pad.length_mm or min(room_d, packaging.distal_pad.max_length_mm or math.inf)
        len_p = max(len_p, 1e-6)
        len_d = max(len_d, 1e-6)
        contact = ContactParams(
            proximal=ContactModuleSpec.parallel(
                len_p, packaging.proximal_pad.height_mm, start_p, v["proximal_drop_mm"]
            ),
            distal=ContactModuleSpec.parallel(
                len_d, packaging.distal_pad.height_mm, outer_d - len_d, v["distal_drop_mm"]
            ),
        )
        return FingerMorphology(
            architecture=arch,
            mech=mech,
            act=act,
            struct=struct,
            contact=contact,
            pivot_radii_mm=radii,
        )

    # RCJ / Idealized RCJ
    vals = [float(val) for val in x]
    if len(vals) == 7:
        vals.append(0.0)
    v = dict(zip(RCJ_VARS, vals, strict=True))
    mech = RcjMechanismParams.from_coordinates((v["a_x_mm"], v["roller_radius_mm"]))
    joint_servo = getattr(config, "joint_servo", mechanism_name == "rcj")
    act = ActuationParams.from_minimal(
        "RCJ",
        mech,
        pip_tilt_rad=math.radians(v["servo_tilt_deg"]),
        mcp_tilt_rad=mcp_tilt,
        mcp_footprint=packaging.mcp_servo,
        pip_footprint=packaging.pip_servo,
        idealized_pip=not joint_servo,
    )
    struct = StructureParams.from_axial(v["app_x_mm"], v["bpp_from_b_mm"], v["bc_length_mm"])
    start_p = packaging.proximal_pad_start_mm
    outer_d = v["bc_length_mm"] - packaging.distal_pad.offset_mm
    room_p = v["app_x_mm"] - start_p
    room_d = outer_d - v["bpp_from_b_mm"]
    len_p = packaging.proximal_pad.length_mm or min(room_p, packaging.proximal_pad.max_length_mm or math.inf)
    len_d = packaging.distal_pad.length_mm or min(room_d, packaging.distal_pad.max_length_mm or math.inf)
    len_p = max(len_p, 1e-6)
    len_d = max(len_d, 1e-6)
    contact = ContactParams(
        proximal=ContactModuleSpec.parallel(
            len_p, packaging.proximal_pad.height_mm, start_p, v["proximal_drop_mm"]
        ),
        distal=ContactModuleSpec.parallel(
            len_d, packaging.distal_pad.height_mm, outer_d - len_d, v["distal_drop_mm"]
        ),
    )
    return FingerMorphology(
        architecture="RCJ",
        mech=mech,
        act=act,
        struct=struct,
        contact=contact,
        pivot_radii_mm=radii,
    )


def design_from_morphology(morphology: FingerMorphology, mechanism_name: str | None = None) -> tuple[float, ...]:
    """Encode a :class:`FingerMorphology` into the flat optimization vector of ``mechanism_name``."""

    arch = morphology.architecture
    if mechanism_name is None:
        if arch == "2R" and morphology.act.pip.footprint is None:
            mechanism_name = "revolute_idealized"
        elif arch == "RCJ" and morphology.act.pip.footprint is None:
            mechanism_name = "rcj_idealized"
        else:
            from core.morphology.canonical import ARCHITECTURE_TO_MECHANISM

            mechanism_name = ARCHITECTURE_TO_MECHANISM[arch]

    tilt_deg = round(math.degrees(morphology.act.pip.dorsal_tilt_rad), 9)
    d_p = morphology.contact.proximal.mount_offset_mm
    d_d = morphology.contact.distal.mount_offset_mm

    if mechanism_name == "linkage":
        assert isinstance(morphology.mech, CoDeMechanismParams)
        m = morphology.mech
        bp0 = m.t_bp0
        app_offset = round(morphology.struct.s_p_end_mm - m.t_ap.x_mm, 9)
        bpp_offset = round(bp0.x_mm - (m.t_b0.x_mm + morphology.struct.s_d_start_mm), 9)
        return (
            m.t_a.x_mm,
            m.t_b0.x_mm,
            m.t_b0.z_mm,
            m.t_ap.x_mm,
            m.t_ap.z_mm,
            bp0.x_mm,
            bp0.z_mm,
            tilt_deg,
            morphology.struct.s_d_end_mm,
            m.t_a.z_mm,
            app_offset,
            bpp_offset,
            d_p,
            d_d,
        )

    if mechanism_name in ("revolute", "revolute_idealized"):
        assert isinstance(morphology.mech, RevoluteMechanismParams)
        return (
            morphology.mech.t_a.x_mm,
            morphology.struct.s_d_end_mm,
            morphology.struct.s_p_end_mm,
            morphology.struct.s_d_start_mm,
            d_d,
            d_p,
            0.0 if mechanism_name == "revolute_idealized" else tilt_deg,
        )

    if mechanism_name == "revolute_parallel":
        assert isinstance(morphology.mech, RevoluteMechanismParams)
        a_x = morphology.mech.t_a.x_mm
        ap_x, ap_z = morphology.act.pip.pose.pos
        return (
            a_x,
            morphology.struct.s_d_end_mm,
            morphology.struct.s_p_end_mm,
            morphology.struct.s_d_start_mm,
            d_d,
            d_p,
            tilt_deg,
            round(ap_x - a_x, 9),
            ap_z,
        )

    if mechanism_name in ("rcj", "rcj_idealized"):
        assert isinstance(morphology.mech, RcjMechanismParams)
        return (
            morphology.mech.t_a.x_mm,
            morphology.mech.roller_radius_mm,
            morphology.struct.s_d_end_mm,
            morphology.struct.s_p_end_mm,
            morphology.struct.s_d_start_mm,
            d_d,
            d_p,
            0.0 if mechanism_name == "rcj_idealized" else tilt_deg,
        )

    raise ValueError(f"unknown mechanism_name {mechanism_name!r}")


__all__ = [
    "LINKAGE_VARS",
    "VARIABLE_SLOT_MAPPING",
    "VariableSlotSpec",
    "design_from_morphology",
    "morphology_from_design",
]
