"""Record a :class:`~core.optimization.linkage.config.LinkageConfig` in
``run.json`` and read it back, dropping the fields of earlier formulations."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, fields

from core.hardware import DShapedExtrusion
from core.morphology.mechanism.linkage.packaging import LinkagePackaging
from core.morphology.packaging.hardware import ContactPad
from core.optimization.linkage.config import (
    DEPRECATED_PACKAGING_FIELDS,
    DEPRECATED_PAD_FIELDS,
    OBJECTIVE_NAMES,
    LinkageConfig,
)


def packaging_from_mapping(values: Mapping[str, object]) -> LinkagePackaging:
    """A :class:`LinkagePackaging` from plain values: servos by actuator name (e.g.
    ``xm335``, ``core.hardware.ACTUATORS``) or as tables, pads as tables."""

    packaging = {name: value for name, value in values.items() if name not in DEPRECATED_PACKAGING_FIELDS}
    # Zero-allowance archived runs have the same geometry. Positive-allowance results require
    # their recorded source; silently dropping that value would change their formulation.
    if float(packaging.pop("body_clearance_mm", 0.0)) != 0.0:
        raise ValueError("body_clearance_mm was removed; use the recorded source for nonzero-clearance runs")
    if values.get("pip_servo_at", "ap") != "ap":
        raise ValueError("the PIP servo is on A' (the servo on A is no longer modelled)")
    if "pin_envelope_diameter_mm" in packaging:  # its earlier name (runs before 2026-10-21)
        packaging["pivot_clearance_diameter_mm"] = packaging.pop("pin_envelope_diameter_mm")
    for joint in ("A", "Bp"):  # as written in a config
        if f"allow_collision_pip_servo_to_joint_{joint}" in packaging:
            packaging[f"allow_collision_pip_servo_to_joint_{joint.lower()}"] = packaging.pop(
                f"allow_collision_pip_servo_to_joint_{joint}")
    for servo in ("pip_servo", "mcp_servo"):
        if isinstance(packaging.get(servo), str):  # an actuator by name, e.g. xm335
            packaging[servo] = actuator(packaging[servo]).geometry
        elif servo in packaging and not isinstance(packaging[servo], DShapedExtrusion):
            packaging[servo] = DShapedExtrusion(**packaging[servo])
    for pad in ("proximal_pad", "distal_pad"):
        if pad in packaging and not isinstance(packaging[pad], ContactPad):
            packaging[pad] = ContactPad(**_plain(packaging[pad]))
    if "mcp_servo_rear" in packaging:
        packaging["mcp_servo_rear"] = tuple(packaging["mcp_servo_rear"])
    return LinkagePackaging(**packaging)


def _plain(values: Mapping[str, object]) -> dict:
    """A pad's values less the deprecated ones (:data:`DEPRECATED_PAD_FIELDS`)."""

    return {name: value for name, value in values.items() if name not in DEPRECATED_PAD_FIELDS}


TUPLE_FIELDS = ("transmission_angle_range_rad", "extended_length_bounds_mm", "proximal_length_bounds_mm",
                "joint_span_bounds_mm", "proximal_length_ratio_bounds", "distal_length_ratio_bounds",
                "joint_span_ratio_bounds", "icr0_share_bounds",
                "objectives", "seeds", "warm_start")


def config_to_json(config: LinkageConfig) -> dict:
    """The config as JSON values, for ``run.json`` (with the search box it used)."""

    values = {f.name: getattr(config, f.name) for f in fields(config)}
    values["bounds"] = dict(config.bounds)
    values["search_bounds"] = dict(config.search_bounds)
    values["packaging"] = asdict(config.packaging)
    values["hypervolume_reference"] = dict(config.hypervolume_reference)
    values["relative_bounds"] = None if config.relative_bounds is None else dict(config.relative_bounds)
    return values


def config_from_json(values: dict) -> LinkageConfig:
    """Rebuild a config recorded by :func:`config_to_json`. Fields of earlier
    formulations (hyperextension, ``torque_ratio_max``, the fold and integrity
    rules, ...) are dropped."""

    raw = dict(values)
    legacy_occlusion = raw.pop("links_block_face", None)
    if "align_mount_offsets" in raw:  # before 2026-11-19 (then the mounting surfaces)
        raw["align_contact_line"] = raw.pop("align_mount_offsets")
    known = {f.name for f in fields(LinkageConfig)}
    values = {name: value for name, value in raw.items() if name in known}
    if "include_linkage_occlusion" not in values:
        values["include_linkage_occlusion"] = bool(legacy_occlusion) if legacy_occlusion is not None else False
    if "profile" not in values:
        values["profile"] = (
            "study" if values.get("mount_policy") == "shallowest_supported" else "prototype"
        )
    # Runs before the problem formulation (2026-11-18) optimized other objectives:
    # they load with its one objective (their designs re-evaluate under its definitions).
    if values.get("objectives") is not None and set(values["objectives"]) - set(OBJECTIVE_NAMES):
        values["objectives"] = ("c_pad_mean",)
    if values.get("hypervolume_reference") is not None:
        values["hypervolume_reference"] = {name: value for name, value in values["hypervolume_reference"].items()
                                           if name in OBJECTIVE_NAMES}
    if "servo_step_rad" in raw:
        values["flexion_step_rad"] = raw["servo_step_rad"]
    values["bounds"] = {name: tuple(bounds) for name, bounds in values["bounds"].items()}
    for name in ("proximal_mount_offset_mm", "distal_mount_offset_mm"):  # runs before 2026-11-04: on the lines
        values["bounds"].setdefault(name, (0.0, 0.0))
    for name in TUPLE_FIELDS:
        if values.get(name) is not None:
            values[name] = tuple(values[name])
    values["packaging"] = packaging_from_mapping(values["packaging"])
    if values.get("relative_bounds") is not None:
        values["relative_bounds"] = {name: tuple(pair) for name, pair in values["relative_bounds"].items()}
    return LinkageConfig(**values)


__all__ = ["config_from_json", "config_to_json", "packaging_from_mapping"]
