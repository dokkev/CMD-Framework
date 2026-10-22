"""Configuration of the linkage formulation v2.

The contact interface is an input (the pad lengths ``L_p``, ``L_d`` as the
least mounting-line ``a-b`` lengths, ``minimum_*_support_mm``, with fixed
height and chamfer), the required ROM a constraint, and the optimizer
minimizes the transmission and the finger's length. Everything else (the
linkage, its packaging, the machining grid and the search-box narrowing) is
formulation v1's :class:`~core.optimization.linkage.config.FingerStudyConfig`.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType

from core._validation import require_nonnegative, require_positive
from core.morphology.linkage.packaging import ContactPad, Packaging

from core.optimization.linkage.config import FingerStudyConfig


EXTENSION_PRIORITIES = ("balanced", "proximal_first", "distal_first")
ALIASES = MappingProxyType({
    "module_max_length_p": "minimum_proximal_support_mm",
    "module_max_length_d": "minimum_distal_support_mm",
    "phi_req_deg": "required_rom_deg",
})
"""Config file names of v2's inputs."""


def _v2_packaging() -> Packaging:
    pad = ContactPad(height_mm=10.0, offset_mm=0.0, chamfer_mm=2.5, length_mm=None)
    return Packaging(proximal_pad=pad, distal_pad=pad)


@dataclass(frozen=True)
class LinkageV2Config(FingerStudyConfig):
    """Formulation v2: v1's fields with these meanings and defaults, and the ROM.

    - ``minimum_proximal_support_mm`` / ``minimum_distal_support_mm``
      (config: ``module_max_length_p`` / ``_d``): ``L_p`` and ``L_d``, the
      longest module each pad must accommodate, enforced as the least
      mounting-line ``a-b`` length (``A''`` / ``B''`` extensions included);
    - ``flexion_stopper_margin_rad``: how far past ``phi_req`` the motion must
      stay danger-free (the stopper's tolerance; default 0). It no longer
      ends the usable range early: ``phi_req`` is always the usable ROM;
    - ``extended_length_bounds_mm``: a search limit on ``|OC|``, now an objective;
    - ``minimum_flexion_rad`` (v1) is not used: :attr:`required_rom_rad` replaces it.
    """

    packaging: Packaging = field(default_factory=_v2_packaging)
    minimum_proximal_support_mm: float = 30.0
    minimum_distal_support_mm: float = 40.0
    extended_length_bounds_mm: tuple[float, float] = (40.0, 150.0)
    enforce_curl_tip: bool = True
    pip_servo_tilt_rad: float | None = math.radians(20.0)
    flexion_stopper_margin_rad: float = 0.0
    required_rom_rad: float = math.radians(150.0)
    """``phi_req``: the distal rotation relative to ``OA`` every design must
    reach safely. ``k_max`` is measured on ``[0, phi_req]``; the pads, ``C_x``
    and the linkage validity must hold on ``[0, phi_req + margin]``, the
    validity (assembly, transmission margin, DIP and PIP backoff) from the PIP
    hyperextension on."""
    extension_priority: str = "balanced"
    """How the pads are lengthened after the optimization
    (``core.optimization.linkage.extend.extend_pads``): ``balanced``,
    ``proximal_first`` or ``distal_first``."""
    hypervolume_reference: Mapping[str, float] | None = None
    """Reference point per objective (``transmission``: ``k_max``,
    ``finger_length``: ``|OC|`` mm); ``None``: ``k_max`` 2.3 and the ``|OC|``
    search upper bound."""

    def __post_init__(self) -> None:
        super().__post_init__()
        require_positive("required_rom_rad", self.required_rom_rad)
        require_nonnegative("flexion_stopper_margin_rad", self.flexion_stopper_margin_rad)
        if self.required_rom_rad + self.flexion_stopper_margin_rad > math.pi + 1e-12:
            raise ValueError("phi_req + flexion_stopper_margin may not exceed 180 deg (BC antiparallel to OA)")
        if self.extension_priority not in EXTENSION_PRIORITIES:
            raise ValueError(f"extension_priority must be one of {EXTENSION_PRIORITIES}")
        if self.reference_point != "C":
            raise ValueError("formulation v2 measures the finger at C")
        reference = {"transmission": 2.3, "finger_length": self.extended_length_bounds_mm[1]}
        reference.update(self.hypervolume_reference or {})
        object.__setattr__(self, "hypervolume_reference", MappingProxyType(reference))

    @property
    def target_rad(self) -> float:
        """``phi_req + margin``: how far the motion must stay safe."""

        return self.required_rom_rad + self.flexion_stopper_margin_rad


__all__ = ["ALIASES", "EXTENSION_PRIORITIES", "LinkageV2Config"]
