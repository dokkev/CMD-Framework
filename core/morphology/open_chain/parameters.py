"""Reference geometry of the open-chain (serial 2R) finger."""

from __future__ import annotations

from dataclasses import dataclass

from core._validation import require_nonnegative, require_positive


@dataclass(frozen=True)
class OpenChainParameters:
    """An open-chain (serial 2R) finger: the MCP ``O``, the distal FE joint ``A`` at the end of
    the proximal segment, and the distal segment ``A-EE``.

    ``B`` is a point of the distal segment ``b_offset_mm`` from ``A``, the
    reference for a distal contact interface (not a joint). At extension
    ``O``, ``A``, ``B`` and ``EE`` lie along ``+X``.
    """

    proximal_length_mm: float
    """``|OA|``."""
    distal_length_mm: float
    """``|A EE|``."""
    b_offset_mm: float = 0.0
    """``|AB|``, within the distal segment."""

    def __post_init__(self) -> None:
        require_positive("proximal_length_mm", self.proximal_length_mm)
        require_positive("distal_length_mm", self.distal_length_mm)
        require_nonnegative("b_offset_mm", self.b_offset_mm)
        if self.b_offset_mm > self.distal_length_mm:
            raise ValueError("b_offset_mm must not exceed distal_length_mm")


__all__ = ["OpenChainParameters"]
