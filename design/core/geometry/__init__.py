"""Shared planar geometry: closed-form distances and interval sets."""

from core.geometry.intervals import (
    IntervalSet,
    disk_occluded_interval,
    face_occluded_set,
    segment_occluded_interval,
)
from core.geometry.primitives import disk_disk_gap_mm, point_rect_distance_mm, rect_disk_gap_mm

__all__ = [
    "IntervalSet",
    "disk_disk_gap_mm",
    "disk_occluded_interval",
    "face_occluded_set",
    "point_rect_distance_mm",
    "rect_disk_gap_mm",
    "segment_occluded_interval",
]
