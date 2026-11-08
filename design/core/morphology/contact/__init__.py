"""The contact faces of any finger mechanism: what of them is lost, and to
what (``loss``), and the retention comparison's modules and solids (``retention``)."""

from core.morphology.contact.loss import (
    CATEGORIES,
    FaceLoss,
    Scene,
    Shapes,
    dorsal_space_mm,
    face_loss,
    integrity_free,
    joint_scene,
    linkage_scene,
    thickness_mm,
)

__all__ = ["CATEGORIES", "FaceLoss", "Scene", "Shapes", "dorsal_space_mm", "face_loss", "integrity_free", "joint_scene",
           "linkage_scene", "thickness_mm"]
