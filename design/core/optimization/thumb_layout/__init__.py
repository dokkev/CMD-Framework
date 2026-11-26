"""Thumb and hand-layout optimisation for a fixed finger morphology (a robotic Kapandji test).

The design (:class:`~core.optimization.thumb_layout.design.ThumbDesign`): the thumb's CMC (RO) servo position
``(x, z)`` in H, the IP servo's orientation (its body distal, as on the prototype, or turned toward the MCP) and the
fingertip case's extension past the servo. The evaluator (:mod:`.evaluator`) scores it by the robotic Kapandji count
and by the best thumb–index / thumb–middle pinch facing angle. The formulation is in
``docs/experimental/thumb_layout_opt.md``.
"""

from core.optimization.thumb_layout.design import ThumbDesign, hand_parameters, lever_arms
from core.optimization.thumb_layout.evaluator import Evaluation, Evaluator

__all__ = ["Evaluation", "Evaluator", "ThumbDesign", "hand_parameters", "lever_arms"]
