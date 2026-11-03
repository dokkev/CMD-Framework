"""Any mechanism's finger behind one view, put together as the finger itself
is (``core.morphology.finger``: segments, a joint, its servo).

What every finger shares is :class:`FingerView`'s: the palm (the MCP servo),
the two contact interfaces, the checked pairs' gaps, and the fingertip, the
centre of rotation and ``k`` against the distal flexion ``phi``. What its
joint is, how it moves and how its parts are drawn is its mechanism's view
(``visualize.finger.mechanism.<name>``, beside ``core.morphology.mechanism.<name>``).

    view = view_from_row(row, study_dir)      # a design of a verified front
    view = view_of("rcj", config, x)          # any design vector of a study config
    view.margins(phi_deg)   # each checked pair's gap, (n,) per pair
    view.icr(phi_deg)       # the distal segment's instantaneous centre of rotation
    view.k(phi_deg)         # d(phi)/d(theta_servo): servo torque per distal moment
    view.draw(axis, phi)    # the sketch at one flexion, colliding parts outlined red

From the command line (``python -m visualize.finger``): a front's design or a
design vector through its motion, see ``visualize.finger.__main__``.

All in the proximal frame (``O`` at the origin, ``+X`` distal, ``+Z``
dorsal), MCP at 0, lengths in mm, angles in degrees at this interface.
"""

from visualize.finger.view import FingerView, JointView, KinematicJoint, view_from_row, view_of

__all__ = ["FingerView", "JointView", "KinematicJoint", "view_from_row", "view_of"]
