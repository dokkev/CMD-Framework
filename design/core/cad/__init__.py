"""CAD integration: an Onshape REST client and the hand-layout driver.

Note: The CAD connection is **not** a source of truth. The canonical source of truth for hand morphology and
kinematics is ``core.morphology.code_hand`` (and its tracked YAML parameter files). ``core.cad`` exists only to
push model parameters to Onshape or inspect/verify CAD assembly snapshots against the parametric model.
"""
