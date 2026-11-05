"""The finger mechanisms, each what it is and does (design vector,
kinematics, solids, their gaps along the motion):

- ``linkage``: the four-bar linkage finger (CoDe)
- ``revolute``, ``revolute_parallel``, ``rcj``: the serial 2R (servo on the
  joint, or through a belt from ``A'``) and the rolling contact joint
- ``open_chain``, ``rolling_contact_joint``: their retention-comparison models
"""


VARIANTS = {"revolute_idealized": "revolute", "rcj_idealized": "rcj"}
"""Mechanisms that are another's with a part left out (its servo on the
joint): the same solids, evaluator and mounting rules, by its name."""


def base_mechanism(name: str) -> str:
    """The mechanism ``name`` is a variant of (:data:`VARIANTS`), else ``name``."""

    return VARIANTS.get(name, name)
