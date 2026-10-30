"""Figures of the CMD framework's models and studies, apart from the models
(``core``) and the studies' runs (``scripts/optimize``).

Shared pieces:

- ``style``: colours, markers and labels per mechanism, themes, where figures are saved
- ``sketch``: the schematic finger (servos as boxes, links red, pads green)
- ``solids``: the finger as filled solids (pads pink and blue, links in the mechanism's colour)
- ``motion``: a linkage swept by its servo, sampled at a flexion, its gaps along the sweep
- ``gaps``: each checked pair's gap against the flexion
- ``fronts``: a study's verified fronts (``portfolio_NNN/seeds.csv``) and their runs
- ``finger``: any mechanism's design behind one interface (pose, gaps, ICR, ``k``, sketch)

Figures, each run from the repository root as ``python -m visualize.<package>.<module>``:

- ``paper``: the paper's figures (``output/paper/``)
- ``comparison``: the linkage against the serial 2R and the RCJ (``output/comparison/``)
- ``study``: a study's fronts (Pareto, convergence, accommodation, length budget, loss profile)
- ``animate``: the 2-DoF finger animated, ``hardware``: the actuator and fingertip models
"""
