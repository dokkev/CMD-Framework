# Study 4: the joint as a contact interface

Hypothesis: a skin/bridge between proximal and distal contact regions might form
a usable continuous joint contact surface, and polycentric CoDe motion might
reduce its kinematic deformation burden relative to a fixed-axis 2R.

This is an inexpensive geometric screen, subordinate to `GOAL.md`. It does not
assume polycentric motion is better, and does not model a tactile sensor or policy.

## Matched construction and observables

Use the existing matched CoDe and belt 2R skeletons/FK (belt and direct-drive
2R have identical fixed-axis motion). Neutral attachment x positions are
43.3 and 59.8 mm, on the common support datums, at common palmar depths 5,8,12
mm. Initial span is 16.5 mm, horizontal, identical in each comparison. Proximal
attachment stays fixed; distal attachment follows the existing distal rigid-body
motion. These are hypothetical skin extensions referenced to the support frames,
**not** a claim that the current pad endpoints already coincide at those depths.
No mechanism geometry or bridge datum is optimized to favor an architecture.

At 0:5:120° distal flexion save attachment trajectories, chord length (only a
lower bound on any connecting path), signed longitudinal gap and transverse
step relative to the proximal datum. Save a second **prescribed** path: a cubic
Hermite curve joining the attachments with endpoint tangents aligned to the
proximal and distal supports. Tangent magnitudes both equal the neutral span,
the same construction in both architectures. Arc length is sampled at 401 nodes;
a 401-vs-801 node focused test bounds the integration difference.

Arc-length change divided by neutral span is **not material strain**: a skin
could bow, wrinkle, slide or choose another equilibrium path. Neither this
Hermite curve nor the chord is a solved required surface path. Negative change
means the prescribed path shortens, which a fixed-length smooth skin would have
to accommodate through slack/buckling or other mechanisms.

`normal_discontinuity_deg` is the rigid **support-normal angular mismatch**
`abs(phi)`, identical at matched flexion. It is not an observed bridge normal
jump. Endpoint tangent continuity and zero attachment gap/step are imposed by the
curve construction. Usable continuous contact is therefore not established by
endpoint continuity alone. Save self-intersections, minimum sampled segment
length and maximum local direction turn to flag pathological constructed paths.

A projected routing screen checks a 0.5 mm-radius capsule around the sampled
curve against existing servo envelopes, pivot/roller disks, links and carriers.
Links use a declared 0.5 mm projected radius; other solids use existing geometry.
A negative gap rejects this projected route. The existing Y-layer assumptions
and unknown skin width prevent reading it as a full 3D collision proof. Pads and
the hypothetical extension's support are excluded: attachment adhesion, transition
geometry and skin material remain unresolved. The 5/8/12 mm datum sweep is a
sensitivity check, not a feasibility search.

## Decision gate

A path with self-intersection or projected interference is not evidence for a
usable contact surface. Compare chord and tangent-path trends before spending on
compliance. If an advantage disappears across depths/paths or the construction
fails, report it and stop at geometric screening. A future model needs an actual
skin cross-section/material, mounting constraints, obstacle routing and contact
loads. No beam/FEM parameters are invented for the current screen.

## Reproduce

```
python scripts/study/study4_joint_interface/run.py
python scripts/study/figures34.py
```

Use the same environment as Study 3. Choose a fresh config `output` for reruns.
`screen.csv`, `paths.csv`, `manifest.json`, configs and source snapshot are saved
under `output/study/study4_joint_interface`. Figures read only these results.
