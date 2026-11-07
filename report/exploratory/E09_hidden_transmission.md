# E09 — The hidden transmission change from adaptation

**Status: NOT_SUPPORTED as CoDe-specific.**
- Adapting the morphology changes the distal force map by a median of
  about 4–5× ε_E in CoDe, the RCJ and the ideal RCJ alike, and 1–2.5× in
  the 2R-DD. The samples are small.
- CoDe's distinctive case is ε_Q: on the `long` target, a mechanism change
  of 0.08 moves its lever by 45% at an ε_E of only 0.053.

## 1. Question
When a hand adapts to an interface (Study 2), how much do its transmission
and kinematics change beyond what ε_E reports?

## 2. Hypothesis and alternative
- **Hypothesis.** At comparable ε_E, CoDe changes more than the 2R or the
  RCJ.
- **Alternative.** Changes scale with ε_E alike for every architecture.

## 3. Model and assumptions
- Study 2 (bounded) final designs (`summary_best.csv`).
- Each is compared with its reference **carrying the same target module**,
  so that only the morphology change remains. Rows where the reference
  cannot seat the target are kept in the CSV but excluded from the
  comparison.
- Levers, |Jᵀn| at the face centres, from the Study 3 adapter (existing
  FK):
  - distal face: the PIP column;
  - proximal face: the MCP column;
  - at φ = 0, 30, 60, 90 and 120°.
- Path of the distal face's outer end, used as a fingertip proxy.

## 4. Method
Largest relative lever change, and tip-path deviation divided by L. Script:
`e09_hidden_transmission.py`. Figure: `e05_e09.png` (b).

## 5. Results (same-module comparisons)

| Reference | Cases | Median distal lever change | Median ratio to ε_E | Notable case |
|---|---|---|---|---|
| CoDe | 4 | 45% | 4.4 | long: 45% at ε_E 0.053, ε_Q 0.08 |
| RCJ 0° | 7 | 64% | 5.4 | long_thick, asym_distal_heavy: >100% |
| RCJ 90° | 6 | 67% | 4.8 | |
| *Ideal RCJ* | *6* | *33%* | *4.4* | |
| 2R-DD 0° | 3 | 3% | 1.0 | |
| 2R-DD 90° | 2 | 24% | 2.5 | |

Belt 2R and most 2R-DD cases could not seat the target at the reference, so
they had no same-module baseline.

## 6. Interpretation
- **Not CoDe-specific.** For every architecture that adapts its distal
  span, the servo torque per unit contact force at the distal pad changes
  by tens of percent: a median of about 4–5 times the normalized external
  change for CoDe and the RCJ.
- **The shared mechanism.** The pad's centre moves along the segment, which
  changes the lever arm about the joint. This is common to all
  architectures.
- **CoDe's extra channel.** CoDe can also change Q, its link geometry, and
  on the `long` target that alone changes the map substantially. ε_E does
  not show it.

## 7. Critical review
- The sample is small, and the cases are uneven across architectures.
- Study 2 designs are best-found, not unique. Another design at the same
  ε_E could change less.
- The lever is a model quantity at zero preload, not a measured response.

## 8. Follow-up
Not justified. The general finding is clear, and the CoDe-specific part
rests on one case.

## 9. Limitations
- One reference morphology.
- Face-centre contact only.
- 2D only.

## 10. Takeaway
This supports GOAL.md Section 2.3 for **every** architecture: an interface
change that forces morphology adaptation silently changes the transmission,
usually by several times the visible morphology change. Part of that change
is the pad centre moving, which belongs to the interface itself; CoDe's
link (Q) changes are a further, separate channel. Interface experiments must
therefore hold morphology fixed (Study 1's regime) or record the
transmission change. ε_E alone hides it.
