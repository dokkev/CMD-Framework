# E08 — Two-face enclosing range with thick interfaces

**Status: INCONCLUSIVE.** The metric did not discriminate. Its differences
are explained by E01's quantity, whether hardware protrudes beyond the
contact face, and it saturates at the scan limit.

## 1. Question
What range of object radii can both pad faces touch at once, and how does
thickness change it?

## 2. Hypothesis and alternative
- **Hypothesis.** Thickness shrinks the range less for CoDe.
- **Alternative.** At matched E, the range is set by the spans and the
  flexion alone.

## 3. Model and assumptions
- Circles tangent to both face lines, from the outside, with r from 1 to
  80 mm in 0.25 mm steps.
- Poses from 30° to 150° in 5° steps.
- A radius is admissible when both tangency points lie on the faces (with
  a 0.5 mm margin) and the circle clears every non-pad solid.
- Geometry only.

## 4. Method
The union of admissible radii over the poses. Script: `e08_enclosing.py`.

## 5. Results (admissible measure, mm of radius)

| Reference | Baseline | Thick | 30×3 | 30×8 |
|---|---|---|---|---|
| CoDe | 44.3 | 76.8 | 75.8 | 79.3 |
| 2R-DD 0° | 1.5 | 18.5 | — | — |
| 2R-DD 90° | 45.8 | 79.3 | — | — |
| Belt 2R | 48.5 | 79.3 | 69.8 | 79.3 |
| RCJ 0° | 75.8 | 76.8 | 79.3 | 79.3 |
| RCJ 90° | 25.3 | 67.5 | 79.3 | 79.3 |

## 6. Interpretation
- **Thicker pads widen the range in every design.** A thicker face lies
  further palmar, beyond the hardware, so circles stop hitting the servo or
  pulley housings.
- **Thin pads are limited by hardware protrusion.** Small values (2R-DD 0°
  with 3 mm pads) occur where the joint hardware protrudes past a thin face.
  This is E01's palmar intrusion, measured another way.
- **Saturation.** Many cells reach the 80 mm cap, so their maxima carry no
  information.

## 7. Critical review
- The metric is the union over poses, which hides how the range depends on
  posture.
- Neither force closure nor finite object size along the face is
  represented.
- The registered direction cannot be judged: the measured effect has the
  opposite sign for every design, and its source is not articulation.

## 8. Follow-up
Not justified. A better-posed question would need contact mechanics, which
is out of scope (GOAL.md claim discipline).

## 9. Limitations
- Geometry only.
- 2D only.
- Capped scan.

## 10. Takeaway
No architectural insight beyond E01. The study is recorded as a negative
methodological result: two-face tangency geometry at matched morphology
measures hardware protrusion, not articulation.
