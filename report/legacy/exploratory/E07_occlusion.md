# E07 — Hardware occlusion across the posture range

**Status: NOT_SUPPORTED as registered.** The registered prediction was that
CoDe's links occlude more at low flexion. The opposite was observed: CoDe
has the **lowest** hardware occlusion of the physical designs at every
posture. Joint-centred hardware occludes the distal face at high flexion.

## 1. Question
Over 0–150°, how much of each face does the finger's own hardware block
from an object approaching along the face normal?

## 2. Hypothesis and alternative
- **Hypothesis.** CoDe's links block more at low flexion and less near
  120°.
- **Alternative.** Occlusion is negligible for all designs.

## 3. Model and assumptions
- The existing scenes at φ = 0–150° in 5° steps.
- A normal probe to depths of 5, 10 and 20 mm (`probe_blocked`).
- Hardware = links, pins, servos, pulleys, rollers and carrier. The MCP
  servo is excluded. The other pad (the fold) is reported separately.
- 2D: a solid in another Y layer still counts.

## 4. Method
- Modules: baseline and thick, plus combined 30×3 and 30×8, added after the
  pilot (registry revision). The 2R-DD does not mount the 30 mm pairs.
- Script: `e07_occlusion.py`. Figure: `e07_occlusion.png`.

## 5. Results: hardware within 10 mm in front of the face, maximum over 0–150°

| Reference | Baseline, distal | Thick, distal | 30×3, distal | 30×8, distal / proximal |
|---|---|---|---|---|
| CoDe | 0 | 0 | 15% (link, at 150°) | 13% / 10% |
| 2R-DD 0° | 60% | 80% | — | — |
| 2R-DD 90° | 38% | 80% | — | — |
| Belt 2R | 38% | 80% | 58% | 87% / 48% |
| RCJ 0° | 63% | 82% | 87% | 88% / 53% |
| RCJ 90° | 0 | 12% | 30% | 52% / 53% |
| *Ideal RCJ* | *0* | *38%* | *43%* | *57% / 38%* |

- At 60° or less, occlusion is at most 1.6% (belt 2R, 30×3 module, at
  50°). Every other design is 0.
- The occluders, by category at 150°:
  - 2R and belt: the proximal bar and the joint servo or pulley;
  - RCJ: the servo (up to 87%) and the rollers.

## 6. Interpretation
- In the side view, hardware concentrated at the joint axis ends up in front
  of the distal face once the finger folds past about 90°.
- CoDe's joint hardware is spread out and dorsal, and its virtual pivot has
  no hardware (E01, E02), so its distal face stays reachable.

## 7. Critical review
- **Normal approach only.** Oblique approach is not tested (candidate C15).
- **Y layers.** Not resolved, so occlusion is an upper bound.
- **A geometric reachability screen.** Occlusion near 150° matters for
  wrapping grasps, not for fingertip pinches. This is not a contact-mechanics
  result.

## 8. Follow-up
None: the result is consistent across modules and probe depths.

## 9. Limitations
- 2D only.
- One morphology.
- Normal probes only.

## 10. Takeaway
Beyond 90°, CoDe keeps its contact faces clear of its own hardware, where
joint-axis hardware (servo, pulley, rollers) occludes up to 60–88% of the
distal face. In this screen, the contact surface stays usable as the
finger closes.
