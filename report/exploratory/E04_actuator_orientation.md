# E04 — Actuator-orientation sensitivity

**Status: SUPPORTED, with a qualification that matters.**
- Remote-actuated designs do not depend on servo tilt over a wide window.
- Joint-servo designs depend strongly on it.
- At their best tilt, the joint-servo designs come within 3–4 pairs of CoDe.

## 1. Question
How much does the accommodation verdict depend on the actuator's
orientation, with the morphology frozen?

## 2. Hypothesis and alternative
- **Hypothesis.** Little dependence for CoDe and the belt 2R (remote);
  strong dependence for the 2R-DD and the RCJ (servo on the joint).
- **Alternative.** Similar for all, because a servo envelope always sits
  near a pad.

## 3. Model and assumptions
- Matched references; E and Q are held.
- `servo_tilt_deg` swept from −30° to 120° in 10° steps, through the goal's
  actuation input (`pip_servo_tilt_rad`, which is not a shared comparison
  rule).
- Policy depths follow the servo.
- Judged on the Study 1 catalog (71 pairs) at 120°.

## 4. Method
For each tilt: pairs carried and pairs mounted, and whether the bare
reference is itself feasible. Script: `e04_actuator_orientation.py`.
Figure: `e04_actuator_orientation.png`.

## 5. Results (pairs carried at 120°)

| Tilt | −30° | −10° | 0° | 10° | 30° | 60° | 90° | 120° |
|---|---|---|---|---|---|---|---|---|
| CoDe | 0 | 0 | 0 | **35** | 35 | 35 | 35 | 35 |
| Belt 2R | 12 | 28 | 33 | **33** | 33 | 33 | 33 | 33 |
| 2R-DD | 0 | 15 | **17** | 20 | 31 | 31 | 31 | 26 |
| RCJ | 0 | 0 | **20** | 24 | 30 | 32 | 32 | 32 |

Bold marks the tilt used in Study 1. The 2R-DD and RCJ 90° references are
points of the same sweep: 31 and 32.

## 6. Interpretation
- **Remote actuation decouples orientation from accommodation.** CoDe's
  count does not change between 10° and 120°, and the belt's does not
  change between 0° and 120°.
- **CoDe has a hard limit at ≤ 0°.** Every pair fails there: the A′ servo
  body meets the linkage and the faces. Its feasible window is wide but
  one-sided.
- **The joint-servo designs vary with tilt.** Over the tilts where the
  reference is feasible, the RCJ varies by 12 pairs and the 2R-DD by 16. Their best
  tilts (30–90°) give 31–32 pairs, close to CoDe's 35 and the belt's 33.
- **Most of Study 1's gap is an orientation choice.** Study 1's large gap
  between CoDe and the laid-back 2R-DD and RCJ (17 and 20 pairs) is mostly
  an orientation choice. It is not intrinsic to those architectures. The
  standing orientations cost dorsal height (E01).

## 7. Critical review
- The tilt sweep holds every other coordinate. A real redesign would
  re-optimize them, so the joint-servo designs could gain more.
- The cliff at ≤ 0° for CoDe is specific to the prototype's A′ placement.
- **First-run artifact.** The first run changed only the vector's tilt, and
  every non-reference tilt was rejected by compose. That run was replaced
  (registry revision).

## 8. Follow-up
Not needed: the sweep is complete across the physical range.

## 9. Limitations
- One morphology.
- No dorsal-envelope limit is applied: a standing servo is allowed
  whatever its height.

## 10. Takeaway
Remote actuation buys **robustness of the interface verdict to actuator
choices**. This is a real decoupling, relevant to GOAL.md Section 3. But
it shrinks CoDe's Study 1 lead over the best-oriented joint-servo designs
to 3–4 pairs, and that should be stated whenever Study 1 is cited.
