# E02 — Polycentric ablation of the thick-pad fold cost

**Status: PARTIALLY_SUPPORTED.** CoDe's thick-pad fold advantage is real
against an ideal 2R pinned at its extended ICR. But the motion of the ICR is
not what produces it. What matters is that the **effective pivot sinks
toward the contact faces** during flexion. A fixed pin placed that close to
the faces reproduces the result.

## 1. Question
Does moving-ICR motion itself let thick pads fold further, or would a single
pin at the same place do as well?

## 2. Hypothesis and alternative
- **Hypothesis.** At equal thickness, CoDe reaches a larger φ_max than an
  ideal 2R pinned at CoDe's extended ICR (ICR0), and the gap grows with
  thickness.
- **Alternative.** The pivot's location, not its motion, decides. Then some
  fixed pin does as well.

## 3. Model and assumptions
- **CoDe.** The matched prototype, frozen at policy depths.
- **Ablation.** `revolute_idealized` (an existing mechanism: a pin and
  links, no joint servo) built to the same E, with its pin swept over
  ICR0 ± 8 mm.
- **Pad faces in the same place.** The ablation's pads are at its own
  policy depth (0) or at CoDe's depths (0 and 1.1 mm).
- **Modules.** 20 mm long, 1–12 mm thick, on both sides.
- **Optimism.** The ideal 2R has no actuator packaging, so it is
  optimistic.

## 4. Method
- φ_max, and feasibility at 90/120/150°, from each mechanism's evaluator.
- **E02b follow-up.**
  - Trace CoDe's ICR (the crossing of A′B and AB′) over 0–150°.
  - Counterfactual: raise the ideal 2R's pads toward its axis by h = 0–8 mm
    (negative drop), which is equivalent to moving the pivot h mm toward
    the faces.
  - The pin stays in the gap between the pads, so nothing interpenetrates.

Scripts: `e02_polycentric_ablation.py`, `e02b_palmar_pivot.py`. Figure:
`e02_polycentric_ablation.png`.

## 5. Results

| Thickness | CoDe φ_max | Ideal 2R at ICR0 φ_max (CoDe depths) | CoDe at 150° | Ideal 2R at ICR0, 150° | Ideal 2R, any pin offset (either depth variant), 150° |
|---|---|---|---|---|---|
| 1 mm | 176.2° | 173° | yes | yes | yes |
| 3 | 176.2 | 164 | yes | yes | yes |
| 5 | 176.2 | 155 | yes | no | yes (policy depth, pin 4–6 mm proximal) |
| 8 | 176.2 | 142 | yes | no | no |
| 10 | 164.0 | 134 | yes | no | no |
| 12 | 142.8 | 127 | no (120° only) | no | no |

- **Moving the pin proximally.** This helps only at 5 mm. With policy depths, a pin moved 4–6 mm proximally is feasible at 150°. That matches the proximal component of CoDe's ICR path. At 8 mm and above, no pin x-position on the link line is feasible at 150°: only lowering the pivot helps.
- **Moving the pin distally.** A pin 5–7 mm distal of ICR0 reaches
  φ_max = 180°. But the folded distal pad then hides part of the proximal
  face, so `pad_usable_mm` fails at 120°. φ_max alone is misleading;
  feasibility (ROM *and* usable pads) is the right metric.
- **CoDe's ICR path.** The ICR moves from (55.1, +5.0) mm at 0°, to (46.7,
  −4.2) at 90°, to (43.0, −5.8) at 120°. The virtual pivot sinks 4–6 mm
  below the link line, into the pad volume.
- **Counterfactual.** The palmar-pivot ideal 2R becomes feasible at 150°
  with:
  - 5 mm pads when moved 1 mm toward the faces;
  - 8 mm pads when moved 4 mm;
  - 10 mm pads when moved 6 mm;
  - 12 mm pads when moved 8 mm.

  This matches CoDe's capability (8 and 10 mm at 150°) at a pivot depth
  close to CoDe's own ICR depth at 90–120° (4.2–5.8 mm).

## 6. Interpretation
**Observation.** A fixed pin at CoDe's extended ICR loses about 4° of φ_max
per mm of pad thickness. CoDe loses nothing up to 8 mm.

**Supported explanation** (an ablation, a sufficiency counterfactual and a correlational replication in E10; not a full causal isolation). A pad pair folds without collision when the
rotation centre lies close to the contact surface. CoDe's crossed four-bar
places its virtual pivot 4–6 mm palmar of the link line at 90–120°. That is
inside the region the pads occupy, but with no hardware there. A physical
pin can reach the same place only if the axis and its hardware sit at the
contact surface.

**Not supported.** That *motion* of the ICR is required. A fixed pivot at
the right depth matches CoDe's 150° capability in this model.

**Caveats.**
- The counterfactual holds x at ICR0, while CoDe's ICR also moves about
  12 mm proximally. A joint (x, z) sweep of the fixed pin would make the
  depth explanation specific.
- The match is approximate. 10 mm pads need h = 6 mm, slightly beyond
  CoDe's deepest ICR (5.9 mm).
- φ_max differs: 162° against 176° at 8 mm.

## 7. Critical review
- **Excluded rows.** The same counterfactual on the physical 2R-DD and
  belt 2R appeared feasible. It is excluded: under `mount_policy:
  searched`, the evaluators do not check a pad against its own body's servo
  or pulley, so those raised pads overlap the joint servo or pulley.
  Physically, a joint servo of 9.4 mm radius or a 6 mm pulley on the axis
  cannot sit at the contact surface; a 3 mm pin can.
- **Is the palmar pin realizable?** The ideal 2R raise is a geometric
  construction. A real palmar pin needs remote actuation (a tendon or a
  linkage) and a gap in the module for the pin housing. That is a design
  option the comparison set does not contain.
- **Fairness.** The ideal 2R was given its best case: no servo, a free pin
  sweep, and both depth variants.

## 8. Follow-up
E02b, described above. It discriminates between "motion" and "palmar
pivot".

## 9. Limitations
- One morphology; 2D.
- Pads 20 mm long; only thickness was swept.
- CoDe's ICR path is design-specific (E10 looks at the archive).

## 10. Takeaway
CoDe's thick-interface advantage comes from a **virtual pivot that drops
into the contact-interface volume without hardware there**. A conventional
pin joint can only match it by putting its axis hardware at the contact
surface. This explains Study 1's high-flexion advantage mechanically. It
also corrects the "moving ICR" story: the motion is the means, and the
palmar location is the effect.
