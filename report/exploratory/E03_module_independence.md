# E03 — Proximal–distal module independence

**Status: NOT_SUPPORTED.** CoDe is not more coupled than the alternatives.
The coupling depends on the architecture: it is largest for the laid-back
joint servos (2R-DD 0°, RCJ 0°), and zero for the 2R-DD at 90° and the belt.

## 1. Question
Does the size of one contact module change whether the other module is
usable?

## 2. Hypothesis and alternative
- **Hypothesis.** CoDe couples the two modules more than the 2R or the RCJ
  do, because its crossing links pass near both faces.
- **Alternative.** Coupling comes from the fold (pad meets pad), which every
  architecture shares.

## 3. Model and assumptions
- Study 1 `assembly` at 120°.
- Grid: proximal length × distal length ∈ {15, 20, 25, 30} mm, with the
  same thickness (3 or 8 mm) on both sides.
- Only pairs that mount, together with both of their one-sided
  counterparts, are counted.

## 4. Method
- **Interaction.** The verdict at (p, d) differs from verdict(p, 20) AND
  verdict(20, d).
- **Coupling.** The largest change in one side's effective length caused
  only by the other side's module.

Script: `e03_module_independence.py`. Data:
`output/study/exploratory/e03_module_independence/`.

## 5. Results

| Reference | Interactions (3 mm) | Interactions (8 mm) | Largest distal change from the proximal module | Largest proximal change from the distal module |
|---|---|---|---|---|
| CoDe | 0/16 | 1/16 | 3.5 mm (8 mm) | 1.5 mm (8 mm) |
| 2R-DD 0° | 2/12 | 2/12 | 11.0 mm | 0 |
| 2R-DD 90° | 0/12 | 0/12 | 0 | 0 |
| Belt 2R | 0/16 | 0/16 | 0 | 0 |
| RCJ 0° | 0/16 | 4/16 | 0 | 3.3 mm |
| RCJ 90° | 0/16 | 1/16 | 8.5 mm | 2.1 mm |
| *Ideal RCJ* | *0/16* | *0/16* | *0* | *0* |

## 6. Interpretation
- CoDe's modules are nearly independent: one interaction in 32 mounted
  grid pairs, and only with 8 mm pads.
- The strongest coupling is in the 2R-DD at 0°, where the proximal module
  changes the distal usable length by up to 11 mm. It also appears in the
  RCJ: roller and servo hardware sit at the joint, between the two faces.

## 7. Critical review
- The grid is small (4×4 per thickness) and uses one morphology.
- Usable lengths are read at the reached flexion for failed assemblies.
  This affects only the coupling magnitudes, not the interaction counts.
- The registered prediction was wrong in direction. This is reported as
  registered.

## 8. Follow-up
Not justified: the effect in CoDe is small and does not discriminate.

## 9. Limitations
2D only; the thickness is equal on both sides; one morphology.

## 10. Takeaway
Section 5.3 of GOAL.md (independent modules) is met by CoDe at the
prototype morphology, and by the belt 2R and the standing 2R-DD. The
laid-back joint-servo designs couple their modules. CoDe's crossing links
do **not** create cross-module coupling for modules up to 30 mm.
