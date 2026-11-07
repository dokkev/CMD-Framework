# E05 — Manufacturing-tolerance robustness

**Status: NOT_SUPPORTED.**
- CoDe's four-bar motion is not fragile under 0.1–0.4 mm geometric errors.
- Feasibility under perturbation is limited by an **exactly touching static
  contact** that several references each have.
- In CoDe that contact is the A pin housing against the PIP servo housing.

## 1. Question
Do feasible assemblies stay feasible under small geometric perturbations?

## 2. Hypothesis and alternative
- **Hypothesis.** CoDe is more fragile, because of tight clearances and
  four-bar closure.
- **Alternative.** Fragility tracks each design's active constraints, not
  its architecture.

## 3. Model and assumptions
- **Perturbation.** Every free design variable gets Gaussian noise of
  σ = 0.1, 0.2 and 0.4 mm; the tilt gets σ = 0.5°.
- **Variables held.** Variables the formulation fixes, such as A and B on
  the link line, are not perturbed.
- **Pad seating.** After each draw, the mounting policy re-seats the pads.
- **Linkage grid.** The link-length grid is refined to 0.001 mm.
- **Length spec.** The finger-length spec is widened to 60–120 mm, because
  the reference sits exactly at 90 mm.
- **Draws.** 200 per (reference, module, σ), from a fixed seed. These are
  Monte Carlo samples of a model, not hardware replicates.
- **Modules.** Baseline and thick. asym_long_distal was dropped, because it
  mounts on no frozen reference.

## 4. Method
- P(feasible), with Wilson intervals.
- The first real violation is named with the INVALID sentinel (100)
  excluded.
- A one-variable-at-a-time scan (±0.1 mm) finds the active contact.

Script: `e05_tolerance.py`. Data: `results.csv`, `one_at_a_time.csv`.

## 5. Results (σ = 0.2 mm)

| Reference | Baseline | Thick | First real violation |
|---|---|---|---|
| CoDe | 0.61 | 0.55 | `pivot_pin_mm` (A pin housing vs PIP servo), every time |
| 2R-DD 0° | 0.61 | (infeasible nominally) | `pad_usable_mm` |
| 2R-DD 90° | 1.00 | 1.00 | — |
| Belt 2R | 1.00 | 1.00 | — |
| RCJ 0° | 0.37 | (infeasible nominally) | `pad_drop_mm` |
| RCJ 90° | 1.00 | 1.00 | — |
| *Ideal RCJ* | *1.00* | *1.00* | — |

- P changes little with σ (CoDe baseline: 0.66, 0.61 and 0.54 at σ = 0.1,
  0.2 and 0.4). That is the signature of a constraint that is exactly
  active, where the sign of a single variable decides.
- The one-at-a-time scan locates each contact:

  | Reference | Variable moved | Constraint violated |
  |---|---|---|
  | CoDe | `ap_z` −0.1 mm | `pivot_pin` |
  | 2R-DD 0° | `a_x` −0.1 mm | `pad_usable` |
  | RCJ 0° | roller radius or tilt −0.1 | `pad_drop` |

- **No motion failures in CoDe.** No CoDe draw failed on a motion-related
  check (pad collision, ROM, pad usability).

## 6. Interpretation
- The fragility comes from static packaging contacts that the 2D model
  allows to touch at zero clearance. It does not come from the crossed
  four-bar's sensitivity.
- CoDe's prototype has one such contact, between the A pin and the PIP
  servo housing. It must be given clearance in hardware.

## 7. Critical review
- **First tally mislabelled.** The first tally read the INVALID sentinel as
  "pad_collision". Corrected, and recorded as a revision.
- **Grid refinement.** Without refining the link grid, sub-0.1 mm noise
  would have been rounded away.
- **Not a manufacturing study.** Joint play, link compliance and
  out-of-plane errors are not modelled.

## 8. Follow-up
The one-at-a-time scan and the nominal margins (`nominal_margins.csv`).

## 9. Limitations
- 2D only.
- Independent Gaussian errors.
- One morphology.

## 10. Takeaway
The crossed four-bar is not a tolerance liability in this model. CoDe's
prototype instead carries one zero-clearance static contact (A pin to PIP
servo) that a build must open up. This is directly actionable for the
hardware now being assembled.
