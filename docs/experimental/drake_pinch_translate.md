# Drake: a tripod pinch translating the object toward the palm (experiment 45, experimental; not Study 3)

User request (2026-11-27): try three-finger-pinch tasks (candidate 2, after the tripod twist 44b). A Fig. 4 candidate; results stay out of report/ until the user decides.

## Pre-registration (2026-11-27; O calibrated, no redesigned hand run)

### Task (`sim/drake/pinch_translate.py`)

- **Setup:** the thumb holds one face of the object, the index and middle fingertips the other. The two fingers flex together and drag the object along the thumb toward the palm. By symmetry one planar finger models it.
- **Scene:** experiment 44's.
  - The thumb is frictionless, on experiment 33's soft squeeze spring.
  - The object's face is a sphere of r 20–40 mm, its spin held.
  - The object is tethered along O's pad tangent by a spring of 600 N/m. Without the tether, friction would not matter and D would be about 0.
- **Command:** close to O's q_s, release, then flex by sweep_deg (18–28°) over 2 s and hold 0.5 s.
- **Measure:** the object's travel along the tangent, from the sweep's start to the end. D = travel(μ 2.0) − travel(μ 1.3); E(H) = median |D(H) − D(O)| / |D(O)|.
- **Draws:** 64, seed 45 (`logs/ptrans_e45.json`).
- **Tuning, on O only:**
  - A stiff thumb (44b's) blocks the object along the normal. The flexing pad then presses the object (10 N, 4.5 mm penetration) instead of dragging it, and nothing slips. The thumb is therefore soft here.
  - Tether stiffness, from a 4-draw pilot on O: at 200 N/m the object never slips, and at 1000 N/m it always does. 600 N/m was chosen between them, so that μ 1.3 slips and μ 2.0 is near the drag limit.
- **Known risk:** the thumb's spring sets the squeeze again, as in experiments 43 and 44. The design can still enter through how far the pad carries the object before it slips (the contact path).

### O (tag e45) and the noise floor (tag e45n, +0.05°)

- O keeps 47 of 64 draws. The other 17 are excluded as |D| < 3% of the travel, all at μ 2.0 = μ 1.3 (the object slips at neither μ).
- D(O) is 0.25 / 1.61 / 3.21 mm, all positive. travel(O, μ 1.3) is 2.24 / 2.91 / 4.53 mm.
- Noise floor: E 0.65% (p75 1.77%).

### Hands and judgments

- As in experiment 44: the 18 hands of `analyze.DESIGN`, plus Skeleton, MO-Pad, S2F and pins-free Skeleton.
- **Judgments:**
  1. E_R < E_U.
  2. ρ(E, Δ_contact) > 0, p < 0.05.
  3. ρ(E, Δ_skel) is reported.
  4. Skeleton's E ≥ U's.
  5. MO-Pad's E is within 3 points of R's.
- **Stop rule:** R and U run first. If E_U < 10%, the run stops and is reported as not discriminating.

## Result (2026-11-27)

All 22 hands scored 47 of 47 pairs (`logs/e45_ptrans.json`).

| hand | E |
|---|---|
| R (CoDe now) | 9.1% |
| U (Coverage) | 39.6% |
| Skeleton (`conv_e25`) | 47.1% |
| MO-Pad (`mrmid_free`) | **40.5%** |
| S2F (`s2free_s2f`) | 53.6% |
| pins-free Skeleton (`conv_e25f`) | 21.3% |
| `h3to5_hr` | 3.1% |
| other 15 hands | 9.3–59.3% |

- **Judgments:**
  1. **Holds:** E_R 9.1% < E_U 39.6%.
  2. **Fails:** ρ(E, Δ_contact) 0.27, p 0.145, 18 hands.
  3. ρ(E, Δ_skel) is −0.05.
  4. **Holds:** Skeleton 47.1% ≥ U's 39.6%.
  5. **Fails:** MO-Pad 40.5% against R's 9.1%.
- Only the two R hands (`h3to5_hr`, R) and `d2p25_maxDs` are low. Most other hands sit at 33–59% whatever their Δ_contact, so the error does not follow it.
- MO-Pad errs like Coverage, and pins-free Skeleton errs less than MO-Pad. Pad depth enters here too: the soft thumb pushes the object onto each hand's pad, and the pad then drags it from a different place. This is not explained further.
- **Not a Fig. 4 candidate.** No hydro run.
