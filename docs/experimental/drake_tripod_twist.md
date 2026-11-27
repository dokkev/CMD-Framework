# Drake: a tripod pinch twisting a cylinder about its axis, the axis normal to the palm (experiment 44, experimental; not Study 3)

User request (2026-11-27): a three-finger pinch, the thumb against the index and middle fingers, the cylinder's axis
normal to the palm, twisted about that axis. It is a Fig. 4 candidate. Results stay out of report/ until the user decides.

## Pre-registration (2026-11-27; O calibrated, no redesigned hand run)

### Why one planar finger models it

- The thumb sits on one end face, on the twist axis. A point contact has no torsional friction, so the thumb holds no torque.
- The index and middle fingertips sit on the other end face, s = 10 mm on either side of the axis. They carry the torque as a couple.
- A twist θ moves each fingertip s·θ along its own distal pad, in its own flexion plane: one toward the fingertip, the other toward the MCP.
- Each finger's load is therefore in the plane the CoDe finger models. With F₊ and F₋ the finger's tangential load in each sense at travel x, τ(θ) = s·(F₊(sθ) + F₋(sθ)).

### Task (`sim/drake/tripod_twist.py`; experiment 33's pinch scene, `pinch_rotate.py`)

- Changes to the pinch scene, as options that leave experiments 33 and 43 unchanged:
  - the object's face is a large sphere, r 20–40 mm, so the end face is nearly flat;
  - its spin is held, so the cylinder does not tilt;
  - the thumb is frictionless;
  - the object is driven along O's pad tangent t by a spring (2 kN/m) whose rest point ramps at 1 mm/s to s·10°. The spring's force is the finger's tangential load.
- Each finger carries the draw's preload (0.4–0.8 N), so the thumb squeezes twice that.
- **Measure:**
  - τ_limit = max over θ ≤ 10° of τ(θ). A finger that loses contact holds nothing after that point.
  - D = τ(μ 2.0) − τ(μ 1.3) per draw.
  - E(H) = median |D(H) − D(O)| / |D(O)|.
- **Draws:** 64, seed 44 (`logs/tripod_e44.json`), the other ranges as in experiment 43. Time step 0.25 ms.

### O (tag e44) and the noise floor (tag e44n, +0.05°)

- O keeps all 64 draws. Nothing is excluded: solver failures, pad loss, penetration (max 0.32 mm), proximal contact and |D| below 3% of τ.
- D(O) is 5.3 / 8.6 / 11.0 mN·m (min/median/max), all positive. τ(O, μ 1.3) is 10.1 / 15.6 / 19.6 mN·m.
- Noise floor: E 0.47% (p75 1.13%). E is continuous here (the ramp is in displacement), unlike experiment 43's torque steps.
- At μ 1.3, 108 of 128 sense curves plateau (slip). At μ 2.0, only 60 do; the rest are still rising at 10°.
  - The limit then depends on the finger's tangential stiffness at the pad, i.e. on how the CFL moves the pad. That is where a redesign can differ.

### Hands and judgments (before any redesigned-hand run)

- **Hands:**
  - the 18 of `analyze.DESIGN` (the correlation);
  - outside it: Skeleton (`conv_e25`), MO-Pad (`mrmid_free`) and S2F (`s2free_s2f`).
- **Judgments:**
  1. E_R < E_U.
  2. ρ(E, Δ_contact) > 0, one-sided permutation p < 0.05 (20,000, seed 0), 18 hands.
  3. ρ(E, Δ_skel) is reported.
  4. Skeleton's E ≥ U's.
  5. MO-Pad's E is within 3 points of R's.
- **Discrimination rule.** If E_U < 10%, the task does not discriminate, as in experiment 43, and is reported as such.
- **Render.** Thumb, index and middle on a short cylinder whose axis is normal to the palm, with a bold arc about the axis and the torque vector along it.

## Experiment 44 stopped (2026-11-27)

Six of 21 hands had run when the run was stopped as hopeless (user: end hopeless runs quickly).

| hand | Δ_contact | E |
|---|---|---|
| R | 0.86 | 0.9% |
| d1p5_minDs | 1.60 | 1.2% |
| d2p25_minDs | 2.33 | 1.2% |
| d2p25_maxDs | 2.30 | 0.6% |
| d3_minDs | 3.09 | 1.0% |
| U | 3.91 | 4.5% |

- E_U is 4.5%, below the 10% discrimination rule, so the task does not discriminate. This is experiment 43's known risk.
- The cause: the thumb's spring sets the squeeze, so each finger's tangential limit is close to μN with N the same on every hand. The design enters only through the finger's tangential stiffness, a few percent.

## Pre-registration: 44b, the finger sets the squeeze (2026-11-27; O calibrated, no redesigned hand run)

- **Change from 44** (`tripod_twist.py run --finger_squeeze`): the grasp tasks' structure.
  - The thumb is held stiff at its pad: 1525 N/m, the real thumb's servo compliance along its pad normal (33b).
  - The finger closes to O's q_s + 3° (O's setpoint, reused on every hand), so the squeeze is the finger's: its pad path under the same command.
  - The drive is stiffer (20 kN/m), so the drive does not set the limit. Everything else is as in experiment 44 (draws `logs/tripod_e44.json`).
- **O (tag e44b) and the noise floor (tag e44bn, +0.05°):**
  - O keeps all 64 draws (penetration max 1.07 mm).
  - D(O) is 20.7 / 22.5 / 23.7 mN·m, all positive. τ(O, μ 1.3) is 31.4 / 35.6 / 38.0 mN·m.
  - Noise floor: E 1.03% (p75 1.06%).
  - On a three-draw check, both senses at both μ were friction-limited, with the normal force from the finger (1.3–1.5 N per finger).
- **Hands, judgments and discrimination rule:** as in experiment 44.
- **Order:** R and U run first. If E_U < 10%, the run stops there and is reported as not discriminating.

## Result: 44b (2026-11-27)

All 22 hands ran (`logs/e44b_tripod.json`).

| hand | Δ_contact | E | scored |
|---|---|---|---|
| R (CoDe now) | 0.86 | **2.1%** | 64/64 |
| U (Coverage) | 3.91 | 19.6% | 64/64 |
| Skeleton (`conv_e25`) | 6.61 | 27.3% | 63/64 |
| pins-free Skeleton (`conv_e25f`) | – | 2.6% | 64/64 |
| MO-Pad (`mrmid_free`) | – | — | 0/64: never touches |
| S2F (`s2free_s2f`) | – | — | 0/64: never touches |
| d1p5_maxDs / d3_maxDs / h3to9_d3p5_maxDs | 1.46 / 3.04 / 3.52 | 58.5% / — / — | 26 / 0 / 0 (over-pressed) |

- **Judgments:**
  1. **Holds:** E_R 2.1% < E_U 19.6%.
  2. **Holds:** ρ(E, Δ_contact) 0.56, p 0.014, on the 16 scorable hands of `analyze.DESIGN`.
  3. ρ(E, Δ_skel) is 0.42, p 0.054.
  4. **Holds:** Skeleton 27.3% ≥ U's 19.6%.
  5. **Fails:** MO-Pad cannot be scored.
- **Discrimination rule:** passed (E_U 19.6%).
- **MO-Pad and S2F never touch the object.** At release, the distal and thumb forces are 0 on all 64 draws, against 1.2–1.5 N on O and R.
  - The object is placed in each hand's proximal-face frame, and the stiff thumb holds it there.
  - These hands mount the proximal pad at 0, so their distal pad stands farther from the object. O's setpoint + 3° does not close that gap.
  - This is the protocol (a thumb held at a fixed place), not the pinch's mechanics: a real pinch closes the thumb until contact.
- Three maxDs hands over-press (penetration > 1.5 mm). The stiff thumb turns their extra pad motion into squeeze.
