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
