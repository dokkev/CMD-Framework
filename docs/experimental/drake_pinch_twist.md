# Drake: a thumb–index pinch twisted about its normal axis (experiment 43, experimental; not Study 3)

User request (2026-11-27): replace the cylinder twist (39, a three-finger power grasp) by a pinch, and draw the torsional
vector clearly. The goal is to make Fig. 4's columns differ in kind. Results stay out of report/ until the user decides.

## Pre-registration (2026-11-27; O calibrated, no redesigned hand run)

### Task (`sim/drake/pinch_twist.py`; experiment 33's pinch, `pinch_rotate.py`, imported read-only)

- The planar CoDe index under condition A, the MCP held. A sphere (r 8–12 mm) is pinched between the index's distal pad
  and a thumb pad that is the same on every hand. The thumb sits on a soft squeeze spring (40 N/m, preload
  0.4–0.8 N), so the thumb sets the squeeze.
- **Sequence:**
  1. The finger closes to O's q_s (30–55°) and holds.
  2. The sphere is released (no spin spring).
  3. A torque about the sphere's spin axis (y, normal to the finger's plane: the pinch's torsional load) rises at 0.01 N·m/s, cap 0.08 N·m.
  4. The ramp stops at the first of: turn > 5°, loss (either pad < LOSS_N for 3 samples), or the sphere moving 5 mm.
- **Measure:** τ_limit at that event, both directions as separate pairs. D = τ(μ 2.0) − τ(μ 1.3); E(H) = median |D(H) − D(O)| / |D(O)|.
- **Draws:** 64, seed 43 (`logs/ptwist_e43.json`; q_s, back 3–8 mm, r, preload). Time step 0.25 ms (experiment 33's).

### O (tag e43) and the noise floor (tag e43n, +0.05°)

- O keeps all 128 pairs. Nothing is excluded: solver, pre-ramp loss, penetration (max 0.7 mm), proximal contact and |D(O)| below 3 torque steps are all 0.
- D(O) is 5.0 / 8.3 / 13.0 mN·m (min/median/max), all positive. τ(O, μ 1.3) is 9.7 / 16.0 / 24.3 mN·m. Every event is slip.
- Noise floor: E 0.00% (p75 0.00%).
- One torque step (0.33 mN·m) is 4% of D(O)'s median, so E is resolved to about 4%.

### Hands and judgments (before any redesigned-hand run)

- **Hands:**
  - the 18 of `analyze.DESIGN` (the correlation);
  - outside it: Skeleton (`conv_e25`), MO-Pad (`mrmid_free`) and S2F (`s2free_s2f`).
- **Judgments:**
  1. E_R < E_U.
  2. ρ(E, Δ_contact) > 0, one-sided permutation p < 0.05 (20,000, seed 0), 18 hands.
  3. ρ(E, Δ_skel) is reported.
  4. Skeleton's E ≥ U's.
- **Known risk.** The thumb's spring sets the squeeze, so a redesign changes τ_limit only through where and how the
  distal pad meets the sphere. All E may be small, near the 4% resolution. If E_U < 10%, the task does not
  discriminate, and it is reported as such.
- **Render.** The pinch from the side, with a bold curved arrow about the spin axis and the axis vector itself
  (normal to the finger's plane), so the torsional load reads at a glance.
