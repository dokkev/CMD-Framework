# Bi-level hand design: finger length, finger, and thumb layout (prototype track)

Status: experimental, prototype track (scripts/optimize/prototype). Nothing here feeds the studies.

## Problem (user, 2026-11-26)

- **Lower level (finger).** For a finger length L_f, the prototype four-bar finger. It maximises C̄_pad (single objective) subject to MA_wc ≥ 0.25, q_req 150 deg, and the prototype's alignment, pad and collision rules (`config_bilevel.yaml`: `config_v1.yaml` with `ma_min: 0.25`; no CAD-fitted pivot or pad-gap bounds).
- **Upper level (hand).**
  - Variables: L_f ∈ [80, 105] mm, shared by the three fingers, together with the thumb layout (CMC x, z, case extension e, base yaw/pitch/roll, IP servo orientation).
  - Objectives: the functional Kapandji score and the thumb lever ratio. The finger's C̄_pad*(L_f) is reported for each L_f.
  - Constraints: the palm envelope.
  - The finger bases keep the prototype's 25 mm spacing.
- **Why it separates.** The lower level depends on L_f only, not on the thumb. It is solved once per L_f on a grid (80–105 mm, 2.5 mm steps: 11 lengths); the upper level then takes L_f from that grid. An exact nested bi-level would need about 800 upper evaluations × a lower run each.
- **Near-ties.** If several fingers of one L_f lie within 0.01 of the best C̄_pad, the upper level may choose among them (optimistic bi-level). This is used only if the near-tie set differs in Kapandji-relevant geometry (pad spans, pivot A).

## Pre-registration (before any lower-level run)

- **Lower level.**
  - Sweep: the `quick` preset (GA, 200 × 300, seed 0) over the 11 lengths, to see the shape of C̄_pad*(L_f); then the `search` preset (GA + NSGA-II, 200 × 1000, seeds 0–2) to settle each length.
  - Report: C̄_pad*, MA_wc and the main dimensions per length, plus the lengths with no feasible finger (the packaging floor).
  - Expectation, not a decision rule: C̄_pad* varies smoothly with L_f.
- **Upper level.**
  - Run: NSGA-II with L_f as a discrete variable on the grid, plus the thumb layout. Strict envelope, orientation ±20 deg, IP orientation chosen as before, population 32 × 25.
  - Hypothesis: the best F-level is at least K8 at some L_f (it is K8 at the prototype's 90 mm with O_thumbopt).
  - Decision rule: a length other than about 90 mm is recommended only if it raises the F-level, or keeps K8 with a lever ratio ≥ 0.05 higher, without a C̄_pad* loss larger than 0.02.
  - Reproduction check first: L_f 90 with the prototype finger and thumb gives F-K3; with O_thumbopt, F-K8.

## Amendments (before any upper-level run)

- **Lengths ±0.5 mm.** With L_f fixed exactly, 98 % of uniform draws fail the length bound, and no feasible design is found from scratch. Each length is a ±0.5 mm window (`bilevel/config_bilevel_L*.yaml`), as the prototype's own [89, 91].
- **Warm start.** Each length's feasible pool comes from `sample_seeds.py`, starting from the 90 mm prototype pool `output/linkage/config/samples/seeds.csv`.
- **Upper level, one run per length.** Instead of one run with L_f as a discrete variable, there is one thumb-layout run per length: `scripts/optimize/thumb_layout/run.py --hand <hand yaml>` (strict, proximal IP, orientation ±20, 32 × 25). This costs the same, and it gives the curve F*(L_f) directly.
- **Hand conversion.** `scripts/optimize/prototype/bilevel/make_hand.py` turns a lower-level finger into hand parameters. It uses the prototype palm, MCP axes and thumb, a 4 mm pad thickness (the config's pad height) and a 19 mm pad width. The hand model's fingertip agrees with the finger's own sweep (80 mm: 1.6e-14 mm).
- **Reproduction.** h0 with O_thumbopt gives F-K8, as before. The CAD `prototype.yaml` has no pads, so it cannot be scored.

## Lower level, quick preset (1 seed; a first look, not the result)

| L_f [mm] | 80 | 82.5 | 85 | 87.5 | 90 | 92.5 | 95 | 97.5 | 100 | 102.5 | 105 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C̄_pad* | 0.592 | 0.599 | 0.616 | 0.645 | 0.652 | 0.651 | 0.671 | 0.657 | 0.644 | 0.713 | 0.674 |
| MA_wc | 0.37 | 0.39 | 0.43 | 0.42 | 0.41 | 0.39 | 0.36 | 0.39 | 0.35 | 0.38 | 0.41 |

- C̄_pad* rises from 80 to about 90 mm and is noisy above that: one seed of 300 generations has not converged. The `search` preset settles each length.
- With the 80 mm finger, O_thumbopt (tuned on h0) gives only F-K3 (8/11). The thumb must be re-optimised per length, as planned.

## Lower level, search preset (stopped by the user at 2026-11-26 16:50)

GA + NSGA-II, 200 × 1000, seeds 0–2. The run stopped after 95 mm; the partial 97.5 mm run was discarded, and 97.5–105 mm were not run.

| L_f [mm] | 80 | 82.5 | 85 | 87.5 | 90 | 92.5 | 95 |
|---|---|---|---|---|---|---|---|
| C̄_pad* | 0.615 | 0.632 | 0.633 | 0.660 | 0.662 | 0.666 | 0.683 |
| MA_wc | 0.37 | 0.37 | 0.42 | 0.38 | 0.41 | 0.40 | 0.38 |

- C̄_pad* rises steadily with L_f over 80–95 mm, by about 0.07 in 15 mm. It does not level off in this range.
- MA_wc stays at 0.37–0.42, well above the 0.25 floor.
- The upper level was not run. The user moved the hand comparison to a task-agnostic, kinematics-only object-set grasp benchmark (`docs/experimental/grasp_set.md`). The hand files from this sweep can be fed to it.
