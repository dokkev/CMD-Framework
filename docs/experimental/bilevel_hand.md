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
