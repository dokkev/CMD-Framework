# CoDe Hand — Optimization Decision Points

*2026-10-22 · Dongho · branch `feature/2dof-fe` at `4b72258`, plus the uncommitted changes listed in §3*

The linkage optimization and the code for comparing mechanisms are working. Before the next production run and the comparison runs, five formulation choices need a decision. Each one changes which designs come out. A sixth item, the optimizer protocol, is reported for information.

## 1. Summary

| # | Decision | Recommendation | Blocks |
|---|---|---|---|
| D1 | Is the contact-interface size an objective or a requirement? | Keep it as a requirement for production and lengthen the pads afterwards. Use contact retention as the objective for the mechanism comparison. | Production v2 |
| D2 | Pad requirements: thickness, minimum lengths, chamfer | Confirm 10 mm thickness (mounting interface included) and the minimum lengths from the real module. Chamfer 2.5–5 mm. | Production v2 |
| D3 | What happens when the fingertip would curl behind the MCP? | End the usable range at C<sub>x</sub> = 0 (a flexion stopper) instead of rejecting the design. | Production v2 |
| D4 | Finger length ≤ 110 mm and BC ≥ 30 mm drive the results. Are they real requirements? | Fix them from the hand layout, or widen them for one exploratory run. | Production v2 |
| D5 | Mechanism comparison: module size, linkage evaluator, provisional choices | Use the same module thickness as D2, build the linkage retention evaluator, then run all mechanisms at full budget. | Comparison results |
| D6 | Optimizer protocol (for information) | Single NSGA-II while developing. A portfolio of 3 algorithms with independent seeds for final results. | — |

## 2. Current linkage formulation

Four-bar finger: MCP servo at O, PIP servo at A′ driving the coupler A′–B. Contact pads hang palmar of O–A″ (proximal) and B″–C (distal).

| Role | Item | Current setting |
|---|---|---|
| Objective | Transmission | Minimize k<sub>max</sub>, the worst ratio of PIP servo torque to the distal moment held, over the usable range |
| Objective | Curl | Minimize 180° − φ<sub>safe</sub>, where φ<sub>safe</sub> is the largest safe distal rotation relative to OA |
| Requirement | Flexion | φ<sub>safe</sub> ≥ 150° |
| Requirement | Pads | Proximal ≥ 30 mm, distal ≥ 40 mm, height 10 mm, 45° chamfer 2.5 mm |
| Requirement | Size | \|OC\| 80–110 mm, BC 30–50 mm, C<sub>x</sub> ≥ 0 at the end of the usable range |
| Requirement | Packaging | XM335 servos. Ø8 pivot housings with 1 mm clearance. A pin keeps clear of a Ø7 zone around A′. B′ pin may pass the PIP servo body (separate Y layer). Pads 1 mm from the MCP servo. PIP servo above the OA line |
| Input | PIP servo tilt | 20°, fixed. It no longer affects k<sub>max</sub> or φ<sub>safe</sub> |
| Free | A″ offset | Proximal pad may run past A′ by 0–20 mm. It has no effect on the objectives (see D1) |

## 3. Changes since the last review

All changes below are implemented and tested. Items marked "uncommitted" are not yet on the branch.

| Change | Effect |
|---|---|
| A pin checked against a Ø7 zone at A′ instead of the PIP servo body (uncommitted) | Never binding: A′<sub>z</sub> ≥ 9.5 mm already keeps \|AA′\| ≥ 9.5 mm against the 8.5 mm needed |
| B′ pin allowed through the PIP servo body (separate Y layer, uncommitted) | Main gain. Representative k<sub>max</sub> fell from 1.71 / 1.89 / 2.12 to 1.52 / 1.69 / 1.86 at the same φ<sub>safe</sub> |
| Servo tilt changed from a design variable to an input (uncommitted) | All 416 designs on the production v1 front keep k<sub>max</sub> and φ<sub>safe</sub> at 0°, 20° and 35° |
| A″ may lie past A′ (uncommitted) | The production v1 seeds take 4.6–16.8 mm more proximal pad for free (D1) |
| Pad height 5 → 10 mm (uncommitted) | Production v1 becomes infeasible: the balanced design collides at φ = 27°. Re-optimizing costs about 13 % of front hypervolume and raises k<sub>max</sub> by 0.05–0.11 (D2) |
| Mechanisms and optimizers made interchangeable, with portfolio runs (committed) | The same scripts optimize the linkage, serial 2R and RCJ with NSGA-II, SMS-EMOA or AGE-MOEA |

## 4. Decisions

### D1 — Contact interface: objective or requirement?

**Question.** Should the optimizer try to make the pads larger, or only keep them above a minimum?

**Evidence**

- On 2026-10-20 pad length was removed as an objective (run_044 vs run_045). As an objective it pushed finger length and BC to their bounds and traded linkage quality for pad. Without it, k<sub>max</sub> was equal or lower at the same curl, and the 5 mm pads still came out longer than the minimums.
- With 10 mm pads the requirement now binds: the distal pad comes out at 40.0–41.8 mm against a 40 mm minimum.
- Because pad size is not rewarded, the optimizer leaves A″ wherever it lands. Lengthening the proximal pad afterwards, as far as k<sub>max</sub> and φ<sub>safe</sub> stay unchanged with 0.5 mm extra clearance, adds 10.0 / 6.0 / 16.3 mm to the three production v1 seeds.

| Option | For | Against |
|---|---|---|
| A. Requirement + extend pads afterwards (current) | Proven; transmission and curl come first; pads lengthened at no cost | Pads sit near the minimums where these bind |
| B. Pad length as a third objective | One run shows the full trade-off | Earlier it drove the geometry to its bounds; a 3-D front is harder to choose from |
| C. Contact-retention objective, as in the mechanism comparison | Same measure for every mechanism; maximizes the contact interface directly | Needs a linkage retention evaluator; transmission must then be a constraint or a third objective |

> **Recommendation.** Use A for the production finger and C for the mechanism comparison.

### D2 — Pad requirements

**Question.** What thickness, minimum lengths and chamfer should the pads have?

**Evidence.** The 10 mm thickness includes the mounting interface, so the PIP servo may sit on the pad's mounting line. The table compares chamfers at 10 mm thickness and 20° tilt: one NSGA-II run each, verified on a 0.05° grid. Hypervolume uses the reference point k<sub>max</sub> = 2.3, φ = 150°. The 5 mm production v1 front scores 18.72 on the same measure.

| Chamfer (mm) | Hypervolume | Best k<sub>max</sub> | Balanced k<sub>max</sub> / φ |
|---:|---:|---:|---:|
| 2.5 | 16.10 | 1.577 | 1.79 / 170.5° |
| 5.0 | 16.22 | 1.558 | 1.81 / 170.7° |
| 7.5 | 16.26 | 1.560 | 1.73 / 166.0° |
| 10.0 | 14.93 | 1.570 | 1.80 / 168.8° |

- Chamfer gains of 1 % or less are within the run-to-run spread, about 2 %.
- Each millimetre of chamfer removes a millimetre of palmar contact face from each pad.
- The drop at 10 mm comes from the C<sub>x</sub> rule in D3, not from the chamfer itself.

> **Recommendation.** Confirm 10 mm. Take the minimum lengths (now 30 / 40 mm) from the actual contact module. Keep the chamfer at 2.5–5 mm.

### D3 — Fingertip curling behind the MCP

**Question.** If a design can curl so far that C passes behind O, should it be rejected, or should its usable range stop at that point?

**Evidence.** The rule is currently C<sub>x</sub> ≥ 0 at the end of the usable range. When the chamfer grew from 2.5 to 5 mm, 41 of 121 designs that had been feasible became infeasible. They could now curl further, and only the tip rule failed. A design that can curl further is penalized, although a flexion stopper at C<sub>x</sub> = 0 would make it at least as good.

> **Recommendation.** End the usable range at C<sub>x</sub> = 0, so curling further never makes a design worse. This is a small code change and needs a new formulation version.

### D4 — Bounds that drive the geometry

**Question.** Are finger length ≤ 110 mm and BC ≥ 30 mm design requirements, or provisional search limits?

**Evidence.** All three production v1 seeds sit on these limits: \|OC\| = 109.98 mm (0.016 mm from the limit), BC = 30.0 mm, and A′<sub>z</sub> = 9.5 mm (the servo touches the OA line). The fronts reflect these limits more than the mechanism.

> **Recommendation.** Set both from the hand layout and anthropometric target. If they are open, widen them in one exploratory run to see what the mechanism prefers.

### D5 — Mechanism comparison set-up

**Question.** Are the comparison measure and its provisional values right for the extended abstract?

**Implemented**

- **Measure.** Contact retention: the XZ area of a fixed nominal module that each segment keeps, collision-free, from extension to the required ROM, divided by the nominal area.
- **Objectives.** Proximal retention r<sub>p</sub> and distal retention r<sub>d</sub>, both maximized. The front's figure is R* = max min(r<sub>p</sub>, r<sub>d</sub>).
- **Required ROM.** 90°, 120°, 150° and 180°, each an independent study with ROM as a hard constraint.
- **Transmission.** Reported only: 1 for the 2R, 2 for the RCJ.
- **Mechanisms.** Idealized 2R, packaged 2R and packaged RCJ. RCJ results are marked provisional because cable routing is not modelled.

**Preliminary results (reduced budget, R\*)**

| Mechanism | 90° | 120° | 150° | 180° |
|---|---:|---:|---:|---:|
| Idealized 2R | 1.000 | 0.960 | 0.821 | unreachable |
| Packaged 2R | 0.828 | 0.844 | 0.764 | unreachable |
| Packaged RCJ (provisional) | 0.611 | 0.588 | 0.631 | 0.546 |
| Packaged CoDe linkage | no evaluator yet | | | |

> **Not yet conclusive.** R* should never rise with ROM. It does twice (packaged 2R at 90°→120°, RCJ at 120°→150°), so these runs, 100 individuals for 60 generations, had not converged. These values must not be quoted.

**Needs a decision**

- Nominal module: currently h = 5 mm and 45 / 45 mm. Should it follow D2 (10 mm)?
- Joint servo housing orientation in the 2R and RCJ: currently standing dorsally.
- RCJ rollers held on the mounting line (A<sub>z</sub> = 0, carrier angle 0). Raising them dorsally may favour the RCJ.
- RCJ cable allowance beyond the servo radius: currently 0.
- For the linkage: transmission as a constraint, or as a third objective?

> **Recommendation.** Use the D2 thickness for every mechanism, build the linkage retention evaluator, then run a full-budget portfolio for each mechanism and ROM.

### D6 — Optimizer protocol (for information)

- **Seed spread at 200 generations.** Across 30 runs (3 algorithms × 10 seeds), the seed's starting population mattered more than the algorithm. Median hypervolumes: NSGA-II 11.43, SMS-EMOA 11.15, AGE-MOEA 11.11. One seed failed for all three algorithms.
- **Mixing runs.** Merging 3–4 runs that differ in algorithm and seed reached ≥ 99 % of the 30-run merge.
- **Production v1 at 400 generations.** 15 runs, 80 000 evaluations each. The best single run reached 99.0 % of the merged front, so the search had converged.
- **Machining grid.** Rounding to the 0.1 mm grid cost nothing: hypervolume 18.72 with rounding vs 18.68 without.

Protocol: develop with a single NSGA-II run. Produce final results with a portfolio of 3 algorithms × 1 independent seed, re-verified on a fine grid and merged. To test whether the portfolio helps, compare it against 3 NSGA-II seeds at the same total budget.

## 5. Next steps once decided

1. Apply D1–D4. Run production v2: portfolio, 3 algorithms × 5 seeds, 400 generations, about 40 minutes. Then lengthen the pads and report the parameters.
2. Apply D5. Build the linkage retention evaluator, then run each mechanism at each required ROM with the full-budget portfolio.
3. Commit the changes listed in §3 as a reproducible baseline.

## Appendix — production v1, balanced design

This design is from the 5 mm-pad formulation and is superseded once the 10 mm pads are confirmed. It is listed as the reference for the current level of performance.

| Item | Value |
|---|---|
| A, A′, B′ − B (mm) | (41.7, 0), (48.5, 9.5), (−16.3, +10.4) |
| Coupler \|A′B\|, middle link \|AB′\|, BC (mm) | 32.9, 24.3, 30.0 |
| PIP servo tilt | 35° (input) |
| Pads, proximal + distal (mm) | 44.0 (A″ 6.0 past A′) + 46.3, height 5, chamfer 2.5 |
| k<sub>max</sub> / k<sub>min</sub>, φ<sub>safe</sub> | 1.685 / 1.348, 166.9° |
| Least pad gap, transmission margin | 1.50 mm, 30.3° |
