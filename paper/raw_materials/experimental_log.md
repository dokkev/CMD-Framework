# experimental_log.md — Studies 0–2 (Study 3 is out of scope for this paper; ignore any learning results)

# Experimental log (Studies 0–2)

All numbers copied from the reports below; the reports cite the run outputs under
`design/output/study/` (not tracked). Re-check against them before submission.

- Study 0: `report/study0/derivation.md`, `report/study2/pole_law.md`, `report/study0/feasible_space.md`
- Study 1: `report/study1/study1_thickness_coverage.md`, `report/study1/study1_fixed_morphology_native_2026-11-20.md`
- Study 2: `report/study2/study2_redesign.md`

## Common setup

| Item | Value |
|---|---|
| Model | 2D side view, MCP fixed at q1 = 0, flexion q2 ∈ [0, q_req] |
| Finger length | L_f = 90 mm (both pads same thickness h) |
| Objective | C̄_pad: mean over Q_eval = {90°, (90°+q_req)/2, q_req} of 2|T1∩T2| / L_f(q) |
| Constraints | static assembly, collision-free motion to q_req (1° grid, re-verified 0.05°), CFL transmission angle γ ∈ [30°,150°]; no MA bound in studies (MA reported) |
| Collision tolerance | gap ≥ −1e−6 mm |
| Occlusion | links/belts on another lateral layer, not counted |
| MA | 2R-DD, 2R-remote ≡ 1; RCJ ≡ 1/2; CFL from |dθ/dq2| |
| 2R-remote PIP servo | fixed: A' = joint + (−10, +15) mm, tilt 15° |
| Optimizers | GA and NSGA-II, seeds 0–2, stop after 150 generations without improvement; CFL re-searched from neighbouring thicknesses (up and down) and from Study 2 designs; cell value = best of all runs |

## Study 0 checks (51 Study 1 designs finished at the time)

| Check | Result |
|---|---|
| H1: exact pad clearance min_q m(q) ≈ 0 at optimum | 47 / 51 within 0.1 mm; exceptions RCJ 120°/9 mm 1.68 mm, RCJ ideal 120°/5 mm 0.29, RCJ ideal 90°/7 mm 0.24, CFL 150°/5 mm 0.25 |
| G − G_sym (symmetric law as estimate) | fixed axis −3.4 … +0.2 mm (large only at 150°, pads not symmetric), CFL −1.6 … +0.2, RCJ ideal 0 … +0.7 |
| Prop. 1 implementation vs evaluator polygons | 400 random configs within 1e−9 |
| Prop. 3 coverage from pole + pads vs evaluator C̄_pad | all 51 within 1e−13 (after using structure end at pivot housing and the closed four-bar's distal link line) |
| Fixed finger + other thickness: predicted first contact vs evaluator q_max | within the 1° grid (99 cases, −0.25° … +2.1°) |
| RCJ pole closed form; CFL pole re-computed independently | unit tests; ≤ 0.13 mm |
| Fixed-axis gap growth per mm thickness | 2.00 / 3.46 / 7.46 mm at q_req 90 / 120 / 150° |

Earlier table (24 designs, 5 mm unless noted; G in mm, q* critical flexion, z pole depth at q_req):

| Design | G design | exact | q* | z(q_req) |
|---|---|---|---|---|
| CFL 90/5 | 15.5 | 15.5 | 87° | 0.2 |
| CFL 120/3 | 13.3 | 13.2 | 105° | 2.4 |
| CFL 120/5 | 13.8 | 13.7 | 99° | 4.4 |
| CFL 150/5 | 11.3 | 10.8 | 93° | 9.0 |
| 2R-remote 90/5 | 10.0 | 10.0 | 90° | 0 |
| 2R-remote 120/5 | 17.4 | 17.3 | 120° | 0 |
| 2R-remote 150/5 | 33.9 | 33.8 | 150° | 0 |
| RCJ 120/5 | 27.8 | 27.8 | 120° | 5.5 |

Key reading: fixed axis has q* = q_req, so G grows with tan(q_req/2) (2R-remote 5 mm: 10.0 → 17.4 → 33.9 mm);
CFL's pole descends with flexion so q* lies inside the range (87–105°) and G does not grow with q_req
(15.5 → 13.8 → 11.3 mm). Fixed finger + 2 mm thicker pad: CFL contacts earlier (65°) than 2R-remote (92°) —
no architecture accepts thicker pads without redesign.

Coverage trade (q_req 120°, 5 mm): CFL pad length 66.7 mm vs 2R-remote 63.1 mm, but C̄_pad 0.748 vs 0.758 at the
time, because a palmar pole lengthens the posed contour (+2 z sin(q/2) term) while a larger fixed-axis gap shortens
when flexed (−G(1−cos(q/2)) term).

## Study 1: best C̄_pad per architecture, thickness, q_req

| Arch | q_req | 3 mm | 5 mm | 7 mm | 9 mm |
|---|---|---|---|---|---|
| CFL | 90° | 0.802 | 0.778 | 0.750 | 0.717 |
| | 120° | 0.781 | 0.753 | 0.725 | 0.695 |
| | 150° | 0.770 | 0.739 | 0.707 | 0.674 |
| 2R-remote | 90° | 0.843 | 0.809 | 0.773 | 0.736 |
| | 120° | 0.815 | 0.758 | 0.697 | 0.633 |
| | 150° | 0.740 | 0.593 | 0.209† | 0.000† |
| 2R-DD | 90° | 0.749 | 0.726 | 0.708 | 0.693 |
| | 120° | 0.717 | 0.700 | 0.682 | 0.633 |
| | 150° | 0.710 | 0.593 | none found | none found |
| RCJ | 90° | 0.698 | 0.663 | 0.634 | 0.606 |
| | 120° | 0.644 | 0.608 | 0.579 | 0.555 |
| | 150° | 0.607 | 0.560 | 0.514 | none found |
| CFL ideal* | 90° | 0.803 | 0.779 | 0.756 | 0.733 |
| | 120° | 0.788 | 0.761 | 0.733 | 0.704 |
| | 150° | 0.779 | 0.746 | 0.713 | 0.679 |
| RCJ ideal* | 90° | 0.803 | 0.787 | 0.775 | 0.745 |
| | 120° | 0.790 | 0.781 | 0.738 | 0.694 |
| | 150° | 0.788 | 0.755 | 0.696 | 0.636 |

\* no joint-servo body (upper bound). † feasible but pads cannot meet a cylinder.

Loss 3 → 9 mm: CFL 0.085 / 0.086 / 0.096; 2R-remote 0.107 / 0.182 / 0.740 (q_req 90 / 120 / 150°).
q_req 90 → 150° at 5 mm: CFL −0.039, 2R-remote −0.216.
Crossover CFL vs 2R-remote: q90 2R-remote higher at all h (gap 4.1/3.1/2.3/1.9 pp); q150 CFL higher at all h;
q120 crossover ≈ 5.3 mm by interpolation, confirmed at 6 mm (2R-remote 0.729, CFL 0.740).
RCJ lowest as hardware, RCJ ideal high → loss is packaging (roller + servo), not kinematics.
Convergence: 2R and RCJ independent runs reach identical values (RCJ q150 7 mm 0.9 pp spread); CFL runs spread
1.3–3.4 pp, continuation raised up to 2.1 pp. All CFL 5/7/9 cells ≥ Study 2's best at that thickness. Not a
proof of global optimum.

Final Study 1 designs at q_req 120°, 5 mm (summary.csv): CFL pads 34.4 + 34.4 mm, pad coverage 0.764, C̄_pad 0.753; 2R-remote 31.6 + 31.5 mm, pad coverage 0.701, C̄_pad 0.758.

Native 5 mm designs from the earlier run (pad coverage (ℓ_p+ℓ_d)/L_f): q90 CFL 0.722, 2R-remote 0.783; q120 CFL 0.741, 2R-remote 0.701;
q150 CFL 0.769, 2R-remote 0.518 (distal pad at 25 mm minimum). CFL q_max reaches 180° at q120/q150 designs.

Feasible-space note (CFL, q120, 7 mm): 25 / 80 000 uniform samples feasible (0.03 %); the binding constraint is
pad–pad collision during flexion (62 % of first collisions), consistent with H1.

## Study 2: redesign for thicker pads, contact kinematics as target

Reference: each architecture's 3 mm Study 1 design. NSGA-II 200 × 200, seeds 0–1 per thickness. Objectives:
min D_c, max C̄_pad. Cells: D_c·L_f (mm) / C̄_pad at the coverage end of the front.

| Arch | q_req | 3→5 | 3→7 | 3→9 |
|---|---|---|---|---|
| fixed-axis bound | 90° | 2.83 | 5.66 | 8.49 |
| | 120° | 3.46 | 6.93 | 10.39 |
| | 150° | 3.86 | 7.73 | 11.59 |
| CFL | 90° | 1.22 / 0.776 | 1.84 / 0.749 | 4.31 / 0.717 |
| | 120° | 0.35 / 0.753 | 0.86 / 0.725 | 2.32 / 0.695 |
| | 150° | 0.77 / 0.738 | 1.71 / 0.707 | 3.65 / 0.673 |
| 2R-remote | 90° | 2.83 / 0.809 | 5.66 / 0.773 | 8.49 / 0.736 |
| | 120° | 3.46 / 0.758 | 6.93 / 0.697 | 10.39 / 0.633 |
| | 150° | 3.86 / 0.593 | 13.77 / 0.209† | 22.86 / 0.000† |
| 2R-DD | 90° | 2.12 / 0.726 | 3.82 / 0.708 | 5.23 / 0.693 |
| | 120° | 2.00 / 0.700 | 4.40 / 0.682 | 5.86 / 0.633 |
| | 150° | 3.00 / 0.593 | none | none |
| RCJ | 90° | 2.83 / 0.663 | 5.16 / 0.634 | 7.14 / 0.606 |
| | 120° | 2.08 / 0.608 | 3.72 / 0.579 | 9.30 / 0.555 |
| | 150° | 1.90 / 0.560 | 7.10 / 0.514 | none |
| CFL ideal* | 90° | 0.46 / 0.779 | 5.70 / 0.756 | 3.10 / 0.731 |
| | 120° | 0.52 / 0.760 | 1.54 / 0.733 | 2.20 / 0.703 |
| | 150° | 1.40 / 0.741 | 2.23 / 0.712 | 4.76 / 0.679 |
| RCJ ideal* | 90° | 2.80 / 0.787 | 1.91 / 0.775 | 4.33 / 0.745 |
| | 120° | 0.35 / 0.781 | 2.81 / 0.738 | 5.07 / 0.694 |
| | 150° | 1.28 / 0.755 | 3.20 / 0.696 | 5.20 / 0.636 |

Readings:
- CFL has the smallest contact-path change among the four physical architectures at every q_req and thickness:
  0.35–4.31 mm = 10–51 % of the fixed-axis bound (q120 10–22 %, q150 20–31 %, q90 32–51 %), with C̄_pad within
  0.2 pp of final Study 1 best (no coverage traded).
- 2R-remote equals the bound exactly at q90/q120 (Prop. 4 bound is tight). q150 7/9 mm: degenerate designs.
- 2R-DD and RCJ go below the bound (joint x can move; mount depth under joint hardware reduced; RCJ pole rolls)
  but stay above CFL.
- Change grows with thickness; CFL at 9 mm 2.3–4.3 mm: partial, not full preservation (2–10× smaller than fixed axis).
- Skeleton (fingertip) motion D_s changes instead for CFL (e.g. q120 3→5: contact 0.84 vs skeleton 3.54 mm in the
  adjacent-thickness Study 1 comparison), i.e. the mechanism absorbs the interface.
- CFL ideal q90 7 mm: coverage-end design changed a lot (5.70 mm); other front points ≤ 0.65 mm.

## Figures available (PNG)

| File | Content |
|---|---|
| `report/figures/formulation_study.png` | study formulation sketch |
