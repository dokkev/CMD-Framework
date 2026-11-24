# experimental_log.md — Studies 0–2

> PROVISIONAL, do not use in the paper: the fixed-finger first-contact angles (CFL 65° vs 2R-remote 92°) and the adjacent-thickness contact/skeleton changes (0.84 / 3.54 mm, CFL 3→5 mm at 120°) come from a 2026-11-20 partial run and are not re-checked against the final designs. The qualitative statements (no unchanged finger accepts a thicker pad; CFL touches earliest; the skeleton changes instead of the contact path) may be used without those numbers.
 (Study 3: nothing from it may be used in the paper — its design is in the playground; see idea.md H3)

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

## Study 2: redesign for thicker pads, relative contact trajectory as target

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
| `report/study0/figures/study0_pole_sketch.png` | pole and retreat distance: fixed axis 5 mm, 9 mm, lowered pole |
| `report/study2/figures/study2_principle_sketch.png` | principle of contact-centric redesign |
| `report/study1/figures/native_designs_paper_2026-11-20.png` | native designs per architecture and q_req |
| `report/study1/figures/study1_thickness_coverage_native.png` | Study 1 curves: C̄_pad and pad coverage vs h |
| `report/study2/figures/study2_redesign_2026-11-21.png` | Study 2: D_c vs h, coverage kept, path gap over flexion |
| `report/study2/figures/study2_pole_law_partial.png` | pole-law check |

## Study 2 supplementary (from `design/output/study/study2_redesign_2026-11-21/front.csv`, coverage end of each front)

Contact-face depth below the link line (proximal / distal, mm) at the coverage end, and MA_wc:

| Arch | q_req | 5 mm | 7 mm | 9 mm | MA_wc (5/7/9) | reference 3 mm MA_wc |
|---|---|---|---|---|---|---|
| CFL | 90° | 8/8 | 10/10 | 12/12 | 0.42 / 0.47 / 0.52 | 0.45 |
| CFL | 120° | 8/8 | 10/10 | 12/12 | 0.56 / 0.52 / 0.53 | 0.58 |
| CFL | 150° | 8/8 | 10/10 | 12/12 | 0.43 / 0.43 / 0.48 | 0.44 |
| 2R-remote | all | 5/5 | 7/7 | 9/9 | 1 (constant) | 1 |
| 2R-DD | 90° | 13.1/5 | 13.5/7 | 13.5/9 | 1 | 1 |
| 2R-DD | 120° | 8.3/5 | 7.9/7 | 9.0/9 | 1 | 1 |
| RCJ | 90° | 14.5/14.5 | 16.2/16.1 | 17.6/17.5 | 0.5 (constant) | 0.5 |
| RCJ | 120° | 13.5/13.5 | 14.2/14.7 | 18.5/9.0 | 0.5 | 0.5 |

Readings:
- CFL keeps its mount depth (face depth = 3 mm pivot-housing depth + h): Δd = 0, the faces move the full Δh
  away from the link line, yet D_c stays far below the fixed-axis bound — the pole moved with the faces. The
  bound plotted at Δd = 0 is therefore the like-for-like comparison for CFL and for 2R-remote (d = 0, cannot be
  reduced). 2R-DD's proximal pad sits under the joint actuator at a depth that barely changes with h (Δd ≈ Δh on
  that side), and RCJ remounts its pads (face depths change by less than Δh); that is how they go below the
  Δd = 0 line, consistent with Corollary 1's |Δh − Δd|.
- Mounting rule in Studies 1–2: each pad at the shallowest depth clear of its phalanx's joint hardware, within the
  support of its structure; spacers are excluded by requirement (SSoT 2026-11-21): they would keep D_c = 0 on any
  architecture but fill the interface's space with a passive block, cost every thinner interface the deep design's
  coverage (120°, 3 mm on the 9 mm design: CFL −8.6 points, 2R-remote −18.2) and fix the thickest interface in advance.
- Transmission: CFL redesigns keep MA_wc between 0.42 and 0.56 against 0.44–0.58 for their 3 mm references
  (−7 % to +16 %); 2R ≡ 1, RCJ ≡ 1/2 by construction. The redesign changes the actuator-side transmission
  modestly; it is part of the recorded embodiment change, compensated by control (SSoT decision 2026-11-21: flexion
  commands through θ(q) with K_θ = K_q/(dθ/dq)² hold flexion within 0.07° in simulation, torque limits off).
- Coverage alone does not keep the trajectory: Study 1's two independent 7 mm CFL designs at 120° change it by
  1.28 and 3.91 mm (Study 2 redesign: 0.86 mm).
- Least-change end of each CFL front: D_c·L_f = 0.33/0.67/2.97 mm (90°), 0.26/0.84/1.43 mm (120°),
  0.67/1.66/3.26 mm (150°) for 5/7/9 mm. 2R-remote's front collapses to the bound (least = most-coverage end).
- Search effort note: CFL received more search (continuation) than 2R/RCJ because its independent runs disagree;
  2R/RCJ runs agree cell by cell. Extra effort can only raise CFL; 2R-remote still leads at 90°. The 120° crossover
  (between 5 and 7 mm, 6 mm: 0.740 vs 0.729) is within the CFL run spread and is stated as approximate.
- 2R-remote actuator pose fixed because its objective is independent of it (MA ≡ 1, occlusion not counted).

## Study 3: does an interface measurement follow the contact-surface motion? (simulation, 2026-11-21/22)

Source: `report/study3/experiments.md` (experiments 3–8, pre-registration and amendments),
logs `sim/isaaclab/logs/study3/two_pad/` (dose_{A,B,C}_*.json, impedance.json), scripts
`sim/isaaclab/scripts/study3_measurement/two_pad_wrench.py`, `dose_analysis.py`, `impedance_analysis.py`.

Setup. Isaac Lab 3 + Newton 1.6.1 / MuJoCo-Warp (implicitfast, elliptic cone, impratio 1000, 2 substeps,
dt 1/120 s, contact stiffness ke 2.5e3 N/m per shape). Three fingers of the CoDe hand model, each a CFL design;
MCP and abduction held. A cylinder (ρ 15 or 20 mm, 70 mm long) free in the sagittal plane (x, z, rotation; gravity
off) is placed where O's two pads touch it at q0 (e17 placements: 33 per thickness, q0 every 5°), held by stiff
drives while the PIP closes to q0 + 5° (flexion stiffness K0 = 5 N·m/rad, scheduled through each hand's
transmission), then released. Gate 0: both pads of every finger ≥ 0.2 N, still, angle between the contact normals
= measured flexion within 2° (Proposition 4 holds in simulation to 0.25°). Load ramp 2 N/s on the cylinder in
+x (to the tip), +x−z, −z (off the face), −x (to the MCP) until slip (10 mm/s relative speed), contact loss or
5 mm collapse. Interface change: pad friction μ 1.3 → 2.0 (validated on O: |f_t|/|f_n| at slip = 1.300/2.000).
Effect Δ(H) = F_limit(μ 2.0) − F_limit(μ 1.3) per placement and direction; error E = |Δ(H) − Δ(O)|/|Δ(O)|,
median over valid pairs (O and H valid at both μ). Load step 0.067 N.

Hands: O (3 mm, Study 1 design); per thickness 3→5/7/9 mm the Study 2 most-coverage redesign R, Study 1's first
independent design U, and ε-constraint redesigns spreading D_c at near-best coverage (e18; chosen on design
metrics only so that D_c, D_s and C̄_pad are decorrelated). Naive swap (O's skeleton with thicker pads): pads
collide at 47° of flexion at 5, 7 and 9 mm → mechanically infeasible, not measured.

Condition A (same flexion command; primary). Over 15 hands that hold the cylinder (5/7/9 mm):
Spearman ρ(E, D_c) = +0.83, one-sided permutation p = 0.0001; per thickness +1.00 (n 3), +0.93 (n 8), +0.80 (n 4);
standardized fit E ~ +0.84 D_c − 0.45 D_s + 0.08 Δh; balanced 4 per thickness ρ = +0.86, p = 0.0009. Symmetric
|ln(Δ_H/Δ_O)|: ρ with D_c +0.69, with D_s −0.17 (the negative D_s sign of the relative error comes from its cap at
100 % for underestimates). R per thickness: E 3 % (5 mm, D_c 0.35), 21 % (7 mm, 0.86), 41 % (9 mm, 2.32); U: 52 %
(1.58), 124 % (3.91), 106 % (3.89). Same-D_c pair at 7 mm (D_c 2.3, D_s 4.6 vs 9.2 mm): E 72 % vs 70 %.
R (D_s 7.7 mm) vs a 7 mm hand with D_s 3.8 mm and D_c 3.1 mm: 21 % vs 92 %. R vs U paired (7 mm, 46 pairs): 45 of 46
R < U, Wilcoxon p = 4e-14. Mechanism: at the same command U presses 3.3× O's distal force (O 1.5–6 N) and the free
cylinder is pushed 2–3.5 mm toward the MCP; q* within 1° of O's. Hands with D_c ≥ 2.9 mm (3 of 18) could not hold
the cylinder under the command (ejected at release, also under torque reuse → geometric).

Condition B (oracle: distal force matched to O's using the simulator's contact force). 7 mm, 6 hands: E 2.8–6.8 %,
ρ(E, D_c) = +0.83, p = 0.029; most differences ≤ 3 load steps. U then grips at 4–7° less flexion (normals turn).

Condition C (O's PIP flexion torque reused, each hand through its own dθ/dq). 16 hands: E 3–40 %,
ρ(E, D_c) = +0.75, p = 0.0006 (coverage confound ρ(E, C̄_pad) = −0.52); distal force 0.89–1.26× O's.
7 mm R vs U paired: 16 % vs 20 %, p = 0.56.

Condition D (flexion impedance τ = τ_O + K_q(q*_O − q), K_q/K0 ∈ {0.1, 0.3, 1, 3, 10}; K0 reproduces A within
1.6 % force; O, R, U at 7 mm): E_R 25/14/20/24/26 %, E_U 44/87/119/125/131 %; R < U at every stiffness
(p = 0.007 at 0.1 K0, ≤ 4e-8 above). The gap grows up to K0 and levels off: U's finger yields 4.5° at 0.1 K0 and
0.1° at 10 K0 while its distal force saturates near 17.5 N (contact compliance takes the interference). Servo
torque median 0.08–0.33 of the XM335 stall torque; above stall for U in 4 % (3 K0) and 11 % (10 K0) of runs.

Robustness (condition A, O/R/U, ρ 15 mm, μ 1.3 → 2.0; R vs U pair by pair, one-sided Wilcoxon): baseline E_R 26 %,
E_U 170 %, 24/24, p 6e-8; contact stiffness ke 1e3: 24/168 %, 21/24, p 3e-4; ke 1e4: 32/127 %, 20/22, p 2e-4; solver
impratio 300: 28/132 %, 24/24; impratio 3000: 25/184 %, 24/24; substeps 4: 29/177 %, 24/24. Conclusion kept in all.

Shape interface (flat → 2 mm crowned pads, μ 1.3, condition A, ρ 15/20 mm). Gate on O passed in 4 of 4 directions
(O's effect −0.33 to −0.83 N; 4–5 valid pairs per direction). Six redesigns, E_shape (median, valid pairs): R 27 %
(13), d1p5_maxDs 46 % (13), d1p5_minDs 95 % (14), d2p25_minDs 50 % (1), d3_minDs 120 % (5), U 157 % (5).
ρ(E_shape, Δ_contact) = 0.94, exact one-sided p = 0.008 (pre-registered criterion met). R vs U: one common valid pair,
not testable. In this set Δ_skel moves opposite to Δ_contact (ρ(E, Δ_skel) = −0.89), so it repeats the ordering but
does not separate the two by itself.

Figures: `fig_overview.png` (Fig. 1: O/U/R grasp drawings + E vs D_c), `fig_study3.png` (A vs C, impedance),
renders `report/study3/figures/study3_two_pad_grasp_render.png`.

Pad compliance (contact stiffness 2.5e3 → 1e3 N/m, μ 1.3, condition A, ρ 15/20): over 14 holding hands
ρ(E_comp, Δ_contact) = 0.80 (hand bootstrap 95% [0.33, 0.99]), one-sided p = 0.0004; partial given Δ_skel 0.91
[0.68, 0.99]. R 25 %, U 173 % at 7 mm; 3/25/61 % for R at 5/7/9 mm.

Touch-referenced closure (condition E: each finger creeps until its own distal pad touches, then 5° further;
friction μ 1.3 → 2.0): all 18 hands hold; E 6–61 % (median about 20 %); ρ(E, Δ_contact) = 0.44 (p 0.034),
ρ(E, Δ_skel) = 0.88, partial of Δ_skel given Δ_contact 0.90. The reused-command error enters through the
over-closure that Δ_contact sets; the geometry to keep depends on what the experiment reuses.

Statistics added: friction (A) ρ 0.83, hand bootstrap 95% [0.41, 1.00]; partial E–Δ_contact | Δ_skel 0.88
[0.64, 1.00]. Study 1: CFL > 2R-DD in all 10 conditions without the CFL refinement (+0.5 to +14.3); with pad
length over fixed L_f, CFL > 2R-remote in every condition beyond 3 mm. Study 2: within 2 coverage points of the
front's end, 2R-DD and RCJ stay above the CFL's Δ_contact in every cell where they have a design.

Hand-independent placement grid (cylinder on each hand's proximal pad at x_c in a fixed grid, commands q0 + 5° with
q0 85–115°, friction μ 1.3 → 2.0, reused command): all 18 hands hold; ρ(E, Δ_contact) = 0.69 [0.24, 0.92], p 0.001;
partial given Δ_skel 0.68 [0.21, 0.92]; Δ_skel +0.18, coverage −0.10, thickness +0.13, transmission −0.37 (intervals
contain zero). Reused torque: Δ_contact 0.75 [0.38, 0.93], others contain zero; R about half of U's error at each
thickness (5/17/12 % vs 9/28/23 %).

Grid, expanded to 45 hands (27 more 3 → 7 mm redesigns, Δ_contact 1.0–4.1 mm × skeleton kept/minimized/maximized):
all hold; ρ(E, Δ_contact) = 0.60 [0.36, 0.76], permutation p < 1e-5; partial given Δ_skel 0.58 [0.31, 0.78].
Δ_skel +0.46 [0.17, 0.69], coverage −0.13, thickness +0.10, transmission −0.34 [−0.61, −0.01] (−0.19 given Δ_skel).
Difference ρ(Δ_contact) − |ρ(other)|: excludes zero for coverage [0.16, 0.65] and thickness [0.15, 0.67]; not for
Δ_skel [−0.12, 0.42] or transmission [−0.02, 0.55]. Route of the skeleton effect: Δ_skel vs share of placements lost
ρ 0.86, vs distal force ratio −0.83 (Δ_contact vs force ratio −0.05). Hands keeping ≥ 80% of placements (n 26):
Δ_contact 0.48 [0.10, 0.78], Δ_skel −0.04 [−0.45, 0.35]; stable for thresholds 15–40%.

Fixed-axis contrast family (one revolute PIP at the centre of O's joint gap, driven directly; links and axis kept
for 5/7/9 mm pads, Δ_skel = 0; pads end at the clearance gap 27.7/34.6/41.6 mm; Δ_contact = √3 Δh = 3.46/6.93/10.39
mm; placements where its 3 mm finger touches at q0; reused command, μ 1.3 → 2.0): E = 95% (IQR 68–118), 179%
(137–216), 228% (199–312); distal pad force 2.2/3.2/4.2× the 3 mm finger's; 54–56 valid pairs each.
Study 2 start offset (e19): removing |Δc(0)|, CFL's motion change is least in 7 of 9 cells; exceptions 2R-DD 90°/9 mm
(3.81 vs 4.21 mm) and RCJ 150°/5 mm (0.70 vs 0.87 mm), at 2.4 and 17.8 points less coverage.

## Study 3: force mediation (post hoc, 2026-11-22; `sim/isaaclab/scripts/study3_measurement/mediation.py`)

Reused command (A, 15 hands): rho(E, |N_d/N_d(O) - 1|) 0.94; rho(Delta_contact, N_d/N_d(O)) 0.62; partial
E-Delta_contact given the force deviation 0.55 [-0.02, 0.93]; partial E-force given Delta_contact 0.87 [0.35, 1.00].
Reused torque (C, 16 hands): forces 0.84-1.45x O's; partial E-Delta_contact given force 0.80 [0.37, 0.95]; partial
E-force given Delta_contact 0.66 [0.30, 0.90]. Candidate intercorrelations over 45 hands: Delta_contact with the
others |rho| <= 0.40; the largest pair is coverage-thickness (-0.67). CFL Study 1 optima: MA_wc 0.41-0.58.

## Study 3: pad stiffness falling with thickness (fifteenth experiment, pre-registered ed8d53a; 2026-11-22)

ke = 2.5e3 x 3/h N/m, condition A, O and 18 redesigns, 76 runs. Both pre-registered tests fail: rho(E, Delta_contact)
0.18, p 0.27 (n 14; rho(E, Delta_skel) 0.54); R vs U (same pads) E 26/53/32% vs 22/39/33% at 5/7/9 mm, Wilcoxon
R<U p 0.53/0.75/0.32. Signed Delta(H)-Delta(O): R -0.5/-1.0/-0.5 N, U +0.3/+0.7/+0.3 N; the softer pad lowers R's
force (0.43x O at 7 mm) and U's over-closure offsets it. The R-U gap remains (-0.8/-1.7/-0.8 N). With pad mechanics
changing with thickness, O's effect is no longer a reference; C2.3 is scoped to one contact model for every pad.

## Study 3: counterfactual reference for softer pads (seventeenth experiment, pre-registered 2d449e9; 2026-11-22)

O re-run with the contact stiffness of a 5, 7 or 9 mm pad (12 runs); the fifteenth experiment's redesigns compared
with O at the same pad stiffness. Valid (mu_eff 1.30/2.00, Delta(O_h) 1.0-1.4 N). rho(E', Delta_contact) 0.81
[0.33, 0.99], p 0.0003 (n 14); only Delta_contact's interval excludes zero. R vs U: 4.7 vs 54.9% (5 mm), 13.6 vs
128.6% (7 mm), 50.0 vs 124.0% (9 mm), Wilcoxon p <= 1.1e-7. Also: grid analysis key fixed (x_c in the key):
45-hand grid rho 0.86 [0.72, 0.94], Delta_skel 0.22 [-0.12, 0.51]; Delta_contact beats every candidate directly.

## Study 3: ideal reused torque on the grid, 27 new hands (eighteenth experiment, pre-registered c51e92d; 2026-11-22)

Condition C on the hand-independent grid for the 27 redesigns of the thirteenth experiment (`torque_grid_analysis.py`,
`torque_grid.json`). All 27 hold; force ratio 0.63-1.23; E 9-53% (median 20%). Pre-registered: partial
rho(E, Delta_contact | force deviation) 0.79 [0.54, 0.89] (holds); rho(E, Delta_contact) 0.69, p 4e-5 (holds).
Candidates: Delta_contact 0.69 [0.42, 0.83], Delta_skel 0.71 [0.41, 0.87], coverage -0.16, transmission -0.50
[-0.75, -0.09]. Post hoc: the force deviation follows Delta_skel (0.67), not Delta_contact (0.10); each predicts E
with the other held fixed (0.78, 0.79). Paper: Table I's torque column; C2.3 scoped to the impedance setpoint.

## Study 3, conclusion part, confirmation: soft 7 mm pads on the grid, 27 new hands (nineteenth experiment, pre-registered 6bc25cf; 2026-11-22)

`conclusion_analysis.py --set grid`, `conclusion_grid.json`. True change (O with the soft pad's stiffness) -37%.
Relative error vs Delta_contact: rho 0.94 [0.84, 0.97], p < 1e-5 (holds). Reversed share vs Delta_contact: rho -0.04
(fails). Measured change -4 to -75%. Post hoc: the sign of the distal face's normal offset over 85-115 deg sets the
direction (rho -0.99 with the signed error, -0.88 with the reversed share): an advancing face presses harder and
hides or reverses the loss (minDs family, up to 40% reversed; U), a retreating face exaggerates it. Paper: the last part of Study 3
folded into Study 3 as "the interface that forced the redesign"; abstract says the error *can* reverse a conclusion.

## Study 3: second engine, contact models and tasks (experiments 21–26, pre-registered; 2026-11-23; `sim/drake/`)

Drake (SAP) replica of the two-pad measurement on one planar finger, condition A, the same gates and load ramp.
- Point contact: ρ(E, Δ_contact) = 0.67 (p 0.003, 16 hands); R 19%, U 72%.
- Hydroelastic pads, 0.1 MPa over 3 mm on every hand: ρ 0.86 (p < 10⁻⁴), Δ_skel 0.09; R 17%, U 254%.
- Pads as soft as they are thick (0.1 MPa over each pad's own thickness), against O carrying the same pad (experiment 25): ρ(E′, Δ_contact) = 0.78 (p 0.0003, 15 hands; three hands with Δ_contact ≥ 4.4 mm cannot hold), Δ_skel −0.10. R 5.9, 16.3, 104.2%; U 109.4, 347.2, 370.4% at 5, 7, 9 mm. Below 1 mm of Δ_contact the errors match point contact; above it they are two to three times larger.
- Tasks under the same open-loop commands (point contact): grasp pull-out (the load ramp above), grasp acquisition (closing from open to draw a cylinder in against a load; ρ 0.87, Δ_skel 0.11, 18 hands), turning a disk with the fingertip alone (experiment 28z, replacing experiment 26: the distal pad drags the top face of a disk whose axis is normal to the proximal face, 12–38° against a torsional spring, the proximal pad never touching; ρ 0.78, p 0.0001, Δ_skel 0.20; R 17%, U 47%; noise floor 0.9%; pad pressed ≤ 1.5 mm). In-hand rolling by a flexion sweep fails (ρ 0.28).
- Figure: `fig_study3.png` (a) task frames rendered in Drake at simulated states, (b) Isaac Lab, (c) contact models, (d) tasks.

## Study 3: contact-referenced joint setpoint (experiment 27, pre-registered d34be24; 2026-11-24; `sim/drake/`)

Drake, point contact, condition A except the setpoint: each hand's flexion setpoint is the one at which the fingertip
point of its distal face, in its proximal-face frame, comes nearest to O's at O's setpoint (kinematics only, no
runtime contact; joint impedance control kept).
- Error falls on 15 of 16 hands scored under both (median 41.7% → 14.0%); R 13.8%, U 25.9%; noise floor median 0%, p75 3.7%.
- What remains follows Δ_contact (ρ 0.73, one-sided p 0.0007, 18 hands), not Δ_skel (0.18).
- Pad-midpoint reference instead: median 35.6%, ρ(Δ_contact) 0.81.
- Reading: re-referencing one point removes that point's preload difference; the other contact points' path
  difference remains, so the contact-surface motion must still be kept. Only sensed touch (condition E) moves the
  residual to the skeleton. Data: `report/study3/data/drake_e27_point.json`, `drake_e27m_point.json`.

## Study 3: the dial task replaced (2026-11-24; experiment 28, pre-registered 1b166a5; results 77b0c9d)

For the manuscript session: Fig. 4 and the text must change wherever experiment 26's dial is used.
- Why: in experiment 26 the dial was pressed between both pads and O's distal pad was pressed 2.0–11.0 mm (median 6.5 mm; 22 of 26 placements deeper than the 3 mm pad). The result lies outside the pad model. It also looked like the other grasp tasks.
- New dial task (28z): only the distal pad touches. It drags the top face of a 15 mm disk whose axis is normal to the proximal face, offset 8–12 mm sideways, against a torsional spring, as the finger sweeps 40° open-loop from O's touch angle. The proximal pad never touches (clearance ≥ 14.9 mm). Max pad press is 0.78–1.47 mm on every hand.
- Results, 18 hands: ρ(E, Δ_contact) 0.78 (p 0.0001), ρ(E, Δ_skel) 0.20; R 17.0%, U 46.8%; E 5.6–54.6%; noise floor median 0.9%.
- Auxiliary 28y (a y-axis dial pushed at its rim by the fingertip): ρ 0.55 (p 0.0095); R 6.5%, U 24.1%; E range near its noise floor (75th percentile 6.9%). Not the headline.
- Reading: the claim now covers a single-contact fingertip task, not only two-pad grasps.
- Data: report/study3/data/drake_e28z_point.json, drake_e28y_point.json. Log: report/study3/experiments.md, experiment 28. The renders for the new task are being made (sim/drake, render_tipdial*.png).
- Wording: the permutation p for 18 hands uses 20,000 random permutations, not exact enumeration. Say "permutation p", not "exact".

