# Outline (Stage 1)

Venue: RSS (≈8 pages + references; check this year's CfP). Scope: Studies 0–2, computational only.
Inputs: `raw_materials/idea_sparse.md`, `raw_materials/experimental_log.md`, `lit_review/`.

Working title: **Keeping the Contact Faces Still: Joint Architecture and Contact-Centric Kinematics
for Swappable Finger Contact Interfaces**
(alt: "Designing Fingers Around Their Contact Interfaces")

## One-paragraph story

Experiments on contact interfaces (pads, tactile modules) are confounded when the finger must be
redesigned to carry them. Whether that redesign is avoidable is a question of joint geometry. Using the
finite displacement pole, we show that a fixed-axis joint pays 2 tan(q_req/2) of pad length per mm of
interface thickness and must change the contact faces' relative motion by at least 2 sin(q_req/2)Δh under
any redesign. A crossed four-bar (CFL) moves its pole and escapes both. Per-architecture optimization
(4 physical + 2 idealized architectures × 4 thicknesses × 3 flexions) confirms the analysis, locates where
the four-bar wins and where it does not (90° flexion), and shows redesigns that keep contact-face motion
to 10–51 % of the fixed-axis bound without giving up coverage.

## Contributions (bullets in Intro)

1. A formulation that makes the contact interface the starting requirement of finger design, with a
   cylinder-based coverage objective measured on the posed contour.
2. A pole-based geometric account: exact clearance for any pole, a closed-form symmetric clearance law,
   closed-form coverage, and a contact-centric kinematics identity giving a redesign-independent lower
   bound for fixed axes.
3. A fair comparison of four finger joint architectures, each at its own optimum, across pad thickness and
   required flexion, validating the account (47/51 designs pad-contact-active; coverage reproduced to 1e−13).
4. Contact-centric redesign: the four-bar keeps the contact faces' motion within 10–51 % of the fixed-axis
   bound at the best coverage for the new interface; 2R-remote meets the bound exactly.

## Sections and paragraph plan

### I. Introduction (~1 p)
- P1 Contact interfaces are growing in size/function (vision-based tactile, compliant pads) → need volume
  near the joint. [cite tactile fingertips, hands]
- P2 On conventional fingers the joint actuator/housing competes; changing interface = redesign; in
  interface experiments this is a confound (intended vs unintended variation). [cite pad-property studies,
  embodiment/morphology learning]
- P3 Key insight: what an object/policy meets is the contact face; the quantity to hold is contact-centric
  kinematics, not skeleton kinematics. Whether it can be held is set by where the phalanx turns about (pole).
- P4 Contributions list; teaser figure ref (Fig. 1).
- Claim boundaries sentence: 2D, computational, hardware exchange not claimed.

### II. Related Work (~0.75 p)
- A. Tactile and compliant contact interfaces on fingers.
- B. Finger joint mechanisms: revolute (direct / remote), rolling contact, four-bar / polycentric.
- C. Computational design of fingers, grippers and linkages.
- D. Morphology and contact as variables in manipulation learning (confounding; cross-embodiment).
- Gap: no account of how joint geometry bounds interface thickness/flexion or keeps contact-face motion.

### III. Problem Formulation (~1 p)
- Morphology M = (K, C, A, S); requirements as bounds; posed length L_f(q).
- Feasibility F_static, F_motion; transmission angle (CFL).
- Coverage: cylinder rule, T = T1 ∩ T2, C_pad(q) = 2|T|/L_f(q), C̄_pad over Q_eval.
- Architectures: CFL, 2R-DD, 2R-remote, RCJ (+ideal upper bounds). Fairness: each at its own optimum,
  common goal (L_f = 90 mm, actuators, collision rules).
- Fig. 2: architectures sketch (native designs) — `native_designs_paper_2026-11-20.png` (a).

### IV. Geometry of the Joint–Pad Conflict (Study 0) (~1.5 p)
- Finite pole (Lemma 1), not the ICR (remark).
- Prop. 1 exact clearance (SAT, 4 axes) — states the design check.
- Prop. 2 symmetric clearance law G_sym; critical flexion q*; convexity in h, slope 2 tan(q*/2).
- Prop. 3 contact-face intersection X and closed-form coverage; Corollary: lowering the pole helps until τ=0.
- Prop. 4 contact-centric kinematics ‖Δc‖ = 2 sin(q/2)‖Δp‖; Corollary: fixed-axis lower bound (13).
- Architecture table: z(q), P_x(q), d (fixed axis / RCJ r tan(q/4) / CFL linkage-set).
- Hypotheses/predictions H1, P2–P4.
- Fig. 3: pole sketch — `study0_pole_sketch.png` (or panels 1–3 of `study2_principle_sketch.png`).

### V. Study 1: Coverage vs Thickness and Flexion (~1.25 p)
- Setup: grid h ∈ {3,5,7,9}, q_req ∈ {90,120,150}; GA+NSGA-II, 3 seeds, continuation for CFL; best of runs.
- Results: Fig. 4 (`study1_thickness_coverage_native.png`), Table I (C̄_pad grid, compact).
  - CFL loses least with thickness (0.085–0.096 vs 2R-remote 0.107–0.740).
  - Crossover: 2R-remote wins at 90°, CFL at 150°, q120 crossover ≈ 5–6 mm (confirmed at 6 mm).
  - RCJ low as hardware, high ideal → packaging.
- Study 0 checks: H1 47/51; coverage reproduced 1e−13; fixed-axis G slope; CFL q* inside range, G does
  not grow with q_req; fixed finger + thicker pad helps no architecture.

### VI. Study 2: Redesign that Keeps the Contact Faces' Motion (~1.25 p)
- D_c definition; bi-objective (min D_c, max C̄_pad); reference = 3 mm design.
- Results: Fig. 5 (`study2_redesign_2026-11-21.png`), Table II (D_c / C̄_pad, physical archs).
  - 2R-remote = bound exactly (q90, q120): Prop. 4 tight.
  - CFL 10–51 % of bound at Study-1 coverage (≤0.2 pp).
  - 2R-DD/RCJ below bound by mount-depth/rolling, above CFL.
  - Skeleton motion D_s changes instead: mechanism absorbs the interface.
  - Partial at 9 mm.

### VII. Discussion and Limitations (~0.5 p)
- What it means for interface experiments: hold contact-centric kinematics, report skeleton change.
- Where CFL does not win (90°, fixed finger, 9 mm); search not proof; 2D; equal-thickness pads; occlusion
  and MA not constrained; no hardware exchange shown; learning experiments are future work.

### VIII. Conclusion (~0.25 p)

## Figures / tables budget

| # | Content | Source | Status |
|---|---|---|---|
| Fig. 1 | Teaser: same pad thicker → fixed axis vs moving pole | new (compose from pole sketch + designs) | TODO |
| Fig. 2 | Architectures, native designs | (study PNG removed 2026-11-24; the paper figure is drawn by paper/figures) | regenerate in paper style |
| Fig. 3 | Pole and retreat distance | (study PNG removed 2026-11-24; the paper figure is drawn by paper/figures) | OK |
| Fig. 4 | Study 1 curves | (study PNG removed 2026-11-24; the paper figure is drawn by paper/figures) | drop suptitle |
| Fig. 5 | Study 2 redesign | (study PNG removed 2026-11-24; the paper figure is drawn by paper/figures) | drop suptitle, maybe 2 columns |
| Tab. I | Study 1 C̄_pad | experimental_log | in tex |
| Tab. II | Study 2 D_c / C̄_pad | experimental_log | in tex |

Not used: `report/figures/formulation_study.png` (old Study 1/2 definitions), panel 4 of
`study2_principle_sketch.png` (spacer approach, excluded by SSoT A.6.3).
