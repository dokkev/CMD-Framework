# Exploratory studies E01–E10: registry

Registered 2026-11-07, before any E-study outcome data was generated.
Criterion: `GOAL.md` (the North Star and claim discipline). Brief:
`INSTRUCTION.md`. Later changes to a study are appended under
"Revisions", each with its reason. The original question stays as
registered.

## Evidence baseline (Phase A)

**Demonstrated (saved runs, current evaluators).**
- **Study 1, fixed morphology.** At 120°, CoDe carries 35 of the 71 catalog
  pairs, against 17–33 for the physical alternatives. At 150° it carries
  28, against 0–22.
- **Study 2, bounded adaptation.** CoDe finds no design for
  asym_long_proximal or asym_proximal_heavy, because its crossing links
  block the proximal face (shown by the face-rule sensitivity run).
- **Belt 2R "not found" results are a search-box artifact.** The relative
  box on `ap_from_a_x_mm` let the joint move only 2.45 mm distally.
- **Study 2 compact.** The belt is shortest on 7 of 8 targets; CoDe is
  shortest on long_thick.
- **Study 3.** The same contact law reflects differently to the output
  shafts; pad thickness is invisible to that model.
- **Study 4.** Negative: no polycentric reduction of bridge deformation.

**Hypotheses, not established.**
- Moving-ICR motion is the cause of CoDe's fold and coverage advantage. No
  ablation has isolated it.
- Remote actuation is the cause of the higher accommodation.

**Model assumptions.**
- 2D side view, Y layers assumed.
- Touching allowed, zero body clearance.
- Shared mounting policy, no spacer.
- Fixed actuation per reference.
- One matched morphology (90 mm prototype).
- Link lengths on a 0.1 mm grid.

**Missing evidence.**
- Robustness to tolerances and to collision inflation.
- Sensitivity to actuator orientation.
- Module independence.
- Accessibility across postures.
- A causal ablation of polycentric motion.
- The hidden transmission change that comes with adaptation.
- Which CoDe features drive capacity.

**Possible implementation issues to watch.**
- Search boxes, as in the belt case.
- Grid quantization.
- INVALID sentinels (100.0) read as real constraint values.

## Candidate hypotheses (Phase B)

| # | Candidate (falsifiable) | Category | Selected |
|---|---|---|---|
| C01 | Remote actuation leaves a larger share of the finger's side-view envelope, and more palmar depth under each segment, to contact modules than joint-mounted servos do | packaging | **E01** |
| C02 | CoDe's fold advantage with thick pads comes from moving-ICR motion, not from where its actuator sits (ICR-pinned ideal 2R ablation) | kinematics / causal | **E02** |
| C03 | CoDe's crossing links couple the proximal and distal modules: changing one changes whether the other is usable more than in 2R or RCJ | modularity | **E03** |
| C04 | Accommodation with a remote actuator does not depend on servo orientation; with a joint-mounted servo it depends strongly | actuation decoupling | **E04** |
| C05 | CoDe's tight clearances make its feasible assemblies more fragile under manufacturing perturbation | robustness | **E05** |
| C06 | CoDe's Study 1 lead disappears once collision geometry is inflated by a realistic clearance | model validity | **E06** |
| C07 | Own hardware occludes the contact faces less for CoDe across the whole posture range, not only at φ_req | accessibility | **E07** |
| C08 | Thick interfaces shrink the range of objects enclosed by two-face contact less for CoDe | contact geometry | **E08** |
| C09 | Morphology adaptation in CoDe carries a larger hidden change in transmission and kinematics than in 2R or RCJ, at equal ε_E | experimental controllability | **E09** |
| C10 | Within CoDe's design space, accommodation capacity is predicted by ICR travel (polycentric strength) more than by joint span or servo position | design-space structure | **E10** |
| C11 | Contact-normal directions available across flexion are architecture-independent at matched morphology | contact geometry | no: by construction, the normals depend only on φ |
| C12 | Force per unit servo torque varies less along the pad for CoDe | transmission | no: overlaps Study 3's Jacobian work |
| C13 | Thickness does not change the force mapping in any architecture | transmission | no: Study 3 already shows it analytically |
| C14 | Total dorsal–palmar finger thickness is larger for CoDe, because the A′ servo sits dorsally | packaging | folded into E01 as a secondary metric |
| C15 | Contact accessibility for oblique object approach directions differs by architecture | accessibility | no: needs a new ray model; E07 covers normal approach first |
| C16 | Interface swap without disassembly | modularity | no: needs 3D assembly; blocked by the abstraction |
| C17 | Neighbourhood size: the share of random ±5 mm E/Q perturbations that stay feasible | design space | no: same method as E05 at a different scale |
| C18 | Shortening \|AB\| reduces CoDe's fold capacity | kinematics | folded into E10 (\|AB\| as a predictor) |
| C19 | Graceful degradation as φ_req rises above 120° | robustness | no: Study 1 already covers 90/120/150° |
| C20 | Hyperextension headroom differs by architecture | kinematics | no: weak North Star relevance |
| C21 | Grid quantization of link lengths changes feasibility verdicts | methodology | no: covered as a check inside E05 |
| C22 | The fingertip pad's usable length near C differs by architecture | contact geometry | no: little relevance to interface variation |

**Why these ten.** They span packaging (E01), causal kinematics (E02),
modularity (E03), actuation (E04), robustness (E05, E06), accessibility
(E07), contact geometry (E08), controllability (E09) and design-space
structure (E10). Each has a null that is informative if CoDe loses. Each
reuses existing evaluators and saved data. Only E02 adds a configuration
(an ideal 2R pinned at CoDe's extended ICR), and that is an existing
mechanism.

## Common protocol

- **References.** Study 1/2 matched references (`shared.load_references`):
  - CoDe (tilt 10°);
  - 2R-DD at 0° and 90°;
  - belt 2R (tilt 10°);
  - RCJ at 0° and 90°;
  - ideal RCJ, an upper bound reported apart.
- **Shared rules.** All references share the comparison rules
  (`check_comparable`).
- **Ideal 2R.** `revolute_idealized` appears only as the E02 ablation and is
  labelled as such.
- **Modules.** The Study 1 catalog (`modules.yaml`). φ_req = 120° unless a
  study sweeps it.
- **Physics.** Each mechanism's own `evaluate()`. Solids and faces come from
  `core.morphology.contact.loss` scenes. Nothing is reimplemented.
- **Outputs.**
  - Results: `output/study/exploratory/eNN/` (manifest, CSV, source
    snapshot).
  - Code: `scripts/study/exploratory/`.
  - Reports: `report/exploratory/`.
  - Figures: `report/figures/exploratory/`.
- **Outcomes.** Each study ends SUPPORTED, PARTIALLY_SUPPORTED,
  NOT_SUPPORTED, INCONCLUSIVE or MODEL_OR_IMPLEMENTATION_BLOCKER.

## Selected studies (Phase C)

### E01 — Actuator-inclusive packaging share
- **Question.** At matched morphology and modules, how much of the finger's
  side-view envelope, and how much palmar depth under each segment, is left
  to contact modules rather than hardware?
- **Hypothesis.** Remote-actuated designs (CoDe, belt 2R) leave more palmar
  depth free along each segment than joint-servo designs (2R-DD, RCJ). The
  joint servo occupies the joint region.
- **Alternative.** Free depth is set by pins and links that every
  architecture has, so the differences are small. Or CoDe's dorsal A′ servo
  costs envelope elsewhere (dorsal thickness).
- **Controlled.** Reference morphology, module (baseline 20×3), extended pose.
- **Varies.** Architecture.
- **Metrics.**
  - Per segment: the length along which hardware stays above z = −d, for
    d = 3, 5, 8, 12 mm ("module-free length").
  - Dorsal–palmar thickness.
  - Pad-area share of the convex envelope.
- **Failure modes.** The envelope depends on Y layers that a side view cannot
  resolve. The extended pose alone misses motion; E07 covers motion.
- **North Star.** Section 2.1: space competition between hardware and
  interfaces.

### E02 — Polycentric ablation of the thick-pad fold cost
- **Question.** Does moving-ICR motion itself let thick pads fold further, or
  would a single pin at the same place do as well?
- **Hypothesis.** At equal thickness, CoDe reaches a larger φ_max than an
  ideal 2R pinned at CoDe's extended ICR, and the gap grows with thickness.
- **Alternative.** The advantage comes from where the pivot sits, or from
  actuator placement. Then the ICR-pinned ideal 2R matches CoDe, and sweeping
  the pin finds a position as good as CoDe.
- **Controlled.** Spans and finger length (matched E), module length 20 mm,
  mounting policy.
- **Varies.**
  - Module thickness: 1, 3, 5, 8, 10, 12 mm, both sides.
  - Ideal 2R pin x: CoDe's ICR0 ± 0–8 mm.
- **Metrics.** φ_max; feasibility at 90/120/150°; least pad clearance.
- **Comparison.** CoDe, ideal 2R at ICR0, ideal 2R at its best pin.
  2R-DD 90° and belt are context only.
- **Failure modes.**
  - The ideal 2R has no servo, so it is optimistic.
  - The ICR moves, so no single pin is "the" ICR. A pin sweep handles this.
- **North Star.** Explains why an articulation choice changes the feasible
  interface space (Section 3), rather than only observing it.

### E03 — Proximal–distal module independence
- **Question.** Is the usability of one module independent of the other
  module's size?
- **Hypothesis.** CoDe shows more cross-coupling than the 2R or the RCJ,
  because its crossing links pass near both faces.
- **Alternative.** Coupling is dominated by the fold (pad meets pad), which
  every architecture shares.
- **Data.** Study 1 rows (saved), plus a small 2D grid run: proximal length
  × distal length (15–30 mm), thickness 3 and 8 mm, at 120°.
- **Metrics.**
  - Interaction rate: the share of (proximal, distal) pairs whose joint
    verdict differs from the AND of the two one-sided verdicts.
  - The change in distal effective length when only the proximal module
    changes, and the reverse.
- **Failure modes.** Mounting failures dominate, so only mounted pairs are
  counted.
- **North Star.** Section 5.3: mechanically independent, replaceable
  modules.

### E04 — Actuator-orientation sensitivity
- **Question.** How much does the accommodation verdict depend on the
  actuator's orientation?
- **Hypothesis.** The pairs carried at 120° vary little with tilt for the
  remote-actuated designs (CoDe, belt 2R) and strongly for the joint-servo
  designs (2R-DD, RCJ).
- **Alternative.** Sensitivity is similar for all, because the servo
  envelope always sits next to a pad.
- **Controlled.** Morphology (matched E and Q), catalog, rules.
- **Varies.** `servo_tilt_deg` from −30° to 120° in steps of 10°.
- **Metrics.**
  - Pairs carried at 120° against tilt.
  - Range and standard deviation over tilt.
  - Tilts at which the bare reference is itself infeasible.
- **Failure modes.** A tilt can make the reference itself infeasible (servo
  against the MCP servo). This is recorded, not excluded.
- **North Star.** Section 3: actuator placement should not dictate the
  interface (decoupling).

### E05 — Manufacturing-tolerance robustness
- **Question.** Do feasible assemblies stay feasible under small geometric
  perturbations?
- **Hypothesis.** CoDe's feasible assemblies are more fragile than those of
  the 2R/RCJ, because of its tight least clearance (0.25 mm) and four-bar
  closure.
- **Alternative.** Fragility tracks each assembly's own margin, not its
  architecture.
- **Method.**
  - Gaussian perturbation of every geometric design variable (pin and joint
    coordinates, link lengths, depths), σ = 0.1, 0.2 and 0.4 mm. Tilt is
    perturbed with σ = 0.5°.
  - 200 draws per (architecture, module).
  - Modules: baseline, thick, asym_long_distal, at 120°.
  - Link resolution is set to 0.001 mm so that perturbations are not
    quantized away. This is documented.
- **Metric.** P(feasible | perturbed), with Wilson intervals over the draws.
  The draws are Monte Carlo samples of a model, not hardware replicates.
- **Failure modes.** A 2D model does not capture joint play.
- **North Star.** An interface must survive real assembly to be swappable
  (Section 5.5).

### E06 — Collision-geometry inflation
- **Question.** Does the Study 1 ranking survive a non-zero body clearance?
- **Hypothesis.** CoDe's lead shrinks faster than the others' as clearance
  grows, because its feasibility relies on near-touching solids.
- **Alternative.** All architectures lose pairs at similar rates.
- **Varies.** `packaging.body_clearance_mm` = 0, 0.25, 0.5, 1.0 and 1.5 mm,
  for all architectures.
- **Metric.** Pairs carried at 120° and 150° against clearance.
- **Failure modes.** Clearance enters each evaluator wherever the code uses
  it. These places are listed in the report.
- **North Star.** Claim discipline: is the Study 1 evidence robust to
  physical realism?

### E07 — Hardware occlusion across the posture range
- **Question.** Over 0–150°, how much of each contact face does the finger's
  own hardware block from an object approaching along the face normal?
- **Hypothesis.** CoDe's links block more than the other architectures at
  low flexion (they lie close to the faces), but less near 120° (they pass
  behind the pads). The order therefore depends on posture.
- **Alternative.** Occlusion is negligible for all except the fold, which is
  the same for all.
- **Metrics.**
  - Blocked share of each face by category (links, servo, pins, rollers,
    other pad), probe depths 5, 10 and 20 mm, poses every 5°.
  - Posture-averaged accessibility.
- **Controlled.** Baseline and thick modules.
- **Failure modes.** The probe is normal-only and 2D; out-of-plane routing
  is not resolved. A face counts as blocked whatever the Y layer.
- **North Star.** The contact surface must be reachable, not merely mounted.

### E08 — Two-face enclosing range with thick interfaces
- **Question.** For an object contacted by both pad faces, what range of
  object radii can each architecture enclose, and how does pad thickness
  shrink it?
- **Hypothesis.** Thickness shrinks the enclosable range less for CoDe,
  because its distal face translates as well as rotates.
- **Alternative.** The range is set by the pad spans and the flexion alone,
  so the architectures are equivalent at matched E.
- **Method.**
  - At poses 30–150°, take circles tangent to both face lines.
  - A radius is admissible if both tangency points lie on the usable faces
    and the circle clears every non-pad solid.
  - Report the admissible radius set over the poses.
- **Modules.** Baseline, thick, long, long_thick.
- **Failure modes.** This is geometry only. It makes no grasp-stability or
  force claim.
- **North Star.** Interface size versus useful interaction geometry.

### E09 — The hidden transmission change from adaptation
- **Question.** When a hand adapts its morphology to an interface (Study 2),
  how much do its transmission and kinematics change beyond ε_E and ε_Q?
- **Hypothesis.** At comparable ε_E, CoDe's adapted designs change their
  servo-torque-per-contact-force map and fingertip path more than the
  2R/RCJ, whose maps depend only on pivot geometry.
- **Alternative.** Changes scale with ε_E alike for all architectures.
- **Data.** Study 2 `summary_best.csv` final designs, against their
  references.
- **Metrics.**
  - Relative change in |Jᵀn| at the distal and proximal face centres
    (0–120°), using the Study 3 adapter.
  - Change in φ_max.
  - Fingertip path deviation.
- **Failure modes.** The Study 2 designs are best-found, not unique.
- **North Star.** Section 2.3: hidden embodiment confounds when the
  interface changes.

### E10 — What drives accommodation inside CoDe's design space
- **Question.** Among feasible CoDe designs, which geometric features predict
  accommodation capacity?
- **Hypothesis.** ICR travel over 0–120° predicts the pairs carried at 120°
  better than |AB|, segment spans or servo position do.
- **Alternative.** Capacity is explained by the spans (room for modules)
  alone.
- **Data.** A deterministic cover of about 150 designs from the archived
  linkage-v12 production fronts (`output/servo_close/linkage/proto_tilt10_align`,
  about 22k designs). Each design is rebuilt under the Study 1 rules (policy
  depths) and evaluated on the catalog at 120°.
- **Metrics.**
  - Spearman correlations.
  - Partial correlations controlling for the spans.
  - Pairs carried against each feature.
- **Failure modes.**
  - The archive was optimized under other objectives and versions, so it is
    a biased sample of the design space.
  - These are correlations; E02 supplies the causal test.
- **North Star.** Which feature of the architecture enables interface
  freedom (Section 3: "means, not ends").

## Revisions

Each entry is dated 2026-11-07 and gives the reason. No question was
changed.

- **E02.**
  - *Two pad-depth variants.* The ideal 2R's policy depth is 0, because it
    has no hardware, while CoDe's distal depth is 1.1 mm. The ideal 2R was
    therefore also run at CoDe's depths, so that the pad faces sit at the
    same place.
  - *Follow-up E02b (palmar pivot).* The pilot showed CoDe's ICR sinking
    below the link line during flexion, so a follow-up was added. It raises
    the ideal 2R's pads toward the axis (negative drop) as a counterfactual
    pivot placed closer to the contact faces.
  - *Physical 2R rows excluded.* The same counterfactual on the physical
    2R-DD and belt 2R is excluded from interpretation. With
    `mount_policy: searched`, the evaluators do not check a pad against its
    own body's servo or pulley, so those rows can interpenetrate. They stay
    in the CSV, flagged in the report.
- **E04.**
  - *Tilt set through the goal.* The first run varied only the design
    vector's tilt, and compose rejected every tilt but the reference's,
    because the tilt is the goal's actuation input (`pip_servo_tilt_rad`,
    with matching bounds).
  - *Rerun.* The rerun sets the tilt through the goal. It is an actuation
    field, excluded from the comparison rules, so nothing shared changes.
- **E05.**
  - *Module dropped.* asym_long_distal was dropped from the analysis. Its
    40 mm distal module mounts on no frozen reference (distal span 30.2 mm),
    so it tests mounting, not tolerance.
  - *Depths re-seated.* Depths are re-seated by the mounting policy after
    each draw, rather than perturbed. A symmetric perturbation of a 0 mm
    depth puts half the draws inside the segment, which is unphysical.
  - *Length spec widened.* The finger-length spec range was widened to
    60–120 mm. The reference sits exactly on its 90 mm upper bound, and a
    spec is not a physical check.
  - *Sentinels excluded.* The worst violated constraint is named without
    the evaluators' 100.0 INVALID sentinels. The first tally mislabelled
    static contacts as "pad_collision".
  - *Follow-up.* A one-variable-at-a-time follow-up locates each
    reference's exactly active static contact.
- **E06.**
  - *Policy and evaluator disagree.* The mounting policy does not apply
    `body_clearance_mm` where the evaluators do. The policy depth therefore
    reads as an unmountable INVALID design (for example, CoDe's distal
    support is short by 0.242 mm at 0.25 mm clearance).
  - *Seat search.* Each pad is lowered in 0.05 mm steps, by the same rule
    for every mechanism, until the evaluator's own seat checks pass.
  - *Variant `pins_exempt`.* Pin housings are shrunk by the clearance. This
    isolates the single static pin contact found in E05; the linkage sizes
    its pins with `a/ap/b/bp_pivot_diameter_mm`.
- **E07.** Two longer modules (combined_L30_T3 and combined_L30_T8) were
  added after the pilot. At 20 mm, no CoDe link ever stood in front of a
  face, so the registered prediction about links could not be tested
  without a module whose faces reach the link lines.
- **E08.** No method change. The metric proved non-discriminative (see its
  report).
- **E10.**
  - *|AB| cannot be tested.* In the archive cover, |AB| spans only 30.7–31.5
    mm: the production goal capped it at 0.35·|OC|. The registered |AB|
    comparison is therefore untestable with this archive. This is reported,
    not replaced.
  - *Follow-up E10b.* Bootstrap intervals were added.

## Final status

| ID | Status | One-line result |
|---|---|---|
| E01 | SUPPORTED, with a cost | Remote actuation keeps the palmar side clear (CoDe: only its 3 mm pins); the hardware moves dorsally |
| E02 | PARTIALLY_SUPPORTED | Real thick-pad fold advantage; the cause is the virtual pivot sinking 4–6 mm toward the faces, not ICR motion as such |
| E03 | NOT_SUPPORTED | CoDe's modules are nearly independent; the laid-back joint-servo designs couple most |
| E04 | SUPPORTED, qualified | Remote: orientation-independent over 110°; joint servos vary by 12–16 pairs and reach 31–32 at their best tilt |
| E05 | NOT_SUPPORTED | No motion fragility; each reference's fragility is one exactly active static contact (CoDe: A pin–PIP servo) |
| E06 | PARTIALLY_SUPPORTED | Uniform clearance collapses CoDe at 0.25 mm through that contact; with pins exempt, no loss to 1 mm |
| E07 | NOT_SUPPORTED as registered | CoDe has the lowest occlusion; joint-axis hardware occludes 60–88% of the distal face beyond 90° |
| E08 | INCONCLUSIVE | The metric re-measures E01's protrusion and saturates |
| E09 | NOT_SUPPORTED as CoDe-specific | Adaptation changes the distal force map in every architecture: median about 4–5× ε_E (CoDe, RCJ), 1–2.5× (2R-DD); small n |
| E10 | PARTIALLY_SUPPORTED | Span-controlled, ICR palmar drop predicts capacity (+0.44); ICR travel is weakly negative (−0.14); \|AB\| untestable |
