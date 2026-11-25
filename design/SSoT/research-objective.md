# CoDe Hand — Research Objective

Experiment design: the principles, what each study must show, the discipline claims are held to, and the paper plan with each contribution's status (Section 9). The problem, what CoDe solves and the contributions are in [north-star.md](north-star.md), which takes precedence; the mathematics is in [problem_formulation.md](problem_formulation.md).

The SSoT has four files, each with one job. When they disagree, the higher one wins and the lower one is fixed. `north-star.md` refers to no other file.

| File | Job |
|---|---|
| `north-star.md` | The paper's story: the problem, what CoDe solves, the contributions claimed |
| `research-objective.md` (this file) | Experiment design: principles, what each study must show, claim discipline, status and paper plan |
| `problem_formulation.md` | Mathematics: definitions, variables and their names, each study's formulation |
| `code_formulation.md` | Implementation: how the formulation is written as code |

## 1. Purpose

CoDe is meant to be a research platform on which contact-interface experiments are possible and interpretable: researchers can study diverse contact morphologies if they wish, and can change only the contact interface while how its contact surfaces move relative to each other is kept. The problem it addresses, its approach and the contributions claimed are in [north-star.md](north-star.md); this file states how the studies test them.

It is not an attempt to build a hand with more degrees of freedom, higher grasping strength, or universally better manipulation performance.

---

## 2. Intended and Unintended Variation

When a manipulation experiment compares two contact interfaces, the observed change has two kinds of source:

- **Intended variation** comes from the contact interface itself: what happens on the contact face. Its geometry and profile, the contact's location, distribution and normals on the face, compliance, friction, tactile observations and interface-dependent force transmission. The contact-point Jacobian changes that follow from these (a contact elsewhere on the face, a different normal) are part of the effect under study.
- **Unintended embodiment variation** comes from how the hand was adapted to the interface: changes in finger length, joint location, linkage geometry, actuation or transmission, and above all a displacement of the contact faces relative to the articulation. A thicker interface on an unchanged joint moves its contact face away from the point the phalanx turns about, which changes how the contact surfaces move relative to each other, that is the finger's contact Jacobian. That change comes from the hand's geometry, not from the interface, and is what a redesign should remove.

The dividing line: a change on the contact face is the interface's; a change of where the contact face is, and how it moves, relative to the other contact face is the embodiment's. CoDe does not aim to remove the first. It aims to keep the second small, and to record what does change (Section 5.2), so that the first can be studied. The objective is to reduce embodiment confounding, not to claim perfect causal isolation.

---

## 3. Problem and Approach

See [north-star.md](north-star.md). Mechanisms are engineering means toward the research objective, not ends, and no mechanism is assumed superior without comparative evidence.

---

## 4. Supporting Questions

1. How does hand architecture constrain the available space and geometry of contact interfaces? (C1)
2. How much must the underlying hand change to accommodate different contact interfaces, and how much does its contact-surface motion change? (C2.1, C2.2)
3. How do changes in contact-interface design influence physical interaction, tactile observations, and ultimately manipulation behavior? (C2.3 answers the part this paper can: what a redesign does to a measured interface effect.)

The third question is the long-term research objective. The first two establish the hardware and experimental foundation required to investigate it.

---

## 5. Design Principles

### 5.1 Contact Interface First

Contact-interface requirements should influence the hand architecture from the beginning.

Do not treat contact interfaces as an afterthought that must fit into whatever space remains after actuator and joint placement.

### 5.2 Preserve the Underlying Embodiment

When comparing contact interfaces, maintain a common underlying hand morphology, actuation architecture, and kinematic configuration whenever practical.

Changes required to accommodate an interface must be recorded separately from the interface itself.

The objective is to reduce embodiment confounding, not to claim perfect causal isolation.

### 5.3 Modular and Mechanically Independent Interfaces

Proximal and distal contact interfaces should be independently configurable and replaceable.

Changing an interface should not unnecessarily require redesigning or disassembling the underlying articulation mechanism.

### 5.4 Functional Contact Structures

A contact interface is not limited to a passive friction pad.

It may include:

- Compliant structures.
- Tactile sensing modules.
- Sensor-integrated surfaces.
- Specialized contact geometries.
- Structures that influence contact-force transmission and deformation.

The platform should accommodate the physical requirements of these functional interfaces.

### 5.5 Experimental Accessibility

The hand should make controlled comparisons practical.

Researchers should be able to change contact-interface properties while keeping the underlying mechanical embodiment sufficiently consistent to interpret the resulting differences.

### 5.6 Operational Definition of Embodiment Invariance

CoDe aims to enable contact-interface variation while preserving the underlying robotic hand embodiment. In the studies' notation, the underlying hand H = (E, Q, A) is fixed and the contact interface C is variable. E is the structural morphology, Q the articulation geometry and A the actuation configuration.

For controlled contact-interface experiments, the following should remain fixed whenever possible:

- How the contact surfaces move relative to each other: in the proximal-phalanx frame, the path of the point under the fingertip on the distal contact surface over flexion and its derivative, the PIP column of the finger's contact Jacobian (its change is Δ_contact in mm, `problem_formulation.md` A.6.3). It excludes the transmission from servo angle to flexion, which is recorded separately. This is the kinematic target of a contact-centric hand; what a policy and an object meet is the contact surface, not the skeleton.
- Structural finger morphology and link geometry.
- Articulation kinematics and joint configuration.
- Actuator placement and transmission architecture.
- Contact-module mounting references.

When these cannot all be kept (an interface thicker than the finger was designed for asks for a redesign), the contact-surface motion has priority over the skeleton's: the mechanism's internal dimensions may change to keep the contact faces moving as before (`problem_formulation.md` A.6.0, A.6.3).

The following are intentionally variable:

- Contact-interface geometry and dimensions.
- Compliance, material properties and friction.
- Tactile sensing configuration.
- Contact location, distribution and surface response.

Preserving the contact-surface motion does not imply preserving contact mechanics or actuator-side contact responses: compliance, friction, sensing and the contact's location on the face change with the interface, which is precisely what the platform is designed to investigate. What should not change with them is where the contact faces are and how they move relative to each other (Section 2). Changes that a redesign does make (transmission from servo angle to flexion, mechanical advantage, the skeleton's motion) are recorded with the result.

The objective is to isolate contact-interface variation from unnecessary changes to the underlying hand embodiment, not to make contact behavior invariant.

### 5.7 Criteria for a Contact-Centric Research Platform

A successful contact-centric research platform should:

1. **Accommodate diverse interfaces.** Support meaningful variation in contact geometry, thickness, compliance and sensing hardware.
2. **Preserve the underlying embodiment.** Allow interface replacement without unnecessary modifications to finger structure, articulation or actuation.
3. **Preserve functional articulation.** Maintain the required range of motion and accessible contact surfaces across supported interface configurations.
4. **Enable controlled and repeatable experiments.** Allow different interfaces to be evaluated on the same physical hand with consistent actuation, sensing and experimental procedures.

These criteria define the platform's research utility. The first three are assessed computationally (Section 6; `problem_formulation.md` A.6). The fourth must be demonstrated on the physical hardware.

Mechanism compactness, transmission characteristics and contact-interface capacity are important evaluation metrics, but they are means of assessing these criteria rather than independent project objectives.

---

## 6. Studies: Hypotheses (H) and Safe Claims (R)

**Naming.** P and C are the north star's problems and contributions (C1, C2; C2 has parts C2.1–C2.3). A study tests hypotheses H, numbered by study (H1 for Study 1; Hk.n when a study has several), each with its failure condition fixed before the data that decide it. R is what the result lets the paper claim safely, within the stated scope, numbered the same way. An R is never wider than its H's scope; a failed H is reported as such, and so is its R. How each study is defined, searched and measured is in `problem_formulation.md` (A.6) alone.

**Study 0: analysis** (`problem_formulation.md` A.6.0; checked on the Study 1 and Study 2 designs).
- **R0.1 (room; → C1).** Derived for planar finger joints through the finite pole: the clearance law (exact under equal face depths S1 and symmetric corners S2) and the coverage law. The best joint turns about a point on the contact face; a physical pin sits above the pad's mounting surface, so such a point must be a virtual pole.
- **R0.2 (motion; → C2.1).** Derived: the contact-motion law ‖Δc‖ = 2 sin(q/2)‖Δp‖, the sum rule Δ_contact + Δ_skel ≥ 2 sin(q_req/2)|Δt|, and the fixed-axis bound, which also holds for a rolling joint of fixed roller radius.
- **H0.1 (empirical).** At a coverage optimum, pad–pad contact is often the active constraint. *R:* the minimum clearance is within 0.1 mm on 47 of 51 Study 1 designs; the four exceptions are listed in `report/study0/study0.md`.
- **H0.2.** Coverage computed from the pole path and the pads equals the evaluator's. *R:* all 51 designs agree within 10⁻¹³.
- **H0.3.** Fixed-axis designs with evenly placed pads follow G = 2(d+h) tan(q_req/2). *R:* G − G_sym is −3.4 to +0.2 mm on fixed axes (the outliers are 150° designs whose pads are not evenly placed) and −1.6 to +0.2 mm on the CFL, with a median |G − G_sym| of 0.05 mm over the 9 CFL designs (`output/study/study2_pole_law/designs.csv`); the law is an estimate, and clearance is judged by the exact separating-axis test.
- **H0.4.** No fixed-axis redesign beats the bound. *R:* Study 2's 2R-remote redesigns sit on it at 90° and 120° (3.46, 6.93 and 10.39 mm at 120°).

**Study 1: best coverage per architecture, pad thickness and required flexion** (A.6.2; → C1).
- **H1.** Where both have a design, the CFL keeps more coverage than 2R-DD at every pad thickness and required flexion. *Fails if* 2R-DD matches or exceeds it in any condition. The ranking against 2R-remote and the idealized bounds is reported per condition, not hypothesized.
- **R1.** In the 2D side model (L_f 90 mm, pads 3–9 mm, q_req 90–150°), the CFL's best coverage exceeds 2R-DD's in all 10 conditions where both have a design, by 2.4–14.6 points, also without the CFL refinement; 2R-DD has no design at 150° for 7 and 9 mm. Against 2R-remote the ranking depends on the condition: 2R-remote is higher at 90°, the two cross between 5 and 7 mm at 120°, and the CFL is higher at 150°. From 3 to 9 mm the CFL loses 0.085–0.096 of C̄_pad. The margin exceeds the search spread: the CFL's worst run (GA and NSGA-II, three seeds each, plus continuation runs in two cells) still beats 2R-DD's best in 9 of 10 conditions (the exception: 90°, 9 mm, −0.6 points; `summary.csv`, `c_pad_run_low`). The CFL optima transmit with a worst-case mechanical advantage of 0.41–0.58 (symmetric RCJ 0.5, revolute 1; unconstrained in the search).

**Study 2: redesign for a thicker interface** (A.6.3; → C2.2).
- **H2.** Re-optimized from its 3 mm design for 5, 7 and 9 mm pads, the CFL changes its contact-surface motion by less than the fixed-axis bound at the coverage end of its front, with coverage near Study 1's best. (No failure condition: the CFL's pole can always move below the bound, so reaching it is not a real test; the result is reported as a size, 10–51% of the bound, and against the other joints. Removed 2026-11-22.)
- **R2.** In the 2D side model, at the coverage end of each front the CFL changes Δ_contact by 0.35–4.31 mm, 10–51% of the bound, with C̄_pad within 0.2 points of Study 1's best; it is the smallest of the four physical architectures in all 9 cells. Without the start offset of 2R-DD and RCJ it is the smallest in 7 of 9; the two exceptions lose 2.4 and 17.8 points of coverage (exploratory e19). Giving up 2 points of coverage along their fronts, 2R-DD and RCJ stay above the CFL. The values are found by the search, not proven minima. A fixed axis that may also slide its contact point along the face (c0 not kept) still cannot go below 0.586, 0.667 and 0.794 of the bound at 90°, 120° and 150° (minimum over the axis position and the along-face c0 shift of max_q |(I − R_q)Δp + R_q Δc0|; v28 review r3, checked numerically 2026-11-22 and again 2026-11-24); the CFL's largest change, 51% at 90° and 9 mm, is below this in every cell.
- Every interface is mounted at its own shallowest depth, without spacers. Raising a thinner interface on a spacer to the depth of the thickest one would keep the contact-surface motion on any architecture, but it fills the space the interface needs with a passive block, costs every thinner interface the coverage of the deep design (at 120°: 8.6 points for the CFL, 18.2 for 2R-remote, 3 mm against a 9 mm design), and fixes the thickest interface in advance. It is therefore excluded by requirement, not compared.

**Study 3: the measurement** (Section 8; → C2.3).
- **H3.** An interface can be studied independently of the finger when the redesign that carries it keeps the contact Jacobian: when the impedance setpoint a servo hand runs (one flexion command at a set stiffness, A) is reused across redesigns, the part of a measured interface effect that the redesign adds, E, follows Δ_contact. *Fails if* Δ_contact does not predict it, or if another candidate a designer might hold (Δ_skel, coverage, pad thickness, transmission) predicts it in every setting (its hand-bootstrap 95% interval excludes zero in each). Under a reused torque (C) and touch-referenced closure (E), which remove the over-closure with an ideal part a servo hand rarely has, the same analysis is reported alongside (eighteenth experiment, pre-registered).
- **R3.** In simulation, within the scope of Section 8.3: E follows Δ_contact for pad friction (ρ = 0.83, one-sided p = 0.0001, 15 hands; partial 0.88 given Δ_skel), pad stiffness (ρ = 0.80, 14 hands), on a hand-independent grid (ρ = 0.86 [0.72, 0.94], 45 hands; corrected 2026-11-22, the first analysis used a quarter of the grid) and under torque reuse (ρ = 0.75, 16 hands). Only Δ_contact's interval excludes zero in every setting. Its five one-sided p-values (0.0001, 0.00044, 0.00055, < 10⁻⁵ and 0.00034 in `dose_A_all`, `comp`, `dose_C_all`, `dose_A-grid_expanded` and `padk_matched`) are each below 0.05/25, so they survive Holm correction over all 25 tests (five candidates in five settings) whatever the other p-values. A fixed-axis finger that keeps its skeleton exactly errs 95, 179 and 228% against its own 3 mm finger; the CFL redesign that keeps Δ_contact errs least at 5 and 7 mm (3, 21%) and within one point of the least at 9 mm (41%), against 52–124% for the coverage-only design. Pad shape repeats the ordering (ρ = 0.94, six hands) but does not by itself separate Δ_contact from Δ_skel. Under touch-referenced closure the residual error follows Δ_skel instead. Under an ideal reused torque on the hand-independent grid (eighteenth experiment, pre-registered, 27 new 7 mm hands): E is 9–53% (median 20%); Δ_contact predicts it with the force deviation held fixed (partial 0.79 [0.54, 0.89]; ρ = 0.69, one-sided p = 4×10⁻⁵), and Δ_skel predicts it as well (0.71 [0.41, 0.87]), through the force a torque produces (ρ with the force deviation 0.67, against 0.10 for Δ_contact). The sole-predictor claim of R3 is therefore limited to the reused impedance setpoint a servo hand runs; under a reused torque the skeleton must be kept too, and the contact Jacobian still matters at equal force. Post hoc (v18 review): the distal face's signed normal offset over the grasped poses also predicts E (|offset| 0.71–0.82), but with it held fixed Δ_contact still does (partial 0.69 on the grid, 0.50 for friction, 0.21 for stiffness), while the offset adds little given Δ_contact (0.06, 0.09, 0.37); the offset's sign sets the error's direction. Scoring hands that cannot hold as the largest errors gives ρ = 0.81 (friction) and 0.67 (stiffness). At 9 mm the CFL redesign (Δ_contact 2.32 mm) errs 41%, so the CFL keeps a pad within the 20% tolerance up to about 7 mm. Post hoc (v19 review): at the reused setpoint on the grid, Δ_contact predicts E with each hand's setpoint force deviation held fixed (partial 0.64 [0.34, 0.86]; the force adds 0.25 given Δ_contact), so Δ_contact carries more than the preload; its maximum over q predicts about as well as its mean over the grasped poses (0.83, 0.80, 0.86 against 0.68–0.86; better for friction, lower for stiffness). Post hoc (v21 review): at the grasps the servo carries at most 0.25 N·m, about 25% of the XM335 stall torque, so position control under any higher current limit is the setpoint protocol; resampling hands within each thickness keeps the friction and stiffness intervals above zero ([0.41, 0.99], [0.35, 0.98]). The distal pads compress 0.5–2.3 mm at the grasps (median by hand; R 0.47, O 0.65, U 2.27 mm), the order of the 1 mm tolerance, and Δ_contact predicts E with that compression included. A spacer arm (Δ_contact = 0) is not simulated: in this contact model its error is zero by construction; its comparison belongs to hardware. R3 also holds when pads soften with thickness (contact stiffness 2.5 kN/m × 3 mm/h), judged against the counterfactual reference, O carrying the same soft pad without the redesign (seventeenth experiment): ρ(E′, Δ_contact) = 0.81 [0.33, 0.99], p = 0.0003 (14 hands), and R errs far less than U at 5, 7 and 9 mm (5, 14, 50% against 55, 129, 124%; paired Wilcoxon p ≤ 1.1e-7). Against the stiff O (fifteenth experiment) neither test holds, because that reference lacks the softer pad's own effect: an interface study can attribute a difference to the interface only against a reference that carries the same interface. Post hoc (`mediation.py`, `mediation.json`): under A the deviation of the distal force from O's tracks the error (ρ = 0.94) and Δ_contact sets that force (ρ = 0.62); with the force held fixed, Δ_contact's partial correlation is 0.55 [−0.02, 0.93]. Under C the forces stay within 0.84–1.45× O's, and Δ_contact predicts the error with the force held fixed (partial 0.80 [0.37, 0.95]): at equal torque the geometry itself matters. Over the 45 hands, Δ_contact correlates with the other candidates by |ρ| ≤ 0.40. Design tolerance (`tolerance.py`, increasing fit of E on Δ_contact): at a reused setpoint, E stays at or below 20% for Δ_contact up to 0.35–0.96 mm and at or below 50% up to 1.46–3.89 mm, depending on the setting; at equal torque, up to 2.33 mm for 20%. The CFL's Study 2 redesigns at 120° (0.35, 0.86, 2.32 mm for 5, 7, 9 mm) stay within the 20% tolerance up to 7 mm. Exploratory (e20): split into the distal face's offset along its normal and the slide along it, both parts predict the error in every setting, but which is stronger changes with the setting (0.47–0.89), while the whole Δ_contact stays at 0.76–0.86; the whole contact-surface motion is what matters. Δ_contact ranks first in every setting. Compared directly (hand-bootstrap interval of ρ(E, Δ_contact) − |ρ(E, X)|, `invariants.json`), it exceeds every other candidate, Δ_skel included, on the 45-hand grid, and 9 of the 16 comparisons over the four settings; with 14–16 hands the intervals against Δ_skel include zero, as the sum rule couples the two, and the fixed-axis family (Δ_skel = 0) separates them. Under the flexion-stiffness sweep (D, 7 mm, K/K₀ from 0.1 to 10) R errs 14–26% and U 44–131%; one load step is 2.5–5.6% of Δ(O). **Second engine and contact model, and other tasks (experiments 21–23 and 28, pre-registered; `report/study3/study3.md`).** The measurement was rebuilt in Drake (SAP; one planar finger, the same condition A, gates and load ramp; `sim/drake/`) because Isaac Lab's hydroelastic path (Newton collision with MuJoCo-Warp) was not repeatable: identical runs differed by E = 30%, against 2.2% for a 0.05° command change in point contact. With point contact the replica reproduces Study 3 (Gate 0 counts within 4 of Isaac Lab's; ρ(E, Δ_contact) = 0.67, p = 0.003, 16 hands; R 19%, U 72%). With hydroelastic pads (an elastic foundation of E = 0.1 MPa over 3 mm on every hand, fixed on O alone) E follows Δ_contact (ρ = 0.86, p < 10⁻⁴; Δ_skel 0.09; R 17%, U 254%); the hands' error sizes rank only loosely alike across the two models (ρ = 0.39, p = 0.069). Under the same reused commands, open loop, in two more tasks the added error follows Δ_contact and not the skeleton: grasp acquisition, the finger closing from open to draw a cylinder in against a load (ρ = 0.87, p < 10⁻⁴; Δ_skel 0.11; 18 hands), and turning a disk with the fingertip alone: the distal pad drags the top face of a disk whose axis is normal to the proximal face, turning it 12–38° against a torsional spring while the proximal pad never touches it (0.78, p = 0.0001; Δ_skel 0.20; R 17%, U 47%; noise floor 0.9%; distal pad pressed at most 1.5 mm; experiment 28z). It replaces the earlier dial (experiments 24 and 26), which pressed the distal pad 2–11 mm, beyond what the pad model represents. With 64 start conditions drawn at random from distributions fixed in advance and used for every hand (experiment 29, pre-registered), it holds: ρ = 0.68 (p 0.0009; Δ_skel 0.00; R 19%, U 55%), so the result does not depend on hand-picked placements. A translational fingertip task, the distal pad dragging a tethered coin 8.5–13 mm toward the palm on a sprung table (experiment 30b, pre-registered, 64 randomized starts), agrees, more weakly than the disk (ρ = 0.55, p 0.0098; Δ_skel −0.48; R 2.2%, U 16.3%; noise floor 0.6%); the same task with a short drag (1.8–4.3 mm, experiment 30) does not reach significance (0.36, p 0.073). In in-hand rolling by a flexion sweep it does not (0.28, p = 0.16): the redesigns add 10–35%, which neither quantity orders. Each task's noise floor (O under a 0.05° command change) is 0.2–0.8% in experiments 22–26 and 0.6–2.4% for the fingertip disk, the coin drag and the soft pads (experiments 25, 28, 29, 30b). **Pads as soft as they are thick (experiment 25, pre-registered):** with each pad an elastic foundation of 0.1 MPa over its own thickness, judged against O carrying the same pad, E′ follows Δ_contact (ρ = 0.78, p = 0.0003, 15 hands; 0.87 scoring the three hands that cannot hold as the largest errors) and not Δ_skel (−0.10); R errs less than U at every thickness (5.9 against 109%, 16 against 347%, 104 against 370% at 5, 7, 9 mm). Below about 1 mm of Δ_contact the errors match point contact (R 6, 16%); above it they are two to three times larger (R at 9 mm, Δ_contact 2.32 mm: 104%), so soft thick pads narrow the tolerance without changing the order. **A contact-referenced joint setpoint (experiment 27, pre-registered; Drake, point contact, 18 hands).** Joint impedance control is kept; only the setpoint changes: each hand's setpoint is the flexion at which the fingertip point of its distal face, in its proximal-face frame, comes nearest to where O's is at O's setpoint, from the kinematics alone (no runtime contact). The error falls on 15 of the 16 hands scored under both (median 41.7% → 14.0%; R 13.8%, U 25.9%; noise floor median 0%, 75th percentile 3.7%), and what remains still follows Δ_contact (ρ = 0.73, one-sided p = 0.0007) and not Δ_skel (0.18).81). Re-referencing one point removes the preload difference at that point; the path difference of the other contact points remains, and keeping the contact-surface motion is still needed. Only sensed touch (condition E) moves the residual to the skeleton.

**Study 3, continued: the conclusion of an interface study** (A.6.5; → C2.3).
- **H4.** When an interface study replaces O's stiff 3 mm pads by a thicker pad that softens with thickness and measures the change in pull-out load on a hand redesigned to carry it, the measured change departs from the true change (O carrying the soft pad, no redesign) as Δ_contact grows: both the share of placements whose conclusion is reversed and the relative error of the measured change rise with Δ_contact. *Fails if* either correlation is not positive (one-sided permutation p < 0.05).
- **R4 (preliminary, post hoc on the fifteenth and seventeenth experiments' runs).** The true change in pull-out load is −26, −41 and −51% (5, 7, 9 mm). Redesigned for coverage alone, U measures +21, +48 and +25%: the conclusion is reversed at 77–79% of placements. The redesign that keeps the contact Jacobian measures −26 and −55% at 5 and 7 mm (reversed at 4%) and −15% at 9 mm (34%). Over 14 hands the reversed share follows Δ_contact (ρ = 0.80 [0.34, 0.98], p = 0.0004) and so does the relative error (0.61, p = 0.011). Redesigns whose distal face retreats (skeleton moved most) do not reverse the sign but exaggerate the loss (−58 to −69%). The confirmatory test is the nineteenth experiment.
- **R4 (confirmation, nineteenth experiment, pre-registered; 27 new 7 mm hands on the hand-independent grid).** The true change is −37% (−34 to −40%). The relative error of the measured change follows Δ_contact (ρ = 0.94 [0.84, 0.97], p < 10⁻⁵): part (1) of H4 holds. The reversed share does not (ρ = −0.04 [−0.50, 0.42]): part (2) fails. The measured change runs from −4 to −75%. Post hoc, its direction follows the sign of the distal face's normal offset in the grid's flexion range (ρ = −0.99 with the signed error, −0.88 with the reversed share): a face that advances toward the object presses harder and shrinks or reverses the loss (up to 40% of placements reversed; U is such a hand), and a face that retreats exaggerates it. Safe claim: a redesign distorts the measured effect of the very interface that forced it by an amount that follows Δ_contact, in a direction set by where the distal face moved; a reversal is a case of the advancing face, not a law in Δ_contact. It is the last part of Study 3, in the SSoT as in the paper.

**Study 4: in-hand rotation with a learned policy** (in progress; `report/study4/in-hand.md`). A PPO policy with a plain MLP turns a 40 mm cube about the vertical axis on hand O in simulation (Isaac Lab, Newton MuJoCo-Warp). It is not yet evidence for any contribution; its intended use is zero-shot transfer of the O policy to R and U.

**Conditions that make the comparison fair:**

- Every architecture is compared at its own best design under the common goal, never built to another architecture's morphology.
- The compared architectures are the CFL (CoDe's mechanism), 2R-DD, 2R-remote and RCJ. Idealized variants without the joint actuator are declared upper bounds and reported apart.
- A result is a statement within the 2D side model and the stated coverage definition. "Not found" is a statement about the search, not about existence.

**What must be shown on hardware** (Section 8.4): that interfaces are in fact exchanged on one physical hand, with the same actuation and procedure (C1; criterion 4, Section 5.7), and that the redesign that keeps the contact Jacobian keeps it and errs less on the bench (C2). The computational studies cannot show this. Interface-effect experiments should use matched or explicitly controlled morphologies.

---

## 7. Claim Discipline

- Keep physical CoDe, the physical alternatives and declared idealized ablations separate. An idealization removes only its named constraint.
- Report failures and negative results. Never tune objectives, constraints or comparison assumptions to make CoDe win.
- Candidate designs are design alternatives, not statistical replicates. A search that finds nothing is not proof that no design exists.
- Matched morphology reduces confounding; it does not establish perfect causal isolation.
- Separate demonstrated observations, model-derived findings, assumptions and hypotheses.
- Planar feasibility is not buildability.
- Quantitative figures must read saved results, with source commit or patch, configs, input artifacts and sampling provenance. Missing models or artifacts block their branch of work; do not invent measurements.
- The next useful research decision is one that improves controlled interface variation or exposes its limits, whether or not it favors CoDe.
- Fix the primary metric, the test and the interpretation criteria before the data that decide them exist, and record them in the experiment's note in `docs/experimental/` before the run. Never change a criterion after seeing its outcome; a failed validity condition is reported as such. A measurement detail changed after a validation run (e.g. a gate on the reference hand) is recorded with its reason and the data it was based on, and only before the data that decide the comparison exist.
- Use enough independent replicates (policy seeds) for the planned test to be able to reject (an exact sign-flip test over 5 seeds cannot reach p < 0.05).
- Verify what a metric measures before interpreting it (e.g. that a goal count counts rotation and not overlapping success regions).

---

## 8. Study 3: Can the Contact Interface Be Studied Independently of the Finger?

### 8.1 The problem: decoupling the contact interface from the morphology in manipulation evaluation

A manipulation evaluation measures a hand and its interface together. To compare interfaces, the hand must be adapted to each one (P1, P2), so the measured difference mixes the interface's effect (intended variation, Section 2) with the adaptation's (unintended variation). The problem Study 3 addresses is to obtain a measurement of an interface's effect on manipulation that does not depend on how the hand was adapted to carry it.

Stated as a measurement: an interface variable c (friction, surface profile, compliance, ...) is changed between two values on a hand H, and a task measure M gives the effect Δ(H) = M(H, c₁) − M(H, c₀). The effect is decoupled from the morphology when Δ(H) is the same on the reference hand H₀ and on the hands adapted from it for a different interface, so that the embodiment-dependent error E(H) = |Δ(H) − Δ(H₀)| is small against Δ(H₀).

What Study 3 must resolve:

1. **Which parts of an adaptation remain.** An adaptation changes the transmission (servo angle to flexion), the skeleton's motion, the contact-surface motion, and the proximal contact surface's place at the MCP. The error that matters is what is left after control has undone what it can; the transmission is undone by control (Section 9, decided), the MCP level is a declared limitation (Section 8.3).
2. **A protocol in which the interface is the variable.** The same task and the same controller on every hand, with commands in flexion and stiffness matched in flexion. What is held fixed between hands (the command, the torque or the contact force) is stated, because an interface's effect can be measured differently under each.
3. **No adaptation in the evaluator.** A controller that is re-trained or re-tuned per hand absorbs the morphology into itself, and a policy trained on one hand and tested on another measures transfer, not the interface. The evaluator is fixed across hands.
4. **A measure and a control.** E(H) is reported against Δ(H₀), together with the raw difference between hands at one interface value (what an identical surface profile shows: the artefact the adaptation alone produces).
5. **A real effect to measure.** The interface change must have an effect on H₀ that is clearly non-zero in the task; a decoupling result on an effect near zero is not informative.

Study 3 answers one question, for contribution C2.3 (`north-star.md`):

> **When the finger's morphology must change to carry a thicker interface, what must the change keep first so that the same commands still handle the object the same way, and the interface's measured effect stays the one measured on H₀: the contact-surface motion (Δ_contact), the skeleton's (Δ_skel), or another candidate?**

Morphology is the skeleton (links and joints) together with the contact surfaces it carries (`north-star.md`, the question). The answer is expected to depend on whether the command senses contact; the protocols below are chosen to show both sides.

### 8.2 Design and result (simulation)

- **Task and measure.** A cylinder free in the sagittal plane, held by the proximal and distal pads of the CoDe hand model's fingers (MCP held), is loaded in four directions until the grasp gives way; Δ(H) is the change in that load when one interface property changes: pad friction (μ 1.3 → 2.0, primary), pad compliance (contact stiffness 2.5e3 → 1e3 N/m) or pad shape (flat → 2 mm crowned). The hands, placements, procedure, gates, protocols, error and statistics are defined in `problem_formulation.md` A.6.4; their pre-registration and amendments are in `report/study3/study3.md` (experiments 3–14).
- **Hands.** CFL, q_req 120°. H₀ is the 3 mm design; for 3 → 5, 7 and 9 mm, the Study 2 redesign, Study 1's independent design and redesigns chosen on design metrics alone so that Δ_contact, Δ_skel and coverage vary apart (18 hands). The unchanged finger with thicker pads (the naive swap) is mechanically infeasible at every thickness (its pads collide at 47° of flexion).
- **Protocols.** Reused flexion command at the same flexion stiffness (A); the same flexion torque (C); a flexion-space impedance between the two (D); the distal contact force matched to H₀'s with the simulator's contact force (B, an upper bound on compensation); each finger closing 5° past its own touch, with no shared command (E).
- **Why the two path changes differ.** Δ_contact and Δ_skel are the change of the same pole of the contact Jacobian, seen from the proximal contact face and from the link line (`problem_formulation.md` A.6.3); keeping the pole relative to the face (the Study 2 redesign moves it down with a deeper face) keeps the contact-surface motion and changes the skeleton's.
- **Result.** Under A, over the 15 hands that hold the cylinder, E follows Δ_contact (Spearman ρ = 0.83, one-sided p = 0.0001; positive at each thickness) and the skeleton's change does not predict it; the Study 2 redesign has the smallest error at 5 and 7 mm (3, 21%) and is within one point of the smallest at 9 mm (41% against 40% for a redesign at Δ_contact 3.5 mm with the skeleton moved most) and the coverage-only design 52–124%. Three redesigns cannot hold the cylinder under the same command (on release it is pushed 4–9 mm; the cause is not confirmed). The error enters through the contact force: under C it is 3–40% (still ordered by Δ_contact, ρ = 0.75), under B 3–7%; under D the redesign stays at 14–26% at every stiffness while the coverage-only design rises from 44% at 0.1 of the nominal stiffness to 119% at the nominal stiffness and levels off (125, 131%; 7 mm hands), where the contact's own compliance takes the interference. The conclusion (the Study 2 redesign errs less than the coverage-only design, pair by pair) holds when the contact stiffness is changed tenfold (ke 1e3 and 1e4 N/m), the friction impedance ratio of the solver threefold either way, and the substeps doubled (one-sided Wilcoxon p ≤ 3×10⁻⁴ in each); the size of the coverage-only design's error moves between 127 and 184%. For pad compliance (contact stiffness 2.5e3 → 1e3 N/m, μ 1.3), the error under A follows Δ_contact over 14 holding hands (ρ = 0.80, one-sided p = 0.0004; partial 0.91 given Δ_skel). When each finger instead closes 5° past its own touch (E; no shared command), all 18 hands hold, the error falls to 6–61% (median about 20%), and what remains follows Δ_skel (ρ = 0.88; partial 0.90 given Δ_contact): the geometry an experiment must keep depends on what it reuses. Under reused joint commands it is the contact-surface motion. With the cylinder placed on a grid chosen independently of any hand (4 places × 7 commands per radius), all 45 redesigns (the 18 above and 27 more at 7 mm) hold it and the error still follows Δ_contact (ρ = 0.86 [0.72, 0.94], one-sided p < 10⁻⁵; partial 0.86 given Δ_skel; Δ_skel 0.22 [−0.12, 0.51]), so the ordering is not an artefact of placing the object where H₀ touches it. (Corrected 2026-11-22: the first analysis keyed placements by q0 alone and used one of the four places per command, which gave ρ = 0.60 and a Δ_skel correlation of 0.46 through lost placements; `report/study3/study3.md`, analysis correction.) Among five candidate invariants a designer might hold (Δ_contact, Δ_skel, coverage, pad thickness, transmission change), only Δ_contact predicts the error under A for both friction and stiffness (hand-bootstrap 95% intervals of all others contain zero; `invariants.py`), and only Δ_contact's interval excludes zero in every setting measured. A fixed-axis finger that keeps its skeleton exactly (one revolute PIP at the centre of the reference's joint gap, the same links and axis for 5, 7 and 9 mm pads, each pad ending at the clearance law's gap, so Δ_skel = 0 and Δ_contact = √3·Δh, the bound) errs 95, 179 and 228% against its own 3 mm finger under A (the deeper faces meet the cylinder early and press 2.2–4.2× harder), against 3, 21 and 41% for the CFL redesign that keeps Δ_contact: keeping the skeleton does not keep the measurement. For a third interface property, pad shape (flat to 2 mm crowned, μ 1.3), the error follows Δ_contact in the same order over six redesigns (ρ = 0.94, exact one-sided p = 0.008); few placements hold under the crowned pad (1–14 per hand), and in that set Δ_skel moves opposite to Δ_contact, so it repeats the ordering but does not by itself separate the two.

### 8.3 Claim boundaries

- The result holds in simulation, in two engines and two contact models (Isaac Lab 3 with Newton and MuJoCo-Warp, point contact; Drake, point and hydroelastic contact; pads of one stiffness, softening with thickness, or as soft as they are thick in hydroelastic contact, each against the counterfactual reference), for one mechanism (the CFL, with one fixed-axis contrast family), one required flexion, three open-loop tasks (grasp pull-out, a load on a free held cylinder rising until the grasp gives way; grasp acquisition; turning a disk with the fingertip alone; not in-hand rolling, where it fails) and three interface properties (friction; compliance, as contact stiffness; shape). Its size depends on the protocol (largest with reused position commands) and, above the nominal stiffness, on contact stiffness. Under touch-referenced closure (E) the residual error follows the skeleton's change instead, so the claim is limited to commands that do not sense contact: reused joint commands, joint setpoints re-referenced from the kinematics (experiment 27) and, with the skeleton also kept, reused torques.
- Do not claim that embodiment differences in manipulation in general come from the contact surfaces rather than the skeleton. The claim is what a forced morphology change must keep first, under contact-unaware commands.
- Do not claim that keeping the contact-surface motion preserves a policy's performance in general, that CoDe is needed for every interface experiment, or that more skeleton change is better.
- **The MCP level is not kept (limitation).** What is kept is finger-internal: the PIP column of the contact Jacobian in the proximal-phalanx frame. The proximal contact surface is carried by the MCP, a revolute joint with its hardware on the axis; a thicker pad at the shallowest mount moves that surface away from the axis, so the MCP column changes on every redesign by the change in pad thickness. Keeping it would need either a spacer (excluded) or the axis inside the pad (not buildable): the fixed-axis bound (C2.1) applies at the MCP as at any fixed axis. A polycentric MCP would lift this and is left as future work.
- Report null results as such. Permutation p-values for 14–18 hands use random permutations (100,000 in Isaac Lab, 20,000 in Drake; seed 0), exact only up to 8–9 hands. Pre-registration can be checked in git from experiment 15 on, where plan and result are separate commits; experiments 3–14 were recorded with their results (`report/study3/study3.md`, experiment index), so they count as documented, not pre-registered; the post-hoc amendments of the experiment (contact-solver setting, hand reselection, two added hands, a symmetric error metric) are reported as exploratory.

### 8.4 Stage 2 plan (hardware; planned 2026-11-22, not yet run)

Hardware is not available for the extended abstract. The physical stage tests the exchange (C1) and checks C2.2 and C2.3 on one finger.

- **Hands.** One CoDe finger with fixed servos, palm and phalanx bodies. Four configurations of the CFL are built: H₀ (3 mm pads), H_R (Study 2 redesign for 7 mm), H_M (a 7 mm redesign at Δ_contact 2.3 mm, `d2p25_minDs_s0`) and H_U (coverage-only 7 mm). Changing configuration swaps the pad modules and the CFL links only.
- **C1, exchange.** Record the parts swapped, the time taken and whether any recalibration is needed beyond loading the hand's θ(q) table.
- **C2.2, contact-surface motion.** Track markers on both contact faces with a camera over a slow flexion sweep. Compute c(q) in the proximal-phalanx frame and Δ_contact of H_R and H_U against H₀. Compare it with the model's value and with the fixed-axis bound; the manufacturing tolerance is stated from repeat sweeps.
- **C2.3, measurement.** Physical version of the Study 3 task: a cylinder on a planar slide, the finger closed by the reused flexion command, a pull in the four directions with a force gauge until slip. The interface change is the pad friction: bare silicone against a textured cover. Use 5 placements × 4 directions × 5 repeats per configuration and friction level, and compute E for H_R, H_M and H_U (simulation, condition A: 21, 72 and 124% at Δ_contact 0.9, 2.3 and 3.9 mm). The ordering R < U is tested by a one-sided paired Wilcoxon test over placements and directions; the relation with Δ_contact by a one-sided Page trend test over R, M, U. Along R, M, U, Δ_contact rises (0.86, 2.33, 3.91 mm) while Δ_skel falls (7.67, 4.62, 3.38 mm) and coverage (0.70–0.73) and transmission (0.20–0.23) stay flat, so a rise in E follows Δ_contact and not the skeleton. In simulation (condition A, friction) the ordering R < M < U holds in 43 of 48 placement-direction blocks; with 20 blocks the Page test has power near 1 at simulation noise (2026-11-22; hardware noise unknown).
- **Mechanism and tolerances (added 2026-11-23).** A load cell under the slide records the settled normal force, so the preload path (E ≈ |N_H/N_O − 1|) is checked directly, not only its end effect. Pin backlash (dial gauge), servo deadband (encoder) and pad thickness (caliper, every module) are measured and compared with H_R's Δ_contact of 0.9 mm.
- **Second protocol (added 2026-11-23).** The same configurations are also run re-zeroed at touch (each finger stopped by its servo-current threshold, then closed by the same 5°). Simulation predicts that the skeleton leads there (Table I: ρ 0.88 against 0.44), so the hardware shows which rule applies to which protocol.
- **Negative results.** These are planned outcomes, not failures to hide:
  - Tracked Δ_contact above the bound: manufacturing tolerance dominates, and the redesign target needs a tolerance model.
  - H_R errs no less than H_U on hardware: pad compliance or friction variation outweighs the geometry. The claim is then limited to simulation, and the size of the physical effects is reported.
  - The exchange needs disassembly of the articulation: the integration cost is the result.

---

## 9. Paper Plan and Status

**Venue.** RSS 2027 (Athens, July 6–11). Extended abstract due 2026-12-04, stage-1 decision 2027-02-05, final paper due 2027-04-16 (from the call for papers; re-check on roboticsconference.org before each deadline).

**How the paper tells the north star's story** (details the north star leaves out):

- **Design section.** The finite pole of the distal phalanx relative to the proximal one is introduced here, not in the introduction. Room (C1): the clearance and coverage laws show that the best joint turns about a point on the contact face; a physical pin must sit above the pad's mounting surface, so that point must be a virtual pole, hence the CFL. Motion (C2.1): the contact Jacobian (PIP column in the proximal-phalanx frame, the same at every distal-face point; `problem_formulation.md` A.6.3), the contact-motion law, the sum rule and the fixed-axis bound.
- **Evidence sections.** C1 is told by R0.1 and Study 1 (H1, R1); 2R-remote (a belt-driven fixed axis) can match the CFL at shallow flexion and pays in C2.2. C2 is told by R0.2 (C2.1), Study 2 (H2, R2; C2.2) and Study 3 (H3, R3; C2.3), with the fixed-axis family that keeps its skeleton exactly.
- **Evidence range.** The interfaces studied are pad thicknesses (3–9 mm), with one pad shape in simulation; camera-based tactile modules are future work. The hardware checks of both contributions are in Section 8.4.

**Evidence for each contribution** (contributions as stated in `north-star.md`; hypotheses and safe claims in Section 6; status 2026-11-22):

| C | H and R | Status |
|---|---|---|
| C1 | R0.1; H1, R1 (H0.1–H0.3 check the laws) | Shown in the 2D side model |
| C2.1 | R0.2; H0.4 | Derived; 2R-remote redesigns sit on the bound |
| C2.2 | H2, R2 | Shown in the 2D side model |
| C2.3 | H3, R3; H4, R4 | Shown in simulation (Section 8), in two engines and contact models and three tasks (fails for in-hand rolling; experiments 21–23 and 28), and under a kinematically contact-referenced setpoint (experiment 27); its conclusion part (H4) confirmed for the size of the distortion (part 1), not for reversal (part 2) |
| Hardware | Section 8.4 | Planned for both contributions; hardware not available for the extended abstract |

The earlier Study 3 programme (3A pilot and main experiment, 3B) is archived in `docs/experimental/study3_interface_platform.md` and is not evidence for C2.3.

**Decided.**

- No spacers (2026-11-21): every interface sits at its own shallowest depth, and redesign minimizes the change in contact-surface motion under that requirement (Section 6, claim 3). Study 3 has no spacer path.
- The transmission is compensated by control and recorded, not added to the redesign target (2026-11-21). Measured on the Study 3 hands (CFL, 120°, 3 → 7 mm; `design/scripts/study/exploratory/e15_transmission_change.py`): dθ/dq differs from H₀'s by up to 20% (H_R) and 21% (H_U), MA_wc by −11% and −18% (Study 2 overall: −7% to +16%). Commands in flexion mapped through each hand's θ(q) table (matching the design kinematics to 0.0003°), with the servo gains scheduled as K_θ = K_q/(dθ/dq)², hold the commanded flexion within 0.07° in simulation (MuJoCo-Warp, servo torque limits off).

**Open decisions.**

1. **Transmission on hardware.** Whether the servos' torque limits allow the compensation where MA_wc falls (up to −18% on the Study 3 hands); simulation runs without torque limits.
   The CFL, like a rolling joint, gains range of motion and room for the pads at the cost of mechanical advantage (MA_wc 0.41–0.58 against 1 for a revolute joint with the servo at it). Study 1 leaves MA_wc free: a fixed floor would select the winner by the choice of its value. The paper states the coverage–range-of-motion–MA trade-off as a limitation; a designer sets the floor from the payload or reads the coverage–MA Pareto front (decided 2026-11-22; a pilot at MA_wc ≥ 0.5 was stopped unfinished, `output/study/linkage/cfl_q*_ma05`, not evidence).
2. **Hardware scope for the final paper.** Section 8.4 plans one finger with three configurations; whether the final paper adds more interfaces (a camera-based tactile module) is open.
3. **"Interfaces concentrate on distal phalanges."** Not established (counterexamples exist). P1 states it as an observation that motivates the work, not as a claim; a survey of hands and tactile-manipulation papers would support it.
