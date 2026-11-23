# Content refinement log (PaperOrchestra AgentReview loop)

| Version | Commit (manuscript repo) | Overall (r1, r2, r3) | Mean sub-scores O/Q/C/Si/So/P/Co | Outcome |
|---|---|---|---|---|
| v0 | 8afd3ea | 5, 5, 5 | 3.0/2.0/2.0/2.33/2.0/2.0/2.67 | baseline |
| v1 | bfb28fc | 5, 5, 5 | 3.0/2.0/2.0/2.0/2.0/2.0/2.67 | accepted: Overall same, rounded sub-axes equal (PaperOrchestra case 3a) |
| v2 | d869061 | 5, 5, 5 | 3.0/2.33/2.0/2.33/2.0/2.0/2.33 | accepted: Study 3 results added (SSoT C4); Overall same, Quality and Significance up, Contribution down by one rater (case 3a) |
| v3 | 932ee03 | 5, 5, 6 | 3.0/2.33/2.0/2.0/2.67/2.0/2.33 | accepted: full rewrite along north-star in the style of Vaswani et al. (2017), shape interface added; Overall and Soundness up (case 1) |
| v4 | a865d79 | 5, 5, 6 | 3.0/2.33/2.0/2.0/2.33/2.33/2.67 | kept: touch-referenced closure, pad stiffness, bootstrap CIs, Study 1/2 checks, Stage-2 plan; Overall unchanged; reviewers turned the touch-referenced result against C4 (case 3a) |
| v5 | 31fc051 | 5, 5, 5 | — | prediction-first framing; r3 down on the bound vs 2R-DD (case 3b, reverted direction) |
| v6 | c570180 | 5, 5, 6 | — | candidate-invariant table, retargeting argument, general fixed-axis statement (case 3a) |
| v7 | ef15bfe | 5, 5, 6 | — | reused-torque column; r3 Accept, r1/r2 unchanged |
| v8 | cfca3fe | 5, 5, 6 | — | falsification lines, scope, fit |
| v9 | 40bee48 | 5, 5, 5 | — | hand-independent placement grid (all 18 hold, rho 0.69) |
| v10 | e1b5910 | 5, 5, 5 | — | grid on 45 redesigns (rho 0.60; skeleton also matters there via lost placements), lug sentence |
| v11 | 7b198b9 | 5, 6, 5 | — | fixed-axis family with the skeleton kept exactly (95/179/228%), offset-free Study 2, C2 restated |
| v12 | 00b632e | 6, 6, 6 | — | Stage 2 protocol, spacers and 2R-remote treated fairly, Holm, post hoc label, Fig. 1 legend |
| v13 | 8f3ecd0 | 6, 5, 6 | — | new north star (C1 room, C2 contact Jacobian), H1-H3 per study, Study 3 setup per A.6.4, new title, CAD Fig. 1; reviewers read the PDF |
| v14 | d7691f1 | — (failed), 6, 6 | — | short contributions, Study 1 as ablation + MA, Study 3 two routes (post hoc mediation), magnitudes, Stage 2 test; r1 stopped by an API safeguard error |
| v15 | f7b5853 | 6, 6, 6 | — | gap-law explanation fixed, CFL floor, direct comparisons (7 of 12), gap law max error; r1 re-run on Opus |
| v16 | 16c483e | 6, 6, 6 | — | soft-pad counterfactual (17th), grid key fix (rho 0.86), Holm 25; r1/r3 Opus (r1 after a safeguard retry), r2 Fable 5.2 after two safeguard errors |
| v17 | d722efa | 6, 6, 6 | — | Study 4 (reversed conclusion, post hoc), Fig. 1(d), ideal torque wording, Stage 2 trend test; all three reviews on Fable 5.2 after Opus safeguard errors on every first try and retry |
| v18 | 6a80bac | 6, 6, 6 | — | independence framing; Study 3 trimmed to one table by protocol (18th: ideal torque, partial 0.79); Study 4 folded in (19th: error rho 0.94, reversal fails, direction = signed face offset); H2 failure condition dropped; MA cost stated; details to captions; r1 Opus, r2/r3 Fable 5.2 after two Opus safeguard errors each |
| v19 | d7ce877 | 6, 6, 6 | — | C2 scoped to servo setpoints in intro and conclusion; H3 scope; placements, R/U/M defined; signed face offset tested (Delta_contact keeps 0.69 given it); dropped hands scored; Holm; 9 mm limit; S1/S2 named; r1 Opus, r2 Opus on retry, r3 Fable 5.2 after two Opus safeguard errors |
| v20 | 3f61a74 | 6, 6, 6 | — | setpoint force and face offset held fixed (0.64, 0.69), CFL pole depth, r_C defined, partial row in Table I, sensitivity under Fig. 5, MA as a stated trade-off (no floor); r1 and r2 Opus (r1 on retry), r3 Fable 5.2 after two Opus safeguard errors |
| v21 | 65bf2eb | 6, 6, 6 | — | force matching vs recomputation separated, contact model named, pre-registration record stated, Fig. 1 one placement vs medians, c(q) keeps the column; all three Opus on the first try with the ASSESSMENT NOTES prompt |
| v22 | 72af53d | 6, 6, 6 | — | current limit (servo at <=25% of stall), within-thickness intervals, near-zero control (R at 0.35 mm errs 3%), Stage 2 reference; all three Opus at the first try |
| v23 | e84ee3e | 6, 6, 6 | — | Stage 2 separation (Delta_contact rises while Delta_skel falls along R, M, U; simulated ordering in 43 of 48 blocks); r2 Presentation 3; all Opus at the first try |
| v24 | 0b6c14b | 6, 6, 6 | — | c0-free fixed-axis floor (0.59-0.79; read as a strength), reversal marked model-level, Stage 2 backlash and pad compression, Prop. 2 remark split; all Opus at the first try |
| v26 | 92ab39b | 6, 6, 6 | — | pad compression in the simulation (0.5-2.3 mm), spacers zero by construction, v25 clarity edits; Presentation 3 from all three (first time), Clarity 3 from r1; all Opus at the first try |
| v27 | 3f3019d | 6, 6, 7 | — | Study 3 as one double-column figure: a second simulator (Drake) with hydroelastic pads and three more open-loop tasks (rolling fails); r1, r2: Presentation 2 (length over the limit, Fig. 1 placeholder); shared doubt that the error is the preload; r3: 7, the first above 6 |
| v28 | a7c8fec | 6, 6, 6 | — | fits 5+1 pages: CAD Fig. 1, Studies 1-2 merged, Drake task scenes, preload stated as the error's path, direct candidate comparison, PIP-column limit; all three Presentation and Clarity 2 (text too compressed); r3 7 -> 6 (RCJ below the bound unexplained, start offset, sign conventions) |
| v29 | 55d3862 | 6, 6, 6 | — | conventions and definitions stated, RCJ below the bound explained, first-order preload prediction, Holm family, touch re-zeroing named; Presentation still 2 from all; r1, r2: thick soft pads never simulated as thick (-> experiment 25), touch re-zeroing understated; r3: Delta_contact is a path, bound assumptions per joint, laws vs optima |

Recurring weaknesses after v1 (not fixable by presentation alone):
1. The premise that a few mm of D_c confounds an interface measurement is not demonstrated; no D_c tolerance.
2. CFL moves the change to the actuator side (MA_wc −7%…+16%, skeleton motion); why that is acceptable is not argued.
3. H1 margins comparable to CFL run spread; extra search only for CFL; H2's fixed-axis half is a theorem.
4. H3 protocol underspecified (interfaces, tracking, trials, acceptance threshold).

After v2 (Study 3 added, 2026-11-22). Recurring weaknesses 1 and 2 above are now answered by Study 3 (the
contact-surface motion change predicts the measurement error; transmission compensated by control). New
weaknesses, all three reviewers:
1. Under reused position commands H3 is close to built in (more over-closure → more force → a different
   friction-limited load); under torque reuse or force feedback the effect is small. → Conclusion narrowed to
   position-command reuse; force feedback named as removing most of the need.
2. Rigid contact while the paper is about compliant pads; one interface property (friction); n = 15 curated
   designs; "not D_s" untested. → Partial correlations (E–D_c given D_s 0.88; symmetric E–D_s given D_c −0.18),
   failed hands ranked worst (ρ 0.81), thickness coefficient (+0.08) and a local predictor (face offset at the
   grasp, ρ 0.92) added; robustness (contact stiffness, solver, timestep) and the shape interface running.
3. Internal inconsistencies (3.9× vs 3.3×, "D_c ≥ 2.9 mm cannot hold", "grips like O", design counts 15/16/18,
   figure order) → fixed. Corollary 1 now states that c_0 is kept (reviewer 3). Tactile modules (15–30 mm)
   beyond the 3–9 mm range → stated as a limitation.

After v3 (rewrite, 2026-11-22). Fixed in the manuscript after the round: p value now one-sided 1e-4 as in Fig. 1d;
n 15 vs 16 explained; impedance U 119 % at K0 (not 131 %); robustness stated as placements of one 7 mm R–U pair at the
15 mm cylinder; per-thickness rho (1.0, 0.93, 0.80); J no longer used for R_{pi/2}; keeping J <=> keeping c stated in
both directions; the fixed-axis bound conditioned on the mount rule (recess Delta d = Delta h is the only escape); the
Study 2 line declared not to bound 2R-DD and RCJ (c0 not kept); the sum rule Delta_contact + Delta_skel >=
2 sin(q_req/2)|Delta t| added (also in SSoT A.6.3; all 18 Study 3 hands satisfy it, min-Delta_skel designs are tight)
and used to say the marginal rho(E, Delta_skel) is not evidence by itself; F_p origin, link line, L_f(q) defined.
Open, not fixable by presentation:
1. Study 3 under reused commands is seen as close to built in (face offset -> normal force); a protocol that places the
   object per hand, or compliant (non-rigid) pads, was asked for by r1 and r2.
2. Statistics: 15 curated hands, partial correlation without CI; robustness over placements of one hand pair.
3. Study 1 margins vs seed spread; Study 2 reports only the coverage end of fronts.
4. Stage-2 (C5) plan thin; SSoT lists the hardware scope as undecided.

After v4 (2026-11-22). All three: Study 3 seen as built in under a reused stiff command; r1/r2/r3 used the
touch-referenced result (residual follows Delta_skel) against C4. Response in v5 (31fc051): the over-closure mechanism
is stated first as the prediction of Sec. III and then tested; command reuse justified (touch or force closure uses
signals the interface changes); the skeleton-keeping redesign (Delta_skel 0.4 mm, E 103 %) answers 'keep the skeleton';
touch-referenced and force-matched closure reported as medians only; pseudo-replicated Wilcoxon p dropped; failed hands
ranked worst (rho 0.81); Delta_skel frame (link line) defined; C1 scoped to hardware-limited mount depth; Lemma typo,
3.3x vs 3.9x fixed. Detailed caveats moved out of the body (user, 2026-11-22: big-picture limitations in the
conclusion only).

After v6: r1/r2 still read Study 3 as built in by position over-closure. v7 adds the reused-torque column (no
over-closure; Delta_contact still the only predictor, 0.75 [0.38, 0.93]), the impedance evidence, the grasp face offset.

Pattern over v3-v8 (2026-11-22): r3 (kinematics) 6 in four of six rounds; r1/r2 stay at 5 with one structural
objection: under a face-referenced placement and rigid-contact pads, C4 is close to true by construction (face offset
-> over-closure), the competing invariants are weak by construction (transmission compensated, sum rule couples
Delta_skel, thickness three levels), and force or torque protocols already cut most of the error. Text changes no
longer move these two scores; what could: a task not referenced to O's contact (e.g. in-hand rolling or a pick from a
fixed pose), deformable pads, or a quantified benefit beyond the best interface-independent protocol.

After v9 (r1, r2 = 5): the 'built in by O's placement' objection left r2's top three; remaining: Study 3 only CFL
hands, comparative statistics (difference-of-correlation intervals include zero vs Delta_skel in every setting at
n 14-18; wording narrowed to 'the only candidate whose interval excludes zero'), no noise floor, rigid-pad contact
model, force/torque protocols with F/T sensing. Bootstrapped rho differences and a five-candidate rank regression
were computed (stiffness: skeleton coefficient -0.74 exceeds contact 0.56 in magnitude), so no 'largest
coefficient' claim is made.

v10 pattern (all three at 5): Study 3 uses only CFL hands; the reused-command protocol works through over-closure
and is dismissed alternatives without care; O treated as ground truth; Delta_contact includes the start offset for
2R-DD/RCJ; C2's failure line contradicted by Fig. 3; Prop. 2 explanation reads as contradictory; missing polycentric
knee and Burmester literature. v11 answers: a fixed-axis family with the skeleton kept exactly (Delta_skel = 0) errs
95/179/228%; offset-free motion change (CFL least in 7 of 9 cells); C2 restated to coverage level in every condition;
Prop. 2 explained by equal growth of numerator and denominator; one shared contact model makes O's effect the
reference; protocol framed as command or torque reuse (current-controlled servos) with sensing protocols reported.

v11 pattern: r2 moves to 6 (borderline accept). Remaining: the Stage 2 plan is two sentences (all three); C2 against
2R-DD only, with a metric switch for 2R-remote; spacers dismissed; abstract mixes n = 6 shape into the rho range and
says 'least of four' (7 of 9 cells offset-free); no multiplicity correction; grid subset looks post hoc; title and
'contact Jacobian' wording. v12 answers: a Stage 2 protocol with counts, the R-vs-U hardware test (same pads) and its
failure line; 2R-remote's coverage lead stated with its cost in C3, the L_f metric switch removed; spacers framed as
the fix when every interface is known in advance; abstract states 6 to 45 redesigns and 'nearly every condition';
Holm over 20 tests; 'post hoc' label; Fig. 1 legend fixed.

v12: all three at 6 (borderline accept); goal reached. r1 and r3 caught that 'R errs least at each thickness' was
false at 9 mm (a maxDs redesign at Delta_contact 3.5 mm errs 39.7% against R's 40.7%); corrected in the paper and the
SSoT after the round. Remaining critiques (not addressed): C4 partly built in by the frame of the placement; why a
redesign rather than a contact-sensing protocol; title length; 'contact Jacobian' wording; C-bar_pad as a grasp
measure; Montana / Kerr-Roth and function-generation synthesis not cited.

v13 pattern (2026-11-22, after the C1/C2 rewrite; reviewers read the PDF with figures): r1 6, r2 5, r3 6. All
three: Study 3 under a reused command works through over-closure (force), and force-matched closing leaves 3-7%;
H1's 2R-DD baseline is weak and CFL vs 2R-remote is mixed; mechanical advantage unreported; dense text;
3.9x (Fig. 1, one placement) vs 3.3x (median). r2: candidates designed, not sampled (no intercorrelations), no noise
floor. r3: H2/H3 falsifiers lax, "keep" overclaims. v14 answers: a post hoc mediation analysis (mediation.py):
under torque reuse the forces stay within 0.84-1.45x O's and Delta_contact predicts the error with the force held
fixed (partial 0.80 [0.37, 0.95]); under the command, Delta_contact sets the force (0.62), which tracks the error
(0.94). Candidates |rho| <= 0.40 over 45 hands. Study 1 told as an ablation (remote actuation makes room, the
polycentric joint keeps it at deep flexion), MA_wc 0.41-0.58. H3 restores "fails if Delta_contact misses in any
setting". Robustness with magnitudes (stiffness sweep R 14-26%, U 44-131%), load-step resolution, Stage 2 effect
size and test, C1 hardware failure line. Contributions shortened; scope sentences moved to the design section.

v14 (r1 stopped by an API safeguard error, not a score): r2 6 (up from 5), r3 6. r2: the reused-command result is
close to built in (force); H3 is comparative but candidates are tested only against zero; Stage 2's R vs U cannot
single out Delta_contact; one contact model for all pads. r3: Study 1's Eq. (3) explanation had the wrong sign (the
cause is the fixed-axis gap, 2 tan(q/2) per mm); "the CFL can keep it" overclaims (10-51% of the bound), with a
non-zero floor from the pivots; H1 baseline. v15 answers: the gap-law explanation, "cuts its change to a fraction",
the floor (lowering the pole path by Delta_h would move the pivots into the pad), direct comparisons (Delta_contact
ranks first in every setting; beats coverage, thickness or transmission in 7 of 12 bootstrap intervals; Delta_skel
separated by the fixed-axis family), the gap law's maximum error (1.6 mm), all 45 grid hands hold.

v15: 6, 6, 6 (r1 re-run on Opus after the v14 safeguard error). Plateau. All three: Study 3 is close to built in
because every pad thickness shares one contact model (real thick pads are softer) and the main protocol is a reused
position command; designed hand set, n 14-16; C1 against 2R-DD only; "contact Jacobian" overreach. r3: S2 does not
hold for the CFL (gap law off by up to 1.6 mm), H2 compared only with the fixed-axis bound, correlations possibly
driven by the fixed-axis family (they are not: it is evaluated apart). Next: the fifteenth experiment (pre-registered,
ed8d53a), pad stiffness falling with thickness (ke = 2.5e3 x 3/h), condition A on O and the 18 redesigns.

v16: 6, 6, 6. The soft-pad and grid results were read as strengths, but the plateau holds. All three: under a
reused command H3 is nearly implied (force mediation 0.94); the real evidence is the torque protocol plus a post hoc
partial (the eighteenth experiment, pre-registered c51e92d, replicates it on 27 new hands); the fixed-axis family is
left out of the correlations without a reason; the CFL's MA 0.41-0.58 halves the PIP torque; no seed spread in
Study 1. r3: Delta_contact also counts the distal face sliding along itself; the offset normal to the face is the
quantity that sets over-closure and is missing as a candidate. r2: Stage 2 tests an ordering, not the relation.

v17: 6, 6, 6 (all on Fable 5.2: Opus stopped every reviewer twice with a safeguard error). Study 4's reversed
conclusion was read as the headline but flagged as post hoc with a counterfactual that favours R; R's 9 mm result
(-15% against -51%) was missing. Repeated: the reused-command result is a preload effect (force matching leaves
3-7%); transmission idealised; MA and backlash; design space unspecified; H2 nearly unfalsifiable. Pending answers:
the eighteenth (equal torque, 27 new hands) and nineteenth (Study 4 confirmation) experiments.

v18: 6, 6, 6 (r1 Opus; r2 and r3 on Fable 5.2 after two Opus safeguard errors each). All three Accept, Clarity 2,
Presentation 2. Read as strengths: the pole analysis (r3 re-derived every equation), the skeleton-kept negative
result, a failed prediction reported openly, the tolerance as a design rule, the protocol columns. Common critiques:
scope C2 to reused impedance setpoints in the C2 bullet and conclusion, not only in Study 3 (all three); mechanical
advantage unconstrained in Study 1 (all three); the signed normal offset of the distal face may be the simpler
governing variable and is not a candidate in Table I (r2, r3); CFL at 9 mm (Delta_contact 2.3 mm) is beyond the 1 mm
tolerance (r2); undefined terms (placements, grid, reversed share, M) and -37% vs -41% (r2, r3); hands dropped for
not holding, clustering, multiplicity (r2); S1/S2 stated only in a caption, pole path of the CoDe linkage never
plotted (r3); H2 still a weak test (r3); simulation and linear pad model (all).

v19: 6, 6, 6 (r1 Opus; r2 Opus on the retry; r3 Fable 5.2 after two Opus safeguard errors). All Accept; r3 raised
Clarity and Presentation to 3. The statistics critique turned into a strength (pre-registered grid, Holm, post hoc
labels). Remaining: under the reused setpoint the effect may be preload; report each hand's setpoint normal force and
Delta_contact's partial given it (r2); the contact-model sensitivity checks are missing from the text (r2); Delta_contact
at the grasp pose vs its maximum over q (r2, r3); the definition of r_C under which Eq. (7) holds (r3); Prop. 1 exact
only when the pole moves in depth alone (r3); the optimised CFL's pole depth vs d + h never shown (r3); RCJ below the
bound at 90 deg (r3); partial correlations per setting not in Table I (r3); MA unconstrained (all); "contact Jacobian"
is the PIP column only (r1, r3); missing underactuated/linkage contact-force literature (r1); simulation only (all).

v20: 6, 6, 6, all Accept. MA moved from a weakness to a stated trade-off for r2; r1 and r3 still ask for a floor.
Misreadings caused by compression: r1 read the force-matching 3-7% as setpoint recomputation; 0.7x/3.9x (Fig. 1, one
placement) vs 0.75x/3.3x (median) still unclear. New: the counterfactual's construct validity and the pad stiffness
implementation (r2); "pre-registered" needs a verifiable record (r2); Delta_contact is a path deviation, not a bound on
the Jacobian column (r3); Study 3 only at 120 deg (r3); the CFL's parametrisation absent (r3); Eq. (7) assumes equal
face deepening (r3); captions carry too much (r2, r3). Safeguard errors: every failure is tagged
[reasoning_extraction]; likely the THOUGHT section of prompt_reviewer.txt.

Reviewer prompt change (from v21, 2026-11-22): the THOUGHT section of prompt_reviewer.txt ("discuss your intuitions
and reasoning... the note-taking phase") is renamed ASSESSMENT NOTES and asks for a concise assessment of the claims
and evidence. Every Opus safeguard failure since v14 was tagged [reasoning_extraction]; the review content and the
JSON are unchanged, so scores stay comparable.

v21: 6, 6, 6, all Accept, all three on Opus at the first try: the ASSESSMENT NOTES prompt removed the safeguard
failures. The misreadings of v20 are gone. Remaining: point contact with a normal stiffness makes part of the
Delta_contact-error relation expected by construction, and the soft-pad reversal is a property of that model (r2);
Stage 2's ground truth for the interface effect is not stated (r2; it is the friction change measured on O itself);
current-limited position control, as on LEAP, between setpoint and torque (r1); a Delta_contact = 0 control (spacers)
in Study 3 (r3); a c0-free metric (r3); which constraint keeps the CFL pole about 4 mm above the face (r3); the q-space
stiffness on hardware (r3); cluster bootstrap by thickness (r2); MA floor, coverage-to-grasp link, density (all).
Plateau: 6, 6, 6 since v12 (ten rounds); the remaining critiques are mostly simulation scope and density.

v22: 6, 6, 6, all Accept, all Opus at the first try. The current-limit and bootstrap answers were taken as
strengths. Remaining, mostly repeats: point contact makes the Delta_contact-error relation partly geometric by
construction (all); no first-order derivation of E from the normal over-closure (r3); the counterfactual's
justification (r1, r2); MA at free advantage and 2R-DD as a weak baseline (r1, r3); spacers designed for the
maximum thickness as a stronger baseline (r3); Stage 2 power and whether R, M, U separate Delta_contact from
Delta_skel (r1, r2); density. Post-round edits (no review): the grid column is the 18 plus 27 new hands; M is
named the middle redesign. Plateau: eleven rounds at 6, 6, 6 since v12; the loop is paused here.

v23: 6, 6, 6, all Accept. Answered by v24 (no further run): the c0-free fixed-axis floor (0.59/0.67/0.79 of the
bound, from r3, checked numerically; the CFL is below it in every cell); the reversal marked as within this contact
model (r2); Stage 2 reports backlash and pad compression with Delta_contact (r1, r2); the Prop. 2 remark split.

v24: 6, 6, 6, all Accept. r1 read the c0-free floor as showing the CFL's advantage is the pole, not contact
placement. r3's "z = r tan(q/2)" is a misreading of the PDF text (the source has r tan(q/4)). Answered in v25
without a new round: the 48 blocks are simulated; spacers cover only interfaces no thicker than the design's; a
saturated current limit acts as a torque (Table I's torque column). Repeats: point contact and pad compression of
the order of the 1 mm tolerance, designed samples, post hoc numbers, the spacer arm, MA, density. The loop stays
paused at the plateau (thirteen rounds at 6, 6, 6).

v26: 6, 6, 6, all Accept; Presentation 3 from all three for the first time (r1 also Clarity 3). Read as strengths:
the Stage 2 hands make Delta_contact and Delta_skel predict opposite trends; the skeleton-kept control; the spacer
cost and the c0-free floor. Remaining: MA (the largest for r1), a C1 metric for the Jacobian claim (r3), Delta_contact
under load (r2), how the 27 grid hands were sampled (r2), -41% vs -37% still read as one experiment (r2), load-
dependent friction of the textured cover with R and U pressing 4x apart (r1), a repeated build in Stage 2 (r2),
contact model (all). Overall unchanged since v12 (fifteen rounds).
| v30 | 7eb982a+ | 6, 6, 6 | — | Fig. 4 task sequences rendered in Drake 3D (grasp pull-out, acquisition, 30° dial of experiment 26), protocol scope stated, non-holding hands scored, Stage 2 preload and tolerances; Presentation 2 -> 3 from all three; remaining: the rule is protocol-specific and the abstract/conclusion read as general (r1, r2), thick soft pads not modelled as thick (r1, r2; experiment 25 running), Delta_contact is a position not a Jacobian and c0 is free in Eq. (11) (r3), the fixed-axis bound assumes the axis keeps its depth (r3), "studied on its own" overstates R's 21/41% (r3), -37% vs -41% (r2), MA 0.41-0.58 (r1) |

v27: 6, 6, 6, all Accept (manuscript e56892b; restructured Design/Experiment, rewritten introduction, Fig. 2
with the CFL kinematics, no study or C/H labels). Identical sub-scores from all three: O3 Q3 C2 Si2 So3 P2 Co3,
confidence 4. Presentation fell from 3 (v26) to 2 and Clarity is 2 for all. Common critiques: the reused-setpoint
result reduces to preload (E ~ |N_H/N_O - 1|; Delta_contact includes the start offset); the prescription holds
only under reused setpoints (touch favours the skeleton, force matching leaves 3-7%); designed samples of 14-16
hands; simulation only; dense text, undefined labels (O, R, U, M, grid, r_{1,e}, r_{2,s}, mounting rule, goal),
Figs. 2c and 4b-d hard to read; 2R-remote gets much of the room at 90-120 deg. r3: Corollary 1 vs the 0.59-0.80
floor reads as a contradiction (the c0-free assumption change is not explained). r1: no convergence or seed
variance for the GA/NSGA-II.
