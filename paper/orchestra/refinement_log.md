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
