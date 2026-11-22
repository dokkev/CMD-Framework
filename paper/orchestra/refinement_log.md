# Content refinement log (PaperOrchestra AgentReview loop)

| Version | Commit (manuscript repo) | Overall (r1, r2, r3) | Mean sub-scores O/Q/C/Si/So/P/Co | Outcome |
|---|---|---|---|---|
| v0 | 8afd3ea | 5, 5, 5 | 3.0/2.0/2.0/2.33/2.0/2.0/2.67 | baseline |
| v1 | bfb28fc | 5, 5, 5 | 3.0/2.0/2.0/2.0/2.0/2.0/2.67 | accepted: Overall same, rounded sub-axes equal (PaperOrchestra case 3a) |
| v2 | d869061 | 5, 5, 5 | 3.0/2.33/2.0/2.33/2.0/2.0/2.33 | accepted: Study 3 results added (SSoT C4); Overall same, Quality and Significance up, Contribution down by one rater (case 3a) |
| v3 | 932ee03 | 5, 5, 6 | 3.0/2.33/2.0/2.0/2.67/2.0/2.33 | accepted: full rewrite along north-star in the style of Vaswani et al. (2017), shape interface added; Overall and Soundness up (case 1) |

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
