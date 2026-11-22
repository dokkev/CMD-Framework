# Content refinement log (PaperOrchestra AgentReview loop)

| Version | Commit (manuscript repo) | Overall (r1, r2, r3) | Mean sub-scores O/Q/C/Si/So/P/Co | Outcome |
|---|---|---|---|---|
| v0 | 8afd3ea | 5, 5, 5 | 3.0/2.0/2.0/2.33/2.0/2.0/2.67 | baseline |
| v1 | bfb28fc | 5, 5, 5 | 3.0/2.0/2.0/2.0/2.0/2.0/2.67 | accepted: Overall same, rounded sub-axes equal (PaperOrchestra case 3a) |
| v2 | d869061 | 5, 5, 5 | 3.0/2.33/2.0/2.33/2.0/2.0/2.33 | accepted: Study 3 results added (SSoT C4); Overall same, Quality and Significance up, Contribution down by one rater (case 3a) |

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
