# CoDe Hand — Exploratory studies E01–E10: synthesis and final report

2026-11-07 · `feature/2dof-fe` at `eb767bd` plus uncommitted exploratory
code. Each run folder saves a source snapshot. Criterion: `GOAL.md`. Brief:
`INSTRUCTION.md`. Registry and revisions: `registry.md`. One report per
study: `E01_*.md` … `E10_*.md`.

All results use the existing 2D side-view evaluators, the Study 1/2 matched
references (one 90 mm prototype morphology) and the Study 1 module catalog.
The ideal RCJ, and the ideal 2R used in E02, are idealized ablations,
reported apart.

## Overview

| ID | Research question | Result | Evidence strength | North Star relevance | Recommendation |
|---|---|---|---|---|---|
| E01 | How much palmar space does each architecture leave free of hardware? | SUPPORTED, with a cost | Strong (direct geometry); single morphology | High (§2.1 space competition) | Use: "remote actuation moves hardware off the contact side, at a dorsal cost" |
| E02 | Does polycentric motion cause the thick-pad fold advantage? | PARTIALLY_SUPPORTED, refined | Moderate–strong: ablation, sufficiency counterfactual, E10 replication; 2D, one morphology | High (why the feasible space grows) | **Top finding**: a virtual pivot inside the interface volume |
| E03 | Are proximal and distal modules independent? | NOT_SUPPORTED (CoDe not coupled) | Moderate (small grid) | Medium (§5.3) | Cite as supporting evidence for modularity |
| E04 | Does accommodation depend on actuator orientation? | SUPPORTED, qualified | Strong (full tilt sweep, other variables held) | High (decoupling) | **Use, with the qualifier**: joint-servo designs at their best tilt come within 3–4 pairs |
| E05 | Is CoDe fragile under tolerances? | NOT_SUPPORTED | Moderate (model MC) | Medium | Hardware action: open the A pin–PIP servo contact |
| E06 | Does the Study 1 ranking survive clearance? | PARTIALLY_SUPPORTED | Moderate | Medium (claim robustness) | State that the Study 1 lead holds to 1 mm except at two static servo contacts |
| E07 | Does own hardware occlude the faces across postures? | NOT_SUPPORTED as registered (CoDe lowest) | Moderate (2D upper bound) | High (usable contact while folding) | Supporting evidence for E02/E01 |
| E08 | Does thickness shrink the two-face enclosing range? | INCONCLUSIVE | Weak (metric not discriminative) | Low | Drop |
| E09 | Does adaptation hide a transmission change? | NOT_SUPPORTED as CoDe-specific; general effect found | Moderate (few cases) | High (§2.3 confounding) | Use as motivation: adaptation changes the force map by about 4–5× ε_E (CoDe, RCJ) and 1–2.5× (2R-DD) |
| E10 | Which CoDe features predict capacity? | PARTIALLY_SUPPORTED, refined | Moderate (biased archive; correlation) | Medium–High | Use as a replication of E02; \|AB\| untestable |

## Cross-study synthesis

**Consistent trend: hardware on the joint axis is what costs contact space.**
Four studies, by different methods, find that a servo, pulley or roller on
the joint axis is what limits interfaces:
- E01: palmar intrusion up to 9.4 mm in the joint region;
- E03: module coupling in the 2R-DD at 0° and the RCJ at 0°;
- E07: distal-face occlusion of 60–88% beyond 90°;
- E02: such hardware prevents a palmar pivot.

CoDe's joint has only 3 mm pin housings at A and B, and puts its rotation
centre where there is no hardware.

**Unexpected advantage: the virtual pivot sinks into the interface volume
(E02, E10).** Two independent methods agree:
- a causal ablation and counterfactual on the matched morphology;
- span-controlled correlation over 150 archived designs.

CoDe's thick-pad and high-flexion capacity follows how close its virtual
pivot comes to the contact faces at high flexion (4–6 mm below the link
line at 90–120°). It does not follow how far the ICR travels. A fixed pin
at that depth would do the same, but only by putting the axis hardware at
the contact surface.

**Unexpected disadvantages and qualifications.**
- **Orientation (E04).** Most of Study 1's gap to the laid-back 2R-DD and
  RCJ is an orientation choice. At their best tilt they reach 31–32 pairs,
  against CoDe's 35.
- **Packaging contacts (E05, E06).** The prototype's packaging has
  zero-clearance static contacts: the A pin against the PIP servo, and the
  PIP servo against the MCP servo. Under a uniform 0.25 mm clearance they
  collapse CoDe's verdict.
- **Dorsal height (E01).** CoDe is not thinner. It moves its hardware
  dorsally, to a 26 mm dorsal top.

**Replicated versus re-measured.**
- **Genuinely different methods:**
  - E02 and E10 (causal ablation versus archive correlation);
  - E01 and E07 (static raster versus a posture-swept probe).
- **The same physics measured twice, not replication:**
  - E08 merely re-measures E01's protrusion;
  - E05 and E06 both expose the same static contact.

**Optimization and implementation artifacts found.**
- E04: tilt changes were rejected through the goal coupling.
- E05: INVALID sentinels were mislabelled as pad collisions.
- E06: the mounting policy ignores `body_clearance_mm` while the evaluators
  apply it. This is a real repository inconsistency, recorded for repair.
- E02: a pad is not checked against its own body's hardware under the
  searched depth policy.
- Earlier: the belt 2R's relative sampling box in Study 2.

**Model limits common to all.**
- 2D side view with Y layers assumed.
- One morphology.
- Zero clearance unless stated.
- No contact mechanics, strength or 3D assembly.
- Idealized variants kept apart.

## 1. Top three discoveries

**1. CoDe's thick-interface advantage comes from a virtual pivot inside the
interface volume (E02, replicated by E10).**
- **What.** An ideal pin joint at CoDe's extended ICR loses about 4° of
  φ_max per mm of pad thickness (173° → 142° from 1 to 8 mm). CoDe loses none up to
  8 mm, and stays feasible at 150° up to 10 mm. Moving the pin 4–6 mm toward
  the faces reproduces CoDe; CoDe's ICR sits 4.2–5.8 mm below the link line
  at 90–120°. In the archive, ICR palmar drop predicts capacity beyond the
  spans (partial ρ = +0.44, interval [0.27, 0.57]). ICR travel does not:
  its partial correlation is weakly negative (ρ = −0.14, interval
  [−0.26, −0.01]).
- **Why unexpected.** The working story was "moving ICR". The evidence says
  the *location* the motion reaches matters, not the motion.
- **Comparisons.** CoDe against the ICR-pinned and pin-swept ideal 2R;
  physical 2R/RCJ as context; 150 archived CoDe designs.
- **Evidence type.** An ablation, a sufficiency counterfactual and a
  correlational replication. It is not a full causal isolation, and it
  holds in 2D for one morphology. CoDe's ICR also moves proximally, and a
  proximal pin helps at 5 mm. At 8 mm and above, only a lower pivot
  recovers 150°.
- **Principle.** Folding pads collide less when the rotation centre lies
  near their contact surface. A real pin there needs its housing and its
  actuator there, inside the interface. A crossed four-bar puts a hardware-
  free virtual pivot there.
- **Missing.**
  - 3D and hardware confirmation.
  - Other morphologies.
  - A physical palmar-pin 2R as a competitor (for example, tendon-driven).

**2. Remote actuation decouples the interface verdict from actuator
orientation (E04), and keeps the palmar side clear (E01).**
- **What.** CoDe carries 35 pairs at every tilt from 10° to 120°, and the
  belt 33 from 0° to 120°. The joint-servo designs vary by 12–16 pairs with
  tilt over their feasible tilts. On the palmar side, CoDe keeps intrusion to its 3 mm pins along
  100% of the finger (d = 3), against 23–80% for the joint-servo designs.
- **Why unexpected.** The size of the orientation effect, and the finding
  that Study 1's gaps to the 2R-DD and RCJ are mostly orientation choices.
- **Principle.** Hardware on the joint axis competes with the faces. Moving
  it off the axis leaves accommodation independent of how that hardware is
  oriented.
- **Missing.**
  - A dorsal-envelope constraint (the standing servos are tall).
  - Re-optimizing the joint-servo designs at each tilt.

**3. Morphology adaptation changes the transmission by several times the
visible morphology change, in every architecture (E09).**
- **What.** The distal servo torque per unit contact force changes by
  33–67% in median, about 4–5× ε_E, for CoDe, the RCJ and the ideal RCJ.
  The 2R-DD changes less (1.0–2.5×). The samples are small (n = 2–7 per
  architecture).
- **Why unexpected.** ε_E looks small (0.05–0.2). The force map does not.
- **Principle.** The pad centre moves along the segment, so its lever arm
  about the joint changes. Arguably, part of this belongs to the interface
  change itself. Link (Q) changes add a further, separate channel in CoDe
  (the `long` case).
- **Missing.** More cases, and a measured response.

## 2. Strongest negative findings
- **CoDe still cannot carry long proximal pads at the prototype morphology.**
  Studies 2 and 2c show this, and E01–E10 do not change it.
- **The best-oriented joint servos nearly match CoDe** (31–32 against 35
  pairs; E04). The belt 2R is as orientation-robust as CoDe, and is
  shorter on most compact targets (Study 2c).
- **CoDe is thicker dorsally (26 mm) than the laid-back designs (9.4 mm;
  E01).**
- **The prototype's verdict collapses under any uniform clearance (E06),**
  until two static servo contacts are opened.
- **Polycentric motion as such is not needed for the fold advantage (E02).**
  A palmar fixed pivot achieves it, if one can be actuated.

## 3. Potential paper contributions (at most three)
1. *Claim:* "In a matched-morphology comparison, the crossed four-bar keeps
   thick contact modules usable to high flexion because its virtual pivot
   moves 4–6 mm into the contact-interface volume, where a physical pin
   joint would need its axis hardware. An ideal pin at the extended ICR
   loses about 4° of range per millimetre of pad thickness, and for pads
   8 mm thick or more, only a pin moved equally close to the faces recovers
   it." (E02, E10; 2D, one morphology)
2. *Claim:* "With remote actuation, which modules a fixed morphology
   accommodates is independent of actuator orientation over a 110° range,
   while joint-mounted servos change by 12–16 of 71 module pairs with
   orientation alone, reaching 31–32 pairs against CoDe's 35 at their best
   orientation." (E04, with E01; 2D, one morphology)
3. *Claim:* "Accommodating an interface by adapting the morphology changes
   the contact-to-actuator force map by roughly 1–5 times the
   normalized morphology change (median by architecture), so interface comparisons with adapted hands are
   confounded unless the morphology is held." (E09, motivating Study 1's
   fixed-morphology protocol)

## 4. North Star assessment
**The findings refine the motivation rather than change it.**
- They give a mechanical reason why a contact-centric architecture can keep
  the morphology fixed while the interface changes. Contact space is lost
  mainly to hardware on the joint axis, and a remote-actuated linkage can
  put its rotation centre into the contact region without hardware there.
- They also show that the advantage is specific: thick pads and high
  flexion, rather than long proximal pads.
- Part of the earlier contrast was actuator orientation, and part was
  packaging details.
- E09 strengthens the confounding argument of GOAL.md §2.3 for all
  architectures.

## 5. Recommended next experiments

**Essential before publication.**
- Repeat E02 and E04 on at least one more matched morphology, for example a
  compact or native one.
- Fix the mounting policy so it applies `body_clearance_mm`, then rerun
  Study 1 at a stated realistic clearance, with the two servo contacts
  opened.
- Physically verify the thick-pad fold to 150° on the prototype, the claim
  behind contribution 1.

**Valuable but optional.**
- Add a physical palmar-pin 2R (tendon or linkage driven) as a competitor,
  to test whether contribution 1 is unique to linkages.
- Add a dorsal-envelope limit to E04.
- Run a targeted |AB| ε-constraint sweep (the open user question; E10 could
  not test it).

**Separate future research.**
- Oblique-approach accessibility.
- Contact mechanics of thick modules.
- Manipulation experiments with matched morphology.

## 6. Implementation and verification
- **Added.** `scripts/study/exploratory/`:
  - `common.py`;
  - `e01`–`e10` scripts;
  - follow-ups `e02b` and `e10b`;
  - `figures.py`, `README.md`.
- **Tests.** `tests/test_exploratory.py`. No change to `core/`.
- **Outputs.** `output/study/exploratory/eNN/`, each with a manifest,
  result CSVs and a source snapshot. Figures:
  `report/figures/exploratory/`, read from those CSVs only.
- **Studies.** 10 attempted and documented.
  - SUPPORTED: E01, E04 (2).
  - PARTIALLY_SUPPORTED: E02, E06, E10 (3).
  - NOT_SUPPORTED: E03, E05, E07, E09 (4).
  - INCONCLUSIVE: E08 (1).
  - No blocked study.
- **Reviewer verdict.** See the end of this file.
- **Reproducibility.** Each script is deterministic (fixed seeds where
  sampling is used), saves its config and source record, and takes seconds
  to minutes. Nothing is committed (INSTRUCTION.md: no commit without
  authorization).

## Independent review

**Reviewer.** One read-only reviewer examined the specifications, code,
saved CSVs and reports on 2026-11-07.

**Verdict: PASS_WITH_NOTES. Blockers: none.**

**What the reviewer recomputed.** The key numbers, from the saved CSVs:
- E02/E02b: φ_max, the 150° verdicts, the raise thresholds and the ICR
  path;
- E04 and E06: the full tables;
- E07: the maxima;
- E10: the correlations and the bootstrap;
- E01 and E09: the medians.

**Fairness.** Confirmed: idealized variants are kept apart, and the seat
search and pin exemption are applied to every mechanism. The exclusion of
the physical-2R E02b rows was confirmed as justified: under the searched
depth policy, a pad is not checked against its own body's servo
(`pip_servo_pad_check` defaults to False; `mount_depth_violation` returns
0).

**Notes, all applied.**
- E02 table: the 5 mm "any pin" cell was wrong (a pin moved 4–6 mm
  proximally is feasible), and the depth variant was unlabelled.
- The ideal 2R's φ_max range (173°, not 176°).
- E07: occlusion at 60° or less is small but not zero.
- The joint-servo tilt range is 12–16 pairs, not 14–16.
- E09 one-line summaries overstated "3–9× in every architecture". The 2R-DD
  is 1–2.5×, and n is small.
- E10: ICR travel is weakly negative, not null.
- E06: the list of clearance sites was missing.
- E02's evidence is to be called an ablation plus a sufficiency
  counterfactual plus a replication, with the (x, z) caveat.
- The "2D, one morphology" qualifier belongs on every contribution.
