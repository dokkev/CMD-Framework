ASSESSMENT NOTES:

The paper says that when a finger is redesigned for a thicker contact interface (pad), the motion of its contact surfaces should be preserved, not the motion of its skeleton. It argues this in three steps:
- a finite-displacement pole analysis (Eqs. 1–5);
- a servo-off-joint crossed four-bar PIP finger (CoDe) found by optimization;
- simulation evidence that a redesign which reuses the original impedance setpoint misreports a pad's effect by an amount that tracks Δ_contact and not Δ_skel.

Claims and evidence:

1. **The kinematic bound (Eqs. 4–5).** The core identity, ‖Δc‖ = 2 sin(q/2)‖Δp‖, is correct for two designs that start from the same initial contact point. Three assumptions are not argued:
   - The bound (4) is then stated in general ("no redesign keeps both motions"), but Eq. 7 counts a moved initial point. The derivation needs the same c0 for both the contact point and the fingertip, and the paper does not say whether the bound survives when c0 moves.
   - The step "F_p moves by Δt relative to F_1" assumes the contact surface is parallel to the link line and the frame moves by pure translation.
   - Eq. 5 depends on the premise that a fixed axis must sit above a full-width pad's mount. Remote-center-of-motion fingers [13,15] are an obvious counterexample class. The paper dismisses them in one clause ("not the same kinematics across interfaces") without analysis or data.

   The bound is neat and useful, but it is a modest result, and it is weaker in generality than the abstract implies.

2. **Room and coverage (Fig. 3a).** CoDe beats the at-joint finger in all 10 feasible conditions, by 2.4–14.6 points. The run-to-run spread of the CoDe optimizations is 1.3–3.4 points, which is comparable to the smallest margins. Remote actuation alone matches CoDe at 90° and up to 5 mm at 120°, so the four-bar is what matters only for thick pads at large flexion. That is a narrower claim than "more room for thick interfaces." Mechanical advantage is reported only for CoDe.

3. **Error tracks Δ_contact (Sec. IV-B1).** This is the strongest part. It rests on:
   - partial correlations with Δ_skel held fixed;
   - permutation tests with Holm correction;
   - the revolute same-axis control (Δ_skel = 0, still 95–228% error);
   - two simulators and two contact models;
   - a pre-registered grid of 27 redesigns.

   Caveats:
   - 12 of the 18 correlation designs are constructed at prescribed Δ_contact and extreme Δ_skel. This makes the decoupling partly a product of design-set selection, and the effective sample is small.
   - The fingertip tasks correlate weakly (ρ 0.43–0.53), and their per-task p-values are uncorrected.
   - Neither contact model deforms the pad. "Compliance" is a scaled contact stiffness or a hydroelastic modulus. For a paper whose whole subject is pads, a compliant pad that adds geometry without deforming is a serious modeling limitation.

4. **Not only preload (Sec. IV-B2).** These are good controls: equal torque, kinematic re-referencing, and force matching. Two of their results bound the scope of the claim:
   - force matching leaves only 3–7% error;
   - under sensed-touch closing, the error follows the skeleton.

   So the claim holds only for commands that do not sense contact. The paper says so, but this restriction should also appear in the abstract.

5. **Policy transfer (Fig. 5).** There are only three policies and two seeds each. Cube rotation shows a large seed effect for all three hands (seed 21 at about 18–20%), so "similar shares" is weak evidence. The relay dial is admitted to be outside the claim. The finger-walk result supports the thesis, but it is a single task.

6. **Headline numbers.**
   - The abstract's "48% more vs 41% less" comes from Coverage, a design that never tried to preserve anything, so it is partly a straw man. The fairer baseline is Skeleton, which errs 222%.
   - Fig. 1(d) gives the ratios behind those figures: CoDe −55% vs reference −41% (relative error ≈ 34%) and Coverage +48% vs −41%. These do not obviously match Table I's compliant-pad column (−14% and +125%). The aggregation used must be explained.
   - The conclusion's "near 20% up to 7 mm" hides −43% (Drake point, convex) and −25% (stiffness).
   - In Table I, CoDe is not better than Coverage under the torque and touch commands.

7. **No hardware.** For a hand-design paper at RSS, there is no physical prototype test, even though Fig. 1(a) presents the hand. Hardware is deferred to future work.

My scores rest on: an original and well-posed design principle; thorough simulation controls for the central correlation; but a simulation-only validation with non-deforming pad models, a bound whose assumptions are not stated, and headline numbers that are selected against a weak baseline.

REVIEW JSON:
```json
{
  "Summary": "The paper proposes contact-centric design (CoDe) for robot fingers that must carry exchangeable contact interfaces (pads, tactile skins). It argues that when a finger is redesigned for a thicker interface, the motion of its contact surfaces (relative to the proximal contact surface) should be preserved rather than the skeleton/fingertip motion, because learned policies reuse joint impedance setpoints. A finite-displacement pole analysis gives a lower bound showing that contact-surface and skeleton motion cannot both be preserved when pad thickness changes, and that a fixed axis above the pad mount cannot preserve contact motion from the same initial contact point; a crossed four-bar PIP, whose pole can reach the contact faces, can. A servo-off-joint finger with a crossed four-bar PIP is optimized (GA/NSGA-II) for coverage and for preserved contact motion, and compared with at-joint revolute, belt-driven remote revolute, and rolling contact joints across pad thicknesses of 3-9 mm and flexion ranges of 90-150 degrees. In Isaac Lab and Drake (point and hydroelastic contact) across four tasks, the error a redesign adds to a measured pad effect correlates with the change in contact-surface motion and not with the change in skeleton motion, including with skeleton change or force deviation partialled out, and under setpoint re-referencing. Policies trained on the original hand transfer best to the CoDe redesign when the task uses the pads.",
  "Strengths": [
    "Clear, well-posed and practically relevant question: which kinematic property should be invariant when a finger's interface changes so that interface effects remain measurable under reused commands.",
    "Elegant use of the finite-displacement pole to link both room for thick pads (Eq. 2) and preserved relative motion (Eqs. 3-5) to a single design quantity.",
    "Strong controls for the central correlation claim: a revolute same-axis control with zero skeleton change that still errs heavily, partial correlations holding skeleton change and force deviation fixed, setpoint re-referencing, force matching, and a pre-registered grid of 27 redesigns.",
    "Replication across two simulators, two contact models, and four Drake tasks, with an explicit self-consistency noise floor (O against itself).",
    "Honest scoping: the paper states that the claim does not cover contact-sensing commands or fingertip-end contact, and reports cases where CoDe is not better (torque, touch).",
    "Systematic comparison of four PIP articulations over a grid of pad thicknesses and flexion ranges, including a remote-actuation baseline that separates the benefit of moving the servo from that of the articulation."
  ],
  "Weaknesses": [
    "Validation is entirely in simulation, and neither contact model deforms the pad: compliance is a scaled contact stiffness or a hydroelastic layer. For a paper about interface pads, how faithfully these models capture pad effects is the main threat to validity. No hardware experiment is shown, although the hand is presented in Fig. 1(a).",
    "The bound in Eq. (4) is stated generally, but its derivation assumes the same initial contact point and fingertip (Eq. 3), and a pure translation of F_p by delta-t (contact surface parallel to the link line). Eq. (7) explicitly allows a moved initial contact point, so the scope of the bound is unclear.",
    "Eq. (5) rests on the premise that a fixed axis must sit above a full-width pad's mount. Remote-center-of-motion joints [13,15], which place a virtual axis at the skin, are dismissed in one clause without analysis or comparison.",
    "The correlation set is partly constructed: 12 of the 18 redesigns are held at prescribed delta_contact, mostly at extreme delta_skel. This builds the decoupling into the sample, and n = 18 from related optimizations is small for the strength of the claim 'with nothing else'.",
    "The coverage margins over the at-joint finger (minimum 2.4 points) are comparable to the reported spread across optimizer runs (1.3-3.4 points). Remote actuation alone matches CoDe in a substantial part of the design space.",
    "The headline numbers compare against Coverage, a design that does not attempt preservation. Against the more relevant Skeleton baseline, the abstract gives no number. The abstract's 48%/41% example and the Fig. 1(d) values do not obviously agree with Table I's compliant-pad column, and the conclusion's 'near 20% up to 7 mm' omits the -43% and -25% entries.",
    "Policy-transfer evidence is thin: three policies and two seeds, with large seed variance (cube rotation). Only one task (finger walk) directly supports the thesis.",
    "The scope is narrow in practice. Force matching reduces the error to 3-7%, and sensed-touch closing makes the error follow the skeleton, so the benefit applies to non-contact-sensing impedance commands. Only the PIP of a planar finger with MCP fixed at zero is studied.",
    "Terminology: the quantity used, delta_contact, is a maximum path deviation in a frame on the proximal pad, not a Jacobian. Calling it 'the contact Jacobian' is loose."
  ],
  "Originality": 3,
  "Quality": 2,
  "Clarity": 2,
  "Significance": 2,
  "Questions": [
    "Does Eq. (4) hold when the redesign moves the initial contact point and/or the fingertip at q = 0, as Eq. (7) allows? If not, please state the bound's assumptions explicitly in the text.",
    "Does the derivation of Eq. (4) assume the contact surfaces are parallel to the link line, so that F_p translates by exactly delta-t? How do inclined or curved pads change the bound?",
    "How would a remote-center-of-motion joint [13,15] fare under Eqs. (2)-(5) and in the design study? Why is it excluded from Fig. 2(a) and Fig. 3?",
    "How are the Fig. 1(d) numbers (CoDe -55% vs reference -41%, Coverage +48%) computed, and how do they relate to the compliant-pad entries in Table I (-14%, +125%)?",
    "How were the 12 prescribed-delta_contact redesigns chosen? Does the correlation survive using only the six optimized redesigns plus the 27-grid designs?",
    "What is the Skeleton design's error in each Table I setting? Why is Skeleton omitted from Table I?",
    "How sensitive are the conclusions to the contact stiffness (2.5 kN/m) and hydroelastic modulus? Would a deformable (FEM) pad model change the ranking?",
    "What does the 'convex' column in Table I denote? How are the 'touch', 'force' and 'setpoint' commands defined precisely?",
    "What is the mechanical advantage of the competing designs, to compare with CoDe's 0.41-0.58?",
    "What exactly is pre-registered, where is the record, and which analyses were not pre-registered?"
  ],
  "Limitations": [
    "Simulation only, with non-deforming pad models. Results may not transfer to real compliant or tactile pads, whose deformation and sensing dynamics matter.",
    "The claim applies to commands that do not sense contact. Force-matched and touch-triggered commands remove the effect or reverse which motion matters.",
    "Planar finger, PIP only, MCP fixed at zero. A full-hand, multi-joint extension is not analyzed.",
    "The kinematic bound assumes the same initial contact point and frame translation parallel to the link line.",
    "Policy-transfer results rest on three policies and two seeds.",
    "No negative societal impacts beyond those generic to dexterous manipulation research."
  ],
  "Ethical Concerns": false,
  "Soundness": 2,
  "Presentation": 2,
  "Contribution": 3,
  "Overall": 5,
  "Confidence": 4,
  "Decision": "Reject"
}
```

PRESENTATION AND CLARITY ITEMS:

1. The prose is extremely compressed. Many sentences pack two or three claims together with parenthetical qualifiers (e.g., Sec. IV-B1, first paragraph, and the Table I discussion). A reader has to re-parse them several times. Split claims and move qualifiers into figure captions or footnotes.
2. Table I's column headers are undefined in the text: "convex", "compl. pad", "touch", "force", "setpoint". The note that the compliant-pad columns are judged against O carrying the same pad is buried in the Results prose. Define every column in the caption, and add a Skeleton row and a spread measure (IQR or n placements).
3. Notation is inconsistent: the pole is P(q) in Eq. (1) but p(q) in Eq. (3) onward. L_f is "a prescribed extended length" yet written L_f(M, q) in Eq. (6). Δp_1 appears without being defined as the pole shift seen from F_1.
4. Eq. (4) ends with a colon that runs into the prose ("…|Δt| : no redesign keeps both motions"). State the result as a proposition with explicit assumptions.
5. "CoDe [3]" in Sec. III-D cites Bai et al. as if it were the authors' design. Sec. III-A cites [16] (rolling contact joints) for the claim that only a crossed four-bar reaches thick pads' contact faces. Both citations look misplaced.
6. Fig. 1(d): the ✓/✗ criterion ("within 20 points of O's") mixes percentage-point and relative-error notions. The values (−55% vs −41%, "83% of the reference's") use different units for the two tasks. Use one metric.
7. Fig. 1(c) has no legend explaining that the y-axis is the deviation from O over flexion. The axis label "change from O (mm)" sits only on the left panel, and the right panel's tick labels are cramped.
8. Fig. 3(a): the scatter markers are very small and the four joint types are hard to tell apart in grayscale. The design sketches lack a scale bar ("to one scale" is in the caption only).
9. Fig. 3(c): "contact motion kept (%)" relative to the fixed-axis bound is an unusual normalization, and negative values to −100 are hard to interpret. Explain the formula in the caption, or plot Δ_contact in mm with the bound drawn as a line.
10. Fig. 4: the per-task E / Δ / ρ text blocks are tiny and float detached from their plots. The renders at three moments are too small to convey anything. Consider a compact table instead. "16–18 correlated redesigns" should say which tasks have fewer and why.
11. Fig. 5: "Relay dial (fingertip end)" is a different task from the Drake "Fingertip dial" in Fig. 4. The similar naming invites confusion; rename one. The "palm shift" quantity is defined only in the caption.
12. The definitions of O, CoDe, Skeleton and Coverage come at the end of the contributions list, before the reader knows what Δ_contact and Δ_skel are. Move them to after Sec. III-C, or define the deltas informally first.
13. The abstract and Fig. 1 use "48% more … 41% less" against Coverage. Add the Skeleton comparison so the headline is not read as a straw-man contrast.
14. "Contact Jacobian" is used as a synonym for the contact-surface path deviation. Either show the Jacobian relation or use "contact-surface motion" consistently.
15. "Pre-registered" analyses are mentioned without saying where the record is kept. Either give an anonymized reference or describe what was fixed in advance.
16. The Conclusion is two sentences. It should state the limitations the reviewers will raise: no pad deformation, simulation-only, non-sensing commands, PIP-only planar finger.
17. Related Work is very short and does not position the work against literature on kinematic invariance and transfer across embodiments (policy transfer across morphologies) or against RCM finger designs in any depth.