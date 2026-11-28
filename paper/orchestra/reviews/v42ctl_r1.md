ASSESSMENT NOTES:
The paper makes three central claims.

(1) **Kinematic claim.** When a finger is redesigned for a thicker interface, the motion of the contact surfaces and the motion of the skeleton cannot both be preserved (Eq. 4). A fixed axis cannot preserve contact-surface motion from the same initial contact point (Eq. 5). A crossed four-bar can.
- The pole argument is clean and, as far as I can verify, correct. Eq. (3) is the standard (I−R)Δp identity. Eq. (4) follows from the triangle inequality once the two frames differ by a pure translation Δt.
- Eq. (5), however, rests on a modelling assumption stated in a single clause: the pin and lugs "must sit above a full-width pad's mount". A revolute axis lowered into a split or narrower pad is not ruled out.
- Fig. 3(c) seems to conflict with the bound. At q_req=90°, the at-joint (fixed-axis) designs sit at roughly +20–40% "kept", above the dashed fixed-axis line. That is presumably because their initial contact point moved, but the paper does not say so. The text's statement that "the remote-servo joint sits on the bound" also does not match its −75 to −100% points at 150°.

(2) **Design-study claim.** CoDe gives more coverage than an at-joint finger in all feasible conditions and preserves contact motion best.
- This is supported by GA/NSGA-II results with three seeds per run. Remote actuation alone gets most of the room benefit at 90° and at ≤5 mm, so the four-bar's added value is concentrated at large thickness or large RoM.
- The worst-case mechanical advantage of 0.41–0.58 is a real cost. It is stated but not discussed.

(3) **Measurement claim, the main selling point.** With a reused impedance setpoint, a redesign's error in a measured pad effect tracks Δcontact rather than Δskel. This holds across two simulators, two contact models, four Drake tasks and three policies.
- The statistics are careful: permutation tests, Holm correction, partial correlations, pre-registration, and a self-noise floor.
- The correlation set is constructed. Twelve of the 18 redesigns are synthesized at prescribed Δcontact, "most at an extreme Δskel", so the decoupling is by design rather than representative.
- The fingertip tasks are weak (ρ 0.43–0.53, p≤0.043 uncorrected).
- The paper's own Sec. IV-B2 shows that preload is the dominant mechanism. CoDe presses 0.75× as hard, which is why its errors are negative. Kinematic re-referencing of the setpoint cuts the error from 42% to 14%, and force matching leaves 3–7%.
- This weakens the argument that "the correction belongs in the hardware". A per-step kinematic retargeting of the setpoint through the known redesign kinematics is a cheap software alternative, and the paper never tests it against a policy.

Further limits on the evidence:
- All evidence is simulation. Neither contact model deforms the pad: compliance is a contact stiffness scaled by 3 mm/h, or a hydroelastic layer. The "thicker, more compliant pad holds 41% less" headline is therefore a property of a contact-parameter model, not of a real compliant pad.
- No hardware exists beyond CAD, although the title frames this as the design of a robot hand.
- The headline numbers in the abstract and Fig. 1(d) (+48% vs −41%; CoDe −55%) do not obviously map onto Table I's compl. pad column (+125%, −14%). Readers will be unsure what is being compared.

Summary judgement: the idea is original and the analysis is elegant, and the simulation study is unusually rigorous for a hand-design paper. But the claim that the hardware principle fixes interface measurement is under-supported, for three reasons: no physical validation, idealized pad models, and an untested software alternative (setpoint retargeting) that the paper's own data suggest would recover most of the error. I lean reject, but could be persuaded.

REVIEW JSON:
```json
{
  "Summary": "The paper argues that when a robot finger is redesigned to carry a thicker contact interface (pad, tactile skin), it should preserve the motion of its contact surfaces (the contact-point path in the proximal-pad frame, called the contact Jacobian) rather than the motion of its skeleton (fingertip path). A finite-displacement pole analysis shows that the room between pads and their relative motion are both set by the PIP joint's pole. It gives a lower bound showing that no redesign preserves both contact and skeleton motion, and that a fixed axis above the pad mount cannot preserve contact motion from the same initial contact point. The proposed CoDe Hand moves the servos off the joints and uses a crossed four-bar PIP whose pole can sit near the pad faces. Design studies (GA/NSGA-II over four PIP articulations, 3-9 mm pads, 90-150 deg RoM) show that CoDe gets more pad coverage than a servo-at-joint finger and best preserves contact motion. In simulation (Isaac Lab/MuJoCo-Warp and Drake with point and hydroelastic contact, four tasks, three PPO policies), the error a redesign adds to a measured pad effect under the original impedance setpoint correlates with the change in contact motion and not with the change in skeleton motion, including with contact force held fixed. CoDe redesigns err about 3-41% versus 124-173% for coverage-only redesigns.",
  "Strengths": [
    "Novel and well-posed question: which kinematic property should be invariant when the contact interface of a hand changes, so that the interface's own effect can be measured. This is relevant to the growing practice of swapping tactile and compliant skins on learning-oriented hands.",
    "Compact, correct kinematic argument. The finite-displacement pole identity (Eq. 3) and the resulting trade-off bound (Eq. 4) give a crisp, reusable design principle and explain why a fixed axis fails.",
    "The design study is fairly broad: four PIP articulations including a remote-actuated revolute control that separates actuator relocation from articulation, four pad thicknesses and three RoM requirements, with multi-objective search and an explicit Pareto treatment of preservation against coverage.",
    "The simulation evaluation is statistically careful for a design paper: two simulators, two contact models, several tasks and pad changes, permutation tests with Holm correction, partial correlations controlling for skeleton deviation and contact force, a measured self-noise floor, and pre-registered analyses.",
    "Confounds are tested directly (matched contact force, re-referenced setpoint, sensed touch) instead of being ignored, and the limits are stated, e.g. that fingertip-end contact is outside the claim and that touch-sensing commands behave differently.",
    "The policy-transfer experiment (Fig. 5) gives an intuitive demonstration that the preserved quantity depends on where the task touches the finger."
  ],
  "Weaknesses": [
    "No physical validation. The hand exists only as CAD, and every result about measured interface effects comes from simulation. The claims concern how real compliant or tactile interfaces would be measured, so this is a major gap. The conclusion itself defers the hardware test.",
    "The pad models do not deform. Compliance is a scaled contact stiffness or a 0.1 MPa hydroelastic layer. The headline finding that a thicker, more compliant pad holds 41% less (and that Coverage reports +48%) is therefore about contact-parameter models, and may not transfer to real pads, whose deformation interacts with geometry.",
    "Preload is the dominant mechanism by the paper's own analysis (CoDe presses 0.75x as hard; re-referencing the setpoint cuts the error from 42% to 14%; force matching leaves 3-7%). This opens an obvious software alternative: retarget the setpoint per step through the known redesign kinematics, e.g. map the commanded joint target so that the contact points match O's. The paper dismisses software correction ('a learned policy issues a new setpoint at every step') without testing per-step kinematic retargeting, which would be cheap given the redesign kinematics are known.",
    "The correlation set is constructed rather than sampled: 12 of 18 redesigns are synthesized at prescribed Delta_contact, mostly at extreme Delta_skel. This makes the decoupling of the two predictors partly an artifact of the design of the set. Claims such as 'the error grows with Delta_contact and with nothing else' overstate what 18 designed points and a few candidate predictors can show.",
    "The fixed-axis bound (Eq. 5) depends on the assumption that the pin and lugs must sit above a full-width pad's mount. Fig. 3(c) appears to show at-joint (fixed-axis) designs above the bound at 90 deg, and remote designs well below it at 150 deg, while the text says the remote joint 'sits on the bound'. The reconciliation (presumably a moved initial contact point) is not explained.",
    "Evidence on the fingertip tasks is weak (rho 0.43-0.53, p<=0.043 uncorrected across tasks). Two 'other commands' columns in Table I (torque, touch) favour Coverage, so the scope ('commands that do not sense contact') is narrower than the abstract suggests.",
    "Inconsistent headline numbers: Fig. 1(d) and the abstract give CoDe -55% vs reference -41% and Coverage +48%, but Table I lists the compliant-pad errors as -14%/-8% (CoDe) and +125%/+347% (Coverage). The relation between these is not explained. 'Within 20 points' in Fig. 1(d) is also a different metric from the relative error used elsewhere.",
    "The four-bar's costs are under-discussed: worst-case mechanical advantage of 0.41-0.58, transmission-angle limits, backlash and clearance, manufacturability. Also, remote actuation alone already captures most of the room benefit at 90 deg and at <=5 mm.",
    "Policy experiments use two training seeds. In the cube task, seed 21 collapses to about 20% on all hands, which limits what Fig. 5 can show about that task.",
    "Related work omits the literature on mechanisms designed for a prescribed instantaneous centre or pole path (e.g. polycentric four-bar knees in prosthetics and exoskeletons) and the literature on cross-embodiment policy transfer and kinematic retargeting, both of which bear directly on the claims."
  ],
  "Originality": 3,
  "Quality": 2,
  "Clarity": 2,
  "Significance": 2,
  "Questions": [
    "How do the Fig. 1(d) numbers (CoDe -55%, Coverage +48%, reference -41%) relate to the Table I compliant-pad columns (-14%/+125% in MuJoCo-Warp; -8%/+347% in Drake hydroelastic)? Are they different placements, tasks or thicknesses?",
    "Fig. 3(c) shows at-joint (fixed-axis) designs above the fixed-axis bound at q_req=90 deg, and remote designs at -75 to -100% at 150 deg. Is this because the initial contact point moved? If so, why is Eq. (5) the right reference line, and what does the text mean by 'the remote-servo joint sits on the bound'?",
    "Have you tried a per-step kinematic retargeting of the policy or impedance setpoint on the Coverage and Skeleton redesigns, e.g. choosing the joint target that reproduces O's contact-point positions? If that recovers most of the error, what remains as the case for hardware-level preservation?",
    "How sensitive are the conclusions to modelling pad compliance as a deformable body (e.g. FEM or a soft-body layer) rather than a contact stiffness? Can you show at least one deformable-pad check?",
    "How was the set of 12 extra redesigns chosen, and would the correlation with Delta_contact hold on a randomly sampled set of feasible redesigns?",
    "In Eq. (2) and Eq. (6), L_f depends on q (L_f(M,q)) although L_f is defined as the prescribed extended length. What does the q-dependence mean?",
    "What exactly is the contact point c(q) in Eq. (7): a fixed material point on the distal pad, or the actual contact location? How is it chosen when the pad length changes between redesigns?",
    "What does the 'convex' column in Table I denote?",
    "Can the MCP joint (held at zero here) be treated by the same pole argument, and does preserving PIP contact motion alone suffice when the MCP flexes?"
  ],
  "Limitations": [
    "Simulation only, with non-deforming pad models; there is no hardware prototype test of the claimed measurement benefit.",
    "Planar analysis of the PIP only, with the MCP fixed; three-dimensional effects and multi-joint interaction are not covered.",
    "The claim is restricted to commands that do not sense contact and to contact on the pads; fingertip-end contact and touch-based commands are outside its scope.",
    "The crossed four-bar trades away mechanical advantage (0.41-0.58) and adds mechanical complexity, which may limit grip force and durability.",
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
1. Fig. 1 floats so that the abstract and the start of the introduction are split across the figure. In the text extraction the contribution list breaks mid-bullet ("when the interface" ... "deepens") across the page and figure, which hurts readability.
2. Fig. 1(d): "within 20 points of O's" uses percentage points of effect, while Table I uses relative error. Use one metric, or explain the mapping, and reconcile the -55%/+48% numbers with Table I.
3. Eq. (1) defines the pole as P(q), but Eq. (3) and later text use bold p(q). Pick one symbol.
4. Eq. (2): the phrase "pole at depth z(q) midway between their joint-side corners" is hard to parse. A small inset sketch, or one more sentence, would help. G_sym's subscript is never explained.
5. "Motion. In a frame F_p on the proximal contact surface, the point c(q) of the distal contact surface below the fingertip..." does not say which point c is. State explicitly whether it is a material point and how it is selected.
6. The paper calls Delta_contact the "contact Jacobian", but Eq. (7) defines a path deviation (a finite displacement), not a Jacobian. Either justify the term or call it the contact-surface path.
7. Eq. (6) writes L_f(M,q), but L_f is introduced as a prescribed constant extended length. Clarify.
8. Sec. III-D: "the CoDe runs spread by 1.3-3.4 points and are refined from neighbouring designs" is cryptic. Say what "refined" means and whether the other joints received the same treatment.
9. Fig. 3(c): label the y-axis definition explicitly in the caption, e.g. "kept = 1 - Delta_contact / fixed-axis Delta_contact". Explain the points above 0 for the at-joint finger and below 0 for remote, and the '×' markers.
10. Fig. 3(a), right panel, and the small numeric annotations under Fig. 4 (E, Delta, rho) are very small at print size. Enlarge them or move the numbers into a table.
11. Table I: define "convex", "compl. pad" and "setpoint" in the caption. Say which column is the setpoint re-referencing of Sec. IV-B2. Explain why bold marks CoDe even where it is worse (torque, touch).
12. The sentence "the compliant-pad columns judged against O carrying the same pad" is confusing, given that O is defined as the 3 mm hand. Clarify what the reference is for thickness or compliance changes.
13. Fig. 4 caption: "its measured starts are 3-6 mm from the grasp" and "16-18 correlated redesigns (both contact models, ranks averaged)" need plainer wording.
14. Fig. 5 legend: "seed 20 / seed 21" reads like an internal identifier. Use "seed 1 / seed 2". The "palm shift" quantity appears only in the caption and the last paragraph and should be defined in Setup.
15. "Analyses marked pre-registered had hands, statistic and criterion recorded, dated, before any redesign was run" is split across a figure. Also say where the pre-registration record is available.
16. Results 1 opens with "The error grows with Delta_contact and with nothing else." Soften this to a statement about the tested predictors.
17. Related Work is one paragraph. Add pole-path or instantaneous-centre mechanism design (e.g. polycentric knees) and cross-embodiment transfer or retargeting.
18. Remove the hyphenation artifacts in the extracted text ("contactsurface", "fourbar", "Rereferencing").
19. The conclusion says "near 20% up to 7 mm", but the Isaac Lab numbers are 3% and 21% at 5 and 7 mm, and Table I ranges up to -43% (Drake point, convex). State the range honestly.
