ASSESSMENT NOTES:

The paper argues that when a robot finger is redesigned to carry a thicker contact interface (pad or skin), the redesign should keep the motion of the contact surfaces ("contact Jacobian"), not the motion of the skeleton or fingertip. If it does, an unchanged impedance command presses the object the same way, so the interface's own effect can be measured. It makes four claims:

(C1) Kinematic analysis based on the joint's pole (Eqs. 1–5). Contact motion and skeleton motion cannot both be preserved: Δcontact + Δskel ≥ 2 sin(q_req/2)|Δt|. A fixed axis above the pad mount cannot preserve contact motion from the same initial contact point. A crossed four-bar can place its pole near the contact faces.
- The derivation is short and, as far as I can check, correct. Eq. 3 is the standard chord relation for a rotation about a displaced pole. Eq. 4 is the triangle inequality on the frame offset Δt.
- The results are elementary, but they frame the problem cleanly and Fig. 2(a) makes them concrete.
- The bound assumes the same initial contact point and only a pure normal deepening of both surfaces. The paper says so, but how often real redesigns satisfy this is not discussed. Eq. 7 deliberately penalizes moved initial points, which is consistent with the assumption.

(C2) The CoDe finger gives more room for thick pads than a servo-at-joint finger and preserves contact motion best.
- Supported by GA/NSGA-II studies over 4 joints × 4 thicknesses × 3 ranges of motion (Fig. 3).
- Much of the "room" benefit comes from remote actuation alone: it is equal at 90° and up to 5 mm at 120°. The crossed four-bar's specific advantage shows mainly at 150° and in preservation (Fig. 3(c)).
- The optimizer spread for CoDe is 1.3–3.4 points, comparable to several of the margins reported. Worst-case mechanical advantage of 0.41–0.58 is a real cost and is mentioned only in passing.

(C3) The error a redesign adds to a measured pad effect tracks Δcontact, not Δskel. This is the core empirical claim.
- The evidence is substantial for a simulation study: 18 Isaac Lab redesigns with permutation tests and Holm correction, and partial correlations controlling for Δskel and for force deviation. Four Drake tasks in two contact models, a pre-registered 27-redesign grid, and a 12-setting comparison in Table I.
- However:
  - (i) All results are in simulation. Neither contact model deforms the pad: "compliance" is a scaled contact stiffness or a hydroelastic layer. So the "thicker, more compliant pad" effect, which is the headline number (+48% vs −41%), is a modeling construct.
  - (ii) The correlation set was built by design: 12 redesigns were held to prescribed Δcontact, most at an extreme Δskel. That raises a concern that the observed ρ partly reflects how the set was sampled.
  - (iii) The fingertip tasks correlate weakly (ρ 0.43–0.53), and the cross-task p-values are uncorrected.
  - (iv) "The error grows with Δcontact and with nothing else" overstates what correlational evidence on 18 designs can show.
  - (v) CoDe still errs by 21% at 7 mm and 41% at 9 mm.

(C4) Policies trained on O transfer best to CoDe on a pad-contact task. Finger walking keeps 61–72%, against under 10% for Skeleton and Coverage.
- The paper is honest that the result is task-dependent: Skeleton wins the fingertip dial, and the whole-hand cube depends on palm shift.
- There are only two seeds, and the cube results differ strongly between them.

Main concern for significance: the re-referencing result (Sec. IV-B2). Re-referencing the setpoint from kinematics alone cuts the median error from 42% to 14%. For a 1-DoF PIP, a monotone joint-space remap q′ = f(q) could be applied to every command a policy issues, at every step. The argument that "the correction belongs in the hardware because a learned policy issues a new setpoint at every step" does not rule this out. A software baseline (a per-step kinematic retargeting that matches the contact-surface pose, possibly also the commanded stiffness) is the natural competitor and is missing. Without it, the practical need for hardware-level contact-centric redesign is not established.

The problem framing (measuring an interface's effect without confounding it with embodiment) is also somewhat niche. The paper does not show that practitioners need it, beyond the motivation in the introduction. There is no hardware.

Presentation is dense but disciplined. Numbers in Fig. 1(d) and Table I are hard to reconcile (see items below). Naming one hand "Coverage" collides with the coverage metric.

Overall: a thoughtful, honest, well-controlled simulation study with a clean kinematic principle. Its evidential and practical weaknesses (simulation only, non-deforming pads, a missing software-retargeting baseline, modest residual error at 7–9 mm) keep it at borderline. I lean reject.

REVIEW JSON:
```json
{
  "Summary": "The paper proposes contact-centric design (CoDe) for robot fingers that carry exchangeable contact interfaces (pads, tactile skins). When a finger is redesigned for a thicker interface, it should preserve the motion of its contact surfaces (the contact-point path in the proximal contact-surface frame) rather than the skeleton/fingertip path. A finite-displacement pole analysis shows that the two motions cannot both be preserved (Delta_contact + Delta_skel >= 2 sin(q_req/2)|Delta_t|), that a fixed axis above the pad mount cannot preserve the contact motion from the same initial contact point, and that a crossed four-bar PIP can place its pole near the contact faces. Design studies with GA/NSGA-II over four PIP articulations (at-joint servo, belt-driven remote servo, rolling contact joint, crossed four-bar) show that the CoDe finger covers more pad surface than the at-joint finger wherever both are feasible and preserves contact motion best. In Isaac Lab (MuJoCo-Warp) and Drake (point and hydroelastic contact), across four tasks and several pad changes, the error a redesign adds to a measured pad effect under the original impedance setpoint correlates with Delta_contact rather than Delta_skel. This also holds with force deviation partialled out and after kinematic re-referencing. Policies trained on the original hand transfer best to the CoDe redesign on pad-contact tasks.",
  "Strengths": [
    "Clear and original framing: kinematic preservation is stated in the contact-surface frame rather than the skeleton frame. A simple pole-based inequality (Eq. 4) makes the trade-off between contact and skeleton motion explicit and proves the limit of fixed-axis joints (Eq. 5).",
    "Fig. 2(a) concretely shows which joint types can place their pole near thick pads, linking the abstract analysis to the choice of mechanism.",
    "The design studies are systematic: four articulations, four pad thicknesses, three ranges of motion, a shared actuator and finger length, and a remote-servo control that separates the benefit of moving the actuator from that of the articulation.",
    "The empirical study is unusually careful for a design paper: two simulators, two contact models, four tasks, permutation tests with Holm correction, partial correlations controlling for Delta_skel and for force deviation, a noise floor (O against itself), and pre-registered analyses on a 27-redesign grid.",
    "The authors test alternative explanations (preload, re-referenced setpoint, matched contact force, sensed touch) instead of only reporting favourable cases.",
    "Results that do not favour the method are reported honestly: Skeleton wins the fingertip-end dial, torque/touch columns in Table I, 41% error at 9 mm, and low mechanical advantage.",
    "The headline example (a redesign reporting that a pad holds +48% more when it holds 41% less) shows concretely why the problem matters for interface research."
  ],
  "Weaknesses": [
    "All validation is in simulation, and neither contact model deforms the pad. Compliance is a scaled contact stiffness or a hydroelastic layer. The central claims about measuring a 'thicker, more compliant pad' therefore rest on a modeling proxy whose fidelity for thick soft interfaces is not established. No hardware result is given.",
    "A software baseline is missing. Kinematic re-referencing of the setpoint alone cuts the median error from 42% to 14%. A per-step joint-space remap (q' = f(q) chosen to match the contact-surface pose, possibly with a stiffness rescale) could be applied to every command a learned policy issues. The argument that the correction 'belongs in the hardware' is therefore not demonstrated.",
    "The correlation set is partly constructed: 12 of the 18 redesigns are held to prescribed Delta_contact, most at an extreme Delta_skel. This may inflate the separation between the two predictors. The statement 'the error grows with Delta_contact and with nothing else' is stronger than correlational evidence over 18 designs supports.",
    "The residual error is still large: CoDe errs by 21% at 7 mm and 41% at 9 mm in pull-out. The fingertip tasks correlate weakly with Delta_contact (rho 0.43-0.53), and their cross-task tests are uncorrected.",
    "The room advantage is largely due to remote actuation, not the crossed four-bar: the remote servo matches CoDe at 90 deg and up to 5 mm at 120 deg. Optimizer run-to-run spread (1.3-3.4 points) is comparable to some reported margins.",
    "The analysis is planar and PIP-only, with the MCP held at zero. Whether preserving contact motion at one joint suffices for multi-joint and whole-hand contact is shown only indirectly (cube rotation, which depends on palm shift instead).",
    "Policy-transfer evidence uses two training seeds, and the cube-rotation results vary strongly between them.",
    "The design gives up mechanical advantage (worst case 0.41-0.58), with implications for grasp force, and this is not discussed."
  ],
  "Originality": 3,
  "Quality": 2,
  "Clarity": 2,
  "Significance": 2,
  "Questions": [
    "Have you compared against a per-step kinematic retargeting of O's commands to the redesign, i.e., a joint map that matches the contact-surface pose and optionally the effective stiffness at the contact? If re-referencing a single setpoint already cuts the error to 14%, what does a per-step version achieve, and what residual error justifies the hardware redesign?",
    "How sensitive are the main correlations to the way the 12 prescribed-Delta_contact redesigns were sampled? What do the correlations look like on the 27-redesign grid or on a randomly sampled set of feasible redesigns?",
    "Can you reconcile Fig. 1(d) with Table I? Fig. 1(d) gives CoDe -55% against a reference of -41% (a relative error of about -34%) and Coverage +48% against -41% (about +217%), while Table I's compl. pad column gives -14% and +125%. Fig. 1's caption speaks of '20 points'. Which reference and which units (absolute points or relative error) does each use?",
    "What does the 'convex' column in Table I denote?",
    "How would a deformable (FEM) pad model, or a hardware test, change the compliant-pad results? Is there any evidence that scaling contact stiffness by 3 mm/h captures the effect of a thicker pad on grasp mechanics?",
    "Does the bound in Eq. 4 extend to redesigns that change the pad thickness of only one phalanx, or that change pad length or position as well as thickness?",
    "How does the CoDe redesign's mechanical advantage compare with O's, and does the force deficit (0.75x preload at 7 mm) limit which interfaces can be evaluated?",
    "Why is 'contact Jacobian' the right name, when the quantity actually used (Delta_contact) is a maximum deviation of a point path, not a Jacobian?"
  ],
  "Limitations": [
    "Simulation only. The pads are rigid bodies with modified contact parameters, so effects of a soft, thick pad's deformation, hysteresis and tactile response are not captured.",
    "The analysis is planar and restricted to the PIP, with the MCP held at zero. Multi-joint and out-of-plane contact are not analyzed.",
    "The claim covers commands that do not sense contact. Fingertip-end contact is explicitly outside it, and Skeleton performs better there.",
    "The method trades mechanical advantage and design complexity (a crossed four-bar and an off-joint servo) for preserved contact motion.",
    "No negative societal impacts are apparent beyond those of general dexterous-manipulation research."
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

1. Fig. 1(d) and Table I use different quantities without saying so. Fig. 1(d) shows the measured effect (-55% vs. reference -41%; "83% of the reference's"), the caption speaks of "within 20 points", and Table I gives relative error (-14%, +125%). State one metric, or label each one explicitly, so the reader can check +48% / -41% against Table I.
2. The hand named "Coverage" collides with the coverage metric C̄_pad and with "Coverage evaluation" in Fig. 3(b). Sentences such as "Coverage, designed for coverage alone" and "coverage within 0.2 points" are hard to parse. Consider a distinct name (e.g., "Max-Cov").
3. "Contact Jacobian" is introduced as the name of the preserved quantity but is never defined as a Jacobian. Δcontact is a maximum point-path deviation. Either define the Jacobian and show how Δcontact relates to it, or drop the term.
4. The frame F_p is not fully specified: its origin and orientation on the proximal contact surface, and how it moves when the pad length or position changes. Eq. 4's "F_p moves by Δt relative to F1" relies on this.
5. Eq. 2: "pole at depth z(q) midway between their joint-side corners" is hard to follow without a sketch. The factor 3.46 (= 2 tan 60°) should be stated as such.
6. Table I: the "convex" column is never defined. The compl. pad column for "Drake, point" is missing without explanation. "Other commands" mixes simulators; the parenthetical in the text ("the contact setpoint runs in Drake, the other commands in MuJoCo-Warp") belongs in the table caption.
7. Sec. III-D reports results (Room, Preserved motion) inside the Design section, before the Experiment section. Consider moving them, or call Sec. III-D "Design Results".
8. The cross-reference to Sec. III-D2 is unclear: "the redesign of Sec. III-D2" refers to a numbered paragraph. Also, the selection rule "the front's highest-coverage design" is in Sec. III-C, not III-D2.
9. Fig. 3(c): "contact motion kept (%)" relative to the fixed-axis bound needs a formula, such as 100·(1 − Δcontact/bound). Values below −100 and the remote-servo points pinned at 0 are otherwise hard to read.
10. Fig. 4: the per-task metric E ("median |error|") is defined only in the text. The caption should say which outcome each task's error is computed on (load at failure, drawn-in distance, dial angle, tilt). The small labels "E point / E hydro / ρ" below the plots are hard to read at print size.
11. Fig. 5: the x-axis labels under each group differ (Δcontact, Δskel, palm shift) but share one row, which reads as one axis. Separate or annotate each group.
12. Some sentences are compressed enough to obscure meaning: "Preload sets the sign of the error; the geometry matters beyond it", "closing at sensed touch leaves an error that follows the skeleton", and "the acquisition drawn from the distal pad; its measured starts are 3–6 mm from the grasp". Expand them, or state the experiment explicitly.
13. The Introduction defines O, CoDe, Skeleton and Coverage at the end of the contributions list, but Fig. 1 uses them first. Introduce them before Fig. 1 or in its caption.
14. The Related Work section is very short. It omits prior work on kinematic retargeting and transfer across embodiments (the natural competitor) and on contact-frame or contact-kinematics analysis of fingers.
15. The meaning of "best of six kept" and "refined from neighbouring designs" (Sec. III-D) is unclear. State whether CoDe receives more optimization effort than the other joints.
16. The abstract claims the result holds "across two simulators, two contact models and four tasks". The four tasks are in Drake only, and Isaac Lab runs pull-out only. Rephrase to avoid implying a full cross product.
