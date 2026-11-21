# Playground: Study 3 설계와, SSoT에서 옮긴 미확정 내용

2026-11-21에 `design/SSoT/`에서 옮겼다. 확정되지 않은 가설과 진행 중인 설계라서 SSoT에 두지 않는다. 결과가 검증되고 합의되면 다시 SSoT로 제안한다. 아래는 옮긴 원문 그대로이며, 그 뒤의 변경(예: friction 실험, 조건 A/B)은 `separability.md`에 있다.

## research-objective.md, Section 8.1-8.3 (Study 3 design)

### 8.1 Design

- **Interface change.** A thin rigid flat pad (C₀) to a thicker rigid flat pad (C₁) of the same geometry and material. On the contact face nothing changes, so the interface's own effect on the task is zero by construction (Section 2); any measured change is embodiment.
- **Hands**, all of one architecture (the CFL), so that the only difference between the two redesigns is the redesign target:
  - **H₀:** the finger designed for C₀ (Study 1).
  - **H_R:** H₀ redesigned for C₁ with the relative contact trajectory as the target (Study 2, small D_c).
  - **H_U:** the finger designed for C₁ for coverage alone (Study 1, larger D_c), at comparable coverage.
- **Measurement.** Policies trained on (H₀, C₀) are run, without retraining, on (H_R, C₁) and (H_U, C₁). With J the task's pre-registered metric,
  ΔJ_R = J(π, H_R, C₁) − J(π, H₀, C₀),  ΔJ_U = J(π, H_U, C₁) − J(π, H₀, C₀),
  paired over policy seeds.
- **Prediction.** |ΔJ_U| > |ΔJ_R|. Neither is expected to be exactly zero: a thicker pad moves every contact face away from the MCP axis and the palm by the same amount on both hands, which the comparison cancels.
- **Refutation.** ΔJ_R and ΔJ_U are equal within the stated precision. Then keeping the relative contact trajectory is not what such an experiment needs, and that is reported.
- **Statistics** (fixed before running, Section 7): at least 10 policy seeds; the primary test, its α and the precision that counts as "equal" are recorded in `docs/decisions.md` before the hands are evaluated.

### 8.2 Preconditions (checked before any training)

1. **The task exercises the difference.** D_c is the largest change over [0, q_req]. The flexions the task's policies actually use must include those where H_R's and H_U's trajectories part (Study 2's per-flexion profile); otherwise an equal measurement says nothing about the trajectory.
2. **The comparison is clean.** H_R and H_U differ in D_c by a clear margin and in coverage by little. Other changes between them (transmission from servo angle to flexion, finger outline, pad mass) are recorded. Actions are defined in flexion so that a transmission change does not enter the comparison.
3. **The hands are valid in simulation** (closed loops, contact, object equilibrium), as in the archived Gate 0.

### 8.3 Choices (2026-11-21)

- **Condition:** CFL, q_req = 120°, 3 → 7 mm (changed from 90°, 3 → 5 mm on 2026-11-21). D_c is 0.86 mm on H_R and 3.91 mm on H_U at similar coverage (0.725, 0.711); at H0's two-pad cylinder grasps the distal face is off by at most 0.33 mm on H_R and 3.10 mm on H_U (medians 0.12 and 2.01 mm), over the whole flexion range rather than only in deep flexion; and the transmission changes alike on both (dθ/dq by about 20%, MA_wc 0.520 and 0.479 against H0's 0.583), so the comparison is cleaner than at 90° (there MA_wc moved −8% and +23%). 120° is also the simulated hand's PIP range.
- **H_U is one fixed design:** Study 1's first independent coverage-only design for 7 mm at 120° (its D_c depends on the search: the Study 1 design Study 2 compared against has 1.28 mm, this one 3.91 mm; the choice is recorded, and a less favourable U would narrow the contrast).
- **Actions and observations in flexion q**, mapped to each hand's servo angle by its θ(q) table. Sent as servo angles, the 3 mm finger's commands bend H_R and H_U off by about the same amount (21.0° and 22.0° at 120°, 7 mm), which would hide the trajectory difference (`docs/decisions.md`, transmission entry).
- **Stiffness matched in flexion:** per-flexion PD gains so that the effective joint stiffness is equal across hands; the transmission ratio dθ/dq differs from H0's by up to 20% (H_R) and 21% (H_U) and MA_wc by −11% and −18%. Residual torque-limit differences are recorded.
- **Interface: compliant pads**, normal contact stiffness k_pad = 2 N/mm as the reference (1 and 5 N/mm as a sensitivity check), the same on every hand and both thicknesses. With rigid pads an offset of a few tenths of a millimetre already loads the servo heavily (quasi-static estimate), so the comparison would be decided by the simulator's contact softness; with k_pad 2 N/mm the force error at H0's grasps stays at or below 0.6 N on H_R and reaches 5.7 N on H_U at 120°, 7 mm (`report/study3/regrasp_geometry.md`). The simulated stiffness is checked in Gate 0.
- **Task:** two-pad grasps. Precondition 1 checked geometrically (`report/study3/regrasp_geometry.md`): at 120°, 7 mm the difference holds over the whole flexion range of the grasps (H_R at most 0.33 mm, H_U 3.10 mm). At 90°, 5 mm it was confined to flexions of 70° and above, one reason for the change of condition.

## research-objective.md, Section 6, claim 4

4. **Keeping the relative contact trajectory keeps an interface measurement interpretable** (hypothesis, Study 3; Section 8). The same interface change, measured on a hand redesigned to keep the relative contact trajectory and on one redesigned without that target, gives different measurements, and the first is the closer to the interface's true effect.

## research-objective.md, Section 9, evidence table for "why the relative contact trajectory"

**Evidence for "why the relative contact trajectory"** (the five steps in `north-star.md`; status 2026-11-21):

| Step | Evidence | Status |
|---|---|---|
| 1. The object meets the contact faces | Definition (Section 2) | Argued |
| 2. Only the trajectory cannot be undone by control | Transmission: flexion-space commands and stiffness scheduling hold flexion within 0.07 deg on the simulated hand (`docs/decisions.md`). Trajectory: problem_formulation proposition 4; re-establishing a two-pad cylinder grasp moves the contact up to 5-9 mm along the distal pad (`report/study3/regrasp_geometry.md`). MCP-level offset: compensability not yet computed | Partly shown; the P/D compensability computation (residual contact error after the best re-command, free and fixed object) is next |
| 3. Its size matters | Quasi-static series-compliance estimate at 120 deg, 3 to 7 mm, k_pad 2 N/mm: force error up to 0.6 N (trajectory kept) against 5.7 N (coverage only), grasp forces 1-5 N | Estimate only; to be measured (quasi-static simulation, then a second engine) |
| 4. Not kept unless targeted | Study 1's coverage-only designs change it by 3.9-5.8 mm (1.28-3.91 mm at 120 deg, 7 mm, depending on the search) | Shown (2D) |
| 5. Kept at little cost | Study 2: 10-51% of the fixed-axis bound, coverage within 0.2 points; C1 | Shown (2D) |

## north-star.md, "Why the relative contact trajectory", steps 2-3

2. **It is the part of an embodiment change that control cannot undo.** A changed transmission is undone by mapping commands through the new servo-to-flexion relation; a rigid offset of the whole finger is carried by the object or largely taken up at the MCP. With one degree of freedom between the two faces, their relative position at a given relative rotation is fixed by the mechanism: no command restores it, and re-establishing a contact moves it along the pad.
3. **Its size matters.** On a compliant interface, an offset of a few millimetres in it changes the contact forces of the same commanded grasp by as much as the forces of the grasp itself; a redesign that keeps it changes them by much less.

## north-star.md, status line of the argument

Steps 1, 4 and 5 rest on the design studies; step 2 on the mechanism's kinematics and is to be quantified; step 3 is so far an estimate and is what C4 must measure (`research-objective.md` Section 9).

## north-star.md, C4

- **C4 — Why it matters.** A change of the relative contact trajectory cannot be undone by control and changes what an interface experiment measures: the same interface change, evaluated on a hand redesigned to keep the trajectory and on one redesigned without that target, gives different results.

## research-objective.md, Section 8 claim boundaries (design-specific bullets)

- The claim is about frozen-policy measurements of an interface change. Whether independently trained policies show the same is not answered.
- Rigid interfaces in simulation only; compliant and tactile interfaces are not simulated.
- (MCP bullet, design-specific end) in Study 3 the offset is common to H_R and H_U and cancels in their comparison.
