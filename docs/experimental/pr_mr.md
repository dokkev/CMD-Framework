# PR and MR: which reference the contact path is anchored to (2026-11-26)

Experimental (playground). Not in the SSoT or the paper.

## Idea

- A redesign for a thicker pad keeps the contact Jacobian: the rate at which the distal contact face's point under the fingertip moves with the PIP flexion.
- The point's path also carries a constant, its initial position. That constant has to be anchored to something.
  - **PR, proximal-face referenced:** the path seen from the proximal contact face. This is Δ_contact, the paper's quantity. PR is the hand called R or H_R in earlier notes.
  - **MR, MCP referenced:** the path seen from the MCP axis, in the proximal link frame F₁. Its deviation is Δ_MCP.
- **Why MR matters with the MCP moving.** An MCP rotation turns F₁ rigidly, so Δ_MCP is the distal pad's deviation at every MCP angle. A task that touches the object with the distal pad only, while the MCP moves, should follow Δ_MCP. A task that holds the object between the two pads should follow Δ_contact.
- **Why one hand cannot have both.** The two references differ by the proximal face's depth change Δt, which is constant. So at every flexion |Δc_face(q)| + |Δc_F1(q)| ≥ |Δt|, and Δ_contact + Δ_MCP ≥ |Δt|.

## Existing hands (middle finger, recomputed from the Isaac exports)

| hand | Δt | Δ_contact | Δ_MCP | Δ_skel |
|---|---|---|---|---|
| PR (7 mm) | 4 | 0.86 | 4.65 | 7.67 |
| U (7 mm) | 4 | 3.91 | 4.26 | 3.38 |
| PR5 / U5 | 2 | 0.35 / 1.58 | 2.10 / 2.16 | 3.55 / 2.02 |
| PR9 / U9 | 6 | 2.32 / 3.89 | 6.36 / 6.11 | 9.37 / 7.17 |

(mm)
- Every 7 mm hand of the attempt-9 grid has Δ_MCP ≥ 3.88. Almost all of it is the initial offset at q = 0: the pads are coplanar at extension, so the distal face sits Δt deeper than O's.
- In the studies, pivot B lies on the link line (b_z = 0), so the distal face cannot be raised.
- Script: `scratchpad rplus/abs_paths.py`. The e20 script reproduces PR and U: 4.65 and 4.26.

## Pre-registration: the MR redesign (before the run)

- **Script.** `design/scripts/study/exploratory/e20_mcp_referenced.py`.
- **Condition.** 120 deg, 3 → 7 mm, CFL, the study's goal (`one_finger.yaml`).
  - The only change is that B's height b_z is searched in [−2, 8] mm. In the studies it is 0.
- **Search.** NSGA-II, three objectives: Δ_MCP, Δ_contact, −C̄_pad. Seeds 0 and 1, population 200, 200 generations, warm-started from PR and U.
- **Picks.** Among front designs with C̄_pad ≥ 0.70 (PR 0.725, U 0.711):
  - MR: the least Δ_MCP;
  - MR-mid: the least max(Δ_MCP, Δ_contact).
- **Probe before the run.** Raising B and B′ by 2 mm on PR gives Δ_MCP 4.35 and Δ_contact 2.00, at C̄_pad 0.702. A hand edit does not get there.
- **Predictions.**
  1. MR reaches Δ_MCP ≤ 2.0 mm at C̄_pad ≥ 0.70, with b_z > 1 mm.
  2. MR's Δ_contact is ≥ 2.0 mm, since the bound is Δ_contact + Δ_MCP ≥ 4.
  3. The front lies near the bound. At its least-sum point, Δ_contact + Δ_MCP ≤ 6 mm.
- **If prediction 1 fails** (no design under 2 mm), MR is not buildable in this design space. Then the PR/MR task test runs on the best MR found, and its Δ_MCP is reported as it is.

## Next: task validation (pre-registered separately, once the hands exist)

- **Drake.**
  - Fingertip dial and block tipping (distal pad), with the MCP moving: MR should err less than PR.
  - Pull-out between the pads, MCP held: PR should err less than MR.
- **Isaac RL.** O policies transferred to PR, MR and U, zero-shot:
  - cube rotation (MCP and fingertips): MR should transfer better than PR;
  - 7b′ rolling on the pads: PR should transfer better than MR.

## Observation: the Drake fingertip tasks are PR-anchored by construction (2026-11-26, before any new run)

- The paper's fingertip tasks place the object in each hand's proximal-face frame: the dial, the pivot and the coin. The source says "the same coordinates on every hand" (`sim/drake/dial_placements.py`, `box_pivot.py`, `coin_drag.py`; world z = z_face − face).
- So on a thicker-pad hand, the environment is moved by Δt along with the proximal face. That choice is the PR reference.
- Placing the same objects at O's world coordinates, fixed to the MCP axis and palm, is the MR reference. With the MCP held, F₁ is the world. A distal-pad-only task with the object fixed to the palm should then follow Δ_MCP, whatever the MCP does.
- The Isaac 7b′ starts are also set in each hand's face frame, which is PR.
- **Correction (2026-11-27, before any MR result; found by the RL agent).** The cube rotation is face-anchored too. `pinch_grasp` sets the cube's rest position in each hand's proximal-face frame at MCP 35 deg, on the proximal face. Each hand also has its own held-out grasp cache (`gen_grasps.py`, seed 76000).
  - So my earlier note that the cube was palm-frame and MR-like was wrong.
  - RL prediction 1 (cube: MR beats PR) was registered on that wrong premise. It is kept as registered and is judged as it stands.
  - Under face anchoring, the corrected expectation is PR ≈ MRmid ≥ MR on the cube. This is noted, not registered.

## Pre-registration: Drake fingertip tasks anchored at F₁ (before the run)

- **Tasks.**
  - The fingertip dial: in-plane `tipdial` y, the paper's starts.
  - The block pivot.
  - Point contact first. Hydroelastic only if the point result is clear.
- **Hands.** The hands each task ran on in the paper, plus MR and MR-mid once built.
- **Change.** One option: objects placed at O's coordinates in F₁ for every hand (anchor f1), instead of each hand's face frame (anchor face, the paper's runs). Everything else is unchanged: commands, gates, friction swap 1.3 → 2.0, the error E.
- **Predictions.**
  1. Anchor f1: E follows Δ_MCP (one-sided permutation p < 0.05). Its |ρ| exceeds that of Δ_contact.
  2. Anchor f1: PR's E is no smaller than U's. Both have Δ_MCP ≈ 4.3–4.7.
  3. Anchor f1: MR (if it reaches Δ_MCP ≤ 2) errs least of PR, U, MR.
  4. Anchor face: the paper's result stands (E follows Δ_contact). With MR added, PR errs less than MR.
- **If prediction 1 fails** (E still follows Δ_contact at f1), the face-anchored result is not an artefact of the placement. The MR idea then has no task support in Drake.

## Result: the MR redesign (2026-11-26)

**Run 1: only B's height free** (as registered; `design/output/study/exploratory/e20_mcp_referenced_bz/`, 2 × 40,000 evaluations, about 2 min).

| hand | Δ_MCP | Δ_contact | Δ_skel | C̄_pad | MA_wc | b_z | face P/D |
|---|---|---|---|---|---|---|---|
| PR | 4.65 | 0.86 | 7.67 | 0.725 | 0.52 | 0 | 10/10 |
| U | 4.26 | 3.91 | 3.38 | 0.711 | 0.48 | 0 | 10/10 |
| MR | **1.66** | 3.47 | 4.34 | 0.701 | 0.58 | 3.4 | 10/10 |
| MR-mid | 2.10 | 2.08 | 5.22 | 0.704 | 0.53 | 2.2 | 10/10 |

- All three predictions held:
  1. Δ_MCP ≤ 2 at C̄_pad ≥ 0.70, with b_z > 1;
  2. Δ_contact ≥ 2;
  3. the least-sum point is within 6 mm (4.18, against the bound of 4).

**Run 2: A's and B's heights both free, joint-axes alignment off** (user, 2026-11-26: "the pads and skeleton need not align at the extended pose"; `.../e20_mcp_referenced_free/`; a_z in [−3, 8], b_z in [−3, 10]).

| hand | Δ_MCP | Δ_contact | Δ_skel | C̄_pad | MA_wc | a_z, b_z | proximal mount | face P/D |
|---|---|---|---|---|---|---|---|---|
| MR | **0.54** | 3.26 | 4.29 | 0.700 | 0.55 | 1, 4.3 | 2 | 9/10 |
| MR-mid | **1.06** | **1.07** | 4.78 | **0.735** | 0.52 | 3.5, 2.8 | 0 | 7/10 |

- **Raising A lets the proximal pad mount shallower.** Under the shallowest-supported mount policy, the proximal mount offset drops from 3 to 0 when A's pin housing sits above the link line. The proximal face then moves only 1 mm deeper than O's, against 4 mm before.
- The bound is Δ_contact + Δ_MCP ≥ |Δt|, where Δt is the proximal face's depth change: 1 mm for MR-mid, 3 mm for MR. Both are consistent with it.
- **MR-mid keeps both references within about 1 mm, at more coverage than PR** (0.735 against 0.725).
  - The lever the paper does not use is the face depth change itself. Freeing A's height shrinks it, so a thicker pad need not move the proximal face.
  - This is the "mount compensation" idea from the RL notes: the skeleton moves dorsally instead of the face moving palmar.
- **Caveat.** A proximal mount offset of 0 puts the pad's mounting surface on the link line. The packaging checks pass, but the PIP servo and A's housing clearance should be checked in CAD before reading much into it.

## MR hands (2026-11-26)

- `design/scripts/study/exploratory/e20_make_hands.py` writes `design/output/sim/study3/hands/cfl_q120_h3to7_{mr_bz,mrmid_bz,mr_free,mrmid_free}.yaml` by Study 3's mapping. The hand model's tip and contact paths match the designs to 3e-14 mm, with a_z and b_z nonzero.
- Isaac exports (`design/scripts/export/code_hand_sim.py`) are in `sim/isaaclab/logs/finger_walk/hands/`. The names do not match attempt 9's grid glob, so attempt 9 is unaffected.

## Pre-registration: RL transfer to MR (before any MR rollout)

- **Policies.** O's, unchanged, zero-shot:
  - the cube-rotation release policy (attempt 1, OmniReset v2c seg2 `model_1497`);
  - the two 7b′ closed-loop walk policies (seeds 20 and 21, attempt 9's checkpoints).
- **Hands.** O, PR (= R), U, MR_free, MRmid_free, plus MR_bz and MRmid_bz.
- **Measures.**
  - The transfer ratio: the hand's score over O's.
  - The friction-swap error E (pad μ 1.3 → 2.0).
  - Protocols, eval seeds and env counts are as in attempts 1 and 9.
- **Predictions.**
  1. Cube rotation, where contact is mostly fingertips and the MCP moves: MR_free and MRmid_free transfer better than PR and U, on the ratio and on E.
  2. 7b′, where the cylinder rolls on the pads with starts in each hand's face frame (PR-anchored): MRmid_free ≈ PR > MR_free on the ratio. MR_free is no better than U.
  3. MRmid_free is within 0.1 of O's ratio on both tasks. It is the only hand predicted to do well on both.
- **Kill rule.** If O itself fails to reproduce its attempt-1/9 score within ±5%, stop and fix the setup before reading any hand.

## Result: MR at 5 and 9 mm, A and B heights free (2026-11-26)

`design/output/study/exploratory/e20_mcp_referenced_free_h{5,9}/`

| t | hand | Δ_MCP | Δ_contact | Δ_skel | C̄_pad | proximal face (O: 6) | a_z, b_z |
|---|---|---|---|---|---|---|---|
| 5 | PR | 2.10 | 0.35 | 3.55 | 0.753 | 8 | 0, 0 |
| 5 | U | 2.16 | 1.58 | 2.02 | 0.748 | 8 | 0, 0 |
| 5 | MR | 0.11 | 0.99 | 2.09 | 0.750 | 6.9 | 1.1, 2.2 |
| 5 | **MR-mid** | **0.28** | **0.28** | 2.02 | **0.763** | **6.0** | 2.0, 1.8 |
| 7 | **MR-mid** | **1.06** | **1.07** | 4.78 | **0.735** | 7.0 | 3.5, 2.8 |
| 9 | PR | 6.36 | 2.32 | 9.37 | 0.695 | 12 | 0, 0 |
| 9 | U | 6.11 | 3.89 | 7.17 | 0.687 | 12 | 0, 0 |
| 9 | MR | 1.47 | 3.99 | 6.98 | 0.700 | 9.0 | 3.3, 7.2 |
| 9 | MR-mid | 2.39 | 2.43 | 7.00 | 0.707 | 9.0 | 3.0, 3.7 |

- **What limits it is the proximal face depth, which cannot go below the pad thickness.** The mount offset is ≥ 0, with no spacer and no pocket below the link line. O's face is at 3 + 3 = 6. So the face moves by at least Δt_min = max(0, h − 6): 0 at 5 mm, 1 at 7 mm and 3 at 9 mm.
- In every row, Δ_contact + Δ_MCP stays above that Δt_min: 0.56 ≥ 0, 2.13 ≥ 1, 4.82 ≥ 3.
- Up to h ≈ t_O, a redesign can keep the proximal face where O's is and both references with it. That holds for 5 mm (both deviations at 0.28) and nearly for 7 mm. Beyond it, the remaining face shift must be split between the two references.
- In every case MR-mid has at least PR's coverage.
