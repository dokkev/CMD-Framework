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

## Results: Drake fingertip tasks anchored at F₁ (2026-11-27)

**Setup.** As pre-registered, with one addition. The pre-registration named the in-plane dial (`tipdial` y). The paper's dial is the z disk (29z), so both were run. Both use experiment 29's 64 draws (seed 29) and `run_e29.sh`. The pivot uses experiment 35's 64 draws (seed 35). Everything else is the experiments' own: point contact, μ 1.3 → 2.0, gates, and E (`analyze_roll.tip_error`, `box_pivot.hand_error`). Hands: the paper's 18 redesigns, plus the e20 MR picks (`b_z`: B raised; `free`: A and B raised, axes not aligned) and the free picks at 5 and 9 mm. Permutation p: one-sided, 20,000 permutations, seed 0. "p < 5e-5" means no permutation reached ρ.

**Code.**
- `--anchor face|f1` in `two_pad_drake.py` (tipdial) and `box_pivot.py run`. f1 uses O's face depth (6 mm) for every hand's placement. The default is face, and the option is recorded in a log's opts only when it is f1.
- `finger_geometry` handles hands off the link line (A or B with z ≠ 0 in the design). For those, B comes from the CoDe hand model's closure and the distal pad sits at its own mount offset. The studies' hands keep the old path unchanged.
- `anchor_f1.py`: `deltas` computes Δ_contact, Δ_MCP and Δ_skel from each parameter file over q ∈ [0, 120°] in 0.25° steps; `analyze` computes E and ρ.

**Checks.**
- **Reproduction.** With anchor face, O and R reproduce the logged e29y, e29z and e35 runs bit-identically: every field and trace, 64 draws × 2 μ. This was checked before and after the geometry change. With anchor f1, O is bit-identical to face in all three tasks (Δt = 0). MR-mid5 (Δt = 0) also gives identical f1 and face E.
- **Δ values.** Δ_contact and Δ_skel reproduce `analyze.DESIGN` for all 18 hands. Δ_MCP reproduces the reference values: PR 4.65, U 4.26, PR5/U5 2.10/2.16, PR9/U9 6.36/6.11. All eight MR picks reproduce e20's Δ_MCP and Δ_contact.
- **MR geometry.** For the four 7 mm MR hands, Drake's tip and contact paths match the design's (`paths()`) within 6e-14 mm.
- **Pre-existing model mismatch, unchanged here.** Drake mounts both pads at `mount_offset_mm`. Three paper hands have a different distal mount: d1p5_maxDs_s0, d3_maxDs_s0 and h3to9_d3p5_maxDs_s0. Their simulated distal face is 1.6, 2.9 and 1.0 mm deeper than designed. As simulated, their Δ_contact is 2.31, 3.29 and 4.40 instead of 1.46, 3.04 and 3.52. The tables use the design values, as the paper does.

**E per hand (%).** The face columns repeat the paper's runs; the MR rows are new. In parentheses: f1 pairs dropped as over-pressed (> 1.5 mm) or lost. Δ in mm.

| hand | Δt | Δ_contact | Δ_MCP | Δ_skel | dial y face | dial y f1 | disk z face | disk z f1 | pivot face | pivot f1 |
|---|---|---|---|---|---|---|---|---|---|---|
| PR | 4.0 | 0.86 | 4.65 | 7.67 | 9.3 | 23.4 (1) | 19.3 | 26.5 (12) | 5.4 | 11.7 |
| d1p5_minDs_s0 | 4.0 | 1.60 | 4.12 | 5.37 | 11.3 | 16.3 (1) | 25.8 | 21.8 (10) | 10.0 | 7.1 |
| d2p25_minDs_s0 | 4.0 | 2.33 | 4.15 | 4.62 | 13.0 | 12.8 (1) | 34.2 | 18.3 (10) | 13.0 | 3.5 (1) |
| d2p25_maxDs_s0 | 4.0 | 2.30 | 5.86 | 9.17 | 10.6 | 28.2 (3) | 24.9 | 14.3 (13) | 4.8 | 12.5 (2) |
| d3_minDs_s0 | 4.0 | 3.09 | 4.01 | 3.83 | 17.5 | 10.1 (1) | 37.7 | 13.5 (6) | 15.4 | 2.2 (2) |
| U | 4.0 | 3.91 | 4.26 | 3.38 | 26.5 | 7.7 (4) | 54.7 | 17.5 (4) | 22.2 | 7.8 (1) |
| d4p5_s0 | 4.0 | 4.41 | 8.06 | 11.33 | 9.5 | 36.1 (4) | 27.3 | 12.4 (12) | 2.3 | 13.7 (4) |
| d5p5_s0 | 4.0 | 5.40 | 8.59 | 12.17 | 10.2 | 38.3 (4) | 27.0 | 14.5 (10) | 2.8 | 13.5 (5) |
| d1p5_maxDs_s0 | 5.6 | 1.46 | 6.77 | 9.67 | 7.4 | 31.4 (12) | 23.2 | 34.5 (13) | 8.9 | 11.0 (16) |
| d3_maxDs_s0 | 6.9 | 3.04 | 8.61 | 11.55 | 8.0 | 44.4 (17) | 31.7 | 30.5 (15) | 12.3 | 11.9 (33) |
| PR5 | 2.0 | 0.35 | 2.10 | 3.55 | 3.2 | 14.7 | 11.0 | 12.9 (7) | 2.3 | 6.6 (1) |
| U5 | 2.0 | 1.58 | 2.16 | 2.02 | 10.6 | 8.6 (1) | 25.0 | 6.8 (4) | 9.3 | 2.2 (1) |
| h3to5_d3_s0 | 2.0 | 2.92 | 4.74 | 6.38 | 9.7 | 23.7 (1) | 7.2 | 15.1 (9) | 3.0 | 11.4 (2) |
| h3to5_d3_minDs_s0 | 2.0 | 3.09 | 2.00 | 0.38 | 18.8 | 6.3 (2) | 30.3 | 13.4 (2) | 13.7 | 5.8 |
| PR9 | 6.0 | 2.32 | 6.36 | 9.37 | 14.8 | 30.7 (5) | 45.9 | 18.3 (13) | 15.2 | 7.7 (11) |
| U9 | 6.0 | 3.89 | 6.11 | 7.17 | 23.6 | 17.7 (5) | 61.8 | 9.3 (9) | 23.0 | 4.3 (4) |
| h3to9_d3p5_maxDs_s0 | 7.0 | 3.52 | 8.62 | 12.72 | 15.5 | 40.2 (19) | 53.4 | 14.9 (18) | 18.7 | 17.3 (21) |
| h3to9_d5_minDs_s0 | 6.0 | 5.08 | 6.05 | 5.32 | 29.8 | 17.1 (7) | 65.1 | 9.4 (10) | 25.2 | 5.1 (3) |
| MR (b_z) | 4.0 | 3.47 | 1.66 | 4.34 | 16.8 | 13.4 | 40.2 | 9.7 (3) | 16.5 | 3.2 (1) |
| MR-mid (b_z) | 4.0 | 2.08 | 2.10 | 5.22 | 13.1 | 13.2 (1) | 30.2 | 25.3 (7) | 12.3 | 6.5 |
| MR (free) | 3.0 | 3.26 | 0.54 | 4.29 | 18.5 | 6.8 (1) | 45.4 | 13.9 (1) | 18.4 | 2.8 |
| MR-mid (free) | 1.0 | 1.07 | 1.06 | 4.78 | 8.0 | 9.2 (1) | 24.1 | 14.4 | 7.4 | 3.0 |
| MR5 (free) | 0.9 | 0.99 | 0.11 | 2.09 | 5.9 | 4.7 (2) | 14.1 | 8.0 | 5.0 | 0.6 |
| MR-mid5 (free) | 0.0 | 0.28 | 0.28 | 2.02 | 4.0 | 4.0 | 10.4 | 10.4 | 1.6 | 1.6 |
| MR9 (free) | 3.0 | 3.99 | 1.47 | 6.98 | 19.9 | 10.8 | 49.3 | 23.3 | 21.0 | 6.5 |
| MR-mid9 (free) | 3.0 | 2.43 | 2.39 | 7.00 | 16.9 | 10.2 (3) | 49.3 | 19.1 (2) | 16.5 | 4.4 (2) |

**Spearman ρ(E, ·), one-sided permutation p.** First the paper's 18 hands, then with the eight MR hands (26).

| task | anchor | Δ_MCP | Δ_contact | Δ_skel |
|---|---|---|---|---|
| dial y | f1 | **0.92, p < 5e-5**; 0.89 | 0.15, p 0.28; 0.27 | 0.98, p < 5e-5; 0.91 |
| dial y | face | −0.16; 0.02 | **0.56, p 0.009**; 0.69 | −0.37; −0.09 |
| disk z | f1 | 0.25, p 0.16; 0.31 (p 0.061) | −0.35; −0.10 | 0.36, p 0.073; 0.49 (p 0.005) |
| disk z | face | 0.25; 0.24 | **0.68, p 0.0009**; 0.74 | −0.00; 0.17 |
| pivot | f1 | **0.71, p 0.0008**; 0.79 | 0.16, p 0.26; 0.28 | 0.79, p < 5e-5; 0.77 |
| pivot | face | 0.03; 0.06 | **0.41, p 0.050**; 0.57 | −0.21; −0.00 |

- The pivot's f1 run loses blocks on the maxDs and 9 mm hands: up to 32 of 63 on d3_maxDs_s0. Counting those failures as infinite error gives the same picture: Δ_MCP 0.76, Δ_contact 0.06, Δ_skel 0.82 (18 hands).
- Δ_MCP and Δ_skel are nearly collinear across the paper's hands (ρ 0.93). Partial ρ(E, Δ_MCP | Δ_skel) on 18 / 26 hands:
  - dial y: 0.11 / 0.61;
  - pivot: −0.13 / 0.43.

  Partial ρ(E, Δ_skel | Δ_MCP):
  - dial y: 0.84 / 0.70;
  - pivot: 0.51 / 0.37.

**Verdicts.**
1. **Partly holds.** At f1, E follows Δ_MCP in the dial y (0.92, p < 5e-5) and in the pivot (0.71, p 0.0008), and Δ_contact drops to about 0.15. The disk z fails (0.25, p 0.16). There the deeper hands over-press the disk: 4–19 of 59 pairs are dropped per hand, which is a gate effect. Also, Δ_MCP is not separable from Δ_skel on the paper's hands. Only the MR hands separate the two partly, and then each keeps a partial ρ of 0.4–0.7. "Follows Δ_MCP" is therefore supported in two of three tasks, but "rather than Δ_skel" is not shown.
2. **Holds.** At f1, PR errs more than U in all three tasks: dial y 23.4 vs 7.7%, disk z 26.5 vs 17.5%, pivot 11.7 vs 7.8%.
3. **Holds for MR (free, Δ_MCP 0.54); mixed for MR (b_z, 1.66).**
   - MR (free) errs least of PR, U and MR in all three tasks: dial y 6.8%, disk z 13.9%, pivot 2.8%.
   - MR (b_z) errs least in the disk z and the pivot (9.7, 3.2%), but not in the dial y (13.4% vs U 7.7%).
   - Dose view: MR5 errs least of PR5, U5 and MR5 in the dial y and the pivot (4.7, 0.6%). In the disk z, U5 is lower (6.8 vs 8.0%). MR9 errs least of the 9 mm hands in the dial y (10.8%), not in the pivot (6.5 vs U9 4.3%) or the disk z (23.3 vs U9 9.3%).
4. **Holds.** At face, the paper's result stands, bit-identical to the paper's runs: Δ_contact ρ 0.56, 0.68, 0.41 for the 18 hands, and 0.69, 0.74, 0.57 with MR. PR errs less than every 7 mm MR in every task. For example, in the pivot PR has 5.4% against 12.3–18.4% for MR.

**Reading.** The placement decides which path deviation the error follows. Fixed to the proximal face, it follows Δ_contact; fixed to F₁ with the MCP held, it follows the F₁ path deviation (Δ_MCP ≈ Δ_skel on these hands). The face-anchored result is therefore a property of the placement, not of the hands alone. Hands with a large Δt cannot serve both frames, since Δ_contact + Δ_MCP ≥ |Δt|. The free picks get around this by shrinking Δt instead: MR-mid (free) has Δt 1 mm and errs ≤ 9% in the dial y and the pivot under both anchors, and MR-mid5 has Δt 0, so its two anchors are identical.

**Commands** (`sim/drake`; logs in `logs/`, `anchor_f1_*.json`):
```
uv run python anchor_f1.py deltas
bash run_e29.sh y e29y_f1 6 --anchor f1 -- O R U ...        # and z e29z_f1; face runs use the paper's tags e29y / e29z
uv run python box_pivot.py run --hands O R U ... --draws logs/pivot_e35.json --tag e35_f1 --anchor f1
uv run python anchor_f1.py analyze --task tipdial_y --tag e29y_f1 --compare e29y --out logs/anchor_f1_e29y_f1.json
uv run python anchor_f1.py analyze --task tipdial_z --tag e29z_f1 --compare e29z --out logs/anchor_f1_e29z_f1.json
uv run python anchor_f1.py analyze --task pivot --tag e35_f1 --compare e35 --out logs/anchor_f1_e35_f1.json
```
Wall time: the 18 hands × 3 tasks took 61 min (3, then 6 workers). The 8 MR hands × 3 tasks × 2 anchors took about 40 min.

## Pre-registration: Δ_MCP or Δ_skel under F₁ anchoring? (2026-11-27, before the run)

- **Why.** Under f1 the Drake error followed Δ_MCP, but also Δ_skel: ρ(Δ_MCP, Δ_skel) is 0.93 on the paper's hands. The paper says "contact, not skeleton". If Δ_skel wins under f1, that claim holds only for face-anchored objects.
- **Hands.** `design/scripts/study/exploratory/e21_mcp_vs_skeleton.py`, 7 mm, A and B heights free, C̄_pad ≥ 0.70. Two families:
  - lowM: Δ_MCP ≤ 1.5 with Δ_skel spread over 4–6, 6–8 and 8+ mm;
  - highM: Δ_skel ≤ 3 with Δ_MCP spread over 3–4.5, 4.5–6 and 6+ mm.
- **Runs.** Drake dial y and pivot at anchor f1, the same protocol as the f1 runs above. The new hands are pooled with the 26 hands already run.
- **Prediction.** E follows the contact point.
  - ρ(E, Δ_MCP | Δ_skel) > 0 with p < 0.05 in both tasks.
  - ρ(E, Δ_skel | Δ_MCP) is not significant.
  - lowM hands err less than highM hands, pairing by rank.
- **If it fails** (Δ_skel carries it), the f1 error is a skeleton effect. The paper's "not the skeleton" then needs the face anchoring stated.

## Result: Δ_MCP or Δ_skel under F₁ (2026-11-27)

- **Hands** (e21, 7 mm; the Drake recomputation matches the design):

| hand | Δ_contact | Δ_MCP | Δ_skel |
|---|---|---|---|
| lowM_s4 | 1.39 | 1.48 | 5.48 |
| highM_m3 | 4.52 | 4.45 | 1.97 |
| highM_m4p5 | 4.93 | 5.93 | 1.93 |

  The lowM search found no design with Δ_MCP ≤ 1.5 and Δ_skel ≥ 6 at C̄_pad ≥ 0.70, so there is one lowM hand, not three.
- **Runs.** dial y and pivot, anchor f1 (and face), the same protocol as above. They are pooled with the 26 hands already run, 29 in all. Across these 29 hands the rank correlation of Δ_MCP with Δ_skel drops to 0.65.

| task (f1) | E lowM_s4 | E highM_m3 | E highM_m4p5 | partial ρ(E, Δ_MCP \| Δ_skel) | partial ρ(E, Δ_skel \| Δ_MCP) |
|---|---|---|---|---|---|
| dial y | 9.6% | 15.0% | 8.8% | 0.71 (p < 1e-4) | 0.72 (p 1e-4) |
| pivot | 3.4% | 19.7% | 12.7% | 0.68 (p < 1e-4) | −0.05 (p 0.61) |

(one-sided permutation, 20,000)
- **Prediction, by task.**
  - **Pivot: holds.** Under f1 the error follows the contact point (Δ_MCP), not the skeleton. The lowM hand errs least.
  - **Dial y: fails in part.** Δ_MCP carries the error given Δ_skel, but Δ_skel carries it too given Δ_MCP. The dial is tangent to the pad near its tip end, so the fingertip's own path matters as well. The lowM hand is not below highM_m4p5.
- **Face anchoring stays as the paper has it** (29 hands): E follows Δ_contact in both tasks (ρ 0.72 for dial y and 0.61 for the pivot, p ≤ 3e-4). E does not follow Δ_MCP (0.10, 0.11) or Δ_skel (−0.17, −0.13).
- **Reading.**
  - Which contact quantity decides the error depends on what the object is fixed to. Fixed to the proximal face (face anchoring): Δ_contact. Fixed to the palm (f1): Δ_MCP, the same contact point seen from the MCP axis.
  - For a fingertip-edge contact (dial y) under f1, the skeleton's tip path adds to it.
  - Only a hand whose proximal face stays put keeps both: MRmid_free (Δt 1 mm) and the 5 mm MRs (Δt ≈ 0) have E ≤ 10% under both anchorings in both tasks.

## Pre-registration: RL relay dial with the dial fixed to the palm (2026-11-27, before any run)

- **Why.** Every RL task so far places the object in each hand's proximal-face frame (PR-anchored):
  - 7b′, the cube, and the 7a′ relay dial (`finger_relay/env_cfg.py`: "the same face-frame coordinates on every hand").
  - A dial bolted to the world sits at one pose relative to the palm. That is anchor f1.
- **Change.** `CODE_RELAY_ANCHOR=f1` places the dial with O's face frame on every hand. The default (`face`) is unchanged.
- **Runs.**
  - Policies: the 7a′ closed-loop policies (dial angle observed), training seeds 20 and 21, `*_relay_seg2/model_398.pt`, zero-shot.
  - Hands: O, PR (R), U, MR (mr_free), MRM (mrmid_free).
  - Anchor face and f1; pad μ 1.3 / 2.0; eval seeds 94000 / 95000; 1024 envs; `eval_relay.py`.
- **Measures.**
  - The transfer ratio: rotation at μ 1.3 over O's. This is the primary measure.
  - E is secondary. O's friction response here is small (+17 deg), so E is unstable, as noted in attempt 8.
- **Predictions.**
  1. Under f1 the ratio orders by Δ_MCP: MR ≥ MRM > U ≈ PR. MR and MRM both beat PR in both training seeds.
  2. Under face the ratio orders by Δ_contact: PR ≈ MRM > MR ≈ U.
  3. MRM is within 0.1 of the best hand under both anchors.
- **Kill rule.** If O's face-anchored rotation is off its logged 280 deg by more than 5%, stop.

## Result: RL zero-shot transfer to MR, 7b′ and cube (2026-11-27)

Driver: `sim/isaaclab/scripts/rotate_cube/run_mr_transfer.sh`; summary: `mr_transfer_summary.py`; outputs in `sim/isaaclab/logs/pr_mr_transfer/`.
- O reproduced its logged values within ±4.5%, so the kill rule passed.
- MR's and MRM's cube grasp generation first hit a CUDA out-of-memory while attempt 9's grid shared the GPU. Both were rerun alone (rc 0).

| hand | Δt (face) | Δ_contact | Δ_MCP | 7b′ ratio (seed 20 / 21) | 7b′ E | cube rotation ratio | cube full≤30° ratio | cube E (rotation) |
|---|---|---|---|---|---|---|---|---|
| PR (R) | 4 | 0.86 | 4.65 | 0.72 / 0.61 | 0.20 | 0.749 | 0.703 | 0.11 |
| U | 4 | 3.91 | 4.26 | 0.08 / 0.07 | 1.34 | 0.703 | 0.800 | 0.15 |
| MR (free) | 3 | 3.26 | 0.54 | 0.06 / 0.07 | 1.36 | 0.766 | 0.730 | 0.19 |
| **MRM (free)** | **1** | 1.07 | 1.06 | 0.71 / 0.59 | 0.41 | **0.938** | **0.976** | 0.44 |
| MRbz | 4 | 3.47 | 1.66 | 0.03 / 0.06 | 1.40 | 0.754 | 0.738 | 0.20 |
| MRMbz | 4 | 2.08 | 2.10 | 0.18 / 0.12 | 1.31 | 0.743 | 0.728 | 0.03 |

- Eval-seed noise of the cube rotation ratio is 0.02–0.08, and 0.005–0.06 for full≤30°.
- 7b′ ratios are pooled over the two training seeds in the E column.
- The cube E noise is 0.03–0.26, and O's cube response is small (Δ(O) −115 deg; full≤30° −0.065). So E does not separate the hands.
- **Verdict on the registered predictions.**
  1. **Cube: MR and MRM better than PR and U on the ratio and on E. Fails as stated.**
     - MRM beats every hand on the ratio, by 0.19 to 0.24, against noise of about 0.05.
     - MR does not beat PR.
     - E separates nothing.
     - The premise (the cube is palm-anchored) was wrong before any result; see the correction above.
  2. **7b′: MRM ≈ PR > MR, and MR no better than U. Holds.**
  3. **MRM within 0.1 of O's ratio on both tasks. Fails.**
     - Cube: holds, at 0.94 and 0.98.
     - 7b′: fails, at 0.65. PR, the best 7b′ hand, is 0.67, so 0.1 of O was too strict a mark.
- **What stands out (post hoc).**
  - **MRM is the first redesign that inherits the cube policy almost whole.** On rotation it gets 0.94 and full turns 0.98. All the other 7 mm redesigns sit at 0.70–0.77, PR included. The cube task uses the MCP, the abduction and the thumb.
  - The hands that fail all move the proximal face 3–4 mm. MRM moves it 1 mm, and also keeps both Δ_contact and Δ_MCP near 1 mm.
  - In 7b′, MRM equals PR. So keeping the face in place costs nothing there, and it buys the cube.
  - Next: a dose check on Δt with the 5 mm and 9 mm MR and MRM hands on the cube.

## Pre-registration: cube transfer versus the proximal face shift (2026-11-27, before the run)

- **Hands added.** R5, U5, R9, U9 (re-run with this code); MR5, MRM5, MR9, MRM9 (e20 free picks). The protocol is the cube one above.

| hand | Δt (proximal face) | Δ_contact | Δ_MCP |
|---|---|---|---|
| MRM5 | 0 | 0.28 | 0.28 |
| MR5 | 0.9 | 0.99 | 0.11 |
| R5 / U5 | 2 | 0.35 / 1.58 | 2.10 / 2.16 |
| MR9 / MRM9 | 3 | 3.99 / 2.43 | 1.47 / 2.39 |
| R9 / U9 | 6 | 2.32 / 3.89 | 6.36 / 6.11 |

- **Predictions (post hoc in origin).** They come from the MRM result, so this is a confirmation run.
  1. Across the 13 hands (7 mm: R, U, MR, MRM, MRbz, MRMbz; plus these 8), the cube rotation ratio falls with Δt. Spearman ρ < 0, one-sided p < 0.05.
  2. MRM5 ≥ 0.90.
  3. Δt predicts the ratio better than Δ_contact or Δ_MCP: |ρ| is larger.

## Result: Drake pull-out (paper experiment 21, point contact) with the MR hands (2026-11-27, post hoc)

The cylinder rests on the proximal face (face-anchored by nature). Runs use `run_batch.sh point e21 <hands>`, and E comes from `analyze.error`, as in the paper.

| hand | Δt | Δ_contact | Δ_MCP | E |
|---|---|---|---|---|
| PR / U (7) | 4 | 0.86 / 3.91 | 4.65 / 4.26 | 19.4% / 71.7% |
| **MRM (7)** | **1** | 1.07 | 1.06 | **8.5%** |
| MR (7) | 3 | 3.26 | 0.54 | 17.3% |
| MRMbz / MRbz (7) | 4 | 2.08 / 3.47 | 2.10 / 1.66 | 13.8% / 33.9% |
| PR5 / U5 | 2 | 0.35 / 1.58 | 2.10 / 2.16 | 4.2% / 36.9% |
| MRM5 / MR5 | 0 / 0.9 | 0.28 / 0.99 | 0.28 / 0.11 | 5.3% / 5.4% |
| PR9 / U9 | 6 | 2.32 / 3.89 | 6.36 / 6.11 | 14.8% / 55.2% |
| **MRM9** / MR9 | 3 | 2.43 / 3.99 | 2.39 / 1.47 | **8.0%** / 16.2% |

- At 7 and 9 mm, MRM errs less than PR in the paper's own pull-out, even though its Δ_contact is a little larger. At 5 mm, MRM5 and PR5 are within 1 point of each other.
- Across Drake's pull-out, dial y and pivot (both anchorings) and Isaac's 7b′ and cube, MRM is the only 7 mm redesign that is never worse than the best other hand by more than a few points. In each task it is best or near best.

## Result: RL relay dial, face against f1 (2026-11-27)

36 evals, all rc 0. O's face-anchored rotation is 280 / 274 deg (training seeds 20 / 21), on the logged 280. The ratio is rotation at μ 1.3 over O's; eval noise is ≤ 0.008.

| hand | Δ_contact | Δ_MCP | Δ_skel | ratio, face (s20 / s21) | ratio, f1 (s20 / s21) |
|---|---|---|---|---|---|
| PR (R) | 0.86 | 4.65 | 7.67 | 0.57 / 0.56 | 0.68 / 0.69 |
| U | 3.91 | 4.26 | **3.38** | **0.72 / 0.68** | **0.84 / 0.89** |
| MR | 3.26 | 0.54 | 4.29 | 0.51 / 0.52 | 0.63 / 0.63 |
| MRM | 1.07 | 1.06 | 4.78 | 0.57 / 0.56 | 0.69 / 0.63 |

- **All three predictions fail.**
  - Under f1 the order is not by Δ_MCP: MR is last and U is first.
  - Under face the order is not by Δ_contact: U is first, and PR equals MRM.
  - MRM is 0.15–0.25 below U under both anchors.
- E is not read here. O's friction response is +17 deg, and E comes out at 1–3 for every hand.
- **Reading.**
  - The relay dial is driven by strokes of the fingertip end. The disk's hand-facing face sits at O's index fingertip.
  - The hand with the smallest Δ_skel (U) transfers best under both anchors. That matches the attempt-7a′ note that the fingertip position sets the result there, and Drake dial y under f1, where Δ_skel carried the error too.
  - So the contact-surface account covers pad contact. A task that touches with the fingertip end follows the skeleton's tip path, whatever the anchoring. That is a scope limit to state.
- All hands transfer better under f1 than under face (by about 0.1–0.17). Placing the dial in each hand's own face frame moves it about 4 mm along the face normal; at O's palm pose it sits closer to where the hand's fingertips actually are.

## A second bound, and the MRS hand (2026-11-27)

- **Bound.** The distal contact point is the fingertip C plus the distal face depth, turned with the distal phalanx: c = C + R(q)(0, −d). In F₁, therefore, Δc(q) − ΔC(q) = R(q)(0, −Δd) at every q, where Δd is the distal face's depth change. So Δ_MCP + Δ_skel ≥ |Δd|.
  - The distal face's depth is set from B on the distal link line, and the distal mount stays at its least supported value, 3 mm. Raising B lifts the frame with it. So Δd = h − 3 for these designs: 4 mm at 7 mm.
  - With the first bound (Δ_contact + Δ_MCP ≥ |Δt|), a redesign cannot keep the palm-frame contact point and the palm-frame fingertip together when the pad thickens. This is the "contact versus skeleton" trade, seen from the palm.
- **MRS** (`design/scripts/study/exploratory/e22_all_three.py`, 7 mm). The least max(Δ_contact, Δ_MCP, Δ_skel) at C̄_pad ≥ 0.70:
  - Δ_contact 2.29, Δ_MCP 2.02, Δ_skel 2.31, C̄_pad 0.701, MA 0.54, face 7.
  - Δ_MCP + Δ_skel = 4.33 ≥ 4.
- **Pre-registration (before any MRS run).** The runs are the relay dial (face and f1), 7b′, the Drake pull-out, dial y and pivot (f1 and face).
  1. Relay: MRS ≥ U under f1. Its Δ_skel is smaller than U's.
  2. 7b′: MRS ratio < 0.5. Its Δ_contact is above attempt 9's Dc50 of 1.7.
  3. Drake pull-out: MRS errs more than MRM.
  - If all three hold, no single 7 mm redesign serves pad and fingertip-end contact together. That is the bound in practice.

## Result: cube transfer versus the proximal face shift (2026-11-27)

64 new evals, all rc 0 (`SUMMARY_HANDS=... mr_transfer_summary.py`). Ratio = the hand's score over O's (rotation, or full turns upright ≤ 30°).

| hand | Δt | Δ_contact | Δ_MCP | Δ_skel | rotation ratio | full≤30° ratio |
|---|---|---|---|---|---|---|
| MRM5 | 0 | 0.28 | 0.28 | 2.02 | **0.989** | **0.995** |
| MR5 | 0.9 | 0.99 | 0.11 | 2.09 | 0.963 | 0.957 |
| MRM | 1 | 1.07 | 1.06 | 4.78 | 0.938 | 0.976 |
| R5 / U5 | 2 | 0.35 / 1.58 | 2.10 / 2.16 | 3.55 / 2.02 | 0.837 / 0.840 | 0.774 / 0.813 |
| MR / MR9 / MRM9 | 3 | 3.26 / 3.99 / 2.43 | 0.54 / 1.47 / 2.39 | 4.29 / 6.98 / 7.00 | 0.766 / 0.809 / 0.815 | 0.730 / 0.814 / 0.812 |
| R / U / MRbz / MRMbz | 4 | 0.86–3.91 | 1.66–4.65 | 3.38–7.67 | 0.703–0.754 | 0.703–0.800 |
| R9 / U9 | 6 | 2.32 / 3.89 | 6.36 / 6.11 | 9.37 / 7.17 | 0.655 / 0.621 | 0.740 / 0.710 |

Spearman ρ over the 14 hands, one-sided permutation (20,000):

| measure | Δt | Δ_contact | Δ_MCP | Δ_skel |
|---|---|---|---|---|
| rotation ratio | **−0.98** (p < 5e-5) | −0.60 (0.013) | −0.76 (0.0009) | −0.66 (0.007) |
| full≤30° ratio | **−0.80** (0.0003) | −0.25 (0.20) | −0.59 (0.017) | −0.57 (0.018) |

- **All three predictions hold** (registered after the MRM result, so this is a confirmation):
  1. The ratio falls with Δt, at ρ −0.98.
  2. MRM5 reaches 0.99.
  3. Δt predicts better than Δ_contact, Δ_MCP or Δ_skel.
  - The registration said 13 hands; the table has 14 (MR9 and MRM9 both sit at Δt 3).
- **Reading.**
  - The cube task uses the thumb, the abduction and the MCP. The cube starts on each hand's own proximal face, but the thumb and palm are the same on every hand. So how far the proximal face moves relative to the palm (Δt) sets how far the whole grasp is displaced against the parts that did not change.
  - At equal Δt, the contact-path quantities hardly matter: the 4 mm group spans Δ_contact 0.86–3.91 at ratios 0.70–0.75.
  - Keeping the face in place restores the cube policy: Δt 0–1 gives 0.94–0.99.

## Result: MRS in Drake, and the hydroelastic pull-out (2026-11-27)

**MRS** (Δ_contact 2.29, Δ_MCP 2.02, Δ_skel 2.31), point contact:

| task | anchor | PR | U | MRM | MRS |
|---|---|---|---|---|---|
| pull-out | face | 19.4% | 71.7% | 8.5% | **53.3%** |
| dial y | f1 | 23.4% | 7.7% | 9.2% | **6.4%** |
| dial y | face | 9.3% | 26.5% | 8.0% | 9.5% |
| pivot | f1 | 11.7% | 7.8% | 3.0% | 7.1% |
| pivot | face | 5.4% | 22.2% | 7.4% | 11.0% |

- Prediction 3 holds: MRS errs far more than MRM in the pull-out.
- MRS is best only in dial y at f1, the fingertip-edge, palm-fixed case.
- So trading the pad references for the tip path buys the fingertip case and costs the grasp.

**Hydroelastic pull-out** (paper experiment 21's hydro, 0.1 MPa over 3 mm):

| hand | E |
|---|---|
| PR / U (7 mm) | 16.7% / 253.8% |
| MRM / MR (7 mm) | **12.8%** / 41.6% |
| PR5 / MRM5 | 4.3% / 4.9% |
| PR9 / MRM9 | 100.0% / **74.0%** |

- The point-contact order holds with hydroelastic pads: MRM ≤ PR at 7 and 9 mm, and a tie at 5 mm.

## Result: MRS in RL (2026-11-27)

All evals rc 0. Ratios are against O.

| task | PR (R) | U | MRM | MRS |
|---|---|---|---|---|
| relay, face (s20 / s21) | 0.57 / 0.56 | 0.72 / 0.68 | 0.57 / 0.56 | 0.72 / 0.72 |
| relay, f1 | 0.68 / 0.69 | **0.84 / 0.89** | 0.69 / 0.63 | 0.75 / 0.78 |
| 7b′ (pooled) | 0.67 | 0.07 | 0.65 | **0.66** |
| cube rotation | 0.75 | 0.70 | 0.94 | 0.93 |
| cube full≤30° | 0.70 | 0.80 | 0.98 | 1.01 |

- **Prediction 1 (relay f1: MRS ≥ U) fails.** MRS is second, at 0.75–0.78 against U's 0.84–0.89. Under face anchoring it ties U.
- **Prediction 2 (7b′: MRS < 0.5) fails.** MRS transfers at 0.66, as well as PR and MRM, although its Δ_contact (2.29) is above attempt 9's Dc50 (1.7).
- **Prediction 3 (Drake pull-out: MRS > MRM) holds** (53% against 8.5%).
- **Reading.**
  - Attempt 9's Dc50 was measured on hands whose proximal face all moved by h − 3: 4 mm at 7 mm. Δt was constant there, so it could not show.
  - Here MRS (Δ_contact 2.29, Δt 1) walks at 0.66, while MRMbz (Δ_contact 2.08, Δt 4) collapses to 0.15.
  - In 7b′, as in the cube, a small proximal face shift protects the transfer. Δ_contact then matters less. All 7b′ hands with Δt 1 transfer (MRM 0.65, MRS 0.66). Among the Δt 4 hands, only the smallest Δ_contact (PR, 0.86) transfers.

## Pre-registration: MS, the skeleton-referenced counterpart of MRM (2026-11-27, before any MS run)

- **Why.** MRS mixed the pad references with the tip path. A fair test sets two opposite redesigns from the same search side by side: MRM, which keeps the pad paths (Δ_contact and Δ_MCP), against a hand that keeps the skeleton tip path (Δ_skel). U is not that hand. It comes from Study 2's space with A and B on the link line, so its Δt is 4.
- **MS** (`e22_all_three.py --repick`, e22 front, A and B heights free, C̄_pad ≥ 0.70): the least Δ_skel.
  - Δ_contact 4.00, Δ_MCP 3.82, Δ_skel **0.60**, C̄_pad 0.703, MA 0.54, face 7 (Δt 1), mounts 0/3.
  - Against MRM (1.07, 1.06, 4.78, 0.735, Δt 1), the space, Δt and the floor are the same.
  - Δ_skel costs coverage on this front: the least Δ_skel is 0.99 at C̄_pad ≥ 0.71, 2.11 at ≥ 0.73, and 2.70 at ≥ 0.735. MS sits at the floor, as the MR picks do.
- **Runs.** These are MRS's runs: Drake pull-out (point, face), dial y and pivot (f1 and face); RL relay (face and f1), 7b′ and cube.
- **Predictions.**
  1. Drake pull-out: MS errs more than MRS (53%), because its Δ_contact is larger.
  2. Drake pivot f1: MS errs more than MRM (3.0%), because pivot follows Δ_MCP. Dial y f1: MS errs less than MRM (9.2%), because dial y also follows Δ_skel.
  3. RL relay: MS ≥ U under both anchors, and MS > MRM.
  4. RL 7b′: MS ratio < 0.5. Its Δ_contact (4.0) is far above anything that transferred.
  5. RL cube: MS ratio ≥ 0.85 if Δt alone sets the cube transfer. If the pad path matters at this size, it is lower.
  - If 1–4 hold, each redesign wins only the contact it keeps. Pad tasks then follow the pad path, and fingertip-end tasks follow the tip path. The choice of reference is then a choice of task.

## Names (2026-11-27, from here on)

The user renamed the hands by reference and kept point:

| new | old | keeps |
|---|---|---|
| Original | O | the 3 mm hand |
| Coverage | U | coverage only |
| Pad-Pad | PR (R) | distal pad contact path from the proximal pad face (Δ_contact) |
| Palm-Pad | MR | the same path from the palm frame F₁ (Δ_MCP) |
| MO-Pad | MRM | both pad paths (multi-objective) |
| Palm-Tip | MS | the fingertip skeleton path in F₁ (Δ_skel) |

MRS (all three) is dropped from the story. Bound 2 makes it a compromise that leads no task except dial y at f1. Code keys and file names are unchanged.

## Result: Palm-Tip (MS) against MO-Pad (2026-11-27)

All runs rc 0. Drake is E (lower is better). RL is the ratio to Original (higher is better).

| task | anchor | Pad-Pad | Coverage | MO-Pad | Palm-Tip | prediction |
|---|---|---|---|---|---|---|
| Drake pull-out (point) | face | 19.4% | 71.7% | **8.5%** | 90.4% | 1. Palm-Tip > MRS (53%): **holds** |
| Drake pivot | f1 | 11.7% | 7.8% | **3.0%** | 12.1% | 2a. Palm-Tip > MO-Pad: **holds** |
| Drake dial y | f1 | 23.4% | 7.7% | 9.2% | 10.1% | 2b. Palm-Tip < MO-Pad: **fails** |
| Drake pivot / dial y | face | 5.4 / 9.3% | 22.2 / 26.5% | 7.4 / 8.0% | 15.3 / 13.9% | – |
| RL relay (s20 / s21) | face | 0.57 / 0.56 | 0.72 / 0.68 | 0.57 / 0.56 | **0.90 / 0.95** | 3. **holds** |
| RL relay | f1 | 0.68 / 0.69 | 0.84 / 0.89 | 0.69 / 0.63 | **0.95 / 1.02** | 3. **holds** |
| RL 7b′ (pooled) | face | 0.67 | 0.07 | 0.65 | 0.18 | 4. < 0.5: **holds** |
| RL cube rotation | face | 0.75 | 0.70 | **0.94** | 0.82 | 5. ≥ 0.85: **fails** |
| RL cube full turn ≤ 30° | face | 0.70 | 0.80 | 0.98 | **0.99** | 5. ≥ 0.85: holds |

- **Each redesign wins the contact it keeps.**
  - MO-Pad leads every pad task: the grasp, the palm-fixed pivot, 7b′ and cube rotation.
  - Palm-Tip leads the fingertip-end relay under both anchors, above Original's own score at f1 (seed 21).
  - Palm-Tip collapses in the grasp (90%) and in 7b′ (0.18).
  - The choice of reference is a choice of task. Bound 2 says it cannot be avoided at 7 mm.
- **Dial y (f1) is not a fingertip-end task.** Palm-Tip's Δ_skel of 0.60 does not beat MO-Pad there. Coverage (7.7%) and MRS (6.4%), which balance both paths, lead it. Dial y needs both paths.
- **Cube: Δt is not all.**
  - Palm-Tip has Δt 1, like MO-Pad, but rotates at 0.82, below MO-Pad's 0.94. It is still above the Δt 4 hands (Pad-Pad 0.75, Coverage 0.70).
  - Full turns are kept at 0.99.
  - So Δt sets most of the cube transfer, and the pad path adds to it once Δ_contact is as large as 4 mm. The ρ −0.98 over 14 hands came from hands whose Δ_contact stayed ≤ 3.3.

## Pre-registration: cube transfer with a second training seed (2026-11-27, before training)

- **Why.** The cube result (ρ(ratio, Δt) −0.98 over 14 hands; MO-Pad 0.94 against Pad-Pad 0.75) rests on one policy, seed 20's v2c seg2 `model_1497`.
- **Training.** Seed 21 repeats the release lineage exactly (`docs/experimental/omnireset_recipe.md` §6.2):
  1. v2 seg1: 500 iterations from scratch.
  2. v2c seg1 from its `model_499`: goal bonus 5, entropy 0.001, std cap 1.0, 500 iterations.
  3. v2c seg2: 500 iterations. The last checkpoint is the release.
  - The only difference is that the NaN world recovery (§6.4) is now in the code for every segment. It touches diverged worlds only.
- **Eval.** The same held-out grasps (seed 76000) and overrides, pad μ 1.3 / 2.0, eval seeds 94000 / 95000. Hands: Original and the 14 hands of the dose run, plus MRS and Palm-Tip. Outputs go to `sim/isaaclab/logs/pr_mr_transfer_cube_s21/`; the driver is `run_mr_transfer.sh` with `TRANSFER_DIR` and `CUBE_CKPT`.
- **Validity gate.** If Original's held-out rotation under seed 21 is below half of seed 20's (919° at μ 1.3), the policy did not learn the task. Then the run counts as a failed training, not a transfer result.
- **Predictions** (rotation ratio, the primary metric):
  1. ρ(ratio, Δt) ≤ −0.8 over the same 14 hands.
  2. MO-Pad exceeds Pad-Pad by ≥ 0.10.
  3. Palm-Tip is below MO-Pad and above Pad-Pad, as under seed 20.

## Pre-registration: Study 2 with A and B heights free (2026-11-27, before the run)

- **Why.** MO-Pad came from exploratory scripts (e20). The paper's Study 2 searches with A and B on the link line. Does the paper's own pipeline, with the heights opened, find the small-Δt designs by itself, or does it need the second reference as an objective?
- **Run.** `redesign.py --free-heights`, the paper's objectives (least Δ_contact, most C̄_pad), CFL, q_req 90/120/150, 3 → 5/7/9 mm, NSGA-II 200 × 200, seeds 0 and 1. The A and B height bounds are e20's (A −3 to 8, B −3 to 10 mm). Rows add Δ_MCP. Output: `design/output/study/study2_redesign_free_heights/`.
- **Predictions** (q 120, 7 mm, against the paper's front in `study2_redesign_2026-11-21`):
  1. Freeing the heights lowers the least Δ_contact at C̄_pad ≥ 0.70 by less than 0.3 mm. Δ_contact alone does not reward a small Δt.
  2. The front's designs keep Δt near h − 3. Their least Δ_MCP at C̄_pad ≥ 0.70 stays above 2.5 mm, far from MO-Pad's 1.06.
  - If both hold, MO-Pad needs Δ_MCP as an objective (or Δt as a constraint): the paper's formulation with one reference does not find it.

## Result: Study 2 with A and B heights free, q 120 (2026-11-27)

`redesign.py --free-heights`, the paper's objectives (least Δ_contact, most C̄_pad); `design/output/study/study2_redesign_free_heights/`. At C̄_pad ≥ 0.70:

| to | least Δ_contact (Δ_MCP, C̄, face) | coverage end (Δ_contact, Δ_MCP, C̄, face) | paper front (coverage end) |
|---|---|---|---|
| 5 mm | 0.25 (0.55, 0.762, 6.3) | 0.36, 1.19, 0.771, 5.0 | 0.35, 2.10, 0.753, 8 |
| 7 mm | 0.75 (4.33, 0.725, 9.6) | **0.85, 1.62, 0.743, 7.0** | 0.86, 4.65, 0.725, 10 |
| 9 mm | 1.96 (4.88, 0.700, 9.0) | 2.13, 4.79, 0.712, 9.0 | none ≥ 0.70 |

- **Prediction 1 holds.** The least Δ_contact drops by 0.09 mm at 7 mm (0.84 → 0.75), less than 0.3.
- **Prediction 2 fails.** With the heights free, the coverage end itself mounts the proximal pad at 0. Its face sits at 7 mm (Δd_p 1), and its Δ_MCP is 1.62 mm, not above 2.5. A deeper mount gives the pads more room, so the paper's own objectives find the face kept at 5 and 7 mm. At 9 mm the face reaches its floor (Δd_p 3), but Δ_MCP stays at 4.79. There, only an objective on Δ_MCP keeps it (MO-Pad 9: 2.39).
- **Reading.** For the paper, the pins off the link line are the change, not a second objective. The 7 mm coverage end ("Pad-Pad, pins free") keeps Δ_contact as the paper's redesign does (0.85), keeps the face near its place, and has the most coverage of all the 7 mm hands (0.743).
- **Next.** Hand files for the 5/7/9 mm coverage ends, then the Drake pull-out, dial and pivot and the RL cube and 7b′ transfer against MO-Pad and Pad-Pad.

## Pre-registration: Pad-Pad with the pins free (S2F) in Drake and RL (2026-11-27, before any S2F run)

- **Hands** (Study 2's coverage end with A and B heights free; `e24_study2_free_picks.py`, `e20_make_hands.py`; code key `s2free_s2f`, Isaac `S2F`, `S2F5`, `S2F9`):

| hand | Δd_p | Δ_contact | Δ_MCP | Δ_skel | C̄_pad | A height | mount |
|---|---|---|---|---|---|---|---|
| S2F5 | −1 | 0.36 | 1.19 | 2.85 | 0.771 | 3.1 | 0 |
| S2F (7 mm) | 1 | 0.85 | 1.62 | 5.24 | 0.743 | 3.0 | 0 |
| S2F9 | 3 | 2.13 | 4.79 | 7.30 | 0.712 | 3.0 | 0 |

- **Runs.**
  - Drake, point contact: pull-out (face), dial y and pivot (face and f1), as for MO-Pad.
  - RL: cube under the seed-20 policy, and under the seed-21 policy once it exists; 7b′ and relay (face, f1) for S2F.
  - The RL runs queue behind the seed-21 training, since the GPU is full.
- **Predictions.**
  1. Drake f1 (dial y, pivot): S2F errs less than Pad-Pad (23.4%, 11.7%). Its Δ_MCP is 1.62 against 4.65.
  2. Drake pull-out (point): S2F ≤ Pad-Pad (19.4%).
  3. RL cube rotation ratio: S2F ≥ 0.90 and S2F5 ≥ 0.90 (Δd_p ≤ 1); S2F9 between 0.75 and 0.88 (Δd_p 3, like MRM9's 0.82).
  4. RL 7b′: S2F ≥ 0.55 (Δ_contact 0.85, like Pad-Pad's 0.86).
  - If 1–4 hold, freeing the pins in the paper's own Study 2 is enough for the paper's redesign at 5 and 7 mm. MO-Pad's extra objective is then needed only at 9 mm.

### Study 2 with the pins free: all flexions (2026-11-27)

Coverage ends of each front (free heights) against the paper's Study 2 (`study2_redesign_2026-11-21`):

| q_req | to | free: Δ_contact / Δ_MCP / C̄ / face | paper: Δ_contact / C̄ / face |
|---|---|---|---|
| 90 | 5 | 2.67 / 1.83 / 0.794 / 5 | 1.22 / 0.776 / 8 |
| 90 | 7 | 1.60 / 2.42 / 0.766 / 7 | 1.84 / 0.749 / 10 |
| 90 | 9 | 4.36 / 7.36 / 0.734 / 9 | 4.31 / 0.717 / 12 |
| 120 | 5 | 0.36 / 1.19 / 0.771 / 5 | 0.35 / 0.753 / 8 |
| 120 | 7 | 0.85 / 1.62 / 0.743 / 7 | 0.86 / 0.725 / 10 |
| 120 | 9 | 2.13 / 4.79 / 0.712 / 9 | 2.32 / 0.695 / 12 |
| 150 | 5 | 1.32 / 1.29 / 0.758 / 5 | 0.77 / 0.738 / 8 |
| 150 | 7 | 2.00 / 1.83 / 0.725 / 7 | 1.71 / 0.707 / 10 |
| 150 | 9 | 3.35 / 5.51 / 0.692 / 9 | 3.65 / 0.673 / 12 |

- **In all 9 cells the coverage end mounts the proximal pad at 0.** Its face sits at h, against h + 3 on the paper's designs. It gains 1.7–2.0 points of coverage.
- **Δ_MCP falls to 1.2–2.4 mm at 5 and 7 mm** at every flexion. At 9 mm the face floor is Δd_p 3, and Δ_MCP stays at 4.8–7.4.
- **Δ_contact at the coverage end:**
  - About equal to the paper's at 120° and at 7–9 mm of 90° and 150°.
  - Worse at 5 mm of 90° (2.67 against 1.22) and of 150° (1.32 against 0.77). The coverage end there trades contact path for coverage.
- So freeing the pins is a free gain in coverage and face place at 120°. Elsewhere the pick along the front matters (not only its coverage end).

## Plan: reproduce every R experiment on the pins-free candidates (2026-11-27)

- **Why.** The user checked the raised pin over a pad mounted at 0 in CAD (it packages) and wants the new hand to replace R. Every experiment that reports R must therefore be reproduced on it.
- **Candidates.** S2F (Study 2 with the pins free, the paper's own objectives) and MO-Pad (an added Δ_MCP objective). The choice waits for the S2F Drake and RL results.
- **Already done.**
  - MO-Pad: pull-out (21, point and hydro), dial y and pivot (35, point), cube and 7b′ transfer.
  - S2F: pull-out and dial/pivot are running.
- **Drake** (queued after the S2F Drake runs; `scratchpad run_drake_replace.sh`, the experiments' own tags), both 7 mm candidates:
  - 23 acquisition: e23 point, e23h hydro.
  - 29z fingertip disk: e29z point, e29zh hydro.
  - 38 hook: e38 point, e38h hydro.
  - 34 pad shape: e34.
  - 27 contact setpoint: e27, e27m.
  - 21 pull-out, hydro: e21.
  - 35 pivot, hydro: e35h.
  - 25 soft as thick: e25.
- **Isaac Study 3** (queued after the seed-21 cube and the S2F RL evals; `run_isaac_replace.sh`, impratio 1000 as in R's runs):
  - Condition A at μ 1.3/2.0 and ρ 15/20, for both candidates at 5, 7 and 9 mm.
  - Conditions B, C, E and the D sweep (0.1–10), and A with ke 1e3 (compliance, experiment 10), for the 7 mm candidates.
  - padk (experiments 15/17; R4 preliminary uses them), at 5, 7 and 9 mm.
- **Not rerun.** The hand-independent grids (12, 13, 18, 19), the fixed-axis family (14) and the candidate comparison (11): R plays no special part there.
- **Then.**
  - Transmission (dθ/dq, MA_wc).
  - Fig. 4 and Table I with the chosen hand.
  - USD renders.
  - Section 8.4's H_R.

## Paper decision and the Conventional hand (2026-11-27)

- **User decision.** The paper keeps R as CoDe's redesign. R is also a contact-centred redesign, so the paper calls it the result of contact-centred design and does not wait for the pins-free reruns. The paper compares three hands:
  - **CoDe** (R): least Δ_contact.
  - **Conventional:** skeleton minimization.
  - **Coverage** (the old U): coverage only.
- **Conventional** (`e25_skeleton_min.py`):
  - Study 2's own space (A and B on the link line, the mounting rule, the goal). NSGA-II on Δ_skel and −C̄_pad, seeds 0 and 1, 200 × 200.
  - Pick: the least Δ_skel at C̄_pad no more than 0.05 below CoDe's at that thickness. A fixed floor of 0.70 has no design at 9 mm, where CoDe itself has 0.695.

| t | Δ_skel | Δ_contact | Δ_MCP | C̄_pad | face |
|---|---|---|---|---|---|
| 5 | 0.06 | 3.43 | 2.04 | 0.738 | 8 |
| 7 | 0.34 | 6.61 | 4.05 | 0.684 | 10 |
| 9 | 1.94 | 8.51 | 6.19 | 0.646 | 12 |

  At 7 mm it sits near the fixed-axis bound (Δ_contact 6.61 against 6.93) with the skeleton nearly unchanged. It is the CFL counterpart of the fixed-axis family (Δ_skel 0).
- **Pre-registration (before any Conventional run).** Drake, the five tasks in point and hydroelastic contact, with the experiments' own tags, and the pull-out at 5 and 9 mm. Isaac condition A (friction) at 5/7/9 mm once the GPU is free.
  1. CoDe errs less than Conventional in every task and both contact models.
  2. In the grasping tasks (pull-out 21, acquisition 23, hook 38), Conventional errs at least as much as Coverage, since its Δ_contact is larger (6.61 against 3.91).
  3. Adding Conventional to the hands of each task keeps E's correlation with Δ_contact positive, and Δ_skel's no better than before.

**Names, revised the same day (user).**
- Paper labels: **CoDe**, **Skeleton**, **Coverage**. "Conventional" is not used.
- CoDe carries R's data now and MO-Pad's once the reruns finish (the user picked MO-Pad).
- Skeleton is the least-Δ_skel hand of the same space: `conv_e25` now, Palm-Tip (`ms_e22`) when CoDe is MO-Pad.
- Coverage is U.
- In this document the analysis names stay (Pad-Pad, MO-Pad, Palm-Tip, S2F; the `conv_e25` hands are "Palm-Tip (S2 space)").

## Pre-registration: Skeleton (conv_e25) in the RL tasks (2026-11-27, before any Skeleton rollout)

- **Hands.** Isaac keys `SK`, `SK5`, `SK9` (`conv_e25`; Δ_skel 0.34, Δ_contact 6.61, Δd_p 4 at 7 mm).
- **Runs.**
  - 7b′ and relay (face, f1), with both training seeds.
  - Cube (5/7/9 mm) under the seed-20 policy, and under seed 21 once it exists.
- **Predictions.**
  1. 7b′: Skeleton ratio < 0.3. Its Δ_contact is far above attempt 9's Dc50 of 1.7, and Palm-Tip (Δ_contact 4.0) gave 0.18.
  2. Relay: Skeleton ≥ Coverage under both anchors. The relay follows the fingertip's path, and Δ_skel is 0.34 against 3.38.
  3. Cube rotation: Skeleton ≤ 0.78. Its Δd_p is 4, like Pad-Pad and Coverage (0.70–0.75).

## Skeleton in MO-Pad's space (pins free; `e25_skeleton_min.py --free-heights`, 2026-11-27)

The least Δ_skel at C̄_pad no more than 0.05 below MO-Pad's. These hands are Skeleton's data once CoDe becomes MO-Pad. Code key `conv_e25f`; Isaac `SKF`, `SKF5`, `SKF9`.

| t | Δ_skel | Δ_contact | Δ_MCP | C̄_pad | face | A height | mount |
|---|---|---|---|---|---|---|---|
| 5 | 0.08 | 2.86 | 2.05 | 0.740 | 7.2 | 0.8 | 2.2 |
| 7 | 0.23 | 4.46 | 4.19 | 0.699 | 7.0 | 4.9 | 0 |
| 9 | 0.98 | 7.01 | 6.98 | 0.658 | 9.0 | 6.0 | 0 |

- At 7 mm it keeps the skeleton better than e22's Palm-Tip (`ms_e22`: Δ_skel 0.60, C̄_pad 0.703), under the same coverage rule as the Skeleton hands of Study 2's space.
- Runs are queued behind the paper's Skeleton runs: the Drake tasks, cube, 7b′ and relay, and Isaac condition A. The predictions are those of `conv_e25`.

## Result: Skeleton (conv_e25) in the Drake tasks (2026-11-27)

All 13 steps rc 0. E with fig4_sync.py's own pair functions (the paper's gates):

| task | contact | CoDe (R) | Coverage (U) | Skeleton |
|---|---|---|---|---|
| pull-out | point | 19.4% (46) | 71.7% (48) | **86.1%** (44) |
| pull-out | hydro | 16.7% (53) | 253.8% (54) | **387.7%** (41) |
| acquisition | point | 21.3% (22) | 74.1% (25) | **106.8%** (25) |
| acquisition | hydro | 15.8% (9) | 65.9% (12) | **91.6%** (13) |
| hook | point | 28.6% (48) | 150.0% (35) | 246% (2 of 48 pairs hold) |
| hook | hydro | 42.5% (38) | — (gate) | — (gate) |
| fingertip disk | hydro (32 draws) | 4.1% | 12.4% | **15.0%** |
| fingertip disk | point (e28z) | 17.0% | 46.8% | pending (e28z queued) |
| box pivot | point | 5.4% | 22.2% | **26.9%** |
| box pivot | hydro | 4.8% | 18.2% | **21.2%** |

- **Prediction 1 (CoDe < Skeleton in every task and model): holds** wherever Skeleton can be scored.
  - In the hook it holds the bar in 2 of 48 pairs. Scoring the hands that fail as the largest errors (the pre-registered variant) puts it last.
- **Prediction 2 (Skeleton ≥ Coverage in the grasping tasks): holds** in pull-out and acquisition, point and hydro, and in the hook.
- **Not predicted:** Skeleton also errs most in the fingertip tasks (disk hydro 15.0%, pivot 26.9 / 21.2%). Keeping the skeleton does not help where the fingertip acts, as long as the contact path changes (Δ_contact 6.61).
- **Prediction 3** (ρ with Δ_contact) waits for the rerun of each task's correlation with Skeleton added.

## Result: every Drake experiment of R reproduced on S2F and MO-Pad (2026-11-27)

The replacement batch finished with all steps rc 0. E with the paper's own pair functions; dial and pivot point on the first 32 draws. Units are %.

| task | R (CoDe now) | S2F | MO-Pad | Skeleton | Coverage |
|---|---|---|---|---|---|
| hold, point / hydro | 19.4 / 16.7 | 16.9 / 14.2 | **8.5 / 12.8** | 86.1 / 387.7 | 71.7 / 253.8 |
| acquisition, point / hydro | 21.3 / 15.8 | 18.5 / 13.0 | **10.4 / 13.5** | 106.8 / 91.6 | 74.1 / 65.9 |
| hook, point / hydro | 28.6 / 42.5 | 25.0 / 36.1 | **13.4 / 19.8** | 246 (2 pairs) / — | 150.0 / — |
| fingertip disk, point / hydro | 17.0 / 4.1 | 16.2 / 4.0 | 18.0 / 5.9 | 45.9 / 15.0 | 47.9 / 12.4 |
| box pivot, point / hydro | 5.2 / 4.8 | 5.2 / 4.7 | 6.7 / 6.3 | 26.8 / 21.2 | 20.3 / 18.2 |

Table I, Drake rows (signed median bias, E in brackets):

| row | R | S2F | MO-Pad |
|---|---|---|---|
| pad shape (point) | −43% (55) | −45% (52) | −18% (47) |
| friction, point | −19% (19) | −17% (17) | −8% (8) |
| friction, hydro | −15% (17) | −13% (14) | +12% (13) |
| softer pad, hydro | −8% (16) | −4% (14) | +16% (17) |
| contact setpoint | −13% (14) | −11% (12) | −8% (8) |

- **MO-Pad halves R's error in the three grasping tasks**, point and hydro. In the fingertip tasks it errs 1–2 points more than R.
- **S2F errs as R does or slightly less everywhere.**
- Replacing R by MO-Pad therefore strengthens the paper's grasping results and leaves the fingertip ones within 2 points.
- The Isaac reruns (friction dose, stiffness, protocols, padk) are queued on the GPU behind Skeleton's.

## Result: Skeleton (conv_e25) in the coin drag (30b) and the three-finger twist (39) (2026-11-27)

`run_drake_alt.sh`, all steps finished. The analyses score only `analyze.DESIGN`, so Skeleton was scored with each script's own `o_pairs`/`o_keep` and `hand_error` (the same O pairs and gates).

| task | contact | CoDe (R) | Coverage (U) | Skeleton |
|---|---|---|---|---|
| coin drag | point (64 pairs) | 2.2% | 16.3% | **19.3%** (64/64) |
| coin drag | hydro (32 pairs) | 3.2% | 15.9% | **20.6%** (32/32) |
| twist, 3 fingers | point (56 O pairs) | 16.1% (48) | 54.4% (50) | **71.9%** (28; 22 over-pressed) |
| twist, 3 fingers | hydro | 13.4% (50) | — (0 pairs; 48 over-pressed) | — (0 pairs; 32 over-pressed) |

- Prediction 1 of the Skeleton block (CoDe < Skeleton) holds in both tasks where Skeleton can be scored.
- Skeleton errs more than Coverage in both, as in the other fingertip tasks: keeping the fingertip path does not keep the pad-contact path (Δ_contact 6.61).
- Twist hydro stays unscorable for both baselines under the force gate; only CoDe passes it.

## Pre-registration: coin drag and twist on MO-Pad, S2F and pins-free Skeleton (2026-11-27, before any run)

These two tasks are Fig. 4 candidates in place of hook and acquisition, so the replacement hands need them.

- Hands: `mrmid_free` (MO-Pad), `s2free_s2f` (S2F), `conv_e25f` (pins-free Skeleton). Same draws, tags and gates as R (`e30b`, `e30bh`, `e39`, `e39h`).
- Predictions:
  1. Coin drag (a fingertip task): MO-Pad and S2F within 3 points of R, point and hydro. In the disk and pivot they were within 2.
  2. Twist (a three-finger grasp): MO-Pad ≤ R at point contact, as in the three grasping tasks where it halved R's error.
  3. `conv_e25f` errs more than R in both tasks.
- Fails if MO-Pad errs more than R + 3 points in either task at point contact.

## Result: coin drag and twist on MO-Pad, S2F and pins-free Skeleton (2026-11-27)

`run_drake_alt2.sh`, all steps rc 0. Scored as for Skeleton above (each script's own `o_pairs`/`o_keep` and `hand_error`).

| task | contact | R | MO-Pad | S2F | pins-free Skeleton | Skeleton | U |
|---|---|---|---|---|---|---|---|
| coin drag | point | 2.2% | **8.8%** | 6.2% | 4.9% | 19.3% | 16.3% |
| coin drag | hydro | 3.2% | **10.2%** | 8.3% | 4.9% | 20.6% | 15.9% |
| twist, 3 fingers | point | 16.1% | **7.0%** (48) | 15.5% (48) | 49.9% (48; 6 over-pressed) | 71.9% (28) | 54.4% (50) |
| twist, 3 fingers | hydro | 13.4% (50) | **12.9%** (54) | 24.0% (46) | — (0) | — (0) | — (0) |

- **Prediction 1 fails:** in the coin drag, MO-Pad errs 6.6 points (point) and 7.0 points (hydro) more than R, against the pre-registered limit of 3. S2F also misses it (4.0 and 5.1 points).
- **Prediction 2 holds:** in the twist, MO-Pad 7.0% < R's 16.1% at point contact. MO-Pad is also the only redesign besides R that scores in twist hydro (12.9% against R's 13.4%).
- **Prediction 3 holds:** pins-free Skeleton errs more than R in both tasks (coin 4.9 vs 2.2 / 3.2%; twist 49.9 vs 16.1%).
- **Fail condition met** (MO-Pad > R + 3 points at point contact, coin).
- Not predicted: in the coin drag, pins-free Skeleton (4.9%) errs less than MO-Pad. The coin is dragged by the fingertip against a table, as in the ball roll (41), whose error followed the path's normal component rather than Δ_contact.
- For Fig. 4: twist (39) is consistent with the grasping tasks, where MO-Pad halves R's error. The coin drag is the one task where replacing R by MO-Pad costs more than the 1–2 points seen in the disk and the pivot.

## Result: cube transfer with the second training seed (2026-11-27)

The seed-21 release (v2c seg2 `model_1497`) was evaluated on Original and 16 hands, all rc 0 (`logs/pr_mr_transfer_cube_s21/`). Ratios use the same summary (`TRANSFER_DIR=... SUMMARY_HANDS=... mr_transfer_summary.py`).

- **Validity gate: passed.** Original's held-out rotation is 786° / 735° at μ 1.3 (two eval seeds), above half of seed 20's 919°.

| hand | Δt | rotation ratio (s20 → s21) | full≤30° ratio (s21) |
|---|---|---|---|
| MRM5 | 0 | 0.989 → **0.975** | 1.017 |
| MR5 | 0.9 | 0.963 → 0.818 | 0.983 |
| MRM (MO-Pad) | 1 | 0.938 → **0.564** | 0.931 |
| R5 / U5 | 2 | 0.84 / 0.84 → 0.243 / 0.273 | 0.118 / 0.182 |
| MR / MR9 / MRM9 | 3 | 0.77–0.82 → 0.213 / 0.213 / 0.215 | 0.053 / 0.085 / 0.076 |
| R (Pad-Pad) / U / MRbz / MRMbz | 4 | 0.70–0.75 → 0.187 / 0.211 / 0.205 / 0.201 | 0.032 / 0.116 / 0.071 / 0.069 |
| R9 / U9 | 6 | 0.66 / 0.62 → 0.166 / 0.171 | 0.046 / 0.052 |
| Palm-Tip (MS) / MRS | – | → 0.516 / 0.658 | 0.908 / 0.966 |

Spearman ρ over the same 14 hands, one-sided permutation (20,000): rotation ratio against Δt **−0.98** (p < 5e-5); full≤30° ratio against Δt **−0.88** (p 0.00015).

- **Prediction 1 holds:** ρ(ratio, Δt) −0.98 ≤ −0.8.
- **Prediction 2 holds:** MO-Pad exceeds Pad-Pad by 0.38 (0.564 vs 0.187).
- **Prediction 3 holds:** Palm-Tip at 0.516 lies between Pad-Pad and MO-Pad.
- **Not predicted: the second policy is far more sensitive to the face shift.**
  - Every hand with Δt ≥ 2 keeps only 17–27% of Original's rotation, against 62–84% under seed 20.
  - MO-Pad (Δt 1) keeps 56%, and only the Δt ≤ 1 hands keep most of the full turns (0.93–1.02).
  - The order by Δt is the same under both seeds; the size of the loss depends on the policy.
  - So the cube result is replicated as an ordering, not as a magnitude.

## Result: Skeleton (conv_e25) in the RL tasks (2026-11-27)

`run_sk_rl.sh`, all evals rc 0. Ratios are the hand's score over Original's (relay: rotation at μ 1.3 over Original's under the face anchor, mean of the two eval seeds, per training seed s20 / s21).

| task | Pad-Pad (R) | Coverage (U) | **Skeleton** | MO-Pad | Palm-Tip |
|---|---|---|---|---|---|
| 7b′ walk ratio (s20 / s21) | 0.72 / 0.61 | 0.08 / 0.07 | **0.025 / 0.009** | 0.71 / 0.59 | 0.22 / 0.13 |
| relay, face (s20 / s21) | 0.57 / 0.56 | 0.72 / 0.68 | **0.82 / 0.76** | 0.57 / 0.56 | 0.90 / 0.95 |
| relay, f1 (s20 / s21) | 0.68 / 0.69 | 0.84 / 0.89 | **0.87 / 0.94** | 0.69 / 0.63 | 0.95 / 1.02 |
| cube rotation ratio (s20 / s21) | 0.749 / 0.187 | 0.703 / 0.211 | **0.645 / 0.198** | 0.938 / 0.564 | 0.822 / 0.516 |

Skeleton at 5 / 9 mm in the cube: 0.810 / 0.531 (s20), 0.342 / 0.153 (s21).

- **Prediction 1 holds:** 7b′ ratio < 0.3 (0.017 pooled). Skeleton walks least of all hands; its Δ_contact (6.61) is the largest.
- **Prediction 2 holds:** relay Skeleton ≥ Coverage under both anchors and both training seeds (face 0.82 / 0.76 vs 0.72 / 0.68; f1 0.87 / 0.94 vs 0.84 / 0.89). The relay follows the fingertip's path; Skeleton keeps it (Δ_skel 0.34), and only Palm-Tip (Δ_skel 0.60) does better.
- **Prediction 3 holds:** cube ratio ≤ 0.78 (0.645 under s20). With Δt 4, like Pad-Pad and Coverage, it sits with them under both seeds.
- **Reading:** the three RL tasks split the hands by what each keeps.
  - The walk (pad contact) follows Δ_contact.
  - The relay (fingertip-end contact) follows Δ_skel.
  - The cube follows the proximal face's place (Δt).
  - MO-Pad is near the top in the walk and the cube and near Pad-Pad in the relay. Skeleton is the reverse.

## Result: Skeleton (conv_e25) in Isaac Lab, condition A (2026-11-27)

`run_isaac_replace.sh` Skeleton block, all steps rc 0 (ISAAC_CONV_DONE). Friction-dose error E with `sim/drake/analyze.py error(load_isaac, ...)`, the same function and pairs as the 18 hands.

| thickness | CoDe (R) | Coverage (U) | Skeleton |
|---|---|---|---|
| 5 mm | 3.4% (54) | 51.9% (52) | **120.0%** (56) |
| 7 mm | 21.0% (48) | 123.6% (52) | **222.3%** (52) |
| 9 mm | 40.7% (48) | 106.0% (52) | **236.1%** (51) |

- Skeleton errs most at every thickness, as in Drake. Its fingertip path stays within 0.06–1.94 mm of O's, while its contact surfaces move 3.4–8.5 mm.
- The other conditions of the block (B, C, D sweep, E, padk) are logged under `sim/isaaclab/logs/study3/two_pad/tp_*conv_e25*`; Table I no longer carries Skeleton, so they are not tabulated here.

## Result: Skeleton with the convex pad in Isaac Lab, condition A (2026-11-27)

`run_gpu_extra.sh` (GPU_EXTRA_DONE). Tag `A-crowned-crowned` for `conv_e25`, scored with `robust_analysis.shape_effect` logic and `table_signed.pairs_from` against O's convex-pad effect (the Table I "convex pad, MuJoCo-Warp" setting).

| hand | pairs | signed median | E |
|---|---|---|---|
| CoDe (R) | 13 | −27% | 27% |
| Coverage (U) | 5 | +117% | 157% |
| Skeleton | 5 | +83% | 198% |

- Skeleton overstates the convex pad's effect and errs more than Coverage in magnitude, on the same 5 pairs. Few pairs pass for the two thick-contact hands (O's convex-pad gate), so this is a small sample.

## Result: pins-free Skeleton (conv_e25f) in the RL tasks (2026-11-27)

`run_skf_rl.sh`, every eval rc 0. Ratios are computed as in the Skeleton (conv_e25) RL result: μ 1.3, mean of the two eval seeds, per training seed s20 / s21. Relay values are over Original's face-anchor rotation.

| task | Skeleton (conv_e25) | **pins-free Skeleton (SKF)** | Coverage (U) |
|---|---|---|---|
| 7b′ walk (s20 / s21) | 0.025 / 0.009 | **0.068 / 0.014** | 0.077 / 0.071 |
| relay, face (s20 / s21) | 0.82 / 0.76 | **1.01 / 0.98** | 0.72 / 0.68 |
| relay, f1 (s20 / s21) | 0.87 / 0.94 | **1.08 / 1.01** | 0.84 / 0.89 |
| cube, 7 mm (s20 / s21) | 0.645 / 0.198 | **0.819 / 0.489** | 0.703 / 0.211 |

SKF at 5 / 9 mm in the cube: 0.879 / 0.627 (s20) and 0.600 / 0.185 (s21).

Against the predictions registered for `conv_e25` and carried over to SKF:
- **Prediction 1 holds.** The walk ratio is below 0.3 (0.068 / 0.014). SKF's Δ_contact is 4.46.
- **Prediction 2 holds.** In the relay, SKF is at or above Coverage under both anchors and both training seeds. It keeps O's relay performance, the best of any hand so far. Its Δ_skel of 0.23 is the smallest.
- **Prediction 3 fails.** The prediction was cube ≤ 0.78, and SKF scores 0.819 under s20. The prediction was copied from conv_e25 (Δd_p 4) without updating SKF's own palm shift. With the pins free, SKF mounts its pads at 0, so its proximal face sits at 7.0 mm: Δd_p is about 1 at 7 mm and 3 at 9 mm, as for S2F. Read against palm shift, the result fits the cube rule:
  - SKF (Δd_p 1) does better than conv_e25, Pad-Pad and Coverage (Δd_p 4) under both seeds.
  - SKF9 (Δd_p 3) falls back to their level.
  - SKF stays below MO-Pad (0.938 / 0.564).
  
  This reading is post hoc and is not counted as a confirmed prediction.
- **For the paper.** If Skeleton becomes conv_e25f, the walk/relay split is unchanged: walk follows Δ_contact and relay follows Δ_skel. Fig. 5's cube column would show palm shifts of 1 (SKF) against 4 (Coverage) instead of three equal values. The swap waits for MO-Pad's Isaac reruns, so that CoDe and Skeleton change together.

## Result: S2F in the RL tasks (2026-11-27)

`run_s2f_rl.sh`, then `run_s2f_cube_redo.sh`. The first S2F and S2F9 cube grasp generations failed for lack of GPU memory (rc 1, around 18:00, alongside a render job). The redo has every eval at rc 0. Ratios are computed as in the Skeleton RL result (μ 1.3, mean of the two eval seeds, per training seed s20 / s21; relay over Original's face-anchor rotation).

| task | **S2F** | Pad-Pad (R) | MO-Pad |
|---|---|---|---|
| 7b′ walk (s20 / s21) | **0.737 / 0.613** | 0.724 / 0.612 | 0.712 / 0.593 |
| relay, face (s20 / s21) | **0.60 / 0.57** | 0.57 / 0.56 | 0.57 / 0.56 |
| relay, f1 (s20 / s21) | **0.62 / 0.62** | 0.68 / 0.69 | 0.69 / 0.63 |
| cube 5 mm (s20 / s21) | **0.933 / 1.021** | — | — |
| cube 7 mm (s20 / s21) | **0.926 / 0.496** | 0.749 / 0.187 | 0.938 / 0.564 |
| cube 9 mm (s20 / s21) | **0.800 / 0.213** | — | MRM9 0.815 / 0.215 |

- **Prediction 3 holds** (seed-20 policy, as registered): S2F 0.926 ≥ 0.90, S2F5 0.933 ≥ 0.90, and S2F9 0.800 within 0.75–0.88. Under seed 21 the order is the same: S2F (0.50) is far above Pad-Pad (0.19), and S2F9 (0.21) is at Pad-Pad's level.
- **Prediction 4 holds:** the walk ratio is 0.737 / 0.613, above 0.55 and level with Pad-Pad (Δ_contact 0.85 against 0.86).
- **Relay** (no prediction): about Pad-Pad's, as Δ_skel (5.24) predicts.
- **Reading:** in RL, S2F matches MO-Pad at 5 and 7 mm, by keeping the face place (Δd_p ≤ 1) without MO-Pad's extra objective. At 9 mm both fall to Δd_p 3 and score alike, so here MO-Pad's extra objective buys nothing over S2F in the cube.

## Result: S2F and MO-Pad in Isaac Lab, Study 3 reruns (2026-11-27)

`run_isaac_replace.sh` (ISAAC_REPLACE_DONE), 0 failed steps. These reruns had no registered prediction; they reproduce R's runs, so they are reported descriptively.

Condition A (friction), `analyze.error(load_isaac, ...)`, the same function and pairs as the 18 hands: E (pairs).

| thickness | R (Pad-Pad) | S2F | MO-Pad | Coverage (U) |
|---|---|---|---|---|
| 5 mm | 3.4% (54) | 6.3% (56) | 3.5% (56) | 51.9% (52) |
| 7 mm | 21.0% (48) | 17.2% (52) | **9.5%** (55) | 123.6% (52) |
| 9 mm | 40.7% (48) | **31.7%** (48) | 38.1% (48) | 106.0% (52) |

Table I settings and command protocols at 7 mm (`table_signed` functions; signed median / E, pairs):

| setting | R | S2F | MO-Pad | U |
|---|---|---|---|---|
| friction, Isaac point | −21 / 21 (48) | −17 / 17 (52) | −0 / **10** (55) | +115 / 124 (52) |
| stiffness, Isaac point | −25 / 25 (48) | −22 / 23 (52) | −0 / **8** (52) | +173 / 173 (48) |
| compliant pad, Isaac point | −14 / 14 (51) | −11 / 12 (52) | +0 / **10** (52) | +125 / 129 (51) |
| convex pad, Drake point | −43 / 55 (52) | −45 / 52 (48) | −18 / **47** (52) | +70 / 123 (39) |
| friction, Drake point | −19 / 19 (46) | −17 / 17 (46) | −8 / **8** (46) | +70 / 72 (48) |
| friction, Drake hydro | −15 / 17 (53) | −13 / 14 (46) | +12 / **13** (54) | +254 / 254 (54) |
| compliant pad, Drake hydro | −8 / 16 (44) | −4 / **14** (43) | +16 / 17 (51) | +347 / 347 (22) |
| contact setpoint | −13 / 14 (46) | −11 / 12 (46) | −8 / **8** (46) | +17 / 26 (49) |
| torque | +13 / 17 (56) | +11 / 16 (56) | +8 / **11** (56) | −0 / 28 (44) |
| touch | −22 / 23 (56) | −17 / 18 (56) | −14 / **14** (56) | −17 / 20 (44) |
| force | +0 / 3 (44) | +0 / 0 (44) | +0 / 2 (44) | −4 / 7 (48) |

- The convex pad in Isaac (`A-crowned-crowned`) was run for R, U and Skeleton only, not for the candidates.
- **MO-Pad errs least at 7 mm in every setting but one.** In the Isaac point settings its median bias is about 0: E 8–10%, against R's 14–25%. The exception is the compliant pad with hydroelastic contact (17% against S2F's 14% and R's 16%).
- **S2F errs a little less than R everywhere.** It improves on R by 1–4 points, and at 9 mm in condition A by 9 points (31.7% against 40.7%).
- At 9 mm the two candidates are close (32% and 38%), as in the cube (Δd_p 3 for both).
- **Reading:**
  - MO-Pad's extra Δ_MCP objective halves the 7 mm error of R and S2F in the commands that reuse the setpoint.
  - S2F, from the paper's own objectives with the pins free, is a modest gain over R.
  - Both keep the ordering CoDe ≪ Coverage in every setting.

## Paper decision: R stays as CoDe (2026-11-27, evening)

- **Decision.** After the reruns above, the user chose: "지금은 R 유지에 그냥 CoDe 라고 부르자; 이미 보여 주고 싶은 결과가 나온건 다시 할 필요없어."
  - The paper keeps R as CoDe's redesign. Skeleton stays `conv_e25`, with the pins fixed as in R.
  - No figure or table moves to MO-Pad, S2F or SKF.
  - Results already shown are not rerun.
- **Status of the other hands.** MO-Pad, S2F and SKF stay here as a possible later improvement. MO-Pad halves the 7 mm error, which is the main reason to revisit them.
- **Not tabulated.** SKF's Isaac condition A finished (GPU_SKF_DONE, all rc 0) under `sim/isaaclab/logs/study3/two_pad/tp_*conv_e25f*`. With this decision it is not tabulated.
