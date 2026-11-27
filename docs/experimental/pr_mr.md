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
