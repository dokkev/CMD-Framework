# Grasp-set benchmark (kinematic, task-agnostic)

Status: experimental. Pre-registered 2026-11-26, before any of the three hands was run.
Code: `design/core/optimization/thumb_layout/grasp_set.py` (benchmark), `design/scripts/optimize/thumb_layout/grasp_set.py` (runner).
Run data: `design/output/thumb_layout/grasp_set/<hand tag>/` (not committed).

## Why

The Kapandji scores (`thumb_layout_opt.md`) measure where the thumb pad can reach. They do not say whether the hand can hold things.
This benchmark asks a standard grasp-analysis question for a fixed hand: over a fixed, diverse object set, for which objects and grasp types does a
kinematically feasible, collision-free, force-closure grasp exist, and how good is the best one found (Ferrari–Canny ε)?
It is task-agnostic (no task, no human grasp taxonomy beyond the contact counts), kinematics-only (no dynamics, no actuator torques) and runs on CPU.
Every ingredient is textbook: point contact with friction, Nguyen's antipodal test, the Ferrari–Canny L1 quality on a discretised friction cone
(Ferrari & Canny 1992; Miller & Allen's GraspIt convention for the torque scale), and a multi-start least-squares search whose force-closure term is
the unit-normal-force residual used by recent differentiable grasp synthesis (Liu et al. 2021; DexGraspNet, Wang et al. 2023).
The final feasibility and ε are always computed exactly; the surrogate only steers the search.

## Hands (first run)

O fingers: `design/output/sim/study3/hands/cfl_q120_h3to7_h0.yaml`.

| tag | thumb |
|---|---|
| `O` | prototype, `ThumbDesign.prototype(p)` |
| `O_thumbopt` | `ThumbDesign(-50.5, -10.5, 34.0, "proximal", -14, -16, 14)` |
| `O_pitch` | `ThumbDesign(-50.5, -10.5, 34.0, "proximal", 0, -16, 0)` (functional Kapandji K6) |

The hand list is a runner input (hand file + thumb spec), so the bi-level finger-length hands can be fed later.

## Object set (17 objects)

Objects are free in pose (position and orientation are search variables); the hand is fixed.
- Sphere, diameter 10, 20, 35, 50, 70 mm.
- Cylinder, length 100 mm with flat caps, diameter 10, 20, 35, 50, 70 mm (axis orientation free).
- Cube, side 10, 20, 35, 50, 70 mm (orientation free).
- Thin rod: cylinder, diameter 5 mm, length 100 mm.
- Flat disc: cylinder, diameter 50 mm, thickness 5 mm.

The 15 core objects carry the size-range metric; the rod and disc are extra pairs in the counts.
Geometry is exact (analytic signed distance and normal for each shape).

## Grasp types (5 per object, 85 object–grasp pairs per hand)

- `pinch_index`, `pinch_middle`, `pinch_ring`: thumb pad + that finger's distal pad (2 contacts).
- `tripod`: thumb pad + index and middle distal pads (3 contacts).
- `four`: thumb pad + one pad of each of index, middle, ring (proximal or distal, chosen by the search; 4 contacts).
- Palm contact is **not** used as a contact (it would need a palm surface model beyond the housing boxes); the palm is an obstacle only.

## Hand model (reused from `evaluator.py`, unchanged)

- Kinematics: `ThumbKinematics`, `FingerKinematics` (PIP four-bar table), joint limits `LIMITS_DEG`.
- Pads are flat rectangles; the contact point is a search variable on the pad face:
  - finger pads: along the pad between its ends 2 mm in (`PAD_EDGE_INSET_MM`), laterally within ±(19/2 − 2) = ±7.5 mm; normal = frame +Y;
  - thumb pad: along within the evaluator's functional reach (pad length/2 − 2 mm), laterally within ±(18/2 − 2) = ±7 mm (sim `THUMB_PAD_MM` width 18); normal = E frame +X.
- Collision primitives: the evaluator's capsules (thumb RAA servo, metacarpal, proximal, distal, pad; each finger's FE servo, PIP servo, proximal and distal
  segments) and the palm housing boxes (hood, block, bridge, heel).

## Feasibility (all must hold)

1. **Contact.** Each contact point on its pad is within 1 mm of the object surface (|signed distance| ≤ 1 mm).
2. **Alignment.** The pad normal opposes the object's outward surface normal at the nearest surface point within the friction cone:
   angle(pad normal, −surface normal) ≤ atan μ (26.6 deg at μ 0.5, 45 deg at μ 1.0).
3. **Joint limits.** All joints within `LIMITS_DEG`; the thumb's four joints 5 deg inside their limits (the functional-Kapandji margin, which applies to the
   thumb). Finger joints within their plain limits. Enforced as bounds in the search.
4. **Collision-free** (signed distance ≥ −1 mm, the evaluator's tolerance), apart from the contacting pads touching the object:
   - object vs every hand capsule (contacting segments included: their palmar surface is the pad face, so a touching object is at ≈ 0) and vs the palm boxes;
   - thumb primitives vs every finger capsule; thumb vs palm boxes with the evaluator's pairs (proximal, distal, pad vs all boxes; RAA servo and
     metacarpal vs hood and block);
   - each finger's proximal and distal segments vs the neighbouring fingers' capsules.
   - Not checked (as in the evaluator): fingers vs palm boxes.
   - Non-contacting fingers are free (their joints are search variables) and must only stay clear.
5. **Force closure** with the object's surface normals at the contact points (nearest surface points), μ as in 2:
   - 2 contacts: Nguyen's antipodal test (the segment between the contacts lies inside both friction cones).
     Torsional friction is ignored (point contact with friction), so a pinch is force closure **up to rotation about the contact line**;
     that convention is used throughout.
   - ≥ 3 contacts: Ferrari–Canny ε_L1 > 0 on the 6-D grasp wrench space: each friction cone discretised into 8 edges with unit normal force, torques about
     the object's centroid divided by ρ = the object's largest centroid-to-surface distance (GraspIt convention), ε = distance from the origin to the
     convex hull's boundary (0 if the origin is not strictly inside; a degenerate hull counts as 0).
   - **Quality for a pinch:** the same ε_L1 computed in the 5-D wrench subspace the two contacts span (the torque about the contact line removed).
     For two contacts ε > 0 in that subspace is equivalent to Nguyen's antipodal condition (checked in the tests).

μ = 0.5 is the main value; μ = 1.0 is reported. One search serves both: the search aligns the normals as far as it can, and each candidate is then checked
at both μ.

## Search

For each hand, object and grasp type:
- **Variables:** thumb joints (4), all three fingers' joints (9), object position (3) and orientation (rotation vector about a seed orientation, 3;
  sphere orientation ignored), and each contact's along/lateral place on its pad (2 per contact).
- **Seeds** (geometric matching, the evaluator's idea): the thumb joint grid (10 deg, within the 5 deg margin, clear of the palm) and each finger's
  collision-free 10 deg grid give pad points and normals; each pad pose proposes an object centre `p + r·n` (r: radius, or half side).
  Candidate centres from the thumb are matched (KD-tree) to the nearest proposal of each required finger pad; seeds are ranked by the mismatch plus the
  unit-normal net-force residual, deduplicated, and the best `S` kept (S = 48 by default), with a seeded random tie-break so independent runs differ.
  Seed object orientation: cylinder axis perpendicular to the thumb–finger centre line (random roll), cube face normal along the thumb normal.
  Non-contacting fingers start extended (0).
- **Refinement:** batched Levenberg–Marquardt with finite-difference Jacobians and box bounds (clipping), 80 iterations, on the residual
  [signed distance of each contact; 20 mm × (pad normal + surface normal); collision hinge max(0, −signed distance) per primitive pair;
  5 mm × (sum of inward unit normals) and the matching torque / ρ (the force-closure surrogate)].
- **Result:** every seed's end point is checked exactly (feasibility 1–5). Found = any seed feasible; quality = the best ε among feasible seeds.
- **Misses:** every not-found pair is rerun with 4× the seeds (192) and a different random stream; the number that flip to found is reported.
  A pair counts as found if either run finds it (the rerun's result is the reported one when it flips).

## Metrics per hand

- Feasible pairs (out of 85, and out of 75 core) at μ 0.5 and μ 1.0.
- ε over pairs (μ 0.5): median over the pairs feasible on **all compared hands** (the paired, common set); also the median over all 85 pairs with an
  infeasible pair counted as 0, and the worst (minimum) ε over the hand's own feasible pairs.
  Pinch ε (5-D) and multi-contact ε (6-D) are also reported separately.
- Size range per grasp type and shape: the smallest and largest core size with a feasible grasp.
- Per-object detail: found, ε, pad choice (four), the joint angles and object pose of the best grasp (`results.json`, `pairs.csv`).

## Comparison rule (registered)

- Run-to-run spread: each hand is run with 2 independent random streams (seed 0, seed 1); the spread is |median ε(seed 0) − median ε(seed 1)| on the
  common set, the larger over the hands.
- Hand A is **better** than hand B only if A's feasible pairs at μ 0.5 exceed B's by ≥ 2, **or** A's common-set median ε exceeds B's by more than the
  spread. If neither holds in either direction the hands are **not distinguished**. If they disagree (A more pairs, B higher ε) both are reported and the
  feasible-pair count decides (it is the primary measure).
- Reported numbers use seed 0 plus the miss reruns.

## Sanity checks (run before the three hands)

1. **Known answer, pinch facing.** For the thumb–index pinch on a 20 mm sphere, the best angle between the pad normals over the search's
   end points should match the evaluator's pinch at 14–20 mm (`thumb_layout_opt.md`: O 18.0 deg, O_thumbopt 8.3 deg) to within a few degrees;
   it may be lower because the benchmark lets the contacts move over the pad (the evaluator pins the thumb pad centre on the finger pad's normal line).
   The evaluator's own `_pinch` is rerun alongside for the reference values.
2. **Trivial case.** The 20 mm sphere thumb–index pinch is found on every hand.
3. **ε unit tests.** Two antipodal contacts on a sphere are force closure (and not when tilted beyond the cone); three contacts 120 deg apart on a
   sphere's equator give ε > 0; three contacts on one hemisphere's side give ε = 0.
4. **Search completeness.** The miss reruns above (flip count).

## Results (2026-11-26)

Run: seed 0 (48 seeds per pair) plus the registered miss rerun (192 seeds), seed 1 for the spread; 6 workers, about 18 + 12 min.
Outputs: `design/output/thumb_layout/grasp_set/{O,O_thumbopt,O_pitch}/seed{0,1}/`, the comparison
`compare_O_O_thumbopt_O_pitch.json`, the sanity check `sanity_pinch.json`, the post-hoc probe `posthoc_deep_misses.json`.
Figure: `report/experimental/figures/grasp_set_eps.png` (best ε per object and grasp type, μ 0.5).

**Before the run.** A pilot on another morphology (R fingers, prototype thumb, `pilot_R/`) checked that the code ran and how long it took
(85 pairs in 4.6 min). Nothing in the protocol was changed after it.

### Sanity checks

1. **Known answer, pinch facing** (thumb–index, best angle between the pad normals; evaluator's `_pinch` rerun at the band's gaps):

| hand | evaluator 6–9 / 9–14 / 14–20 mm | benchmark, sphere 10 | benchmark, sphere 20 |
|---|---|---|---|
| O | 26.8 / 21.4 / 16.9 deg | 23.1 | 16.5 |
| O_thumbopt | 16.3 / 12.7 / 8.1 | 14.3 | 7.7 |
| O_pitch | 21.9 / 18.0 / 13.4 | 18.4 | 15.5 |

   At the 20 mm gap the benchmark's search (free object, contacts free on the pads) lands 0.4 deg below the evaluator on O and O_thumbopt, as expected
   (the evaluator pins the thumb pad centre on the finger pad's normal line), and 2.1 deg above it on O_pitch. The benchmark minimises the
   force-closure residual, not the facing angle, so its facing is a by-product, not a minimum; the agreement is close enough to trust the kinematics
   and the contact model. The 10 mm values sit between the evaluator's 9 and 14 mm values. The doc's earlier values (O 18.0, O_thumbopt 8.3 at
   14–20 mm) are reproduced to within 1.1 deg.
2. **Trivial case.** The 20 mm sphere thumb–index pinch is found on all three hands (ε 0.24–0.28).
3. **ε unit tests** pass (`design/tests/test_grasp_set.py`), including the antipodal/ε agreement for two contacts away from the cone boundary.
4. **Search completeness — the search is not complete for tripod and four-contact grasps.**
   - Registered rerun (4× seeds): 4 of 13 misses flipped on O, 1 of 10 on O_thumbopt, 3 of 10 on O_pitch (at μ 0.5).
   - Post hoc (not registered): every remaining miss rerun with 16× seeds (768). 6 of 25 more flipped: O sphere10 tripod, box10 tripod, sphere50 four;
     O_thumbopt sphere20 tripod, box10 tripod; O_pitch sphere50 four. Every flip is a tripod or four-contact grasp on a small object or the 50 mm sphere,
     with a small ε (0.016–0.14).
   - Pinches never needed a rerun: all 51 pinch pairs were found on every hand in the first pass.
   - Still not found after 768 seeds, on all three hands: four-contact on sphere 10/20/35 and cube 10/20; plus sphere10 tripod on O_thumbopt and
     O_pitch, disc50 four at μ 0.5 on O. The closest end points miss by 2–20 mm in contact or 20–60 deg in alignment, so these look geometrically
     infeasible (three fingers 25 mm apart with 19 mm pads cannot all reach a small object), but this is not proven.

### Per hand (registered numbers: seed 0 + rerun)

| hand | feasible μ 0.5 (core 75) | feasible μ 1.0 | median ε, common 75 pairs (pinch / multi) | median ε, all 85 (miss = 0) | worst ε (own feasible) |
|---|---|---|---|---|---|
| O | 76 (67) | 77 | 0.121 (0.209 / 0.051) | 0.094 | 0.004 |
| O_thumbopt | 76 (66) | 77 | 0.222 (0.268 / 0.084) | 0.216 | 0.018 |
| O_pitch | 78 (68) | 79 | 0.223 (0.280 / 0.095) | 0.213 | 0.003 |

μ 1.0, common-set median ε: O 0.327, O_thumbopt 0.415, O_pitch 0.418.
Run-to-run spread of the common-set median ε (seed 0 vs seed 1): O 0.020, O_thumbopt 0.0002, O_pitch 0.001 (μ 0.5); the rule uses the largest, 0.020.

By grasp type (feasible at μ 0.5, of 17): every pinch 17/17 on every hand; tripod O 15, O_thumbopt 14, O_pitch 16; four O 10, O_thumbopt 11, O_pitch 11.

Size range (core sizes with a feasible grasp, μ 0.5):

| grasp | O | O_thumbopt | O_pitch |
|---|---|---|---|
| pinch (index, middle, ring) | all shapes 10–70 | all shapes 10–70 | all shapes 10–70 |
| tripod | sphere 20–70, cylinder 10–70, cube 20–70 | sphere 35–70, cylinder 10–70, cube 20–70 | sphere 20–70, cylinder 10–70, cube 10–70 |
| four | sphere 70, cylinder 10–70, cube 35–70 | same | same |

(With the post-hoc probe: O's tripod reaches sphere 10 and cube 10, O_thumbopt's sphere 20 and cube 10, and O and O_pitch reach four on sphere 50.)

Median ε by grasp type over each hand's feasible pairs (μ 0.5): pinch index / middle / ring: O 0.22 / 0.20 / 0.12, O_thumbopt 0.29 / 0.27 / 0.26,
O_pitch 0.29 / 0.28 / 0.27; tripod O 0.064, O_thumbopt 0.070, O_pitch 0.088; four O 0.037, O_thumbopt 0.090, O_pitch 0.091.
O's pinch quality drops from index to ring (the prototype thumb faces the ring pad badly: median best-grasp facing 29 deg against ≤ 2 deg on the tilted
thumbs); the tilted thumbs hold all three fingers' pinches at about the same ε. The largest gaps are thin objects (10 mm cylinder, 5 mm rod) pinched by
the middle and ring fingers (figure).

Four-contact pad choice in the best grasps: mostly all three distal pads (O 4/10, O_thumbopt 6/11, O_pitch 8/11); the rest mix proximal and distal.

### Verdict under the registered rule (μ 0.5)

| pair | Δ feasible | Δ common median ε (spread 0.020) | registered verdict |
|---|---|---|---|
| O_thumbopt vs O | 0 | +0.101 | **O_thumbopt better** (ε) |
| O_pitch vs O | +2 | +0.102 | **O_pitch better** (both) |
| O_pitch vs O_thumbopt | +2 | +0.001 | **O_pitch better** (count only) |

- **Robust:** both tilted thumbs give better grasps than the prototype thumb. The median ε gain (+0.10, five times the spread) holds at μ 1.0
  (+0.09) and comes from every grasp type, most of all the middle and ring pinches and the four-contact grasps.
- **Not robust:** O_pitch over O_thumbopt rests only on the feasible count, +2 pairs (sphere20 tripod and cube10 tripod). The post-hoc 768-seed
  probe finds both on O_thumbopt too, and with all runs pooled the counts are O 79, O_thumbopt 78, O_pitch 79 (μ 0.5): no difference of 2 remains.
  The registered verdict stands as written, but it is a search artefact; read O_pitch and O_thumbopt as **not distinguished** by this benchmark.
- Feasibility counts barely separate the hands: the pinches are feasible everywhere, and the differences sit in small-object tripod and four-contact
  pairs, where the search is incomplete. Grasp quality (ε) is the measure that separates them.

### Limitations

- The search is a local multi-start method: "not found" is not a proof of infeasibility (see the flip counts). Pinch results are reliable (no misses);
  tripod and four-contact misses on small objects are not.
- ε is the best found, not the global maximum; the force-closure surrogate steers the search toward balanced unit-normal-force grasps.
- Point contacts on flat pads at the pads' centre plane; no soft-contact patch, no torsional friction (pinch closure is up to rotation about the contact
  line, ε in the 5-D subspace), no palm contact, no actuator torques or stiffness, no dynamics.
- The collision model is the evaluator's coarse capsules and unrounded palm boxes; fingers are not checked against the palm; object vs palm uses sampled
  box surfaces (5 mm) and object edges.
- Pinch ε (5-D) and multi-contact ε (6-D) are on different scales; the medians mix them (the split is reported).
