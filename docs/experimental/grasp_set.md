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

## Results

(to be filled after the run)
