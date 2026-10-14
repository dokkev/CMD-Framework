# Stage-0: Linkage Optimization for the Ordinary Fingers

> **Legacy.** The 1-DoF coupled linkage is no longer the planned finger
> architecture. This study is kept for reference in `cmd/experimental`.

Stage-0 searches the 1-DoF coupled linkage shared by the index, middle, ring
and little fingers. It is a **linkage-only kinematic** study: one actuator
drives the MCP, two coupler loops drive the PIP and DIP, and Stage-0 looks for
morphologies that

1. move healthily (no singularity, no joint reversal) over a useful MCP range,
2. bring the distal pad to face the proximal phalanx with room for an object
   (a grasp pose), as early in the MCP flexion as possible, and
3. transmit force well on the way there,

and that stay safe for at least 20° of MCP flexion past that grasp pose.

Fingertip geometry, palm/housing interference, actuator torque and contact
forces are out of scope here (see [Next stages](#next-stages)).

Code: [`cmd/experimental/stage0/`](../../cmd/experimental/stage0/).
Linkage model and conventions: [morphology source of truth](../design/morphology_source_of_truth.md), §4.

---

## 1. Linkage and notation

Points are in the canonical XZ plane (+X distal, +Z dorsal) of the fully
extended reference pose, with the MCP joint O at the origin.

| Body | Points | Role |
| --- | --- | --- |
| ground | O, O' | O: MCP axis (actuated); O': coupler 1 ground pivot |
| proximal link | O, A, A' | rotates by q1 (MCP) |
| middle link | A, B, B' | rotates about A by q2 (PIP) |
| distal link (pad) | B, C, C' | rotates about B by q3 (DIP); BC is the pad |
| coupler 1 | O'–B' | closes loop 1, sets q2 |
| coupler 2 | A'–C' | closes loop 2, sets q3 |

Flexion angles q1, q2, q3 are positive in flexion and measured from the
reference pose. The pad rotation from +X is q1 + q2 + q3.

## 2. Design vector

Ten components of `FingerLinkageParameters`, searched on a **0.1 mm grid**:

| Variable | Bounds [mm] | Note |
| --- | --- | --- |
| `op_from_o_x_mm` | 14.5 – 30.0 | physical (actuator/palm packaging) |
| `op_from_o_z_mm` | 3.8 – 27.5 | physical |
| `a_from_o_x_mm` | 20 – 50 | |
| `a_from_o_z_mm` | −10 – 30 | A at most 30 mm dorsal |
| `ap_from_o_x_mm` | 15 – 45 | |
| `b_from_a_x_mm` | 10 – 40 | |
| `bp_from_a_x_mm` | −10 – 20 | |
| `bc_length_mm` | 20 – 35 | pad length |
| `cp_from_b_x_mm` | −25 – 5 | |
| `cp_from_b_z_mm` | −10 – 20 | |

Only the O' bounds (and the A height limit) are physical. The others are wide
enough not to bind; the design space is shaped by the constraints instead.

**Derived, not searched.** In the reference pose Stage-0 places A', B', B and C
on z = 0, so OA', B'B and BC all lie along +X:

```text
ap_from_o.z = 0,   b_from_a.z = bp_from_a.z = -a_from_o.z,   C = B + (bc_length_mm, 0)
```

The last relation is the morphology's definition of the fully extended pose.
Fixing the heights lost almost no transmission quality while simplifying the
link shapes and making the search easier.

## 3. Motion evaluation

* The MCP is swept from 0 to the **search horizon 1.5 rad (86°)** on
  **31 uniform samples** (0.05 rad). Both loops are solved in closed form on the
  reference assembly branch.
* A sample is **healthy** when both loops close, both transmission margins are
  at least **30°**, and the PIP and DIP motion ratios dq/dq1 lie in
  **[0.1, 3]** (monotonic flexion without runaway).
* The **motion limit** is the last sample of the contiguous healthy range. No
  MCP angle is prescribed; every design is evaluated up to its own limit.

The **transmission margin** of a loop is min(μ, 180° − μ), where μ is the angle
between the coupler and the output arm at the coupler joint (B' for loop 1, C'
for loop 2). It reaches 0 exactly at the loop's dead point, which is the only
kind of singularity this chain has: the actuator drives OA directly.
Transmission quality is sin(margin) ∈ [0, 1].

## 4. Grasp pose

A sample within the motion limit is a **grasp pose** when both pad ends B and C

* lie on the palmar side of the proximal segment OA', at least
  **15 mm** from its line (link thickness), and
* project onto OA' between O and A' (the pad is in front of the proximal
  phalanx, not slid back past the MCP).

The pad need not be the same length as OA' or aligned end to end with it.
Among grasp poses, Stage-0 uses the one whose pad best faces OA'.

## 5. Objectives

NSGA-II minimizes the following (maximized ones are negated):

| # | Objective | Sense |
| --- | --- | --- |
| 1 | Worst transmission quality of both loops from extension to the grasp pose | max |
| 2 | Facing error: angle between B→C and A'→O at the grasp pose | min |
| 3 | MCP angle of the grasp pose (the earlier the pad faces OA', the better) | min |

Designs without a grasp pose score infinity on objectives 2 and 3 (sent to the
solver as 1e12); they are also infeasible.

## 6. Constraints (g ≤ 0, fixed residual count)

| Group | Requirement |
| --- | --- |
| `usable_motion` | the healthy range spans at least one sample (optionally `minimum_motion_range_rad`) |
| `extended_length` | \|OC\| at q = 0 in **[80, 110] mm** |
| `minimum_pin_spacing` | bearing pins sharing a rigid link at least **9 mm** apart: A–A' (proximal), A–B' and B–B' (middle), B–C' (distal). MR63 (3×6×2.5) in aluminium: 6 mm OD + 3 mm web. Pins on different links sit on different Y layers and are not checked. |
| `grasp_pose` | some sample is a grasp pose (graded by the summed gap shortfall and projection overshoot of B and C) |
| `post_grasp_safety` | from the grasp pose to **grasp + 20°**, both loops close, both margins stay ≥ 30° and neither q2 nor q3 reverses (dq/dq1 > 0) |

The grasp pose plus the post-grasp travel is the **safe operating limit**.
A mechanical stopper is a last-resort guard placed past this limit and before
the first near-singular pose; it does not replace the safe range.

## 7. Solver

* pymoo NSGA-II over integer grid steps (SBX η = 15, p = 0.9; PM η = 20; both
  rounded), population **200**, **200** generations, LHS initial population
  plus the `BASELINE_LINKAGE_V0` seed.
* One run takes about 1.5 minutes.

## 8. Selection workflow

Pareto score alone does not pick the design. Candidates go through:

1. **Screening** on `pareto.csv`: grasp MCP ≤ 60°, facing error ≤ 10°
   (post-grasp safety ≥ 20° and margin ≥ 30° are already enforced).
2. **Dense verification**
   ([`verify.py`](../../cmd/experimental/stage0/verify.py)): the sweep is
   repeated on a 0.05° grid. The safe range must keep margins ≥ 30°, close, and
   let q2/q3 fall back by at most 0.1° (a DIP that stalls at its peak backs off
   by about 0.01°). Past the safe limit the first near-singular pose (margin
   shortfall or no closure) is reported as the upper bound for the stopper.
   This step rejected 6 of 29 screened candidates in one run, so the coarse
   grid alone is not enough.
3. **Comparison** of the verified shortlist: transmission quality versus facing
   error, grasp MCP, grasp gap and stopper window. 40° margin is the stricter
   comparison level.

Robustness: three runs with seeds 0, 1 and 2 gave best verified qualities of
0.705, 0.703 and 0.702, and their top designs fell in the same region of the
design space.

## 9. Selected design: `SELECTED_LINKAGE_V1`

Run with seed 1, Pareto candidate #122 (code at commit `b7a495e`): the best
transmission quality among verified candidates with a facing error ≤ 5°.

Design vector (0.1 mm grid):

| Variable | Value [mm] |
| --- | --- |
| `op_from_o_x_mm` | 27.0 |
| `op_from_o_z_mm` | 5.3 |
| `a_from_o_x_mm` | 38.3 |
| `a_from_o_z_mm` | 19.4 |
| `ap_from_o_x_mm` | 32.7 |
| `b_from_a_x_mm` | 40.0 |
| `bp_from_a_x_mm` | 7.7 |
| `bc_length_mm` | 29.2 |
| `cp_from_b_x_mm` | −13.4 |
| `cp_from_b_z_mm` | 7.8 |

Equivalent `FingerLinkageParameters` (the derived z components filled in):

```python
FingerLinkageParameters(
    op_from_o=PointXZ(27.0, 5.3),
    a_from_o=PointXZ(38.3, 19.4),
    ap_from_o=PointXZ(32.7, 0.0),
    b_from_a=PointXZ(40.0, -19.4),
    bp_from_a=PointXZ(7.7, -19.4),
    bc_length_mm=29.2,
    cp_from_b=PointXZ(-13.4, 7.8),
)
```

| Reference point | Position [mm] |
| --- | --- |
| O' | (27.0, 5.3) |
| A, A' | (38.3, 19.4), (32.7, 0) |
| B', B | (46.0, 0), (78.3, 0) |
| C', C | (64.9, 7.8), (107.5, 0) |

| Derived length | Value [mm] |
| --- | --- |
| \|OA\|, \|AB\|, \|BC\| | 42.9, 44.5, 29.2 |
| Coupler 1 \|O'B'\|, coupler 2 \|A'C'\| | 19.7, 33.1 |
| Extended length \|OC\| | 107.5 |

| Property | Value |
| --- | --- |
| Worst transmission quality | 0.692 (min margin 43° over the safe range) |
| Grasp pose | MCP 52°, PIP 75°, DIP 101°, facing error 3.9°, gap 24.3 mm |
| Safe operating range | MCP 0–72° |
| First near-singular pose | MCP 79.1° → stopper between 72° and 79° |
| Pin spacings | A–A' 20.1, A–B' 20.8, B–B' 32.3, B–C' 15.5 mm |

`BASELINE_LINKAGE_V0` (the optimizer seed) is kept as the comparison reference.
Both live in [`seed.py`](../../cmd/experimental/linkage/seed.py).

## 10. Design decisions

| Decision | Reason |
| --- | --- |
| No prescribed MCP range; designs run to their own motion limit | A fixed range forced late PIP/DIP stalls; the grasp angle is an outcome. |
| Grasp facing OA' instead of a target pad angle or a parallelogram | Orientation alone let the pad slide past the MCP or collapse onto the proximal phalanx; the pad must face OA' from in front, with a gap. |
| Facing error minimized, no tolerance | The trade-off against quality and grasp angle is read off the Pareto front. |
| Grasp MCP angle as an objective | The pad should face OA' as early as possible. |
| Post-grasp safety as a hard constraint, not an objective | A fourth objective spread the front too thin. Requiring 20° instead of 10° cost about 5 % quality but doubles the distance to the dead point. |
| Margin 30° hard, 40° for comparison | 40–50° is a load- and friction-dependent rule of thumb; the final figure follows actuator torque, friction and link compliance. |
| No PIP/DIP ratio constraint | The goal is useful grasp coordination with few actuators, not a copy of human joint coupling. PIP/DIP angles, motion ratios and the S-shape of the curves stay as diagnostics. |
| Point-C excursion is a diagnostic only | It rewards link size and does not imply grasp quality. |
| A', B', B, C fixed on z = 0 | Little quality loss and simpler link shapes. |

## 11. Running

```bash
# Optimize (edit CONFIG in the script first); writes outputs/stage0/run_NNN/
python scripts/optimize/run_stage0.py

# Screen and verify one or more runs into outputs/stage0/shortlist.csv
python scripts/optimize/select_stage0.py outputs/stage0/run_004 outputs/stage0/run_005

# Animate a Pareto candidate next to its pareto.csv
python scripts/animate_linkage.py --pareto outputs/stage0/run_005/pareto.csv --candidate 122 --flexion-rad 1.257
```

`run.json` records the configuration, pymoo version and source commit;
`pareto.csv` records each feasible Pareto design with its objectives, grasp
pose (MCP/PIP/DIP, gap, pad position along OA'), post-grasp travel, safe MCP
limit, pin spacings and workspace diagnostics.

## Next stages

1. **Finger-level geometric evaluation (Stage-1).** Attach the fingertip
   morphology to the distal link, define a simple palm/housing envelope, check
   swept collisions over the safe range, and measure grasp geometry (object
   size held between pad and proximal phalanx, contact normals). Contact force
   from the MCP actuator torque can be added as a diagnostic.
   Needs: fingertip attachment on BC, palm/housing envelope, target objects.
2. **Final design selection** across the verified Pareto candidates with the
   Stage-1 results, then per-digit checks with each finger's fingertip length.
