# Thumb and hand-layout optimisation (robotic Kapandji)

Status: experimental. Code: `design/core/optimization/thumb_layout/` (evaluator), `design/scripts/optimize/thumb_layout/run.py` (NSGA-II runner). Run data: `design/output/thumb_layout/<hand>/<envelope>/<orientation>/seed<N>/` (not committed). Runs under superseded rules are in `design/output/thumb_layout/_superseded_*`.

## Why

Motivation: in the Drake pinch experiments a parallel thumb–index pinch was impossible on the current hand. At 6–9 mm gaps the best opposition was 29–31 deg from facing, with `thumb_ro` (-90 deg) and `thumb_ip` at their joint limits.

Goal (user, 2026-11-25): general dexterity, not pinch. The measure is the Kapandji test as RLWRLD's All Hands Up (allhandsup.org) runs it on robot hands: "Measured in simulation by sweeping each hand's thumb against the 11 Kapandji landmarks on its own URDF model. A level counts as reached when the thumb tip makes contact." This study asks how far the thumb's base position, the IP servo's orientation and the fingertip length move that score for a given finger morphology M. M and the three fingers' bases stay fixed.

## Formulation (current: All Hands Up Kapandji; the decision log has the earlier rules)

### Model and frames

Everything is in the parametric model (`core/morphology/code_hand`), in hand frame H [mm].
- The thumb chain is `CoDeHand.thumb_frames`. The finger chains are `finger_frames`, with the PIP four-bar from `four_bar`.
- Joint limits are the exporter's `LIMITS` (`design/scripts/export/code_hand_sim.py`), which are the limits written into the URDF/USD:
  - thumb RO ±90, RAA -30..90, MCP and IP -10..100;
  - finger AA ±30, FE -10..100;
  - PIP 0..120 (the required flexion). The four-bar table itself runs -10..120.
- Pads in model frames:
  - The finger pads are `PadBox`es. The contact face is at `mount + thickness` palmar of the link line, with outward normal +Y of the box (palmar).
  - The URDF's index distal pad point (-6.5, 0, -6) mm in `index_distal` is the distal pad's face centre for O: (-23.8 + 10.8)/2 = -6.5, and 3 + 3 = 6 mm palmar.
  - The thumb pad is the sim-side `THUMB_PAD_MM` (3 mm thick, mounted 10 mm off the centre line, the case's last 25 mm). In the IP servo frame turned by IP, its face centre is (13, -(ee - L/2), 0) with normal +X. This is the URDF's (0.013, 0.041, 0) in `thumb_distal`.
- Cross-check: the design model and the Drake URDF FK (`load_hand.py`, O) agree to 1e-3 mm and 1e-4 on the normals, for the thumb pad and the index distal pad, at two poses (thumb (-60, 20, 40, 30) / index (10, 50, 60), and thumb (-90, 0, 48.6, -10) / index (0.3, 80.7, 0)).

### Design variables

1. **CMC (RO) servo position (x, z)** in H: the thumb base `Pose` translation. y = 0.77 and the orientation (yaw -90, roll -90; RO axis along +X) stay the prototype's. Step 0.5 mm.
2. **IP servo orientation**, discrete; each variant is its own run. The XM335 packaging profile is r = 9.5 mm cap radius, 25.5 mm rear extent and 22 mm depth. The prototype's MCP-body-to-IP-cap gap is 0.5 mm.
   - (a) `distal` (the prototype): the IP body lies in the distal segment. `mcp_to_ip` = 25.5 + 9.5 + 0.5 = 35.5 mm. The case starts at the IP body's rear face, 25.5 mm from the IP axis.
   - (b) `proximal` (turned 180 deg about the IP axis, which keeps the flexion sense): the IP body lies back toward the MCP, rear to rear with the MCP body. `mcp_to_ip` = 2 × 25.5 + 0.5 = 51.5 mm. The distal segment is the horn bracket round the cap and then the case, which starts 9.5 + 0.5 = 10 mm from the IP axis.
3. **Distal extension e** [15, 45] mm: the case past the servo (a) or past the cap bracket (b). The end effector is at `case_start + e`. The prototype has e = 53.5 - 25.5 = 28.0. The pad covers the case's last min(25, e) mm.

4. **Exploratory, beyond the user's original variable list: base orientation.** Offsets of ±20 deg on the CMC servo pose's yaw, pitch and roll (intrinsic Z-Y-X; step 1 deg), in the strict envelope. With the orientation free, the RO servo body itself must also clear the abduction servos and the fingers at rest by 2 mm.

**Palm envelope (bounds).** The fingers' abduction servos fix the palm.
- **Forward limit.** The RAA servo turns with RO. Over the full RO range it must clear the abduction servos by 2 mm. While the RAA body is dorsal of the abduction servos' bottom (z = 0), this caps x at -49.0. The prototype is at -51.2.
- **Dorsal limit.** The RO servo must stay below the hood's top (z_top = 24 mm), so z ≤ -3.5.
- **Palmar limit.** The RO servo stays at most 20 mm below the palm's flat palmar face (-6.1), so z ≥ -16.5. The prototype servo is 9.4 mm below that face.
- **Rear limit.** At most 20 mm behind the prototype: x ≥ -71.
- `strict`: x ∈ [-71, -49], z ∈ [-16.5, -3.5]. This is the envelope asked for.
- `relaxed` (an extension, to show what the envelope costs): x ∈ [-80, -25], z ∈ [-46, -3.5], floor 50 mm below the palm face. The RAA-sweep clearance is a constraint here; it lets the base go forward only once the RAA body passes under the abduction servos (z ≲ -28.5).
- The palm housing is rebuilt per design with the exporter's `palm_housing`.

### Kapandji levels on a 3-finger, 2-phalanx hand

The clinical levels are:
- K0: radial side of the index proximal phalanx; K1: radial side of the index middle phalanx; K2: radial side of the index distal phalanx;
- K3: index tip; K4: middle tip; K5: ring tip; K6: little tip;
- K7: little DIP crease; K8: little PIP crease; K9: little MCP (proximal digital) crease;
- K10: the distal palmar crease.

Our hand has three fingers (index, middle, ring) with two phalanges each: proximal and distal, no middle phalanx and no DIP. The mapping:

| level | clinical | here | where (finger frames: P proximal, D distal; pad spans as in `PadParameters`) |
|---|---|---|---|
| K0 | index proximal phalanx, radial | the same | P, 25 % along the proximal pad span, on the finger capsule's radial (thumb-side, +FE-axis) surface |
| K1 | index middle phalanx, radial | index proximal phalanx, radial, near the PIP | P, 85 % along the proximal span, radial surface |
| K2 | index distal phalanx, radial | the same | D, the distal pad span's middle, radial surface |
| K3 | index tip | the same | D, the distal pad face, 5 mm short of its distal end |
| K4 | middle tip | the same | as K3 on the middle |
| K5 | ring tip | ring tip (stands for both ring and little tips) | as K3 on the ring |
| K6 | little tip | ring distal pad | D, the ring distal pad face centre |
| K7 | little DIP crease | ring PIP crease | P, the ring proximal pad face, 5 mm short of its distal end |
| K8 | little PIP crease | ring proximal phalanx base (MCP crease) | P, 25 % along the ring proximal pad face |
| K9 | little MCP crease | palm at the ring MCP base | palm block's palmar face, 6 mm behind its front edge, at the ring's y |
| K10 | distal palmar crease | palm under the index–middle | palm block's palmar face, 12 mm behind its front edge, at the mean y of index and middle |

Reasons for the choices:
- **K1.** There is no middle phalanx, so K1 is the distal part of the proximal phalanx, next to the PIP. This was the coordinator's suggestion.
- **K5–K8.** The ring finger is the last finger, so it stands in for the little finger. Its levels run from the tip toward the palm (tip, distal pad, PIP crease, proximal-phalanx base) as the little finger's K6–K9 do. Each level is shifted down by one because K5 and K6 collapse onto the ring.
- **K9 and K10, palm position.** The palm landmarks sit on the flat palmar face. That face is fixed by the abduction servos: `palm_housing`'s block front, -6.1 mm, independent of the thumb. A palm landmark is counted only if a palm box surface lies within 1 mm of it, which matters for the relaxed envelope, where the block can vanish.
- **K10.** The suggestion was "the palm under the index–middle MCPs". I put it 12 mm behind the block's front edge instead of 6 mm. The distal palmar crease lies proximal of the MCP creases, and this keeps K10 deeper in the palm than K9.
- **Radial sides.** These are on the finger's capsule (r 9.5 mm, the pad half-width) at its widest point. The capsule is the same one the collision model uses, so contact and collision are consistent.

**Contact (All Hands Up).** A level is reached when the **thumb tip** makes contact:
- some point of the thumb tip is within 1 mm of the landmark;
- the configuration is within the joint limits: thumb, plus the landmark finger's AA, FE and PIP (the fingers move, as in the clinical test); the other fingers stay extended at zero;
- nothing collides;
- there is **no facing condition**.

The thumb tip is the fingertip case's rounded end. In the IP servo frame's X–Y plane it is a quarter circle of radius 13 mm (the pad face's offset), from the pad face (13 mm off the centre line, 13 mm short of the end effector) round to the end effector on the centre line. The contact point's place φ ∈ [0, 90] deg on it is an IK variable. The normal is used only to steer the seeds and the IK's null space.

**Score.** The clinical score is the highest level k such that K0…k are all reached (-1 if K0 is not). The count of levels reached is reported too. The optimised value is `k + 0.999 / (1 + r_{k+1})`: the tie-break is the margin to the next level, with r = (pos - 1)+/5 + (penetration - 1)+/5 at its best candidate.

### Objectives (NSGA-II, both maximised)

1. **Functional Kapandji score**, primary: the same levels and chain score, with each level functionally reached (pad contact, facing ≤ 45 deg, 2 N within the stall torques, thumb joints ≥ 5 deg from their limits, collision-free; decision log). The touch score above (All Hands Up) is reported alongside.
2. **Pad lever ratio:** min(IP, MCP) of (the prototype's lever / the design's lever), i.e. pad force per joint torque relative to the prototype. Chosen over the thumb's workspace volume as the second measure of general dexterity:
   - the Kapandji levels already measure the opposable workspace;
   - a larger workspace mostly rewards reach away from the fingers;
   - the lever is the quantity that trades against reach (a longer thumb reaches further and pushes less), so the front shows what each extra level costs in force.
   - It stays bounded below by the 0.7 constraint.

**Reported, not optimised** (computed for the final front's designs):
- **Touch Kapandji** (All Hands Up) level and count.
- **Pinch:** the thumb–index / thumb–middle best facing angle, as before (pad centre on the finger pad's normal line within 2 mm, gap 6–20 mm, the worse of the two).
- **Minimum graspable diameter** (All Hands Up's metric, geometric approximation). This is the smallest cylinder held between the thumb pad and the index or middle distal pad with force closure:
  - the pads are D apart along the finger pad's normal through its centre (within 2 mm);
  - the configuration is collision-free;
  - the pads are within 2·atan(μ) of parallel, with μ = 0.5 (53 deg). For two flat contacts on a cylinder, the chord between them must lie inside both friction cones.
  - D is searched over 2, 3, 4, 6, 8, … mm.
- **Independent DIP actuation:** absent by design. The fingers have two phalanges; the PIP four-bar drives the distal segment and there is no DIP joint.

**Constraints.**
- Joint limits (in the IK).
- Collision-free at the solution poses (inside the target test).
- Envelope margins ≥ 0: RAA sweep, RO under the hood top, RO above the floor (and, with the base orientation free, the RO servo body clear of the abduction servos and the fingers at rest by 2 mm).
- Pad lever: the IP and MCP pad-force-per-torque at least 0.7 × the prototype's. The lever is the pad centre's distance along the segment from the IP axis (prototype 41.0 mm) and from the MCP axis (76.5 mm).

### Collision model (capsules, boxes; a signed distance below -1 mm is a collision)

- **Thumb:**
  - RAA servo body (capsule, r 9.5);
  - metacarpal (RAA to MCP centres, r 6);
  - proximal segment (MCP to IP centres, r 9.5; it holds the MCP servo and, in (b), the IP servo);
  - distal segment (IP axis to 9.5 mm short of the end effector, r 9.5);
  - pad (a capsule of r 1.5 along its face's centre line).
- **Fingers:**
  - MCP-FE and PIP servo bodies (capsules, r 9.5);
  - proximal and distal segments: capsules of the pad width (r 9.5) whose palmar side is the pad face, over the pad's length.
- **Palm:** the housing's hood, block, bridge and heel as plain boxes.
- **Pairs:**
  - thumb (pad included) against the other fingers;
  - thumb (pad excluded) against the landmark's finger;
  - thumb proximal/distal/pad against every palm box (the pad not for a palm landmark);
  - thumb RAA servo/metacarpal against hood and block only (the heel and bridge are built round the base);
  - the moving finger's segments against the other fingers.

### Solver

- **Seeds.**
  - Thumb grid at 10 deg and finger grid at 10 deg, each filtered to be collision-free.
  - The thumb tip is sampled at φ = 0, 45 and 90 deg.
  - Seeds are matched on [position, w × normal] with a KD-tree (w = 10 mm per unit for Kapandji contacts, 30 for the pinch). The best 16 are kept.
- **Refinement.** Each seed gets 40 steps of task-priority damped least squares:
  - position first, the normal in its null space;
  - finite-difference Jacobians;
  - an active set at the joint limits;
  - a step is accepted only if it keeps a clear pose clear.
- **Cost.** About 4 s per design, including the two pinch tasks.
- **Grid step.** Because the score is a chain, one marginal level decides it, so the grid step matters. With a 15 deg thumb grid the prototype's K4 stalled at 1.1 mm (level 3); at 10 deg it is reached at 0.5 mm (level 9). That holds both for 16 seeds × 40 steps and for 32 × 80. Five test designs gave the same levels at 10 deg/16/40 as at 10 deg/32/80, so 10 deg/16/40 is used.
- **Brute-force check (25 deg rule).** On the prototype it agreed with the solver. Thumb on a 5 deg grid, index on a 2.5 deg grid, pad within 1.5 mm: the best angles were 30.3 deg (index tip) and 33.6 deg (index distal); the solver found 30.5 and 34.0.

## Decision log

- **2026-11-25, Kapandji rule changed from "touch + facing within 25 deg" to "touch" (user decision).**
  - Why: under the 25 deg rule the prototype reached 0/10 targets, and every failure was the facing angle, not reach. The pad centres were within 1 mm, and the best angles were 30.5 deg at the index tip, 34 at the index distal and 28 at the palm. The score therefore duplicated the pinch objective (opposition quality) instead of measuring reach.
  - Now a target counts if the pad centre is within 1 mm of it, collision-free and within the limits, with only a loose pad-side condition: the normal within 60 deg of facing, which excludes side or back contact. The pinch objective is unchanged.
  - When: after the prototype baseline and before any optimisation result was used. The first NSGA-II batch under the 25 deg rule (O, strict, 3 distal seeds and 1 proximal seed) was stopped, set aside in `design/output/thumb_layout/_superseded_facing25/`, and not used.
  - With the rule, the touch seeds are matched with a weaker normal weight (10 mm per unit instead of 30). On four test designs this weight reached as many targets or more than 30 or 3; the pinch keeps 30.

- **2026-11-25, bug fix (found while switching to the All Hands Up rule).** The clearance routine cached the pair lists together with the palm boxes' geometry. Each worker process therefore kept the palm boxes of the first design it evaluated. The prototype baselines (single designs, fresh process) are unaffected. The optimisation runs under the touch rule (in `_superseded_touch60/`) used stale palm boxes and are not trustworthy for that reason either. The box geometry is now taken per call.
- **2026-11-25, reformulated to All Hands Up's Kapandji method (user decision; goal = general dexterity, not pinch).**
  - Why: the goal is general dexterity, and All Hands Up gives an external, comparable Kapandji protocol for robot hands.
  - What changed:
    - **Levels.** 11 clinical levels K0–K10 mapped onto our hand (table above), replacing the 10 pad/palm targets.
    - **Contact.** Thumb tip contact (the rounded tip, not only the pad centre) within 1 mm, collision-free, within limits, fingers free. No facing condition.
    - **Score.** The highest level with all lower levels reached, plus the count.
    - **Objectives.** The pinch is dropped from the objectives but still reported. The second objective is now the pad lever ratio.
    - **Metrics.** All Hands Up's minimum graspable diameter is added as a report.
  - When: after the touch-rule runs. Those were O strict (both orientations, 3 seeds each) and two relaxed distal seeds; the rest of that batch was stopped. Their outputs are in `design/output/thumb_layout/_superseded_touch60/` and are not used for the recommendation.
  - What they showed (qualitatively, under the touch rule and the stale-palm bug):
    - in the strict envelope, the flipped IP reached 8/10 and the current IP 7/10, against 5/10 for the prototype; pinch 17–21 deg;
    - the relaxed envelope (base about 24 mm deeper in the palm, x ≈ -26, z ≈ -30) brought the pinch to about 10 deg.

- **2026-11-25, functional Kapandji becomes the optimised score; the touch Kapandji is kept as a report (user decision).**
  - Why: the user distinguishes touching a landmark from being able to use the contact. The All Hands Up touch score stays as a reported metric for comparability.
  - When: after the touch-Kapandji prototype baseline and before any optimisation result. The touch-score runs had just started, and no run had finished. They were stopped and set aside in `design/output/thumb_layout/_superseded_touch_allhandsup/`.
  - A level is **functionally reached** if a configuration exists (thumb and the landmark's finger free; the other fingers extended) where all of these hold:
    1. **Pad contact.** The thumb pad face is within 1 mm of the landmark. The contact is on the face's centre line, 2 mm in from the pad's ends (the face, not its edge), and the place is an IK variable.
    2. **Facing.** The pad normal is within 45 deg of facing the landmark's surface normal.
    3. **Force.** At least 2 N along the contact normal, within every servo's stall torque. The torque is the XM335's at 11.1 V, 1.03 N m (`core.hardware.xm335.XM335DriveModel`). τ = Jᵀf is checked for the thumb's four direct-drive joints **and** for the landmark's finger: AA and FE are direct drive; the PIP servo carries the joint torque divided by the four-bar's dθ/dq. J is the contact point's finite-difference Jacobian.
    4. **Margin.** Every thumb joint is at least 5 deg inside its limits.
    5. Collision-free.
  - Score: the highest level with all lower levels functionally reached, plus the margin tie-break (r adds (angle - 45)+/45 and (2 N - force)+/2 to the reach and penetration terms). The count is reported too.
  - NSGA-II objectives: the functional score and the lever ratio. The touch Kapandji, the pinch angle and the minimum graspable diameter are computed for the final front's designs.
  - Threshold check on the prototype (below), and why the thresholds were kept:
    - The score is not degenerate: level 3, 4/11.
    - The facing (45 deg) and the 5 deg margin both bind. RO sits at its -85 margin and IP at -5 in most solutions.
    - The **2 N force condition never binds**: the force capacity at the best solutions is 12–33 N. The thumb's lever is at most about 0.09 m, and 1.03 N m over that is about 11 N. It is kept as specified, and is redundant on this hand.

## Prototype baseline (M = O)

Prototype thumb: (x, z) = (-51.2, -6.0), IP body distal, e = 28.0 (EE 53.5 mm), levers IP 41.0 / MCP 76.5 mm.

### Functional Kapandji (optimised): level 3, 4/11, score 3.93

| level | reached | pad error | facing | force capacity | why not |
|---|---|---|---|---|---|
| K0 index proximal, radial, near MCP | yes | 0.0 mm | 0 | 13.5 N | |
| K1 index proximal, radial, near PIP | yes | 0.05 | 1 | 13.3 | |
| K2 index distal, radial | yes | 0.03 | 1 | 11.9 | |
| K3 index tip | yes | 0.55 | 31 | 12.1 | |
| K4 middle tip | no | 1.4 | 41 | 12.1 | pad 1.4 mm short (thumb RO -85, IP -5 at their margins) |
| K5 ring tip | no | 0.97 | 56 | 12.1 | facing 56 deg |
| K6 ring distal pad | no | 1.4 | 57 | 14.2 | pad 1.4 mm short, facing 57 |
| K7 ring PIP crease | no | 2.0 | 47 | 19.4 | pad 2.0 mm short, facing 47 |
| K8 ring proximal base | no | 4.1 | 37 | 18.3 | pad 4.1 mm short |
| K9 palm at ring base | no | 1.1 | 31 | 15.9 | pad 1.1 mm short |
| K10 palm under index–middle | no | 6.0 | 59 | 33.3 | pad 6.0 mm short, facing 59 |

### Touch Kapandji (All Hands Up, reported)
- **Kapandji level 9** (K0–K9 all reached), **10/11 levels**, score 9.47.
- **K10 fails.** The distal palmar crease under the index–middle is missed by 6.7 mm. The thumb IP is at its 100 deg limit (thumb (-38.9, 36.9, 80.7, 100.0)) and the thumb's distal segment is at -0.95 mm clearance to the palm. The thumb cannot fold its tip back onto the palm next to its own base.
- **Marginal levels.** K4 (middle tip) at 0.52 mm, K7 at 0.67 mm and K8 at 0.64 mm. K4 depends on the solver's grid: 1.1 mm (unreached) with a 15 deg grid.
- **Pinch (reported).** 29.4 deg. Index: 18.0 deg at a 20 mm gap. Middle: 24.9 deg, 2.5 mm off the line.
- **Minimum graspable diameter.** 2 mm (index, pads 31 deg from parallel; middle 3 mm at 38 deg). The pads can close on a thin rod within the μ = 0.5 cone.
- **Independent DIP actuation.** None (by design).

### Earlier baselines (superseded rules)

Touch rule (pad centre within 1 mm, facing ≤ 60 deg, 10 pad/palm targets):

Prototype thumb: (x, z) = (-51.2, -6.0), IP body distal, e = 28.0 (EE 53.5 mm), levers IP 41.0 / MCP 76.5 mm.
- **Kapandji 5/10, score 5.78.** Reached: index tip, index distal, index proximal, middle tip, middle distal.
- **Pinch objective 29.1 deg.** Index: 18.0 deg from facing at a 20 mm gap. Middle: 24.5 deg, but only 2.5 mm off the line (penalised to 29.1).
- Unreached targets:

| target | best pad error | angle | why |
|---|---|---|---|
| middle/proximal | 1.8 mm | 26 | thumb at its limits (RO -90, IP -10, MCP 92) |
| ring/tip | 2.8 mm | 59 | workspace: ring at AA -30 and FE 100, thumb RO -90 and IP -10, all at limits |
| ring/distal | 3.4 mm | 56 | the same |
| ring/proximal | 3.4 mm | 50 | thumb distal segment hits the ring's proximal segment (-1.1 mm) |
| palm | 1.1 mm | 28 | MCP at its 100 deg limit |

- In almost every solution `thumb_ro` sits at -90 and `thumb_ip` at -10, as in the Drake pinch finding.
- Under the superseded 25 deg rule the same prototype scored 0/10.

## Results (M = O)

All runs use NSGA-II on (functional score, lever ratio), population 24 × 20 generations (32 × 20 with the orientation free), 3 workers. Each row is the run's best functional score; ties are broken by lever ratio.

| run | x, z [mm] | e [mm] | base yaw/pitch/roll [deg] | functional | touch | lever ratio (IP, MCP) | pinch [deg] | min grasp |
|---|---|---|---|---|---|---|---|---|
| prototype | -51.2, -6.0 | 28.0 (distal IP) | 0/0/0 | K3 (4/11) | K9 (10/11) | 1.00 (1.00, 1.00) | 29.4 | 2 mm |
| strict, distal IP, seed 0 / 1 | -50, -5 / -4.5 | 43.5 | 0/0/0 | K4 (6/11) | K9 (10) | 0.73 (0.73, 0.83) | 19.4 / 19.7 | 2 mm |
| strict, proximal IP | -51, -4.5 | 42 | 0/0/0 | K4 (6/11) | K9 (10) | 0.84 (1.04, 0.84) | 18.1 | 2 mm |
| relaxed, distal IP | -44.5, -31.5 | 42.5 | 0/0/0 | K4 (7/11) | K9 (10) | 0.74 | 22.8 | 2 mm |
| relaxed, proximal IP | -34, -33 | 31 | 0/0/0 | K4 (6/11) | K6 (7) | 0.96 | 19.1 | 2 mm |
| **strict + base orientation ±20 (exploratory), proximal IP** | **-50.5, -10.5** | **34** | **-14/-16/+14** | **K8 (9/11)** | **K9 (10)** | **0.92 (1.30, 0.92)** | **8.8** | **2 mm** |

- In the position-only runs (strict and relaxed, both IP orientations) the levels reached are K0–K4, plus K8 (strict) or K6 (relaxed), out of order.
- The orientation optimum reaches K0–K8 in order.
- The functional score's chain stops at **K5 (ring tip) on facing** in every position-only run: the pad gets there, but more than 45 deg from facing.
- The distal-IP orientation run is queued after R and U (its first attempt crashed on a warm-start bug, since fixed).

### What unlocks K5: the RO axis direction

The probe takes the strict optimum (proximal IP, x -51, z -4.5, e 42) and changes one thing at a time, solving the ring-tip level alone:

| change | K5 | facing | thumb RO |
|---|---|---|---|
| none | no | 47.5 deg | -85 (its 5 deg margin) |
| RO range to -120 (a joint-limit change) | yes | 32.7 | -115 |
| base pitch -20 | yes | 29.5 | -85 |
| base pitch +20 / roll ±20 / yaw ±20 (each alone) | no | 48–62 | -85 |
| base moved forward and palmar (x -26, z -30, relaxed) | yes | 33.1 | -85 |

- The thumb always runs out of RO at the margin. Turning the RO axis (pitch), or giving RO more range, lets the pad roll onto the ring tip.
- Moving the base forward and palmar also reaches K5, but it loses K0 (index radial side near the MCP: pad 7 mm off), so the chain does not gain. This is why the relaxed runs stay at K4.
- The orientation optimum combines pitch -16 with yaw -14 and roll +14, keeps K0–K4, and adds K5–K8.

### Why a tilt, not a move or a longer thumb (probes, 2026-11-26)

O's fingers, proximal IP, functional Kapandji. Each probe changes one thing from the O_thumbopt design (x -50.5, z -10.5, e 34, yaw/pitch/roll -14/-16/+14).

**Which angle does the work.**

| base angles | F-level (reached) | K5 facing |
|---|---|---|
| -14/-16/+14 (O_thumbopt) | K8 (9/11) | 37 deg |
| pitch -16 only | K6 (8/11) | 41 |
| yaw -14 + roll +14, no pitch | K4 (6/11) | 51 |
| none | K4 (6/11) | 51 |

Pitch carries most of the gain; yaw and roll add K7–K8 only together with it. At K5 the pad is about 1 mm from the landmark in every variant: the facing angle, not the reach, decides.

**Without the pitch (pitch fixed at 0).** Same position (x -50.5, z -10.5), with yaw and roll each over -20…+20 deg in 5 deg steps and e ∈ {30, 34, 40, 44}:
- The best design is F-K4 (6/11) at every e: yaw -10, roll +5 and e 30 gives lever 0.97; e 34 gives 0.92.
- The tilt's pitch is worth four functional levels (K5–K8) at the same lever, and the pinch goes from about 18–19 deg (position-only optima) to 8.8.
- Pitch sweep at yaw -14, roll +14, e 34:

| pitch [deg] | functional (reached) | K5 facing |
|---|---|---|
| -4 | K4 (6/11) | 47 deg |
| -8 | K4 (6/11) | 44 |
| -12 | K4 (7/11) | 40 |
| -16 | K8 (9/11) | 37 |

  The chain opens between -12 and -16 deg.

**Moving the base instead (no rotation, envelope constraints ignored).** A 7 × 6 grid over x -60…-25, z +5…-32:
- The facing at K5 drops below 45 deg only at x ≥ -35 (to 36–44 deg).
- There the RAA servo's sweep overlaps the abduction servos by 5–12 mm. The RO servo body itself stays 17–36 mm clear, so the RAA sweep is the collision that blocks forward placement (the strict front, x -49, is where its clearance reaches 0).
- Every one of those forward positions loses K0: the base sits under the index MCP, and the pad cannot fold back onto the index's proximal radial side. Up to 8/11 levels are reached, but the chain is broken at K0.

**A longer thumb instead (no rotation).**

| e [mm] | F-level (reached) | K5 facing | K0 pad error | lever ratio |
|---|---|---|---|---|
| 34 | K4 (6/11) | 51 | 0.0 | 0.92 |
| 42 | K4 (6/11) | 48 | 0.9 | 0.84 |
| 50 | chain broken at K0 (8/11) | 44 | 3.7 | 0.77 |
| 60 | chain broken at K0 (8/11) | 35 | 2.4 | 0.70 |

A longer case reaches the ring side but loses K0 and pad force, the same trade as moving the base forward. The tilt alone keeps K0, the lever (0.92) and the RAA clearance (+1.0 mm) while opening K5–K8.

**An arched finger layout instead (ORCA-like; `palm_arch`).** The middle and ring bases turn palmar about H's x axis through the index base, by φ and 2φ:

| φ per finger | prototype thumb | O_thumbopt thumb |
|---|---|---|
| 0 | K3 (4/11) | K8 (9/11) |
| 5 | K4 (7/11) | K3 (6/11) |
| 10 | K6 (8/11), K9 reached | K2 (3/11) |
| 15 | K2 (3/11) | K2 (3/11) |
| -10 | K2 (3/11) | K4 (6/11) |

Arching the fingers toward the thumb does what tilting the thumb toward the fingers does. The two double up when combined unless they are optimised together. Caveats: the palm housing is the exporter's axis-aligned `palm_housing`, so its flat palmar face drops with the ring's abduction servo; the arch changes the finger geometry every finger study shares.

### Pre-registration: arch + thumb co-optimisation (2026-11-26, before any run)

- **Runs** (O, strict envelope, proximal IP, seed 0, population 32 × 25 generations):
  - A: position, e, base orientation ±20 deg and arch φ ∈ [0, 15] deg (step 1);
  - B: position, e and arch φ ∈ [0, 15] deg, no base orientation.
- Constraints as before, plus the RO servo body's clearance whenever the arch is free.
- **Hypotheses:** B reaches at least F-K6, the prototype-thumb arch probe's level. A reaches at least F-K8, the tilt-only optimum's level.
- **Decision rule:** The arch is worth carrying forward only if A beats the tilt-only optimum, i.e. a higher F-level (K9: the palm), or the same F-K8 with a lever ratio ≥ 0.05 higher. Otherwise the tilt-only O_thumbopt stays the recommendation, because it leaves the shared finger geometry unchanged.

### Result: arch + thumb co-optimisation (2026-11-26)

Run data: `design/output/thumb_layout/h0/strict_arch_orient20/proximal/seed0/` (A) and `strict_arch/proximal/seed0/` (B). Each run took about 790 evaluations in 19 min.

| design | arch φ | base yaw/pitch/roll | x, z, e [mm] | functional | touch | lever ratio (IP, MCP) | pinch |
|---|---|---|---|---|---|---|---|
| O_thumbopt (tilt only, earlier run) | 0 | -14/-16/+14 | -50.5, -10.5, 34 | K8 (9/11) | K9 | **0.92** (1.30, 0.92) | **8.8 deg** |
| A best | 13 | +1/+8/-9 | -55, -10, 44.5 | K8 (9/11) | K9 | 0.82 (0.98, 0.82) | 20.8 |
| A, best lever at K8 | 9 | -6/+9/+10 | -55, -13, 39.5 | K8 (9/11) | K9 | 0.86 (1.11, 0.86) | 19.2 |
| B best (arch, no tilt) | 7 | 0/0/0 | -56.5, -6, 44 | K8 (9/11) | K9 | 0.82 (0.99, 0.82) | 16.9 |
| A/B at K6 | 11–12 | about 0 | about -51…-58, -10…-16.5, 19–25 | K6 (7/11) | K9 | 1.03–1.08 | 20–22 |

- **Hypotheses:** B ≥ F-K6, held (F-K8). A ≥ F-K8, held.
- **Decision rule:** not met. A reaches F-K8 but no higher (K9, the palm, is still out of reach). Its best lever at K8 (0.86) is below the tilt-only design's 0.92.
- **The arch and the tilt substitute for each other.** With the arch free, the optimiser turns the thumb's pitch the other way (+8, against -16). An arch of 7 deg alone (B) also reaches F-K8. Both routes turn the pad and the ring tip to face each other.
- **The arch route costs pinch and lever.** It needs a longer case (e 39.5–44.5 against 34): the lever drops to 0.82–0.86, and the thumb–index pinch stays at 17–21 deg from facing instead of 8.8. The arch turns the ring toward the thumb but not the thumb toward the index.
- **Recommendation unchanged:** O_thumbopt (tilt only). It also leaves the finger geometry that every finger study shares unchanged.

### Pre-registration: relaxed envelope + base orientation (2026-11-26, before the run)

- **Why.** The relaxed envelope lets the CMC go palmar (z down to -46) and forward once the RAA servo passes under the index abduction servo (z ≲ -28.5). It was run with position only (F-K4), never with the base orientation free.
- **Run.** O, relaxed, proximal IP, orientation ±20 deg, seed 0, population 32 × 25 generations.
- **Hypothesis.** It reaches at least F-K8.
- **Decision rule.** It replaces O_thumbopt as the recommendation only if it reaches a higher level (K9/K10, the palm), or the same F-K8 with a lever ratio ≥ 0.05 higher or a pinch ≥ 3 deg better. Its packaging cost is reported alongside: how far the RO servo sits below the palm face.

### Result: relaxed envelope + base orientation (2026-11-26)

Run data: `design/output/thumb_layout/h0/relaxed_orient20/proximal/seed0/` (794 evaluations).
- **Best:** x -52.5, z -6.5, e 38, base +3/-19/+7. F-K8 (9/11), touch K9, lever 0.88 (IP 1.15, MCP 0.88), pinch 8.7 deg.
- **The optimum stays beside the abduction servos, not under them.** Every front design has z between -4.5 and -16.5, inside the strict envelope.
- **Under the index abduction servo** (z ≤ -28.5): 176 feasible designs, 109 of them forward of x -49. The best is F-K6 (7/11): x -38.5, z -43.5, e 36.5, base -1/-7/+3, lever 0.89.
  - K7–K9 fail by 6–10 mm and 52–72 deg.
  - K10 has no palm surface under it: the palm block is gone where the thumb sits.
  - Going under frees the forward placement. But it puts the base 30–40 mm below the palm face, and the thumb works from below the fingers' level.
- **Hypothesis:** F-K8 held.
- **Decision rule:** not met. The level is the same (K8) and the lever is lower (0.88 against 0.92). The pinch is 8.7 against 8.8 deg, not 3 deg better.
- O_thumbopt stays the recommendation.

### Double-support RAA bracket (Onshape, 2026-11-26)

- **The structure.** The `oblique-cmc` branch's `thumb-example` assembly holds the RAA servo between two plates: `cmc-aa-bracket`, driven by the RO horn, and `cmc-aa-idler-bracket`, on the far side of the RO axis. Both turn with RO. Neither plate was in the collision model, which had the servo bodies only.
- **Frame check.** The CAD X330 part frame equals the evaluator's servo frame shifted 5.5 mm along the output axis; the CAD body spans z −16.5…5.5 and the evaluator's ±11. With that shift, the CAD RO → RAA transform matches the evaluator's to 0.000.
- **Geometry.** In the RAA frame each plate is 3.5 mm thick on a side face of the servo (x ±9.5…13), so along the RO axis. The idler plate reaches 3.5 mm further toward the fingers.
- **Clearance to the abduction servos, beyond the 2 mm gap,** with the plates swept over the RO range (−90…90):

| design | servo only (before) | with the plates |
|---|---|---|
| prototype | +2.18 | −1.32 (0.68 mm actual gap) |
| O_thumbopt | +0.98 | −2.39 (0.39 mm overlap) |
| O_pitch | +1.46 | −2.04 (0.04 mm overlap) |

- The plates come within 2 mm of the abduction servos at every RO angle: the idler plate sits near the RO axis, so turning RO hardly moves it.
- **Moving the CMC rearward (x), plates included:**

| design | x −50.5 | −52.0 | −53.5 | −55.0 |
|---|---|---|---|---|
| O_pitch | F-K6, −2.04 | F-K6, −0.54 | **F-K6, +0.96** | F-K6, +2.46 |
| O_thumbopt | F-K8, −2.39 | F-K3, −1.10 | F-K3, +0.35 | F-K3, +1.81 |

- O_pitch clears with the CMC 3 mm further back (x −53.5) and keeps F-K6. O_thumbopt's F-K8 does not survive the move, which is one more reason to prefer the pitch-only base.
- **Palm boxes.** The plates also overlap the model's palm hood/block by about 3 mm at some RO angle in every design, the prototype included. The palm boxes are the exporter's axis-aligned stand-in, so this needs checking against the real `palm-base` part, not the model.
- **Not yet done:** the plates are not yet part of the optimisation's constraint (`Evaluator.envelope`).

### Pre-registration: re-optimisation with the double-support plates (2026-11-26, before the run)

- **Change.** `Evaluator(double_support=True)` / `run.py --double-support` adds the two plates (`RAA_PLATES_MM`, conservative boxes) to the RAA sweep in `envelope`. The default is unchanged.
- **Run.** O, strict envelope, proximal IP, orientation ±20 deg, seed 0, population 32 × 25.
- **Hypothesis.** The front moves rearward (x < −52), and pitch-only-like designs keep F-K6. Whether F-K8 survives is open.
- **Decision.**
  - The recommended base is the front's highest F-level that keeps a lever ratio ≥ 0.85. Ties go to the smallest yaw/roll magnitude, since those are easier to build and more robust.
  - Fallback: O_pitch at x −53.5 (F-K6, +0.96 mm), from the probe above.

### Result: re-optimisation with the double-support plates (2026-11-26)

Run data: `design/output/thumb_layout/h0/strict_orient20_dsup/proximal/seed0/` (792 evaluations, 11 min).
- **F-K8 is gone.** With the plates the front's best is F-K6 (8/11; K7 fails), from lever 0.86 (x −54.5, e 40, −7/−17/+14, pinch 9.3 deg) to lever 1.01 (x −55, e 26.5, −13/−19/+10, pinch 14.9 deg).
- **The hypothesis held:** the front moved rearward (x −54…−59.5), and F-K6 survives.
- **Pitch-only base with the plates** (z −10.5, proximal IP):

| x | e 30 | e 32 | e 34 | e 36 | clearance beyond the 2 mm gap |
|---|---|---|---|---|---|
| −53.5 | F-K5 | F-K6 (lever 0.94) | F-K6 (0.92) | F-K4 | +0.96 |
| −55.0 | F-K4 | F-K6 (0.94) | F-K6 (0.92) | F-K4 | +2.46 |

  Its F-K6 window in e is narrow (32–34 mm). The pinch is 15–17 deg.
- **Decision (registered rule).** The highest level is F-K6, with lever ≥ 0.85; ties go to the smallest yaw/roll. That picks the **pitch-only base: x −55, z −10.5, e 33, pitch −16, yaw = roll = 0**. It has F-K6, lever about 0.93 and +2.46 mm clearance with the plates. Choosing x −55 over −53.5 buys margin at no level cost.
- **CAD mesh check of the plates, current CAD pose (O_thumbopt):** sampled surfaces over RO −90…90 put the idler plate 0.29–1.27 mm from the index abduction servo. There is no overlap, but the gap is below the 2 mm design gap. The box model (−0.39 mm) is about 0.7 mm conservative and agrees in direction.
- **Palm.** The `palm-base` part studio is empty, and `hand-layout` has no palm housing around the thumb base. The model's overlap with the exporter's palm boxes is therefore a requirement for the housing still to be designed: it must leave room for the plates' sweep. It is not a clash with existing parts.

### The 45 deg facing rule, and double support against cantilever (2026-11-26)

- **Where 45 deg comes from.** The threshold was set when the functional Kapandji was defined (decision log, 2026-11-25). The decision log records only why it was kept: it binds and the score is not degenerate. It was not derived from anything physical.
- **A physical reading.** A pad pushing along its own normal stays inside the other surface's friction cone if the two normals differ by at most atan(μ). So 45 deg ≈ μ 1.0. The sim pads (μ 1.3) give 52.4 deg, and μ 0.5 gives 26.6 deg, close to the superseded 25 deg rule.
- **Sensitivity** (flat pads, double-support plates in the envelope):

| facing threshold | prototype | O_thumbopt (cantilever, x −50.5) | pitch-only (double support, x −55, e 33) |
|---|---|---|---|
| 26.6 deg (μ 0.5) | K2 (3/11) | K4 (7/11) | K2 (4/11) |
| 45 deg (μ 1.0) | K3 (4/11) | K8 (9/11) | K6 (8/11) |
| 52.4 deg (μ 1.3) | K3 (4/11) | K8 (9/11) | K6 (8/11) |

  The order is the same at every threshold. The levels are the same at 45 and 52.4 deg, and at μ 0.5 every design collapses.
- **Grasp-set benchmark for the double-support pitch-only base** (`O_pitch_ds`, x −55, e 33): median ε over the common pairs (μ 0.5) is 0.220, against O_thumbopt 0.222, O_pitch 0.224 and O 0.129; the spread is 0.016. With 77 feasible pairs, it is not distinguished from O_thumbopt or O_pitch, and is better than O. At μ 1.0: 0.416 against 0.415 / 0.418 / 0.328.
- **Reading.** K7–K8 (ring PIP crease, ring proximal base) are what the cantilever O_thumbopt keeps over the double-support pitch-only base. Those two levels do not show up in grasp quality on the object set.

### Thumb IP hyperextension (user request, 2026-11-26)

- **Change.** `Evaluator(thumb_limits_deg={"ip": (lo, 100)})` and `run.py --ip-min` override the thumb's IP lower limit, which is otherwise the exporter's −10 deg. The sim assets keep the exporter's limits. The 5 deg functional margin still applies.
- **Probe** (double-support plates in the envelope; flat pads; 45 deg facing):

| IP lower limit | prototype | A: pitch-only, double support (x −55, e 33) | B: O_thumbopt, cantilever |
|---|---|---|---|
| −10 (now) | K3, pinch 29.4 | K6, pinch 16.5 | K8, pinch 8.8 |
| −20 | K3, 27.3 | K6, 10.3 | K8, 1.9 |
| −30 | K3, 22.0 | **K8**, 4.8 | K8, 0.0 |
| −45 | K3, 33.8 | K8, 0.0 | K8, 0.0 |

- Every solution uses IP at its margin, the limit + 5 deg.
- With −30 deg the double-support pitch-only base reaches F-K8, so it matches the cantilever design.
- The prototype gains nothing: its limit is RO, not IP.
- The thumb's own self-collision (distal case against the proximal segment) is not modelled. At −30 to −45 deg the case starts 10 mm from the IP axis, outside the servo's 9.5 mm cap, but this needs a CAD check.
- **Pre-registration: re-optimisation with IP ≥ −30 deg** (before the run).
  - Run: O, strict, proximal IP, orientation ±20, double support, `--ip-min -30`, 32 × 25.
  - Hypothesis: the front reaches F-K8 with yaw/roll near zero.
  - Decision rule: as for the double-support run. Highest F-level with lever ≥ 0.85; ties to the smallest yaw/roll; then the best pinch.

### Result: re-optimisation with IP ≥ −30 deg and the double-support plates (2026-11-26)

Run data: `design/output/thumb_layout/h0/strict_orient20_dsup_ip30/proximal/seed0/` (791 evaluations, 12 min).
- **The hypothesis held.** F-K8 is back with near-zero yaw/roll, and a short case gives a high lever.

| design | x, z, e [mm] | yaw/pitch/roll | F | lever | pinch |
|---|---|---|---|---|---|
| c1 (registered pick: K8, \|yaw\|+\|roll\| 7, best pinch) | −55, −11, 23.5 | −2/−17/+5 | K8 | 1.04 | 3.2 |
| c2 | −64.5, −4.5, 31 | −1/−19/+7 | K8 | 0.96 | 6.2 |
| c3 | −64, −7, 40.5 | 0/−11/+7 | K8 | 0.85 | 9.9 |

- **Robustness (post hoc, not in the registered rule).** Each K8 design was perturbed by x, z, e ±1 mm and yaw, pitch, roll ±3 deg, twelve perturbations in all:
  - c1 keeps K8 in 0/12;
  - c2 in 1/12;
  - c3 in 4/12.
  - The rest fall to K4–K6.
  - The K8 optima are isolated points, and K7–K8 sit on the 1 mm / 45 deg edges of the functional rule.
  - K6 is the level a built hand can count on. The registered pick (c1) is the least robust.

### Bent proximal phalanx (user proposal, 2026-11-26)

- **Change.** `ThumbDesign(proximal_bend_deg=β)` adds a fixed bend where the MCP and IP servo bodies meet: 25.75 mm from the MCP axis, about the flexion axis, with negative values toward the dorsal side. The proximal capsule splits in two. β = 0 is unchanged, as the tests check.
- **Probe** (base x −55, z −10.5, pitch −16, double-support plates):

| IP limit | β | e 28 | e 33 | pinch (e 33) |
|---|---|---|---|---|
| −10 | 0 | K4 | K6 | 16.5 |
| −10 | −10 | K5 | K6 | 13.4 |
| −10 | −20 | K5 | chain broken at K0 | 10.6 |
| −10 | −30 | broken at K0 | broken at K0 | 8.2 |
| −10 | +10 | K4 | K4 | 19.7 |
| −30 | 0 | K6 | K8 | 4.8 |
| −30 | −10 | K7 | K6 | 2.5 |
| −30 | −20 | K6 | broken at K0 | 0.4 |

- **What it shows.**
  - A dorsal bend is not the same as hyperextension. It shifts the IP range toward extension and moves the IP axis, so the thumb loses the fold it needs for K0 (index proximal phalanx near the MCP) beyond about −10 deg.
  - Hyperextension adds range at the extension end and keeps the flexion end.
  - A small bend (−10 deg) mainly improves the pinch: 16.5 → 13.4 deg without hyperextension, and 4.8 → 2.5 with it.
  - The bend was probed on one base only. It has not been optimised jointly with the base.

### Correction: the base "pitch" is RO clocking, not an axis tilt (2026-11-26)

- **What the angles do.** The base orientation offsets are added to the prototype's intrinsic Z-Y-X angles (−90, 0, −90). With that base, a pitch offset turns the CMC frame about its own z, which is the RO axis.
  - So pitch leaves the RO axis where it is. It turns the RO servo body and everything after it about the shaft. That is the same chain as the RO joint shifted by the pitch.
  - Check: x −55, z −10.5, e 33 with the double-support plates. Pitch −16 and pitch 0 with the RO limits shifted from ±90 to (−106, 74) give the same F-K6 (6.439), the same pinch (16.5) and the same envelope. A +16 shift gives K3.
  - `test_base_pitch_is_ro_clocking` pins this down.
- **What tilts the axis.** Yaw turns the RO axis within the palm plane, about H z. Roll tilts it out of the palm plane.
- **Earlier designs, split into axis direction and clocking:**

| design | axis in-plane [deg] | axis elevation [deg] | RO clocking [deg] |
|---|---|---|---|
| O_thumbopt (−14/−16/+14) | −10.1 | +13.4 | −17.7 |
| pitch-only A (0/−16/0) | 0 | 0 | −16.0 |
| c1 (−2/−17/+5) | −0.5 | +4.8 | −17.1 |
| c2 (−1/−19/+7) | +1.3 | +6.6 | −19.1 |
| c3 (0/−11/+7) | +1.3 | +6.9 | −11.0 |

- **What this means.**
  - The step from K4 to K6 that these notes credited to the base pitch is about 16 deg of RO clocking. Mounting the RAA bracket on the RO horn 16 deg round gives it; so does moving the RO range to (−106, 74). Neither tilts the RO axis.
  - An axis out of the palm plane (elevation) is what the K8 designs add: 5–13 deg.
  - The user's manufacturing constraint (2026-11-26) was "yaw oblique is fine, pitch is hard". In this parametrisation that means roll = 0, with yaw and pitch (clocking) free.

### Pre-registration: RO axis in the palm plane (2026-11-26, before the run)

- **Run.** Strict envelope, proximal IP, double-support plates, IP ≥ −30.
  - Free: x, z, e; yaw ±20; pitch (clocking) ±30; bend β in [−15, +5].
  - Fixed: roll = 0.
  - Runner options: `--pitch-max 30 --roll-max 0 --bend -15 5`.
- **Two runs:**
  - nominal: population 32 × 25 generations;
  - robust (`--robust`): population 24 × 20. The objective is the functional score averaged over the design and its ±1 mm (x, z, e) and ±3 deg (yaw, pitch, bend) one-at-a-time perturbations, 13 points in all. The level is taken as kept when every perturbation reaches at least the nominal level.
- **Hypotheses.**
  - With the axis kept in the palm plane, the nominal front reaches F-K6 and not K8. The earlier K8 designs all needed 5–13 deg of elevation.
  - The robust front holds K6 under all 12 perturbations at a lever ratio ≥ 0.9.
- **Decision rule.** Among robust-front designs with lever ≥ 0.85: the highest level kept under every perturbation, then the highest robust score, then the best pinch.

### Result: RO axis in the palm plane, nominal run (2026-11-26)

Run data: `design/output/thumb_layout/h0/strict_flat_ip30/proximal/seed0/` (796 evaluations, 14 min).
- **The first hypothesis failed: F-K8 is reachable with the RO axis in the palm plane (roll 0).** IP ≥ −30 and the free clocking and bend do what the axis elevation did before.

| x, z, e [mm] | yaw (in-plane) | clocking | bend | F | lever (IP/MCP) | pinch |
|---|---|---|---|---|---|---|
| −53.5, −4.5, 36.5 | +2 | −20 | +2 | K8 | 0.89 (1.21/0.89) | 1.6 |
| −57.5, −3.5, 40.5 | +8 | −8 | −1 | K8 | 0.85 (1.08/0.85) | 9.3 |
| −56.5, −12.5, 38.5 | −1 | −7 | +4 | K8 | 0.87 (1.14/0.87) | 11.7 |
| −56.5, −9.5, 25.5 | −10 | −19 | −8 | K6 | 1.03 (1.78/1.03) | 3.4 |

- **The RO range (user, 2026-11-26).** The RO joint has no limit. Clocking therefore only chooses which 180 deg arc of RO the RAA servo has to clear the abduction servos over (the envelope's RAA sweep), and where the ±90 deg working range sits. It is not a hardware tilt.
- K8 needs a case of 36–44 mm. The short cases (e 15–25) stay at K6.
- Robustness: the robust run is still going.

### Lever trade-off

- Within a functional level the front runs mostly along e: a longer case reaches more but pushes less.
- Strict, proximal IP: F-K4 holds from e = 42 (lever 0.84) down to e = 29.5 (lever 0.97). K8 drops out below e ≈ 31.
- Distal IP has the IP lever as the binding constraint (0.73 at e = 43.5). Turning the IP servo toward the MCP moves the pad closer to the IP axis for the same reach (IP lever 1.04–1.30), and costs only MCP lever.
- Orientation front, proximal IP:

| level | lever ratio | e [mm] | pinch [deg] |
|---|---|---|---|
| F-K8 | 0.92 | 34 | 8.8 |
| F-K6 | 0.93–1.04 | 24–33 | 9.6–12 |
| F-K5 | 1.05 | 22.5–23 | 11–12 |

### Recommended design: O_thumbopt

- **Thumb design:**
  - CMC servo at x -50.5, z -10.5 (y 0.77 as before);
  - base pose (yaw, pitch, roll) = (-104, -16, -76) deg, i.e. the prototype's (-90, 0, -90) plus offsets (-14, -16, +14);
  - IP servo turned toward the MCP (`ip_servo_flipped`), mcp_to_ip 51.5 mm;
  - fingertip case e = 34 mm past the cap bracket, EE 44 mm from the IP axis.
- **Scores:**
  - functional Kapandji **K8** (9/11; K9 palm at the ring base fails: pad 4.4 mm short, facing 46 deg; K10 fails: 14.7 mm, 75 deg);
  - touch Kapandji K9 (10/11);
  - lever ratio 0.92: IP lever 31.5 mm against 41.0 for the prototype (1.30×), MCP 83.0 mm against 76.5 (0.92×).
- **Pinch** (pad centre on the finger pad's normal line, the finger free), by gap band. Prototype first, then O_thumbopt:

| gap | index | middle |
|---|---|---|
| 6–9 mm | 26.3 → 17.5 deg | 33.6 → 15.6 deg (2.4 mm off the line) |
| 9–14 mm | 21.2 → 12.9 | 30.1 → 12.3 |
| 14–20 mm | 18.0 → 8.3 | 24.7 → 6.9 |

- **Drake URDF cross-check** of the solver's best 6–20 mm pinch poses (`sim/drake`, URDF `cfl_q120_h3to7_h0_thumbopt_thumbpad.urdf`):

| hand, finger | from facing | gap | off the line |
|---|---|---|---|
| O_thumbopt, index | 8.4 deg (opposition 171.6) | 20.5 mm | 1.3 mm |
| O_thumbopt, middle | 8.8 deg | 18.2 mm | 1.9 mm |
| O, index | 18.0 deg | 20.4 mm | — |
| O, middle | 24.8 deg | 21.4 mm | — |

  The solver and the URDF agree to within 0.1 deg.
- **The earlier `pinchfk/cmc.py` scan.** It used a joint grid with the finger's abduction at 0 and the thumb normal within 3 deg of the finger's flexion plane. Best angle from facing, prototype → O_thumbopt:

| finger | 6–9 mm | 12–20 mm |
|---|---|---|
| index | 30 → 19 deg | 20–25 → 14 deg |
| middle | — | 38–43 deg (8–18 mm) on O_thumbopt |

  The prototype numbers match the earlier Drake finding of 29–31 deg. With RO at -70, it is no longer at its limit. The middle needs abduction (about ±15 deg) to face the tilted thumb, which that scan forbids.
- **Minimum graspable diameter:** 2 mm, the same as the prototype. No independent DIP (by design).
- **Hardware implications:**
  - The CMC servo pose changes by yaw -14 deg (about H's z), pitch -16 deg and roll +14 deg (intrinsic Z-Y-X). It sits 4.5 mm more palmar and 0.7 mm further forward than now.
  - Envelope margins:
    - the RAA servo's sweep clears the abduction servos by 0.98 mm beyond the 2 mm gap;
    - the RO servo is 5.5 mm under the hood top and 3.9 mm above the 20 mm floor;
    - the RO body is 25.9 mm clear of the abduction servos and fingers.
  - The proximal segment grows from 35.5 to 51.5 mm (both servo bodies end to end). The distal segment shrinks from 53.5 to 44 mm (cap bracket + 34 mm case).
  - The thumb pad is unchanged (25 mm, on the case).
- **Simulation assets:**
  - `design/output/sim/study3/hands/cfl_q120_h3to7_h0_thumbopt.yaml`, written by `scripts/optimize/thumb_layout/write_hand.py` from `sim/isaaclab/scripts/rotate_cube/export_transfer_hands.sh`;
  - description `sim/isaaclab/logs/rotate_cube/transfer/hands/cfl_q120_h3to7_h0_thumbopt.json`;
  - URDF and USD (with colliders) in `sim/isaaclab/logs/urdf/cfl_q120_h3to7_h0_thumbopt_thumbpad*`;
  - registered as `O_thumbopt` (`grasp.HANDS`, `CoDe-Rotate-Cube-O_thumbopt`).
- **Visual fix (2026-11-26).** The first O_thumbopt render showed the fingertip case floating about 16 mm past the IP servo's cap. The sim's visual builder (`sim/isaaclab/src/code_isaaclab/assets/visuals.py`, `thumb_tip_case`) always started the case at the IP servo's rear face (25.5 mm from the IP axis), which is right only when the body lies distal. It now starts past the cap (10 mm) when the servo is turned toward the MCP. Only the drawing-only visuals were wrong. The colliders were not: the link capsules and the thumb pad box come from the description's joint anchors and end effector. The URDF and USD were re-exported.
- **Contiguity check** after the fix: consecutive thumb bodies (palm, CMC, metacarpal, proximal, distal) touch or overlap, by surface distance between their visual pieces. At the zero pose and at (-60, 30, 50, 60) the largest gap is 0.5 mm, the designed IP-cap clearance; the prototype's is 0. The collision capsules are contiguous (gap 0).
- **Renders** (Drake VTK, `_vis.urdf`):
  - zero pose, palm and side views: `thumbviz/thumb_compare_fixed.png`;
  - best thumb–index pinch with a 20 mm sphere between the pads (pad centres 20.4 and 20.5 mm apart): `thumbviz/thumb_pinch_sphere.png`.
  - Both are in the session scratchpad, beside the coordinator's `r.py`.
  - The tilted CMC servo and the CMC plate stick out of the palm's heel box. The exporter's housing boxes are axis-aligned and wrap the tilted servo's bounding box, so a tilted base reads as a block with a slanted plate on top. This is a drawing and housing-shape matter; the palm–thumb collision pairs are filtered in the sim.
- **Smoke checks:**
  - the URDF FK matches the model to 1e-3 mm, for the pad and normals, including the turned IP servo and the tilted base (re-checked after the fix);
  - the Isaac Lab asset (`description_asset`, thumb pad, visuals) builds on CPU with 19 tree joints and 7 pads, and the four-bars close in Drake;
  - the cube-rotation task's grasp (`grasp.pinch_grasp`) fixes the thumb at RO -90 / RAA 20 for O. That grasp does not exist on O_thumbopt ("the thumb pad cannot press the cube's top face"), so the cube-task env needs a per-hand thumb grasp before it can run on this hand. Fixing other RO/RAA values does not help: none of (RO, RAA) = (-90, 20), (-70, 0), (-70, 20), (-50, 0), (-50, 20), (-90, 40), (-70, 40) gives a grasp either. The grasp needs the pad within 20 deg of straight down on the cube's top face, within 8 mm of the finger plane, and the tilted base moves the reachable pad poses off that. The grasp's thumb search would have to include RO, RAA and the cube's placement. That is a task change, not done here.

### O, distal IP with the orientation free

The distal-IP orientation run was rerun after the warm-start fix. Best design: x -50.5, z -7, e 38, base -5/-15/+8 deg.
- F-K8 (9/11, score 8.93); touch K9; pinch 10.3 deg.
- Lever ratio 0.80 (IP 0.80, MCP 0.88), against 0.92 for the proximal-IP optimum at the same level.
- Both orientations land at a base pitch of -15 to -16 deg; the proximal IP keeps the better lever and pinch.

## Results: R and U (strict envelope, proximal IP, 1 seed each)

| hand | run | x, z [mm] | e [mm] | base yaw/pitch/roll [deg] | functional | touch | lever ratio (IP, MCP) | pinch [deg] | min grasp |
|---|---|---|---|---|---|---|---|---|---|
| R | prototype thumb | -51.2, -6.0 | 28 (distal IP) | 0/0/0 | K3 (4/11) | K9 (10) | 1.00 | 23.3 | 2 mm |
| R | position only | -52.5, -11 | 43 | 0/0/0 | K4 (7/11) | K9 (10) | 0.83 (1.01, 0.83) | 17.4 | 2 mm |
| R | + orientation ±20 | -59.5, -5 | 43 | -10/-19/+11 | **K8 (9/11)** | K9 (10) | 0.83 (1.01, 0.83) | 9.1 | 2 mm |
| U | prototype thumb | -51.2, -6.0 | 28 (distal IP) | 0/0/0 | K3 (4/11) | K9 (10) | 1.00 | 23.4 | 2 mm |
| U | position only | -52, -8.5 | 40 | 0/0/0 | K4 (6/11) | K9 (10) | 0.86 (1.09, 0.86) | 17.4 | 2 mm |
| U | + orientation ±20 | -56, -5.5 | 44.5 | -1/-16/+13 | **K8 (9/11)** | K9 (10) | 0.82 (0.98, 0.82) | 4.3 | 2 mm |

- **Same picture on all three morphologies.**
  - The prototype thumb reaches F-K3. On R and U, K5 fails at 55 deg facing.
  - Position and extension alone stop at F-K4: the ring tip fails on facing.
  - A base pitch of -15 to -20 deg, with roll +6 to +14 deg, opens K5–K8 and brings the pinch under 10 deg.
- **K9 and K10 (palm) stay out of reach everywhere.** The pad cannot fold flat onto the palm next to its own base within the IP/MCP ranges and the 5 deg margin.
- **The R optimum's pitch sits at the -20 deg bound.** On R, more tilt may help further.
- **Shorter-case designs on the orientation fronts:**
  - R: F-K6 at e 31 with lever ratio 0.95 and pinch 8.0 deg;
  - U: F-K6 at e 33 with lever ratio 0.93 and pinch 4.7 deg.
- **Morphology barely changes the thumb's best layout.** The three optima share the IP orientation and the pitch/roll range. They differ mainly in x (-50.5 for O against -56…-59.5 for R/U) and in e (34 against 43–44.5).
- **For O, R and U the CMC base orientation is the design variable that matters for dexterity here.** Its position within the palm envelope, the IP servo turn and the case length refine levers and pinch.
