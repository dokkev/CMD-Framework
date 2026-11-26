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

1. **Kapandji score** (above), primary.
2. **Pad lever ratio:** min(IP, MCP) of (the prototype's lever / the design's lever), i.e. pad force per joint torque relative to the prototype. Chosen over the thumb's workspace volume as the second measure of general dexterity:
   - the Kapandji levels already measure the opposable workspace;
   - a larger workspace mostly rewards reach away from the fingers;
   - the lever is the quantity that trades against reach (a longer thumb reaches further and pushes less), so the front shows what each extra level costs in force.
   - It stays bounded below by the 0.7 constraint.

**Reported, not optimised:**
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

## Prototype baseline (M = O, All Hands Up Kapandji)

Prototype thumb: (x, z) = (-51.2, -6.0), IP body distal, e = 28.0 (EE 53.5 mm), levers IP 41.0 / MCP 76.5 mm.
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

## Results

(below, after the runs)
