# Thumb and hand-layout optimisation (robotic Kapandji)

Status: experimental. Code: `design/core/optimization/thumb_layout/` (evaluator), `design/scripts/optimize/thumb_layout/run.py` (NSGA-II runner). Run data: `design/output/thumb_layout/<hand>/<envelope>/<orientation>/seed<N>/` (not committed).

## Why

In the Drake pinch experiments a parallel thumb–index pinch was impossible on the current hand. At 6–9 mm gaps the best opposition was 29–31 deg from facing, with `thumb_ro` (-90 deg) and `thumb_ip` at their joint limits. This study asks whether the thumb's base position, the IP servo's orientation and the fingertip length can fix that for a given finger morphology M. M and the three fingers' bases stay fixed.

## Formulation (written before the runs)

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

**Palm envelope (bounds).** The fingers' abduction servos fix the palm.
- **Forward limit.** The RAA servo turns with RO. Over the full RO range it must clear the abduction servos by 2 mm. While the RAA body is dorsal of the abduction servos' bottom (z = 0), this caps x at -49.0. The prototype is at -51.2.
- **Dorsal limit.** The RO servo must stay below the hood's top (z_top = 24 mm), so z ≤ -3.5.
- **Palmar limit.** The RO servo stays at most 20 mm below the palm's flat palmar face (-6.1), so z ≥ -16.5. The prototype servo is 9.4 mm below that face.
- **Rear limit.** At most 20 mm behind the prototype: x ≥ -71.
- `strict`: x ∈ [-71, -49], z ∈ [-16.5, -3.5]. This is the envelope asked for.
- `relaxed` (an extension, to show what the envelope costs): x ∈ [-80, -25], z ∈ [-46, -3.5], floor 50 mm below the palm face. The RAA-sweep clearance is a constraint here; it lets the base go forward only once the RAA body passes under the abduction servos (z ≲ -28.5).
- The palm housing is rebuilt per design with the exporter's `palm_housing`.

### Objectives (NSGA-II, both maximised; minimised as -score and the angle)

1. **Kapandji score.** There are 10 targets for the thumb pad's face centre:
   - per finger (index, middle, ring): `tip` (the distal pad face, 5 mm short of its distal end), `distal` (the distal pad face centre) and `proximal` (the proximal pad face centre);
   - `palm`: the palm block's palmar face under the ring finger's base (6 mm behind its front edge), normal -Z.

   A target is reached if some configuration meets all of these (touch, as in the clinical test; see the decision log):
   - thumb joints and the target finger's AA/FE/PIP lie within their limits; the other fingers stay extended at zero;
   - pad centre to target point ≤ 1 mm;
   - the pad normal within 60 deg of facing the target's. This only excludes side or back contact; opposition quality is objective 2's;
   - no collision.

   Score = count + 0.999 × mean over unreached targets of 1/(1+r), where r = (pos - 1)+/5 + (angle - 60)+/60 + (penetration - 1)+/5 at that target's best candidate.
2. **Pinch opposition.** For the index and the middle distal pad separately: the thumb pad centre must lie on the finger pad's normal line (≤ 2 mm off it) at gap g ∈ [6, 20] mm (g is free), thumb and that finger free, collision-free. The objective is the smallest angle from facing (= 180 - opposition) over such solutions. Off-line or colliding candidates are penalised by 10 deg per mm. The worse of index and middle is taken.

**Constraints.**
- Joint limits (in the IK).
- Collision-free at the solution poses (inside the target test).
- Envelope margins ≥ 0: RAA sweep, RO under the hood top, RO above the floor.
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
  - thumb (pad excluded) against the target finger;
  - thumb proximal/distal/pad against every palm box (the pad not for the palm target);
  - thumb RAA servo/metacarpal against hood and block only (the heel and bridge are built round the base);
  - the target finger's segments against the other fingers.

### Solver

- Seeds: a 15 deg thumb grid and a 10 deg finger grid, each filtered to be collision-free, then matched on [position, 30 mm × normal] with a KD-tree. The best 16 pairs are kept.
- Each seed is refined with 40 steps of task-priority damped least squares: position first, the normal in its null space. It uses finite-difference Jacobians and an active set at the joint limits, and a step is accepted only if it keeps a clear pose clear.
- About 1.8 s per design.
- A brute-force check on the prototype agrees with the solver: thumb on a 5 deg grid, index on a 2.5 deg grid, pad within 1.5 mm. Its best angles are 30.3 deg (index tip) and 33.6 deg (index distal); the solver finds 30.5 and 34.0.

## Decision log

- **2026-11-25, Kapandji rule changed from "touch + facing within 25 deg" to "touch" (user decision).**
  - Why: under the 25 deg rule the prototype reached 0/10 targets, and every failure was the facing angle, not reach. The pad centres were within 1 mm, and the best angles were 30.5 deg at the index tip, 34 at the index distal and 28 at the palm. The score therefore duplicated the pinch objective (opposition quality) instead of measuring reach.
  - Now a target counts if the pad centre is within 1 mm of it, collision-free and within the limits, with only a loose pad-side condition: the normal within 60 deg of facing, which excludes side or back contact. The pinch objective is unchanged.
  - When: after the prototype baseline and before any optimisation result was used. The first NSGA-II batch under the 25 deg rule (O, strict, 3 distal seeds and 1 proximal seed) was stopped, set aside in `design/output/thumb_layout/_superseded_facing25/`, and not used.
  - With the rule, the touch seeds are matched with a weaker normal weight (10 mm per unit instead of 30). On four test designs this weight reached as many targets or more than 30 or 3; the pinch keeps 30.

## Prototype baseline (M = O, touch rule)

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
