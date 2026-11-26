# Drake: prototype thumb (O) against the oblique-CMC thumb layout (O_thumbopt)

Status: experimental. Not for Study 3, the paper or the SSoT. Code: `sim/drake/thumbopt_drake.py` (new; the task modules are not edited), `sim/drake/load_hand.py` (O_thumbopt added, with the transfer-hand description fallback). The hand and its design are in `docs/experimental/thumb_layout_opt.md`.

Question: on the Drake tasks where the thumb matters, does O_thumbopt do the task better than O?

## Pre-registration (2026-11-26, before any O_thumbopt task data)

### Inventory: which Drake tasks involve the thumb

- **The fingers are identical.** The two descriptions agree exactly on the finger joints, bodies, pads, four-bar tables, servo and collision filters. Every planar task builds one finger from the hand's parameter file, with the MCP fixed (the proximal phalanx is the world) and no palm or thumb. So 21 (pull-out), 23 (acquisition), 29z (fingertip disk), 30/30b (coin), 34 (pad shape), 35 (pivot), 38 (hook), 39 (twist), 40 (slider), 41 (ball), 42 (sphere acquisition) and `acq_mcp` give O_thumbopt **bit-identical results by construction**. None is run; this is the null control by construction rather than by a run.
- **33 (`pinch_rotate.py`, floating thumb pad).** The opposing pad is a spring-mounted box, the same on every hand. It does not see the hand's thumb, so it is also identical by construction. What 33 does involve is whether the real thumb can put its pad where 33's opposing pad sits (`pinch_thumb_ik.py`: O 0/64). That reach check is task T1 below.
- **33b (`pinch3.py` + `pinch_rotate.py`, real thumb).** The thumb's pad normal, contact point and servo compliance (through its lever arms) come from the hand's URDF at a pinch pose. These are thumb properties. Task T2.
- **36 (`peg_pinch.py`, pinch peg-in-hole).** It uses the same real-thumb pose and was stopped at O's calibration because the index face was 33–42 deg off the thumb face (no parallel pinch within the joint limits). O_thumbopt was designed to fix exactly this. Task T3.
- **Thumb pose per hand.** 33b and 36 used O's hand-picked pose (thumb RO −90, RAA 10, MCP 45, IP −7.5 deg; index/middle FE 72.5, or 70 for spheres of 20 mm and more). That pose does not exist on O_thumbopt. Both hands therefore get their pose from one rule (`POSE_RULE`, `BANDS` in `thumbopt_drake.py`), fixed before any task run:
  - fingers: index FE free (30–100 deg on a 2.5 deg grid, ±1.25 in the refinement), AA 0, PIP 0, ring curled; for 33b the middle at the index's FE (the thumb faces the index–middle mid-plane), for 36 the middle curled (the thumb faces the index);
  - the thumb within its URDF limits, pad face centre within 3 mm of that lateral plane, within 2 mm of the index distal pad's normal line, at the band's gap from the index pad face (33b: 16–20 mm for the task's FE-72.5 spheres, 20–24 mm for its FE-70 spheres; 36: 6–9 mm, the peg diameters), and clear of the palm and fingers (signed distance > 0);
  - minimise the 3D angle between the thumb pad normal and the index pad's inward normal. 5 deg thumb grid at every FE, then 1 deg ±5 round the 20 best clear poses.
  - The planar models do not depend on FE (MCP fixed, no gravity); FE only places the thumb relative to the finger.
  - **Check.** The rule reproduces O's hand-picked 33b pose: (−90, 10, 47, −10) at FE 72.5, 19.9 deg in plane, against (−90, 10, 45, −7.5), 20.4 deg. The generalised `thumb_in_plane` matches `pinch3_pose.thumb_in_plane` on the hand-picked pose to 4e-6.
  - **A variant considered and rejected before any task data.** PIP at the midpoint of the task's start flexion (33b 5 deg, 36 17 deg) instead of 0. It made every pinch worse (33b: O 24.6, O_thumbopt 13.6 deg), and for 36 O had no clear pose at all (O_thumbopt 30.1 deg). PIP 0 is kept, as in `pinch3_pose`; the tasks flex or extend the PIP from there. Kept in `logs/thumbopt_pose_pipmid_rejected.json`.

Poses (`logs/thumbopt_pose.json`; angles from facing, 3D / in the finger's plane):

| band | hand | FE | thumb RO, RAA, MCP, IP | opposition 3D / in-plane | gap | k_thumb (N/m) |
|---|---|---|---|---|---|---|
| 33b, 16–20 mm | O | 72.5 | −90, 10, 47, −10 | 21.4 / 19.9 | 19.6 | 1533 |
| | O_thumbopt | 61.25 | −75, −9, 36, −10 | 10.3 / 10.2 | 19.8 | 1477 |
| 33b, 20–24 mm | O | 71.25 | −90, 10, 45, −10 | 18.5 / 16.7 | 23.4 | 1532 |
| | O_thumbopt | 60.0 | −74, −8, 34, −10 | 7.2 / 7.2 | 23.8 | 1487 |
| 36, 6–9 mm | O | 78.75 | −90, 0, 49, −10 | 27.7 / 27.7 | 8.6 | 1591 |
| | O_thumbopt | 70.0 | −67, −9, 34, −10 | 18.4 / 18.3 | 9.0 | 1476 |

These match `thumb_layout_opt.md`'s pinch table (index 6–9 mm 26.3 → 17.5 deg, 14–20 mm 18.0 → 8.3 deg). O's RO sits at its −90 limit, and every pose has IP at its −10 limit.

### Tasks, metrics, success criteria

The settings are each task's own: point contact, SAP, time step 0.25 ms, μ 1.3 and 2.0, the task's draw distributions and seeds, O's open-loop finger command. The finger is O on both hands. Nothing is retuned per hand; only the draws' thumb geometry (normal, contact point, k_thumb) comes from the hand's rule pose.

**T1. Reach of experiment 33's opposing-pad placements** (`thumbopt_drake.py reach`: the committed `pinch_thumb_ik.py` logic, per hand).
- Setup: the 64 e33 draws (`logs/pinch_e33.json`), the sweep's start and end poses. The thumb and the index MCP (FE, AA) are solved by least squares within the URDF limits, from 24 random starts.
- Draw reached: pad face centre within 1 mm and normal within 5 deg at both start and end, with no thumb–hand collision and the thumb ≥ −0.5 mm from the sphere.
- Metric: the number reached (of 64). Secondary: the median normal error at the start.
- Noise: the IK's random starts. Seeds 0, 1, 2 per hand; the band is each hand's range over the seeds.
- Hypothesis: O_thumbopt reaches more placements than O (O 0/64 in `pinch_thumb_ik.py`). e33 asks for exact facing (5 deg) at PIP 30–55 deg, so both may stay near 0. The secondary error then shows whether O_thumbopt is closer.
- Decision: "O_thumbopt better" if its smallest count over the seeds exceeds O's largest by ≥ 3 draws. "Worse" is the mirror. Otherwise "no difference". The secondary differs if the seed ranges of the medians do not overlap.

**T2. 33b three-finger pinch rotate, real thumb** (`thumbopt_drake.py draws33b`, then `pinch_rotate.py run --hands O`).
- Setup: 64 draws (seed 34, 33b's ranges), each hand's 33b rule poses.
- Draw success: at both μ, no solver failure, not lost (`pinch_rotate.is_lost`), penetration ≤ 1.5 mm and no proximal-pad contact (> 1e-3 N). These are 33b's own gates.
- Metrics:
  - primary: the success count;
  - secondary: the sphere's rotation at μ 1.3 (median over the draws both hands complete), and D = rot(2.0) − rot(1.3) (median).
- Noise:
  - O at its rule pose with the command +0.05 deg (tag `thoO33bn`): the success-count difference, and the 75th percentile of the per-draw |Δ rotation|;
  - O's rule pose against O's hand-picked pose (the existing `pinch_O_e33b.json`): |Δ median rotation|.
  - The rotation band is the largest of these three numbers; the count band is max(3, 2 × the count difference).
- Hypothesis:
  - success: both hands at or near the ceiling (O kept contact in 64/64 before), so no difference;
  - rotation and D: no directional prediction. The opposition is closer to facing, but the planar model only sees the normal's direction and k_thumb; two-sided.
- Decision:
  - success differs if |Δ count| exceeds the count band;
  - rotation (and D) differ if |Δ median| > 2 × the rotation band;
  - otherwise "no difference".

**T3. 36 pinch peg-in-hole** (`thumbopt_drake.py draws36`, then `peg_pinch.py run --hands O`).
- Setup: 64 draws (seed 36, 36's ranges: q_s 12–22 deg, PIP extension 6–10 deg, peg r 3–4.5 mm, etc.), each hand's 36 rule pose, k_thumb from that pose. Everything else is 36's DEFAULTS.
- Draw success at μ 1.3 (the primary): no solver failure, not lost (`peg_pinch.is_lost`), penetration ≤ 1.5 mm, and the peg's leading tip ≥ 1.0 mm past the slot entrance at the end (`depth_mm` ≥ 1). μ 2.0 is reported the same way.
- Metric: the success count. Secondary: the median depth gain (end − start), and the number of draws where the index touches the peg's front end (geometry, from the draws).
- Noise: O at its rule pose with the command +0.05 deg (tag `thoO36n`). The count band is max(3, 2 × |Δ count|).
- Hypothesis: O fails as in its calibration (12/12 lost, index on the slot wall). O_thumbopt, 9 deg closer to facing, succeeds in more draws.
- Decision: "O_thumbopt better" if its count exceeds O's by more than the band, "worse" in the mirror case, else "no difference".
- At q_s 12–22 deg the index pad turns a further 12–22 deg away from the thumb. O_thumbopt's pinch at the task's start is then still about 30–40 deg from facing, so T3 may fail on both hands. That would still be thumb-related (opposition), so it counts as a result, not an abort.

**Reference, not part of the decision:** O at its hand-picked pose on T3 (the existing 12-draw calibration `peg_O_pgcal0.json` used the same draws file at the 33b pose).

### Order, self-check, abort

1. T1, O and O_thumbopt, seeds 0–2 (about 4 s each).
2. T3 self-check: the first 12 draws for both hands. If both hands fail every draw through a solver failure or a numerical artefact (not a contact outcome), stop T3 and record why. Otherwise run all 64 for both, plus O's noise run.
3. T2: 64 draws for both hands, plus O's noise run (about 1 min each at 6 workers).

Limits: at most 6 worker processes (5 for T1).

## Results

Paused after T1 at the user's request and resumed the same day. T3 and T2 were then run exactly as registered (the commands under "Resume").

### Summary

| task | metric | O | O_thumbopt | noise band | verdict |
|---|---|---|---|---|---|
| T1 e33 reach | placements reached (1 mm, 5 deg) | 0/64 | 0/64 | 0 (IK seeds) | no difference |
| T1 (secondary) | median normal error at start | 25.6 deg | 19.0 deg | ~0 | O_thumbopt closer |
| T3 36 peg | successes at mu 1.3 (2.0) | 0/64 (0/64) | 0/64 (0/64) | 3 (command noise 0) | no difference: both fail |
| T2 33b | successes (both mu) | 64/64 | 64/64 | 3 (command noise 0) | no difference (ceiling) |
| T2 (secondary) | median rotation at mu 1.3 | 3.10 deg | 3.72 deg | 0.050 | O_thumbopt +0.62 deg (> 2 x band) |
| T2 (secondary) | median D = rot(2.0) - rot(1.3) | 1.64 deg | 0.72 deg | 0.074 | O_thumbopt -0.91 deg (> 2 x band) |

### T1. Reach of the e33 placements (2026-11-26; `logs/thumbopt_reach_<hand>_s<seed>.json`)

| hand | reached, start and end (seeds 0 / 1 / 2) | start only | end only | median normal error at start | median position error |
|---|---|---|---|---|---|
| O | 0 / 0 / 0 of 64 | 0 | 0 | 25.6 deg | 18.1 mm |
| O_thumbopt | 0 / 0 / 0 of 64 | 0 | 0 | 19.0 deg | 12.4 mm |

- Noise: the three IK seeds agree to 1e-4 deg and 1e-4 mm, so the band is effectively zero.
- **Primary: no difference.** Neither hand reaches a single placement within 1 mm and 5 deg. No placement is within tolerance even with collision ignored.
- **Secondary: O_thumbopt is closer.** Its median normal error is 6.7 deg smaller, and the seed ranges do not overlap. The position error is also 5.7 mm smaller (reported, not registered).
- Reading: e33's opposing pad asks for exact facing on a distal pad flexed 30–55 deg. O_thumbopt moves about a quarter of the way toward it but does not get there. The floating pad of experiment 33 remains unreachable by either real thumb.

### T3. 36 pinch peg-in-hole (2026-11-26; `logs/peg_O_thoO36.json`, `peg_O_thoT36.json`, `peg_O_thoO36n.json`)

- Geometry from the draws: at the task's start flexion (q_s 12-22 deg) the index-thumb angle from facing is 40.0-49.7 deg on O and 30.5-40.3 deg on O_thumbopt. The index touches the peg's front end in 64/64 draws on both hands.
- Self-check (first 12 draws): both hands 0/12, every draw lost, with no solver failure. This is a contact outcome, which the registration counts as thumb-related (opposition), not an abort, so the full 64 were run.
- Full 64: **0/64 successes on both hands at both mu.** Every draw is lost and none reaches 1 mm past the entrance. The command-noise run is also 0/64, so the band is 3.
  - The final depth (median) is -3.8 mm on O and -2.8 mm on O_thumbopt. The depth gain over the extension is 0.0 mm (median) on both, at most 1.4 and 1.3 mm.
  - The index pad presses the slot wall at up to 30 N on both hands, the failure seen in O's original calibration.
- **Verdict: no difference.** The hypothesis that O_thumbopt succeeds in more draws is not supported.
- Why: O_thumbopt's pinch is 9 deg closer to facing, but the task's 12-22 deg PIP start flexion still leaves it 30-40 deg off. The index still meets the peg's end rather than its side and follows it into the slot. The pre-registration flagged this risk. 36 needs a pinch within a few degrees of facing at the task's flexion, which neither thumb has within its joint limits (every rule pose has IP at its -10 deg limit).

### T2. 33b three-finger pinch rotate (2026-11-26; `logs/pinch_O_thoO33b.json`, `pinch_O_thoT33b.json`, `pinch_O_thoO33bn.json`)

- Draws: the thumb is 16.4-30.3 deg from facing on O and 6.9-20.6 deg on O_thumbopt (in the plane, per draw); k_thumb is 1533 and 1477 N/m.
- **Success: 64/64 on both hands.** No solver failure, loss, over-pressing or proximal contact; penetration at most 1.22 and 1.27 mm. Verdict: no difference (both at the ceiling, as predicted).
- **Rotation at mu 1.3:** median 3.10 deg on O and 3.72 deg on O_thumbopt, +0.62 deg; O_thumbopt turns the sphere further in 58 of the 64 draws.
  - The band is 0.050 deg: command noise (75th percentile) 0.012, and O's rule pose against its hand-picked pose (`pinch_O_e33b.json`, 3.05 deg) 0.050.
  - **The difference is 12 x the band.**
- **D (mu effect):** median 1.64 deg on O and 0.72 deg on O_thumbopt, -0.91 deg, against a band of 0.074 (command 0.009, pose 0.074). With the thumb closer to facing, the turn depends less than half as much on the pads' friction.
- No direction was registered for either secondary metric (two-sided). Both differ by more than twice the band.
- Reading (not tested): with the normals nearly opposed, less of the squeeze is spent on tangential friction to hold the sphere in place. More of the finger's sweep therefore goes into turning it, whatever the friction. In this task that is the better behaviour: the same command turns the sphere about 20 % further, with a turn that is more robust to friction. It is a modest, planar-model effect. The grasp itself was secure on both hands.

### Overall

- Where the task grasp is already secure (33b), O_thumbopt keeps it and turns the object further, with less dependence on friction: a real but modest gain.
- Where the task needs near-parallel opposition at a flexed finger (36, e33 reach), O_thumbopt moves the geometry the right way (6.7-9 deg closer) but not far enough: both hands fail alike.
- The finger-only tasks are unchanged by construction (identical fingers, no thumb in their models).

### Resume (as run)

```
cd sim/drake
# T3 self-check (first 12 draws), then full
uv run python thumbopt_drake.py draws36 --hand O --out logs/peg_tho_O.json
uv run python thumbopt_drake.py draws36 --hand O_thumbopt --out logs/peg_tho_O_thumbopt.json
uv run python peg_pinch.py run --hands O --draws logs/peg_tho_O.json --tag thoO36 --only 12 --workers 6
uv run python peg_pinch.py run --hands O --draws logs/peg_tho_O_thumbopt.json --tag thoT36 --only 12 --workers 6
uv run python peg_pinch.py run --hands O --draws logs/peg_tho_O.json --tag thoO36 --workers 6
uv run python peg_pinch.py run --hands O --draws logs/peg_tho_O_thumbopt.json --tag thoT36 --workers 6
uv run python peg_pinch.py run --hands O --draws logs/peg_tho_O.json --tag thoO36n --delta_deg 0.05 --workers 6
# T2
uv run python thumbopt_drake.py draws33b --hand O --out logs/pinch3_tho_O.json
uv run python thumbopt_drake.py draws33b --hand O_thumbopt --out logs/pinch3_tho_O_thumbopt.json
uv run python pinch_rotate.py run --hands O --draws logs/pinch3_tho_O.json --tag thoO33b --time_step 2.5e-4 --workers 6
uv run python pinch_rotate.py run --hands O --draws logs/pinch3_tho_O_thumbopt.json --tag thoT33b --time_step 2.5e-4 --workers 6
uv run python pinch_rotate.py run --hands O --draws logs/pinch3_tho_O.json --tag thoO33bn --delta_deg 0.05 --time_step 2.5e-4 --workers 6
```

Every run writes `logs/{peg,pinch}_O_<tag>.json` (the finger is O on both hands; the tag names the thumb). The T3 noise tag is `thoO36n`, and T2's is `thoO33bn`, as registered.
