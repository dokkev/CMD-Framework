# Recheck after the fold-rounding fix (2026-11-17)

**Study track.** Code `d89960b`. Every run below is in `design/output/study/recheck_d89960b/`, together with the
scripts and configs used (`scripts/`). The original runs are untouched.

## What was wrong

With `goal.loss.links_block_face` on (the study's rule), the occlusion step re-read the fold limit on the probe's
0.5 mm pieces. The fold limit is where the two contact faces cross when the finger is flexed. Re-reading it this
way rounded the usable length of a face by up to ±0.25 mm. This affected CoDe and the belt 2R, the two mechanisms
the rule covers. The 2R-DD and the RCJs were not affected.

Since `9115e3f` and `d89960b`, an unblocked face keeps exactly its fold length. Turning the rule off can no longer
shorten a face (`study1_faceoff_2026-11-17.md`).

## Method

- **Deterministic experiments** (Study 1 and the interface family) were rerun in full and compared assembly by
  assembly. Only the usable lengths changed (by ≤ 0.25 mm), so every verdict change below is the fix's.
- **Searches** (ROM-conditioned pilot, transmission sensitivity, the 2026-11-06 Study 2) were checked in two ways.
  First, the saved selected designs were re-evaluated. Then the CoDe and belt 2R searches were rerun with the same
  config and seeds, against the same saved baselines where there are any. A changed evaluation can steer a GA
  elsewhere, so a rerun differs from the original by more than the fix alone. For one-seed pilots this is mostly
  search noise.

## Results

| Experiment | What changed | Effect on stated results |
|---|---|---|
| Study 1, active 15° (2026-11-17) | Belt 2R 37/33/17 → 36/31/13 module pairs at 90/120/150° | Already in `study1_faceoff_2026-11-17.md` |
| Interface family (2026-11-09) | 34 CoDe and 97 belt 2R assemblies feasible → infeasible (out of 62,208) | **C7 unchanged**: dense 28/144 and fold-off 106/144 for CoDe, every dense number identical. Result 1 and Result 3 tables change for CoDe and belt (below) |
| ROM-conditioned pilot (2026-11-09) | 5 of 145 saved designs now short by 0.03–0.25 mm. Rerun: CoDe successes 8/5/2 (unchanged); belt 2R at 90° 7 → 6 | **C3, C4 unchanged.** CoDe still carries both long proximal targets at 90° with a Q change. Per-condition costs moved a lot (one-seed pilot) |
| Transmission sensitivity (2026-11-09) | The one face-rule-ON solution (main_margin10, asym_long_proximal, 1/3 seeds) is still feasible under the current code, but the rerun did not find it (0/3). Every other ON run: not found, as before | "A face-rule-ON solution exists" still holds through the saved design. Finding it is fragile. The face-off control is unaffected (rule off) |
| Study 1, 10° (2026-11-06, historical) | Belt 2R 37/33/17 → 36/31/13; CoDe unchanged | Report numbers for the belt 2R are high by 1/2/4 |
| Study 2, 10° (2026-11-06, historical) | All 3 saved CoDe long_thick designs short by 0.11–0.25 mm. Rerun: found by 3/3 seeds, excess 0.060–0.063 (was 0.054–0.088) | "CoDe is the real architecture closest to the floor on long_thick" holds, at excess 0.060. Every other CoDe and belt 2R outcome unchanged |

### Interface family tables, corrected

Result 1, length 20/20 mm, thickness pairs supported (/144):

| Reference | 90° endpoint | 120° endpoint | 150° endpoint | All initial sampled poses ≤ 150° |
|---|---:|---:|---:|---:|
| CoDe 15° | 132 → **131** | 117 → **114** | 95 | 54 |
| Belt 15° | 120 → **108** | 82 → **77** | 25 → **24** | 23 → **22** |

Other references are unchanged. At 90° endpoint CoDe (131) is now one below the physical RCJ at 90° (132); they
were tied. The report's claim concerns 120° and 150°, where CoDe still leads.

Result 3, sum over the six length strata, initial sampled poses (/864):

| Reference | 90° | 120° | 150° |
|---|---:|---:|---:|
| CoDe 15° | 301 → **299** | 238 → **236** | 154 → **152** |
| Belt 15° | 273 → **247** | 194 → **186** | 93 → **90** |

Length 30/15 mm at 150°: belt 25 → **23**. Without the rounding, the belt 2R's interface-family results equal the
2R-DD at 90°'s in Result 1. Both share the same 2R kinematics.

## What this means

- None of the current claims in PROGRESS.md (C3, C4, C7) changes.
- The belt 2R had been credited with up to 12 extra thickness pairs (interface family) and 4 extra module pairs
  (Study 1). Every comparison with the belt 2R is now slightly more favourable to the others.
- Optimizer results that sit on a constraint boundary move when the boundary moves by 0.25 mm. One-seed pilot costs
  should not be quoted as stable numbers. Rerunning a search changes them more than this fix does.

Reproduce from `design/`, with the scripts in `output/study/recheck_d89960b/scripts/` (they take paths relative to
`design/`).
