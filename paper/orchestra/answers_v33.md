# Data answers to v33 review points (for the manuscript session)

Written 2026-11-23 from the experiment records; no manuscript file was changed. Each answer gives its source.

## R and U under touch re-zeroing (r2: "The paper does not report what R and U score under re-zeroing")

Source: ninth experiment, condition E (`docs/experimental/separability.md`, "9차 결과";
`sim/isaaclab/logs/experimental/two_pad/dose_E_all.json`).

| Pad | R, reused setpoint (A) | R, re-zeroed at touch (E) | U, reused setpoint (A) | U, re-zeroed at touch (E) |
|---|---|---|---|---|
| 5 mm | 3% | 6% | 52% | 9% |
| 7 mm | 21% | 23% | 124% | 20% |
| 9 mm | 41% | 22% | 106% | 20% |

Under re-zeroing, R and U err alike (6–23% against 9–20%). The redesigns that err most are those that moved the
skeleton most (d4p5, d5p5 61%; h3to9_d3p5_maxDs 42%). Possible sentence:

> Re-zeroed at touch, $R$ and $U$ err alike ($6$--$23\%$ and $9$--$20\%$); the largest errors ($42$--$61\%$) come from the redesigns that move the skeleton most.

## Hand counts in Table I (r2: "15, 14 differ from the 18 stated")

Source: `dose_A_all.json` (15 hands) and `comp.json` (14 hands); separability.md, results of the fifth and sixth experiments.
Under the reused setpoint, three redesigns cannot hold the cylinder at any shared placement with friction 1.3:
d4p5_s0, d5p5_s0 and h3to5_d3_s0 (Δ_contact 4.41, 5.40, 2.92 mm). A fourth one cannot hold with the stiffness change.
Scored as the largest errors, they give ρ = 0.81 (friction, 18 hands) and 0.67 (stiffness).
Under re-zeroing (E) all 18 hold. Possible caption clause:

> Hands: those that hold the cylinder under both settings ($3$ of $18$ cannot with the friction change, $4$ with the stiffness change).

## Pre-registration (r2: "cannot be checked")

Each experiment's plan was committed to `docs/experimental/separability.md` before its data. Examples:
- experiment 25: plan commit a280bef, then the small sample, then the full run;
- experiment 26: plan commit made before any hand other than O was run.
The repository history therefore dates every plan. If the paper can point to a public log or a supplement, this is the
record to cite.

## Pads as soft as they are thick (r1, r2 earlier: "never modelled as thick")

Source: experiment 25 (`sim/drake/logs/e25_hydro_own.json`).
- ρ(E′, Δ_contact) = 0.78 (p 0.0003, 15 hands); Δ_skel −0.10.
- R: 5.9, 16, 104% at 5, 7, 9 mm. U: 109, 347, 370%.
- Noise floor 2.4%.
This is already in the manuscript as of e56892b. It is worth checking that it survived the later rewrites.

## Room claim mostly from remote actuation (r2)

Source: Fig. 3a–c data. 2R-remote meets or beats the CFL at 90° and at most thicknesses at 120°. The CFL separates at
150° and at the thickest pads; at 150°, 2R-remote's 7 and 9 mm designs cannot hold a cylinder. The contribution bullet
could split the two: remote actuation gives the room, and the virtual pole keeps it as flexion deepens.
