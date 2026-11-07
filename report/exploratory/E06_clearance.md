# E06 — Collision-geometry inflation

**Status: PARTIALLY_SUPPORTED.**
- Under a literal uniform clearance, CoDe's lead collapses first: 35 → 0
  pairs at 0.25 mm.
- The cause is the single A pin–PIP servo contact found in E05.
- With pin housings exempted, CoDe loses **nothing** up to 1.0 mm.
- At 1.5 mm, CoDe and the belt 2R both fail on a static gap between the PIP
  servo and the MCP servo.

## 1. Question
Does the Study 1 ranking survive a non-zero body clearance?

## 2. Hypothesis and alternative
- **Hypothesis.** CoDe's lead shrinks fastest, because its feasibility
  relies on near-touching solids.
- **Alternative.** All architectures lose pairs at similar rates.

## 3. Model and assumptions
- **Clearance.** The shared `packaging.body_clearance_mm` is set to 0, 0.25,
  0.5, 1.0 and 1.5 mm for all references at once.
- **Pad seating.** The mounting policy ignores this clearance where the
  evaluators apply it, so the policy depth alone reads as INVALID. Each pad
  is therefore lowered in 0.05 mm steps, at most 2 mm, until the
  evaluator's own seat checks pass. The rule is the same for every
  mechanism.
- **Variant `pins_exempt`.** Pin-housing diameters are reduced by 2c,
  including the linkage's own `a/ap/b/bp_pivot_diameter_mm`, so that the
  clearance applies everywhere except at pin housings.

## Where the clearance enters

`body_clearance_mm` is read at the following places:

| Place | Mechanism |
|---|---|
| `core/optimization/revolute/evaluate.py:223` | 2R-DD |
| `core/optimization/revolute_parallel/evaluate.py:114` | belt 2R |
| `core/optimization/rcj/evaluate.py:107` | RCJ |
| `core/morphology/mechanism/revolute/solids.py:214` | 2R-DD |
| `core/morphology/mechanism/revolute_parallel/solids.py:73` | belt 2R |
| `core/morphology/mechanism/rcj/solids.py:187` | RCJ |
| `core/morphology/mechanism/linkage/packaging.py:380` | linkage: pin and pad gaps |
| `core/morphology/mechanism/linkage/packaging.py:423` | linkage: pin and pad gaps |
| `core/morphology/mechanism/linkage/motion.py:255` | linkage: driver-link clearance |
| `core/morphology/packaging/gaps.py`, `assembly.py` | servo and body gaps, all mechanisms |

The mounting policy (`core/optimization/mounting.py`) does not apply it.
That is the inconsistency the seat search works around.

## 4. Method
Catalog pairs carried at 120° and 150°. Script: `e06_clearance.py`. Figure:
`e06_clearance.png`.

## 5. Results (pairs carried at 120°; 150° in brackets)

| Reference | 0 mm | 0.25 | 0.5 | 1.0 | 1.5 | Pins exempt, 1.0 | Pins exempt, 1.5 |
|---|---|---|---|---|---|---|---|
| CoDe | 35 (28) | **0 (0)** | 0 | 0 | 0 | **35 (28)** | 0 |
| 2R-DD 0° | 17 (0) | 6 (0) | 6 | 6 | 6 | 6 | 6 |
| 2R-DD 90° | 31 (13) | 31 (13) | 26 | 26 | 26 (13) | 26 | 26 |
| Belt 2R | 33 (17) | 33 (17) | 33 (16) | 33 (16) | 0 | 33 (16) | 0 |
| RCJ 0° | 20 (5) | 0 | 0 | 0 | 0 | 0 | 0 |
| RCJ 90° | 32 (22) | 29 (21) | 27 | 25 | 23 (20) | 25 | 23 |
| *Ideal RCJ* | *34 (26)* | *34* | *34* | *34* | *34 (26)* | *34* | *34* |

**Causes of the cliffs.**

| Reference | Clearance | Cause |
|---|---|---|
| CoDe | 0.25 mm | `pivot_pin` |
| CoDe, belt 2R | 1.5 mm | `mcp_servo` (PIP servo against MCP servo) |
| RCJ 0° | 0.25 mm | `pad_drop` / seat; no seat within 2 mm |
| 2R-DD 0° | 0.25 mm | `no_depth` |

The joint-servo designs at 90° decline gradually, through seating and ROM.

## 6. Interpretation
- **Literally, the hypothesis holds.** CoDe is the most clearance-sensitive
  design under a uniform clearance.
- **Mechanically, it does not.** The sensitivity is a single static
  hardware contact in the prototype layout. The kinematic and contact
  results (fold, faces, ROM) of CoDe are unchanged up to 1 mm of
  clearance, while the RCJ at 90° and the 2R-DD at 90° each lose 5–9 pairs.

## 7. Critical review
- **An implementation inconsistency, recorded for repair.** The mounting
  policy does not apply `body_clearance_mm` while the evaluators do. That is
  a real inconsistency in the repository, not a property of any mechanism.
  The seat search works around it equally for all.
- **The exemption is not physical.** The pins-exempt variant is an
  ablation, and shows where clearance bites. A real design would instead
  move the A′ servo or shrink the pin housing.
- **Seat-search bias.** The seat search lowers both pads together, which is
  conservative for one side.

## 8. Follow-up
The `pins_exempt` variant above.

## 9. Limitations
- One morphology.
- A uniform clearance stands in for real tolerancing.
- 2D only.

## 10. Takeaway
CoDe's accommodation lead is robust to a 1 mm clearance on everything
except two static servo contacts in the prototype layout: the A pin to the
PIP servo, and the PIP servo to the MCP servo. The prototype should be
built with explicit clearance at both. The repository's mounting policy
should apply `body_clearance_mm`.
