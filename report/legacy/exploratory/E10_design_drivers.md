# E10 — What drives accommodation inside CoDe's design space

**Status: PARTIALLY_SUPPORTED, refined.**
- The *distance* the ICR travels does not predict capacity once the spans
  are controlled for.
- How far the ICR **drops palmar** by 120° does predict it.
- |AB| cannot be tested with this archive.

## 1. Question
Among feasible CoDe designs, which geometric features predict accommodation
capacity?

## 2. Hypothesis and alternative
- **Hypothesis.** ICR travel over 0–120° predicts capacity better than
  |AB|, the spans or the servo position.
- **Alternative.** The spans alone explain capacity.

## 3. Model and assumptions
- **Archive.** 22,678 archived linkage-v12 production designs, run_001–009.
  They were optimized for transmission, coverage and curl with |OC| 80–90
  mm, so they are a biased sample.
- **Cover.** A deterministic farthest-point cover of 150 designs in
  normalized design space, using the Study 3 `cover`.
- **Judging.** Each design is frozen under the Study 1 rules (policy depths)
  and judged on the 71-pair catalog at 120°.
- **Correlations.** Spearman, plus partial correlations with both spans as
  controls. Bootstrap intervals use 2000 resamples of the cover; they
  describe resampling of this cover, not a population.

## 4. Method
Script: `e10_design_drivers.py`, then `e10b_bootstrap.py`. Figure:
`e10_design_drivers.png`.

## 5. Results
Pairs carried: minimum 9, median 30, maximum 36. 146 of the 150 designs
carry the baseline module.

| Feature (sample range) | Spearman | Partial, spans controlled | 95% bootstrap interval |
|---|---|---|---|
| ICR palmar drop, 0→120° (4.6–11.9 mm) | +0.32 | **+0.44** | [0.27, 0.57] |
| ICR height at 120° (−7.7 to +0.7 mm) | −0.47 | **−0.33** | [−0.49, −0.17] |
| ICR travel distance (13.7–21.0 mm) | +0.53 | −0.14 | [−0.26, −0.01] |
| Extended ICR share (0.53–0.72) | +0.49 | −0.02 | [−0.11, 0.08] |
| \|AB\| (30.7–31.5 mm) | −0.06 | +0.01 | [−0.16, 0.17] |
| A′ height (11.8–13.0 mm) | +0.45 | +0.29 | [0.10, 0.44] |
| BC (13.8–26.4 mm) | −0.25 | +0.28 | [0.15, 0.39] |
| Proximal span | +0.59 | — | — |
| Distal span | +0.40 | — | — |

## 6. Interpretation
- **The spans are the first-order driver**, as expected: they set how much
  module fits.
- **The ICR's raw correlations come from the spans.** Raw ICR travel and
  ICR0 correlate with capacity only because they co-vary with the spans.
  Controlled, ICR0 carries no signal and ICR travel is weakly
  negative (−0.14, interval [−0.26, −0.01]).
- **The ICR's palmar drop carries signal beyond the spans.** So does a lower
  ICR at 120°. This is an independent, correlational replication of E02's
  ablation result: what matters is that the virtual pivot gets close to the
  contact faces, not that it moves far.
- **|AB| is untestable here.** The archive pins it to 0.35·|OC| (a 0.8 mm
  range). This dataset cannot answer whether a shorter |AB| hurts.

## 7. Critical review
- The archive was optimized under other objectives, so it is a selected
  sample. These are correlations within it.
- Designs are not independent samples; the bootstrap describes this cover
  only.
- A′ height and BC are also partially associated with capacity. They may
  act through the ICR geometry; that is untested.

## 8. Follow-up
E10b bootstrap. A targeted |AB| sweep would need new optimization (the
ε-constraint idea discussed with the user), out of this study's scope.

## 9. Limitations
- A biased sample, one formulation version (v12), 2D, 120° only.

## 10. Takeaway
Within CoDe's own design space, accommodation capacity follows the spans
first. Beyond the spans, it follows how low the virtual pivot goes at high
flexion, not how far it travels. This agrees with E02 by a different
method. **For design, aim for a deep virtual pivot at the target flexion,
not a long ICR path.**
