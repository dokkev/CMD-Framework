#!/usr/bin/env python3
"""Write the compact-extension report from finalized machine-readable results."""
from pathlib import Path
import csv
import json
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SOURCE=ROOT/'output/study/study2_compact_morphology'
sys.path[:0]=[str(ROOT),str(HERE.parent)]
import shared

REFS=['code_matched','2r_dd_tilt0_matched','2r_dd_tilt90_matched','belt_2r_matched','rcj_tilt0_matched','rcj_tilt90_matched','rcj_ideal_matched']
NAMES=['CoDe','DD 0°','DD 90°','Belt 2R','RCJ 0°','RCJ 90°','Ideal RCJ']
TARGETS=['baseline','thick','long','long_thick','asym_long_proximal','asym_long_distal','asym_proximal_heavy','asym_distal_heavy']


def report():
    with(SOURCE/'summary_best.csv').open()as f:rows=list(csv.DictReader(f))
    lookup={(r['target_id'],r['reference_id']):r for r in rows}
    m=json.loads((SOURCE/'manifest.json').read_text())
    a=json.loads((SOURCE/'aggregation.json').read_text())
    with(SOURCE/'convergence.csv').open()as f:convergence=list(csv.DictReader(f))
    tail=[float(r['improvement_last_20_mm'])for r in convergence if r['improvement_last_20_mm']]
    improved=sum(v>.1+1e-8 for v in tail)
    max_tail=max(tail,default=0.)
    catalog=shared.catalog(SOURCE/'inputs')
    interfaces=['| Given interface | Proximal length × thickness (mm) | Distal length × thickness (mm) |','|---|---:|---:|']
    for target in TARGETS:
        c=catalog[target]
        interfaces.append(f'| {target} | {c.proximal_length_mm:g} × {c.proximal_thickness_mm:g} | {c.distal_length_mm:g} × {c.distal_thickness_mm:g} |')
    table=['| Given interface | '+' | '.join(NAMES)+' |','|---|'+'---:|'*7]
    extra=['| Given interface | '+' | '.join(NAMES)+' |','|---|'+'---:|'*7]
    for target in TARGETS:
        line=[];delta=[]
        for ref in REFS:
            r=lookup[(target,ref)]
            line.append(f"{float(r['minimum_length_min_mm']):.1f}"if r['found']=='True'else 'not found')
            delta.append(f"{float(r['delta_compact_baseline_finger_length_mm']):+.1f}"if r['found']=='True'and r.get('delta_compact_baseline_finger_length_mm')else '—')
        table.append('| '+target+' | '+' | '.join(line)+' |')
        extra.append('| '+target+' | '+' | '.join(delta)+' |')
    details=['| Target / reference | Selected L | ΔL | Δ proximal span | Δ distal span | Δ articulation | Seeds found |',
             '|---|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        if r['found']!='True':continue
        fields=[float(r[k])for k in ['final_length_mm','delta_finger_length_mm','delta_proximal_span_mm','delta_distal_span_mm','delta_articulation_mm']]
        details.append('| '+r['target_id']+' / '+r['reference_id']+' | '+' | '.join(f'{v:.1f}'for v in fields)+f" | {r['seeds_found']}/{r['seeds_run']} |")
    edge=[r for r in rows if r['found']=='True'and (r['length_upper_active']=='True'or r['length_lower_active']=='True')]
    missing=[r for r in rows if r['found']!='True']
    physical=[]
    for t in TARGETS:
        found=[lookup[(t,r)]for r in REFS[:-1]if lookup[(t,r)]['found']=='True']
        if found:
            lo=min(float(r['minimum_length_min_mm'])for r in found)
            winners=[r['reference_id']for r in found if float(r['minimum_length_min_mm'])<=lo+.1+1e-8]
            physical.append(f'- `{t}`: shortest physical best-found length {lo:.1f} mm; within 0.1 mm: '+', '.join(winners)+'.')
    lt={rid:lookup[('long_thick',rid)] for rid in REFS}
    successful=[r for r in rows if r['found']=='True']
    max_spread=max(float(r['minimum_length_max_mm'])-float(r['minimum_length_min_mm']) for r in successful)
    max_readback=max(abs(float(r['assembled_minus_descriptor_length_mm'])) for r in successful)
    interpretation=f"""The broad hypothesis that CoDe always finds the smallest accommodating finger is
not supported. Belt has the shortest retained physical design on seven of the
eight catalog targets; CoDe does on long_thick. On that target the shortest
nominal lengths found are CoDe {float(lt['code_matched']['minimum_length_min_mm']):.1f},
DD 0° {float(lt['2r_dd_tilt0_matched']['minimum_length_min_mm']):.1f},
DD 90° {float(lt['2r_dd_tilt90_matched']['minimum_length_min_mm']):.1f},
belt {float(lt['belt_2r_matched']['minimum_length_min_mm']):.1f} and physical RCJ
{float(lt['rcj_tilt90_matched']['minimum_length_min_mm']):.1f} mm. This is a conditional
size advantage, not general superiority. The ideal RCJ finds
{float(lt['rcj_ideal_matched']['minimum_length_min_mm']):.1f} mm under its no-joint-servo assumption.

Relative to their own selected compact baselines, long_thick adds
{float(lt['code_matched']['delta_compact_baseline_finger_length_mm']):.1f} mm for CoDe versus
{float(lt['belt_2r_matched']['delta_compact_baseline_finger_length_mm']):.1f} mm for belt.
Thus the architecture with the smallest baseline need not have the smallest
size increase under an interface change. This increment is descriptive of the
selected compact morphologies, not a causal decomposition of mechanical effects.

The expanded DD 0° domain finds long_thick at 120.7 mm, whereas the original
bounded Study2 did not find it. Its previous failure cannot be generalized beyond
that domain. Conversely, CoDe still finds no design for asym_long_proximal or
asym_proximal_heavy, at either upper limit and across all three seeds. Length
freedom alone therefore does not establish universal accommodation. Do not infer
an exact collision cause from these search failures.

Across successful cases the largest retained per-seed best-length range is
{max_spread:.1f} mm. The largest selected assembled/nominal length difference is
{max_readback:.4f} mm. These checks support the stated reporting precision, not a
proof that every feasible basin was found.

For the North Star, retain original Study2 as the direct test of morphology
preservation, and use this extension to show the size and shape price when
compact accommodation is the priority. A future comparison of interface effects
should use matched or explicitly controlled morphologies; these independently
compacted designs do not remove the morphology confound."""
    text=f'''# Study 2 extension: compact morphology for given contact interfaces

2026-11-07 · feature/2dof-fe · source HEAD `{m['source']['commit']}` plus recorded working-tree source snapshots.

## Question and relation to the North Star

For a fixed contact interface, what compact morphology can each architecture
find while preserving common articulation and contact requirements? Report how
its proximal/distal contact-bearing spans and articulation differ from a common
90 mm matched reference. This measures the spatial price of accommodation and
supports GOAL.md's aim of reducing interface-driven hand redesign and confounding.
It is not a manipulation-performance or hardware-buildability experiment.

The original Study2 allowed total length 70–110 mm and each external descriptor
within ±20 mm of the reference. Its results are preserved. The extension removes
that reference-proximity box and minimizes total length rather than eps_E first.

## Given interfaces

'''+ '\n'.join(interfaces)+'''

## Method and fair comparison

Seven original matched references, eight catalog targets, NSGA-II population100,
100 generations, seeds0/1/2. Every target/reference is optimized, even when its
matched reference carries the interface unchanged. All use required flexion120°,
existing mounting/servo/pivot/pad/face checks and existing E/Q/A partitions.
Interface dimensions are fixed. Architecture-specific actuation placement rules
and servo orientations are held; the remote shaft stays in the proximal frame,
and a joint-mounted servo follows its joint. No independent actuator relocation.

Primary objective: minimum absolute extended finger length. Secondary: minimum
maximum proximal-span, distal-span and articulation change (eps_shape)
within primary best-found length +0.1 mm. Total length is excluded from this
secondary norm because its fixed change would mask smaller shape changes.
The original full four-descriptor eps_E is reported separately. Pool
retained primary/secondary incumbents across seeds, then select minimum eps_shape
under the pooled best length +0.1 mm cap (sorted absolute shape changes break
retained-candidate ties, then length and eps_Q). The shortest actual secondary discovery
also counts toward the pooled length. Diagnostic secondary runs that included total length in the max norm were
retired, as that term masked shape changes. Valid primary searches were reused
after exact config/settings checks, and only secondary searches reran.
Selected designs are re-evaluated using
their saved stage configs. eps_Q is reported separately, never added to eps_E.

The common length domain starts [40,160] mm; a failed search or an upper-active
best triggers [40,240] mm. Support/span and dependent mechanism bounds propagate.
Belt shaft-to-joint dx covers shaft_x minus the permitted joint-position domain:
the former [-20,30] mm relative sampling box is not a physical belt-length limit.
Pulley geometry and all collision conditions remain. Other declared hardware and
mechanism layout bounds remain; this is not unrestricted physical design.

Initial baseline optimizer fronts from original Study2 provide deterministic warm
starts where present. Exact vectors/source IDs and per-stage configs/seeds are
saved. Main population100 admits all up-to40 recorded warm vectors; half of every
population remains exploration. See the study README for pilot admission limits,
source snapshots and warm replay. No new unrelated random design sampler.

The general nondominated archive has an exact scalar fast path: every minimum
scalar-objective row, retaining ties. Tests compare it to the original algorithm.
This changes runtime only. Early diagnostic belt trials with the old relative
box are retired under the result directory and excluded from the final dataset.
The original bounded Study2 also retained that relation box; its belt failures
therefore cannot establish an architecture-wide impossibility. A new feasible
result must be attributed to the expanded, consistently propagated domain, not
to removal of the total-length limit alone.

## Results: compact size

All numbers below are **best found nominal design-descriptor lengths**, not
certified global minima, in mm. They use the existing Study2 partition and
0.1 mm grid. CoDe link-length rounding/reassembly can slightly move the evaluated
fingertip; finalized CSVs separately retain assembled-length readback and its
difference. Do not interpret sub-grid rankings as physical precision. Ideal
RCJ removes the joint servo under the existing model and is a separate upper-bound
comparison. It is not a physical packaged competitor.

'''+ '\n'.join(table)+f'''

There are {len(rows)-len(missing)}/{len(rows)} target/reference combinations with
at least one feasible retained design. Domain-edge-active selected solutions:
{len(edge)}. A failed search is not an impossibility result.

Physical best-found comparisons, without converting size into a universal merit:

'''+ '\n'.join(physical)+'''

![Best-found lengths and compact-baseline increments](figures/study2_compact/compact_lengths.png)

The matched 90 mm reference is not a compactness optimum. Therefore shortening
relative to 90 mm includes ordinary baseline redesign, not just interface effects.
The following increments compare selected target designs against each
architecture's own selected compact baseline (also within the 0.1 mm length cap):

'''+ '\n'.join(extra)+'''

## Morphology changes

Spans are common contact-bearing support lengths, not identical anatomical
phalanx links across mechanisms. Articulation is the extended ICR for CoDe, pin
for 2R and roller contact for RCJ. The selected design can be up to 0.1 mm longer
than the shortest retained design to reduce external morphology change.

![Span and articulation changes](figures/study2_compact/compact_morphology_changes.png)

![Independent search seeds](figures/study2_compact/compact_seed_spread.png)

![Same-scale geometry](figures/study2_compact/compact_geometry.png)

All changes in this readback table are from the matched reference, in mm:

'''+ '\n'.join(details)+f'''

## Interpretation and limits

{interpretation}

Compactness and morphology preservation are different objectives. This experiment
answers the most compact accommodation question first, then reduces support/articulation
change among retained near-shortest designs. It does not establish the smallest
possible change to an existing hand; that was the original Study2's objective.
The compact-baseline increments help distinguish interface demand from generic
compaction, but different optimized morphologies still confound dynamics.

Every comparison remains conditional on current fixed-actuation, mounting,
2D collision/contact-face rules and finite domains. More total length need not
resolve a local fold, inaccessible contact face or fixed actuator-layout issue.
Neither successful search nor failed search is a mechanical acceptance test.
Production details, strength, width, cable retention and practical module
replacement are not established. Ideal RCJ remains separately labeled.

Three optimizer seeds assess search variability; candidate designs are not
statistical replicates. A single-objective archive can contain many tied designs.
The secondary search's cap is set per seed; pooled selection compares retained
incumbents under the common cap and does not prove a global lexicographic optimum.
Seed spread and the bound flags must accompany any smallest-size claim.
Primary-search histories are saved in `convergence.csv`: {improved}/{len(tail)}
feasible primary traces improved by more than 0.1 mm during generations81–100;
the largest such late improvement was {max_tail:.3f} mm. These traces describe
primary-search convergence, not the additional secondary discoveries or a
global optimality certificate.

## Verification and reproducibility

The original bounded Study2 dataset is unchanged. Focused compact/domain,
actuator-preservation, serialization, length-cap, pooled-selection and scalar-front
tests plus existing Study1/2 and transmission regression tests: **72 focused and regression tests passed** (including archived warm replay,
shape-objective masking and primary-stage reuse).
One independent read-only reviewer found the cross-seed primary-candidate loss;
it was fixed and regression-tested. The same reviewer verified the aggregation,
belt-domain and scalar-front corrections. Selected-design achieved ROM and bound
flags are refreshed from actual saved-config feasibility readback, addressing its
remaining metadata caveat. Verified selected designs: {a['verified_selected']}.

Results: `output/study/study2_compact_morphology/` contains per-seed stages,
exact warm vectors, source snapshots, inputs, per-seed and finalized best tables,
and aggregation provenance. Figures read the finalized tables and saved configs;
figure manifests contain input hashes and source snapshots. Configs and full
formulation: `scripts/study/study2_compact_morphology/README.md`.

```
python scripts/study/study2_compact_morphology/run.py [fresh-output-config.yaml]
python scripts/study/study2_compact_morphology/finalize.py [output-directory]
python scripts/study/study2_compact_morphology/figures.py --source [output-directory]
```

For exact warm replay, point `warm_start.replay` to the archived `targets/`
directory and choose a fresh output. No commit, push or hardware experiment.
'''
    path=ROOT/'report/study2_compact_2026-11-07.md';path.write_text(text)
    print(path)


if __name__=='__main__':report()
