#!/usr/bin/env python3
"""Compact accommodation extension. Does not overwrite original Study 2 results."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import shutil
import sys
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path[:0]=[str(ROOT),str(HERE.parent)]
import shared
from core.optimization.analysis.adaptation import DESCRIPTORS, save_step
from core.optimization.analysis.compact import search_compact
from core.optimization.analysis.tradeoff import with_modules
from core.optimization.mechanism import mechanism, read_config_file
from core.optimization.optimizer import OptimizerSettings
from core.optimization.study import load_run


def warm_designs(reference, target, cfg, out, inputs):
    """Re-use prior optimizer fronts from the same target/reference, all seeds.

    Include the baseline design (not assumed feasible). Save selected vectors,
    source IDs and original metadata/hashes, so untracked old output is not the
    only surviving source of warm starts. No synthetic random design sampler.
    """
    if cfg['warm_start'].get('replay'):
        source=ROOT/cfg['warm_start']['replay']/target.module_id/reference.reference_id/'warm_start.json'
        data=json.loads(source.read_text())
        destination=out/'inputs'/'warm_replay'/target.module_id/reference.reference_id/'warm_start.json'
        destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,destination)
        inputs[str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
        return [tuple(item['x'])for item in data],data
    part=shared.partitions.partition(reference.mechanism)
    candidates=[(reference.x,'matched_reference')]
    old=ROOT/cfg['warm_start']['study2']/'targets'/target.module_id/reference.reference_id
    for path in sorted(old.glob('seed_*/*/run.json')):
        run=load_run(path.parent)
        if run.mechanism.name!=reference.mechanism:
            raise ValueError('warm source mechanism mismatch')
        rows=run.candidates()
        vectors=[tuple(run.mechanism.read_design(r))for r in rows]
        # Keep source metadata and only the selected numerical vectors below;
        # full old fronts are not needed to reproduce the new optimizer run.
        rel=str(path.relative_to(ROOT))
        inputs[rel]=hashlib.sha256(path.read_bytes()).hexdigest()
        dst=out/'inputs'/path.relative_to(ROOT)
        dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,dst)
        if vectors:
            ranked=sorted(zip(vectors,rows),key=lambda v:(part.external(v[0],reference.config)['finger_length_mm'],
                       part.external(v[0],reference.config)['proximal_span_mm']))
            count=min(8,len(ranked))
            indices=sorted(set([0]+[round(i*(len(ranked)-1)/max(1,count-1))for i in range(count)]))
            candidates.extend((ranked[i][0],f'{path.parent.relative_to(ROOT)}/pareto.csv#{ranked[i][1]["candidate"]}')for i in indices)
    unique={}
    for x,source in candidates:
        unique.setdefault(tuple(x),source)
    vectors=list(unique)[:cfg['warm_start']['max_designs']]
    return vectors,[dict(x=list(x),source=unique[x])for x in vectors]


def design_metrics(row, x, stage):
    """Both saved primary and secondary incumbents remain eligible globally."""
    part=shared.partitions.partition(row['mechanism'])
    e=part.external(x,None)
    e0={k:float(row[f'reference_{k}'])for k in DESCRIPTORS}
    reference=json.loads(row['reference_x'])
    q,q0=part.mechanism(x,None),part.mechanism(reference,None)
    scale=e0['finger_length_mm']
    return {**row,'x':json.dumps(x),'selected_stage':stage,
        **({'phi_max_deg':None,'length_upper_active':None,'length_lower_active':None}if stage=='minimum_length'else {}),
        'final_length_mm':e['finger_length_mm'],
        'external_cost':max(abs(e[k]-e0[k])for k in e0)/scale,
        'mechanism_cost':max((abs(q[k]-q0[k])for k in q0),default=0.)/scale,
        'shape_cost':max(abs(e[k]-e0[k])for k in e0 if k!='finger_length_mm')/scale,
        **{f'final_{k}':v for k,v in e.items()},
        **{f'delta_{k}':e[k]-e0[k]for k in DESCRIPTORS}}


def best_rows(rows,tolerance):
    groups={}
    for r in rows:groups.setdefault((r['target_id'],r['reference_id']),[]).append(r)
    best=[]
    for key,g in groups.items():
        found=[r for r in g if r['found'] is True or r['found']=='True']
        candidates=[]
        for r in found:
            candidates.append(design_metrics(r,json.loads(r['minimum_length_x']),'minimum_length'))
            candidates.append(design_metrics(r,json.loads(r['x']),'minimum_change'))
        if candidates:
            shortest=min(r['final_length_mm']for r in candidates)
            eligible=[r for r in candidates if r['final_length_mm']<=shortest+tolerance+1e-8]
            chosen=min(eligible,key=lambda r:(r['shape_cost'],
                tuple(sorted((abs(r[f'delta_{k}'])for k in DESCRIPTORS if k!='finger_length_mm'),reverse=True)),
                r['final_length_mm'],r['mechanism_cost']))
            seed_minima=[min(c['final_length_mm']for c in candidates if c['seed']==r['seed'])for r in found]
        else:
            shortest=None;chosen=g[0];seed_minima=[]
        best.append({**chosen,'seeds_found':len(found),'seeds_run':len(g),
            'minimum_length_min_mm':shortest,
            'minimum_length_max_mm':max(seed_minima,default=None)})
    return best


def run(path,only_targets=None,only_refs=None,resume=False):
    cfg=read_config_file(path);out=ROOT/cfg['output']
    if out.exists() and not resume:raise FileExistsError(f'choose a fresh output: {out}')
    out.mkdir(parents=True,exist_ok=resume)
    refs=[r for r in shared.load_references()if r.reference_id in (only_refs or cfg['references'])]
    catalog=shared.catalog();targets=[catalog[n]for n in (only_targets or cfg['targets'])]
    rules=shared.check_comparable(refs)
    manifest=dict(warm_population_admission='inherited optimizer admits first min(recorded warm count,population//2); secondary prepends primary winner',study='study2_compact_morphology',config=cfg,source=None if resume else shared.git_commit(out),
        comparison_rules=rules,formulations=shared.formulations(refs),inputs={},
        run_filters=dict(targets=[t.module_id for t in targets],references=[r.reference_id for r in refs]),
        objective='min absolute extended length; then min external change within length tolerance',
        claim='best-found; no global minimum certificate; physical and ideal references separate')
    for p in [Path(path).resolve(),ROOT/shared.REFERENCES,ROOT/shared.CATALOG,ROOT/'GOAL.md']:
        dst=out/'inputs'/p.relative_to(ROOT)
        if resume and dst.exists():continue
        dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dst)
        manifest['inputs'][str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
    rows=[]
    if resume:
        prior=json.loads((out/'manifest.json').read_text())
        if prior['config']!=cfg or prior['run_filters']!=manifest['run_filters']:
            raise ValueError('resume config/filter mismatch')
        segment=out/'resume_source'
        segment.mkdir(exist_ok=True)
        manifest={**prior,'resume_source':shared.git_commit(segment)}
        with(out/'summary.csv').open()as f:rows=list(csv.DictReader(f))
        for row in rows:
            row['found']=row['found']=='True'
    completed={(r['target_id'],r['reference_id'],int(r['seed']))for r in rows}
    for target in targets:
        for ref in refs:
            part=shared.partitions.partition(ref.mechanism);base=mechanism(ref.mechanism)
            e0=part.external(ref.x,ref.config)
            carried=shared.with_required_flexion(with_modules(ref.config,target.modules),cfg['required_flexion_deg'])
            warm,provenance=warm_designs(ref,target,cfg,out,manifest['inputs'])
            pair=out/'targets'/target.module_id/ref.reference_id;pair.mkdir(parents=True,exist_ok=resume)
            (pair/'warm_start.json').write_text(json.dumps(provenance,indent=2))
            for seed in cfg['optimizer']['seeds']:
                if (target.module_id,ref.reference_id,seed)in completed:continue
                started=time.monotonic();settings=OptimizerSettings('nsga2',cfg['optimizer']['population'],cfg['optimizer']['generations'],seed)
                attempts=[];final=None;primary=None;secondary_status='not_run';selected_upper=None
                attempt_warm=list(warm)
                for upper in cfg['length_upper_sequence_mm']:
                    print(f'{target.module_id} / {ref.reference_id} / seed {seed} / upper {upper}',flush=True)
                    primary_path=pair/f'seed_{seed}'/f'upper_{upper}'/'minimum_length'
                    steps=search_compact(base,part,ref.x,carried,settings,(cfg['length_lower_mm'],upper),attempt_warm,
                                        cfg['workers'],cfg['length_tolerance_mm'],
                                        primary_run=primary_path if resume and (primary_path/'run.json').exists()else None)
                    for s in steps:
                        if s.result is not None:save_step(s,pair/f'seed_{seed}'/f'upper_{upper}'/s.level,source_git_commit=manifest['source']['commit'])
                    found=[s for s in steps if s.status=='found']
                    attempts.append(dict(upper_mm=upper,status=[s.status for s in steps]))
                    if not found:continue
                    primary=found[0];final=found[-1];secondary_status=steps[-1].status;selected_upper=upper
                    if min(s.evaluation.external['finger_length_mm']for s in found)<upper-cfg['upper_margin_mm']:break
                    attempt_warm=[s.x for s in found]+attempt_warm
                row=dict(target_id=target.module_id,reference_id=ref.reference_id,mechanism=ref.mechanism,
                    upper_bound=ref.upper_bound,seed=seed,found=final is not None,attempts=json.dumps(attempts),
                    secondary_status=secondary_status,elapsed_seconds=time.monotonic()-started,
                    reference_x=json.dumps(ref.x),selected_upper_mm=selected_upper,minimum_length_mm=None,final_length_mm=None)
                if final:
                    e=final.evaluation.external
                    row.update(minimum_length_mm=primary.evaluation.external['finger_length_mm'],
                        final_length_mm=e['finger_length_mm'],external_cost=final.evaluation.external_cost,
                        shape_cost=max(abs(e[k]-e0[k])for k in DESCRIPTORS if k!='finger_length_mm')/e0['finger_length_mm'],
                        mechanism_cost=final.evaluation.mechanism_cost,x=json.dumps(final.x),
                        minimum_length_x=json.dumps(primary.x),
                        length_upper_active=e['finger_length_mm']>=selected_upper-cfg['upper_margin_mm'],
                        length_lower_active=e['finger_length_mm']<=cfg['length_lower_mm']+cfg['upper_margin_mm'],
                        phi_max_deg=__import__('math').degrees(final.evaluation.base.phi_max_rad))
                    row.update({f'reference_{k}':v for k,v in e0.items()})
                    row.update({f'final_{k}':v for k,v in e.items()})
                    row.update({f'delta_{k}':e[k]-e0[k]for k in DESCRIPTORS})
                rows.append(row)
                manifest['objective']='min nominal total length; then min max change of proximal/distal support spans and articulation; full eps_E reported separately'
                shared.write_csv(out/'summary.csv',rows)
                shared.write_csv(out/'summary_best.csv',best_rows(rows,cfg['length_tolerance_mm']))
                (out/'manifest.json').write_text(json.dumps(manifest,indent=2,default=str))
                print(f'  found={row["found"]}, minimum={row["minimum_length_mm"]}, final={row["final_length_mm"]}',flush=True)
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('config',nargs='?',type=Path,default=HERE/'config.yaml')
    p.add_argument('--targets');p.add_argument('--references');p.add_argument('--resume',action='store_true')
    a=p.parse_args();run(a.config,a.targets.split(',')if a.targets else None,a.references.split(',')if a.references else None,a.resume)
