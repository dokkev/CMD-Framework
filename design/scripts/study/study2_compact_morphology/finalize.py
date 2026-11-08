#!/usr/bin/env python3
"""Select both-stage incumbents across seeds, verify, add baseline-relative costs."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path[:0]=[str(ROOT),str(HERE.parent),str(HERE)]
import shared
from run import best_rows
from core.optimization.study import load_run
from core.optimization.mechanism import mechanism


def finalize(out):
    (out/'aggregation_source').mkdir(exist_ok=True)
    with (out/'summary.csv').open()as f:rows=list(csv.DictReader(f))
    manifest=json.loads((out/'manifest.json').read_text())
    cfg=manifest['config']
    expected=len(manifest['run_filters']['targets'])*len(manifest['run_filters']['references'])*len(cfg['optimizer']['seeds'])
    if len(rows)!=expected:raise ValueError(f'incomplete: {len(rows)}/{expected} seed results')
    best=best_rows(rows,cfg['length_tolerance_mm'])
    for r in best:
        if not (r['found']is True or r['found']=='True'):continue
        pair=out/'targets'/r['target_id']/r['reference_id']/f'seed_{r["seed"]}'
        found=[]
        for attempt in json.loads(r['attempts']):
            if attempt['status'][0]=='found':found.append(attempt['upper_mm'])
        upper=found[-1]
        stage=r['selected_stage']
        path=pair/f'upper_{upper}'/stage
        saved=load_run(path)
        if stage=='minimum_change' and not saved.candidates():
            stage='minimum_length';path=pair/f'upper_{upper}'/stage;saved=load_run(path)
        x=json.loads(r['x'])
        e=saved.mechanism.evaluate(x,saved.config)
        if e is None or not e.feasible:raise ValueError(f'aggregated design is not feasible: {path}')
        e0=shared.partitions.partition(r['mechanism']).external(x,saved.config)
        if abs(e0['finger_length_mm']-r['final_length_mm'])>1e-7:raise ValueError('length readback mismatch')
        if r['final_length_mm']>r['minimum_length_min_mm']+cfg['length_tolerance_mm']+1e-8:
            raise ValueError('global length cap mismatch')
        r['selected_stage']=stage;r['selected_run']=str(path.relative_to(ROOT))
        r['selected_upper_mm']=upper
        readback=saved.mechanism.report(e)
        r['phi_max_deg']=readback['phi_max_deg']
        r['assembled_length_mm']=readback['finger_length_mm']
        r['assembled_minus_descriptor_length_mm']=r['assembled_length_mm']-r['final_length_mm']
        r['length_upper_active']=r['final_length_mm']>=upper-cfg['upper_margin_mm']
        r['length_lower_active']=r['final_length_mm']<=cfg['length_lower_mm']+cfg['upper_margin_mm']
        r['final_config']=json.dumps(saved.mechanism.config_to_json(saved.config))
    baseline={r['reference_id']:r for r in best if r['target_id']=='baseline'and (r['found']is True or r['found']=='True')}
    for r in best:
        if not (r['found']is True or r['found']=='True')or r['reference_id']not in baseline:continue
        b=baseline[r['reference_id']]
        for field in ('finger_length_mm','proximal_span_mm','distal_span_mm','articulation_mm'):
            r[f'delta_compact_baseline_{field}']=float(r[f'final_{field}'])-float(b[f'final_{field}'])
    shared.write_csv(out/'summary_best.csv',best)
    convergence=[]
    for path in sorted((out/'targets').glob('*/*/seed_*/upper_*/minimum_length/history.npz')):
        with np.load(path)as history:
            generation,objective=history['generation'],history['objectives'][:,0]
            values={n:float(objective[generation<n].min())if np.any(generation<n)else None
                    for n in (20,50,80,100)}
            convergence.append(dict(run=str(path.parent.relative_to(ROOT)),
                history_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                **{f'best_length_after_{n}_generations_mm':v for n,v in values.items()},
                improvement_last_20_mm=None if values[80]is None or values[100]is None else values[80]-values[100]))
    shared.write_csv(out/'convergence.csv',convergence)
    record=dict(input_summary_sha256=hashlib.sha256((out/'summary.csv').read_bytes()).hexdigest(),
        aggregation='pool saved primary and secondary winners across seeds; retain L <= pooled best L + tolerance; min support/articulation max change; sorted absolute shape changes then L then eps_Q',
        source=shared.git_commit(out/'aggregation_source'),verified_selected=sum('final_config'in r for r in best),
        primary_domains='finite common domains; existing hardware and actuator layout bounds preserved',
        reference='matched 90mm morphology; compact baseline differences reported separately')
    (out/'aggregation.json').write_text(json.dumps(record,indent=2))
    print(json.dumps(dict(rows=len(rows),comparisons=len(best),found=sum('final_config'in r for r in best))))
    return best


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',nargs='?',type=Path,default=ROOT/'output/study/study2_compact_morphology')
    args=p.parse_args();(args.output/'aggregation_source').mkdir(exist_ok=True);finalize(args.output)
