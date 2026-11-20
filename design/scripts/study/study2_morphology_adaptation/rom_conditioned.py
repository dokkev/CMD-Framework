#!/usr/bin/env python3
"""Study 2 with ROM-feasible baselines: nominal -> ROM baseline -> interface adaptation.

Reuse the existing adapt hierarchy. Each target and each ROM has an independent search.
The selected baseline is frozen before any non-baseline target is examined.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path[:0] = [str(ROOT), str(HERE.parent)]
import shared
from core.optimization.analysis.adaptation import adapt, save_step, config_resolution, change
from core.optimization.analysis.tradeoff import with_modules
from core.optimization.mechanism import mechanism, read_config_file
from core.optimization.optimizer import OptimizerSettings

study2 = shared.load_module('rom_study2', HERE / 'run.py')


def trials(reference, target, cfg, directory, commit):
    """Search against this exact baseline, retaining every seed and selected stage config."""
    found = mechanism(reference.mechanism)
    part = shared.partitions.partition(reference.mechanism)
    carried = with_modules(reference.config, target.modules)
    scale = part.external(reference.x, carried)['finger_length_mm']
    rows, results = [], {}
    for seed in cfg['optimizer']['seeds']:
        settings = OptimizerSettings(cfg['optimizer']['algorithm'], cfg['optimizer']['population'],
                                     cfg['optimizer']['generations'], seed)
        result = adapt(found, part, reference.x, carried, settings, scale, cfg['external_range_mm'],
                       workers=cfg['workers'])
        results[seed] = result
        for step in result.steps:
            if step.result is not None:
                save_step(step, directory / f'seed_{seed}' / step.level, source_git_commit=commit)
        rows.append(dict(seed=seed, scale_mm=scale, **study2.summary_row(reference, target, result, scale)))
        if result.status == 'frozen_feasible':
            break
    delta = config_resolution(carried) / scale
    selected = study2.best_rows(rows, delta)[0]
    result = results[selected['best_seed']]
    final_config = carried
    if result.final is not None and result.final.result is not None:
        final_config = result.final.result.config
    if result.final is not None:
        evaluation = found.evaluate(result.final.x, final_config)
        if not evaluation.feasible:
            raise ValueError(f'selected design failed readback: {reference.reference_id}, {target.module_id}')
        selected['ma_wc'] = getattr(evaluation, 'ma_wc', None)
        selected['q_max_deg'] = math.degrees(evaluation.q_max_rad)
    return rows, selected, result, final_config


def run(config_path):
    cfg = read_config_file(config_path)
    output = ROOT / cfg['output']
    output.mkdir(parents=True, exist_ok=False)
    refs = shared.load_references(tuple(cfg['kinds']), ROOT, cfg['references'], cfg.get('servo_tilt_sweep'))
    if not refs:
        raise ValueError('no references')
    shared.check_comparable(refs)
    catalog = shared.catalog()
    unknown = set(cfg['targets']) - set(catalog)
    if unknown:
        raise ValueError(f'unknown targets: {unknown}')
    source = shared.git_commit(output)
    inputs = output / 'inputs'
    inputs.mkdir()
    for name, path in [('config.yaml', Path(config_path)), ('references.yaml', ROOT / cfg['references']),
                       ('modules.yaml', ROOT / shared.CATALOG), ('research-objective.md', ROOT / 'SSoT/research-objective.md')]:
        shutil.copy2(path, inputs / name)
    record = dict(study='study2_rom_conditioned', status='running', config=cfg, source=source,
                  baseline_selection='existing E-then-Q hierarchy, best over declared seeds; no target-specific baseline selection',
                  normalization='baseline cost / original nominal length; interface cost / selected ROM-baseline length; never summed',
                  conditional='failed ROM baseline blocks interface stage, not an interface infeasibility verdict',
                  formulations=shared.formulations(refs), rules=shared.check_comparable(refs))
    manifest = output / 'manifest.json'
    manifest.write_text(json.dumps(record, indent=2, default=str)+'\n')
    baseline_rows, baseline_best, interface_rows, interface_best, blocked = [], [], [], [], []
    for degrees in cfg['required_flexion_deg']:
        for original in refs:
            ref = replace(original, config=shared.with_required_flexion(original.config, degrees))
            found = mechanism(ref.mechanism)
            part = shared.partitions.partition(ref.mechanism)
            folder = output / f'rom_{degrees:g}' / ref.reference_id
            folder.mkdir(parents=True)
            print(f'ROM {degrees} / {ref.reference_id}: prepare baseline', flush=True)
            rows, best, result, baseline_config = trials(ref, catalog['baseline'], cfg, folder / 'baseline', source['commit'])
            baseline_rows.extend(dict(required_flexion_deg=degrees, **row) for row in rows)
            baseline_best.append(dict(required_flexion_deg=degrees, **best))
            shared.write_csv(output / 'baseline_summary.csv', baseline_rows)
            shared.write_csv(output / 'baseline_best.csv', baseline_best)
            if result.final is None:
                for target_id in cfg['targets']:
                    blocked.append(dict(required_flexion_deg=degrees, reference_id=ref.reference_id,
                                        target=target_id, status='baseline_not_found'))
                shared.write_csv(output / 'blocked.csv', blocked)
                print('  baseline not found: interface stage blocked', flush=True)
                continue
            conditioned = replace(ref, x=result.final.x, config=baseline_config)
            baseline_spec = dict(reference_id=ref.reference_id, required_flexion_deg=degrees,
                                 selected_seed=best['best_seed'], nominal_x=list(original.x),
                                 baseline_x=list(conditioned.x), config=found.config_to_json(baseline_config),
                                 rom_external_cost=best['final_external_cost'], rom_mechanism_cost=best['final_mechanism_cost'])
            (folder / 'baseline.json').write_text(json.dumps(baseline_spec, indent=2, default=str)+'\n')
            print(f"  baseline {best['status']}: eps_d={best['final_external_cost']}, eps_kappa={best['final_mechanism_cost']}", flush=True)
            nominal_external = part.external(original.x, original.config)
            nominal_scale = nominal_external['finger_length_mm']
            for target_id in cfg['targets']:
                target = catalog[target_id]
                rows, best_target, adapted, final_config = trials(conditioned, target, cfg,
                    folder / 'targets' / target_id, source['commit'])
                extra = dict(required_flexion_deg=degrees, baseline_seed=best['best_seed'],
                             rom_external_cost=best['final_external_cost'], rom_mechanism_cost=best['final_mechanism_cost'])
                best_target['net_external_cost_from_nominal'] = None if adapted.final is None else change(
                    part.external(adapted.final.x, final_config), nominal_external, nominal_scale)
                interface_rows.extend(dict(**extra, **row) for row in rows)
                interface_best.append(dict(**extra, **best_target))
                shared.write_csv(output / 'interface_summary.csv', interface_rows)
                shared.write_csv(output / 'interface_best.csv', interface_best)
                print(f"    {target_id}: {best_target['status']}, eps_d={best_target['final_external_cost']}, eps_kappa={best_target['final_mechanism_cost']}", flush=True)
    record.update(status='complete', baseline_conditions=len(baseline_best),
                  interface_conditions=len(interface_best), blocked_conditions=len(blocked))
    manifest.write_text(json.dumps(record, indent=2, default=str)+'\n')
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    args = parser.parse_args()
    print(f'wrote {run(args.config)}')
