"""Read back the ROM-conditioned pilot, validate saved designs, and plot interface costs."""
from __future__ import annotations
import csv
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts/study')]
import shared
from core.optimization.mechanism import mechanism
from core.optimization.analysis.tradeoff import with_modules
from core.optimization.study import load_run

OUT = ROOT / 'output/study/study2_rom_conditioned_2026-11-09/pilot'


def read(name):
    path = OUT / name
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    manifest = json.loads((OUT / 'manifest.json').read_text())
    assert manifest['status'] == 'complete'
    baseline, interfaces = read('baseline_best.csv'), read('interface_best.csv')
    blocked = read('blocked.csv') if (OUT / 'blocked.csv').exists() else []
    catalog = shared.catalog()
    checks = []
    for row in baseline:
        if row['status'] == 'not_found':
            assert not any(r['reference_id'] == row['reference_id'] and r['required_flexion_deg'] == row['required_flexion_deg'] for r in interfaces)
            continue
        path = OUT / f"rom_{float(row['required_flexion_deg']):g}" / row['reference_id'] / 'baseline.json'
        saved = json.loads(path.read_text())
        found = mechanism(row['mechanism'])
        config = found.config_from_json(saved['config'])
        e = found.evaluate(saved['baseline_x'], config)
        assert e.feasible
        assert math.isclose(getattr(config, 'goal', config).required_rom_rad, math.radians(float(row['required_flexion_deg'])))
        checks.append(dict(stage='baseline', required_flexion_deg=row['required_flexion_deg'], reference_id=row['reference_id'], target='baseline', feasible=True))
    for row in interfaces:
        if row['status'] == 'not_found':
            continue
        folder = OUT / f"rom_{float(row['required_flexion_deg']):g}" / row['reference_id']
        baseline_spec = json.loads((folder / 'baseline.json').read_text())
        found = mechanism(row['mechanism'])
        if row['final_level'] == 'frozen':
            config = with_modules(found.config_from_json(baseline_spec['config']), catalog[row['target']].modules)
        else:
            config = load_run(folder / 'targets' / row['target'] / f"seed_{row['best_seed']}" / row['final_level']).config
        x = tuple(map(float, row['final_x'].split(';')))
        evaluation = found.evaluate(x, config)
        assert evaluation.feasible, (row['reference_id'], row['target'], row['required_flexion_deg'])
        names = found.variable_names
        assert math.isclose(x[names.index('servo_tilt_deg')], baseline_spec['baseline_x'][names.index('servo_tilt_deg')])
        assert math.isclose(getattr(config, 'goal', config).required_rom_rad, math.radians(float(row['required_flexion_deg'])))
        if row['target'] == 'baseline':
            assert float(row['final_external_cost']) == float(row['final_mechanism_cost']) == 0.0
        checks.append(dict(stage='interface', required_flexion_deg=row['required_flexion_deg'], reference_id=row['reference_id'], target=row['target'], feasible=True))
    shared.write_csv(OUT / 'readback.csv', checks)
    plot(interfaces, blocked)
    lines = ['# Study 2: ROM-conditioned baseline pilot', '',
        '2026-11-09. Every 90/120/150-degree ROM has a separately selected baseline before target-interface adaptation.',
        'This is a one-seed pilot (population 32, 20 generations per search), not a final architecture ranking.', '',
        '## Protocol', '',
        '- Start from the same matched nominal reference descriptors; CoDe/belt tilt fixed at 15 degrees.',
        '- DD and physical RCJ at 0 and 90 degrees are separate pilot conditions; ideal RCJ is an upper bound.',
        '- Prepare the baseline interface (20x3 mm on both segments) for each ROM with the existing E-then-Q hierarchy.',
        '- Freeze the chosen baseline, then adapt independently to each of eight interfaces at the same ROM.',
        '- Retain the 30-150-degree transmission-angle range and 10-degree hyperextension for all CoDe conditions.',
        '- No k_max cap; face usability at required ROM and actual nonpenetration checks remain active.',
        '- Baseline cost is normalized by nominal length; interface cost by selected ROM-baseline length. They are never added.',
        '- Net external change from nominal is independently measured, rather than summing two max norms.',
        '- If a ROM baseline is not found, its target conditions are blocked, not labelled interface-infeasible.',
        '- ROM-conditioned baselines may differ in E/Q across architectures; only the initial nominal descriptors are matched.', '',
        '## Baseline preparation', '',
        '| Reference | ROM | Status | eps_E from nominal | eps_Q from nominal |',
        '|---|---:|---|---:|---:|']
    for row in baseline:
        lines.append(f"| {row['reference_id']} | {row['required_flexion_deg']} | {row['status']} | {row['final_external_cost']} | {row['final_mechanism_cost']} |")
    lines += ['', '## Interface outcomes', '', '| Reference | ROM | Frozen | Adapted | Not found |', '|---|---:|---:|---:|---:|']
    for row in baseline:
        selected = [r for r in interfaces if r['reference_id'] == row['reference_id'] and r['required_flexion_deg'] == row['required_flexion_deg']]
        if not selected:
            lines.append(f"| {row['reference_id']} | {row['required_flexion_deg']} | baseline blocked | - | - |")
        else:
            counts = [sum(r['status'] in statuses for r in selected) for statuses in [('frozen_feasible',), ('level1','level2'), ('not_found',)]]
            lines.append(f"| {row['reference_id']} | {row['required_flexion_deg']} | {counts[0]} | {counts[1]} | {counts[2]} |")
    lines += ['', '## Artifacts and scope', '',
        'Outputs: design/output/study/study2_rom_conditioned_2026-11-09/pilot/.',
        'baseline_best.csv records ROM preparation; interface_best.csv records additional interface adaptation.',
        'Each baseline.json includes the exact selected geometry and config; searches store per-seed stage configs and Pareto rows.',
        f'Readback validated {len(checks)} feasible baseline/interface designs; {len(blocked)} target conditions were blocked by baseline search failure.',
        'interface_costs.png/.pdf shows per-interface external cost versus required ROM. Missing solutions are not interpolated.',
        'not_found is search-budget/domain dependent. Pilot costs need not be monotone. In addition, endpoint-only face usability',
        'means feasible sets at different required ROMs are not guaranteed nested, even though collision-free reach requirements are nested.',
        'No physical torque, buildability, 3D accessibility or learning performance was measured.', '']
    (ROOT.parent / 'report/study2_rom_conditioned_2026-11-09.md').write_text('\n'.join(lines))
    inputs = {name: hashlib.sha256((OUT / name).read_bytes()).hexdigest() for name in ['manifest.json','baseline_best.csv','interface_best.csv']}
    analysis_source = OUT / 'analysis_source'
    analysis_source.mkdir(exist_ok=True)
    shutil.copy2(__file__, analysis_source / Path(__file__).name)
    (OUT / 'analysis_manifest.json').write_text(json.dumps(dict(inputs_sha256=inputs, script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), source=shared.git_commit(analysis_source)),indent=2)+'\n')
    print(f'Validated {len(checks)} saved feasible designs; baseline conditions {len(baseline)}, interface conditions {len(interfaces)}, blocked {len(blocked)}')
    for rom in (90,120,150):
        sel=[r for r in interfaces if float(r['required_flexion_deg'])==rom and r['reference_id']=='code_matched']
        print('CoDe',rom,[(r['target'],r['status'],r['final_external_cost'],r['final_mechanism_cost']) for r in sel])


def plot(rows, blocked):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    refs = list(dict.fromkeys(r['reference_id'] for r in rows))
    targets = ['thick','long','long_thick','asym_long_proximal','asym_long_distal','asym_proximal_heavy','asym_distal_heavy']
    labels = {'code_matched':'CoDe 15 deg','belt_2r_matched':'Belt 15 deg','2r_dd_tilt0_matched__tilt0':'DD 0 deg','2r_dd_tilt0_matched__tilt90':'DD 90 deg','rcj_tilt0_matched__tilt0':'RCJ 0 deg','rcj_tilt0_matched__tilt90':'RCJ 90 deg','rcj_ideal_matched':'Ideal RCJ (upper bound)'}
    fig, axes = plt.subplots(2,4,figsize=(15,8),sharex=True,constrained_layout=True)
    colors = plt.get_cmap('tab10')
    for ax,target in zip(axes.flat,targets):
        for i,ref in enumerate(refs):
            sel={float(r['required_flexion_deg']):r for r in rows if r['reference_id']==ref and r['target']==target}
            for rom in (90,120,150):
                row=sel.get(rom)
                if row is not None and row['status']!='not_found':
                    ax.scatter(rom,float(row['final_external_cost']),color=colors(i),marker='D' if 'ideal' in ref else 'o',s=30)
                elif row is not None:
                    ax.scatter(rom,-.025-i*.004,color=colors(i),marker='x',s=24)
            # Only join adjacent successful conditions; failures do not become zero-cost values.
            for a,b in ((90,120),(120,150)):
                if all(v in sel and sel[v]['status']!='not_found' for v in (a,b)):
                    ax.plot([a,b],[float(sel[v]['final_external_cost']) for v in (a,b)],color=colors(i),alpha=.65)
        ax.set_title(target.replace('_',' '),fontsize=10)
        ax.set_xticks([90,120,150])
        ax.tick_params(axis='x', labelbottom=True)
        ax.set_xlabel('Required ROM [deg]')
        ax.set_ylabel('Additional interface eps_E')
        ax.axhline(0,color='#aaaaaa',linewidth=.6)
        ax.grid(alpha=.2)
    axes.flat[-1].axis('off')
    handles=[plt.Line2D([],[],color=colors(i),marker='D' if 'ideal' in ref else 'o',label=labels.get(ref,ref)) for i,ref in enumerate(refs)]
    axes.flat[-1].legend(handles=handles,loc='center',frameon=False,fontsize=10)
    axes.flat[-1].text(.03,.08,'x below zero: not found\nAbsent ROM baseline: no target search\nOne-seed pilot; best found, not optimum',fontsize=9,transform=axes.flat[-1].transAxes)
    fig.suptitle('Interface adaptation from independently ROM-feasible baselines\nCoDe 15 degrees; actual nonpenetration; original face rule; transmission angle 30-150 degrees',fontsize=12)
    for ext in ('png','pdf'):
        fig.savefig(OUT/f'interface_costs.{ext}',dpi=180)
    plt.close(fig)


if __name__=='__main__':
    main()
