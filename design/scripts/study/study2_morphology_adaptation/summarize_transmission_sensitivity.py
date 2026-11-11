"""Summarize the saved 2026-11-09 transmission-angle sensitivity runs (no search)."""
from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts/study')]
import shared
from core.optimization.study import load_run

OUT = ROOT / 'output/study/transmission_sensitivity_2026-11-09'
RUNS = [f'{phase}_margin{margin}' for phase in ('pilot', 'main') for margin in (30, 20, 10)]
RUNS += ['control_faceoff_margin30']


def candidate_figure(row, x, config):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Circle
    import numpy as np
    from core.morphology.mechanism.linkage.design import decode
    from core.morphology.mechanism.linkage.packaging import proximal_pad_polygon, distal_pad_polygon, pip_footprint

    finger = decode(x, config.link_length_resolution_mm)
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    for ax, deg in zip(axes[:2], (0, 120)):
        sweep = finger.sweep_pip([math.radians(deg)])
        o, a, ap = np.zeros(2), finger.joint.a.array, finger.joint.ap.array
        b, bp, c = sweep.b[0], sweep.bp[0], sweep.tip[0]
        for points, color in ((proximal_pad_polygon(finger, config.packaging), '#db6685'),
                              (distal_pad_polygon(finger, config.packaging, sweep.b, sweep.tip)[0], '#428ac2'),
                              (pip_footprint(config.packaging, finger).outline, '#aaaaaa')):
            ax.add_patch(Polygon(points, facecolor=color, edgecolor=color, alpha=.45))
        for chain in ((o, a, ap, o), (ap, b), (a, bp), (bp, b, c)):
            points = np.array(chain)
            ax.plot(*points.T, color='#333333', linewidth=1.5)
        for name, point in (('a', a), ('ap', ap), ('b', b), ('bp', bp)):
            ax.add_patch(Circle(point, config.packaging.pivot_radius_mm(name), facecolor='#ecdca8', edgecolor='#555555'))
        ax.set_aspect('equal', adjustable='datalim')
        ax.autoscale_view()
        ax.set_title(f'Distal flexion {deg} degrees')
        ax.set_xlabel('x [mm]')
        ax.set_ylabel('z dorsal [mm]')
        ax.grid(alpha=.2)
    phi = np.linspace(-10, 120, 261)
    sweep = finger.sweep_pip(np.radians(phi))
    axes[2].plot(phi[:len(sweep.q_pip_rad)], np.degrees(sweep.margin_at_bp_rad), color='#3b6fb6')
    for margin, color in ((30, '#c34a36'), (20, '#b48a16'), (10, '#448247')):
        axes[2].axhline(margin, color=color, linestyle='--', label=f'{margin}-degree minimum')
    axes[2].axvspan(-10, 0, color='#eeeeee', label='Hyperextension')
    axes[2].set(xlabel='Distal flexion [deg]', ylabel="Transmission angle margin at B' [deg]",
                title='Reference angle at every sampled posture')
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=.2)
    fig.suptitle(f"{row['target']}: face rule ON, CoDe tilt 15 degrees, k_max={row['k_max']:.3f}\n"
                 f"eps_E={float(row['final_external_cost']):.3f}, eps_Q={float(row['final_mechanism_cost']):.3f}; planar model")
    for extension in ('png', 'pdf'):
        fig.savefig(OUT / f"{row['target']}_candidate.{extension}", dpi=180)
    plt.close(fig)


def historical_recheck(inputs):
    """Re-evaluate saved face-off candidates at 15 degrees under the current evaluator."""
    folder = ROOT / 'output/study/study2_sensitivity_links'
    summary = folder / 'summary_best.csv'
    inputs[str(summary.relative_to(ROOT))] = hashlib.sha256(summary.read_bytes()).hexdigest()
    with summary.open() as stream:
        rows = list(csv.DictReader(stream))
    checks = []
    for row in rows:
        if row['reference_id'] != 'code_matched' or row['target'] not in ('asym_long_proximal', 'asym_proximal_heavy'):
            continue
        path = folder / 'targets' / row['target'] / row['reference_id'] / f"seed_{row['best_seed']}" / row['final_level']
        saved = load_run(path)
        inputs[str((path / 'run.json').relative_to(ROOT))] = hashlib.sha256((path / 'run.json').read_bytes()).hexdigest()
        x = list(map(float, row['final_x'].split(';')))
        x[saved.mechanism.variable_names.index('servo_tilt_deg')] = 15.0
        for margin in (30, 20, 10):
            for face in (True, False):
                config = saved.config
                config = replace(config, pip_servo_tilt_rad=math.radians(15),
                                 bounds={**config.bounds, 'servo_tilt_deg': (15., 15.)}, links_block_face=face,
                                 transmission_angle_range_rad=(math.radians(margin), math.radians(180-margin)))
                e = saved.mechanism.evaluate(x, config)
                checks.append(dict(target=row['target'], margin_deg=margin, links_block_face=face,
                                   feasible=e.feasible, k_max=e.torque_ratio_max,
                                   minimum_angle_margin_deg=math.degrees(e.min_margin_rad),
                                   failures=';'.join(f'{k}={v:.5g}' for k, v in e.constraints.items() if v > 0)))
    shared.write_csv(OUT / 'historical_candidate_recheck.csv', checks)


def main():
    rows, selected, crosschecks, inputs = [], [], [], {}
    for name in RUNS:
        folder = OUT / name
        manifest = json.loads((folder / 'manifest.json').read_text())
        cfg = manifest['config']
        angle = cfg['goal_overrides']['transmission_angle_range_rad']
        common = dict(run=name, angle_min_deg=round(math.degrees(angle[0])),
                      angle_max_deg=round(math.degrees(angle[1])),
                      links_block_face=cfg['goal_overrides']['links_block_face'],
                      population=cfg['optimizer']['population'], generations=cfg['optimizer']['generations'])
        assert cfg['goal_overrides']['torque_ratio_max'] is None
        for filename in ('manifest.json', 'summary.csv', 'summary_best.csv'):
            path = folder / filename
            inputs[str(path.relative_to(OUT))] = hashlib.sha256(path.read_bytes()).hexdigest()
        with (folder / 'summary.csv').open() as stream:
            rows.extend({**common, **row} for row in csv.DictReader(stream))
        with (folder / 'summary_best.csv').open() as stream:
            best = list(csv.DictReader(stream))
        for row in best:
            result = {**common, **row}
            result['k_max'] = result['k_min'] = result['min_transmission_margin_deg'] = None
            if row['status'] != 'not_found':
                directory = folder / 'targets' / row['target'] / row['reference_id'] / f"seed_{row['best_seed']}" / row['final_level']
                saved = load_run(directory)
                x = tuple(float(v) for v in row['final_x'].split(';'))
                config = saved.config
                evaluation = saved.mechanism.evaluate(x, config)
                assert evaluation.feasible, (name, row['target'], dict(evaluation.constraints))
                assert math.isclose(x[saved.mechanism.variable_names.index('servo_tilt_deg')], 15)
                result.update(k_max=evaluation.torque_ratio_max, k_min=evaluation.torque_ratio_min,
                              min_transmission_margin_deg=math.degrees(evaluation.min_margin_rad))
                if name.startswith('main_'):
                    candidate_figure(result, x, config)
                for margin in (30, 20, 10):
                    checked = saved.mechanism.evaluate(x, replace(config, transmission_angle_range_rad=(math.radians(margin), math.radians(180-margin))))
                    crosschecks.append(dict(source_run=name, target=row['target'], margin_deg=margin,
                                            feasible=checked.feasible, k_max=checked.torque_ratio_max,
                                            failures=';'.join(k for k, v in checked.constraints.items() if v > 0)))
            selected.append(result)
    shared.write_csv(OUT / 'summary_runs.csv', rows)
    shared.write_csv(OUT / 'selected.csv', selected)
    if crosschecks:
        shared.write_csv(OUT / 'crosscheck.csv', crosschecks)
    historical_recheck(inputs)
    source = Path(__file__)
    snapshot = OUT / 'analysis_source'
    snapshot.mkdir(exist_ok=True)
    shutil.copy2(source, snapshot / source.name)
    source_record = shared.git_commit(snapshot)
    (OUT / 'analysis_manifest.json').write_text(json.dumps(dict(inputs_sha256=inputs,
        analysis_script=str(source.relative_to(ROOT)), analysis_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        source=source_record,
        note='Saved searches plus fresh selected-design readback; not_found is not impossibility.'), indent=2)+'\n')
    report = ROOT.parent / 'report/transmission_sensitivity_2026-11-09.md'
    lines = ['# Proximal accommodation under relaxed transmission-angle limits', '',
        '2026-11-09. Model-derived sensitivity study, current CoDe nominal servo tilt 15 degrees.', '',
        'The question is whether sacrificing transmission quality can recover proximal-heavy interfaces while',
        'retaining the same contact-face rule. No k_max cap is imposed in any condition. The changed variable',
        'is the permitted transmission angle, not a direct optimization objective or limit on k.', '',
        '## Controlled conditions', '',
        '- Interfaces: proximal 40 x 3 / distal 20 x 8 mm; proximal 45 x 5 / distal 15 x 3 mm.',
        '- Required distal flexion 120 degrees, hyperextension 10 degrees; closure, collision and face checks retained.',
        '- Servo shaft and 15-degree tilt fixed; Study 2 E/Q hierarchy and external range +/-20 mm unchanged.',
        '- Current actual-envelope nonpenetration model, with no universal body allowance; planar assumptions remain.',
        '- Pilot: population 32, 20 generations, seeds 0-2. Main: population 100, 100 generations, seeds 0-2.',
        '- Positive control: same main budget, 30-150 degrees, links_block_face=False. It does not belong to the primary comparison.', '',
        '## Saved results', '',
        '| Run | Target | Found seeds / run | Status | eps_E | eps_Q | Selected k_max |',
        '|---|---|---:|---|---:|---:|---:|']
    for row in selected:
        k = '-' if row['k_max'] is None else f"{row['k_max']:.4f}"
        lines.append(f"| {row['run']} | {row['target']} | {row['seeds_found']}/{row['seeds_run']} | {row['status']} | {row['final_external_cost']} | {row['final_mechanism_cost']} | {k} |")
    lines += ['', '## Interpretation and limits', '',
        'At least one relaxed-angle search finds a face-rule-ON solution: the earlier proximal failure is not',
        'an established architectural impossibility. This does not demonstrate a global optimum or robustness',
        'across optimizer seeds. k_max is read back from each selected design rather than inferred from its angle bound.', '',
        'For the asym_long_proximal main-margin10 candidate, k_max is about 5.063, eps_E about 0.202 and',
        'eps_Q about 0.171. Rechecking that exact geometry at a 20-degree minimum fails only the required',
        'hyperextension (about 2 degrees missing); at a 30-degree minimum both ROM and hyperextension fail.',
        'Thus the relaxed limit matters at the extension end as well as at the requested 120-degree posture.', '',
        'That candidate changes finger length by +8.5 mm, proximal support span by +15.6 mm, distal span by',
        '-7.0 mm and articulation station by +18.2 mm. Its largest independent Q-coordinate change is 15.4 mm.',
        'It therefore demonstrates recoverable accommodation with changed embodiment, not a fixed-H interface swap.', '',
        'The historical face-off candidates rechecked at 15 degrees already satisfy the 30-degree minimum.',
        'Turning the face rule on causes pad_usable_mm violations of 4 and 6.5 mm respectively. Those specific',
        'geometries are limited by the face rule, not k or transmission-angle constraints.', '',
        'All not_found entries mean no feasible design was found under the stated search domain and budget.',
        'No physical torque, strength, 3D accessibility or hardware behavior was validated.', '',
        '## Artifacts', '',
        '- Root: design/output/study/transmission_sensitivity_2026-11-09/',
        '- summary_runs.csv: per-seed outcomes; selected.csv: best selected outcomes and re-evaluated k.',
        '- crosscheck.csv: same selected geometries under 30/20/10-degree limits.',
        '- historical_candidate_recheck.csv: separate counterfactual diagnostic at 15 degrees.',
        '- Each run stores manifest, source snapshot, exact stage configs, optimizer records and candidate CSVs.',
        '- analysis_manifest.json hashes input tables and the analysis script; analysis_source stores its source snapshot.',
        '- asym_long_proximal_candidate.png/.pdf: geometry at 0/120 degrees and transmission-angle-margin trajectory.', '',
        'Reproduce searches with sensitivity_transmission_{30,20,10}_{pilot,main}.yaml and',
        'sensitivity_transmission_faceoff_control.yaml through the existing Study 2 run.py.',
        'Summarize using scripts/study/study2_morphology_adaptation/summarize_transmission_sensitivity.py.', '']
    report.write_text('\n'.join(lines))
    for row in selected:
        print(row['run'], row['target'], row['status'], 'seeds', row['seeds_found']+'/'+row['seeds_run'],
              'eps_E', row['final_external_cost'], 'eps_Q', row['final_mechanism_cost'], 'k_max', row['k_max'])


if __name__ == '__main__':
    main()
