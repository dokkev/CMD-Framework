#!/usr/bin/env python3
"""Recheck historical Pareto alternatives and reflect identical contact models."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import shutil
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path[:0] = [str(ROOT), str(HERE.parent)]
import shared
from core.optimization.analysis.transmission import Transmission, reflect
from core.optimization.analysis.tradeoff import with_modules
from core.optimization.mechanism import mechanism, read_config_file
from core.optimization.study import load_run


def cover(scores, count, designs=None):
    """Farthest-point deterministic cover, original normalized objective space.

    Start at minimum first objective; include extremes through farthest-point
    choice. CSV order breaks ties. No random sampler or replication inference.
    """
    if not len(scores):
        return []
    scores = np.asarray(scores)
    span = np.ptp(scores, axis=0)
    z = (scores - scores.min(axis=0)) / np.where(span > 0, span, 1.)
    design = np.asarray(scores if designs is None else designs, dtype=float)
    span_x = np.ptp(design, axis=0)
    design = (design - design.min(axis=0)) / np.where(span_x > 0, span_x, 1.)
    selected = [int(np.argmin(z[:, 0]))]
    design_distance = np.linalg.norm(design - design[selected[0]], axis=1)
    distance = np.linalg.norm(z - z[selected[0]], axis=1)
    for _ in range(min(count, len(scores)) - 1):
        distance[selected] = -1
        tied = np.flatnonzero(distance >= distance.max() - 1e-12)
        i = int(tied[np.argmax(design_distance[tied])])
        selected.append(i)
        distance = np.minimum(distance, np.linalg.norm(z - z[i], axis=1))
        design_distance = np.minimum(design_distance, np.linalg.norm(design - design[i], axis=1))
    return selected


def remount(name, x, config, modules):
    """Keep E/Q/A, apply the EXISTING mounting policy for this common module."""
    part = shared.partitions.partition(name)
    config = with_modules(config, modules)
    external = part.external(x, config)
    return part.compose(external, part.coordinate_values(x, config), x, config), config


def snapshot(path, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, destination)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(config_path=HERE / 'config.yaml', pilot=False):
    cfg = read_config_file(config_path)
    out = ROOT / cfg['output']
    if pilot:
        out = out.with_name(out.name + '_pilot')
    if out.exists():
        raise FileExistsError(f'results already exist: {out}; choose a new output')
    out.mkdir(parents=True)
    refs = shared.load_references()
    rules = shared.check_comparable(refs)
    record = dict(config=cfg, pilot=pilot, source=shared.git_commit(out), comparison_rules=rules,
                  sampling='farthest point in normalized original objectives; Study 2 adaptation proposals re-evaluated; geometry breaks objective ties; not architecture-optimal Pareto fronts',
                  coordinate='output shafts [MCP, PIP servo], positive flexion, XZ world; m/rad',
                  normalization='K/(kn * finger_length_m**2), B/(bn * finger_length_m**2)',
                  inputs={}, fronts=[], models='frozen contact frames, zero preload, fixed object, massless Kelvin-Voigt')
    for path in [Path(config_path), ROOT / shared.REFERENCES, ROOT / shared.CATALOG, ROOT / 'GOAL.md']:
        relative = path.resolve().relative_to(ROOT)
        record['inputs'][str(relative)] = snapshot(path, out / 'inputs' / relative)
    catalog = shared.catalog()
    candidates = []
    for r in refs:
        candidates.append(dict(id=r.reference_id, name=r.mechanism, x=r.x, config=r.config,
                               group='matched', upper=r.upper_bound, provenance=r.source))
    for spec in cfg['fronts']:
        path = ROOT / spec['run']
        saved = load_run(path)  # missing artifact is a hard blocker, never fabricated
        found, config = shared.load_config(spec['config'])
        if saved.mechanism.name != found.name:
            raise ValueError('front and current config mechanisms differ')
        from core.optimization.analysis.adaptation import with_finger_lengths
        config = with_finger_lengths(config, *cfg['front_finger_length_bounds_mm'])
        if not saved.metadata.get('adaptation'):
            raise ValueError('expected declared Study 2 adaptation front')
        rows = saved.candidates()
        objectives = [[(found.objectives[o].minimized(float(row[found.objectives[o].column]))
                        if o in found.objectives else float(row[o]) if o in ('external_cost', 'mechanism_cost')
                        else (_ for _ in ()).throw(ValueError(f'unknown objective {o}')))
                       for o in saved.objectives] for row in rows]
        indices = cover(objectives, 3 if pilot else cfg['pareto_samples'], [found.read_design(row) for row in rows])
        # Full historical front retained, even though local dynamics use a cover.
        for filename in ('run.json', 'pareto.csv'):
            src = path / filename
            record['inputs'][str(src.relative_to(ROOT))] = snapshot(src, out / 'inputs' / spec['id'] / filename)
        record['fronts'].append(dict(**spec, original_formulation=saved.formulation,
                                    current_formulation=found.formulation, count=len(rows),
                                    selected_csv_indices=indices, optimizer=saved.metadata.get('optimizer'),
                                    adaptation=saved.metadata.get('adaptation')))
        for i in indices:
            candidates.append(dict(id=f"{spec['id']}__{rows[i]['candidate']}", name=found.name,
                x=tuple(found.read_design(rows[i])), config=config, group='adaptation_front_cover', upper=found.name.endswith('_idealized'),
                provenance=f"{spec['run']}/pareto.csv#candidate={rows[i]['candidate']}"))
    designs, acceptance, results = [], [], []
    max_error = 0.
    for c in candidates:
        found = mechanism(c['name'])
        designs.append(dict(candidate_id=c['id'], mechanism=c['name'], group=c['group'], upper_bound=c['upper'],
                            provenance=c['provenance'], original_design=json.dumps(c['x']),
                            current_config=json.dumps(found.config_to_json(c['config']))))
        for module_id in cfg['modules']:
            x, config = remount(c['name'], c['x'], c['config'], catalog[module_id].modules)
            config = shared.with_required_flexion(config, cfg['required_flexion_deg'])
            e = None if x is None else found.evaluate(x, config)
            feasible = e is not None and e.feasible
            base = dict(candidate_id=c['id'], mechanism=c['name'], group=c['group'], upper_bound=c['upper'], module_id=module_id)
            acceptance.append(dict(**base, feasible=feasible, failure='no_mount' if e is None else
                    ';'.join(k for k, v in e.constraints.items() if v > 0),
                    evaluated_design=json.dumps(x), phi_max_deg=None if e is None else float(np.degrees(e.phi_max_rad)),
                    coverage=None if e is None else found.report(e)['pad_coverage']))
            if not feasible:
                continue
            t = Transmission(c['name'], x, config)
            length = shared.partitions.partition(c['name']).external(x, config)['finger_length_mm'] * .001
            for phi_deg in cfg['flexions_deg'][::4] if pilot else cfg['flexions_deg']:
                phi = np.radians(phi_deg)
                for side in ('proximal', 'distal'):
                    for abstraction, fractions in cfg['contacts'].items():
                        js, ns = [], []
                        for fraction in fractions:
                            j = t.jacobian(phi, side, fraction)
                            error = float(abs(j - t.finite_difference(phi, side, fraction)).max())
                            max_error = max(max_error, error)
                            if error > cfg['fd_tolerance_m_per_rad']:
                                raise ValueError(f'Jacobian FD mismatch {c["id"]}: {error}')
                            js.append(j)
                            ns.append(t.contact(phi, side, fraction)[1])
                        for model in cfg['contact_models']:
                            k, b = reflect(js, ns, model['kn_N_per_m'], model['kt_N_per_m'],
                                           model['bn_Ns_per_m'], model['bt_Ns_per_m'])
                            row = dict(**base, phi_deg=phi_deg, side=side, abstraction=abstraction,
                                contact_model=model['id'], finger_length_m=length,
                                fd_max_abs_m_per_rad=max(float(abs(j - t.finite_difference(phi, side, f)).max())
                                    for j, f in zip(js, fractions)),
                                K_trace_normalized=float(np.trace(k) / (model['kn_N_per_m'] * length**2)),
                                B_trace_normalized=float(np.trace(b) / (model['bn_Ns_per_m'] * length**2)),
                                K_min_eigenvalue=float(np.linalg.eigvalsh(k)[0]))
                            for prefix, matrix in [('K', k), ('B', b)]:
                                row.update({f'{prefix}{i}{j}': float(matrix[i,j]) for i,j in [(0,0),(0,1),(1,1)]})
                            # Frobenius norm of mechanical impedance tau / qdot.
                            for hz in cfg['frequencies_hz']:
                                row[f'Z_frobenius_{hz:g}Hz'] = float(np.linalg.norm(b + k / (2j*np.pi*hz)))
                            results.append(row)
    for name, rows in [('designs', designs), ('acceptance', acceptance), ('reflection', results)]:
        shared.write_csv(out / f'{name}.csv', rows)
    record['max_fd_error_m_per_rad'] = max_error
    record['formulations'] = {r.mechanism: mechanism(r.mechanism).formulation for r in refs}
    (out / 'manifest.json').write_text(json.dumps(record, indent=2, default=str))
    print(json.dumps(dict(output=str(out), designs=len(designs), accepted=sum(r['feasible'] for r in acceptance),
                          reflection_rows=len(results), max_fd_error=max_error)))
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', nargs='?', type=Path, default=HERE / 'config.yaml')
    parser.add_argument('--pilot', action='store_true')
    args = parser.parse_args()
    run(args.config, args.pilot)
