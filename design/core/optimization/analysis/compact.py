"""Study 2 extension: smallest finger accommodating a fixed interface.

Reuse existing E/Q/A partitions, mounting, evaluators and optimizer. First
minimize absolute extended length; then minimize support/articulation change
within one declared length tolerance. This is a best-found result, not a global
optimality certificate. Actuation remains the matched reference's.
"""
from __future__ import annotations

import dataclasses
from core.optimization.analysis.adaptation import (
    AdaptationMechanism, DESCRIPTORS, OFFSETS, Step, config_resolution, with_finger_lengths,
)
from core.optimization.analysis.tradeoff import goal_of
from core.optimization.mechanism import Objective
from core.optimization.study import run_study

LENGTH_CAP = 'length_cap_mm'


class CompactMechanism(AdaptationMechanism):
    def __init__(self, base, partition, reference, config, objective='finger_length', length_cap_mm=None):
        scale = partition.external(reference, config)['finger_length_mm']
        super().__init__(base, partition, reference, config, 'external', 'external_cost', scale)
        if objective not in ('finger_length', 'shape_change'):
            raise ValueError('compact objective must be finger_length or shape_change')
        self.name = f'compact:{base.name}'
        self.formulation = f'{base.formulation}+compact-v3'
        self.objective = objective
        self.length_cap_mm = length_cap_mm
        self.constraint_names = (*base.constraint_names, LENGTH_CAP)
        self.objectives['shape_change'] = Objective(self.shape_cost, 'shape_cost',
            'Support/articulation change', 'least_shape_change', False)
        self.objectives['finger_length'] = Objective(lambda e: e.external['finger_length_mm'],
            'e_finger_length_mm', 'Length (mm)', 'shortest', False)

    def shape_cost(self, evaluation):
        return max(abs(evaluation.external[k]-self.reference_external[k])
                   for k in DESCRIPTORS if k!='finger_length_mm')/self.scale_mm

    def report(self, evaluation):
        return {**super().report(evaluation),'shape_cost':self.shape_cost(evaluation)}

    def search_box(self, config):
        """Absolute E box, expressed through existing reference-offset coordinates.

        Unlike old Study 2, no +/-20 mm proximity restriction. Support lower
        bounds follow the given module; all upper bounds follow the length domain.
        Remaining mechanism/hardware/relative-layout bounds are preserved.
        """
        goal = goal_of(config)
        low, high = goal.extended_length_bounds_mm
        p = goal.packaging
        external = {'finger_length_mm': (low, high),
            'proximal_span_mm': (p.proximal_pad_start_mm + goal.minimum_proximal_support_mm, high),
            'distal_span_mm': (goal.minimum_distal_support_mm + p.distal_pad.offset_mm, high),
            'articulation_mm': (.1, high)}
        bounds = self.partition.coordinate_bounds(config)
        box = {f'd_{name}': (lo-self.reference_external[name], hi-self.reference_external[name])
               for name, (lo,hi) in external.items()}
        box.update(bounds)
        names = self.search_names(config)
        return tuple(box[n][0] for n in names), tuple(box[n][1] for n in names)

    def evaluate(self, x, config):
        e = super().evaluate(x, config)
        if e is None:
            return None
        cap = 0. if self.length_cap_mm is None else e.external['finger_length_mm']-self.length_cap_mm
        constraints = dict(e.base.constraints)
        constraints[LENGTH_CAP] = cap
        return dataclasses.replace(e, cap=cap, constraints=constraints)

    def metadata(self):
        return {**super().metadata(), 'length_cap_mm': self.length_cap_mm,
                'search_domain': 'absolute external dimensions; no reference proximity box',
                'secondary': 'min max absolute change of proximal span, distal span, articulation / reference length; full eps_E reported separately'}


def compact_config(base, reference, config, bounds_mm):
    """Propagate the absolute length domain, including fixed-shaft belt reach.

    A legacy relative-coordinate search box is not a physical maximum belt
    length. With shaft position fixed and joint A free, dx=shaft_x-A_x must
    cover the full allowed joint domain. Pulleys/clearances remain unchanged.
    """
    config=with_finger_lengths(config,*bounds_mm)
    if base.name=='revolute_parallel':
        values=dict(zip(base.variable_names,reference))
        shaft_x=values['a_x_mm']+values['ap_from_a_x_mm']
        low,high=config.bounds['a_x_mm']
        bounds=dict(config.bounds)
        bounds['ap_from_a_x_mm']=(shaft_x-high,shaft_x-low)
        config=dataclasses.replace(config,bounds=bounds)
    return config


def search_compact(base, partition, reference, config, settings, bounds_mm, warm=(), workers=1,
                   length_tolerance_mm=None, primary_run=None):
    """Two searches; admitted warm starts use the same grid and feasibility checks.

    The existing optimizer admits at most population//2 warm vectors. Secondary
    starts prepend the primary incumbent; remaining warm vectors are truncated
    by that same rule. A recorded warm vector is not assumed feasible.
    """
    config = compact_config(base, reference, config, bounds_mm)
    tolerance = config_resolution(config) if length_tolerance_mm is None else length_tolerance_mm
    steps = []
    for level, objective in [('minimum_length','finger_length'),('minimum_change','shape_change')]:
        cap = None if not steps else steps[0].evaluation.external['finger_length_mm'] + tolerance
        adapted = CompactMechanism(base, partition, reference, config, objective, cap)
        initial = list(warm) if not steps else [steps[0].x, *warm]
        if level=='minimum_length' and primary_run is not None:
            import json
            from core.optimization.study import load_run
            saved=load_run(primary_run)
            if (json.dumps(base.config_to_json(config),sort_keys=True)!=json.dumps(saved.metadata['config'],sort_keys=True)
                    or saved.settings.to_json()!=settings.to_json()):
                raise ValueError('reused primary config/settings mismatch')
            rows=saved.candidates()
            if not rows:
                steps.append(Step(level,'not_found'))
                break
            row=min(rows,key=lambda r:(float(r['e_finger_length_mm']),float(r['external_cost']),float(r['mechanism_cost'])))
            x=tuple(base.read_design(row));e=adapted.evaluate(x,config)
            if e is None or not e.feasible:raise ValueError('reused primary no longer feasible')
            steps.append(Step(level,'found',x,e))
            continue
        result = run_study(adapted, config, settings, initial_designs=initial, workers=workers)
        if not result.pareto:
            steps.append(Step(level, 'not_found', result=result, evaluations=result.evaluation_count))
            break
        candidate = min(result.pareto, key=lambda c: (
            c.evaluation.external['finger_length_mm'], c.evaluation.external_cost,
            c.evaluation.mechanism_cost) if objective=='finger_length' else (
            adapted.shape_cost(c.evaluation),
            tuple(sorted((abs(c.evaluation.external[k]-adapted.reference_external[k])for k in DESCRIPTORS if k!='finger_length_mm'),reverse=True)),
            c.evaluation.external['finger_length_mm'], c.evaluation.mechanism_cost))
        steps.append(Step(level,'found',candidate.x,candidate.evaluation,result.evaluation_count,result))
    return steps
