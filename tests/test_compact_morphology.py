"""Compact extension contracts, independent of stochastic search success."""
from pathlib import Path
import dataclasses
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/study'))
import shared
from core.optimization.analysis.compact import CompactMechanism, LENGTH_CAP
from core.optimization.analysis.adaptation import with_finger_lengths
from core.optimization.analysis.tradeoff import with_modules,goal_of
from core.optimization.mechanism import mechanism
from core.optimization.study import MechanismProblem


@pytest.mark.parametrize('ref',shared.load_references(),ids=lambda r:r.reference_id)
def test_domain_expands_length_and_dependent_geometry_without_changing_hardware(ref):
    carried=with_modules(ref.config,shared.catalog()['baseline'].modules)
    cfg=with_finger_lengths(carried,40,160)
    part=shared.partitions.partition(ref.mechanism)
    wrapper=CompactMechanism(mechanism(ref.mechanism),part,ref.x,cfg)
    lo,hi=wrapper.search_box(cfg)
    box=dict(zip(wrapper.search_names(cfg),zip(lo,hi)))
    e0=part.external(ref.x,cfg)
    assert box['d_finger_length_mm']==pytest.approx((40-e0['finger_length_mm'],160-e0['finger_length_mm']))
    assert goal_of(cfg).extended_length_bounds_mm==(40,160)
    assert goal_of(cfg).packaging==goal_of(carried).packaging
    # Existing mapping must reconstruct the reference, preserving actuator placement.
    x=wrapper.to_design(wrapper.from_design(ref.x,cfg),cfg)
    assert x==pytest.approx(ref.x,abs=1e-7)
    evaluation=wrapper.evaluate(x,cfg)
    assert evaluation.feasible
    assert wrapper.objective_values(evaluation,('finger_length',))==pytest.approx((90.,))
    assert wrapper.objective_values(evaluation,('external_cost',))==pytest.approx((0.,))


def test_wide_domain_can_represent_longer_than_old_110mm_cap():
    ref=next(r for r in shared.load_references()if r.reference_id=='2r_dd_tilt90_matched')
    carried=with_modules(ref.config,shared.catalog()['baseline'].modules)
    cfg=with_finger_lengths(carried,40,160);part=shared.partitions.partition(ref.mechanism)
    wrapper=CompactMechanism(mechanism(ref.mechanism),part,ref.x,cfg)
    external={**part.external(ref.x,cfg),'finger_length_mm':130.,'proximal_span_mm':65.,
              'distal_span_mm':45.,'articulation_mm':75.}
    x=part.compose(external,{},ref.x,cfg)
    assert x is not None
    assert part.external(x,cfg)==pytest.approx(external)
    assert wrapper.to_design(wrapper.from_design(x,cfg),cfg)==pytest.approx(x)


def test_secondary_length_cap_is_enforced_on_real_evaluator_result():
    ref=next(r for r in shared.load_references()if r.reference_id=='belt_2r_matched')
    carried=with_modules(ref.config,shared.catalog()['baseline'].modules)
    cfg=with_finger_lengths(carried,40,160);part=shared.partitions.partition(ref.mechanism)
    wrapper=CompactMechanism(mechanism(ref.mechanism),part,ref.x,cfg,'shape_change',89.9)
    e=wrapper.evaluate(ref.x,cfg)
    assert e.base.feasible
    assert not e.feasible
    assert e.cap==pytest.approx(.1)
    assert e.constraints[LENGTH_CAP]==pytest.approx(.1)
    assert len(e.g)==len(wrapper.constraint_names)


def test_modules_and_actuation_are_held_while_external_search_moves():
    ref=next(r for r in shared.load_references()if r.reference_id=='belt_2r_matched')
    cfg=with_finger_lengths(with_modules(ref.config,shared.catalog()['long'].modules),40,160)
    part=shared.partitions.partition(ref.mechanism)
    e=part.external(ref.x,cfg)
    moved={**e,'finger_length_mm':125.,'proximal_span_mm':46.,'distal_span_mm':40.,'articulation_mm':55.}
    x=part.compose(moved,{},ref.x,cfg)
    assert x is not None
    old,new=part.values(ref.x),part.values(x)
    assert new['a_x_mm']+new['ap_from_a_x_mm']==pytest.approx(old['a_x_mm']+old['ap_from_a_x_mm'])
    assert new['ap_from_a_z_mm']==old['ap_from_a_z_mm']
    assert new['servo_tilt_deg']==old['servo_tilt_deg']
    assert goal_of(cfg).packaging.distal_pad.length_mm==35.


def test_global_selection_keeps_eligible_primary_from_another_seed():
    runner=shared.load_module('compact_run',ROOT/'scripts/study/study2_compact_morphology/run.py')
    ref=next(r for r in shared.load_references()if r.reference_id=='2r_dd_tilt90_matched')
    part=shared.partitions.partition(ref.mechanism)
    e0=part.external(ref.x,None)
    # Independent synthetic descriptor example; these test selection, not feasibility.
    def design(length,prox):
        x=list(ref.x);x[1]=length-x[0];x[2]=prox
        return x
    import json
    rows=[]
    for seed,pL,pP,fL,fP in [(0,100.,90.,100.1,90.),(1,100.05,60.,100.15,50.)]:
        rows.append(dict(target_id='test',reference_id=ref.reference_id,mechanism=ref.mechanism,
            found=True,seed=seed,reference_x=json.dumps(ref.x),minimum_length_x=json.dumps(design(pL,pP)),
            x=json.dumps(design(fL,fP)),**{f'reference_{k}':v for k,v in e0.items()}))
    best=runner.best_rows(rows,.1)[0]
    assert best['seed']==1
    assert best['selected_stage']=='minimum_length'
    assert best['final_length_mm']==pytest.approx(100.05)
    assert best['minimum_length_min_mm']==pytest.approx(100.)
    # A shorter secondary discovery must update the global best-found length.
    rows[1]['x']=json.dumps(design(99.8,50.))
    best=runner.best_rows(rows,.1)[0]
    assert best['minimum_length_min_mm']==pytest.approx(99.8)


def test_single_objective_fast_front_matches_general_nondomination():
    from core.optimization.study import non_dominated
    from pymoo.util.nds.non_dominated_sorting import NonDominatedSorting
    for values in [np.array([[3.],[1.],[1.],[2.]]),np.zeros((500,1)),np.array([[-2.]])]:
        expected=NonDominatedSorting().do(values,only_non_dominated_front=True)
        np.testing.assert_array_equal(non_dominated(values),expected)


def test_belt_joint_can_move_beyond_legacy_relative_box_with_shaft_fixed():
    from core.optimization.analysis.compact import compact_config
    ref=next(r for r in shared.load_references()if r.reference_id=='belt_2r_matched')
    base=mechanism(ref.mechanism);part=shared.partitions.partition(ref.mechanism)
    carried=with_modules(ref.config,shared.catalog()['long'].modules)
    cfg=compact_config(base,ref.x,carried,(40,160))
    e={**part.external(ref.x,cfg),'finger_length_mm':150.,'proximal_span_mm':75.,
       'distal_span_mm':45.,'articulation_mm':90.}
    x=part.compose(e,{},ref.x,cfg)
    assert x is not None
    v,v0=part.values(x),part.values(ref.x)
    assert v['a_x_mm']+v['ap_from_a_x_mm']==pytest.approx(v0['a_x_mm']+v0['ap_from_a_x_mm'])
    assert v['ap_from_a_x_mm'] < -20
    assert cfg.pulley_radius_mm==carried.pulley_radius_mm
    # Resolved configs must round-trip the enlarged relation, not silently rebuild it.
    restored=base.config_from_json(base.config_to_json(cfg))
    assert restored.bounds==cfg.bounds


def test_archived_warm_vectors_replay_without_original_optimizer_outputs(tmp_path):
    import json
    runner=shared.load_module('compact_replay_run',ROOT/'scripts/study/study2_compact_morphology/run.py')
    ref=shared.load_references()[0];target=shared.catalog()['long']
    source=tmp_path/'archived'/target.module_id/ref.reference_id
    source.mkdir(parents=True)
    recorded=[dict(x=list(ref.x),source='archived original candidate')]
    (source/'warm_start.json').write_text(json.dumps(recorded))
    destination=tmp_path/'new';destination.mkdir();hashes={}
    cfg={'warm_start':{'replay':str(tmp_path/'archived'),'study2':'missing-old-output','max_designs':40}}
    vectors,provenance=runner.warm_designs(ref,target,cfg,destination,hashes)
    assert vectors==[ref.x]
    assert provenance==recorded
    assert hashes
    copy=destination/'inputs/warm_replay'/target.module_id/ref.reference_id/'warm_start.json'
    assert json.loads(copy.read_text())==recorded


def test_secondary_shape_cost_is_not_masked_by_primary_total_length_change():
    from types import SimpleNamespace
    ref=next(r for r in shared.load_references()if r.reference_id=='2r_dd_tilt0_matched')
    cfg=with_finger_lengths(with_modules(ref.config,shared.catalog()['long_thick'].modules),40,160)
    part=shared.partitions.partition(ref.mechanism)
    wrapper=CompactMechanism(mechanism(ref.mechanism),part,ref.x,cfg,'shape_change',120.8)
    e0=part.external(ref.x,cfg)
    a={**e0,'finger_length_mm':120.7,'proximal_span_mm':e0['proximal_span_mm']+5.5,
       'distal_span_mm':e0['distal_span_mm']+20.6,'articulation_mm':e0['articulation_mm']+15}
    b={**a,'distal_span_mm':e0['distal_span_mm']+4.8}
    assert wrapper.shape_cost(SimpleNamespace(external=a))==pytest.approx(20.6/90)
    assert wrapper.shape_cost(SimpleNamespace(external=b))==pytest.approx(15/90)
    # The original full eps_E would be identical (30.7/90) for both designs.
    from core.optimization.analysis.adaptation import change
    assert change(a,e0,90)==pytest.approx(change(b,e0,90))


def test_secondary_rerun_reuses_only_exact_matching_primary_and_checks_feasibility(tmp_path,monkeypatch):
    from core.optimization.analysis import compact
    from core.optimization.analysis.adaptation import Step,save_step
    from core.optimization.optimizer import OptimizerSettings
    from core.optimization.study import Candidate,StudyResult
    ref=next(r for r in shared.load_references()if r.reference_id=='2r_dd_tilt90_matched')
    base=mechanism(ref.mechanism);part=shared.partitions.partition(ref.mechanism)
    carried=with_modules(ref.config,shared.catalog()['baseline'].modules)
    cfg=compact.compact_config(base,ref.x,carried,(40,160))
    settings=OptimizerSettings('nsga2',4,1,0)
    wrapper=CompactMechanism(base,part,ref.x,cfg)
    e=wrapper.evaluate(ref.x,cfg)
    result=StudyResult(wrapper,cfg,settings,(Candidate(ref.x,e),),1,1)
    source=tmp_path/'primary'
    save_step(Step('minimum_length','found',ref.x,e,result=result),source)
    called=[]
    def secondary_only(model,config,settings,**kwargs):
        called.append(model.objective)
        assert model.objective=='shape_change'
        evaluated=model.evaluate(ref.x,config)
        return StudyResult(model,config,settings,(Candidate(ref.x,evaluated),),1,1)
    monkeypatch.setattr(compact,'run_study',secondary_only)
    steps=compact.search_compact(base,part,ref.x,carried,settings,(40,160),primary_run=source)
    assert called==['shape_change']
    assert steps[0].result is None
    assert steps[0].evaluation.feasible
    assert steps[1].status=='found'
    with pytest.raises(ValueError,match='config/settings mismatch'):
        compact.search_compact(base,part,ref.x,carried,settings,(40,180),primary_run=source)
