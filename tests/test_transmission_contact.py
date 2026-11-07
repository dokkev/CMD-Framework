"""Focused scientific contracts: independent FD, energy, quadrature, bridge limits."""
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts/study'))
import shared
from core.optimization.analysis.transmission import Transmission, reflect, rotate
from core.optimization.analysis.bridge import screen, attachment


@pytest.mark.parametrize('reference', shared.load_references(), ids=lambda r: r.reference_id)
def test_analytic_jacobian_against_existing_servo_fk(reference):
    t = Transmission(reference.mechanism, reference.x, reference.config)
    for phi in np.radians([0, 35, 80, 115]):
        for side in ['proximal', 'distal']:
            for fraction in [.1, .5, .9]:
                np.testing.assert_allclose(t.jacobian(phi, side, fraction, .3),
                    t.finite_difference(phi, side, fraction, .3, step=2e-6), atol=1e-8, rtol=1e-6)


def test_reflection_energy_psd_and_node_count_invariance():
    j = np.array([[.03, -.01], [.07, .02]])
    n = np.array([0., -1.])
    k, b = reflect([j], [n], 1000., 200., 2., .4)
    dq = np.array([.2, -.3])
    dx = j @ dq
    assert dq @ k @ dq == pytest.approx(dx @ np.diag([200.,1000.]) @ dx)
    np.testing.assert_allclose(b, .002*k)
    assert np.linalg.eigvalsh(k).min() >= 0
    k5, b5 = reflect([j]*5,[n]*5,1000.,200.,2.,.4)
    np.testing.assert_allclose(k,k5)
    np.testing.assert_allclose(b,b5)
    with pytest.raises(ValueError):
        reflect([j],[n],-1,0,1,0)


def test_pin_jacobian_known_lever_and_world_rotation():
    r = next(r for r in shared.load_references() if r.reference_id=='belt_2r_matched')
    t = Transmission(r.mechanism,r.x,r.config)
    p,_ = t.contact(0,'distal',.5)
    a = np.array([t.geo.a_x_mm*.001,0.])
    j = t.jacobian(0,'distal',.5)
    np.testing.assert_allclose(j[:,1], [p[1]-a[1], -(p[0]-a[0])], atol=1e-14)
    k,_ = reflect([j],[[0.,-1.]],1000,200,1,.2)
    angle=.7
    jr = t.jacobian(0,'distal',.5,angle)
    kr,_ = reflect([jr],[rotate([0.,-1.],-angle)],1000,200,1,.2)
    np.testing.assert_allclose(k,kr,atol=1e-14)


def test_bridge_neutral_and_straight_chord_bound():
    p,d = np.array([.04,-.008]),np.array([.06,-.008])
    m,c = screen(p,d,0,.02)
    assert m['arc_m'] == pytest.approx(.02)
    assert m['arc_change_fraction'] == pytest.approx(0)
    for angle in [.3,1.,2.]:
        metrics,_ = screen(p,d,angle,.02)
        assert metrics['arc_m'] >= metrics['chord_m']-1e-14


def test_matched_bridge_datums_and_curve_resolution():
    refs={r.reference_id:r for r in shared.load_references()}
    d=np.array([.0598,-.008]); p=np.array([.0433,-.008])
    for name in ['code_matched','belt_2r_matched']:
        r=refs[name]; t=Transmission(r.mechanism,r.x,r.config)
        np.testing.assert_allclose(attachment(t,0,d,'distal'),d,atol=1e-14)
        for phi in [.5,1.,2.]:
            moved=attachment(t,phi,d,'distal')
            m,_=screen(p,moved,phi,.0165,401)
            fine,_=screen(p,moved,phi,.0165,801)
            assert abs(m['arc_m']-fine['arc_m']) < 2e-7


def test_objective_ties_cover_design_space_and_preserve_extremes():
    run = shared.load_module('study3_run', ROOT/'scripts/study/study3_transmission/run.py')
    selected = run.cover([[0],[0],[0],[0]], 3, [[0],[1],[2],[3]])
    assert selected[:2] == [0, 3]
    assert selected[2] in (1, 2)  # symmetric geometric tie, floating-point ordering
    chosen = run.cover([[0,1],[.5,.5],[1,0]],2)
    assert chosen == [0,2]
