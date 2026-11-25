"""Hydroelastic contact for the fingertip-task modules that build their own plant with point contact (experimental;
the point-vs-hydro rerun after experiments 30b-41). Import and call enable() before a run: no task module is edited.

What changes (the setup of experiments 21/25, two_pad_drake.py --contact hydro --pad_kh_mm own):
- every plant uses ContactModel.kHydroelasticWithFallback;
- a geometry registered with the pad's point stiffness (T.POINT_STIFFNESS) becomes a compliant hydroelastic body with
  modulus MODULUS_PA / h * h / 2 (experiment 25's "own" rule: a pad as soft as its own thickness, i.e. 0.05 MPa on
  every hand), resolution hint 2 mm; every other geometry (objects, tables, rails) is rigid hydroelastic, hint 0.5 mm.
  Friction is kept; Drake's default dissipation, as in 21 and 25;
- the plant's contact results are read through a view that lists each hydroelastic contact as one point pair (the
  surface's centroid, its total force, its area-weighted normal), so the modules' point-pair loops see every contact.

The modules' over-press gate stays force / 2.5 N/mm: under hydro it is a force gate (3.75 N), not a penetration.
"""
from __future__ import annotations

import numpy as np

import two_pad_drake as T

_ON = False


class _Pair:
    def __init__(self, a, b, f_on_b, p, n_ba):
        self._a, self._b, self._f, self._p, self._n = a, b, f_on_b, p, n_ba

    def bodyA_index(self):
        return self._a

    def bodyB_index(self):
        return self._b

    def contact_force(self):
        return self._f

    def contact_point(self):
        return self._p

    def point_pair(self):
        return self

    @property
    def nhat_BA_W(self):
        return self._n


class _Results:
    def __init__(self, res, plant, ctx):
        self._pairs = [res.point_pair_contact_info(k) for k in range(res.num_point_pair_contacts())]
        insp = plant.EvalSceneGraphInspector(ctx)
        for k in range(res.num_hydroelastic_contacts()):
            c = res.hydroelastic_contact_info(k)
            s = c.contact_surface()
            bm = plant.GetBodyFromFrameId(insp.GetFrameId(s.id_M())).index()
            bn = plant.GetBodyFromFrameId(insp.GetFrameId(s.id_N())).index()
            f_m = np.asarray(c.F_Ac_W().translational())  # acts on M's body
            n = sum(s.area(i) * np.asarray(s.face_normal(i)) for i in range(s.num_faces()))  # out of N into M
            n = n / (np.linalg.norm(n) + 1e-12)
            self._pairs.append(_Pair(bm, bn, -f_m, np.asarray(s.centroid()), n))

    def num_point_pair_contacts(self):
        return len(self._pairs)

    def point_pair_contact_info(self, k):
        return self._pairs[k]

    def num_hydroelastic_contacts(self):
        return 0


class _Port:
    def __init__(self, port, plant):
        self._port, self._plant = port, plant

    def Eval(self, ctx):
        return _Results(self._port.Eval(ctx), self._plant, ctx)

    def __getattr__(self, k):
        return getattr(self._port, k)


def enable():
    """Patch pydrake so the modules' own builders make hydroelastic plants (idempotent; per process)."""
    global _ON
    if _ON:
        return
    import pydrake.geometry as G
    from pydrake.multibody.plant import ContactModel, MultibodyPlant

    add_material = G.AddContactMaterial

    def material(*args, point_stiffness=None, properties=None, **kw):
        pad = point_stiffness is not None and abs(point_stiffness - T.POINT_STIFFNESS) < 1e-6
        if pad:
            G.AddCompliantHydroelasticProperties(resolution_hint=0.002, hydroelastic_modulus=T.MODULUS_PA / 2,
                                                 properties=properties)
        else:
            G.AddRigidHydroelasticProperties(resolution_hint=0.0005, properties=properties)
        return add_material(*args, properties=properties, **kw)

    G.AddContactMaterial = material
    set_model = MultibodyPlant.set_contact_model
    MultibodyPlant.set_contact_model = lambda self, m: set_model(self, ContactModel.kHydroelasticWithFallback)
    port = MultibodyPlant.get_contact_results_output_port
    MultibodyPlant.get_contact_results_output_port = lambda self: _Port(port(self), self)
    _ON = True
