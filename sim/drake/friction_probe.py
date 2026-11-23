"""Does Drake apply the set friction to a cylinder on a pad, with point and hydroelastic contact? (No hand.)

A cylinder (radius 15 mm, axis y, rotation locked: it may only slide in x and z) is pressed onto a fixed pad (the
hand's pad box) by a constant force N, then pushed along x by a force that ramps up. mu_eff = the push at which it
slides (speed > 10 mm/s) / N, read from the applied forces alone. Same materials as two_pad_drake.py.

    uv run python friction_probe.py
"""
import math

import numpy as np

import two_pad_drake as T


def run(contact: str, mu: float, modulus: float, press: float = 1.0) -> float | None:
    from pydrake.geometry import (AddCompliantHydroelasticProperties, AddContactMaterial, AddRigidHydroelasticProperties,
                                  Box, Cylinder, ProximityProperties)
    from pydrake.math import RigidTransform, RotationMatrix
    from pydrake.multibody.plant import (AddMultibodyPlantSceneGraph, ContactModel, CoulombFriction,
                                         DiscreteContactApproximation)
    from pydrake.multibody.tree import PrismaticJoint, SpatialInertia, UnitInertia
    from pydrake.systems.analysis import Simulator
    from pydrake.systems.framework import DiagramBuilder

    builder = DiagramBuilder()
    plant, sg = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    plant.set_contact_model(ContactModel.kHydroelasticWithFallback if contact == "hydro" else ContactModel.kPoint)
    w = plant.world_body()
    h = 0.007
    cx = plant.AddRigidBody("cx", SpatialInertia(0.002, np.zeros(3), UnitInertia.SolidSphere(0.005)))
    cyl = plant.AddRigidBody("cyl", SpatialInertia(T.MASS_KG, np.zeros(3), UnitInertia.SolidCylinder(0.015, T.LENGTH_M, [0, 1, 0])))
    jx = plant.AddJoint(PrismaticJoint("x", w.body_frame(), cx.body_frame(), [1, 0, 0]))
    jz = plant.AddJoint(PrismaticJoint("z", cx.body_frame(), cyl.body_frame(), [0, 0, 1]))

    def props(kind):
        pr = ProximityProperties()
        if contact == "hydro":
            if kind == "pad":
                AddCompliantHydroelasticProperties(resolution_hint=0.002, hydroelastic_modulus=modulus / 0.003 * h / 2,
                                                   properties=pr)
            else:
                AddRigidHydroelasticProperties(resolution_hint=0.0005, properties=pr)
            AddContactMaterial(friction=CoulombFriction(mu, mu), properties=pr)
        else:
            AddContactMaterial(point_stiffness=T.POINT_STIFFNESS if kind == "pad" else 1e8,
                               friction=CoulombFriction(mu, mu), properties=pr)
        return pr

    plant.RegisterCollisionGeometry(w, RigidTransform([0, 0, -h / 2]), Box(0.044, 0.019, h), "pad", props("pad"))
    plant.RegisterCollisionGeometry(cyl, RigidTransform(RotationMatrix.MakeXRotation(math.pi / 2)),
                                    Cylinder(0.015, T.LENGTH_M), "cyl", props("cyl"))
    plant.Finalize()
    plant.mutable_gravity_field().set_gravity_vector(np.zeros(3))
    diagram = builder.Build()
    sim = Simulator(diagram)
    ctx = sim.get_mutable_context()
    pctx = plant.GetMyMutableContextFromRoot(ctx)
    jz.set_translation(pctx, 0.015)
    port = plant.get_applied_generalized_force_input_port()
    t = 0.0
    for k in range(int(1.0 / 0.01)):  # press for 1 s
        port.FixValue(pctx, np.array([0.0, -press]))
        t += 0.01
        sim.AdvanceTo(t)
    for k in range(int(4.0 / 0.01)):  # push up to 3 mu N over 4 s
        push = 3.0 * mu * press * (k + 1) / 400
        port.FixValue(pctx, np.array([push, -press]))
        t += 0.01
        sim.AdvanceTo(t)
        if abs(plant.GetVelocities(pctx)[0]) > 0.01:
            return push / press
    return None


if __name__ == "__main__":
    for contact, modulus in (("point", None), ("hydro", 1.0e6), ("hydro", 1.0e5)):
        for mu in (1.3, 2.0):
            r = run(contact, mu, modulus or 0.0)
            print(f"{contact:5s} E {modulus} mu {mu}: mu_eff {r}")
