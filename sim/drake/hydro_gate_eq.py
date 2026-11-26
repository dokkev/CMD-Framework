"""Post hoc (drake_hydro.md, 38/39): O pad hydro force at a given interpenetration.
    uv run python hydro_gate_eq.py 5 6 7 8 12.5 15 17.5 20
"""
# Hydro force of O's pad (box 44 x 19 x 3 mm, E = 0.1 MPa / 3 mm * h/2, hint 2 mm) against a rigid cylinder (hint 0.5 mm)
# pressed delta into the pad face: Drake's own contact surface, and the Winkler estimate k * w * segment area.
import math, sys
import numpy as np
from pydrake.geometry import (SceneGraph, Box, Cylinder, GeometryInstance, ProximityProperties, MakePhongIllustrationProperties,
                              AddCompliantHydroelasticProperties, AddRigidHydroelasticProperties, GeometryFrame, FramePoseVector,
                              HydroelasticContactRepresentation)
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantValueSource
from pydrake.common.value import AbstractValue

def force(rho_mm, delta_mm, L=0.044, w=0.019, h=0.003, cyl_len=0.0233333):
    sg = SceneGraph()
    src = sg.RegisterSource("s")
    fpad = sg.RegisterFrame(src, GeometryFrame("pad"))
    fcyl = sg.RegisterFrame(src, GeometryFrame("cyl"))
    pp = ProximityProperties(); AddCompliantHydroelasticProperties(0.002, 1e5 / 0.003 * h / 2, pp)
    gp = GeometryInstance(RigidTransform(), Box(L, w, h), "pad"); gp.set_proximity_properties(pp)
    sg.RegisterGeometry(src, fpad, gp)
    pc = ProximityProperties(); AddRigidHydroelasticProperties(0.0005, pc)
    gc = GeometryInstance(RigidTransform(RotationMatrix.MakeXRotation(math.pi / 2)), Cylinder(rho_mm * 1e-3, cyl_len), "cyl")
    gc.set_proximity_properties(pc)
    sg.RegisterGeometry(src, fcyl, gc)
    b = DiagramBuilder(); b.AddSystem(sg)
    fp = FramePoseVector()
    fp.set_value(fpad, RigidTransform())
    fp.set_value(fcyl, RigidTransform([0, 0, h / 2 + rho_mm * 1e-3 - delta_mm * 1e-3]))
    srcsys = b.AddSystem(ConstantValueSource(AbstractValue.Make(fp)))
    b.Connect(srcsys.get_output_port(0), sg.get_source_pose_port(src))
    d = b.Build(); ctx = d.CreateDefaultContext()
    q = sg.get_query_output_port().Eval(sg.GetMyContextFromRoot(ctx))
    surfs = q.ComputeContactSurfaces(HydroelasticContactRepresentation.kPolygon)
    F = 0.0
    for s in surfs:
        m = s.poly_mesh_W(); e = s.poly_e_MN()
        for i in range(m.num_elements()):
            c = m.element_centroid(i)
            F += e.EvaluateCartesian(i, c) * m.area(i) * abs(m.face_normal(i)[2])
    k = 1e5 / 0.003
    r, dl = rho_mm * 1e-3, delta_mm * 1e-3
    seg = r * r * math.acos((r - dl) / r) - (r - dl) * math.sqrt(2 * r * dl - dl * dl)
    return F, k * w * seg

for rho in [float(x) for x in sys.argv[1:]]:
    for dl in (1.0, 1.5):
        F, Fw = force(rho, dl)
        print(f"rho {rho:5.1f} mm delta {dl} mm: Drake {F:6.2f} N, Winkler {Fw:6.2f} N, point {2.5 * dl:.2f} N")
