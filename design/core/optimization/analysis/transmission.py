"""Local contact reflection at actuator *output shafts* (m, rad, N, s).

World is the XZ side-view frame, MCP at O; +X distal, +Z dorsal.
Positive flexion turns +X toward -Z. Columns are MCP and PIP servo angles.
FK and contact faces are the existing mechanism implementations, in mm at their
boundary. No gear ratio inside the servo, controller, inertia or preload model.
"""
from __future__ import annotations

import importlib
import numpy as np

from core.morphology.contact.loss import joint_scene, linkage_scene
from core.morphology.mechanism import base_mechanism
from core.morphology.mechanism.linkage.design import decode
from core.optimization.analysis.tradeoff import goal_of


def perp(v):
    """Positive (counterclockwise) XZ quarter turn."""
    v = np.asarray(v)
    return np.stack((-v[..., 1], v[..., 0]), axis=-1)


def rotate(v, angle):
    return np.asarray(v) * np.cos(angle) + perp(v) * np.sin(angle)


class Transmission:
    """Thin adapter around existing FK/solids, not a second closure solver."""

    def __init__(self, name, x, config):
        self.name, self.x, self.config = name, tuple(x), config
        self.packaging = goal_of(config).packaging
        self.finger = decode(x, config.link_length_resolution_mm) if name == 'linkage' else None
        self.geo = None if self.finger else importlib.import_module(
            f'core.optimization.{base_mechanism(name)}.evaluate').geometry(x, config)

    def scene(self, phi):
        """Exact existing scene at distal flexion phi (rad), MCP=0."""
        if self.finger:
            s = self.finger.sweep_pip([phi])
            if len(s.q_pip_rad) != 1:
                raise ValueError('four-bar cannot close at requested flexion')
            return linkage_scene(self.finger, self.packaging, s.b, s.tip, s.bp, [np.degrees(phi)])
        return joint_scene(self.name, self.geo, self.packaging, [np.degrees(phi)])

    def servo(self, phi):
        if self.finger:
            return float(self.finger.sweep_pip([phi]).servo_rad[0])
        return phi / 2 if base_mechanism(self.name) == 'rcj' else phi

    def phi_from_servo(self, servo):
        if self.finger:
            s = self.finger.sweep_servo([servo])
            if len(s.q_pip_rad) != 1:
                raise ValueError('four-bar cannot close at requested servo angle')
            return float(s.q_pip_rad[0])
        return 2 * servo if base_mechanism(self.name) == 'rcj' else servo

    def contact(self, phi, side, fraction, mcp=0.):
        """Material point from outer (0) to joint-facing (1) end; outward normal."""
        faces, normals = self.scene(phi).faces[side + '_pad']
        p = faces[0, 0] + fraction * (faces[0, 1] - faces[0, 0])
        return rotate(p * .001, -mcp), rotate(normals[0], -mcp)

    def jacobian(self, phi, side, fraction, mcp=0.):
        """Analytic d(contact position)/d(output-shaft angles), m/rad.

        Four-bar: differentiate |B-A'|, |B'-A|, |B-B'| at the existing
        FK solution. RCJ: carrier angle = servo, distal angle = 2 servo.
        """
        p, _ = self.contact(phi, side, fraction)
        velocity = np.zeros(2)
        if side == 'distal':
            if self.finger:
                s = self.finger.sweep_pip([phi])
                a, ap = self.finger.joint.a.array * .001, self.finger.joint.ap.array * .001
                b, bp = s.b[0] * .001, s.bp[0] * .001
                db = -perp(b - ap)
                u, v = bp - a, b - bp
                denominator = np.dot(v, perp(u))
                if abs(denominator) < 1e-12:
                    raise ValueError('singular four-bar differential closure')
                dbp = perp(u) * np.dot(v, db) / denominator
                dv = db - dbp
                omega = (v[0] * dv[1] - v[1] * dv[0]) / np.dot(v, v)
                velocity = db + omega * perp(p - b)
            elif base_mechanism(self.name) == 'rcj':
                b = self.geo.b([phi])[0] * .001
                velocity = -perp(b - self.geo.a * .001) - 2 * perp(p - b)
            else:
                a = np.array([self.geo.a_x_mm, 0.]) * .001
                velocity = -perp(p - a)
        return rotate(np.column_stack((-perp(p), velocity)).T, -mcp).T

    def finite_difference(self, phi, side, fraction, mcp=0., step=1e-6):
        servo = self.servo(phi)
        first = (self.contact(phi, side, fraction, mcp + step)[0]
                 - self.contact(phi, side, fraction, mcp - step)[0]) / (2 * step)
        second = (self.contact(self.phi_from_servo(servo + step), side, fraction, mcp)[0]
                  - self.contact(self.phi_from_servo(servo - step), side, fraction, mcp)[0]) / (2 * step)
        return np.column_stack((first, second))


def reflect(jacobians, normals, normal_stiffness, tangent_stiffness,
            normal_damping, tangent_damping, weights=None):
    """Sum independent fixed-object point impedances, frozen normals, zero load.

    Weights sum to one: distributed quadrature preserves the *total* interface
    stiffness/damping of a point model, rather than multiplying by node count.
    Returns K_a (N m/rad), B_a (N m s/rad). Z_a(s)=B_a+K_a/s for torque/velocity.
    """
    if min(normal_stiffness, tangent_stiffness, normal_damping, tangent_damping) < 0:
        raise ValueError('contact stiffness and damping must be nonnegative')
    jacobians, normals = np.asarray(jacobians), np.asarray(normals)
    if weights is None:
        weights = np.full(len(jacobians), 1 / len(jacobians))
    weights = np.asarray(weights)
    if np.any(weights < 0) or not np.isclose(weights.sum(), 1):
        raise ValueError('quadrature weights must be nonnegative and sum to one')
    k, b = np.zeros((2, 2)), np.zeros((2, 2))
    for j, n, w in zip(jacobians, normals, weights, strict=True):
        if not np.isclose(n @ n, 1):
            raise ValueError('contact normal must be unit length')
        nn = np.outer(n, n)
        kc = tangent_stiffness * np.eye(2) + (normal_stiffness - tangent_stiffness) * nn
        bc = tangent_damping * np.eye(2) + (normal_damping - tangent_damping) * nn
        k += w * j.T @ kc @ j
        b += w * j.T @ bc @ j
    return k, b
