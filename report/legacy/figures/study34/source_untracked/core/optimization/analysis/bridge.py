"""Prescribed bridge geometry screening, not a skin equilibrium/material solver."""
import numpy as np
from core.optimization.analysis.transmission import rotate
from core.morphology.packaging.shapes import point_segment_distance, segment_polygon_gaps, segments_cross


def attachment(transmission, phi, neutral_point_m, side):
    """Move a common neutral datum rigidly with its supporting phalanx."""
    if side == 'proximal':
        return np.asarray(neutral_point_m)
    p0 = transmission.contact(0., side, 0.)[0]
    p = transmission.contact(phi, side, 0.)[0]
    return p + rotate(np.asarray(neutral_point_m) - p0, -phi)


def bridge_curve(p, d, phi, tangent_length, samples=201):
    """Cubic Hermite path, tangent to both support surfaces; common neutral span.

    Tangent magnitude is held at the neutral span for both mechanisms. This
    choice is a geometric construction, NOT a required skin path or minimum
    energy solution. Chord length alone is a lower bound on an unconstrained path.
    """
    t = np.linspace(0., 1., samples)[:, None]
    m0 = np.array([tangent_length, 0.])
    m1 = rotate(m0, -phi)
    curve = ((2*t**3 - 3*t**2 + 1)*p + (t**3 - 2*t**2 + t)*m0
             + (-2*t**3 + 3*t**2)*d + (t**3 - t**2)*m1)
    return curve


def screen(p, d, phi, neutral_span, samples=201):
    curve = bridge_curve(p, d, phi, neutral_span, samples)
    delta = d - p
    chord = float(np.linalg.norm(delta))
    arc = float(np.linalg.norm(np.diff(curve, axis=0), axis=1).sum())
    # Discrete diagnostics detect a pathological prescribed path before any
    # interpretation as a usable surface. Not an elastic buckling calculation.
    segments = np.diff(curve, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    angles = np.unwrap(np.arctan2(segments[:, 1], segments[:, 0]))
    turn = np.abs(np.diff(angles))
    pairs = np.triu_indices(len(segments), k=2)
    intersections = segments_cross(curve[pairs[0]], curve[pairs[0]+1],
                                   curve[pairs[1]], curve[pairs[1]+1])
    return dict(chord_m=chord, arc_m=arc,
                path_self_intersects=bool(intersections.any()),
                min_segment_m=float(lengths.min()),
                max_local_turn_deg=float(np.degrees(turn.max(initial=0))), arc_change_fraction=arc / neutral_span - 1,
                chord_change_fraction=chord / neutral_span - 1,
                gap_along_proximal_m=float(delta[0]), step_m=float(delta[1]),
                normal_discontinuity_deg=float(np.degrees(abs(phi)))), curve


def hardware_gap_mm(scene, curve_m, skin_radius_mm=.5, link_radius_mm=.5):
    """Projected capsule clearance to existing servo/pivot/link geometry.

    A conservative side-view routing flag; layer/width resolution and the
    supporting skin/pad material are not modelled. Pads intentionally excluded:
    the common attachment datums describe a hypothetical extended contact skin.
    """
    curve = np.asarray(curve_m) * 1000
    start, end = curve[:-1], curve[1:]
    gaps = []
    for category in ('servo', 'mcp_servo', 'pin', 'link', 'pulley', 'roller', 'carrier'):
        shapes = scene.solids['proximal_pad'][category]
        for polygon in shapes.polygons:
            polygon = np.asarray(polygon)
            if polygon.ndim == 3:
                polygon = polygon[0]
            polygons = np.broadcast_to(polygon, (len(start), *polygon.shape))
            gaps.append(float(segment_polygon_gaps(polygons, start, end).min()))
        for center, radius in shapes.disks:
            center = np.asarray(center).reshape(-1, 2)[0]
            gaps.append(float(point_segment_distance(center, start, end).min()) - radius)
        for segment in shapes.segments:
            a, b = np.asarray(segment).reshape(-1, 2, 2)[0]
            distance = np.minimum.reduce([point_segment_distance(a, start, end),
                        point_segment_distance(b, start, end),
                        point_segment_distance(start, a, b), point_segment_distance(end, a, b)])
            distance[segments_cross(start, end, a, b)] = 0.
            gaps.append(float(distance.min()) - link_radius_mm)
    return min(gaps, default=float('inf')) - skin_radius_mm
