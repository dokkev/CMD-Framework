import math

import pytest

from core.morphology.open_chain import OpenChainFinger, OpenChainParameters

PARAMETERS = OpenChainParameters(proximal_length_mm=45.0, distal_length_mm=50.0, b_offset_mm=15.0)


def xz(point) -> tuple[float, float]:
    return (point.x_mm, point.z_mm)


def distances(pose) -> tuple[float, ...]:
    points = [xz(pose.o), xz(pose.a), xz(pose.b), xz(pose.ee)]
    return (math.dist(points[0], points[1]), math.dist(points[1], points[2]),
            math.dist(points[1], points[3]), math.dist(points[2], points[3]))


def direction(start, end) -> float:
    return math.atan2(end.z_mm - start.z_mm, end.x_mm - start.x_mm)


def test_at_zero_the_finger_lies_along_x() -> None:
    pose = OpenChainFinger(PARAMETERS).solve(q_mcp_rad=0.0, q_distal_rad=0.0)

    assert xz(pose.o) == (0.0, 0.0)
    assert xz(pose.a) == pytest.approx((45.0, 0.0))
    assert xz(pose.b) == pytest.approx((60.0, 0.0))
    assert xz(pose.ee) == pytest.approx((95.0, 0.0))
    assert distances(pose) == pytest.approx((45.0, 15.0, 50.0, 35.0))


def test_mcp_flexion_turns_the_whole_finger_rigidly() -> None:
    finger = OpenChainFinger(PARAMETERS)
    straight = finger.solve(0.0, 0.0)
    pose = finger.solve(math.radians(40.0), 0.0)

    # +X turns toward -Z, all points together.
    assert pose.a.z_mm < 0.0 and pose.ee.z_mm < 0.0
    for point in ("a", "b", "ee"):
        assert direction(pose.o, getattr(pose, point)) == pytest.approx(-math.radians(40.0))
    assert distances(pose) == pytest.approx(distances(straight))


def test_distal_flexion_turns_b_and_ee_about_a() -> None:
    finger = OpenChainFinger(PARAMETERS)
    straight = finger.solve(0.0, 0.0)
    pose = finger.solve(0.0, math.radians(60.0))

    assert xz(pose.o) == pytest.approx(xz(straight.o))
    assert xz(pose.a) == pytest.approx(xz(straight.a))
    assert math.dist(xz(pose.a), xz(pose.b)) == pytest.approx(15.0)
    assert math.dist(xz(pose.a), xz(pose.ee)) == pytest.approx(50.0)
    assert direction(pose.a, pose.ee) == pytest.approx(-math.radians(60.0))


def test_the_joints_move_independently_and_add_up() -> None:
    finger = OpenChainFinger(PARAMETERS)
    q_mcp, q_distal = math.radians(25.0), math.radians(50.0)
    pose = finger.solve(q_mcp_rad=q_mcp, q_distal_rad=q_distal)

    assert (pose.q_mcp_rad, pose.q_distal_rad) == (q_mcp, q_distal)
    assert pose.q_distal_absolute_rad == pytest.approx(q_mcp + q_distal)
    assert direction(pose.o, pose.a) == pytest.approx(-q_mcp)
    assert direction(pose.a, pose.ee) == pytest.approx(-(q_mcp + q_distal))
    # The frames agree with the points.
    assert pose.distal.translation_mm == pytest.approx((pose.a.x_mm, 0.0, pose.a.z_mm))
    assert pose.distal_joint.translation_mm == pytest.approx(pose.distal.translation_mm)
    for joint_row, proximal_row in zip(pose.distal_joint.rotation_matrix, pose.proximal.rotation_matrix):
        assert joint_row == pytest.approx(proximal_row)


@pytest.mark.parametrize("degrees", (90.0, 120.0, 150.0, 180.0))
def test_the_distal_joint_flexes_to_180_degrees(degrees: float) -> None:
    pose = OpenChainFinger(PARAMETERS).solve(0.0, math.radians(degrees))

    turned = direction(pose.a, pose.ee)
    assert math.remainder(turned + math.radians(degrees), 2.0 * math.pi) == pytest.approx(0.0, abs=1e-9)
    if degrees == 180.0:
        assert xz(pose.ee) == pytest.approx((45.0 - 50.0, 0.0), abs=1e-9)


@pytest.mark.parametrize("kwargs", (
    {"proximal_length_mm": 0.0, "distal_length_mm": 50.0},
    {"proximal_length_mm": 45.0, "distal_length_mm": -1.0},
    {"proximal_length_mm": 45.0, "distal_length_mm": 50.0, "b_offset_mm": -1.0},
    {"proximal_length_mm": 45.0, "distal_length_mm": 50.0, "b_offset_mm": 51.0},
    {"proximal_length_mm": math.nan, "distal_length_mm": 50.0},
    {"proximal_length_mm": 45.0, "distal_length_mm": math.inf},
))
def test_invalid_parameters_are_rejected(kwargs) -> None:
    with pytest.raises(ValueError):
        OpenChainParameters(**kwargs)
