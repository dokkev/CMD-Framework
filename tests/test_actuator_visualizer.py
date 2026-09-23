import pytest

from cmd.visualization.actuator_visualizer import ActuatorVisualizer


def test_xm335_visualizer_profile_has_rear_rectangle_and_forward_half_ellipse() -> None:
    visualizer = ActuatorVisualizer()
    radius_mm = visualizer.actuator.geometry.radius_mm
    rear_extent_mm = visualizer.actuator.geometry.rear_extent_mm

    profile = visualizer._profile_xz()
    arc = profile[1:-1]

    assert profile[0] == pytest.approx((-rear_extent_mm, -radius_mm))
    assert profile[-1] == pytest.approx((-rear_extent_mm, radius_mm))
    assert len(arc) == 33
    assert arc[0] == pytest.approx((0.0, -radius_mm))
    assert arc[16] == pytest.approx((radius_mm, 0.0))
    assert arc[-1] == pytest.approx((0.0, radius_mm))
    assert all(x_mm >= -1.0e-12 for x_mm, _ in arc)
