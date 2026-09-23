import pytest

from cmd.visualization.actuator_visualizer import ActuatorVisualizer


def test_xm335_visualizer_profile_has_rear_rectangle_and_forward_half_ellipse() -> None:
    visualizer = ActuatorVisualizer()
    radius_mm = visualizer.actuator.geometry.radius_mm
    rear_extent_mm = visualizer.actuator.geometry.rear_extent_mm

    profile = visualizer._profile_xy()
    arc = profile[2:-1]

    assert profile[0] == pytest.approx((-radius_mm, -rear_extent_mm))
    assert profile[1] == pytest.approx((radius_mm, -rear_extent_mm))
    assert profile[-2] == pytest.approx((-radius_mm, 0.0))
    assert profile[-1] == pytest.approx((-radius_mm, -rear_extent_mm))
    assert len(arc) == 33
    assert arc[0] == pytest.approx((radius_mm, 0.0))
    assert arc[16] == pytest.approx((0.0, radius_mm))
    assert arc[-1] == pytest.approx((-radius_mm, 0.0))
    assert all(y_mm >= -1.0e-12 for _, y_mm in arc)
