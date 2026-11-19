
import pytest

from core.hardware import (
    ACTUATOR_SHAFT_AXIS_XYZ,
    XM335Actuator,
)


def test_xm335_geometry_frame_and_project_parameters() -> None:
    actuator = XM335Actuator()
    geometry = actuator.geometry

    assert geometry.radius_mm == pytest.approx(9.5)
    assert geometry.rear_extent_mm == pytest.approx(25.5)
    assert geometry.depth_mm == pytest.approx(22.0)
    assert geometry.longitudinal_extent_mm == pytest.approx(35.0)
    assert geometry.width_mm == pytest.approx(19.0)
    bounds = [value for axis_bounds in geometry.bounds_xyz_mm for value in axis_bounds]
    assert bounds == pytest.approx([-9.5, 9.5, -25.5, 9.5, -11.0, 11.0])
    assert actuator.shaft_axis_xyz == pytest.approx((0.0, 0.0, 1.0))
    assert ACTUATOR_SHAFT_AXIS_XYZ == pytest.approx((0.0, 0.0, 1.0))
    assert actuator.mechanics.gear_ratio == pytest.approx(323.04)
    assert actuator.mechanics.max_output_torque_nm == pytest.approx(1.0)
    assert actuator.mechanics.mass_g == pytest.approx(27.0)


def test_the_compact_servos_build_a_prototype():
    from pathlib import Path

    from core.hardware import ACTUATORS, actuator
    from core.optimization.mechanism import mechanism
    from core.optimization.study import load_yaml

    config_path = Path(__file__).resolve().parents[1] / "scripts" / "optimize" / "prototype" / "config.yaml"
    for name in ("xc330", "feetech_hd1910", "unitree_j288"):
        body = actuator(name).geometry
        assert body.width_mm == 20.0 and body.longitudinal_extent_mm == 34.0
        config = load_yaml(mechanism("linkage"), config_path,
                           (f"goal.hardware.pip_servo.model={name}", f"goal.hardware.mcp_servo.model={name}"))[0]
        assert config.packaging.pip_servo == body and config.packaging.mcp_servo == body and name in ACTUATORS
        # The CAD's design vector evaluates with it (feasible or not).
        x = (35.7, 67.2, 0.0, 37.5, 12.4, 60.5, 6.5, 15.0, 22.8, 0.0, 5.8, 0.7, 4.0, 4.0)
        assert mechanism("linkage").evaluate(x, config) is not None


def test_a_box_body_is_a_rectangle_about_its_offset_shaft():
    import numpy as np

    from core.hardware import actuator
    from core.morphology.packaging.shapes import servo_footprint

    body = actuator("xc330").geometry
    assert (body.radius_mm, body.front_extent_mm, body.rear_extent_mm, body.cap_radius_mm) == (10.0, 9.5, 24.5, 0.0)
    # Shaft at the origin, rear along -x: the box spans x in [-24.5, 9.5], z in [-10, 10].
    box = servo_footprint(body, (0.0, 0.0), (-1.0, 0.0))
    xs, zs = box.outline[:, 0], box.outline[:, 1]
    assert (xs.min(), xs.max(), zs.min(), zs.max()) == (-24.5, 9.5, -10.0, 10.0)
    points = np.array([(12.5, 0.0), (9.5 + 3.0, 10.0 + 4.0), (0.0, 0.0), (-30.0, 0.0)])
    distance = box.distance_to_segments(points, points)
    assert distance == pytest.approx([3.0, 5.0, 0.0, 5.5])
    # No rounded cap: a point 9 mm ahead at 9.5 mm aside is outside a D (radius 10) but inside the box.
    corner = np.array([(9.0, 9.4)])
    assert box.distance_to_segments(corner, corner)[0] == 0.0
    d_shape = servo_footprint(actuator("xm335").geometry, (0.0, 0.0), (-1.0, 0.0))
    assert d_shape.distance_to_segments(corner, corner)[0] > 0.0
