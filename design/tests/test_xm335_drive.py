"""The XM335 datasheet values and the drive law built on them."""

import math

import pytest

from core.hardware.xm335 import XM335Datasheet, XM335DriveModel


@pytest.mark.parametrize("voltage,torque,rpm", [(9.0, 0.85, 43.0), (11.1, 1.03, 53.0), (12.0, 1.12, 57.0)])
def test_rated_points_are_reproduced(voltage, torque, rpm):
    d = XM335Datasheet()
    assert d.stall_torque_at(voltage) == pytest.approx(torque)
    assert d.no_load_speed_at(voltage) == pytest.approx(rpm * 2.0 * math.pi / 60.0)


def test_voltage_outside_the_rating_is_rejected():
    with pytest.raises(ValueError):
        XM335Datasheet().stall_torque_at(13.0)


def test_drive_law_follows_the_torque_speed_curve():
    m = XM335DriveModel()
    big = 10.0 * m.saturation_error_rad
    assert m.torque(big, 0.0) == pytest.approx(m.stall_torque_nm)  # stall
    assert m.torque(big, m.no_load_speed_rad_s) == pytest.approx(0.0, abs=1e-12)  # no-load speed
    assert m.torque(-big, 0.0) == pytest.approx(-m.stall_torque_nm)
    # Below saturation: the stiffness, and back-EMF damping at any speed.
    e = 0.1 * m.saturation_error_rad
    assert m.torque(e, 0.0) == pytest.approx(m.stiffness_nm_per_rad * e)
    assert m.torque(e, 1.0) - m.torque(e, 0.0) == pytest.approx(-m.back_emf_damping_nms_per_rad)


def test_default_gain_saturates_at_about_five_degrees():
    # K_PP = 2000 / 128 PWM LSB per pulse, 885 LSB at the limit (assumed units: see the module docstring).
    assert math.degrees(XM335DriveModel().saturation_error_rad) == pytest.approx(885 / (2000 / 128) * 360 / 4096)
