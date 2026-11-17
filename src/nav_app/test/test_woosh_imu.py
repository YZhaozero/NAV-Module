"""WOOSH's g-valued IMU must reach DLIO in SI without losing timing."""
import pytest
from sensor_msgs.msg import Imu

from nav_app.woosh_imu import GRAVITY, normalize_imu


def test_si_conversion_preserves_time_and_sensor_local_gyro():
    raw = Imu()
    raw.header.frame_id = "livox_frame"
    raw.header.stamp.sec, raw.header.stamp.nanosec = 123, 456
    raw.linear_acceleration.x, raw.linear_acceleration.z = 0.1, 1.0
    raw.angular_velocity.x, raw.angular_velocity.y = 0.2, -0.3
    raw.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.03]
    converted = normalize_imu(raw)
    assert converted.linear_acceleration.z == pytest.approx(GRAVITY)
    assert converted.linear_acceleration.x == pytest.approx(0.1 * GRAVITY)
    assert converted.linear_acceleration_covariance[4] == pytest.approx(0.02 * GRAVITY ** 2)
    assert converted.angular_velocity == raw.angular_velocity
    assert converted.header.stamp == raw.header.stamp
    assert converted.header.frame_id == "woosh_livox3_imu"
    assert converted.orientation_covariance[0] == -1
    assert raw.linear_acceleration.z == 1.0
    assert raw.header.frame_id == "livox_frame"


def test_unknown_acceleration_covariance_stays_unknown():
    raw = Imu()
    raw.header.frame_id = "livox_frame"
    raw.linear_acceleration_covariance[0] = -1
    assert normalize_imu(raw).linear_acceleration_covariance[0] == -1


def test_wrong_source_frame_is_reported():
    with pytest.raises(ValueError, match="Unexpected WOOSH IMU frame"):
        normalize_imu(Imu())
