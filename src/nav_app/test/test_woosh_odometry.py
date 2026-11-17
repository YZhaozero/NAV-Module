"""Rigid-body origin, orientation, velocity and timing of navigation odometry."""
import math
import numpy as np
import pytest
from geometry_msgs.msg import Transform
from nav_msgs.msg import Odometry
from nav_app.woosh_odometry import base_odometry


def test_base_origin_and_velocity_include_rotating_mount_offset():
    message = Odometry(); message.header.frame_id="odom"; message.child_frame_id="woosh_livox3_imu"
    message.header.stamp.sec=123; message.header.stamp.nanosec=456
    message.pose.pose.orientation.w=1.
    message.pose.pose.position.x=10.
    message.twist.twist.linear.x=1.; message.twist.twist.angular.z=2.
    mount=Transform(); mount.rotation.w=1.; mount.translation.x=.3
    result=base_odometry(message,mount)
    assert result.child_frame_id=="base_link" and result.header==message.header
    assert result.pose.pose.position.x==pytest.approx(10.3)
    assert result.twist.twist.linear.y==pytest.approx(.6)
    assert result.twist.twist.angular.z==2.
    assert message.pose.pose.position.x==10. and message.child_frame_id=="woosh_livox3_imu"


def test_world_pose_and_body_velocity_use_the_calibrated_rotation():
    message=Odometry(); message.header.frame_id="odom"; message.child_frame_id="woosh_livox3_imu"
    message.pose.pose.orientation.z=math.sqrt(.5); message.pose.pose.orientation.w=math.sqrt(.5)
    message.twist.twist.linear.x=1.
    mount=Transform(); mount.rotation.z=-math.sqrt(.5); mount.rotation.w=math.sqrt(.5)
    mount.translation.x=1.
    result=base_odometry(message,mount)
    assert result.pose.pose.position.y==pytest.approx(1.)
    assert result.pose.pose.orientation.w==pytest.approx(1.)
    assert result.twist.twist.linear.y==pytest.approx(1.)
    assert result.twist.twist.linear.x==pytest.approx(0.,abs=1e-12)


def test_covariance_follows_the_rotating_offset_and_invalid_frames_fail():
    message=Odometry(); message.header.frame_id="odom"; message.child_frame_id="woosh_livox3_imu"
    message.pose.pose.orientation.w=1.; message.pose.covariance[35]=.1; message.twist.covariance[35]=.2
    mount=Transform(); mount.rotation.w=1.; mount.translation.x=.3
    result=base_odometry(message,mount)
    assert result.pose.covariance[7]==pytest.approx(.1*.3**2)
    assert result.twist.covariance[7]==pytest.approx(.2*.3**2)
    message.child_frame_id="base_link"
    with pytest.raises(ValueError,match="requires"):base_odometry(message,mount)


def test_native5_mount_and_explicit_frame_preserve_measurement_stamp():
    message=Odometry();message.header.frame_id="odom";message.child_frame_id="woosh_livox5_imu"
    message.header.stamp.sec=123;message.header.stamp.nanosec=456;message.pose.pose.orientation.w=1.
    message.twist.twist.linear.x=1.
    mount=Transform();mount.translation.x=-.0690248982235242;mount.translation.z=-.7384453692895854
    mount.rotation.y=-math.sin(math.pi/12);mount.rotation.w=math.cos(math.pi/12)
    result=base_odometry(message,mount,imu_frame="woosh_livox5_imu")
    assert result.header == message.header and result.child_frame_id == "base_link"
    assert result.pose.pose.position.x == pytest.approx(mount.translation.x)
    assert result.pose.pose.position.z == pytest.approx(mount.translation.z)
    assert result.twist.twist.linear.x == pytest.approx(math.cos(math.pi/6))
    assert result.twist.twist.linear.z == pytest.approx(-.5)
    with pytest.raises(ValueError,match="woosh_livox3_imu"):
        base_odometry(message,mount)
