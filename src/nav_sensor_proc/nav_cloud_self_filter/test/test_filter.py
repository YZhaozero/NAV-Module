"""Exercise the real ROS node and compare every retained point byte."""
import os
from pathlib import Path
import struct
import subprocess
import time
from types import SimpleNamespace

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import PointCloud2, PointField


@pytest.fixture(scope="module")
def running_filter(tmp_path_factory):
    old = {key: os.environ.get(key) for key in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")}
    os.environ.update(ROS_DOMAIN_ID="214", ROS_LOCALHOST_ONLY="1")
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("self_filter_contract", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = []
    publisher = node.create_publisher(PointCloud2, "/self_filter_test/input", qos_profile_sensor_data)
    subscription = node.create_subscription(PointCloud2, "/self_filter_test/output", received.append, qos_profile_sensor_data)
    prefix = Path(get_package_prefix("nav_cloud_self_filter"))
    share = Path(get_package_share_directory("nav_cloud_self_filter"))
    log = tmp_path_factory.mktemp("self-filter") / "node.log"
    stream = log.open("w")
    process = subprocess.Popen([str(prefix / "lib/nav_cloud_self_filter/cloud_self_filter"),
        "--ros-args", "--params-file", str(share / "config/woosh_livox5.yaml"),
        "-r", "input:=/self_filter_test/input", "-r", "output:=/self_filter_test/output"],
        stdout=stream, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 8
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            assert process.poll() is None, log.read_text()
        assert publisher.get_subscription_count() == 1, log.read_text()
        yield SimpleNamespace(node=node, executor=executor, publisher=publisher, subscription=subscription,
                              received=received, process=process, log=log)
    finally:
        process.terminate()
        process.wait(timeout=5)
        stream.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        for key, value in old.items():
            if value is None: os.environ.pop(key, None)
            else: os.environ[key] = value


def cloud(points, frame="base_footprint"):
    message = PointCloud2()
    message.header.frame_id = frame
    message.header.stamp.sec = 100
    message.header.stamp.nanosec = 123456789
    message.height = 1
    message.width = len(points)
    for name, offset, datatype in [("x",0,7),("y",4,7),("z",8,7),("intensity",12,7),
                                    ("tag",16,2),("line",17,2),("timestamp",18,8)]:
        message.fields.append(PointField(name=name, offset=offset, datatype=datatype, count=1))
    message.point_step = 26
    message.row_step = 26 * len(points)
    message.is_dense = True
    message.data = b"".join(struct.pack("<ffffBBd", *point, float(i), i+1, i%6,
                                      1794876000000000000.+i*5000.) for i, point in enumerate(points))
    return message


def exchange(running, message):
    running.received.clear()
    deadline = time.monotonic()+3
    next_send = 0.
    while time.monotonic() < deadline:
        if time.monotonic() >= next_send:
            running.publisher.publish(message)
            next_send = time.monotonic()+.1
        running.executor.spin_once(timeout_sec=.03)
        matching = [m for m in running.received if m.header.stamp.sec == message.header.stamp.sec]
        if matching: return matching[-1]
    raise AssertionError(running.log.read_text())


def test_remove_pillar_and_origin_but_preserve_floor_surroundings_and_all_fields(running_filter):
    source = cloud([(.4,.1,.8),(.429,0,.605),(.4,.1,.02),(.49,0,.8),
                    (.30,0,.8),(.4,.3,.8),(.4,0,1.7),(.34,.25,.30)])
    result = exchange(running_filter, source)
    assert result.header == source.header  # Exact nanoseconds, not PCL microseconds.
    assert result.fields == source.fields
    assert result.point_step == 26 and result.height == 1 and result.width == 5
    assert result.row_step == 5*26
    assert bytes(result.data) == b"".join(bytes(source.data)[i*26:(i+1)*26] for i in (2,3,4,5,6))


def test_empty_output_does_not_stop_following_frames(running_filter):
    source = cloud([(.4,0,.8)])
    source.header.stamp.sec = 101
    result = exchange(running_filter, source)
    assert result.width == 0 and not result.data and result.header == source.header
    source = cloud([(1.,0,.8)])
    source.header.stamp.sec = 102
    result = exchange(running_filter, source)
    assert result.width == 1 and bytes(result.data) == bytes(source.data)


def test_wrong_frame_is_reported_and_rejected(running_filter):
    running_filter.received.clear()
    source = cloud([(1.,0,.8)], frame="map")
    source.header.stamp.sec = 103
    running_filter.publisher.publish(source)
    deadline = time.monotonic()+.5
    while time.monotonic() < deadline:
        running_filter.executor.spin_once(timeout_sec=.05)
    assert not [m for m in running_filter.received if m.header.stamp.sec == 103]
    assert "Refusing cloud in frame 'map'" in running_filter.log.read_text()


def test_bypass_publishes_original_message(running_filter):
    client = running_filter.node.create_client(SetParameters, "/cloud_self_filter/set_parameters")
    assert client.wait_for_service(timeout_sec=3)
    request = SetParameters.Request(parameters=[Parameter("enabled", value=False).to_parameter_msg()])
    future = client.call_async(request)
    running_filter.executor.spin_until_future_complete(future, timeout_sec=3)
    assert future.done() and future.result().results[0].successful
    try:
        source = cloud([(.4,0,.8),(.429,0,.605)])
        source.header.stamp.sec = 104
        assert exchange(running_filter, source) == source
    finally:
        request.parameters = [Parameter("enabled", value=True).to_parameter_msg()]
        future = client.call_async(request)
        running_filter.executor.spin_until_future_complete(future, timeout_sec=3)
        assert future.done() and future.result().results[0].successful


def test_nested_launch_uses_filter_config_without_overwriting_mapper_config():
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    share = Path(get_package_share_directory("nav_cloud_self_filter"))
    spec = importlib.util.spec_from_file_location("self_filter_launch", share / "launch/livox5_self_filter.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations["params_file"] = "/tmp/mapper-parameters.yaml"
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument): action.execute(context)
    assert context.launch_configurations["filter_params_file"] == str(share / "config/woosh_livox5.yaml")
    assert context.launch_configurations["params_file"] == "/tmp/mapper-parameters.yaml"
