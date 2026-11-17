"""Actual ROS delivery while the application's executor is not spinning."""
import time
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformBroadcaster
from nav_app.transform_listener import DisplayTransformListener


def test_tf_stays_current_while_display_executor_is_busy_and_thread_closes():
    rclpy.init()
    publisher = rclpy.create_node('display_tf_delivery_test')
    buffer = Buffer()
    listener = DisplayTransformListener(buffer)
    broadcaster = TransformBroadcaster(publisher)
    transform = TransformStamped()
    transform.header.frame_id = 'test_map'
    transform.child_frame_id = 'test_body'
    transform.transform.rotation.w = 1.
    try:
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            transform.header.stamp = publisher.get_clock().now().to_msg()
            broadcaster.sendTransform(transform)
            time.sleep(.05)  # deliberately no spin of the application node
            if buffer.can_transform('test_map', 'test_body', Time.from_msg(transform.header.stamp)):
                break
        assert buffer.can_transform('test_map', 'test_body', Time.from_msg(transform.header.stamp))
    finally:
        listener.close(); listener.close()
        assert not listener.dedicated_listener_thread.is_alive()
        publisher.destroy_node(); rclpy.shutdown()


def test_private_tf_node_keeps_its_name_and_selected_clock_under_app_remaps():
    rclpy.init(args=["--ros-args", "-r", "__node:=main_app"])
    listener = DisplayTransformListener(Buffer(), use_sim_time=True)
    try:
        assert listener.node.get_name().startswith("nav_display_tf_")
        assert listener.node.get_parameter("use_sim_time").value is True
    finally:
        listener.close()
        rclpy.shutdown()
