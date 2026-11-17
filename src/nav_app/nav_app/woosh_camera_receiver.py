"""Receive one lossless camera stream, then serve raw images locally on Jetson."""
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from .features.image_archive import decode_image


def main():
    rclpy.init()
    node = rclpy.create_node("woosh_camera1_receiver")
    output = node.create_publisher(Image, "/foo/camera_1/color/image_raw", qos_profile_sensor_data)
    subscription = node.create_subscription(
        CompressedImage, "/foo/nav/camera_1/image/compressed",
        lambda message: output.publish(decode_image(message)), qos_profile_sensor_data)
    node.get_logger().info("Lossless camera1 PNG -> /foo/camera_1/color/image_raw")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
