"""Receive small TF messages independently of expensive GUI callbacks."""
import rclpy
from tf2_ros import TransformListener


class DisplayTransformListener(TransformListener):
    def __init__(self, buffer, use_sim_time=False):
        self._closed = False
        # Humble requires an explicit node. Sharing the GUI node between
        # executors would steal its callbacks, so only this private node spins
        # in TransformListener's dedicated executor.
        node = rclpy.create_node(
            f"nav_display_tf_{id(self):x}", use_global_arguments=False,
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=use_sim_time)])
        self.node = node
        try:
            super().__init__(buffer, node, spin_thread=True)
        except Exception:
            node.destroy_node()
            raise

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.executor.shutdown()
        self.dedicated_listener_thread.join()
        self.unregister()
        self.node.destroy_node()

    def __del__(self):
        if hasattr(self, 'dedicated_listener_thread'):
            self.close()
