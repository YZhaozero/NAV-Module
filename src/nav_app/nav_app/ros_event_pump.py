"""Drain ROS bursts on the GUI thread while reserving time for Qt input."""
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor, TimeoutException, ShutdownException, ExternalShutdownException


class GuiRosExecutor(SingleThreadedExecutor):
    """Execute on the Qt owner thread, stopping a burst at the first empty wait.

    A per-callback worker/Qt handoff was slower in the complete replay. Keep
    state ownership and the existing 5ms timer, without four empty wait-set
    rebuilds per timer tick. No private rclpy executor fields are used.
    """
    def spin_once_ready(self, timeout_sec=0.):
        try:
            handler, _entity, _node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except (TimeoutException, ShutdownException, ExternalShutdownException):
            return False
        handler()
        if handler.exception() is not None: raise handler.exception()
        return True


def process_ros_events(node, max_callbacks=16, budget_s=.003, *, executor=None):
    deadline = time.monotonic() + budget_s
    for _ in range(max_callbacks):
        if executor is None:
            rclpy.spin_once(node, timeout_sec=0.)
        elif hasattr(executor, 'spin_once_ready'):
            if not executor.spin_once_ready(timeout_sec=0.):
                break
        else:
            executor.spin_once(timeout_sec=0.)
        if time.monotonic() >= deadline:
            break
    node.poll_image()
