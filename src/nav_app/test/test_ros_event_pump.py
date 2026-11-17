import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from nav_app.ros_event_pump import GuiRosExecutor, process_ros_events


def test_idle_stops_at_first_empty_wait_set_but_still_polls_image():
    class Executor:
        calls = 0
        def spin_once_ready(self, timeout_sec=0.):
            self.calls += 1
            return False
    class Node:
        images = 0
        def poll_image(self): self.images += 1
    executor, node = Executor(), Node()
    process_ros_events(node, executor=executor)
    assert executor.calls == 1 and node.images == 1


def test_real_executor_drains_bursts_on_owner_thread_and_preserves_futures():
    context = Context(); context.init()
    node = rclpy.create_node('pump_regression', context=context)
    node.poll_image = lambda: None
    executor = GuiRosExecutor(context=context); executor.add_node(node)
    seen = []; owner = threading.get_ident()
    def pump_until(predicate):
        deadline=time.monotonic()+3
        while not predicate() and time.monotonic()<deadline:
            process_ros_events(node,max_callbacks=4,budget_s=.01,executor=executor)
            time.sleep(.001)
        assert predicate()
    try:
        tasks = [executor.create_task(lambda i=i: seen.append((i, threading.get_ident()))) for i in range(6)]
        pump_until(lambda:len(seen)==6)
        # Independent executor tasks have no FIFO guarantee across Humble
        # builds. Require complete, once-only execution on the owner thread.
        assert sorted(seen) == [(i, owner) for i in range(6)]
        assert all(task.done() for task in tasks)
        assert not executor.spin_once_ready(timeout_sec=0.)
        # A new event after the idle exit is still processed on the next tick.
        executor.create_task(lambda: seen.append((6, threading.get_ident())))
        pump_until(lambda:len(seen)==7)
        assert seen[-1] == (6, owner)
        def failure(): raise ValueError('callback failure')
        task = executor.create_task(failure)
        with pytest.raises(ValueError, match='callback failure'):
            pump_until(lambda:task.done())
        assert task.done()
    finally:
        executor.remove_node(node); executor.shutdown(); node.destroy_node(); context.shutdown()


def test_qt_timer_drains_burst_and_yields_between_batches():
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QTimer
    app=QApplication.instance() or QApplication([])
    context=Context(); context.init()
    node=rclpy.create_node('pump_qt_regression',context=context)
    node.poll_image=lambda:None
    executor=GuiRosExecutor(context=context);executor.add_node(node)
    seen=[];owner=threading.get_ident()
    ticks=[]
    def pump():
        before=len(seen)
        process_ros_events(node,max_callbacks=4,budget_s=.003,executor=executor)
        ticks.append(len(seen)-before)
    timer=QTimer();timer.timeout.connect(pump);timer.start(5)
    try:
        for i in range(100):executor.create_task(lambda i=i:seen.append((i,threading.get_ident())))
        deadline=time.monotonic()+3
        while len(seen)<100 and time.monotonic()<deadline:
            app.processEvents();time.sleep(.001)
        assert sorted(seen)==[(i,owner) for i in range(100)]
        assert len(ticks)>=25 and max(ticks)<=4
    finally:
        timer.stop()
        executor.remove_node(node);executor.shutdown();node.destroy_node();context.shutdown()
