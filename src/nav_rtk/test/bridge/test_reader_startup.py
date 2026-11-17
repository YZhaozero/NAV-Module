"""Real ROS node + PTY startup regression; no hardware or correction account."""
import json
import os
import time

import pytest
import rclpy

from nav_rtk.bridge import rtk_node
from nav_rtk.parser import correction_settings
from nav_rtk.parser.checksum import calculate_nmea_checksum


@pytest.fixture
def bridge(tmp_path, monkeypatch):
    monkeypatch.setenv("NAV_EVIDENCE_DIR", str(tmp_path / "evidence"))
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))
    monkeypatch.setattr(correction_settings, "load", lambda *args: None)
    master, slave = os.openpty()
    path = os.ttyname(slave)
    node = None
    try:
        rclpy.init(args=["--ros-args", "-p", f"port:={path}",
                        "-p", "record_raw_gnss:=false"])
        node = rtk_node.RtkBridge()
        yield node, master, path
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        os.close(master)
        os.close(slave)


def test_real_reader_starts_receives_and_publishes_status(bridge):
    node, master, path = bridge
    body = "GNGGA,120000.00,,,,,0,00,99.99,,,,,,"
    raw = f"${body}*{calculate_nmea_checksum(body)}\r\n".encode()
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and node._receiver_diagnostics.counts["GGA"] < 3:
        os.write(master, raw)
        time.sleep(.04)
    assert node._reader.is_alive(), node._serial_error
    assert node._serial_ok, node._serial_error
    assert node._receiver_diagnostics.counts["GGA"] >= 3
    assert node._parse_error_total == 0
    assert node.port == path
    messages = []
    class Publisher:
        def publish(self, message):
            messages.append(json.loads(message.data))
    node.status_publisher = Publisher()
    node._publish_status()
    assert messages[0]["serial"]["autoDetect"] is False
    assert messages[0]["serial"]["ok"] is True
    assert messages[0]["receiver"]["counts"]["GGA"] >= 3
    node._stop_event.set()
    node._reader.join(timeout=3)
    assert not node._reader.is_alive()


def test_reader_programming_error_propagates(bridge, monkeypatch):
    node, _, _ = bridge
    node._stop_event.set()
    node._reader.join(timeout=3)
    assert not node._reader.is_alive()
    def crash():
        raise UnboundLocalError("synthetic startup failure")
    monkeypatch.setattr(node, "_read_loop", crash)
    with pytest.raises(UnboundLocalError, match="synthetic startup failure"):
        node._run_reader()


def test_reader_device_failure_is_reported(bridge, monkeypatch):
    node, _, _ = bridge
    node._stop_event.set()
    node._reader.join(timeout=3)
    assert not node._reader.is_alive()
    def fail():
        raise OSError("synthetic device failure")
    monkeypatch.setattr(node, "_read_loop", fail)
    node._run_reader()
    assert not node._serial_ok
    assert "串口读取线程异常" in node._serial_error
    assert "OSError" in node._serial_error


def test_missing_selected_port_does_not_change_to_another_device(bridge):
    node, _, _ = bridge
    node._stop_event.set()
    node._reader.join(timeout=3)
    node.configured_port = "/dev/ttyUSB99999"
    assert node._resolve_reader_port() == "/dev/ttyUSB99999"
