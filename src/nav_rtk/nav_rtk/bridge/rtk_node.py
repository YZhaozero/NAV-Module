"""WitRTK 串口 → ROS 话题桥接节点。

设计要点：

- 串口读取放在独立 daemon 线程里，ROS 发布放在定时器里，两者通过一把锁交换
  「最新一帧」。不跨线程直接 publish，避免和 executor 的交互问题。
- 既有 UI JSON 保持兼容；同一串口读者额外发布逐历元观测与 NavSatFix。
- GGA 固定解标志不冒充厘米级协方差；只有同期 GST 提供精度时填写协方差。
- 解析异常必须逐条兜住。`parse_nmea_sentence` 校验失败时是**抛 ValueError 而不是
  返回 None**，历史上还出现过字段越界抛 IndexError；单条坏帧绝不能打断读线程。
"""

from __future__ import annotations

import json
import signal
import threading
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus

from nav_rtk.parser import nmea_parser
from nav_rtk.parser.serial_client import SerialPortUnavailableError
from nav_rtk.parser.state_store import RTKStateStore
from nav_rtk.parser.wtrtk_runtime import WTRTKFrameEvent, format_system_time
from nav_rtk.parser.observation_time import time_state

from .termios_reader import TermiosLineReader
from .observation import ObservationBuilder
from .corrections import Corrections
from .receiver_diagnostics import ReceiverDiagnostics
from .epochs import EpochBuffer
from .receiver_recovery import ReceiverRecovery
from .serial_discovery import require_explicit_port
from .debug_console import DebugConsole


class RtkBridge(Node):
    """读取 WitRTK 串口并发布 `/rtk/frame` 与 `/rtk/status`。"""

    def __init__(self) -> None:
        super().__init__("rtk_node")

        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("frame_topic", "/rtk/frame")
        self.declare_parameter("status_topic", "/rtk/status")
        # 轮询频率只决定「新帧到发布」的最大延迟（20ms），不代表数据率。
        # 真正的数据率由设备决定，实测约 20 Hz。
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("reconnect_interval_s", 2.0)
        self.declare_parameter("warning_history", 50)
        self.declare_parameter("log_received_frames", True)
        self.declare_parameter("record_raw_gnss", True)
        self.record_raw_gnss = bool(self.get_parameter("record_raw_gnss").value)

        configured_port = require_explicit_port(self.get_parameter("port").value)
        self.configured_port = configured_port
        # Reconnect only to the explicitly selected path; never switch devices.
        self.port = configured_port
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.reconnect_interval_s = max(
            0.2, float(self.get_parameter("reconnect_interval_s").value))
        self.log_received_frames = bool(
            self.get_parameter("log_received_frames").value)
        warning_history = max(1, int(self.get_parameter("warning_history").value))

        frame_topic = str(self.get_parameter("frame_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        status_rate_hz = max(0.1, float(self.get_parameter("status_rate_hz").value))

        self.frame_publisher = self.create_publisher(String, frame_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.fix_publisher = self.create_publisher(NavSatFix, "/rtk/fix", 10)
        self.observation_publisher = self.create_publisher(String, "/rtk/observation", 10)
        self._observations = deque(maxlen=100)
        self._epochs = EpochBuffer()
        self._pending_fixes = deque(maxlen=100)
        self._observation_builder = ObservationBuilder()
        self._receiver_diagnostics = ReceiverDiagnostics()

        self._lock = threading.Lock()
        self._pending_frame: Optional[str] = None
        self._state_json: Optional[str] = None
        self._warnings = deque(maxlen=warning_history)
        self._serial_ok = False
        self._serial_error = ""
        self._frame_index = 0
        self._frames_total = 0
        self._parse_error_total = 0
        self._generation = 0
        self._started_at = time.time()

        self._stop_event = threading.Event()
        from nav_rtk.parser.evidence import EvidenceWriter
        self.evidence = EvidenceWriter()
        self.debug_console = DebugConsole(self.evidence, self._stop_event)
        self.debug_publisher = self.create_publisher(String, '/rtk/debug/state', 10)
        self.create_subscription(String, '/rtk/debug/request', self._debug_request, 10)
        self.create_subscription(String, '/rtk/debug/guard', self._debug_guard, 10)
        from nav_rtk.parser.raw_gnss import RawGnssStatus
        self.raw_gnss = RawGnssStatus(self.evidence)
        self.corrections = Corrections(evidence=self.evidence)
        self._receiver_recovery = ReceiverRecovery(self.evidence, self._stop_event)
        self._reader = threading.Thread(
            target=self._run_reader, name="rtk-serial-reader", daemon=True)
        self._reader.start()

        self._frame_timer = self.create_timer(
            1.0 / publish_rate_hz, self._publish_frame)
        self._status_timer = self.create_timer(
            1.0 / status_rate_hz, self._publish_status)

        self.get_logger().info(
            f"RTK 桥接进程已启动，串口连接状态请查看诊断：{self.port} @ {self.baudrate}，"
            f"发布 {frame_topic} / {status_topic}")

    # ---------------------------------------------------------------- 读线程

    def _debug_request(self, message):
        if len(message.data) > 4096:
            return
        try:
            request = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not self._serial_ok:
            if isinstance(request, dict):
                self.debug_console.result(request, 'rejected', '串口尚未打开，未发送')
            return
        self.debug_console.submit(request)

    def _debug_guard(self, message):
        if len(message.data) > 1024:
            return
        try:
            guard = json.loads(message.data)
        except (ValueError, TypeError):
            return
        self.debug_console.set_guard(guard)

    def _run_reader(self) -> None:
        try:
            self._read_loop()
        except (OSError, RuntimeError, ValueError, TypeError) as exc:  # Surface startup failures outside reconnect handling.
            self._mark_serial_down(f"串口读取线程异常：{type(exc).__name__}: {exc}")
            self.get_logger().error(f"RTK 串口读取已停止：{type(exc).__name__}: {exc}")

    def _resolve_reader_port(self) -> str:
        """Retain the user's path even when that device is disconnected."""
        return require_explicit_port(self.configured_port)

    def _configure_raw_output(self, reader):
        if self.debug_console.exclusive:
            self.raw_gnss.configured(dict(
                phase='debug_paused', message='串口调试中，原始测量配置等待正常接收恢复'))
            return False
        if self.record_raw_gnss:
            from .raw_output import enable_raw_output
            self.raw_gnss.configured(enable_raw_output(reader, self._stop_event))
        else:
            self.raw_gnss.configured(dict(phase='disabled', message='原始测量自动配置已关闭'))
        return True

    def _read_loop(self) -> None:
        """串口读取主循环；串口断开时按间隔重连，不退出线程。"""

        store = RTKStateStore()
        # `$WTRAW` 是二进制报文，按 \n 切行后剩下的碎片会被当成普通行喂给解析器
        # 并刷出一堆无意义的告警。用这个标志把它的续行吃掉。
        in_wtraw = False

        while not self._stop_event.is_set():
            reader = None
            try:
                port = self._resolve_reader_port()
                reader = TermiosLineReader(port, self.baudrate, writable=True)
                reader.evidence = self.evidence
                reader.monitor = self.debug_console.traffic
                reader.manual_control_active = self.debug_console.exclusive
                reader.on_rover_rtcm = self.raw_gnss.receive

                def control_frame(frame):
                    nonlocal in_wtraw
                    in_wtraw = self._handle_frame(frame, store, in_wtraw)

                reader.on_control_frame = control_frame
                reader.open()
                raw_configured = RtkBridge._configure_raw_output(self, reader)
            except (SerialPortUnavailableError, ValueError) as exc:
                self._mark_serial_down(str(exc))
                if reader is not None:
                    reader.close()
                self._stop_event.wait(self.reconnect_interval_s)
                continue
            except (OSError, RuntimeError, ValueError, TypeError) as exc:  # Report device boundary failures before reconnecting.
                self._mark_serial_down(f"{type(exc).__name__}: {exc}")
                if reader is not None:
                    reader.close()
                self._stop_event.wait(self.reconnect_interval_s)
                continue

            self._mark_serial_up()
            self._receiver_recovery.connected()
            raw_recovery_pending = not raw_configured

            def maintain_receiver(active_reader):
                nonlocal raw_recovery_pending
                was_debugging = self.debug_console.exclusive
                consumed = self.debug_console.tick(active_reader, self.corrections)
                self.baudrate = active_reader.baudrate
                if was_debugging and not self.debug_console.exclusive:
                    self._receiver_recovery = ReceiverRecovery(self.evidence, self._stop_event)
                    raw_recovery_pending = not RtkBridge._configure_raw_output(self, active_reader)
                    consumed = True
                if consumed or self.debug_console.exclusive:
                    return consumed
                consumed = self._receiver_recovery.tick(active_reader, self.corrections)
                raw_recovery_pending = raw_recovery_pending or consumed
                if (raw_recovery_pending and self.record_raw_gnss
                        and self._receiver_recovery.snapshot().get('phase') == 'recovered'):
                    raw_recovery_pending = not RtkBridge._configure_raw_output(self, active_reader)
                    consumed = True
                if consumed or self._receiver_recovery.suspended():
                    return consumed
                return self.corrections.tick(active_reader)

            try:
                for frame in reader.iter_frames(stop_event=self._stop_event, on_idle=maintain_receiver):
                    if self._stop_event.is_set():
                        break
                    in_wtraw = self._handle_frame(frame, store, in_wtraw)
            except (OSError, RuntimeError, ValueError, TypeError) as exc:  # noqa: BLE001
                self._add_warning("serial", f"{type(exc).__name__}: {exc}")
                self._mark_serial_down(f"{type(exc).__name__}: {exc}")
            finally:
                if self.debug_console.exclusive:
                    self.corrections.pause_for_debug()
                else:
                    self.corrections.close(reader)
                reader.close()

            if not self._stop_event.is_set():
                self._stop_event.wait(self.reconnect_interval_s)

    def _handle_frame(self, frame, store: RTKStateStore, in_wtraw: bool) -> bool:
        """处理一行串口数据，返回更新后的 wtraw 续行标志。"""

        text = frame.text
        self._receiver_recovery.feed(text)
        received_at = time.time()
        with self._lock:
            self._receiver_diagnostics.feed(text, received_at)
        self.corrections.feed(text)
        if text.startswith("$WTRAW,"):
            return True
        from .framing import is_receiver_sentence
        if not is_receiver_sentence(frame.raw_bytes):
            return in_wtraw

        try:
            observation = self._observation_builder.feed(text, received_at)
            if observation is not None:
                observation = self.corrections.qualify_observation(observation)
                observation['_raw_sentence'] = text[:512]
                with self._lock:
                    self._observations.append(observation)
            parsed = nmea_parser.parse_nmea_sentence(text)
        except ValueError as exc:
            kind = "checksum" if str(exc).startswith("checksum 校验失败") else "parse"
            self._add_warning(kind, str(exc))
            return False
        except (OSError, RuntimeError, ValueError, TypeError) as exc:  # noqa: BLE001
            self._add_warning("parse", f"{type(exc).__name__}: {exc}")
            return False
        if parsed is None:
            return False

        state = store.update(parsed)
        state_json = json.dumps(state.to_dict(), ensure_ascii=False)

        payload = None
        if parsed.sentence_id == "WTRTK":
            event = WTRTKFrameEvent.from_data(
                index=self._frame_index,
                receive_system_time=format_system_time(),
                # WTRTK 报文本身不带时间字段，通常为 None。
                sentence_timestamp=None,
                raw_sentence=parsed.raw,
                display_line=text,
                data=parsed.data,
            )
            payload = json.dumps(event.to_dict(), ensure_ascii=False)

        with self._lock:
            self._state_json = state_json
            if payload is not None:
                self._pending_frame = payload
                self._frame_index += 1
                self._frames_total += 1
                if self.log_received_frames and self._frames_total % 200 == 1:
                    self.get_logger().info(
                        f"已接收 {self._frames_total} 帧 WTRTK")
        return False

    # ------------------------------------------------------------ 状态记录

    def _mark_serial_up(self) -> None:
        with self._lock:
            if not self._serial_ok:
                self._generation += 1
            self._serial_ok = True
            self._serial_error = ""
        self.get_logger().info(f"串口已打开：{self.port} @ {self.baudrate}")

    def _mark_serial_down(self, message: str) -> None:
        self._observation_builder = ObservationBuilder()
        with self._lock:
            first = self._serial_ok or self._serial_error != message
            self._receiver_diagnostics = ReceiverDiagnostics()
            self._observations.clear()
            self._epochs.clear_pending()
            self._reset_pending_fixes = True
            self._pending_frame = None
            self._state_json = None
            self._serial_ok = False
            self._serial_error = message
            if first:
                self._generation += 1
        if first:
            self.get_logger().warning(f"串口不可用：{message}")

    def _add_warning(self, kind: str, message: str) -> None:
        entry = {
            "ts": format_system_time(),
            "level": "WARN",
            "kind": kind,
            "message": message,
        }
        with self._lock:
            self._warnings.append(entry)
            self._parse_error_total += 1

    # --------------------------------------------------------------- 发布

    def _publish_frame(self) -> None:
        with self._lock:
            if getattr(self, '_reset_pending_fixes', False):
                self._pending_fixes.clear()
                self._reset_pending_fixes = False
            payload = self._pending_frame
            self._pending_frame = None
            observations = []
            now_mono = time.monotonic()
            for observation in self._observations:
                observations.extend(self._epochs.add(observation, now_mono))
            observations.extend(self._epochs.drain(now_mono))
            self._observations.clear()
        for observation in observations:
            message = String()
            message.data = json.dumps(observation, ensure_ascii=False, allow_nan=False)
            self.observation_publisher.publish(message)
            # No UTC means no timestamped ROS fix. Keep the raw UI observation
            # with stamp=None as evidence, never fabricate zero or reception time.
            if observation.get('stamp') is not None:
                self._pending_fixes.append(observation)
        now = time.time()
        while self._pending_fixes:
            observation = self._pending_fixes[0]
            timing = time_state(observation, now)
            if timing == 'waiting':
                break
            self._pending_fixes.popleft()
            fix = NavSatFix()
            fix.header.frame_id = "gnss_antenna"
            stamp = observation["stamp"]
            valid = observation["valid_position"] and timing == 'ready'
            # Missing altitude is explicitly unknown, never silently MSL-as-ellipsoid.
            fix.latitude = observation["latitude"] if valid else float("nan")
            fix.longitude = observation["longitude"] if valid else float("nan")
            fix.altitude = observation["altitude_ellipsoid"] if valid and observation["altitude_ellipsoid"] is not None else float("nan")
            fix.status.status = NavSatStatus.STATUS_FIX if valid else NavSatStatus.STATUS_NO_FIX
            # NMEA talker GN does not identify individual constellation usage.
            fix.status.service = 0
            if stamp is not None:
                fix.header.stamp.sec = int(stamp)
                fix.header.stamp.nanosec = min(999999999, int((stamp-int(stamp))*1e9))
            if valid and observation["covariance"]:
                fix.position_covariance = observation["covariance"]
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.fix_publisher.publish(fix)
        # 没有新帧就不发，避免把上一帧当成本帧重复广播。
        if payload is None:
            return
        message = String()
        message.data = payload
        self.frame_publisher.publish(message)

    def _publish_status(self) -> None:
        with self._lock:
            state_json = self._state_json
            warnings = list(self._warnings)
            serial_ok = self._serial_ok
            serial_error = self._serial_error
            frames_total = self._frames_total
            parse_errors = self._parse_error_total
            generation = self._generation
            receiver = self._receiver_diagnostics.snapshot()
            receiver['epochs'] = self._epochs.snapshot()

        status = {
            "raw_gnss": self.raw_gnss.snapshot(),
            "receiver": receiver,
            "recovery": self._receiver_recovery.snapshot(),
            "corrections": self.corrections.snapshot(),
            "serial": {
                "ok": serial_ok,
                "error": serial_error,
                "port": self.port,
                "configuredPort": self.configured_port,
                "autoDetect": False,
                "baudrate": self.baudrate,
                "generation": generation,
            },
            "stats": {
                "framesTotal": frames_total,
                "parseErrors": parse_errors,
                "uptimeS": round(time.time() - self._started_at, 1),
            },
            "warnings": warnings,
            # 聚合状态（GGA/GSA/RMC）：卫星数、HDOP、fix_quality 这些 WTRTK 帧里没有。
            "state": json.loads(state_json) if state_json else None,
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False)
        status["evidence"] = self.evidence.snapshot()
        self.evidence("rtk_status", status=status)
        message.data = json.dumps(status, ensure_ascii=False)
        self.status_publisher.publish(message)
        debug = String()
        debug.data = json.dumps(self.debug_console.snapshot(), ensure_ascii=False)
        self.debug_publisher.publish(debug)

    # --------------------------------------------------------------- 收尾

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._reader.is_alive():
            self._reader.join(timeout=4.0)
        self.evidence.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # The process group and ros2 launch can both deliver SIGINT. A second
        # one must not interrupt the bounded reader/evidence drain below.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
