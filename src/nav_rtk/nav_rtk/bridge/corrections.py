"""Correction lifecycle driven by the single serial-owner thread."""
import json
import threading
import time
from nav_rtk.parser import correction_settings as settings
from nav_rtk.parser.checksum import validate_nmea_sentence
from nav_rtk.parser.ntrip import Client, gga_rejection_reason


class Corrections:
    def __init__(self, config_path=None, control_path=None, evidence=None):
        self.evidence = evidence
        self.config_path = config_path
        self.control_path = control_path or settings.control_path()
        self.client = None
        self.key = None
        self.poll_at = 0.
        self.gga = ""
        self.gga_at = 0.
        self.gga_stream_at = None
        self.gga_no_fix = False
        self.gga_injection_blocked = False
        self.gga_reason = "尚未收到卫星定位报文"
        self.lock = threading.Lock()
        self.state = dict(enabled=False, phase="off", message="差分已关闭", injected_bytes=0, injected_packets=0)
        self.opened = False
        self.failed_key = None
        self.reference_datum = "unknown"
        self.last_injected_at = None
        self.receiver_retry_at = 0.
        self.receiver_retry_delay = 3.
        self.receiver_healthy_since = None

    def feed(self, text):
        if text.startswith(("$GNGGA,", "$GPGGA,")):
            if not validate_nmea_sentence(text):
                with self.lock:
                    self.gga_injection_blocked = True
                    self.state['rejected_gga_frames'] = self.state.get('rejected_gga_frames', 0) + 1
                return  # A corrupt frame must not erase a still-fresh valid fix.
            reason = gga_rejection_reason(text)
            if len(text.partition('*')[0].split(',')) < 15 or reason == '卫星定位报文格式无效':
                with self.lock:
                    self.gga_injection_blocked = True
                    self.state['rejected_gga_frames'] = self.state.get('rejected_gga_frames', 0) + 1
                return
            with self.lock:
                self.gga_injection_blocked = False
                self.gga = text if not reason else ""
                self.gga_reason = reason
                self.gga_at = time.monotonic()
                valid = validate_nmea_sentence(text)
                fields = text.split(',')
                self.gga_no_fix = valid and len(fields) >= 15 and fields[6] == '0'
                if valid:
                    self.gga_stream_at = self.gga_at

    def get_gga(self):
        with self.lock:
            return self.gga if time.monotonic() - self.gga_at <= 3 else ""

    def gga_stream_fresh(self):
        with self.lock:
            return self.gga_stream_at is not None and time.monotonic() - self.gga_stream_at <= 3

    def live_no_fix(self):
        with self.lock:
            return self.gga_no_fix and time.monotonic() - self.gga_at <= 3

    def update(self, **value):
        with self.lock:
            self.state.update(value)

    def snapshot(self):
        with self.lock:
            result = dict(self.state)
            age = time.monotonic() - self.gga_at if self.gga_at else None
            reason = self.gga_reason
            if age is not None and age > 3:
                reason = "卫星定位报文已停止更新"
            result.update(gga_received_age_s=age, gga_rejection_reason=reason)
            if result.get("enabled") and result.get("phase") == "waiting" and reason:
                result["message"] = "差分等待：" + reason
            result['last_injected_age_s'] = (max(0., time.monotonic() - self.last_injected_at)
                                              if self.last_injected_at is not None else None)
            result['receiver_retry_in_s'] = max(0., self.receiver_retry_at - time.monotonic())
            return result

    def tick(self, reader):
        if reader is not None and hasattr(reader, "on_control_gga"):
            reader.on_control_gga = self.feed
        now = time.monotonic()
        consumed = False
        if now >= self.poll_at:
            self.poll_at = now + .5
            try:
                config = settings.load(self.config_path)
                wanted = config is not None and settings.requested(config, self.control_path)
                control_rev = self.control_path.stat().st_mtime_ns if self.control_path.exists() else 0
                key = (json.dumps(config, sort_keys=True), control_rev) if wanted else None
            except (OSError, ValueError):
                wanted, key, config = False, None, None
            # Retain service frame metadata even when manually switched off: the
            # receiver may briefly keep a corrected solution after input stops.
            self.reference_datum = config["datum"] if config else "unknown"
            if key != self.key:
                if self.client:
                    self.client.stop()
                    if self.client.thread.is_alive():
                        return False  # Never spawn an unbounded collection of stalled DNS threads.
                    self.client = None
                if self.opened:
                    reader.rover(False)
                    self.opened = False
                    consumed = True
                self.key = key
                self.failed_key = None
                self.last_injected_at = None
                self.receiver_retry_at = 0.
                self.receiver_retry_delay = 3.
                self.receiver_healthy_since = None
                self.update(enabled=wanted, phase="waiting" if wanted else "off",
                            message="已开启，等待有效卫星定位" if wanted else "差分已关闭",
                            injected_bytes=0, injected_packets=0, received_bytes=0, rtcm_packets=0)
                if wanted:
                    self.client = Client(config, self.get_gga, self.live_no_fix)
                    self.client.evidence = self.evidence
                    self.client.thread.start()
        if self.client and self.failed_key != self.key:
            client_state = self.client.snapshot()
            self.update(**client_state)
            with self.lock:
                invalid_gga = self.gga_injection_blocked
            if invalid_gga:
                # Preserve a still-fresh valid fix for diagnostics, but a corrupt
                # newer row cannot authorize either RTCM injection or recovery.
                self.client.take()
                self.update(phase='waiting', message='卫星报文校验失败，等待新的有效报文')
                return consumed
            # Keep feeding the already established caster/rover connection
            # across a live quality-0 interval. Serial silence, invalid GGA,
            # a new login or a closed rover still follow the recovery gates.
            assist_recovery = (not self.get_gga() and self.live_no_fix() and self.opened
                               and client_state.get('session_active', False))
            if not self.get_gga() and not assist_recovery:
                self.client.take()
                if self.gga_stream_fresh():
                    # No existing usable caster session: wait for a real fix
                    # to establish one, without cycling the receiver.
                    if client_state.get("phase") != "auth_error":
                        self.update(phase="waiting", message="已开启，等待有效卫星定位")
                    return consumed
                if self.opened:
                    # A stuck rover channel can suppress GGA indefinitely. Stop
                    # injection and restore ordinary reception once. Keep the
                    # single client's GGA-gated wait/reconnect loop alive:
                    # a transient UART gap must not require a manual toggle.
                    reader.rover(False)
                    self.opened = False
                    self.receiver_retry_at = time.monotonic() + self.receiver_retry_delay
                    self.receiver_retry_delay = min(30., self.receiver_retry_delay * 2)
                    self.receiver_healthy_since = None
                    self.update(phase="waiting",
                                message="卫星报文已中断，差分已暂停；定位恢复后自动重试")
                    return True
                return consumed
            if not self.opened and now < self.receiver_retry_at:
                self.client.take()
                if client_state.get("phase") != "auth_error":
                    self.update(phase="waiting", message="卫星定位已恢复，差分稍后自动重试")
                return consumed
            packets = self.client.take()
            if packets:
                if not self.opened:
                    try:
                        resumed_gga = reader.rover(True)
                        self.opened = True
                        consumed = True
                    except (OSError, RuntimeError, ValueError, TypeError) as exc:
                        self.opened = False
                        self.receiver_retry_at = time.monotonic() + self.receiver_retry_delay
                        self.receiver_retry_delay = min(30., self.receiver_retry_delay * 2)
                        self.receiver_healthy_since = None
                        self.client.take()
                        from nav_rtk.parser.serial_client import SerialPortUnavailableError
                        message = str(exc) if isinstance(exc, SerialPortUnavailableError) else "差分通道操作失败，请检查接收机连接"
                        self.update(phase="receiver_error", message=message,
                                    receiver_last_error=message,
                                    receiver_control=getattr(reader, "control_diagnostics", {}))
                        return True
                    self.update(receiver_control=getattr(reader, "control_diagnostics", {}))
                    # UART confirmation can be quality 0. Replace the pre-AT
                    # fix with the actual post-EXIT GGA, atomically so a valid
                    # transition does not trigger an unnecessary caster login.
                    if resumed_gga:
                        self.feed(resumed_gga)
                    else:
                        # Compatibility with readers without a returned GGA:
                        # only the next main-loop GGA may authorize injection.
                        with self.lock:
                            self.gga = ""
                            self.gga_at = 0.
                            self.gga_stream_at = time.monotonic()
                            self.gga_no_fix = False
                            self.gga_reason = "差分通道已开启，等待新的有效卫星定位"
                    # Discard these batches and return to normal serial reads;
                    # remaining client packets retain their own 2 s age limit.
                    self.client.take()
                    self.update(phase="waiting", message="已开启，等待有效卫星定位")
                    return consumed
                for packet in packets:
                    reader.write(packet)
                    with self.lock:
                        injected_at = time.monotonic()
                        if (self.receiver_healthy_since is None or self.last_injected_at is None
                                or injected_at - self.last_injected_at > 3):
                            self.receiver_healthy_since = injected_at
                        if injected_at - self.receiver_healthy_since >= 30:
                            self.receiver_retry_delay = 3.
                        self.state["injected_bytes"] += len(packet)
                        self.state["injected_packets"] += 1
                        self.last_injected_at = injected_at
                if packets:
                    self.update(phase="injecting", message=("定位暂失，差分持续送入，等待接收器恢复"
                                if assist_recovery else "差分数据已送入接收机，等待定位结果"))
        return consumed

    def pause_for_debug(self):
        """No UART writes: debug and corrections share the same owner thread."""
        if self.client:
            self.client.stop()
        self.update(phase='debug_paused', message='串口调试中，差分注入暂停')

    def resume_after_debug(self):
        # Keep the stopped client until tick observes its thread exit. Do not
        # replay its queued RTCM or silently reuse a pre-debug rover state.
        self.key = ('debug-resume',)
        self.opened = False
        self.gga = ''
        self.gga_at = 0.
        self.failed_key = None

    def close(self, reader):
        if self.client:
            self.client.stop()
        if self.opened:
            try:
                reader.rover(False)
            except (OSError, RuntimeError, ValueError) as exc:
                self.update(phase="receiver_error", message=f"关闭接收机差分模式失败：{exc}")
        self.opened = False
        self.receiver_retry_at = 0.
        self.receiver_retry_delay = 3.
        self.receiver_healthy_since = None
        # Preserve a still-exiting client until tick observes thread completion.
        self.key = object()
        with self.lock:
            self.gga = ""
            self.gga_at = 0.
            self.gga_stream_at = None
            self.gga_no_fix = False
            self.gga_reason = "尚未收到卫星定位报文"
        self.update(phase="serial_down", message="接收器未连接，差分已暂停")

    def qualify_observation(self, observation):
        datum = self.reference_datum if observation["fix_quality"] in (2, 4, 5) else "WGS84"
        observation["coordinate_system"] = datum
        if datum != "WGS84":
            observation["valid_position"] = False
            observation["coordinate_reason"] = "差分坐标系尚未转换为 WGS84，暂不用于地图定位"
        return observation
