"""Bounded no-GGA recovery, executed by the existing serial-owner thread."""
import time

from nav_rtk.parser.checksum import validate_nmea_sentence
from nav_rtk.parser.nmea_parser import parse_nmea_sentence


class ReceiverRecovery:
    def __init__(self, evidence=None, stop_event=None, timeout=10., retry_delay=5., healthy_period=30.):
        self.evidence, self.stop_event = evidence, stop_event
        self.timeout, self.retry_delay, self.healthy_period = timeout, retry_delay, healthy_period
        self.started = self.last_gga = self.healthy_since = None
        self.attempts, self.retry_at = 0, 0.
        self.state = dict(phase='monitoring', attempts=0, message='正在检查卫星报文')

    def connected(self):
        # Keep the fault age and retry budget across tty reconnects.
        if self.started is None:
            self.started = time.monotonic()

    def feed(self, text):
        if not text.startswith(('$GNGGA,', '$GPGGA,')) or len(text.split(',')) < 15:
            return
        if not validate_nmea_sentence(text):
            return
        try:
            parsed = parse_nmea_sentence(text)
            if parsed is None or parsed.data.fix_quality is None:
                return
        except (ValueError, OverflowError, IndexError):
            return
        now = time.monotonic()
        if self.healthy_since is None or self.last_gga is None or now-self.last_gga > 3:
            self.healthy_since = now
        self.last_gga = now  # Quality 0 is a live receiver, never a reset trigger.
        if self.state['phase'] in ('blocked', 'retry_wait'):
            self.state = dict(phase='recovered', attempts=self.attempts, message='卫星报文已恢复，正在观察稳定性')
        if self.attempts and now-self.healthy_since >= self.healthy_period:
            self.attempts, self.retry_at = 0, 0.
            self.state = dict(phase='monitoring', attempts=0, message='卫星报文持续正常')

    def suspended(self):
        return self.attempts > 0 and (self.last_gga is None or time.monotonic()-self.last_gga > 3)

    def snapshot(self):
        return dict(self.state)

    def tick(self, reader, corrections):
        self.connected()
        now = time.monotonic()
        reference = self.last_gga if self.last_gga is not None else self.started
        if now-reference < self.timeout or now < self.retry_at or self.attempts >= 3:
            return False
        if self.stop_event is not None and self.stop_event.is_set():
            return False
        level = self.attempts
        self.attempts += 1
        self.healthy_since = None
        self.state = dict(phase='recovering', attempts=self.attempts,
                          message=('正在恢复卫星输出', '正在重新启动接收器', '正在重新启动卫星芯片')[level])
        if self.evidence:
            self.evidence('receiver_recovery_start', level=level, gga_age_s=now-reference)
        # Pause the caster and discard cached fixes/queued corrections before any
        # AT transaction. Do not run the old CLOSE transaction recursively here.
        corrections.opened = False
        corrections.close(None)
        corrections.update(phase='receiver_error', message=self.state['message'])
        reader.on_control_gga = corrections.feed
        try:
            gga = reader.recover_satellite_stream(level, stop_event=self.stop_event)
            self.feed(gga)
            if self.last_gga is None or time.monotonic()-self.last_gga > 1:
                raise RuntimeError('未收到新的有效卫星报文')
            corrections.feed(gga)
            self.state = dict(phase='recovered', attempts=self.attempts, message='卫星报文已恢复，正在观察稳定性')
            corrections.update(phase='waiting', message=self.state['message'])
            if self.evidence:
                self.evidence('receiver_recovery_success', level=level)
        except (OSError, RuntimeError, ValueError, TypeError) as exc:
            self.state = dict(phase='blocked' if self.attempts >= 3 else 'retry_wait',
                              attempts=self.attempts,
                              message='接收器自动恢复未成功' if self.attempts >= 3 else '卫星报文尚未恢复，稍后重试',
                              error=str(exc))
            corrections.update(phase='receiver_error', message=self.state['message'])
            if self.evidence:
                self.evidence('receiver_recovery_failed', level=level, error=str(exc))
        self.retry_at = time.monotonic()+self.retry_delay
        return True  # This transaction consumed serial bytes.
