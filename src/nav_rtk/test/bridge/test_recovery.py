from types import SimpleNamespace
import pytest
from nav_rtk.bridge.corrections import Corrections
from nav_rtk.bridge.termios_reader import TermiosLineReader
from nav_rtk.parser.serial_client import SerialPortUnavailableError
from nav_rtk.parser.checksum import calculate_nmea_checksum


def gga(quality=1):
    body = f'GNGGA,120000.00,4000.0,N,11600.0,E,{quality},12,0.8,20,M,0,M,,'
    return '$'+body+'*'+calculate_nmea_checksum(body)


def correction(monkeypatch):
    import nav_rtk.bridge.corrections as module
    now = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(module.time, 'time', lambda: 43200.)
    obj = Corrections()
    obj.key = 'active'
    obj.poll_at = float('inf')
    obj.client = SimpleNamespace(snapshot=lambda: dict(phase='receiving'), take=lambda: [b'rtcm'],
                                 stop=lambda: pytest.fail('recoverable failure stopped client'))
    obj.feed(gga())
    return obj, now


def test_bad_checksum_does_not_clear_or_freshen_valid_gga(monkeypatch):
    obj, now = correction(monkeypatch)
    valid = obj.get_gga()
    now[0] += 1
    obj.feed(valid[:-2] + 'ZZ')
    assert obj.get_gga() == valid and obj.gga_at == 10.
    obj.feed('$GNGGA,broken*00')
    assert obj.get_gga() == valid
    obj.feed(gga(0))
    assert not obj.get_gga() and obj.gga_stream_fresh()


def test_receiver_control_failure_retries_with_bounded_backoff(monkeypatch):
    obj, now = correction(monkeypatch)
    calls = []
    def rover(enabled):
        calls.append(enabled)
        if len(calls) < 3:
            raise SerialPortUnavailableError('synthetic unavailable')
        return gga()
    writes = []
    reader = SimpleNamespace(rover=rover, write=writes.append)
    obj.tick(reader)
    assert obj.receiver_retry_at == 13. and len(calls) == 1
    for _ in range(10):
        obj.tick(reader)
    assert len(calls) == 1
    now[0] = 13.1
    obj.feed(gga())
    obj.tick(reader)
    assert obj.receiver_retry_at == 19.1 and len(calls) == 2
    now[0] = 19.2
    obj.feed(gga())
    obj.tick(reader)
    assert obj.opened and not writes
    obj.tick(reader)
    assert writes == [b'rtcm'] and obj.snapshot()['phase'] == 'injecting'


def test_quality_zero_keeps_rover_open_but_never_injects(monkeypatch):
    obj, now = correction(monkeypatch)
    obj.opened = True
    writes = []
    reader = SimpleNamespace(rover=lambda _: pytest.fail('live no-fix is not a UART failure'), write=writes.append)
    obj.feed(gga(0))
    obj.tick(reader)
    assert not writes and obj.opened
    obj.feed(gga())
    obj.tick(reader)
    assert writes == [b'rtcm']


@pytest.mark.parametrize('quality', [2, 4, 5])
def test_all_differential_modes_require_declared_datum(quality):
    obj = Corrections()
    result = obj.qualify_observation(dict(fix_quality=quality, valid_position=True))
    assert not result['valid_position'] and result['coordinate_system'] == 'unknown'


def test_select_error_is_reported_not_clean_disconnect(monkeypatch):
    import nav_rtk.bridge.termios_reader as module
    reader = TermiosLineReader('/unused', 230400)
    reader._fd = 123
    def fail(*args):
        raise OSError('synthetic')
    monkeypatch.setattr(module.select, 'select', fail)
    with pytest.raises(SerialPortUnavailableError, match='串口等待失败'):
        next(reader.iter_frames())
