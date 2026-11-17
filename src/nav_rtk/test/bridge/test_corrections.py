import os
import pty
import select
import threading
import time
import socket
from pathlib import Path
import pytest
from nav_rtk.bridge.termios_reader import TermiosLineReader
from nav_rtk.bridge.corrections import Corrections
from nav_rtk.parser import correction_settings as settings
from nav_rtk.parser.ntrip import crc24q
from nav_rtk.parser.serial_client import SerialPortUnavailableError


def gga_bytes():
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    body = 'GNGGA,' + time.strftime('%H%M%S', time.gmtime()) + '.00,4000.0,N,11600.0,E,1,12,0.8,20,M,0,M,,'
    return ('$' + body + '*' + calculate_nmea_checksum(body) + '\r\n').encode()


def test_waiting_reason_follows_receiver_and_expiry_without_hiding_other_errors(monkeypatch):
    import nav_rtk.bridge.corrections as module
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    correction = Corrections()
    correction.update(enabled=True, phase='waiting')
    assert '尚未收到' in correction.snapshot()['message']
    body = 'GNGGA,081350.400,,,,,0,07,99.99,,,,,,'
    correction.feed('$'+body+'*'+calculate_nmea_checksum(body))
    assert '定位质量 0' in correction.snapshot()['message']
    assert not correction.get_gga()
    clock[0] += 4.
    assert '停止更新' in correction.snapshot()['message']
    correction.update(phase='auth_error', message='账号认证失败')
    assert correction.snapshot()['message'] == '账号认证失败'


def recovery_fixture(monkeypatch, tmp_path):
    import nav_rtk.bridge.corrections as module
    clock = [10.]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    correction = Corrections(tmp_path / "account", tmp_path / "control")
    correction.key = "same-switch"
    correction.poll_at = float("inf")
    correction.opened = True
    correction.feed(gga_bytes().decode().strip())
    events = []
    class Client:
        packets = []
        phase = "receiving"
        def snapshot(self):
            return dict(phase=self.phase, message=self.phase)
        def take(self):
            packets, self.packets = self.packets, []
            return packets
        def stop(self):
            events.append(("stop", clock[0]))
    class Reader:
        def rover(self, enabled):
            events.append(("rover", enabled, clock[0]))
            return gga_bytes().decode().strip()
        def write(self, data):
            events.append(("write", data, clock[0]))
    correction.client = Client()
    return correction, Reader(), clock, events


def test_lost_gga_closes_once_and_resumes_from_fresh_fix_without_toggle(monkeypatch, tmp_path):
    correction, reader, clock, events = recovery_fixture(monkeypatch, tmp_path)
    clock[0] = 14.
    assert correction.tick(reader) is True
    assert events == [("rover", False, 14.)]
    assert not correction.opened
    assert correction.snapshot()["phase"] == "waiting"
    assert correction.failed_key is None
    clock[0] = 15.
    correction.feed(gga_bytes().decode().strip())
    correction.client.packets = [b"before recovery"]
    correction.tick(reader)
    assert events == [("rover", False, 14.)]
    assert not correction.client.packets, "Packets received during recovery must be discarded"
    clock[0] = 17.
    correction.feed(gga_bytes().decode().strip())
    correction.client.packets = [b"before ROVER confirmation"]
    assert correction.tick(reader) is True
    assert events[-1] == ("rover", True, 17.)
    assert not any(event[0] == "write" for event in events)
    clock[0] = 17.1
    correction.client.packets = [b"fresh correction"]
    correction.tick(reader)
    assert events[-1] == ("write", b"fresh correction", 17.1)


def test_gga_recovery_requires_valid_fresh_fix_and_preserves_auth_error(monkeypatch, tmp_path):
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    correction, reader, clock, events = recovery_fixture(monkeypatch, tmp_path)
    clock[0] = 14.
    correction.tick(reader)
    clock[0] = 18.
    body = "GNGGA," + time.strftime("%H%M%S", time.gmtime()) + ".00,,,,,0,08,99.99,,,,,,"
    correction.feed("$" + body + "*" + calculate_nmea_checksum(body))
    correction.client.packets = [b"no fix"]
    correction.tick(reader)
    assert events == [("rover", False, 14.)]
    correction.feed(gga_bytes().decode().strip())
    clock[0] = 22.
    correction.client.packets = [b"expired fix"]
    correction.tick(reader)
    assert events == [("rover", False, 14.)]
    correction.client.phase = "auth_error"
    correction.feed(gga_bytes().decode().strip())
    correction.tick(reader)
    assert correction.snapshot()["phase"] == "auth_error"
    assert events == [("rover", False, 14.)]


def test_repeated_gga_interruptions_back_off_and_healthy_injection_resets_delay(monkeypatch, tmp_path):
    correction, reader, clock, events = recovery_fixture(monkeypatch, tmp_path)
    clock[0] = 14.
    correction.tick(reader)
    clock[0] = 17.
    correction.feed(gga_bytes().decode().strip())
    correction.client.packets = [b"first reopen"]
    correction.tick(reader)
    clock[0] = 21.
    correction.tick(reader)
    assert events[-1] == ("rover", False, 21.)
    clock[0] = 24.
    correction.feed(gga_bytes().decode().strip())
    correction.client.packets = [b"too soon"]
    correction.tick(reader)
    assert events[-1] == ("rover", False, 21.), "Repeated interruption needs a longer cooldown"
    clock[0] = 27.
    correction.feed(gga_bytes().decode().strip())
    correction.client.packets = [b"second reopen"]
    correction.tick(reader)
    assert events[-1] == ("rover", True, 27.)
    for second in range(28, 59):
        clock[0] = float(second)
        correction.feed(gga_bytes().decode().strip())
        correction.client.packets = [b"healthy correction"]
        correction.tick(reader)
    clock[0] = 62.
    correction.tick(reader)
    clock[0] = 65.
    correction.feed(gga_bytes().decode().strip())
    correction.client.packets = [b"reopen after healthy operation"]
    correction.tick(reader)
    assert events[-1] == ("rover", True, 65.), "Healthy operation should restore the short cooldown"


def test_receiver_control_failure_retries_after_fresh_gga_with_backoff(monkeypatch, tmp_path):
    correction, reader, clock, events = recovery_fixture(monkeypatch, tmp_path)
    correction.opened = False
    attempts = []
    def fail_control(enabled):
        attempts.append(clock[0])
        raise SerialPortUnavailableError("AT+OPEN ROVER missing acknowledgement")
    reader.rover = fail_control
    correction.client.packets = [b"correction"]
    correction.tick(reader)
    assert correction.failed_key is None
    assert correction.snapshot()["phase"] == "receiver_error"
    assert correction.receiver_retry_at == 13.
    clock[0] = 12.9
    correction.feed(gga_bytes().decode().strip())
    correction.client.packets = [b"too soon"]
    correction.tick(reader)
    assert attempts == [10.]
    clock[0] = 13.
    correction.feed(gga_bytes().decode().strip())
    correction.client.packets = [b"retry after cooldown"]
    correction.tick(reader)
    assert attempts == [10., 13.]
    assert correction.receiver_retry_at == 19.
    assert correction.snapshot()["phase"] == "receiver_error"


def test_rover_open_requires_a_new_valid_gga_before_injection(tmp_path):
    correction = Corrections(tmp_path / "account", tmp_path / "control")
    correction.key = "enabled-switch"
    correction.poll_at = float("inf")
    correction.feed(gga_bytes().decode().strip())
    events = []
    class Client:
        def snapshot(self):
            return dict(phase="receiving", message="receiving")
        def take(self):
            return [b"synthetic RTCM packet"]
    class Reader:
        def rover(self, enabled):
            # The stream confirmation inside rover consumes its GGA; quality
            # may have become 0 since the old cached fix caused the login.
            events.append(("rover", enabled))
        def write(self, data):
            events.append(("write", data))
    correction.client = Client()
    assert correction.tick(Reader()) is True
    assert events == [("rover", True)]
    assert not correction.get_gga(), "The pre-open fix must not survive AT mode switching"
    correction.tick(Reader())
    assert events == [("rover", True)]
    correction.feed(gga_bytes().decode().strip())
    correction.tick(Reader())
    assert events[-1] == ("write", b"synthetic RTCM packet")


def test_no_fix_gga_waits_and_recovers_without_manual_toggle(tmp_path):
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    correction = Corrections(tmp_path / "account", tmp_path / "control")
    correction.key = "enabled-switch"
    correction.poll_at = float("inf")
    correction.opened = True
    correction.update(enabled=True)
    events = []
    class Client:
        def snapshot(self):
            return dict(phase="reconnecting", message="waiting for fix")
        def take(self):
            return [b"synthetic RTCM packet"]
        def stop(self):
            events.append("client stopped")
    class Reader:
        def rover(self, enabled):
            events.append(("rover", enabled))
        def write(self, data):
            events.append(("write", data))
    correction.client = Client()
    body = "GNGGA," + time.strftime("%H%M%S", time.gmtime()) + ".00,,,,,0,08,99.99,,,,,,"
    correction.feed("$" + body + "*" + calculate_nmea_checksum(body))
    assert correction.tick(Reader()) is False
    assert events == [], "No-fix is not a serial failure and must not stop the client"
    assert correction.failed_key is None and correction.opened
    assert correction.snapshot()["phase"] == "waiting"
    assert "定位质量 0" in correction.snapshot()["message"]
    correction.feed(gga_bytes().decode().strip())
    correction.tick(Reader())
    assert events == [("write", b"synthetic RTCM packet")]
    assert correction.snapshot()["phase"] == "injecting"
    assert correction.key == "enabled-switch"


@pytest.mark.parametrize("quality", [0, 1])
def test_rover_confirmation_replaces_cached_fix_atomically(tmp_path, quality):
    from types import SimpleNamespace
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    correction = Corrections(tmp_path / "account", tmp_path / "control")
    correction.key, correction.poll_at = "enabled-switch", float("inf")
    correction.feed(gga_bytes().decode().strip())
    body = gga_bytes().decode().strip().split("*")[0][1:].replace(",E,1,12,", f",E,{quality},12,")
    resumed = "$" + body + "*" + calculate_nmea_checksum(body)
    writes = []
    correction.client = SimpleNamespace(snapshot=lambda: dict(phase="receiving"), take=lambda: [b"RTCM"])
    reader = SimpleNamespace(rover=lambda enabled: resumed, write=writes.append)
    assert correction.tick(reader) is True
    assert not writes
    assert bool(correction.get_gga()) is bool(quality)
    correction.tick(reader)
    assert writes == ([b"RTCM"] if quality else [])


def test_bad_checksum_cannot_keep_receiver_stream_fresh(monkeypatch):
    import nav_rtk.bridge.corrections as module
    clock = [10.]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    correction = Corrections()
    valid = gga_bytes().decode().strip()
    correction.feed(valid)
    assert correction.gga_stream_fresh()
    clock[0] += 4.
    correction.feed(valid[:-2] + ("00" if valid[-2:] != "00" else "FF"))
    assert not correction.gga_stream_fresh()
    assert not correction.get_gga()


def test_live_no_fix_preserves_injection_without_control_or_fake_position(monkeypatch, tmp_path):
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    correction, reader, clock, events = recovery_fixture(monkeypatch, tmp_path)
    correction.client.snapshot = lambda: dict(phase='receiving', session_active=True)
    body = 'GNGGA,,,,,,0,08,99.99,,,,,,'
    no_fix = '$' + body + '*' + calculate_nmea_checksum(body)
    for elapsed in (1., 2., 10., 120.):
        clock[0] = 10. + elapsed
        correction.feed(no_fix)
        correction.client.packets = [b'fresh RTCM']
        correction.tick(reader)
        assert events[-1] == ('write', b'fresh RTCM', clock[0])
        assert not correction.get_gga(), 'Do not expose an old coordinate as a valid fix'
        assert correction.live_no_fix()
        assert '定位暂失' in correction.snapshot()['message']
    assert len(events) == 4, 'No AT mode changes or client restart on loss of solution'
    # Returning to a real fix uses the same channel.
    correction.feed(gga_bytes().decode().strip())
    correction.client.packets = [b'after fix']
    correction.tick(reader)
    assert events[-1][1] == b'after fix' and not correction.live_no_fix()
    # Silence still triggers the existing recovery, not blind injection.
    clock[0] += 4.
    correction.client.packets = [b'must not write']
    correction.tick(reader)
    assert events[-1] == ('rover', False, clock[0])


def test_invalid_no_fix_cannot_authorize_injection(monkeypatch, tmp_path):
    correction, reader, clock, events = recovery_fixture(monkeypatch, tmp_path)
    correction.client.snapshot = lambda: dict(phase='receiving', session_active=True)
    correction.feed('$GNGGA,,,,,,0,08,99.99,,,,,,*ZZ')
    correction.client.packets = [b'must not write']
    correction.tick(reader)
    assert not correction.live_no_fix() and not events


def test_corrupt_gga_preserves_diagnostic_fix_but_waits_for_valid_injection(monkeypatch, tmp_path):
    correction, reader, clock, events = recovery_fixture(monkeypatch, tmp_path)
    original = correction.get_gga()
    correction.feed('$GNGGA,,,,,,0,08,99.99,,,,,,*ZZ')
    assert correction.get_gga() == original
    correction.client.packets = [b'invalid window']
    correction.tick(reader)
    assert events == []
    correction.feed(gga_bytes().decode().strip())
    correction.client.packets = [b'valid window']
    correction.tick(reader)
    assert events == [('write', b'valid window', clock[0])]


def test_no_fix_wait_does_not_hide_authentication_error(tmp_path):
    from types import SimpleNamespace
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    correction = Corrections(tmp_path / "account", tmp_path / "control")
    correction.poll_at = float("inf")
    correction.key = "enabled"
    correction.client = SimpleNamespace(snapshot=lambda: dict(phase="auth_error", message="账号认证失败"), take=lambda: [])
    body = "GNGGA,120000.00,,,,,0,08,99.99,,,,,,"
    correction.feed("$" + body + "*" + calculate_nmea_checksum(body))
    correction.tick(None)
    assert correction.snapshot()["phase"] == "auth_error"
    assert correction.snapshot()["message"] == "账号认证失败"


def test_payload_waits_for_receiver_stream_even_beyond_fixed_guard():
    master, slave = pty.openpty()
    stop = threading.Event()
    accepted = threading.Event()
    timings = {}
    def receiver():
        buffer = b""
        while not stop.is_set():
            if not select.select([master], [], [], .02)[0]:
                continue
            buffer += os.read(master, 4096)
            while b"\r\n" in buffer:
                line, buffer = buffer.split(b"\r\n", 1)
                now = time.monotonic()
                if line == b"AT+CFG":
                    os.write(master, b"+CFG\r\n")
                elif line == b"AT+OPEN ROVER":
                    os.write(master, b"ROVER OPEN OK\r\n")
                elif line == b"AT+EXIT":
                    timings["exit"] = now
                    timer = threading.Timer(.25, lambda: os.write(master, gga_bytes()))
                    timer.daemon = True
                    timer.start()
                elif line == b"correction":
                    timings["payload"] = now
                    if now - timings["exit"] >= .24:
                        accepted.set()
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()
    thread = threading.Thread(target=receiver, daemon=True)
    thread.start()
    try:
        resumed = reader.rover(True)
        assert resumed.startswith("$GNGGA,") and ",E,1,12," in resumed
        reader.write(b"correction\r\n")
        assert accepted.wait(1), timings
    finally:
        reader.close()
        stop.set()
        thread.join(2)
        os.close(master)
        os.close(slave)


def test_open_ack_without_resumed_gga_restores_rover_and_fails_closed():
    master, slave = pty.openpty()
    stop = threading.Event()
    commands = []
    def receiver():
        buffer = b""
        while not stop.is_set():
            if not select.select([master], [], [], .02)[0]:
                continue
            buffer += os.read(master, 4096)
            while b"\r\n" in buffer:
                line, buffer = buffer.split(b"\r\n", 1)
                commands.append(line)
                reply = {b"AT+CFG": b"+CFG", b"AT+OPEN ROVER": b"ROVER OPEN OK",
                         b"AT+CLOSE ROVER": b"ROVER CLOSE OK"}.get(line)
                if reply:
                    os.write(master, reply + b"\r\n")
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()
    thread = threading.Thread(target=receiver, daemon=True)
    thread.start()
    try:
        with pytest.raises(SerialPortUnavailableError, match="未恢复卫星报文"):
            reader.rover(True)
        assert b"AT+CLOSE ROVER" in commands
        assert not any(line.startswith(b"\xd3") for line in commands)
    finally:
        reader.close()
        stop.set()
        thread.join(2)
        os.close(master)
        os.close(slave)


def test_last_injected_age_is_elapsed_not_a_permanent_success(monkeypatch):
    import nav_rtk.bridge.corrections as module
    clock=[10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    correction=Corrections()
    assert correction.snapshot()['last_injected_age_s'] is None
    correction.last_injected_at=10.
    clock[0]=18.
    assert correction.snapshot()['last_injected_age_s'] == 8.


def test_one_serial_owner_can_enable_write_disable_and_exit():
    master, slave = pty.openpty()
    stop = threading.Event()
    commands = []
    def receiver():
        buffer = b""
        while not stop.is_set():
            if not select.select([master], [], [], .05)[0]:
                continue
            buffer += os.read(master, 4096)
            while b"\r\n" in buffer:
                line, buffer = buffer.split(b"\r\n", 1)
                commands.append(line)
                # Actual receiver can enter config mode without returning +CFG.
                reply = {b"AT+VERSION?": b"\n+VERSION=V10277.1.3",
                         b"AT+OPEN ROVER": b"ROVER OPEN OK",
                         b"AT+CLOSE ROVER": b"ROVER CLOSE OK", b"AT+EXIT": b"+EXIT"}.get(line)
                if reply:
                    os.write(master, reply + b"\r\n")
                if line == b"AT+EXIT":
                    os.write(master, gga_bytes())
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()
    thread = threading.Thread(target=receiver, daemon=True)
    thread.start()
    try:
        reader.rover(True)
        reader.write(b"test-data\r\n")
        reader.rover(False)
        time.sleep(.05)
        assert commands == [b"AT+CFG", b"AT+VERSION?", b"AT+OPEN ROVER", b"AT+EXIT", b"test-data",
                            b"AT+CFG", b"AT+VERSION?", b"AT+CLOSE ROVER", b"AT+EXIT"]
    finally:
        reader.close()
        stop.set();thread.join(2)
        os.close(master);os.close(slave)


def test_manual_default_does_not_open_network_or_write_receiver(tmp_path):
    cfg, control = tmp_path / "account.json", tmp_path / "switch.json"
    settings.save(dict(provider="custom",host="127.0.0.1",port=1,mountpoint="TEST",username="test",password="test",
                       tls=False,datum="WGS84",auto_start=False), cfg)
    correction = Corrections(cfg, control)
    class Reader:
        def rover(self, enabled):
            raise AssertionError("Should not touch receiver")
        def write(self, data):
            raise AssertionError("Should not write")
    correction.tick(Reader())
    assert correction.client is None and not correction.snapshot()["enabled"]
    settings.set_requested(True, control)
    correction.poll_at = 0
    correction.tick(Reader())
    assert correction.client is not None
    assert correction.snapshot()["enabled"]
    assert correction.client.socket is None  # Indoors / no valid GGA cannot start a paid login.
    settings.set_requested(False, control)
    for _ in range(10):
        correction.poll_at = 0
        correction.tick(Reader())
        if correction.client is None:
            break
        time.sleep(.02)
    assert correction.client is None and not correction.snapshot()["enabled"]


def test_no_ack_keeps_live_gga_and_reports_exact_failed_command(tmp_path):
    from types import SimpleNamespace
    master, slave = pty.openpty()
    stop = threading.Event()
    commands, frames, writes = [], [], []
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()
    reader.on_control_frame = frames.append
    correction = Corrections(tmp_path / 'account', tmp_path / 'control')
    correction.key, correction.poll_at = 'enabled-switch', float('inf')
    correction.feed(gga_bytes().decode().strip())
    correction.client = SimpleNamespace(snapshot=lambda: dict(phase='receiving'),
                                        take=lambda: [b'RTCM'], stop=lambda: None)
    def receiver():
        while not stop.wait(.04):
            os.write(master, gga_bytes())
            if select.select([master], [], [], 0)[0]:
                commands.append(os.read(master, 4096))
    worker = threading.Thread(target=receiver, daemon=True)
    worker.start()
    try:
        assert correction.tick(reader)
        assert correction.get_gga() and correction.gga_stream_fresh()
        status = correction.snapshot()
        assert status['phase'] == 'receiver_error' and not correction.opened
        assert 'AT+VERSION?' in status['message']
        assert '卫星报文仍在接收' in status['message']
        assert status['gga_received_age_s'] < .5
        assert status['receiver_control']['gga_frames'] > 10
        assert status['receiver_control']['stream_restored'] is True
        assert not any(item['acknowledged'] for item in status['receiver_control']['commands'])
        assert len(frames) > 10
        assert b'AT+OPEN ROVER' not in b''.join(commands)
        assert b'RTCM' not in b''.join(commands)
    finally:
        stop.set(); worker.join(2)
        reader.close(); os.close(master); os.close(slave)


def test_differential_reference_is_not_silently_treated_as_wgs84(tmp_path):
    correction = Corrections(tmp_path / "account", tmp_path / "control")
    for datum in ("unknown", "CGCS2000", "ITRF2008", "WGS84"):
        correction.reference_datum = datum
        value = correction.qualify_observation(dict(fix_quality=4, valid_position=True))
        assert value["coordinate_system"] == datum
        assert value["valid_position"] == (datum == "WGS84")
    assert correction.qualify_observation(dict(fix_quality=1, valid_position=True))["coordinate_system"] == "WGS84"


def test_local_caster_to_serial_then_manual_stop(tmp_path):
    """Real TCP + PTY, synthetic data only; never contacts a paid service."""
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    server=socket.socket();server.bind(('127.0.0.1',0));server.listen(1);server.settimeout(3)
    stop=threading.Event();seen=[];commands=[]
    raw=b'\xd3\x00\x04\x3e\xd0\x00\x00'
    packet=raw+crc24q(raw).to_bytes(3,'big')
    def caster():
        try:
            conn,_=server.accept()
            with conn:
                conn.settimeout(3);conn.recv(4096)
                conn.sendall(b'ICY 200 OK\r\n')
                while not stop.wait(.05):conn.sendall(packet)
        except OSError:pass
    master,slave=pty.openpty()
    def receiver():
        buffer=b''
        while not stop.is_set():
            if not select.select([master],[],[],.02)[0]:continue
            buffer+=os.read(master,4096)
            while buffer:
                if buffer.startswith(b'\xd3'):
                    if len(buffer)<len(packet):break
                    seen.append(buffer[:len(packet)]);buffer=buffer[len(packet):];continue
                if b'\r\n' not in buffer:break
                line,buffer=buffer.split(b'\r\n',1);commands.append(line)
                response={b'AT+CFG':b'+CFG',b'AT+OPEN ROVER':b'ROVER OPEN OK',b'AT+CLOSE ROVER':b'ROVER CLOSE OK'}.get(line)
                if response:os.write(master,response+b'\r\n')
                if line == b'AT+EXIT':os.write(master,gga_bytes())
    cfg,control=tmp_path/'account',tmp_path/'control'
    settings.save(dict(provider='custom',host='127.0.0.1',port=server.getsockname()[1],mountpoint='TEST',
                       username='test',password='test',tls=False,datum='WGS84',auto_start=False),cfg)
    settings.set_requested(True,control)
    correction=Corrections(cfg,control);reader=TermiosLineReader(os.ttyname(slave),230400,writable=True)
    reader.open();threads=[threading.Thread(target=f,daemon=True) for f in (caster,receiver)]
    for t in threads:t.start()
    try:
        deadline=time.monotonic()+5
        while not seen and time.monotonic()<deadline:
            body='GNGGA,'+time.strftime('%H%M%S',time.gmtime())+'.00,4000.0,N,11600.0,E,1,12,0.8,20,M,0,M,,'
            correction.feed('$'+body+'*'+calculate_nmea_checksum(body))
            correction.tick(reader);time.sleep(.02)
        assert seen and all(p==packet for p in seen)
        assert correction.snapshot()['injected_packets']>0
        settings.set_requested(False,control)
        deadline=time.monotonic()+3
        while correction.snapshot()['enabled'] and time.monotonic()<deadline:
            correction.poll_at=0;correction.tick(reader);time.sleep(.02)
        assert not correction.snapshot()['enabled'] and not correction.opened
        assert b'AT+CLOSE ROVER' in commands
        count=len(seen);time.sleep(.15);assert len(seen)==count
    finally:
        correction.close(reader);reader.close();stop.set();server.close()
        for t in threads:t.join(3)
        os.close(master);os.close(slave)
