import json
import os
import socket
import threading
import time
import pytest
from nav_rtk.parser import correction_settings as settings
from nav_rtk.parser.ntrip import Body, Client, AuthenticationError, NtripError, RtcmDecoder, crc24q, fresh_gga


def config(**extra):
    return dict(provider="custom", host="127.0.0.1", port=12345, mountpoint="TEST", username="test-user",
                password="synthetic-secret", tls=False, datum="WGS84", auto_start=False, **extra)


def gga(quality=1, when=None):
    stamp = time.strftime("%H%M%S", time.gmtime(time.time() if when is None else when))
    body = f"GNGGA,{stamp}.00,4000.0,N,11600.0,E,{quality},12,0.8,20.0,M,0.0,M,,"
    checksum = 0
    for c in body.encode():
        checksum ^= c
    return f"${body}*{checksum:02X}"


def packet():
    data = b"\xd3\x00\x04\x3e\xd0\x00\x00"
    return data + crc24q(data).to_bytes(3, "big")


@pytest.mark.parametrize('status', [408, 429, 500, 502, 503, 504])
def test_transient_http_errors_are_recoverable(status):
    with pytest.raises(NtripError) as error:
        Body().feed(f'HTTP/1.1 {status} Error\r\n\r\n'.encode())
    assert not isinstance(error.value, AuthenticationError)


@pytest.mark.parametrize('status', [401, 403, 404])
def test_account_or_mountpoint_errors_still_require_user_action(status):
    with pytest.raises(AuthenticationError):
        Body().feed(f'HTTP/1.1 {status} Error\r\n\r\n'.encode())


def test_transient_retry_backoff_is_bounded(monkeypatch):
    client = Client(config(), gga)
    waits = []
    def fail():
        raise NtripError('temporary')
    def wait(delay):
        waits.append(delay)
        if len(waits) == 6:
            client.stop_event.set()
    monkeypatch.setattr(client, 'session', fail)
    monkeypatch.setattr(client.stop_event, 'wait', wait)
    client.run()
    assert waits == [3, 6, 12, 24, 30, 30]


@pytest.mark.parametrize("error,expected", [
    (NtripError("卫星定位已失效"), "卫星定位已失效"),
    (OSError(104, "synthetic-secret"), "网络连接异常"),
    (ValueError("synthetic-secret"), "数据格式异常"),
])
def test_failures_keep_specific_category_without_secrets(monkeypatch, caplog, error, expected):
    client = Client(config(), gga)
    def fail():
        raise error
    monkeypatch.setattr(client, "session", fail)
    monkeypatch.setattr(client.stop_event, "wait", lambda _: client.stop_event.set())
    client.run()
    state = client.snapshot()
    assert expected in state["message"]
    assert state["last_error"] == type(error).__name__
    assert "synthetic-secret" not in repr(state) + caplog.text


def test_intentional_socket_close_is_not_reported_as_failure(monkeypatch):
    client = Client(config(), gga)
    def close():
        client.stop_event.set()
        raise OSError(9, "Bad file descriptor")
    monkeypatch.setattr(client, "session", close)
    client.run()
    assert "last_error" not in client.snapshot()


def test_stop_keeps_socket_reference_when_network_thread_clears_it():
    client = Client(config(), gga)
    closed = []
    class Socket:
        def shutdown(self, how):
            client.socket = None  # Network thread finishes during shutdown.
        def close(self):
            closed.append(True)
    client.socket = Socket()
    client.stop()
    assert closed == [True] and client.stop_event.is_set()


def test_real_gga_time_fix_and_checksum_required():
    assert fresh_gga(gga())
    assert not fresh_gga(gga(quality=0))
    assert not fresh_gga(gga(when=time.time()-300))
    assert not fresh_gga(gga()[:-2]+"ZZ")


def test_gga_waiting_explains_receiver_and_clock_failures():
    from nav_rtk.parser.ntrip import gga_rejection_reason
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    assert '尚未收到' in gga_rejection_reason('')
    raw = 'GNGGA,081350.400,,,,,0,07,99.99,,,,,,'
    raw = '$' + raw + '*' + calculate_nmea_checksum(raw)
    assert '定位质量 0' in gga_rejection_reason(raw)
    assert '时间相差' in gga_rejection_reason(gga(when=time.time()-300))
    valid = gga()
    bad = valid[:-2] + f'{int(valid[-2:], 16) ^ 1:02X}'
    assert '校验' in gga_rejection_reason(bad)
    assert not gga_rejection_reason(gga())


def test_rtcm_fragmentation_and_crc_rejects_corruption():
    decoder = RtcmDecoder()
    good = packet()
    bad = good[:-1] + bytes([good[-1] ^ 1])
    assert decoder.feed(b"garbage" + bad + good[:4]) == []
    assert decoder.feed(good[4:]) == [good]


@pytest.mark.parametrize("headers", [b"ICY 200 OK\r\n", b"HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n"])
def test_http_and_icy_fragmented(headers):
    body = Body()
    result = b"".join(body.feed(bytes([b])) for b in headers + packet())
    assert result == packet()


def test_chunked_response_and_auth_error():
    body = Body()
    data = b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\nA\r\n" + packet() + b"\r\n"
    assert b"".join(body.feed(bytes([b])) for b in data) == packet()
    with pytest.raises(AuthenticationError):
        Body().feed(b"HTTP/1.1 401 Unauthorized\r\n\r\n")


def test_local_caster_stream_and_stop_without_receiver(tmp_path):
    from nav_rtk.parser.evidence import EvidenceWriter
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    finished = threading.Event()
    received = []
    def caster():
        with server:
            conn, _ = server.accept()
            with conn:
                conn.settimeout(3)
                request = conn.recv(4096)
                received.append(request)
                conn.sendall(b"ICY 200 OK\r\n" + packet())
                finished.wait(2)
    thread = threading.Thread(target=caster, daemon=True)
    thread.start()
    value = config()
    value["port"] = server.getsockname()[1]
    client = Client(value, gga)
    client.evidence = EvidenceWriter(root=tmp_path)
    client.thread.start()
    until = time.monotonic() + 3
    while client.snapshot()["rtcm_packets"] == 0 and time.monotonic() < until:
        time.sleep(.02)
    assert client.take() == [packet()]
    assert b"GET /TEST" in received[0]
    assert "synthetic-secret" not in repr(client.snapshot())
    client.stop()
    finished.set()
    client.thread.join(3)
    thread.join(3)
    assert not client.thread.is_alive() and not client.take()
    client.evidence.close()
    traces="".join(p.read_text() for p in client.evidence.path.glob("events-*.jsonl"))
    assert "ntrip_body_rx" in traces and "ntrip_gga" in traces and "ntrip_rtcm" in traces
    assert "ntrip_http_status" in traces and '"code": "200"' in traces
    assert "synthetic-secret" not in traces and "Authorization" not in traces
    import base64
    assert base64.b64encode((value["username"]+":"+value["password"]).encode()).decode() not in traces


def test_live_no_fix_keeps_real_tcp_session_and_fresh_corrections():
    """The caster survives quality 0; no stale/fabricated GGA is uploaded."""
    server = socket.socket()
    server.bind(('127.0.0.1', 0))
    server.listen(1)
    value = config()
    value['port'] = server.getsockname()[1]
    current = [gga(4)]
    live = [False]
    release = threading.Event()
    connected = threading.Event()
    uploads, errors = [], []
    client = Client(value, lambda: current[0], lambda: live[0])
    def caster():
        try:
            with server:
                conn, _ = server.accept()
                with conn:
                    conn.settimeout(.1)
                    connected.set()
                    request = b''
                    while b'\r\n\r\n' not in request:
                        request += conn.recv(4096)
                    uploads.append(request)
                    conn.sendall(b'ICY 200 OK\r\n')
                    while not release.is_set():
                        conn.sendall(packet())
                        try:
                            data = conn.recv(4096)
                            if data:
                                uploads.append(data)
                            elif not client.stop_event.is_set():
                                errors.append('Client disconnected during live no-fix')
                                return
                        except socket.timeout:
                            pass
        except OSError as exc:
            if not client.stop_event.is_set():
                errors.append(str(exc))
    thread = threading.Thread(target=caster, daemon=True)
    thread.start()
    client.thread.start()
    def wait_for(predicate):
        until = time.monotonic() + 3
        while not predicate() and time.monotonic() < until:
            time.sleep(.01)
        assert predicate(), (client.snapshot(), errors)
    try:
        assert connected.wait(3)
        wait_for(lambda: client.snapshot()['rtcm_packets'] >= 2)
        current[0], live[0] = '', True
        before = client.snapshot()['rtcm_packets']
        wait_for(lambda: client.snapshot()['rtcm_packets'] >= before + 4)
        assert client.snapshot()['session_active']
        assert '定位暂失' in client.snapshot()['message']
        assert client.take() and not errors
        current[0], live[0] = gga(5), False
        wait_for(lambda: client.snapshot()['message'] == '已收到有效差分数据')
        assert b''.join(uploads).count(b'GET /TEST') == 1
        assert b',E,0,' not in b''.join(uploads)
        # A dead/invalid stream is different from a live quality-0 report.
        current[0] = ''
        wait_for(lambda: not client.snapshot()['session_active'])
        assert client.snapshot()['phase'] == 'reconnecting'
    finally:
        client.stop()
        release.set()
        client.thread.join(3)
        thread.join(3)


def test_live_no_fix_cannot_start_a_new_caster_login(monkeypatch):
    client = Client(config(), lambda: '', lambda: True)
    calls = []
    monkeypatch.setattr(client, 'session', lambda: calls.append('login'))
    monkeypatch.setattr(client.stop_event, 'wait', lambda _: client.stop_event.set())
    client.run()
    assert calls == [] and client.snapshot()['phase'] == 'waiting'


def test_fresh_packet_queue_still_drops_expired_corrections(monkeypatch):
    client = Client(config(), lambda: '', lambda: True)
    client.queue.extend([(10., b'expired'), (12., packet())])
    monkeypatch.setattr(time, 'monotonic', lambda: 12.5)
    assert client.take() == [packet()]


def test_secret_storage_and_manual_override_are_separate(tmp_path):
    path, runtime = tmp_path / "account.json", tmp_path / "control.json"
    value = config()
    settings.save(value, path)
    assert settings.load(path) == value
    assert os.stat(path).st_mode & 0o077 == 0
    assert not settings.requested(value, runtime)
    value["auto_start"] = True
    assert settings.requested(value, runtime)
    settings.set_requested(False, runtime)
    assert not settings.requested(value, runtime)
    settings.set_requested(True, runtime)
    assert settings.requested(value, runtime)
    assert "synthetic-secret" not in runtime.read_text()


@pytest.mark.parametrize("key,value", [("host", "host\r\nInjected: x"), ("username", "u:p"), ("password", "bad\n"), ("port", True), ("auto_start", "true"), ("mountpoint", "../../x")])
def test_invalid_account_cannot_be_saved(tmp_path, key, value):
    settings_value = config()
    settings_value[key] = value
    with pytest.raises(ValueError):
        settings.save(settings_value, tmp_path / "account.json")
    assert not (tmp_path / "account.json").exists()
