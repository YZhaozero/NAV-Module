import base64
import os
import select
import threading
import time
from types import SimpleNamespace as NS
import uuid

import pytest

from nav_rtk.bridge.command_catalog import BY_KEY, COMMANDS, prepare, custom_command
from nav_rtk.bridge.debug_console import DebugConsole
from nav_rtk.bridge.termios_reader import TermiosLineReader


def setup():
    events = []
    console = DebugConsole(lambda event, **kw: events.append((event, kw)), threading.Event())
    client = uuid.uuid4().hex
    console.set_guard(dict(session=console.session, client=client, allowed=True, sent_at=time.time()))
    return console, client, events


def request(console, client, key='VERSION?', **kw):
    return dict(id=uuid.uuid4().hex, session=console.session, client=client,
                sent_at=time.time(), key=key, **kw)


def corrections():
    calls = []
    return NS(pause_for_debug=lambda: calls.append('pause'), resume_after_debug=lambda: calls.append('resume')), calls


@pytest.mark.parametrize('row', [c for c in COMMANDS if not c.unavailable], ids=lambda c: c.key)
def test_all_document_commands_have_valid_exact_crlf_wire(row):
    values = [v for _, v in row.choices] or ['']
    for value in values:
        spec, wire = prepare(row.key, value, True)
        assert wire.endswith(b'\r\n') and wire.count(b'\n') == 1
        assert b'{' not in wire
        decoded, custom_wire, actual = custom_command(wire.hex(), 'hex', True, True)
        assert decoded == spec and custom_wire == wire and actual == value


@pytest.mark.parametrize('key,value', [('IMU', '15'), ('GGA', '0x0F'), ('BAUD', '230400'),
                                       ('RTKMODE', '2'), ('GPSMODE', '3'), ('AT', '7')])
def test_parameters_do_not_accept_unverified_encodings(key, value):
    with pytest.raises(ValueError):
        prepare(key, value, True)


def test_dangerous_hex_cannot_bypass_confirmation_or_batch_checks():
    for text, encoding in [('AT+RST', 'text'), ('41542b5253540d0a', 'hex')]:
        with pytest.raises(ValueError, match='确认'):
            custom_command(text, encoding)
    with pytest.raises(ValueError):
        custom_command('AT\r\nAT+RST', confirmed=True)
    with pytest.raises(ValueError):
        custom_command('AT+MSG=,,,WTRTK', confirmed=True)
    assert BY_KEY['RST'].restart and BY_KEY['RESTORE'].restart


def test_unknown_and_incomplete_binary_have_no_success_match():
    spec, wire, value = custom_command('00ff', 'hex', False, True)
    assert wire == b'\x00\xff'
    assert not DebugConsole.matches(spec, b'OK')
    assert not DebugConsole.matches(BY_KEY['OPEN'], b'prefix ROVER OPEN OK')
    assert not DebugConsole.matches(BY_KEY['VERSION?'], b'+VERSION=')


def test_guard_stale_session_and_queue_reject_without_uart():
    console, client, _ = setup()
    old = request(console, client)
    old['session'] = 'stale'
    console.submit(old)
    assert console.pending is None
    first = request(console, client)
    console.submit(first)
    console.submit(first)  # duplicate delivery is not another execution
    console.submit(request(console, client))
    assert console.pending['id'] == first['id']
    assert console.results[-1]['state'] == 'rejected'
    console.guard_at -= 3
    console.tick(NS(), corrections()[0])
    assert not console.exclusive
    assert console.results[-1]['state'] == 'rejected'


def test_auto_query_and_explicit_resume(monkeypatch):
    console, client, events = setup()
    wires = []
    def exchange(reader, wire, spec, req, value='', timeout=1.2):
        wires.append(wire)
        return spec.reply + (value or 'V10277.1.3' if spec.key == 'VERSION?' else '')
    monkeypatch.setattr(console, 'exchange', exchange)
    reader = NS(manual_control_active=False)
    corr, calls = corrections()
    console.submit(request(console, client))
    assert console.tick(reader, corr)
    assert wires == [b'AT+CFG\r\n', b'AT+VERSION?\r\n', b'AT+EXIT\r\n']
    assert console.mode == 'normal' and console.exclusive
    console.submit(request(console, client, 'RESUME'))
    console.tick(reader, corr)
    assert not console.exclusive and not reader.manual_control_active
    assert calls == ['pause', 'resume']
    assert wires == [b'AT+CFG\r\n', b'AT+VERSION?\r\n', b'AT+EXIT\r\n']


def test_manual_mode_does_not_silently_enter_configuration(monkeypatch):
    console, client, _ = setup()
    monkeypatch.setattr(console, 'exchange', lambda *args: pytest.fail('unexpected UART command'))
    console.submit(request(console, client, automatic=False))
    console.tick(NS(), corrections()[0])
    assert console.results[-1]['state'] == 'unconfirmed'
    assert '先进入' in console.results[-1]['message']


def test_timeout_never_retries_or_resumes_corrections(monkeypatch):
    console, client, _ = setup()
    console.mode = 'config'
    calls = []
    def fail(*args):
        calls.append(args[1])
        raise TimeoutError('no ACK')
    monkeypatch.setattr(console, 'exchange', fail)
    console.submit(request(console, client, 'RST', confirmed=True))
    console.tick(NS(), corrections()[0])
    assert calls == [b'AT+RST\r\n']
    assert console.mode == 'unknown' and console.exclusive
    assert console.results[-1]['state'] == 'unconfirmed'


def test_setter_is_read_back_and_baud_switch_is_single_write(monkeypatch):
    console, client, _ = setup()
    calls = []
    reader = NS(write=lambda wire: calls.append(wire), set_baudrate=lambda baud: calls.append(baud))
    monkeypatch.setattr(console, 'exchange', lambda r, wire, *a: calls.append(wire) or '+BAUD=6')
    console.submit(request(console, client, 'BAUD', value='6', confirmed=True))
    console.tick(reader, corrections()[0])
    assert calls == [b'AT+CFG\r\n', b'AT+BAUD=6\r\n', 115200, b'AT+BAUD?\r\n', b'AT+EXIT\r\n']
    assert console.results[-1]['state'] == 'verified'


def test_live_preview_is_bounded_and_byte_preserving():
    console, _, _ = setup()
    console.traffic('RX', b'\x00\xff\r\n')
    assert base64.b64decode(console.snapshot()['records'][0]['base64']) == b'\x00\xff\r\n'
    console.traffic('RX', b'x' * 512 * 300)
    snap = console.snapshot()
    assert len(snap['records']) == 256 and snap['trimmed'] == 45


def test_at_test_does_not_require_exit_in_normal_mode(monkeypatch):
    console, client, _ = setup()
    wires = []
    monkeypatch.setattr(console, 'exchange', lambda r, wire, *args: wires.append(wire) or 'OK')
    corr, calls = corrections()
    console.submit(request(console, client, 'AT'))
    console.tick(NS(), corr)
    assert wires == [b'AT\r\n'] and not console.exclusive
    assert calls == ['pause', 'resume']


@pytest.mark.parametrize('exit_reply', ['ack', 'telemetry', 'binary', 'silent', 'invalid'])
def test_real_pty_fragmented_reply_and_telemetry_preserved(exit_reply):
    console, client, events = setup()
    master, slave = os.openpty()
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.monitor = console.traffic
    frames, sent = [], []
    reader.on_control_frame = frames.append
    done = threading.Event()
    def firmware():
        pending = b''
        while not done.is_set():
            console.set_guard(dict(session=console.session, client=client, allowed=True, sent_at=time.time()))
            if not select.select([master], [], [], .05)[0]:
                continue
            pending += os.read(master, 4096)
            while b'\r\n' in pending:
                cmd, pending = pending.split(b'\r\n', 1)
                sent.append(cmd)
                reply = {b'AT+CFG': b'+CFG\r\n', b'AT+VERSION?': b'+VERSION=V10277.1.3\r\n',
                         b'AT+EXIT': b'+EXIT\r\n'}[cmd]
                if cmd == b'AT+VERSION?':
                    os.write(master, b'$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n')
                if cmd == b'AT+EXIT' and exit_reply != 'ack':
                    if exit_reply == 'telemetry':
                        from nav_rtk.parser.checksum import calculate_nmea_checksum_bytes
                        body = b'WTRTK,0.000,0.000,0.000,0.000,176.60,-0.46,-6.00,0'
                        os.write(master, b'$'+body+b'*'+calculate_nmea_checksum_bytes(body).encode()+b'\r\n')
                    elif exit_reply == 'binary':
                        from nav_rtk.parser.checksum import calculate_nmea_checksum_bytes
                        body = b'WTRAW,' + b'\x00' * 200
                        os.write(master, b'$'+body+b'*'+calculate_nmea_checksum_bytes(body).encode()+b'\r\n')
                    elif exit_reply == 'invalid':
                        os.write(master, b'$WTRTK,corrupt*00\r\n')
                    continue
                os.write(master, reply[:3]); time.sleep(.01); os.write(master, reply[3:])
    reader.open()
    thread = threading.Thread(target=firmware)
    thread.start()
    try:
        console.submit(request(console, client))
        console.tick(reader, corrections()[0])
        last = console.results[-1]
        if exit_reply == 'ack':
            assert last['state'] == 'acknowledged'
            assert console.mode == 'normal'
        else:
            assert last['state'] == 'unconfirmed'
            assert last['reply'] == '+VERSION=V10277.1.3'
            assert '查询固件版本成功' in last['message']
            assert '退出配置模式未确认' in last['message']
            assert '差分及自动恢复仍暂停' in last['message']
            assert console.mode == 'unknown' and console.exclusive
            if exit_reply in ('telemetry', 'binary'):
                assert '已观察到1帧校验有效' in last['message']
            else:
                assert '未观察到校验有效' in last['message']
        assert any(r.get('reply') == '+VERSION=V10277.1.3' and r['state'] == 'running'
                   for r in console.results)
        assert [r['command'] for r in console.results if r.get('sent')] == ['CFG', 'VERSION?', 'EXIT']
        assert sent == [b'AT+CFG', b'AT+VERSION?', b'AT+EXIT']
        assert any(frame.text.startswith('$GNGGA') for frame in frames)
        assert {row['direction'] for row in console.records} == {'RX', 'TX'}
    finally:
        done.set(); thread.join(2); reader.close(); os.close(master); os.close(slave)


def test_readback_failure_keeps_setter_ack_without_claiming_verified(monkeypatch):
    console, client, _ = setup()
    console.mode = 'config'
    wires = []
    def exchange(reader, wire, row, req, value=''):
        wires.append(wire)
        if row.key == 'GGA?':
            raise TimeoutError('读回超时')
        return '+GGA=6'
    monkeypatch.setattr(console, 'exchange', exchange)
    console.submit(request(console, client, 'GGA', value='6', confirmed=True))
    console.tick(NS(), corrections()[0])
    last = console.results[-1]
    assert last['state'] == 'unconfirmed' and last['reply'] == '+GGA=6'
    assert '设置读回未确认' in last['message'] and '读回值已确认' not in last['message']
    assert wires == [b'AT+GGA=6\r\n', b'AT+GGA?\r\n', b'AT+EXIT\r\n']
    assert console.exclusive


@pytest.mark.parametrize('failure', ['CFG', 'VERSION?'])
def test_failure_names_current_step_without_claiming_query_success(monkeypatch, failure):
    console, client, _ = setup()
    sent = []
    def exchange(reader, wire, row, req, value=''):
        sent.append(row.key)
        if row.key == failure or (failure == 'CFG' and row.key == 'VERSION?'):
            raise TimeoutError('没有回执')
        return '+CFG'
    monkeypatch.setattr(console, 'exchange', exchange)
    console.submit(request(console, client, _completed='伪造成功', _reply='伪造回执'))
    console.tick(NS(), corrections()[0])
    last = console.results[-1]
    assert last['stage'] == ('确认配置状态' if failure == 'CFG' else BY_KEY[failure].title)
    assert last['state'] == 'unconfirmed' and not last['completed'] and not last['reply']
    assert '查询固件版本成功' not in last['message']
    assert sent == ['CFG', 'VERSION?', 'EXIT']
    assert last['cleanup'] == '已退出配置'


@pytest.mark.parametrize('main', ['GGA', 'CAL', 'RTKMODE', 'BAUD', 'CUSTOM'])
def test_missing_config_proof_never_sends_main_but_exits_once(monkeypatch, main):
    console, client, _ = setup()
    sent = []
    def exchange(reader, wire, row, req, value=''):
        sent.append(row.key)
        if row.key != 'EXIT':
            raise TimeoutError('缺回执')
        return '+EXIT'
    monkeypatch.setattr(console, 'exchange', exchange)
    extra = {'confirmed': True}
    if main == 'CUSTOM':
        extra.update(text='AT+UNKNOWN', encoding='text')
    elif BY_KEY[main].choices:
        extra['value'] = BY_KEY[main].choices[0][1]
    console.submit(request(console, client, main, **extra))
    console.tick(NS(), corrections()[0])
    assert sent == ['CFG', 'VERSION?', 'EXIT']
    assert console.mode == 'normal' and console.exclusive
    assert console.results[-1]['state'] == 'unconfirmed'


@pytest.mark.parametrize('stop,manual', [(True, False), (False, True)])
def test_cleanup_does_not_write_after_shutdown_or_in_manual_mode(monkeypatch, stop, manual):
    console, client, _ = setup()
    console.mode = 'config'
    sent = []
    def exchange(reader, wire, row, req, value=''):
        sent.append(row.key)
        if stop:
            console.stop_event.set()
        raise TimeoutError('查询超时')
    monkeypatch.setattr(console, 'exchange', exchange)
    console.submit(request(console, client, automatic=not manual))
    console.tick(NS(), corrections()[0])
    assert sent == ['VERSION?']
