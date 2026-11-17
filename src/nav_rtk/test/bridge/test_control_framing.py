"""Firmware ACK framing: never accept telemetry or bytes predating a command."""
import pytest
from nav_rtk.bridge.termios_reader import TermiosLineReader
from nav_rtk.parser.serial_client import SerialPortUnavailableError


def run_command(monkeypatch, chunks, prefix=b''):
    import nav_rtk.bridge.termios_reader as module
    reader = TermiosLineReader('/unused', 230400, writable=True)
    reader._fd = 123
    reader.write = lambda data: None
    if prefix:
        reader._scan_control_frames(prefix)
    def read(*args):
        chunk = chunks.pop(0)
        reader._scan_control_frames(chunk)
        return chunk
    monkeypatch.setattr(reader, '_read', read)
    monkeypatch.setattr(module.select, 'select', lambda *args: ([123] if chunks else [], [], []))
    reader.command(b'AT+OPEN ROVER', b'ROVER OPEN OK', timeout=.01)
    return reader


def test_split_exact_ack_is_accepted(monkeypatch):
    reader = run_command(monkeypatch, [b'ROVER OPEN ', b'OK\r\n'])
    assert reader.control_diagnostics['commands'][-1]['acknowledged']


@pytest.mark.parametrize('chunks,prefix', [
    ([b'junk ROVER OPEN OK\r\n'], b''),
    ([b'OK\r\n'], b'ROVER OPEN '),
    ([b'ROVER OPEN OK'], b''),
    ([b'$WTRAW,'+b'\0'*30+b'ROVER OPEN OK\r\n'+b'\0'*160], b''),
])
def test_partial_stale_embedded_and_binary_acks_fail_closed(monkeypatch, chunks, prefix):
    with pytest.raises(SerialPortUnavailableError):
        run_command(monkeypatch, chunks, prefix)


def test_control_consumed_telemetry_is_forwarded_once():
    reader = TermiosLineReader('/unused', 230400, writable=True)
    frames = []
    reader.on_control_frame = frames.append
    chunk = b'$GNGST,120000,1,1,1,0,0.1,0.2,0.3*00\r\nROVER OPEN OK\r\n'
    reader._scan_control_frames(chunk[:20])
    reader._observe_control_chunk(chunk[:20])
    reader._scan_control_frames(chunk[20:])
    reader._observe_control_chunk(chunk[20:])
    assert [frame.text for frame in frames] == [
        '$GNGST,120000,1,1,1,0,0.1,0.2,0.3*00', 'ROVER OPEN OK']
