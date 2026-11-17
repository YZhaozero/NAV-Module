import os
import pty
import select
import threading
import time

import pytest

from nav_rtk.bridge.receiver_recovery import ReceiverRecovery
from nav_rtk.bridge.termios_reader import TermiosLineReader
from nav_rtk.parser.checksum import calculate_nmea_checksum
from nav_rtk.parser.serial_client import SerialPortUnavailableError


def sentence(body):
    return '$'+body+'*'+calculate_nmea_checksum(body)


GGA = sentence('GNGGA,120000.00,,,,,0,00,99.99,,,,,,')


class Correction:
    def __init__(self):
        self.opened = True
        self.closes = 0
        self.received = []
        self.state = {}

    def close(self, reader):
        assert reader is None and not self.opened
        self.closes += 1

    def update(self, **state):
        self.state.update(state)

    def feed(self, text):
        self.received.append(text)


def test_zero_fix_is_healthy_and_bad_or_truncated_gga_is_not(monkeypatch):
    import nav_rtk.bridge.receiver_recovery as module
    clock = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    guard = ReceiverRecovery();guard.connected()
    class Reader:
        def recover_satellite_stream(self, *args, **kwargs):
            raise AssertionError('Live quality-0 GGA must not reset')
    for i in range(100):
        clock[0] += 1
        guard.feed(GGA)
        assert not guard.tick(Reader(), Correction())
    before = guard.last_gga
    clock[0] += 1
    guard.feed(sentence('GNGGA,120000.00'))
    guard.feed(GGA[:-2]+'XX')
    assert guard.last_gga == before


def test_budget_survives_reconnect_and_stable_stream_rearms(monkeypatch):
    import nav_rtk.bridge.receiver_recovery as module
    clock = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    guard = ReceiverRecovery();guard.connected();correction = Correction();calls=[]
    class Reader:
        def recover_satellite_stream(self, level, **kwargs):
            calls.append(level)
            raise SerialPortUnavailableError('故障保持')
    for i in range(3):
        clock[0] += 11
        guard.connected()
        assert guard.tick(Reader(), correction)
        assert guard.suspended()
    clock[0] += 600
    assert not guard.tick(Reader(), correction)
    assert calls == [0, 1, 2] and correction.closes == 3
    assert guard.snapshot()['phase'] == 'blocked'
    for i in range(32):
        clock[0] += 1
        guard.feed(GGA)
    assert guard.attempts == 0 and not guard.suspended()


def test_short_return_of_gga_does_not_reset_attempt_budget(monkeypatch):
    import nav_rtk.bridge.receiver_recovery as module
    clock = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    guard = ReceiverRecovery();guard.connected();guard.attempts=2
    guard.feed(GGA)
    clock[0] += 15
    guard.feed(GGA)
    assert guard.attempts == 2


def test_return_after_failed_attempt_observes_before_rearming(monkeypatch):
    import nav_rtk.bridge.receiver_recovery as module
    clock = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    guard = ReceiverRecovery();guard.feed(GGA)
    guard.attempts = 3;guard.healthy_since = None
    guard.state = dict(phase='blocked', attempts=3)
    clock[0] += .1
    guard.feed(GGA)
    assert guard.snapshot()['phase'] == 'recovered'
    assert guard.attempts == 3 and not guard.suspended()


@pytest.mark.parametrize('recovers_at', [0, 1, 2])
def test_real_pty_owner_escalates_until_new_gga(recovers_at):
    master, slave = pty.openpty()
    port = os.ttyname(slave)
    done = threading.Event();commands=[];errors=[]
    def device():
        config=False;broken=True;pending=b'';next_output=0
        def emit(text):os.write(master, (text+'\r\n').encode())
        try:
            while not done.is_set():
                if select.select([master], [], [], .005)[0]:
                    pending += os.read(master,4096)
                    while b'\n' in pending:
                        line,pending=pending.split(b'\n',1);cmd=line.strip().decode()
                        commands.append(cmd)
                        if cmd=='AT+CFG':config=True;emit('+CFG')
                        elif cmd=='AT+VERSION?':emit('+VERSION=V10277.1.3')
                        elif cmd=='AT+CLOSE ROVER':
                            emit('ROVER CLOSE OK')
                            if recovers_at==0:broken=False
                        elif cmd=='AT+RESET':
                            config=False;emit('SOFTVERSION:10277.1.3')
                            if recovers_at==1:broken=False
                        elif cmd=='AT+RTKUTU=1':emit('+RTKUTU=1')
                        elif cmd.startswith('$QXMONVER'):
                            emit(sentence('QXMONVER,MC280M -02,SW,HW,SN'))
                        elif cmd.startswith('$QXCFGRST,1,0'):
                            emit('$ACK');broken=False
                        elif cmd=='AT+EXIT':config=False
                if not config and time.monotonic()>=next_output:
                    emit(GGA if not broken else sentence('WTRTK,0'))
                    next_output=time.monotonic()+.02
        except Exception as exc:errors.append(str(exc))
    worker=threading.Thread(target=device);worker.start()
    class FastTimeoutReader(TermiosLineReader):
        def _wait_gga_stream(self, timeout=1., stop_event=None):
            return super()._wait_gga_stream(min(timeout,.2),stop_event)
    guard=ReceiverRecovery(stop_event=done,timeout=.15,retry_delay=.01)
    correction=Correction()
    timer=threading.Timer(8,done.set);timer.start()
    try:
        with FastTimeoutReader(port,230400,writable=True) as reader:
            reader.on_control_frame=lambda f:guard.feed(f.text)
            for frame in reader.iter_frames(stop_event=done,on_idle=lambda r:guard.tick(r,correction)):
                guard.feed(frame.text)
                if guard.snapshot()['phase']=='recovered':break
        assert guard.snapshot()['phase']=='recovered'
        assert guard.attempts == recovers_at+1
        assert bool('AT+RESET' in commands)==(recovers_at>=1)
        assert any(c.startswith('$QXCFGRST') for c in commands)==(recovers_at==2)
        assert correction.received[-1]==GGA
    finally:
        done.set();timer.cancel();worker.join(2);os.close(master);os.close(slave)
    assert not worker.is_alive() and not errors


def test_unknown_firmware_never_receives_reset(monkeypatch):
    reader=TermiosLineReader('unused',230400,writable=True);writes=[]
    monkeypatch.setattr(reader,'command',lambda cmd,*a,**k: b'+VERSION=unknown' if cmd==b'AT+VERSION?' else b'+CFG')
    monkeypatch.setattr(reader,'write',writes.append)
    with pytest.raises(SerialPortUnavailableError,match='版本'):
        reader.recover_satellite_stream(1)
    assert writes==[b'AT+EXIT\r\n']


def test_ack_alone_never_counts_as_recovery(monkeypatch):
    reader=TermiosLineReader('unused',230400,writable=True)
    monkeypatch.setattr(reader,'command',lambda cmd,*a,**k: b'+VERSION=V10277.1.3' if cmd==b'AT+VERSION?' else b'OK')
    monkeypatch.setattr(reader,'write',lambda b:None)
    def absent(**kwargs):raise SerialPortUnavailableError('没有GGA')
    monkeypatch.setattr(reader,'_wait_gga_stream',absent)
    with pytest.raises(SerialPortUnavailableError,match='没有GGA'):
        reader.recover_satellite_stream(0)
