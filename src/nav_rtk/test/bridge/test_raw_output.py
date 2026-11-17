"""Real PTY transactions, modelling only the verified MC280M commands."""
import os
import pty
import select
import threading
import time
import pytest
from nav_rtk.bridge.termios_reader import TermiosLineReader
from nav_rtk.bridge.raw_output import enable_raw_output, sentence, rate_plan


@pytest.mark.parametrize('failure', ['', 'readback', 'version', 'rate_timeout'])
def test_configure_readback_restore_and_exit_over_real_tty(failure):
    master, slave=pty.openpty()
    reader=TermiosLineReader(os.ttyname(slave),230400,writable=True)
    reader.open()
    stop=threading.Event(); commands=[]; state=dict(port='QXCFGPRT,0,0,,115200,h00000007,h00000003',rate=1,mode='normal')
    original=state['port']
    gga=sentence('GNGGA,120000.00,,,,,0,00,99.9,,,,,,')+b'\r\n'
    received=[];reader.on_control_gga=received.append
    def device():
        buf=b''
        while not stop.is_set():
            if select.select([master],[],[],.02)[0]:
                buf+=os.read(master,4096)
                while b'\r\n' in buf:
                    line,buf=buf.split(b'\r\n',1);commands.append(line)
                    if line==b'AT+CFG':state['mode']='cfg';os.write(master,b'+CFG\r\n')
                    elif line==b'AT+VERSION?':os.write(master,b'+VERSION='+ (b'unknown' if failure=='version' else b'V10277.1.3')+b'\r\n')
                    elif line==b'AT+RTKUTU=1':state['mode']='native';os.write(master,b'+RTKUTU=1\r\n')
                    elif line==b'AT+EXIT':state['mode']='normal'
                    elif line.startswith(b'$'):
                        body=line[1:].split(b'*')[0].decode();reply=None
                        if body=='QXMONVER':reply='QXMONVER,MC280M -02,BG1101V3.05.88,3.2,test'
                        elif body=='QXCFGPRT,,':reply=state['port']
                        elif body=='QXCFGMSG,2,0':reply=f"QXCFGMSG,2,0,0,{state['rate']},0,{state['rate']},0,0"
                        elif body=='QXCFGMODE':reply='QXCFGMODE,1'
                        elif body=='QXCFGRATE' and failure!='rate_timeout':reply='QXCFGRATE,100,1'
                        elif body.startswith('QXCFGPRT,0,0,'):state['port']=body
                        elif body.startswith('QXCFGMSG,2,0,') and failure!='readback':state['rate']=int(body.split(',')[-1])
                        if reply:
                            wire=sentence(reply)+b'\r\n'
                            os.write(master,wire[:6]);time.sleep(.005);os.write(master,wire[6:])
            if state['mode']=='normal':os.write(master,gga)
    thread=threading.Thread(target=device);thread.start()
    try:
        result=enable_raw_output(reader)
        assert result['stream_restored'] and received
        assert state['mode']=='normal'
        assert result['phase']==('error' if failure else 'enabled')
        if failure:
            assert state['port']==original and state['rate']==1
            if failure=='readback':assert result['restored']
        else:
            assert state['port'].endswith('h00000007') and state['rate']==10
            assert result['output_interval_ms']==1000
        assert not any(b'SAVE' in line or b'RESET' in line or b'BASE' in line for line in commands)
    finally:
        stop.set();thread.join(2);reader.close();os.close(slave);os.close(master)


def test_unknown_rate_layout_never_guesses_a_restore_value():
    with pytest.raises(ValueError):rate_plan('QXCFGMSG,2,0,0,1,0,2,0,0',10)
    assert rate_plan('QXCFGMSG,2,0,0,1,0,1,0,0',10)==('QXCFGMSG,2,0,0,10,0,10,0,0',1)


def test_rtcm_wtraw_ascii_mixed_and_fragmented():
    from nav_rtk.parser.ntrip import crc24q
    reader=TermiosLineReader('/unused',230400)
    rtcm=b'\xd3\x00\x0b'+b'$GNGGA,\r\nxx'
    rtcm+=crc24q(rtcm).to_bytes(3,'big')
    fake=rtcm+b'\r\n+VERSION=not-real\r\n'
    wtraw=b'$WTRAW,'+fake+b'\0'*(212-7-len(fake))
    gga=sentence('GNGGA,120000.00,,,,,0,00,99.9,,,,,,')
    packets=[];frames=[];reader.on_rover_rtcm=packets.append
    wire=wtraw+rtcm+gga+b'\r\n'+rtcm+b'\nOK\r\n'
    for index in range(0,len(wire),3):
        reader._scan_control_frames(wire[index:index+3])
        frames.extend((raw,ascii_frame) for _,raw,ascii_frame in reader._reply_frames)
    assert packets==[rtcm,rtcm]
    assert frames==[(wtraw,False),(gga,True),(b'OK',True)]


def test_late_gga_after_recovery_timeout_still_reenables_raw_output(monkeypatch):
    from types import SimpleNamespace as NS
    from nav_rtk.bridge import rtk_node, raw_output
    stop=threading.Event(); configured=[]; phases=iter(['retry_wait','recovered','recovered'])
    recovery=NS(connected=lambda:None,suspended=lambda:False)
    def tick(*args):
        recovery.phase=next(phases)
        return recovery.phase=='retry_wait'
    recovery.tick=tick;recovery.snapshot=lambda:dict(phase=recovery.phase)
    class Reader:
        def __init__(self,*args,**kwargs):self.baudrate=230400
        def open(self):pass
        def close(self):pass
        def iter_frames(self,stop_event,on_idle):
            # Recovery times out, then a late real GGA returns during normal RX.
            on_idle(self);assert len(configured)==1
            on_idle(self);assert len(configured)==2
            on_idle(self);assert len(configured)==2
            stop.set()
            return iter(())
    monkeypatch.setattr(rtk_node,'TermiosLineReader',Reader)
    monkeypatch.setattr(raw_output,'enable_raw_output',lambda *args:dict(phase='enabled'))
    node=NS(_stop_event=stop,_resolve_reader_port=lambda:'/unused',baudrate=230400,evidence=None,
            debug_console=NS(traffic=lambda *args:None,exclusive=False,tick=lambda *args:False),
            record_raw_gnss=True,raw_gnss=NS(receive=lambda data:None,configured=configured.append),
            _mark_serial_up=lambda:None,_receiver_recovery=recovery,
            corrections=NS(tick=lambda r:False,close=lambda r:None))
    rtk_node.RtkBridge._read_loop(node)


def test_manual_debug_defers_automatic_raw_configuration_until_released(monkeypatch):
    from types import SimpleNamespace as NS
    from nav_rtk.bridge import rtk_node, raw_output
    configured, writes = [], []
    node = NS(debug_console=NS(exclusive=True), record_raw_gnss=True,
              raw_gnss=NS(configured=configured.append), _stop_event=threading.Event())
    reader = object()
    monkeypatch.setattr(raw_output, 'enable_raw_output',
                        lambda active, stop: writes.append(active) or dict(phase='enabled'))
    assert not rtk_node.RtkBridge._configure_raw_output(node, reader)
    assert writes == [] and configured[-1]['phase'] == 'debug_paused'
    node.debug_console.exclusive = False
    assert rtk_node.RtkBridge._configure_raw_output(node, reader)
    assert writes == [reader] and configured[-1]['phase'] == 'enabled'
