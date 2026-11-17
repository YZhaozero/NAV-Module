"""Enable rover measurements on the verified MC280M, on its existing serial owner.

Settings are read back and re-applied after reconnect. No FLASH save, reset,
baud change, base mode, or RTCM input-port selection is performed.
"""
import math
import select
import time

from nav_rtk.parser.checksum import calculate_nmea_checksum, validate_nmea_sentence
from nav_rtk.parser.serial_client import SerialPortUnavailableError


def sentence(body):
    return ('$' + body + '*' + calculate_nmea_checksum(body)).encode('ascii')


def rate_plan(original, divisor):
    fields = original.split(',')
    if fields[:3] != ['QXCFGMSG', '2', '0']:
        raise ValueError('Unexpected RTCM message setting')
    rates = [int(v) for v in fields[3:]]
    if any(not 0 <= n <= 255 for n in rates):
        raise ValueError('Unsupported RTCM output rate')
    if len(rates) == 1:
        return 'QXCFGMSG,2,0,' + str(divisor), rates[0]
    # BG1101V3.05.88 returns six slots. A scalar setter changes slots 1 and 3.
    # Verified on the physical receiver; do not treat the first zero as off.
    if len(rates) != 6 or any(rates[i] for i in (0, 2, 4, 5)) or rates[1] != rates[3]:
        raise ValueError('Unverified RTCM rate layout; original settings retained')
    return f'QXCFGMSG,2,0,0,{divisor},0,{divisor},0,0', rates[1]


def enable_raw_output(reader, stop_event=None):
    result = dict(phase='configuring', message='正在开启原始卫星测量输出')
    original_port = original_message = restore_rate = None
    changed = False

    def check_stop():
        if stop_event is not None and stop_event.is_set():
            raise InterruptedError('RTK connection closing')

    def query(body, check_cancel=True):
        if check_cancel:
            check_stop()
        time.sleep(.1)
        after = reader._reply_received
        reader.write(sentence(body) + b'\r\n')
        deadline = time.monotonic() + .7
        prefix = ('$' + body.split(',')[0] + ',').encode()
        while time.monotonic() < deadline:
            if check_cancel:
                check_stop()
            if not select.select([reader._fd], [], [], .05)[0]:
                continue
            try:
                reader._observe_control_chunk(reader._read(reader._fd, reader.read_chunk))
            except BlockingIOError:
                continue
            for start, raw, ascii_frame in reader._reply_frames:
                if (ascii_frame and start >= after and raw.startswith(prefix)
                        and validate_nmea_sentence(raw.decode('ascii'))):
                    return raw[1:raw.rfind(b'*')].decode('ascii')
        raise SerialPortUnavailableError('原始测量配置未得到确认：' + body.split(',')[0])

    def write(body):
        time.sleep(.1)
        reader.write(sentence(body) + b'\r\n')
        # Drain the ACK before issuing readback; an ACK never substitutes for it.
        deadline = time.monotonic() + .15
        while time.monotonic() < deadline:
            if select.select([reader._fd], [], [], .03)[0]:
                try:
                    reader._observe_control_chunk(reader._read(reader._fd, reader.read_chunk))
                except BlockingIOError:
                    pass

    try:
        check_stop()
        try:
            reader.command(b'AT+CFG', b'+CFG', timeout=.5)
        except SerialPortUnavailableError:
            pass
        time.sleep(.1)
        version = reader.command(b'AT+VERSION?', b'+VERSION=', timeout=.5)
        if version not in (b'+VERSION=V10277.1.3', b'+VERSION=10277.1.3'):
            raise ValueError('接收器版本尚未验证原始测量配置')
        time.sleep(.1)
        reader.command(b'AT+RTKUTU=1', b'+RTKUTU=1', timeout=.7)
        identity = query('QXMONVER').split(',')
        if len(identity) != 5 or identity[1].strip() != 'MC280M -02' or identity[2] != 'BG1101V3.05.88':
            raise ValueError('卫星芯片版本尚未验证原始测量配置')
        original_port = query('QXCFGPRT,,')
        original_message = query('QXCFGMSG,2,0')
        mode = query('QXCFGMODE')
        rate = query('QXCFGRATE')
        if mode != 'QXCFGMODE,1':
            raise ValueError('原始测量配置仅适用于流动站')
        period_ms = int(rate.split(',')[1])
        if not 50 <= period_ms <= 10000:
            raise ValueError('Unexpected navigation period')
        divisor = max(1, math.ceil(1000 / period_ms))  # at most 1 Hz; retain full MSM cells
        expected_message, restore_rate = rate_plan(original_message, divisor)
        fields = original_port.split(',')
        if len(fields) != 7 or fields[1:3] != ['0', '0']:
            raise ValueError('原生端口未验证，保留原配置')
        fields[-1] = f'h{int(fields[-1][1:],16) | 4:08X}'
        expected_port = ','.join(fields)
        result.update(original_port=original_port, original_message=original_message,
                      navigation_period_ms=period_ms, output_divisor=divisor,
                      output_interval_ms=period_ms*divisor)
        if reader.evidence:
            reader.evidence('raw_gnss_config_before', **result)
        changed = True
        if original_port != expected_port:
            write(expected_port)
        if original_message != expected_message:
            write(f'QXCFGMSG,2,0,{divisor}')
        actual_port = query('QXCFGPRT,,')
        actual_message = query('QXCFGMSG,2,0')
        if actual_port != expected_port or actual_message != expected_message:
            raise ValueError('原始测量配置读回与请求不一致')
        result.update(phase='enabled', message='原始测量输出已开启，等待观测数据',
                      port=actual_port, output=actual_message)
    except (OSError, RuntimeError, ValueError, TypeError) as exc:
        result.update(phase='error', message='原始测量输出尚未启用', error=str(exc))
        if changed:
            try:
                write(f'QXCFGMSG,2,0,{restore_rate}')
                write(original_port)
                result['restored'] = (query('QXCFGPRT,,', False) == original_port
                                      and query('QXCFGMSG,2,0', False) == original_message)
            except (OSError, RuntimeError, ValueError, TypeError) as restore_exc:
                result['restore_error'] = str(restore_exc)
    finally:
        try:
            reader.write(b'AT+EXIT\r\n')
            reader._wait_gga_stream(timeout=1.5)
            result['stream_restored'] = True
        except (OSError, RuntimeError, ValueError, TypeError) as exc:
            result.update(stream_restored=False, exit_error=str(exc))
    return result
