"""Bounded debug transactions, executed ONLY by the existing serial-owner thread."""
import base64
from collections import deque
import json
import re
import select
import threading
import time
import uuid

from nav_rtk.parser.checksum import validate_nmea_sentence_bytes

from .command_catalog import BY_KEY, prepare, custom_command, BAUDS


class DebugConsole:
    def __init__(self, evidence, stop_event):
        self.evidence, self.stop_event = evidence, stop_event
        self.session = uuid.uuid4().hex
        self.lock = threading.RLock()
        self.pending = None
        self.guard = {}
        self.guard_at = 0.
        self.seen = deque(maxlen=128)
        self.records = deque(maxlen=256)
        self.sequence = 0
        self.trimmed = 0
        self.results = deque(maxlen=24)
        self.exclusive = False
        self.mode = 'unknown'
        self.owner = ''
        self.busy = False

    def traffic(self, direction, data):
        # The durable EvidenceWriter separately receives ALL original bytes.
        # Live preview is bounded and explicitly reports trimmed records.
        with self.lock:
            for offset in range(0, len(data), 512):
                self.sequence += 1
                if len(self.records) == self.records.maxlen:
                    self.trimmed += 1
                self.records.append(dict(seq=self.sequence, wall_ns=time.time_ns(),
                                         direction=direction,
                                         base64=base64.b64encode(data[offset:offset+512]).decode()))

    def set_guard(self, value):
        if not isinstance(value, dict) or value.get('session') != self.session:
            return
        sent = value.get('sent_at')
        if not isinstance(sent, (int, float)) or not 0 <= time.time()-sent <= 2:
            return
        with self.lock:
            self.guard = value
            self.guard_at = time.monotonic()

    def allowed(self, client):
        with self.lock:
            return (time.monotonic() - self.guard_at < 2.
                    and self.guard.get('client') == client
                    and self.guard.get('allowed') is True)

    def result(self, request, state, message, **extra):
        row = dict(id=request.get('id', ''), state=state, message=message,
                   wall_ns=time.time_ns(), **extra)
        with self.lock:
            self.results.append(row)
        self.evidence('debug_result', result=row)

    def submit(self, request):
        if not isinstance(request, dict):
            return
        ident = request.get('id')
        if not isinstance(ident, str) or not re.fullmatch(r'[a-f0-9]{32}', ident):
            return
        with self.lock:
            if ident in self.seen:
                return
            self.seen.append(ident)
            valid = (request.get('session') == self.session
                     and isinstance(request.get('sent_at'), (float, int))
                     and 0 <= time.time()-request['sent_at'] <= 3
                     and self.allowed(request.get('client')))
            if not valid:
                self.result(request, 'rejected', '页面授权过期或有任务运行，请刷新后重试')
                return
            if self.busy or self.pending or (self.exclusive and self.owner != request.get('client')):
                self.result(request, 'rejected', '另一条指令或调试会话尚未结束')
                return
            self.pending = dict(request)
            self.result(request, 'queued', '等待串口执行')

    def snapshot(self):
        with self.lock:
            return dict(session=self.session, exclusive=self.exclusive, busy=self.busy or bool(self.pending),
                        mode=self.mode, sequence=self.sequence, trimmed=self.trimmed,
                        records=list(self.records), results=list(self.results))

    @staticmethod
    def matches(row, reply, value=''):
        if not row.reply:
            return False
        if row.key == 'VERSION?':
            return re.fullmatch(rb'\+VERSION=[A-Za-z0-9._-]+', reply) is not None
        if row.key == 'MSG?':
            return re.fullmatch(rb'\+MSG=[A-Za-z0-9, ]*', reply) is not None
        if row.reply.endswith('='):
            expected = row.reply.encode()
            if value:
                return reply == expected + value.encode()
            suffix = reply[len(expected):] if reply.startswith(expected) else b''
            return bool(suffix) and suffix.isdigit()
        return reply == row.reply.encode()

    def exchange(self, reader, wire, row, request, value='', timeout=1.2):
        if self.stop_event.is_set() or (row.key != 'EXIT' and not self.allowed(request['client'])):
            raise RuntimeError('任务状态或页面授权已变化，未发送后续指令')
        # Drain already queued bytes into the normal evidence/telemetry path.
        # A previous reply must not acknowledge a newly submitted transaction.
        for _ in range(16):
            if not select.select([reader._fd], [], [], 0)[0]:
                break
            try:
                reader._observe_control_chunk(reader._read(reader._fd, reader.read_chunk))
            except BlockingIOError:
                break
        else:
            raise RuntimeError('串口输入积压，未发送；请等待数据读取后再试')
        after = reader._reply_received
        reader.write(wire)
        self.evidence('debug_send', id=request['id'], command=row.key,
                      wire_hex=wire.hex(), automatic=wire != request.get('_wire'))
        self.result(request, 'running', f'{request.get("_stage", row.title)}：已发送，等待回执',
                    command=row.key, sent=wire.decode('ascii', errors='backslashreplace').rstrip('\r\n'),
                    expected=row.reply)
        telemetry_frames = 0
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                raise RuntimeError('正在停止RTK，操作结果未确认')
            if not self.allowed(request['client']) and row.key != 'EXIT':
                raise RuntimeError('任务状态或页面授权已变化，停止后续指令')
            if not select.select([reader._fd], [], [], .05)[0]:
                continue
            try:
                chunk = reader._read(reader._fd, reader.read_chunk)
            except BlockingIOError:
                continue
            reader._observe_control_chunk(chunk)
            for start, raw, ascii_frame in reader._reply_frames:
                if start < after:
                    continue
                # Only complete, checksum-valid frames received after EXIT count
                # as output evidence. Output is NOT a substitute for its ACK.
                if row.key == 'EXIT' and raw.startswith((b'$WTRAW,', b'$WTRTK,', b'$G')):
                    if validate_nmea_sentence_bytes(raw):
                        telemetry_frames += 1
                if not ascii_frame:
                    continue
                if raw in (b'ERROR', b'+ERROR') or raw.startswith(b'+ERROR='):
                    raise RuntimeError('接收器返回错误：'+raw.decode())
                if self.matches(row, raw, value):
                    return raw.decode('ascii')
        if row.key == 'EXIT':
            observed = (f'已观察到{telemetry_frames}帧校验有效的普通报文，但不能据此确认配置模式已退出'
                        if telemetry_frames else '等待期间未观察到校验有效的普通报文')
            raise TimeoutError('未收到退出回执 +EXIT；'+observed+'；不自动重发')
        raise TimeoutError('未收到本条指令的完整回执；不能确定是否执行，不自动重发')

    def step(self, reader, wire, row, request, value='', stage=None):
        """Publish each ACK before starting the next step, including cleanup."""
        request['_stage'] = stage or row.title
        reply = self.exchange(reader, wire, row, request, value)
        self.result(request, 'running', f'{request["_stage"]}：已收到有效回执',
                    command=row.key, reply=reply)
        return reply

    def enter_configuration(self, reader, request):
        try:
            self.step(reader, b'AT+CFG\r\n', BY_KEY['CFG'], request)
            return None
        except TimeoutError:
            # The receiver can enter CFG without returning +CFG. A single
            # configuration-only, read-only query establishes that state;
            # never repeat CFG or proceed with a setter on silence alone.
            return self.step(reader, b'AT+VERSION?\r\n', BY_KEY['VERSION?'], request,
                             stage='确认配置状态')

    def tick(self, reader, corrections):
        with self.lock:
            request, self.pending = self.pending, None
            if request is None:
                return False
            self.busy = True
        cleanup_required = cleanup_attempted = False
        link_change_attempted = False
        row = None
        try:
            request.pop('_completed', None)
            request.pop('_reply', None)
            request['_stage'] = '准备'
            if not self.allowed(request['client']) or time.time()-request['sent_at'] > 3:
                raise ValueError('任务状态已变化或指令过期，未发送')
            if request.get('key') == 'RESUME' and self.mode == 'normal':
                # EXIT was already acknowledged. The protocol does not promise
                # another EXIT reply outside configuration mode; don't send one.
                with self.lock:
                    self.exclusive, self.owner = False, ''
                reader.manual_control_active = False
                corrections.resume_after_debug()
                self.result(request, 'ended', '配置退出此前已确认，已结束调试并恢复正常管理')
                return True
            if request.get('key') == 'CUSTOM':
                row, wire, value = custom_command(request.get('text', ''), request.get('encoding', 'text'),
                                                  request.get('crlf', True), request.get('confirmed', False))
            elif request.get('key') == 'RESUME':
                row, wire = prepare('EXIT')
                value = ''
            else:
                value = request.get('value', '')
                row, wire = prepare(request.get('key'), value, request.get('confirmed', False))
            request['_wire'] = wire
            was_exclusive = self.exclusive
            with self.lock:
                self.exclusive, self.owner = True, request['client']
            reader.manual_control_active = True
            corrections.pause_for_debug()
            self.result(request, 'running', '正在执行；差分注入及自动恢复已暂停', command=row.key)
            automatic = request.get('automatic', True) is True
            entry_reply = None
            cleanup_required = automatic and row.key not in ('AT', 'CFG', 'EXIT')
            if row.key not in ('AT', 'CFG', 'EXIT') and self.mode != 'config':
                if not automatic:
                    raise ValueError('请先进入配置模式')
                self.mode = 'unknown'
                entry_reply = self.enter_configuration(reader, request)
                self.mode = 'config'
            if row.key in ('CFG', 'EXIT'):
                self.mode = 'unknown'
            if row.key == 'BAUD':
                # Firmware may change speed before returning ACK: never resend the setter.
                if self.stop_event.is_set() or not self.allowed(request['client']):
                    raise RuntimeError('任务状态或页面授权已变化，未发送波特率设置')
                request['_stage'] = row.title
                link_change_attempted = True
                reader.write(wire)
                self.evidence('debug_send', id=request['id'], command=row.key, wire_hex=wire.hex(), automatic=False)
                self.result(request, 'running', '设置波特率：已发送一次，切换本机速率后读回确认',
                            command=row.key, sent=wire.decode('ascii').rstrip('\r\n'))
                time.sleep(.1)
                reader.set_baudrate(int(dict((v, t) for t, v in BAUDS)[value]))
                reply = self.step(reader, b'AT+BAUD?\r\n', BY_KEY['BAUD?'], request, value, '波特率读回确认')
            elif row.key == 'VERSION?' and entry_reply is not None:
                # The fallback probe already returned the requested version.
                reply = entry_reply
            else:
                link_change_attempted = row.restart or row.key == 'CUSTOM'
                reply = self.step(reader, wire, row, request, value)
            request['_completed'] = row.title + ('成功' if row.key.endswith('?') else '已收到有效回执')
            request['_reply'] = reply
            if row.key == 'CFG':
                self.mode = 'config'
            elif row.key == 'EXIT':
                self.mode = 'normal'
            if row.restart:
                self.mode = 'unknown'
                self.result(request, 'acknowledged', '收到回执；设备重启与收报尚未确认，调试仍占用', reply=reply)
                return True
            if row.query and row.key != 'BAUD':
                query = BY_KEY[row.query]
                reply = self.step(reader, query.wire.encode()+b'\r\n', query, request, value, '设置读回')
            if row.query:
                request['_completed'] = row.title+'：读回值已确认'
                request['_reply'] = reply
            if automatic and self.mode == 'config' and row.key != 'CFG':
                self.mode = 'unknown'
                cleanup_attempted = True
                self.step(reader, b'AT+EXIT\r\n', BY_KEY['EXIT'], request)
                self.mode = 'normal'
            # Ending debug is explicit: don't silently undo manual OPEN/CLOSE or inject RTCM.
            if request.get('key') == 'RESUME' or (row.key == 'AT' and not was_exclusive):
                with self.lock:
                    self.exclusive, self.owner = False, ''
                reader.manual_control_active = False
                corrections.resume_after_debug()
            message = request['_completed']
            if self.exclusive:
                message += ('；配置退出已确认' if self.mode == 'normal' else '')
                message += '。调试占用保留，完成后请点击“结束调试并恢复接收”。'
            else:
                message += '；已结束调试并恢复正常管理。'
            message += '\n回执不代表有效定位或校准完成。'
            self.result(request, 'verified' if row.query else 'acknowledged',
                        message, reply=reply, summary=request['_completed'])
        except (ValueError, TypeError, KeyError, OSError, RuntimeError, TimeoutError) as exc:
            # No retry of setters, calibration or reset. Keep ownership when state is uncertain.
            failed_stage = request['_stage']
            if self.exclusive:
                self.mode = 'unknown'
            cleanup = ''
            # Finish an automatic configuration transaction on failure too.
            # Do not write after an uncertain reboot/custom/baud-changing send,
            # or retry EXIT that has already been attempted.
            if (cleanup_required and not cleanup_attempted and not link_change_attempted
                    and not self.stop_event.is_set()):
                try:
                    self.step(reader, b'AT+EXIT\r\n', BY_KEY['EXIT'], request, stage='退出配置模式')
                    self.mode = 'normal'
                    cleanup = '已退出配置'
                except (ValueError, TypeError, KeyError, OSError, RuntimeError, TimeoutError) as cleanup_exc:
                    cleanup = '退出未确认'
                    self.evidence('debug_cleanup_failed', id=request['id'], error=str(cleanup_exc))
            completed = request.get('_completed', '')
            summary = (completed+'；' if completed else '')+failed_stage+'未确认'
            if cleanup:
                summary += '；'+cleanup
            message = summary+'：'+str(exc)
            if self.exclusive:
                message += '\n调试占用保留，差分及自动恢复仍暂停。请核对原始收发，不要重复危险操作。'
            self.result(request, 'unconfirmed' if self.exclusive else 'rejected', message,
                        command=request.get('key'), stage=failed_stage,
                        completed=completed, reply=request.get('_reply', ''),
                        summary=summary, failed_stage=failed_stage, cleanup=cleanup)
        finally:
            with self.lock:
                self.busy = False
        return True
