"""Bounded NTRIP transport. Network thread never opens or writes a serial port."""
import base64
from collections import deque
import logging
import math
import socket
import ssl
import threading
import time


class NtripError(RuntimeError):
    pass


class AuthenticationError(NtripError):
    pass


def fresh_gga(sentence, wall_time=None):
    """Accept only a checksummed real fix with a current UTC time-of-day."""
    return not gga_rejection_reason(sentence, wall_time)


def gga_rejection_reason(sentence, wall_time=None):
    """Explain the same GGA gate used by the transport, without guessing a fix."""
    if not sentence:
        return "尚未收到卫星定位报文"
    try:
        data, checksum = sentence.strip().split("*")
        total = 0
        for byte in data[1:].encode("ascii"):
            total ^= byte
        fields = data.split(",")
        if total != int(checksum, 16) or fields[0] not in ("$GNGGA", "$GPGGA"):
            return "卫星定位报文校验失败或类型不支持"
        quality = int(fields[6])
        if quality == 0:
            return "接收器尚未定位（定位质量 0）"
        if quality not in (1, 2, 4, 5):
            return "接收器定位类型不支持差分"
        if fields[3] not in ("N", "S") or fields[5] not in ("E", "W") or not fields[2] or not fields[4]:
            return "接收器未提供有效经纬度"
        if int(fields[7]) < 4:
            return "用于定位的卫星不足 4 颗"
        for value, limit in ((fields[2], 90), (fields[4], 180)):
            val = float(value)
            deg, minutes = divmod(val, 100)
            if not math.isfinite(val) or not 0 <= minutes < 60 or not 0 <= deg + minutes / 60 <= limit:
                return "接收器经纬度格式无效"
        stamp = fields[1]
        hour, minute, second = int(stamp[:2]), int(stamp[2:4]), float(stamp[4:])
        if not (0 <= hour < 24 and 0 <= minute < 60 and 0 <= second < 61):
            return "卫星 UTC 时间无效"
        now = time.time() if wall_time is None else wall_time
        delta = (hour * 3600 + minute * 60 + second - now % 86400 + 43200) % 86400 - 43200
        if not math.isfinite(delta):
            return "卫星 UTC 时间无效"
        return "" if abs(delta) <= 15 else f"卫星时间与设备时间相差 {abs(delta):.1f} 秒"
    except (ValueError, IndexError, UnicodeError, OverflowError, AttributeError, TypeError):
        return "卫星定位报文格式无效"


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class RtcmDecoder:
    def __init__(self):
        self.buffer = b""

    def feed(self, data):
        self.buffer += data
        packets = []
        while len(self.buffer) >= 3:
            start = self.buffer.find(b"\xd3")
            if start < 0:
                self.buffer = b""
                break
            self.buffer = self.buffer[start:]
            if len(self.buffer) < 3:
                break
            if self.buffer[1] & 0xFC:
                self.buffer = self.buffer[1:]
                continue
            size = ((self.buffer[1] & 3) << 8 | self.buffer[2]) + 6
            if len(self.buffer) < size:
                break
            packet = self.buffer[:size]
            if crc24q(packet[:-3]) == int.from_bytes(packet[-3:], "big"):
                packets.append(packet)
                self.buffer = self.buffer[size:]
            else:
                self.buffer = self.buffer[1:]
        return packets


class Body:
    """Incremental HTTP/ICY response with bounded headers and chunk lengths."""
    def __init__(self):
        self.on_status = None
        self.header = False
        self.chunked = False
        self.remaining = None
        self.buffer = b""

    def feed(self, data):
        self.buffer += data
        if not self.header:
            if self.buffer.startswith(b"ICY "):
                end = self.buffer.find(b"\r\n")
                if end < 0:
                    if len(self.buffer) > 4096:
                        raise NtripError("差分服务响应无效")
                    return b""
                first, headers = self.buffer[:end], b""
                self.buffer = self.buffer[end+2:]
            else:
                end = self.buffer.find(b"\r\n\r\n")
                if end < 0:
                    if len(self.buffer) > 16384:
                        raise NtripError("差分服务响应过长")
                    return b""
                headers = self.buffer[:end].lower()
                first = self.buffer.split(b"\r\n", 1)[0]
                self.buffer = self.buffer[end+4:]
            fields = first.split()
            if len(fields) < 2 or fields[0] not in (b"ICY", b"HTTP/1.0", b"HTTP/1.1"):
                raise NtripError("服务器未返回差分数据，请检查地址和挂载点")
            if self.on_status:
                self.on_status(fields[1].decode("ascii",errors="replace")[:3])
            if fields[1] in (b"401", b"403"):
                raise AuthenticationError("账号认证失败，请检查账号、密码及有效期后重新开启")
            if fields[1] in (b"408", b"429") or (len(fields[1]) == 3 and fields[1].startswith(b"5")):
                raise NtripError("差分服务暂时不可用（HTTP " + fields[1].decode('ascii', errors='replace') + "）")
            if fields[1] != b"200":
                raise AuthenticationError("差分服务拒绝连接，请检查挂载点后重新开启")
            if b"sourcetable" in headers:
                raise AuthenticationError("收到挂载点列表，请填写具体挂载点后重新开启")
            self.header = True
            self.chunked = any(line.split(b":", 1)[0].strip() == b"transfer-encoding"
                               and b"chunked" in line.split(b":", 1)[1].strip()
                               for line in headers.split(b"\r\n") if b":" in line)
        if not self.chunked:
            result, self.buffer = self.buffer, b""
            return result
        result = bytearray()
        while True:
            if self.remaining is None:
                end = self.buffer.find(b"\r\n")
                if end < 0:
                    if len(self.buffer) > 128:
                        raise NtripError("差分数据分块无效")
                    break
                try:
                    self.remaining = int(self.buffer[:end].split(b";", 1)[0], 16)
                except ValueError as exc:
                    raise NtripError("差分数据分块无效") from exc
                self.buffer = self.buffer[end+2:]
                if not 0 < self.remaining <= 65536:
                    raise NtripError("差分数据流已结束或分块过大")
            if len(self.buffer) < self.remaining + 2:
                break
            if self.buffer[self.remaining:self.remaining+2] != b"\r\n":
                raise NtripError("差分数据分块无效")
            result.extend(self.buffer[:self.remaining])
            self.buffer = self.buffer[self.remaining+2:]
            self.remaining = None
        return bytes(result)


class Client:
    def __init__(self, config, get_gga, live_no_fix=None):
        self.config = dict(config)
        self.get_gga = get_gga
        self.live_no_fix = live_no_fix or (lambda: False)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.queue = deque(maxlen=64)
        self.state = dict(phase="waiting", message="等待有效卫星定位", received_bytes=0, rtcm_packets=0,
                          session_active=False)
        self.socket = None
        self.thread = threading.Thread(target=self.run, daemon=True, name="nav-ntrip")
        self.evidence = None
        self._evidence_state = None

    def trace(self, event, **details):
        if self.evidence:
            self.evidence(event, **details)

    def update(self, phase, message, **details):
        with self.lock:
            self.state.update(phase=phase, message=message, **details)
        if self._evidence_state != (phase,message):
            self.trace("ntrip_state", phase=phase, message=message, **details)
            self._evidence_state = (phase,message)

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def take(self):
        with self.lock:
            packets = []
            while self.queue and len(packets) < 4:
                stamp, packet = self.queue.popleft()
                if time.monotonic() - stamp < 2:
                    packets.append(packet)
        return packets

    def stop(self):
        self.stop_event.set()
        # The network thread may clear self.socket while shutdown wakes recv().
        sock = self.socket
        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        with self.lock:
            self.queue.clear()
            self.state['session_active'] = False

    def run(self):
        delay = 3
        while not self.stop_event.is_set():
            if not self.get_gga():
                self.update("waiting", "已开启，等待有效卫星定位")
                self.stop_event.wait(.5)
                continue
            try:
                self.session()
                delay = 3
            except AuthenticationError as exc:
                self.update("auth_error", str(exc), last_error=type(exc).__name__,
                            last_error_detail=str(exc))
                return  # No repeated paid-service login with a bad account.
            except (NtripError, OSError, ValueError) as exc:
                if self.stop_event.is_set():
                    return  # Closing our own socket is an intentional stop.
                if isinstance(exc, NtripError):
                    detail = str(exc)
                    message = detail + "，正在等待恢复"
                elif isinstance(exc, OSError):
                    detail = "errno=" + str(exc.errno)
                    message = "差分网络连接异常，正在等待重连"
                else:
                    detail = "差分数据格式无效"
                    message = "差分数据格式异常，正在等待重连"
                self.update("reconnecting", message, last_error=type(exc).__name__,
                            last_error_detail=detail)
                logging.getLogger(__name__).warning("NTRIP %s: %s", type(exc).__name__, detail)
            finally:
                if self.socket:
                    self.socket.close()
                    self.socket = None
                with self.lock:
                    self.queue.clear()
                    self.state['session_active'] = False
            self.stop_event.wait(delay)
            delay = min(30, delay * 2)

    def session(self):
        value = self.config
        self.update("connecting", "正在连接差分服务")
        self.trace("ntrip_connect", host=value["host"], port=value["port"], tls=value["tls"])
        sock = socket.create_connection((value["host"], value["port"]), timeout=4)
        self.socket = sock
        if self.stop_event.is_set():
            return
        if value["tls"]:
            sock = ssl.create_default_context().wrap_socket(sock, server_hostname=value["host"])
            self.socket = sock
        sock.settimeout(1)
        token = base64.b64encode((value["username"] + ":" + value["password"]).encode()).decode()
        gga = self.get_gga()
        if not gga:
            raise NtripError("等待有效卫星定位")
        headers = (f"GET /{value['mountpoint']} HTTP/1.1\r\nHost: {value['host']}:{value['port']}\r\n"
                   "Ntrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP NAV/1.0\r\n"
                   f"Authorization: Basic {token}\r\nNtrip-GGA: {gga}\r\nConnection: close\r\n\r\n")
        sock.sendall(headers.encode())
        self.trace("ntrip_gga", sentence=gga, transport="request")
        body, decoder = Body(), RtcmDecoder()
        body.on_status = lambda code: self.trace("ntrip_http_status", code=code)
        last_gga, last_rtcm = 0., time.monotonic()
        while not self.stop_event.is_set():
            now = time.monotonic()
            gga = self.get_gga()
            # A caster already selected from a real position can keep supplying
            # fresh observations while the rover reacquires satellites. Do not
            # disconnect it just because a *live* GGA reports quality zero.
            # Never resend stale coordinates or fabricate a replacement GGA.
            if not gga and not self.live_no_fix():
                raise NtripError("卫星定位已失效")
            if gga and now - last_gga >= 5:
                sock.sendall((gga + "\r\n").encode("ascii"))
                self.trace("ntrip_gga", sentence=gga, transport="stream")
                last_gga = now
            if now - last_rtcm > 15:
                raise NtripError("未收到有效差分数据")
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                raise NtripError("连接已断开")
            decoded_body = body.feed(data)
            if decoded_body and self.evidence:
                self.evidence.bytes("ntrip_body_rx", decoded_body)
            packets = decoder.feed(decoded_body)
            if packets:
                last_rtcm = now
                with self.lock:
                    lost = max(0, len(self.queue)+len(packets)-self.queue.maxlen)
                    self.queue.extend((now, packet) for packet in packets)
                    self.state["received_bytes"] += sum(map(len, packets))
                    self.state["rtcm_packets"] += len(packets)
                self.trace("ntrip_rtcm", packets=len(packets), bytes=sum(map(len,packets)), queue_dropped=lost)
                self.update("receiving", "已收到有效差分数据" if gga else "定位暂失，继续接收差分以帮助恢复",
                            session_active=True)
