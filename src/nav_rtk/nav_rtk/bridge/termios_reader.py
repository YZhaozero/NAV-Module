"""纯标准库串口读取，与 `nav_rtk.parser.serial_client.SerialLineReader` 接口对齐。

存在的理由：本机没有 pip、PyPI 不通，`python3-serial` 装不上，而串口读取只需要
termios 就能完成。这里复用 witrtk_parser 的 `SerialFrame` / `SerialPortUnavailableError`，
因此两个读取器可以互换，上层代码不需要知道用的是哪一个。
"""

from __future__ import annotations

import errno
import fcntl
import os
import re
import select
import termios
import time
from typing import Generator, Optional

from nav_rtk.parser.serial_client import SerialFrame, SerialPortUnavailableError

# termios 的波特率常量只在部分平台上存在，逐个取用，缺哪个就少支持哪个。
_BAUD_TABLE = {}
for _name in (
    "B9600", "B19200", "B38400", "B57600",
    "B115200", "B230400", "B460800", "B921600",
):
    if hasattr(termios, _name):
        _BAUD_TABLE[int(_name[1:])] = getattr(termios, _name)

# 单次 select 的最长等待，决定 stop_event 的响应延迟上限。
_SELECT_TIMEOUT_S = 0.05
# 单次 os.read 的上限；230400 baud 下 20 Hz 报文远小于这个值。
_READ_CHUNK = 4096
_STARTUP_SILENCE_S = 1.0
_CONFIG_QUERY_TIMEOUT_S = .35
_VERSION_REPLY = re.compile(rb"\+VERSION=([A-Za-z0-9][A-Za-z0-9._-]{0,63})")


class TermiosLineReader:
    """按行读取串口输出的 termios 实现，不依赖 pyserial。"""
    evidence = None
    monitor = None
    manual_control_active = False

    def _read(self, fd, size):
        data = os.read(fd,size)
        if not data:
            raise SerialPortUnavailableError("串口已断开，请检查接收机连接")
        if data and self.evidence:
            self.evidence.bytes("serial_rx",data)
        if data and self.monitor:
            self.monitor('RX', data)
        self._scan_control_frames(data)
        return data

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout: float = 1.0,
        read_chunk: int = _READ_CHUNK,
        writable: bool = False,
    ) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        # timeout 只为对齐 SerialLineReader 的签名而保留；实际读取由 select 驱动，
        # 不接受它就意味着上层不能靠超时来推进循环。
        self.timeout = timeout
        self.read_chunk = read_chunk
        self.writable = writable
        self._fd: Optional[int] = None
        self._saved_attrs = None
        self.on_control_frame = None
        self.on_control_gga = None
        self.on_rover_rtcm = None
        self._control_buffer = b""
        self._reply_buffer = b""
        self._reply_received = 0
        self._reply_frames = []
        self._reply_discard = False
        self.control_diagnostics = {}

    def _scan_control_frames(self, chunk):
        """Track framing across normal reads and AT exchanges without decoding binary."""
        self._reply_received += len(chunk)
        self._reply_buffer += chunk
        self._reply_frames = []
        while self._reply_buffer:
            if self._reply_discard:
                end = self._reply_buffer.find(b"\r\n")
                if end < 0:
                    self._reply_buffer = self._reply_buffer[-1:]
                    return
                self._reply_buffer = self._reply_buffer[end + 2:]
                self._reply_discard = False
            # A lone LF before a reply occurs on V10277.1.3. Only consume
            # separators at frame boundaries, never inside the IMU payload.
            self._reply_buffer = self._reply_buffer.lstrip(b"\r\n")
            if not self._reply_buffer:
                break
            start = self._reply_received - len(self._reply_buffer)
            if self._reply_buffer.startswith(b'\xd3'):
                from nav_rtk.parser.ntrip import crc24q
                if len(self._reply_buffer) < 3:
                    break
                if self._reply_buffer[1] & 0xfc:
                    self._reply_buffer = self._reply_buffer[1:]
                    continue
                size = ((self._reply_buffer[1] & 3) << 8 | self._reply_buffer[2]) + 6
                if len(self._reply_buffer) < size:
                    break
                packet = self._reply_buffer[:size]
                if crc24q(packet[:-3]) == int.from_bytes(packet[-3:], 'big'):
                    self._reply_buffer = self._reply_buffer[size:]
                    if self.on_rover_rtcm:
                        self.on_rover_rtcm(packet)
                else:
                    self._reply_buffer = self._reply_buffer[1:]
                continue
            if self._reply_buffer.startswith(b"$WTRAW,"):
                if len(self._reply_buffer) < 212:
                    break
                raw, self._reply_buffer = self._reply_buffer[:212], self._reply_buffer[212:]
                self._reply_frames.append((start, raw, False))
                continue
            end = self._reply_buffer.find(b"\n")
            if end < 0:
                if len(self._reply_buffer) > 8192:
                    self._reply_discard = True
                    self._reply_buffer = self._reply_buffer[-1:]
                break
            raw, self._reply_buffer = self._reply_buffer[:end].rstrip(b'\r'), self._reply_buffer[end + 1:]
            if raw and all(32 <= value <= 126 for value in raw):
                self._reply_frames.append((start, raw, True))

    def open(self) -> None:
        """打开串口并配置成 8N1 原始模式。"""

        if self.baudrate not in _BAUD_TABLE:
            raise ValueError(
                f"不支持的波特率 {self.baudrate}；"
                f"本平台可用：{sorted(_BAUD_TABLE)}")
        try:
            # O_NOCTTY：不要把串口变成进程的控制终端。
            # O_NONBLOCK：打开阶段不阻塞，避免设备未就绪时卡死在这里。
            self._fd = os.open(
                self.port, (os.O_RDWR if self.writable else os.O_RDONLY) | os.O_NOCTTY | os.O_NONBLOCK)
        except FileNotFoundError as exc:
            raise SerialPortUnavailableError("端口不存在") from exc
        except PermissionError as exc:
            raise SerialPortUnavailableError(
                f"没有权限打开 {self.port}（用户需在 dialout 组）") from exc
        except OSError as exc:
            if exc.errno == errno.EBUSY:
                # 上一个持有者设了 TIOCEXCL。多半是上一次运行留下的孤儿节点
                # （应用被 Ctrl-C / 杀进程时，它拉起的 ros2 launch 树不会被回收）。
                raise SerialPortUnavailableError(
                    f"{self.port} 已被另一个进程独占；"
                    f"先确认没有残留的 rtk_node 再启动") from exc
            raise SerialPortUnavailableError(
                f"打开 {self.port} 失败：{exc}") from exc

        # 串口**必须独占**。两个进程同时读同一个 tty 不会报错，而是各读走一半
        # 字节，两边的报文都被撕碎——实测两读者并存时校验失败率 62%
        # （2780/4451），单读者 0%（0/509）。那种状态下界面看起来就是「收不到
        # 数据」，极难往「端口被别人占着」上想。
        # TIOCEXCL 让后续 open() 直接 EBUSY 失败，把一个静默的数据损坏变成
        # 一条明确的报错。它是 fd 上的属性，本进程一退出/关闭就自动释放。
        try:
            fcntl.ioctl(self._fd, termios.TIOCEXCL)
        except OSError as exc:
            self.close()
            raise SerialPortUnavailableError(
                f"无法独占 {self.port}：{exc}") from exc

        try:
            self._saved_attrs = termios.tcgetattr(self._fd)
            attrs = termios.tcgetattr(self._fd)
            # 原始模式：关掉回显、行缓冲和信号处理，串口数据原样进出。
            attrs[0] = 0                                      # iflag
            attrs[1] = 0                                      # oflag
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0                                      # lflag
            attrs[4] = attrs[5] = _BAUD_TABLE[self.baudrate]  # ispeed / ospeed
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
            # 丢掉打开前残留在缓冲里的半截报文，否则第一帧可能是碎片。
            termios.tcflush(self._fd, termios.TCIFLUSH)
            self._reply_buffer = b""
            self._reply_frames = []
            self._reply_received = 0
            self._reply_discard = False
            if self.evidence:
                self.evidence("serial_open", port=self.port, baudrate=self.baudrate,
                              startup_input_flushed=True)
        except termios.error as exc:
            self.close()
            raise SerialPortUnavailableError(f"配置 {self.port} 失败：{exc}") from exc

    def close(self) -> None:
        """关闭串口，并把终端属性恢复成打开前的样子。"""

        if self._fd is None:
            return
        if self._saved_attrs is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSANOW, self._saved_attrs)
            except termios.error:
                pass
            self._saved_attrs = None
        try:
            os.close(self._fd)
        finally:
            self._fd = None
            if self.evidence:
                self.evidence("serial_close", port=self.port)

    def __enter__(self) -> "TermiosLineReader":
        self.open()
        return self

    def __exit__(self, *_exc_info) -> None:
        self.close()

    def iter_frames(
        self,
        stop_event: Optional[object] = None,
        pause_event: Optional[object] = None,
        on_idle=None,
    ) -> Generator[SerialFrame, None, None]:
        """持续读取串口，按行产出 `SerialFrame`。

        ASCII 按换行分帧；WTRAW 按固定长度分帧，RTCM 按长度及 CRC 分帧并
        交给 on_rover_rtcm。二进制内部的换行和文字不作为独立报文。
        """

        if self._fd is None:
            self.open()
        fd = self._fd
        if fd is None:  # pragma: no cover - open() 失败时会先抛异常
            raise SerialPortUnavailableError("端口未打开")

        startup_probe_pending = self.writable
        startup_deadline = time.monotonic() + _STARTUP_SILENCE_S
        while True:
            if stop_event is not None and getattr(stop_event, "is_set")():
                return
            if on_idle:
                on_idle(self)  # Control exchanges publish their own complete frames.
            if pause_event is not None and getattr(pause_event, "is_set")():
                # 暂停期间照读不误但直接丢弃，这样恢复时拿到的是当下最新数据，
                # 而不是暂停这段时间攒下来的积压。
                if select.select([fd], [], [], _SELECT_TIMEOUT_S)[0]:
                    try:
                        if not self._read(fd, self.read_chunk):
                            raise SerialPortUnavailableError("串口已断开，请检查接收机连接")
                        startup_probe_pending = False
                    except BlockingIOError:
                        pass
                    except OSError as exc:
                        raise SerialPortUnavailableError("串口已断开，请检查接收机连接") from exc
                continue
            try:
                ready = select.select([fd], [], [], _SELECT_TIMEOUT_S)[0]
            except (OSError, ValueError) as exc:
                raise SerialPortUnavailableError("串口等待失败，请检查接收机连接") from exc
            if not ready:
                if startup_probe_pending and not self.manual_control_active and time.monotonic() >= startup_deadline:
                    startup_probe_pending = False
                    self._exit_stranded_configuration()
                continue
            try:
                chunk = self._read(fd, self.read_chunk)
            except BlockingIOError:
                continue
            except OSError as exc:
                raise SerialPortUnavailableError("串口已断开，请检查接收机连接") from exc
            if not chunk:
                raise SerialPortUnavailableError("串口已断开，请检查接收机连接")
            startup_probe_pending = False
            for _, raw, ascii_frame in self._reply_frames:
                yield SerialFrame(text=raw.decode('ascii' if ascii_frame else 'utf-8', errors='replace'),
                                  raw_bytes=raw)

    def iter_lines(self) -> Generator[str, None, None]:
        """兼容旧接口，仅返回逐行文本。"""

        for frame in self.iter_frames():
            yield frame.text

    def set_baudrate(self, baudrate):
        """Change host speed on the same descriptor, from the sole owner thread."""
        speed = getattr(termios, 'B'+str(baudrate), None)
        if speed is None or self._fd is None:
            raise ValueError('无法切换串口波特率')
        attrs = termios.tcgetattr(self._fd)
        attrs[4] = attrs[5] = speed
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        self.baudrate = baudrate

    def write(self, data, timeout=.3):
        if self._fd is None or not self.writable:
            raise SerialPortUnavailableError("串口不可写")
        deadline, offset = time.monotonic() + timeout, 0
        while offset < len(data):
            left = deadline - time.monotonic()
            if left <= 0:
                raise SerialPortUnavailableError("差分串口写入超时")
            if select.select([], [self._fd], [], left)[1]:
                try:
                    written = os.write(self._fd, data[offset:])
                    if self.evidence and written:
                        self.evidence.bytes("serial_tx",data[offset:offset+written])
                    if self.monitor and written:
                        self.monitor('TX', data[offset:offset+written])
                    offset += written
                except BlockingIOError:
                    pass

    def _observe_control_chunk(self, chunk, after=None):
        # Control and telemetry share one descriptor. Keep publishing telemetry
        # while waiting for ACKs, including the no-ACK case (over 3 seconds).
        self.control_diagnostics["received_bytes"] = self.control_diagnostics.get("received_bytes", 0) + len(chunk)
        latest_gga = None
        for start, raw, ascii_frame in self._reply_frames:
            if not ascii_frame:
                # Preserve the complete binary frame for telemetry consumers,
                # without treating embedded CRLF/ASCII as separate messages.
                if self.on_control_frame:
                    self.on_control_frame(SerialFrame(
                        text=raw.decode("utf-8", errors="replace"), raw_bytes=raw))
                continue
            text = raw.decode("ascii")
            if self.on_control_frame:
                self.on_control_frame(SerialFrame(text=text, raw_bytes=raw))
            if raw.startswith((b"$GNGGA,", b"$GPGGA,")):
                from nav_rtk.parser.checksum import validate_nmea_sentence
                from nav_rtk.parser.nmea_parser import parse_nmea_sentence
                if len(raw.split(b",")) < 15 or not validate_nmea_sentence(text):
                    continue
                try:
                    parsed = parse_nmea_sentence(text)
                except (ValueError, OverflowError):
                    continue
                if parsed is None or parsed.data.fix_quality is None:
                    continue
                if self.on_control_gga:
                    self.on_control_gga(text)
                if after is None or start >= after:
                    latest_gga = text
                self.control_diagnostics["gga_frames"] = self.control_diagnostics.get("gga_frames", 0) + 1
        return latest_gga

    def command(self, command, expected, timeout=1):
        after = self._reply_received
        self.write(command + b"\r\n")
        trace = dict(command=command.decode("ascii"), acknowledged=False)
        self.control_diagnostics.setdefault("commands", []).append(trace)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if select.select([self._fd], [], [], .05)[0]:
                try:
                    chunk = self._read(self._fd, self.read_chunk)
                    self._observe_control_chunk(chunk)
                except BlockingIOError:
                    continue
                except OSError as exc:
                    raise SerialPortUnavailableError(
                        "串口已断开，请检查接收机连接") from exc
                for start, raw, ascii_frame in self._reply_frames:
                    matches = (_VERSION_REPLY.fullmatch(raw) if command == b"AT+VERSION?"
                               else raw == expected)
                    if ascii_frame and start >= after and matches:
                        trace["acknowledged"] = True
                        if self.evidence:
                            self.evidence("receiver_command", **trace)
                        return raw
        stage = {b"AT+CFG": "进入配置", b"AT+OPEN ROVER": "开启差分",
                 b"AT+CLOSE ROVER": "关闭差分", b"AT+VERSION?": "配置状态"}.get(command, "控制命令")
        detail = ("卫星报文仍在接收，控制状态尚未确认" if self.control_diagnostics.get("gga_frames")
                  else "未收到完整有效回执")
        if self.evidence:
            self.evidence("receiver_command", **trace, timeout_s=timeout)
        raise SerialPortUnavailableError(f"接收机未确认{stage}（{command.decode('ascii')}）；{detail}")

    def _wait_gga_stream(self, timeout=1., stop_event=None):
        """Require a complete checksummed GGA after EXIT before binary input."""
        deadline = time.monotonic() + timeout
        after = self._reply_received
        while time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                raise SerialPortUnavailableError("接收器恢复已取消")
            if not select.select([self._fd], [], [], .05)[0]:
                continue
            try:
                gga = self._observe_control_chunk(self._read(self._fd, self.read_chunk), after=after)
            except BlockingIOError:
                continue
            if gga:
                return gga
        raise SerialPortUnavailableError("接收机切换后未恢复卫星报文")

    def recover_satellite_stream(self, level, stop_event=None):
        """Recover missing GGA on verified 280R firmware; no saved settings.

        Each level is independently bounded. Never send binary correction data
        during configuration/reboot, and never treat an ACK as stream recovery.
        """
        if level not in (0, 1, 2):
            raise ValueError("Unsupported recovery level")
        from nav_rtk.parser.checksum import calculate_nmea_checksum, validate_nmea_sentence

        def pause(seconds):
            if stop_event is not None:
                if stop_event.wait(seconds):
                    raise SerialPortUnavailableError("接收器恢复已取消")
            else:
                time.sleep(seconds)

        self.control_diagnostics = dict(operation='recover', level=level, commands=[])
        entered, reset_sent = False, False
        try:
            entered = True  # Even an unacknowledged CFG may have changed mode.
            try:
                self.command(b'AT+CFG', b'+CFG', timeout=.5)
            except SerialPortUnavailableError:
                pass
            pause(.1)
            version = self.command(b'AT+VERSION?', b'+VERSION=', timeout=.5)
            if version not in (b'+VERSION=V10277.1.3', b'+VERSION=10277.1.3'):
                raise SerialPortUnavailableError("接收器版本尚未验证自动复位")
            pause(.1)
            if level == 0:
                self.command(b'AT+CLOSE ROVER', b'ROVER CLOSE OK', timeout=.7)
            elif level == 1:
                self.write(b'AT+RESET\r\n')
                reset_sent = True
                # RESET has no reliable ACK. Fresh GGA is the success criterion.
                return self._wait_gga_stream(timeout=6., stop_event=stop_event)
            else:
                self.command(b'AT+RTKUTU=1', b'+RTKUTU=1', timeout=.7)
                pause(.1)
                after = self._reply_received
                self.write(b'$QXMONVER*04\r\n')
                deadline, identity = time.monotonic()+1., None
                while time.monotonic() < deadline and identity is None:
                    pause(0)
                    if not select.select([self._fd], [], [], .05)[0]:
                        continue
                    try:
                        self._observe_control_chunk(self._read(self._fd, self.read_chunk))
                    except BlockingIOError:
                        continue
                    for start, raw, ascii_frame in self._reply_frames:
                        if ascii_frame and start >= after and raw.startswith(b'$QXMONVER,') and validate_nmea_sentence(raw.decode('ascii')):
                            fields = raw.split(b',')
                            if len(fields) == 5 and fields[1].strip() == b'MC280M -02':
                                identity = raw
                if identity is None:
                    raise SerialPortUnavailableError("卫星芯片身份未确认，未执行芯片重启")
                pause(.1)
                body = 'QXCFGRST,1,0'
                command = ('$'+body+'*'+calculate_nmea_checksum(body)).encode('ascii')
                self.command(command, b'$ACK', timeout=.7)
                pause(.3)
        finally:
            if entered and not reset_sent:
                self.write(b'AT+EXIT\r\n')
        pause(.1)
        return self._wait_gga_stream(timeout=3., stop_event=stop_event)

    def _exit_stranded_configuration(self):
        """Recover a config session left by a terminated serial owner.

        WTRTK-280R V10277.1.3 stays silent in configuration mode across tty
        close/open. AT/CFG acknowledgements can be absent even when the command
        executed. VERSION is config-only: a complete reply proves the state.
        Probe once on an entirely silent connection; never enter configuration,
        reset the device or change saved settings to diagnose missing GNSS.
        """
        self._control_buffer = b""
        self.write(b"AT+VERSION?\r\n")
        after = self._reply_received
        deadline = time.monotonic() + _CONFIG_QUERY_TIMEOUT_S
        while time.monotonic() < deadline:
            if not select.select([self._fd], [], [], .05)[0]:
                continue
            try:
                chunk = self._read(self._fd, self.read_chunk)
            except BlockingIOError:
                continue
            if not chunk:
                raise SerialPortUnavailableError("串口已断开，请检查接收机连接")
            gga = self._observe_control_chunk(chunk)
            version = next((_VERSION_REPLY.fullmatch(raw) for start, raw, ascii_frame
                            in self._reply_frames if ascii_frame and start >= after
                            and _VERSION_REPLY.fullmatch(raw)), None)
            if version is not None:
                self.write(b"AT+EXIT\r\n")
                time.sleep(.1)
                self._wait_gga_stream()
                if self.evidence:
                    self.evidence("receiver_config_recovered", version=version.group(1).decode("ascii"))
                return
            if gga:
                return  # Live GGA alone, including quality 0, requires no recovery.

    def rover(self, enabled):
        self._control_buffer = b""
        self.control_diagnostics = dict(operation="open" if enabled else "close",
                                        received_bytes=0, gga_frames=0, commands=[])
        for attempt in range(2):
            confirmed = False
            self.control_diagnostics["stream_restored"] = False
            # V10277.1.3 sometimes suppresses +CFG while still entering config.
            # A complete config-only VERSION reply confirms entry in that case.
            try:
                try:
                    self.command(b"AT+CFG", b"+CFG", timeout=.5)
                except SerialPortUnavailableError:
                    time.sleep(.1)
                    self.command(b"AT+VERSION?", b"+VERSION=", timeout=_CONFIG_QUERY_TIMEOUT_S)
                time.sleep(.1)  # Allow CFG to settle before the next AT command.
                self.command(b"AT+OPEN ROVER" if enabled else b"AT+CLOSE ROVER",
                             b"ROVER OPEN OK" if enabled else b"ROVER CLOSE OK")
                confirmed = True
            except SerialPortUnavailableError:
                if attempt:
                    raise
            finally:
                # Retry the complete transaction only after EXIT and normal GGA.
                # Entry failures need the same recovery as missing OPEN/CLOSE ACKs.
                time.sleep(.1)  # OPEN/CLOSE acknowledgement does not mean EXIT is ready.
                self.write(b"AT+EXIT\r\n")
                # V10277.1.3 can stop UART output when RTCM immediately follows EXIT.
                # Outdoor A/B/A/B tests recovered continuous output with this guard.
                # Keep the port owned here; the first correction write must wait.
                time.sleep(.1)
                try:
                    resumed_gga = self._wait_gga_stream()
                    self.control_diagnostics["stream_restored"] = True
                except SerialPortUnavailableError:
                    self.control_diagnostics["stream_restored"] = False
                    if enabled and confirmed:
                        self.rover(False)  # Restore normal reception before failing closed.
                    raise
            if confirmed:
                return resumed_gga
