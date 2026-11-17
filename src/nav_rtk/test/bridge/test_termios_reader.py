"""串口独占（TIOCEXCL）的回归测试。

为什么要独占：两个进程同时读同一个 tty **不会报任何错**，而是各读走一半字节，
两边的报文都被撕碎。实机实测（`/dev/ttyTHS1` @ 230400，WTRTK 接收机）：

    单读者      parseErrors =    0 / 509 帧   （0%    ）
    两读者并存  parseErrors = 2780 / 4451 帧  （62.4% ）

62% 那种状态下界面显示的就是「收不到数据」——经纬度 `--`、卫星 0 颗、
诊断日志里全是 `checksum 校验失败`，而没有任何一处提示「端口被占着」。
用户能想到的原因全是外在的（接收机坏了 / 波特率不对 / 没信号），
很难猜到是上一次运行留下的孤儿节点还在读同一个口。

TIOCEXCL 把这种**静默的数据损坏**变成一条明确的 EBUSY。
"""

import errno
import os
import pty
import select
import threading
import time

import pytest

from nav_rtk.bridge.termios_reader import TermiosLineReader
from nav_rtk.parser.serial_client import SerialPortUnavailableError


@pytest.fixture()
def pty_port():
    """一对 PTY，返回从端路径（`TermiosLineReader` 只需要是个 tty）。"""
    master, slave = pty.openpty()
    try:
        yield os.ttyname(slave)
    finally:
        os.close(slave)
        os.close(master)


def test_a_second_reader_is_refused_instead_of_silently_sharing(pty_port):
    first = TermiosLineReader(pty_port, 230400)
    first.open()
    try:
        second = TermiosLineReader(pty_port, 230400)
        with pytest.raises(SerialPortUnavailableError) as excinfo:
            second.open()
        # 报错要能直接指向「有残留进程」，而不是一句 errno
        assert "独占" in str(excinfo.value)
    finally:
        first.close()


def test_close_really_releases_the_descriptor(pty_port):
    """`close()` 必须真的把 fd 关掉——独占锁随 fd 走，漏关就等于锁死端口。

    「持有者进程退出后端口自动可用」是内核行为（最后一个持有者消失时清掉
    TTY_EXCLUSIVE），**在 PTY 上测不出来**：PTY 的主端还握在本进程手里，
    从端就不算完全释放，标志位不会清。那一条用真机 `/dev/ttyTHS1` 两进程
    实测过：持有者被 `kill` 掉、不做任何清理，下一个进程仍能正常打开。
    这里只守住我们自己该负责的部分。
    """
    first = TermiosLineReader(pty_port, 230400)
    first.open()
    fd = first._fd
    assert fd is not None
    first.close()
    assert first._fd is None
    with pytest.raises(OSError):
        os.fstat(fd)          # fd 确实已经不存在了
    first.close()             # 幂等，重复关闭不能再抛


def test_unsupported_baudrate_still_reports_itself(pty_port):
    """波特率校验要排在打开端口之前，别让独占检查把它盖过去。"""
    reader = TermiosLineReader(pty_port, 12345)
    with pytest.raises(ValueError) as excinfo:
        reader.open()
    assert "不支持的波特率" in str(excinfo.value)
    assert errno.EBUSY  # 只是提醒这个模块确实用到了 errno 常量


@pytest.mark.parametrize("paused", [False, True])
def test_hung_up_serial_leaves_reader_instead_of_spinning(paused):
    master, slave = pty.openpty()
    reader = TermiosLineReader(os.ttyname(slave), 230400)
    reader.open()
    stop = threading.Event()
    pause = threading.Event()
    if paused:
        pause.set()
    result = []

    def consume():
        try:
            list(reader.iter_frames(stop_event=stop, pause_event=pause))
        except SerialPortUnavailableError:
            result.append("disconnected")
        else:
            result.append("ended")

    os.close(master)
    worker = threading.Thread(target=consume, daemon=True)
    worker.start()
    worker.join(.5)
    completed_without_stop = not worker.is_alive()
    stop.set()
    worker.join(1)
    reader.close()
    os.close(slave)
    assert completed_without_stop, "unplugged tty kept spinning until externally stopped"
    assert result == ["disconnected"]


@pytest.mark.parametrize("initial", [
    "config", "config_lf", "config_lf_bytewise", "config_gga_version",
    "streaming", "unresponsive", "binary", "incomplete_lf",
])
def test_reconnect_only_exits_a_confirmed_stranded_configuration(initial, monkeypatch):
    """Firmware config state survives host close/reopen; quality 0 is still live."""
    import nav_rtk.bridge.termios_reader as module
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    monkeypatch.setattr(module, "_STARTUP_SILENCE_S", .08, raising=False)
    monkeypatch.setattr(module, "_CONFIG_QUERY_TIMEOUT_S", .12, raising=False)
    body = "GNGGA,120000.00,,,,,0,00,99.9,,,,,,"
    gga = ("$" + body + "*" + calculate_nmea_checksum(body) + "\r\n").encode()
    master, slave = pty.openpty()
    reader = TermiosLineReader(
        os.ttyname(slave), 230400, writable=True,
        read_chunk=1 if initial == "config_lf_bytewise" else 4096,
    )
    reader.open()
    done = threading.Event()
    recovered = threading.Event()
    commands, frames, failures = [], [], []
    reader.on_control_frame = frames.append

    def receiver():
        streaming = initial == "streaming"
        buffer = b""
        while not done.wait(.01):
            if streaming:
                os.write(master, gga)
            if not select.select([master], [], [], .005)[0]:
                continue
            buffer += os.read(master, 4096)
            while b"\r\n" in buffer:
                command, buffer = buffer.split(b"\r\n", 1)
                commands.append(command)
                if command == b"AT+VERSION?":
                    if initial == "config":
                        os.write(master, b"+VERSION=V10277.1.3\r\n")
                    elif initial in ("config_lf", "config_lf_bytewise"):
                        # Actual module 3 reply, identity-1733/rx.bin at 16106:
                        # a standalone LF precedes the complete version line.
                        os.write(master, b"\n+VERSION=V10277.1.3\r\n")
                    elif initial == "config_gga_version":
                        os.write(master, gga + b"+VERSION=V10277.1.3\r\n")
                    elif initial == "incomplete_lf":
                        os.write(master, b"\n+VERSION=V10277.1.3\r")
                    elif initial == "binary":
                        os.write(master, b"\x80noise+VERSION=V10277.1.3\r\n")
                elif command == b"AT+EXIT" and initial.startswith("config"):
                    streaming = True
                    recovered.set()

    def consume():
        try:
            for frame in reader.iter_frames(stop_event=done):
                frames.append(frame)
        except Exception as exc:
            failures.append(exc)

    workers = [threading.Thread(target=f, daemon=True) for f in (receiver, consume)]
    for worker in workers:
        worker.start()
    try:
        time.sleep(.65)
    finally:
        done.set()
        for worker in workers:
            worker.join(2)
        reader.close()
        os.close(master)
        os.close(slave)
    assert not failures
    assert all(not worker.is_alive() for worker in workers)
    if initial.startswith("config"):
        assert recovered.is_set(), "reopening tty left the receiver stranded in config"
        assert commands == [b"AT+VERSION?", b"AT+EXIT"]
        assert any(frame.raw_bytes.startswith(b"$GNGGA,") for frame in frames)
    elif initial == "streaming":
        assert commands == [], "quality-0 telemetry must not trigger receiver commands"
        assert frames
    else:
        assert commands == [b"AT+VERSION?"], "unconfirmed device must not be reset or reconfigured"


def _wtraw_with(payload):
    return b"$WTRAW," + payload.ljust(200, b"\x00") + b"*00\r\n"


@pytest.mark.parametrize("reply", [
    b"OK",
    b"OK\r",
    b"NOT OK\r\n",
    b"$OTHER,OK\r\n",
    b"\xffOK\r\n",
    _wtraw_with(b"\r\nOK\r\n"),
])
def test_command_rejects_incomplete_substring_and_binary_ack(reply):
    master, slave = pty.openpty()
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()
    worker = threading.Thread(
        target=lambda: (os.read(master, 4096), os.write(master, reply)), daemon=True)
    worker.start()
    try:
        with pytest.raises(SerialPortUnavailableError):
            reader.command(b"AT", b"OK", timeout=.12)
        assert reader.control_diagnostics["commands"][-1]["acknowledged"] is False
    finally:
        worker.join(1)
        reader.close()
        os.close(master)
        os.close(slave)


@pytest.mark.parametrize("reply", [
    b"OK\r\n",
    b"\nOK\r\n",
    _wtraw_with(b"\r\nNOT OK\r\n") + b"\nOK\r\n",
])
def test_command_accepts_complete_reply_at_window_start_and_fragmented(reply):
    master, slave = pty.openpty()
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True, read_chunk=1)
    reader.open()
    worker = threading.Thread(
        target=lambda: (os.read(master, 4096), os.write(master, reply)), daemon=True)
    worker.start()
    try:
        reader.command(b"AT", b"OK", timeout=.4)
        assert reader.control_diagnostics["commands"][-1]["acknowledged"] is True
    finally:
        worker.join(1)
        reader.close()
        os.close(master)
        os.close(slave)


def test_binary_version_does_not_trigger_configuration_exit(monkeypatch):
    import nav_rtk.bridge.termios_reader as module
    monkeypatch.setattr(module, "_CONFIG_QUERY_TIMEOUT_S", .12)
    master, slave = pty.openpty()
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()
    sent = []

    def receiver():
        sent.append(os.read(master, 4096))
        os.write(master, _wtraw_with(b"\r\n+VERSION=V10277.1.3\r\n"))
        if select.select([master], [], [], .25)[0]:
            sent.append(os.read(master, 4096))

    worker = threading.Thread(target=receiver, daemon=True)
    worker.start()
    try:
        reader._exit_stranded_configuration()
    finally:
        worker.join(2)
        reader.close()
        os.close(master)
        os.close(slave)
    assert sent == [b"AT+VERSION?\r\n"]


@pytest.mark.parametrize("prefix,reply", [
    (b"O", b"K\r\n"),
    (_wtraw_with(b"\r\nOK\r\n")[:8], _wtraw_with(b"\r\nOK\r\n")[8:]),
])
def test_command_does_not_forget_partial_frames_from_normal_reads(prefix, reply):
    master, slave = pty.openpty()
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()
    os.write(master, prefix)
    assert select.select([reader._fd], [], [], .2)[0]
    assert reader._read(reader._fd, 4096) == prefix
    worker = threading.Thread(
        target=lambda: (os.read(master, 4096), os.write(master, reply)), daemon=True)
    worker.start()
    try:
        with pytest.raises(SerialPortUnavailableError):
            reader.command(b"AT", b"OK", timeout=.12)
    finally:
        worker.join(1)
        reader.close()
        os.close(master)
        os.close(slave)


@pytest.mark.parametrize("reply", [b"+VERSION=\r\n", b"+VERSION=V1", b"+VERSION=V1 \r\n"])
def test_config_confirmation_requires_complete_nonempty_version(reply):
    master, slave = pty.openpty()
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()
    worker = threading.Thread(
        target=lambda: (os.read(master, 4096), os.write(master, reply)), daemon=True)
    worker.start()
    try:
        with pytest.raises(SerialPortUnavailableError, match="配置状态"):
            reader.command(b"AT+VERSION?", b"+VERSION=", timeout=.12)
    finally:
        worker.join(1)
        reader.close()
        os.close(master)
        os.close(slave)


@pytest.mark.parametrize("body", [
    "GNGGA,120000.00",
    "GNGGA,120000.00,,,,,bad,00,99.9,,,,,,",
    "GNGGA,120000.00,,,,,,00,99.9,,,,,,",
])
def test_restored_stream_requires_complete_parseable_gga(body):
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    master, slave = pty.openpty()
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()
    os.write(master, ("$" + body + "*" + calculate_nmea_checksum(body) + "\r\n").encode())
    try:
        with pytest.raises(SerialPortUnavailableError, match="未恢复卫星报文"):
            reader._wait_gga_stream(timeout=.12)
    finally:
        reader.close()
        os.close(master)
        os.close(slave)


def test_control_hangup_reports_disconnect_without_waiting_for_ack():
    master, slave = pty.openpty()
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()

    def receiver():
        os.read(master, 4096)
        os.close(master)

    worker = threading.Thread(target=receiver, daemon=True)
    worker.start()
    try:
        started = time.monotonic()
        with pytest.raises(SerialPortUnavailableError, match="断开"):
            reader.command(b"AT", b"OK", timeout=2)
        assert time.monotonic() - started < .5
    finally:
        worker.join(1)
        reader.close()
        os.close(slave)


def test_close_ack_requires_stream_to_resume():
    master, slave = pty.openpty()
    reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
    reader.open()
    commands = []
    done = threading.Event()

    def receiver():
        buffer = b""
        while not done.is_set():
            if not select.select([master], [], [], .02)[0]:
                continue
            buffer += os.read(master, 4096)
            while b"\r\n" in buffer:
                command, buffer = buffer.split(b"\r\n", 1)
                commands.append(command)
                reply = {b"AT+CFG": b"+CFG\r\n",
                         b"AT+CLOSE ROVER": b"ROVER CLOSE OK\r\n"}.get(command)
                if reply:
                    os.write(master, reply)

    worker = threading.Thread(target=receiver, daemon=True)
    worker.start()
    try:
        with pytest.raises(SerialPortUnavailableError, match="未恢复卫星报文"):
            reader.rover(False)
        assert commands == [b"AT+CFG", b"AT+CLOSE ROVER", b"AT+EXIT"]
        assert reader.control_diagnostics["stream_restored"] is False
    finally:
        done.set()
        worker.join(1)
        reader.close()
        os.close(master)
        os.close(slave)


@pytest.fixture()
def rover_exchange():
    """Real PTY model for the module-2 entry failure and bounded recovery."""
    from nav_rtk.parser.checksum import calculate_nmea_checksum
    body = "GNGGA,120000.00,,,,,0,00,99.9,,,,,,"
    gga = ("$" + body + "*" + calculate_nmea_checksum(body) + "\r\n").encode()
    resources = []

    def start(ignored_entries=0, ignored_actions=0, restore=True):
        master, slave = pty.openpty()
        reader = TermiosLineReader(os.ttyname(slave), 230400, writable=True)
        reader.open()
        stop, commands, frames = threading.Event(), [], []
        reader.on_control_gga = frames.append

        def receiver():
            buffer = b""
            streaming, configured = restore, False
            entries, actions = 0, 0
            while not stop.wait(.01):
                if streaming:
                    os.write(master, gga)
                if not select.select([master], [], [], .005)[0]:
                    continue
                buffer += os.read(master, 4096)
                while b"\r\n" in buffer:
                    command, buffer = buffer.split(b"\r\n", 1)
                    commands.append((command, len(frames)))
                    if command == b"AT+CFG":
                        entries += 1
                        if entries > ignored_entries:
                            configured, streaming = True, False
                            os.write(master, b"+CFG\r\n")
                    elif command == b"AT+VERSION?" and configured:
                        os.write(master, b"\n+VERSION=V10277.1.3\r\n")
                    elif command in (b"AT+OPEN ROVER", b"AT+CLOSE ROVER") and configured:
                        actions += 1
                        if actions > ignored_actions:
                            os.write(master, (b"ROVER OPEN OK\r\n" if b"OPEN" in command
                                              else b"ROVER CLOSE OK\r\n"))
                    elif command == b"AT+EXIT":
                        configured, streaming = False, restore

        worker = threading.Thread(target=receiver, daemon=True)
        resources.append((reader, master, slave, stop, worker))
        worker.start()
        return reader, commands

    yield start
    for reader, master, slave, stop, worker in resources:
        stop.set()
        worker.join(1)
        reader.close()
        os.close(master)
        os.close(slave)


@pytest.mark.parametrize("enabled", [False, True])
def test_ignored_first_cfg_recovers_stream_before_second_transaction(rover_exchange, enabled):
    reader, seen = rover_exchange(ignored_entries=1)
    assert reader.rover(enabled).startswith("$GNGGA,")
    operation = b"AT+OPEN ROVER" if enabled else b"AT+CLOSE ROVER"
    assert [c for c, _ in seen] == [
        b"AT+CFG", b"AT+VERSION?", b"AT+EXIT",
        b"AT+CFG", operation, b"AT+EXIT",
    ]
    assert seen[3][1] > seen[2][1], "second CFG was sent before observing restored GGA"
    assert reader.control_diagnostics["stream_restored"] is True


def test_permanent_entry_failure_has_only_two_complete_transactions(rover_exchange):
    reader, seen = rover_exchange(ignored_entries=100)
    with pytest.raises(SerialPortUnavailableError, match="配置状态"):
        reader.rover(True)
    assert [c for c, _ in seen] == [b"AT+CFG", b"AT+VERSION?", b"AT+EXIT"] * 2
    assert reader.control_diagnostics["stream_restored"] is True
    assert not any(b"ROVER" in c or c.startswith(b"\xd3") for c, _ in seen)


def test_failed_entry_without_stream_restoration_cannot_retry(rover_exchange):
    reader, seen = rover_exchange(ignored_entries=1, restore=False)
    with pytest.raises(SerialPortUnavailableError):
        reader.rover(True)
    assert [c for c, _ in seen] == [b"AT+CFG", b"AT+VERSION?", b"AT+EXIT"]
    assert reader.control_diagnostics["stream_restored"] is False


def test_failed_action_also_exits_before_retrying_complete_transaction(rover_exchange):
    reader, seen = rover_exchange(ignored_actions=1)
    assert reader.rover(False).startswith("$GNGGA,")
    assert [c for c, _ in seen] == [
        b"AT+CFG", b"AT+CLOSE ROVER", b"AT+EXIT",
        b"AT+CFG", b"AT+CLOSE ROVER", b"AT+EXIT",
    ]
    assert seen[3][1] > seen[2][1]
