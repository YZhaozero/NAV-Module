"""串口读取封装。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Generator, Optional


@dataclass(frozen=True)
class SerialFrame:
    """一帧串口数据，同时保留文本和原始字节。"""

    text: str
    raw_bytes: bytes


class SerialPortUnavailableError(RuntimeError):
    """串口设备不存在或未插入。"""


class SerialLineReader:
    """按行读取串口输出的简单封装。"""

    def __init__(self, port: str, baudrate: int, timeout: float = 1.0) -> None:
        """初始化串口参数，但不立即打开串口。"""

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[object] = None

    def open(self) -> None:
        """打开串口连接。"""

        try:
            import serial
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("缺少 pyserial 依赖，请先执行 pip install -r requirements.txt") from exc

        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.SerialException as exc:
            if getattr(exc, "errno", None) == 2 or "No such file or directory" in str(exc):
                raise SerialPortUnavailableError("端口不存在") from exc
            raise

    def close(self) -> None:
        """关闭串口连接。"""

        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def iter_frames(self, stop_event: Optional[object] = None, pause_event: Optional[object] = None) -> Generator[SerialFrame, None, None]:
        """持续读取串口并逐行返回文本与原始字节。"""

        # 如果串口未初始化，先打开串口连接
        if self._serial is None:
            self.open()
        # 断言确保串口实例已正确创建，后续可安全访问
        assert self._serial is not None
        # 无限循环持续读取串口数据
        while True:
            if stop_event is not None and getattr(stop_event, "is_set")():
                return
            if pause_event is not None and getattr(pause_event, "is_set")():
                time.sleep(0.05)
                continue
            # 读取一行串口数据，遇到换行符或超时返回
            raw_line = self._serial.readline()
            # 如果读取到空数据（超时场景），跳过本次循环继续读取
            if not raw_line:
                continue
            # 去除原始行数据首尾的空白字符（换行、空格等）
            stripped_bytes = raw_line.strip()
            # 如果去除空白后得到空字节，跳过空行
            if not stripped_bytes:
                continue
            # 将字节流解码为UTF-8字符串，忽略无法解码的非法字符，再次去除首尾空白
            line = stripped_bytes.decode("utf-8", errors="ignore").strip()
            # 确保解码后的字符串非空，才生成帧对象返回
            if line:
                yield SerialFrame(text=line, raw_bytes=stripped_bytes)

    def iter_lines(self) -> Generator[str, None, None]:
        """兼容旧接口，仅返回逐行 UTF-8 文本。"""

        for frame in self.iter_frames():
            yield frame.text
