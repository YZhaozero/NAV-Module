"""Linux RTK 串口解析命令行入口。"""

from __future__ import annotations

import argparse
import datetime as dt
import sys
from dataclasses import dataclass
from pathlib import Path

from nav_rtk.parser.serial_client import SerialPortUnavailableError
from nav_rtk.parser.wtrtk_runtime import (
    SerialRuntimeConfig,
    SentenceUpdate,
    WarningUpdate,
    WTRTKFrameEvent,
    build_wtrtk_summary,
    format_state_lines,
    log_checksum_warning,
    run_serial_monitor,
    write_wtrtk_log,
)

__all__ = [
    "DefaultConfig",
    "build_argument_parser",
    "build_wtrtk_summary",
    "log_checksum_warning",
    "print_state",
    "resolve_config",
    "write_wtrtk_log",
]

class Tee:
    def __init__(self, *streams: object) -> None:
        self.streams = streams

    def write(self, text: str) -> int:
        written = 0
        for stream in self.streams:
            written = stream.write(text)
        return written

    def flush(self) -> None:
        for stream in self.streams:
            stream.flush()


@dataclass(frozen=True)
class DefaultConfig:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 230400
    timeout: float = 1.0
    output_json: bool = False
    output_hex: bool = False


DEFAULT_CONFIG = DefaultConfig()


def build_argument_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器。"""

    parser = argparse.ArgumentParser(description="WitRTK Linux 串口解析工具")
    parser.add_argument(
        "--port",
        default=argparse.SUPPRESS,
        help=f"Linux 串口设备，例如 /dev/ttyUSB0 (默认: {DEFAULT_CONFIG.port})",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=argparse.SUPPRESS,
        help=f"串口波特率 (默认: {DEFAULT_CONFIG.baudrate})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=argparse.SUPPRESS,
        help=f"串口读取超时时间 (默认: {DEFAULT_CONFIG.timeout})",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        default=argparse.SUPPRESS,
        help=f"是否输出 JSON 状态 (默认: {DEFAULT_CONFIG.output_json})",
    )
    parser.add_argument(
        "--hex",
        action="store_true",
        default=argparse.SUPPRESS,
        help=f"是否按十六进制显示串口原始字节 (默认: {DEFAULT_CONFIG.output_hex})",
    )
    return parser


def resolve_config(namespace: argparse.Namespace) -> tuple[DefaultConfig, set[str]]:
    provided = set(vars(namespace).keys())
    config = DefaultConfig(
        port=getattr(namespace, "port", DEFAULT_CONFIG.port),
        baudrate=getattr(namespace, "baudrate", DEFAULT_CONFIG.baudrate),
        timeout=getattr(namespace, "timeout", DEFAULT_CONFIG.timeout),
        output_json=getattr(namespace, "json", DEFAULT_CONFIG.output_json),
        output_hex=getattr(namespace, "hex", DEFAULT_CONFIG.output_hex),
    )
    used_defaults = {field for field in DEFAULT_CONFIG.__dataclass_fields__ if field not in provided}
    return config, used_defaults


def print_config(defaults: DefaultConfig, config: DefaultConfig, used_defaults: set[str]) -> None:
    print("默认配置：")
    print(f"  port={defaults.port}")
    print(f"  baudrate={defaults.baudrate}")
    print(f"  timeout={defaults.timeout}")
    print(f"  json={defaults.output_json}")
    print(f"  hex={defaults.output_hex}")
    print("本次启动配置：")
    print(f"  port={config.port}")
    print(f"  baudrate={config.baudrate}")
    print(f"  timeout={config.timeout}")
    print(f"  json={config.output_json}")
    print(f"  hex={config.output_hex}")
    if len(used_defaults) == len(DEFAULT_CONFIG.__dataclass_fields__):
        print("提示：当前使用全默认配置启动，请确认这些默认配置是否符合你的设备。")
    elif used_defaults:
        fields = ",".join(sorted(used_defaults))
        print(f"提示：本次启动包含默认配置项({fields})，请确认是否正确，必要时通过参数覆盖。")


def print_error_block(message: str) -> None:
    """输出醒目的错误块提示。"""

    print("================ 串口状态 ================")
    print(f"ERROR: {message}")
    print("========================================")


def print_state(update: SentenceUpdate, output_json: bool) -> None:
    """输出原始报文与可选状态信息。"""

    for line in format_state_lines(
        raw_line=update.display_line,
        state_json=update.state_json,
        output_json=output_json,
        sentence_timestamp=update.sentence_timestamp,
        receive_system_time=update.receive_system_time,
    ):
        print(line)


def print_wtrtk_summary(summary: str) -> None:
    """将 WTRTK 摘要打印到终端。"""

    print(summary)


def main() -> int:
    """启动串口监听并持续解析 NMEA 报文。"""

    args = build_argument_parser().parse_args()
    config, used_defaults = resolve_config(args)

    log_dir = Path(__file__).resolve().parent.parent / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    wtrtk_log_dir = Path(__file__).resolve().parent.parent / "log_WTRTK"
    wtrtk_log_dir.mkdir(parents=True, exist_ok=True)
    log_filename = f"witrtk_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    log_path = log_dir / log_filename
    wtrtk_log_path = wtrtk_log_dir / log_filename
    log_file = log_path.open("w", encoding="utf-8")
    wtrtk_log_file = wtrtk_log_path.open("w", encoding="utf-8")
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    sys.stdout = Tee(original_stdout, log_file)
    sys.stderr = Tee(original_stderr, log_file)

    warning_count = 0
    warning_suppressed_notice_printed = False

    print(f"日志：{log_path}")
    print(f"WTRTK日志：{wtrtk_log_path}")
    print_config(DEFAULT_CONFIG, config, used_defaults)
    try:
        def handle_sentence(update: SentenceUpdate) -> None:
            print_state(update, config.output_json)

        def handle_warning(update: WarningUpdate) -> None:
            nonlocal warning_count, warning_suppressed_notice_printed
            warning_count += 1
            if update.kind == "checksum":
                log_checksum_warning(log_file, update.display_line)
            else:
                log_file.write(f"警告：{update.message}\n")
                log_file.flush()
            if not warning_suppressed_notice_printed:
                print("提示：checksum 校验失败等警告已写入日志文件，终端不再逐条输出。")
                warning_suppressed_notice_printed = True

        def handle_wtrtk(event: WTRTKFrameEvent) -> None:
            print_wtrtk_summary(event.summary)
            write_wtrtk_log(
                wtrtk_log_file,
                event.time_text,
                event.sentence_timestamp,
                event.display_line,
                event.summary,
            )

        run_serial_monitor(
            SerialRuntimeConfig(
                port=config.port,
                baudrate=config.baudrate,
                timeout=config.timeout,
                output_json=config.output_json,
                output_hex=config.output_hex,
            ),
            on_sentence=handle_sentence,
            on_warning=handle_warning,
            on_wtrtk=handle_wtrtk,
        )
    except SerialPortUnavailableError:
        print_error_block("端口不存在")
        return 1
    except KeyboardInterrupt:
        print("已收到退出指令。")
        if warning_count:
            print(f"本次运行累计警告数：{warning_count} (详见日志文件)")
        return 0
    finally:
        sys.stdout = original_stdout
        sys.stderr = original_stderr
        log_file.close()
        wtrtk_log_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
