"""WTRTK 运行时公共逻辑。

这个模块的目标是把“串口读取 -> 解析 -> 生成 WTRTK 结构化事件”这条核心链路
从 `main.py` 和 `server.py` 里抽出来，避免两边各写一套。

你可以把它理解成“中间层”：
- 上游：串口原始字节 / NMEA 文本
- 下游：CLI 打印、日志写入、Web 实时推送

这个模块本身不直接决定“怎么显示界面”或“怎么提供 HTTP 接口”，
它只负责把底层数据整理成上层容易消费的统一事件。

大致数据流：
1. `SerialLineReader` 从串口持续读出一帧
2. 判断这帧是普通 NMEA、WTRAW 还是无效内容
3. 普通 NMEA 走 `parse_nmea_sentence()`
4. `RTKStateStore` 更新聚合状态
5. 如果当前帧是 WTRTK，则额外构造 `WTRTKFrameEvent`
6. 通过回调把结果交给 CLI 或 Web 层
"""

from __future__ import annotations

import datetime as dt
import json
from dataclasses import dataclass
from typing import Callable, Optional

from .models import ParsedSentence, WTRTKData
from .nmea_parser import parse_nmea_sentence
from .serial_client import SerialLineReader
from .state_store import RTKStateStore


@dataclass(frozen=True)
class SerialRuntimeConfig:
    """串口运行时配置。

    字段说明：
    - `port`: Linux 串口设备路径，例如 `/dev/ttyUSB0`
    - `baudrate`: 串口波特率，例如 `230400`
    - `timeout`: 单次读取超时时间，单位秒
    - `output_json`: 是否在文本输出中附带状态 JSON
    - `output_hex`: 是否把串口原始字节显示成十六进制
    """

    port: str
    baudrate: int
    timeout: float = 1.0
    output_json: bool = False
    output_hex: bool = False


@dataclass(frozen=True)
class SentenceUpdate:
    """一次普通报文输出更新。

    字段说明：
    - `display_line`: 当前帧最终要显示的原始内容，可能是文本，也可能是 HEX
    - `state_json`: 聚合后的状态 JSON；如果当前帧不需要输出状态则为 `None`
    - `sentence_timestamp`: 报文本身自带的时间，例如 RMC/GGA 的 UTC 时间
    - `receive_system_time`: 系统接收到该帧时记录的时间，主要给 WTRTK 用
    - `parsed_sentence_id`: 当前帧解析出的句型 ID，例如 `RMC`、`WTRTK`
    """

    display_line: str
    state_json: Optional[str]
    sentence_timestamp: Optional[str]
    receive_system_time: Optional[str]
    parsed_sentence_id: Optional[str]


@dataclass(frozen=True)
class WarningUpdate:
    """一次警告更新。

    字段说明：
    - `message`: 原始错误信息
    - `display_line`: 触发警告的原始串口内容
    - `kind`: 警告类型，目前主要是 `checksum` 或 `parse`
    """

    message: str
    display_line: str
    kind: str


@dataclass(frozen=True)
class WTRTKFrameEvent:
    """已解析完成的 WTRTK 帧事件。

    这个对象是“公共 emitter”向 CLI 或 Web 层发送的核心载体。
    可以把它理解成：一条 WTRTK 报文在完成解析和中文化之后的最终结构。
    """

    index: int
    time_text: str
    sentence_timestamp: Optional[str]
    raw_sentence: str
    display_line: str
    summary: str
    positioning_text: str
    rtk_mode_text: str
    alignment_text: str
    latitude: Optional[float]
    longitude: Optional[float]
    altitude: Optional[float]
    speed_kmh: Optional[float]
    heading_deg: Optional[float]
    gps_heading: Optional[float]
    diff_x: Optional[float]
    diff_y: Optional[float]
    diff_z: Optional[float]
    diff_r: Optional[float]
    angle_x: Optional[float]
    angle_y: Optional[float]
    angle_z: Optional[float]
    mag_x: Optional[float]
    mag_y: Optional[float]
    mag_z: Optional[float]
    temperature_c: Optional[float]
    base_distance_m: Optional[float]

    @classmethod
    def from_data(
        cls,
        index: int,
        receive_system_time: str,
        sentence_timestamp: Optional[str],
        raw_sentence: str,
        display_line: str,
        data: WTRTKData,
    ) -> "WTRTKFrameEvent":
        """从 `WTRTKData` 构造一个可直接消费的事件对象。

        参数说明：
        - `index`: 当前运行时里的 WTRTK 帧序号，从 0 开始递增
        - `receive_system_time`: 这一帧被系统收到时记录的时间
        - `sentence_timestamp`: 报文内部时间；WTRTK 一般没有，所以多数时候是 `None`
        - `raw_sentence`: 解析后的原始 WTRTK 文本，例如 `$WTRTK,...*64`
        - `display_line`: 当前输出层看到的内容；若开启 `HEX`，这里可能不是纯文本报文
        - `data`: 已完成字段级解析的 `WTRTKData`
        """
        summary = build_wtrtk_summary(data)
        return cls(
            index=index,
            time_text=receive_system_time,
            sentence_timestamp=sentence_timestamp,
            raw_sentence=raw_sentence,
            display_line=display_line,
            summary=summary,
            positioning_text=describe_positioning_status(data.positioning_status),
            rtk_mode_text=describe_rtk_mode(data.rtk_mode),
            alignment_text=describe_alignment_flag(data.alignment_flag),
            latitude=data.latitude,
            longitude=data.longitude,
            altitude=data.altitude,
            speed_kmh=data.speed_kmh,
            heading_deg=data.heading_deg,
            gps_heading=data.gps_heading,
            diff_x=data.diff_x,
            diff_y=data.diff_y,
            diff_z=data.diff_z,
            diff_r=data.diff_r,
            angle_x=data.angle_x,
            angle_y=data.angle_y,
            angle_z=data.angle_z,
            mag_x=data.mag_x,
            mag_y=data.mag_y,
            mag_z=data.mag_z,
            temperature_c=data.temperature_c,
            base_distance_m=data.base_distance_m,
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "index": self.index,
            "timeText": self.time_text,
            "sentenceTimestampText": self.sentence_timestamp,
            "rawSentence": self.raw_sentence,
            "displayLine": self.display_line,
            "summary": self.summary,
            "positioningText": self.positioning_text,
            "rtkModeText": self.rtk_mode_text,
            "alignmentText": self.alignment_text,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "speedKmh": self.speed_kmh,
            "headingDeg": self.heading_deg,
            "gpsHeading": self.gps_heading,
            "diffX": self.diff_x,
            "diffY": self.diff_y,
            "diffZ": self.diff_z,
            "diffR": self.diff_r,
            "angleX": self.angle_x,
            "angleY": self.angle_y,
            "angleZ": self.angle_z,
            "magX": self.mag_x,
            "magY": self.mag_y,
            "magZ": self.mag_z,
            "temperatureC": self.temperature_c,
            "baseDistanceM": self.base_distance_m,
        }


def format_utc_time(value: Optional[str]) -> Optional[str]:
    """将 NMEA UTC 时间格式化为更易读的 HH:MM:SS(.sss)。

    例如：
    - 输入：`070225.100`
    - 输出：`07:02:25.100`

    注意：这里只做字符串格式整理，不做时区转换。
    """

    if not value or len(value) < 6:
        return None
    hour = value[0:2]
    minute = value[2:4]
    second = value[4:]
    return f"{hour}:{minute}:{second}"


def format_system_time(value: Optional[dt.datetime] = None) -> str:
    """将系统时间格式化为 HH:MM:SS.mmm。

    这个时间主要用于“系统真正收到串口数据的时刻”，
    和报文内部自带时间不是一个概念。
    """

    current = value or dt.datetime.now()
    return current.strftime("%H:%M:%S.%f")[:-3]


def get_sentence_timestamp(parsed: ParsedSentence) -> Optional[str]:
    """仅使用当前报文自带时间。

    当前实现里只有带 `utc_time` 字段的句型会返回时间，
    例如 `RMC`、`GGA`。`WTRTK` 一般没有这个字段，所以通常返回 `None`。
    """

    current_utc_time = getattr(parsed.data, "utc_time", None)
    return format_utc_time(current_utc_time)


def format_serial_output(raw_bytes: bytes, text: str, output_hex: bool) -> str:
    """根据配置格式化串口输出。

    参数说明：
    - `raw_bytes`: 串口原始字节
    - `text`: UTF-8 解码后的文本
    - `output_hex`: 是否启用 HEX 输出

    返回值：
    - `output_hex=False` 时返回文本
    - `output_hex=True` 时返回十六进制字符串
    """

    if output_hex:
        return raw_bytes.hex(" ").upper()
    return text


def format_state_lines(
    raw_line: str,
    state_json: Optional[str],
    output_json: bool,
    sentence_timestamp: Optional[str] = None,
    receive_system_time: Optional[str] = None,
) -> list[str]:
    """构建终端/日志输出行。

    参数说明：
    - `raw_line`: 最终要展示的原始输入内容
    - `state_json`: 当前聚合状态的 JSON 文本
    - `output_json`: 是否把 `state_json` 也输出出来
    - `sentence_timestamp`: 报文自带时间；没有就传 `None`
    - `receive_system_time`: 系统接收时间；没有就传 `None`
    """

    lines: list[str] = []
    if receive_system_time is not None:
        lines.append(f"接收系统时间：{receive_system_time}")
    if sentence_timestamp is not None:
        lines.append(f"时间：{sentence_timestamp}")
    lines.append(f"收：{raw_line}")
    if output_json and state_json is not None:
        lines.append(f"状态：{state_json}")
    return lines


def describe_positioning_status(value: Optional[int]) -> str:
    """把 WTRTK 的定位状态码转成中文说明。"""

    mapping = {
        0: "初始化",
        1: "单点定位",
        2: "码差分",
        4: "固定解",
        5: "浮点解",
    }
    if value is None:
        return "未知"
    return mapping.get(value, f"未知({value})")


def describe_rtk_mode(value: Optional[int]) -> str:
    """把 WTRTK 的 RTK 模式码转成中文说明。"""

    mapping = {0: "流动站", 1: "基站"}
    if value is None:
        return "未知"
    return mapping.get(value, f"未知({value})")


def describe_alignment_flag(value: Optional[int]) -> str:
    """把 WTRTK 的安装对准状态码转成中文说明。"""

    mapping = {0: "未对准", 1: "已对准", 2: "已二次对准"}
    if value is None:
        return "未知"
    return mapping.get(value, f"未知({value})")


def format_optional(value: Optional[float], suffix: str = "") -> str:
    """把可选浮点数格式化成展示文本。

    - 没值：返回 `--`
    - 有值：保留 3 位小数并拼接单位后缀
    """

    if value is None:
        return "--"
    return f"{value:.3f}{suffix}"


def format_optional_coordinate(value: Optional[float]) -> str:
    """把经纬度格式化成固定 8 位小数文本。"""

    if value is None:
        return "--"
    return f"{value:.8f}"


def build_wtrtk_summary(data: WTRTKData) -> str:
    """构建 WTRTK 的用户友好摘要。

    这份摘要有两个主要用途：
    - CLI 直接打印给用户看
    - 写入 `log_WTRTK`，给 Web 回放模式反向解析

    也就是说，这个摘要既是“人读的输出”，也是“回放模式的数据中间格式”。
    """

    lines = [
        "================ WTRTK 解析 ================",
        f"定位状态：{describe_positioning_status(data.positioning_status)} | RTK模式：{describe_rtk_mode(data.rtk_mode)} | 安装对准：{describe_alignment_flag(data.alignment_flag)}",
        f"经纬高：lat={format_optional_coordinate(data.latitude)} lon={format_optional_coordinate(data.longitude)} alt={format_optional(data.altitude, ' m')}",
        f"速度航向：speed={format_optional(data.speed_kmh, ' km/h')} heading={format_optional(data.heading_deg, '°')} gps_heading={format_optional(data.gps_heading, '°')}",
        f"差分距离：X={format_optional(data.diff_x, ' m')} Y={format_optional(data.diff_y, ' m')} Z={format_optional(data.diff_z, ' m')} R={format_optional(data.diff_r, ' m')}",
        f"姿态角：X={format_optional(data.angle_x, '°')} Y={format_optional(data.angle_y, '°')} Z={format_optional(data.angle_z, '°')}",
        f"磁场：X={format_optional(data.mag_x, ' uT')} Y={format_optional(data.mag_y, ' uT')} Z={format_optional(data.mag_z, ' uT')}",
        f"辅助信息：温度={format_optional(data.temperature_c, ' ℃')} 基站距离={format_optional(data.base_distance_m, ' m')}",
        "===========================================",
    ]
    return "\n".join(lines)


def write_wtrtk_log(
    log_file: object,
    receive_system_time: str,
    sentence_timestamp: Optional[str],
    raw_line: str,
    summary: str,
) -> None:
    """将 WTRTK 解析结果写入独立日志。

    日志格式目前约定为：
    - 接收系统时间：...
    - 时间：...（可选，只有报文本身有时间时才写）
    - 原始报文：...
    - WTRTK 中文摘要

    Web 回放模式依赖这个格式，所以这里不能随意改动字段名。
    """

    log_file.write(f"接收系统时间：{receive_system_time}\n")
    if sentence_timestamp is not None:
        log_file.write(f"时间：{sentence_timestamp}\n")
    log_file.write(f"原始报文：{raw_line}\n")
    log_file.write(f"{summary}\n")
    log_file.flush()


def log_checksum_warning(log_file: object, display_line: str) -> None:
    """将 checksum 警告按当前显示格式写入日志。

    这里刻意不再做额外格式转换，因为：
    - 如果当前是 HEX 模式，就应该把 HEX 原样写进去
    - 如果当前是文本模式，就应该把文本原样写进去
    """

    log_file.write(f"警告：checksum 校验失败: {display_line}\n")
    log_file.flush()


def run_serial_monitor(
    config: SerialRuntimeConfig,
    on_sentence: Optional[Callable[[SentenceUpdate], None]] = None,
    on_warning: Optional[Callable[[WarningUpdate], None]] = None,
    on_wtrtk: Optional[Callable[[WTRTKFrameEvent], None]] = None,
    stop_event: Optional[object] = None,
    pause_event: Optional[object] = None,
) -> None:
    """持续读取串口，并把 WTRTK 事件推送给订阅者。

    参数说明：
    - `config`: 串口与输出格式配置
    - `on_sentence`: 每处理完一条普通输出时触发；CLI 用它打印终端内容
    - `on_warning`: 出现 checksum/解析异常时触发；CLI 或 Web 可据此记录警告
    - `on_wtrtk`: 仅当成功解析出一条 WTRTK 时触发；Web 实时推送主要依赖它
    - `stop_event`: 外部停止信号；Web 服务关闭实时线程时会设置它
    - `pause_event`: 外部暂停信号；Web 服务点击“暂停”时会设置它
    """

    reader = SerialLineReader(config.port, config.baudrate, config.timeout)
    state_store = RTKStateStore()
    # 这个计数器只给 WTRTK 使用；普通句型不参与这个序号。
    wtrtk_index = 0
    try:
        for frame in reader.iter_frames(stop_event=stop_event, pause_event=pause_event):
            # Web 侧的实时线程通过 stop_event 主动中断循环，避免只能靠串口超时退出。
            if stop_event is not None and getattr(stop_event, "is_set")():
                break
            line = frame.text
            display_line = format_serial_output(frame.raw_bytes, frame.text, config.output_hex)
            receive_system_time = format_system_time()
            if frame.raw_bytes.startswith(b"$WTRAW,"):
                # WTRAW 仍需保留原始日志输出，但按当前需求不再做 checksum 校验，
                # 也不再把这类报文上抛为 warning。
                if on_sentence is not None:
                    on_sentence(SentenceUpdate(display_line, None, None, None, None))
                continue
            try:
                # 普通文本句型统一走 NMEA 解析入口。
                parsed = parse_nmea_sentence(line)
            except ValueError as exc:
                # parse_nmea_sentence 里会把 checksum 或字段错误抛成 ValueError；
                # 这里统一转成 WarningUpdate 交给上层决定怎么显示/记录。
                kind = "checksum" if str(exc).startswith("checksum 校验失败") else "parse"
                if on_warning is not None:
                    on_warning(WarningUpdate(str(exc), display_line, kind))
                continue
            except (IndexError, TypeError, OverflowError) as exc:
                # 兜底：畸形报文原则上只应抛 ValueError，但解析器一旦冒出别的异常
                # （历史上就出现过字段越界抛 IndexError），也绝不能让单条坏帧
                # 打断整个串口读取循环——CLI 会直接退出，Web 端实时线程会静默死亡。
                if on_warning is not None:
                    on_warning(WarningUpdate(f"{type(exc).__name__}: {exc}", display_line, "parse"))
                continue
            if parsed is None:
                # 说明这行内容不是当前支持的句型，但它本身又不是报错；
                # 这种情况仍然可以把原始内容交给上层决定是否显示。
                if on_sentence is not None:
                    on_sentence(SentenceUpdate(display_line, None, None, None, None))
                continue
            # 到这里说明已经拿到一条合法的已解析句型。
            state = state_store.update(parsed)
            # 状态 JSON 只在真正需要输出时才序列化：下游（CLI 和 Web）都只在
            # output_json 打开时才会用到它，默认配置下每帧做一次全量 dumps
            # （含全部 gsv_batches）纯属浪费，20Hz 下开销可观。
            state_json = (
                json.dumps(state.to_dict(), ensure_ascii=False) if config.output_json else None
            )
            sentence_timestamp = get_sentence_timestamp(parsed)
            # 只有 WTRTK 才额外附带“接收系统时间”给上层显示；
            # 这是之前项目里确定过的规则。
            receive_system_time_for_display = receive_system_time if parsed.sentence_id == "WTRTK" else None
            if on_sentence is not None:
                # 这里的 on_sentence 是“通用输出回调”。
                # CLI 会把它打印出来；Web 如果需要也可以拿它做普通文本流显示。
                on_sentence(
                    SentenceUpdate(
                        display_line=display_line,
                        state_json=state_json,
                        sentence_timestamp=sentence_timestamp,
                        receive_system_time=receive_system_time_for_display,
                        parsed_sentence_id=parsed.sentence_id,
                    )
                )
            if parsed.sentence_id == "WTRTK" and on_wtrtk is not None:
                # WTRTK 是项目重点句型，这里把它升格为结构化事件，直接交给上层消费。
                on_wtrtk(
                    WTRTKFrameEvent.from_data(
                        index=wtrtk_index,
                        receive_system_time=receive_system_time,
                        sentence_timestamp=sentence_timestamp,
                        raw_sentence=parsed.raw,
                        display_line=display_line,
                        data=parsed.data,
                    )
                )
                wtrtk_index += 1
    finally:
        # 无论是正常结束、用户 Ctrl+C，还是外部 stop_event 中断，都统一关闭串口。
        reader.close()
