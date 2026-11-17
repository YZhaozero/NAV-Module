"""NMEA 报文解析逻辑。"""

from __future__ import annotations

import re
from typing import List, Optional

from .checksum import split_nmea_sentence, validate_nmea_sentence, validate_nmea_sentence_bytes
from .models import GGAData, GSAData, GSVData, ParsedSentence, RMCData, SatelliteInfo, WTRTKData

# NMEA 报文正则表达式
NMEA_SENTENCE_PATTERN = re.compile(r"(\$[^*\r\n]*\*[0-9A-Fa-f]{2})")
HEX_BYTES = b"0123456789ABCDEFabcdef"


def parse_float(value: str) -> Optional[float]:
    """将字符串安全转换为浮点数。"""

    if not value or value == "--":
        return None
    return float(value)


def parse_int(value: str) -> Optional[int]:
    """将字符串安全转换为整数。"""

    if not value or value == "--":
        return None
    return int(value)


def field_at(fields: List[str], index: int) -> str:
    """安全读取字段，越界时返回空字符串。

    串口里可能混入设备噪声或被截断的报文，直接下标取值会抛 IndexError。
    返回空串后，parse_float / parse_int / convert_nmea_coordinate 会统一把它
    转成 None，正好符合项目里“设备在线但字段缺失”的既有语义。
    """

    if 0 <= index < len(fields):
        return fields[index]
    return ""


def parse_date(date_value: str) -> Optional[str]:
    """将 ddmmyy 日期转换为 yyyy-mm-dd。"""

    if len(date_value) != 6:
        return None
    day = int(date_value[0:2])
    month = int(date_value[2:4])
    year = int(date_value[4:6])
    full_year = 2000 + year if year < 80 else 1900 + year
    return f"{full_year:04d}-{month:02d}-{day:02d}"


def convert_nmea_coordinate(value: str, direction: str) -> Optional[float]:
    """将 NMEA 格式坐标转换为十进制度。"""

    if not value or not direction:
        return None
    dot_index = value.find(".")
    if dot_index < 0 or dot_index < 2:
        return None
    degree_length = dot_index - 2
    degrees = float(value[:degree_length])
    minutes = float(value[degree_length:])
    decimal = degrees + minutes / 60.0
    if direction in {"S", "W"}:
        decimal = -decimal
    return decimal


def parse_rmc(fields: List[str], talker: str, raw: str) -> ParsedSentence:
    """解析 RMC 报文。"""

    date_text = field_at(fields, 9)
    data = RMCData(
        talker=talker,
        utc_time=field_at(fields, 1) or None,
        status=field_at(fields, 2) or None,
        latitude=convert_nmea_coordinate(field_at(fields, 3), field_at(fields, 4)),
        longitude=convert_nmea_coordinate(field_at(fields, 5), field_at(fields, 6)),
        speed_knots=parse_float(field_at(fields, 7)),
        course=parse_float(field_at(fields, 8)),
        date=parse_date(date_text) if date_text else None,
        mode=field_at(fields, 12) or None,
        nav_status=field_at(fields, 13) or None,
        raw=raw,
    )
    return ParsedSentence(sentence_id="RMC", talker=talker, data=data, raw=raw)


def parse_gga(fields: List[str], talker: str, raw: str) -> ParsedSentence:
    """解析 GGA 报文。"""

    data = GGAData(
        talker=talker,
        utc_time=field_at(fields, 1) or None,
        latitude=convert_nmea_coordinate(field_at(fields, 2), field_at(fields, 3)),
        longitude=convert_nmea_coordinate(field_at(fields, 4), field_at(fields, 5)),
        fix_quality=parse_int(field_at(fields, 6)),
        satellites_used=parse_int(field_at(fields, 7)),
        hdop=parse_float(field_at(fields, 8)),
        altitude=parse_float(field_at(fields, 9)),
        raw=raw,
    )
    return ParsedSentence(sentence_id="GGA", talker=talker, data=data, raw=raw)


def parse_gsa(fields: List[str], talker: str, raw: str) -> ParsedSentence:
    """解析 GSA 报文。"""

    satellites_used = [item for item in fields[3:15] if item]
    data = GSAData(
        talker=talker,
        mode1=field_at(fields, 1) or None,
        fix_type=parse_int(field_at(fields, 2)),
        satellites_used=satellites_used,
        pdop=parse_float(field_at(fields, 15)),
        hdop=parse_float(field_at(fields, 16)),
        vdop=parse_float(field_at(fields, 17)),
        system_id=field_at(fields, 18) or None,
        raw=raw,
    )
    return ParsedSentence(sentence_id="GSA", talker=talker, data=data, raw=raw)


def parse_gsv(fields: List[str], talker: str, raw: str) -> ParsedSentence:
    """解析 GSV 报文，并兼容末尾扩展 signal_id。"""

    # 报文总数/序号/卫星总数是结构性字段，缺失说明报文被截断。
    # 这里显式抛 ValueError，让 runtime 归到“parse 警告”，而不是 IndexError。
    if len(fields) < 4:
        raise ValueError(f"GSV 字段数量不足({len(fields)}): {raw}")

    satellites: List[SatelliteInfo] = []
    tail_fields = fields[4:]
    signal_id = None
    if len(tail_fields) % 4 == 1:
        signal_id = tail_fields[-1] or None
        tail_fields = tail_fields[:-1]
    for index in range(0, len(tail_fields), 4):
        group = tail_fields[index : index + 4]
        if len(group) < 4 or not group[0]:
            continue
        satellites.append(
            SatelliteInfo(
                prn=group[0],
                elevation=parse_int(group[1]),
                azimuth=parse_int(group[2]),
                snr=parse_int(group[3]),
            )
        )
    data = GSVData(
        talker=talker,
        total_messages=int(fields[1]),
        message_number=int(fields[2]),
        total_satellites=int(fields[3]),
        satellites=satellites,
        signal_id=signal_id,
        raw=raw,
    )
    return ParsedSentence(sentence_id="GSV", talker=talker, data=data, raw=raw)


def parse_wtrtk(fields: List[str], raw: str) -> ParsedSentence:
    """解析 WTRTK 报文。"""

    if len(fields) < 26:
        raise ValueError(f"WTRTK 字段数量不足: {raw}")
    data = WTRTKData(
        diff_x=parse_float(fields[1]),
        diff_y=parse_float(fields[2]),
        diff_z=parse_float(fields[3]),
        diff_r=parse_float(fields[4]),
        angle_x=parse_float(fields[5]),
        angle_y=parse_float(fields[6]),
        angle_z=parse_float(fields[7]),
        positioning_status=parse_int(fields[8]),
        mag_x=parse_float(fields[9]),
        mag_y=parse_float(fields[10]),
        mag_z=parse_float(fields[11]),
        rtk_mode=parse_int(fields[12]),
        gps_heading=parse_float(fields[13]),
        alignment_flag=parse_int(fields[14]),
        reserved_1=parse_float(fields[15]),
        temperature_c=parse_float(fields[16]),
        base_distance_m=parse_float(fields[17]),
        reserved_2=parse_float(fields[18]),
        latitude=convert_nmea_coordinate(fields[19], fields[20]),
        longitude=convert_nmea_coordinate(fields[21], fields[22]),
        latitude_raw=fields[19] or None,
        latitude_dir=fields[20] or None,
        longitude_raw=fields[21] or None,
        longitude_dir=fields[22] or None,
        speed_kmh=parse_float(fields[23]),
        heading_deg=parse_float(fields[24]),
        altitude=parse_float(fields[25]),
        raw=raw,
    )
    return ParsedSentence(sentence_id="WTRTK", talker="WT", data=data, raw=raw)


def extract_nmea_sentence(text: str) -> Optional[str]:
    """从原始文本中提取一条 NMEA 报文，兼容日志前缀。"""
    # 去除输入文本首尾的空白字符（空格、换行符等）
    stripped = text.strip()
    # 如果处理后文本为空，直接返回None，无法提取有效报文
    if not stripped:
        return None
    # 使用预定义的NMEA正则表达式在处理后的文本中搜索匹配的报文
    match = NMEA_SENTENCE_PATTERN.search(stripped)
    # 如果成功匹配到符合格式的NMEA报文
    if match is not None:
        # 提取匹配到的完整报文字符串
        matched = match.group(1)
        # 拆分报文主体和校验和：去掉开头的$符号，从最后一个*处分割
        body, checksum = matched[1:].rsplit("*", 1)
        # 重新拼接为标准格式，并将校验和转为大写，保证格式统一
        return f"${body}*{checksum.upper()}"
    # 未匹配到任何符合格式的NMEA报文，返回None
    return None


def extract_wtraw_sentence_bytes(raw_bytes: bytes) -> Optional[bytes]:
    """从原始字节中提取完整的 WTRAW 报文。"""

    stripped = raw_bytes.strip()
    if not stripped.startswith(b"$WTRAW,"):
        return None
    star_index = stripped.rfind(b"*")
    if star_index <= 0 or len(stripped) - star_index != 3:
        return None
    checksum = stripped[star_index + 1 :]
    if len(checksum) != 2 or any(byte not in HEX_BYTES for byte in checksum):
        return None
    return stripped


def validate_wtraw_sentence_bytes(raw_bytes: bytes) -> bool:
    """对 WTRAW 使用原始字节级 checksum 校验。"""

    sentence = extract_wtraw_sentence_bytes(raw_bytes)
    if sentence is None:
        return False
    return validate_nmea_sentence_bytes(sentence)


def parse_nmea_sentence(sentence: str) -> Optional[ParsedSentence]:
    """解析一条受支持的 NMEA 报文。"""

    normalized_sentence = extract_nmea_sentence(sentence)
    if normalized_sentence is None:
        return None
    if not validate_nmea_sentence(normalized_sentence):
        raise ValueError(f"checksum 校验失败: {sentence}")
    body, _ = split_nmea_sentence(normalized_sentence)
    fields = body.split(",")
    formatter = fields[0]
    if formatter == "WTRTK":
        return parse_wtrtk(fields, normalized_sentence)
    if len(formatter) < 5:
        return None
    talker = formatter[:2]
    sentence_id = formatter[2:]
    if sentence_id == "RMC":
        return parse_rmc(fields, talker, normalized_sentence)
    if sentence_id == "GGA":
        return parse_gga(fields, talker, normalized_sentence)
    if sentence_id == "GSA":
        return parse_gsa(fields, talker, normalized_sentence)
    if sentence_id == "GSV":
        return parse_gsv(fields, talker, normalized_sentence)
    return None
