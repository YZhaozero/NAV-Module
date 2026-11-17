"""NMEA checksum 相关工具。"""

from __future__ import annotations

from typing import Tuple


def split_nmea_sentence(sentence: str) -> Tuple[str, str]:
    """拆分 NMEA 报文主体与 checksum。"""
    # 去除输入字符串首尾的空白字符（换行符、空格等）
    text = sentence.strip()
    # 检查NMEA格式合法性：必须以$开头，且包含分隔*符
    if not text.startswith("$") or "*" not in text:
        # 格式不符合要求时抛出异常，附带原始报文信息
        raise ValueError(f"无效的 NMEA 报文格式: {sentence}")
    # 从索引1开始截取（跳过开头的$），以*为分隔符最多分割1次，得到报文主体和原始校验和
    body, checksum = text[1:].split("*", 1)
    # 将校验和转为大写格式后返回，统一格式方便后续比对
    return body, checksum.upper()


def calculate_nmea_checksum(body: str) -> str:
    """计算 NMEA 报文主体对应的十六进制 checksum。"""

    value = 0
    for char in body:
        value ^= ord(char)
    return f"{value:02X}"


def split_nmea_sentence_bytes(sentence: bytes) -> tuple[bytes, str]:
    """拆分原始字节报文主体与 checksum。"""

    text = sentence.strip()
    if not text.startswith(b"$") or b"*" not in text:
        raise ValueError(f"无效的 NMEA 报文格式: {sentence!r}")
    body, checksum = text[1:].rsplit(b"*", 1)
    return body, checksum.decode("ascii").upper()


def calculate_nmea_checksum_bytes(body: bytes) -> str:
    """计算原始字节报文主体对应的十六进制 checksum。"""

    value = 0
    for byte in body:
        value ^= byte
    return f"{value:02X}"


def validate_nmea_sentence(sentence: str) -> bool:
    """校验一条 NMEA 报文的 checksum 是否正确。"""

    try:
        body, expected_checksum = split_nmea_sentence(sentence)
    except ValueError:
        return False
    return calculate_nmea_checksum(body) == expected_checksum


def validate_nmea_sentence_bytes(sentence: bytes) -> bool:
    """校验一条原始字节报文的 checksum 是否正确。"""

    try:
        body, expected_checksum = split_nmea_sentence_bytes(sentence)
    except ValueError:
        return False
    return calculate_nmea_checksum_bytes(body) == expected_checksum
