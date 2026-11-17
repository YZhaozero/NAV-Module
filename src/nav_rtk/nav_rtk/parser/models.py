"""项目使用的数据结构定义。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class RMCData:
    """保存 RMC 报文的关键字段。"""

    talker: str
    utc_time: Optional[str]
    status: Optional[str]
    latitude: Optional[float]
    longitude: Optional[float]
    speed_knots: Optional[float]
    course: Optional[float]
    date: Optional[str]
    mode: Optional[str]
    nav_status: Optional[str]
    raw: str


@dataclass
class GGAData:
    """保存 GGA 报文的关键字段。"""

    talker: str
    utc_time: Optional[str]
    latitude: Optional[float]
    longitude: Optional[float]
    fix_quality: Optional[int]
    satellites_used: Optional[int]
    hdop: Optional[float]
    altitude: Optional[float]
    raw: str


@dataclass
class GSAData:
    """保存 GSA 报文的关键字段。"""

    talker: str
    mode1: Optional[str]
    fix_type: Optional[int]
    satellites_used: List[str]
    pdop: Optional[float]
    hdop: Optional[float]
    vdop: Optional[float]
    system_id: Optional[str]
    raw: str


@dataclass
class SatelliteInfo:
    """表示一颗卫星的观测信息。"""

    prn: str
    elevation: Optional[int]
    azimuth: Optional[int]
    snr: Optional[int]


@dataclass
class GSVData:
    """保存 GSV 报文的一帧解析结果。"""

    talker: str
    total_messages: int
    message_number: int
    total_satellites: int
    satellites: List[SatelliteInfo]
    signal_id: Optional[str]
    raw: str


@dataclass
class WTRTKData:
    """保存 WTRTK 报文的结构化字段。"""

    diff_x: Optional[float]       # X方向基线偏移量，单位：米
    diff_y: Optional[float]       # Y方向基线偏移量，单位：米
    diff_z: Optional[float]       # Z方向基线偏移量，单位：米
    diff_r: Optional[float]       # 差分R距离，单位：米
    angle_x: Optional[float]      # X轴姿态角，单位：度
    angle_y: Optional[float]      # Y轴姿态角，单位：度
    angle_z: Optional[float]      # Z轴姿态角，单位：度
    positioning_status: Optional[int] # 0：表示初始化，1：表示单点定位，2：表示码差分，4：表示固定解，5：表示浮点解
    mag_x: Optional[float]        # X轴磁力计数据，单位：高斯
    mag_y: Optional[float]        # Y轴磁力计数据，单位：高斯
    mag_z: Optional[float]        # Z轴磁力计数据，单位：高斯
    rtk_mode: Optional[int]       # RTK工作模式：0=流动站，1=基站
    gps_heading: Optional[float]  # GPS航向角，单位：度
    alignment_flag: Optional[int] # 对准状态：0=未对准，1=已对准
    reserved_1: Optional[float]    # 保留字段1
    temperature_c: Optional[float] # 设备温度，单位：摄氏度
    base_distance_m: Optional[float] # 基准站距离，单位：米
    reserved_2: Optional[float]    # 保留字段2
    latitude: Optional[float]     # 纬度（十进制度），单位：度
    longitude: Optional[float]    # 经度（十进制度），单位：度
    latitude_raw: Optional[str]   # 原始纬度字符串（ddmm.mmmm格式）
    latitude_dir: Optional[str]   # 纬度方向：N=北纬，S=南纬
    longitude_raw: Optional[str]  # 原始经度字符串（dddmm.mmmm格式）
    longitude_dir: Optional[str]  # 经度方向：E=东经，W=西经
    speed_kmh: Optional[float]    # 地面速度，单位：公里/小时
    heading_deg: Optional[float]   # 真航向角，单位：度
    altitude: Optional[float]     # 海拔高度，单位：米
    raw: str                      # 报文原始字符串


@dataclass
class ParsedSentence:
    """统一封装一条已解析报文。"""

    sentence_id: str
    talker: str
    data: Any
    raw: str


@dataclass
class RTKState:
    """聚合多个 NMEA 句型后的当前状态。"""

    utc_time: Optional[str] = None
    utc_date: Optional[str] = None
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None
    fix_valid: bool = False
    fix_quality: Optional[int] = None
    # GSA 的定位类型：1=无定位，2=二维定位，3=三维定位
    fix_type: Optional[int] = None
    # WTRTK 自带的定位状态码：0=初始化，1=单点定位，2=码差分，4=固定解，5=浮点解
    # 和 fix_type 是两套互不兼容的枚举，必须分开存，否则会互相覆盖。
    wt_fix_type: Optional[int] = None
    satellites_used: Optional[int] = None
    hdop: Optional[float] = None
    pdop: Optional[float] = None
    vdop: Optional[float] = None
    speed_knots: Optional[float] = None
    speed_kmh: Optional[float] = None
    course: Optional[float] = None
    positioning_status: Optional[int] = None
    rtk_mode: Optional[int] = None
    gps_heading: Optional[float] = None
    alignment_flag: Optional[int] = None
    temperature_c: Optional[float] = None
    base_distance_m: Optional[float] = None
    diff_x: Optional[float] = None
    diff_y: Optional[float] = None
    diff_z: Optional[float] = None
    diff_r: Optional[float] = None
    angle_x: Optional[float] = None
    angle_y: Optional[float] = None
    angle_z: Optional[float] = None
    mag_x: Optional[float] = None
    mag_y: Optional[float] = None
    mag_z: Optional[float] = None
    last_rmc_raw: Optional[str] = None
    last_gga_raw: Optional[str] = None
    last_gsa_raw: Optional[str] = None
    last_wtrtk_raw: Optional[str] = None
    gsv_batches: Dict[str, GSVData] = field(default_factory=dict)
    raw_sentences: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """将状态对象转换为便于输出的字典。"""

        gsv_sources = {
            key: {
                "talker": value.talker,
                "total_messages": value.total_messages,
                "message_number": value.message_number,
                "total_satellites": value.total_satellites,
                "signal_id": value.signal_id,
                "satellites": [
                    {
                        "prn": satellite.prn,
                        "elevation": satellite.elevation,
                        "azimuth": satellite.azimuth,
                        "snr": satellite.snr,
                    }
                    for satellite in value.satellites
                ],
            }
            for key, value in self.gsv_batches.items()
        }
        return {
            "utc_time": self.utc_time,
            "utc_date": self.utc_date,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "fix_valid": self.fix_valid,
            "fix_quality": self.fix_quality,
            "fix_type": self.fix_type,
            "wt_fix_type": self.wt_fix_type,
            "satellites_used": self.satellites_used,
            "hdop": self.hdop,
            "pdop": self.pdop,
            "vdop": self.vdop,
            "speed_knots": self.speed_knots,
            "speed_kmh": self.speed_kmh,
            "course": self.course,
            "positioning_status": self.positioning_status,
            "rtk_mode": self.rtk_mode,
            "gps_heading": self.gps_heading,
            "alignment_flag": self.alignment_flag,
            "temperature_c": self.temperature_c,
            "base_distance_m": self.base_distance_m,
            "diff_x": self.diff_x,
            "diff_y": self.diff_y,
            "diff_z": self.diff_z,
            "diff_r": self.diff_r,
            "angle_x": self.angle_x,
            "angle_y": self.angle_y,
            "angle_z": self.angle_z,
            "mag_x": self.mag_x,
            "mag_y": self.mag_y,
            "mag_z": self.mag_z,
            "gsv_sources": gsv_sources,
            "raw_sentence_count": len(self.raw_sentences),
        }
