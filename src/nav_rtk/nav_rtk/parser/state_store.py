"""RTK 状态聚合逻辑。"""

from __future__ import annotations

from .models import GGAData, GSAData, GSVData, ParsedSentence, RMCData, RTKState, WTRTKData


class RTKStateStore:
    """聚合多种句型并输出当前 RTK 状态。"""

    def __init__(self) -> None:
        """初始化空状态。"""

        self._state = RTKState()

    def update(self, parsed: ParsedSentence) -> RTKState:
        """根据一条解析结果更新当前状态。"""

        self._state.raw_sentences.append(parsed.raw)
        if parsed.sentence_id == "RMC":
            self._apply_rmc(parsed.data)
        elif parsed.sentence_id == "GGA":
            self._apply_gga(parsed.data)
        elif parsed.sentence_id == "GSA":
            self._apply_gsa(parsed.data)
        elif parsed.sentence_id == "GSV":
            self._apply_gsv(parsed.data)
        elif parsed.sentence_id == "WTRTK":
            self._apply_wtrtk(parsed.data)
        return self.snapshot()

    def snapshot(self) -> RTKState:
        """返回当前状态快照。"""

        return self._state

    def _apply_rmc(self, data: RMCData) -> None:
        """应用 RMC 报文内容。"""

        self._state.utc_time = data.utc_time or self._state.utc_time
        self._state.utc_date = data.date or self._state.utc_date
        self._state.fix_valid = data.status == "A"
        self._state.speed_knots = data.speed_knots
        self._state.course = data.course
        self._state.last_rmc_raw = data.raw
        if data.latitude is not None:
            self._state.latitude = data.latitude
        if data.longitude is not None:
            self._state.longitude = data.longitude

    def _apply_gga(self, data: GGAData) -> None:
        """应用 GGA 报文内容。"""

        self._state.utc_time = data.utc_time or self._state.utc_time
        self._state.fix_quality = data.fix_quality
        self._state.satellites_used = data.satellites_used
        self._state.hdop = data.hdop
        self._state.altitude = data.altitude
        self._state.last_gga_raw = data.raw
        if data.latitude is not None:
            self._state.latitude = data.latitude
        if data.longitude is not None:
            self._state.longitude = data.longitude
        if data.fix_quality is not None:
            self._state.fix_valid = data.fix_quality > 0

    def _apply_gsa(self, data: GSAData) -> None:
        """应用 GSA 报文内容。"""

        self._state.fix_type = data.fix_type
        self._state.pdop = data.pdop
        self._state.hdop = data.hdop if data.hdop is not None else self._state.hdop
        self._state.vdop = data.vdop
        self._state.last_gsa_raw = data.raw

    def _apply_gsv(self, data: GSVData) -> None:
        """应用 GSV 报文内容。"""

        batch_key = f"{data.talker}:{data.signal_id or 'default'}:{data.message_number}"
        self._state.gsv_batches[batch_key] = data

    def _apply_wtrtk(self, data: WTRTKData) -> None:
        """应用 WTRTK 报文内容。"""

        self._state.positioning_status = data.positioning_status
        self._state.rtk_mode = data.rtk_mode
        self._state.gps_heading = data.gps_heading
        self._state.alignment_flag = data.alignment_flag
        self._state.temperature_c = data.temperature_c
        self._state.base_distance_m = data.base_distance_m
        self._state.diff_x = data.diff_x
        self._state.diff_y = data.diff_y
        self._state.diff_z = data.diff_z
        self._state.diff_r = data.diff_r
        self._state.angle_x = data.angle_x
        self._state.angle_y = data.angle_y
        self._state.angle_z = data.angle_z
        self._state.mag_x = data.mag_x
        self._state.mag_y = data.mag_y
        self._state.mag_z = data.mag_z
        self._state.last_wtrtk_raw = data.raw
        if data.latitude is not None:
            self._state.latitude = data.latitude
        if data.longitude is not None:
            self._state.longitude = data.longitude
        if data.altitude is not None:
            self._state.altitude = data.altitude
        if data.speed_kmh is not None:
            self._state.speed_kmh = data.speed_kmh
            self._state.speed_knots = data.speed_kmh / 1.852
        if data.heading_deg is not None:
            self._state.course = data.heading_deg
        if data.positioning_status is not None:
            self._state.fix_valid = data.positioning_status in {2, 4, 5}
            # 写独立的 wt_fix_type，而不是复用 fix_type：
            # 两者的枚举含义不同（WTRTK 是 0/1/2/4/5，GSA 是 1/2/3），
            # 而 WTRTK 频率远高于 GSA，共用同一个字段时实际总是 WTRTK 覆盖掉 GSA。
            self._state.wt_fix_type = data.positioning_status
