"""WTRTK runtime 公共逻辑测试。"""

import unittest
from unittest.mock import patch

from nav_rtk.parser.nmea_parser import parse_nmea_sentence
from nav_rtk.parser.serial_client import SerialFrame
from nav_rtk.parser.models import WTRTKData
from nav_rtk.parser.wtrtk_runtime import SerialRuntimeConfig, WTRTKFrameEvent, format_state_lines, run_serial_monitor


def fake_reader_for(frames):
    """构造一个替代 SerialLineReader 的假读取器类，按顺序产出给定的帧。"""

    class FakeReader:
        def __init__(self, _port: str, _baudrate: int, _timeout: float) -> None:
            pass

        def iter_frames(self, stop_event=None, pause_event=None):
            """按测试数据依次产出串口帧。"""
            del stop_event, pause_event
            for frame in frames:
                yield frame

        def close(self) -> None:
            """测试结束时的空关闭实现。"""

    return FakeReader


class WTRTKRuntimeTests(unittest.TestCase):
    def test_format_state_lines(self) -> None:
        lines = format_state_lines(
            raw_line="$WTRTK,...*75",
            state_json='{"ok":true}',
            output_json=True,
            sentence_timestamp="06:54:48.300",
            receive_system_time="11:04:50.925",
        )
        self.assertEqual(
            lines,
            [
                "接收系统时间：11:04:50.925",
                "时间：06:54:48.300",
                "收：$WTRTK,...*75",
                '状态：{"ok":true}',
            ],
        )

    def test_wtrtk_frame_event_to_dict(self) -> None:
        data = WTRTKData(
            diff_x=0.1,
            diff_y=0.2,
            diff_z=0.3,
            diff_r=0.4,
            angle_x=1.0,
            angle_y=2.0,
            angle_z=3.0,
            positioning_status=4,
            mag_x=-1.0,
            mag_y=-2.0,
            mag_z=-3.0,
            rtk_mode=0,
            gps_heading=None,
            alignment_flag=1,
            reserved_1=0.0,
            temperature_c=23.7,
            base_distance_m=0.0,
            reserved_2=0.0,
            latitude=23.1,
            longitude=114.2,
            latitude_raw="2304.75",
            latitude_dir="N",
            longitude_raw="11415.23",
            longitude_dir="E",
            speed_kmh=0.0,
            heading_deg=86.35,
            altitude=0.0,
            raw="$WTRTK,...*64",
        )
        event = WTRTKFrameEvent.from_data(
            index=7,
            receive_system_time="11:04:50.925",
            sentence_timestamp=None,
            raw_sentence=data.raw,
            display_line=data.raw,
            data=data,
        )
        payload = event.to_dict()
        self.assertEqual(payload["index"], 7)
        self.assertEqual(payload["timeText"], "11:04:50.925")
        self.assertEqual(payload["positioningText"], "固定解")
        self.assertEqual(payload["rtkModeText"], "流动站")
        self.assertEqual(payload["alignmentText"], "已对准")

    def test_run_serial_monitor_keeps_wtraw_output_without_warning(self) -> None:
        """WTRAW 报文应保留原始输出，但不触发 checksum 警告。"""

        frames = [
            SerialFrame(
                text="$WTRAW,\x13\x00\x0c\x08*EE",
                raw_bytes=b"$WTRAW,\x13\x00\x0c\x08*EE",
            )
        ]
        sentence_updates = []
        warning_updates = []

        config = SerialRuntimeConfig(port="/dev/null", baudrate=230400)
        with patch("nav_rtk.parser.wtrtk_runtime.SerialLineReader", fake_reader_for(frames)):
            run_serial_monitor(
                config,
                on_sentence=sentence_updates.append,
                on_warning=warning_updates.append,
            )

        self.assertEqual(len(sentence_updates), 1)
        self.assertEqual(sentence_updates[0].display_line, "$WTRAW,\x13\x00\x0c\x08*EE")
        self.assertIsNone(sentence_updates[0].state_json)
        self.assertEqual(warning_updates, [])

    def test_run_serial_monitor_survives_unexpected_parser_exception(self) -> None:
        """解析器抛出非 ValueError 异常时，串口读取循环也必须继续。

        回归用例：字段越界曾让 IndexError 逃出 run_serial_monitor，
        导致 CLI 直接退出、Web 端实时线程静默死亡（界面仍显示“运行中”）。
        """

        bad = "$GNRMC,,V*03"
        good = "$GNRMC,083126.100,V,,,,,,,030726,,E,N,V*63"
        frames = [
            SerialFrame(text=bad, raw_bytes=bad.encode("ascii")),
            SerialFrame(text=good, raw_bytes=good.encode("ascii")),
        ]

        warning_updates = []
        sentence_updates = []

        def flaky_parse(sentence: str):
            """第一条报文模拟解析器内部异常，其余走真实解析。"""

            if sentence == bad:
                raise IndexError("模拟字段越界")
            return parse_nmea_sentence(sentence)

        config = SerialRuntimeConfig(port="/dev/null", baudrate=230400)
        with patch("nav_rtk.parser.wtrtk_runtime.SerialLineReader", fake_reader_for(frames)), patch(
            "nav_rtk.parser.wtrtk_runtime.parse_nmea_sentence", side_effect=flaky_parse
        ):
            run_serial_monitor(
                config,
                on_sentence=sentence_updates.append,
                on_warning=warning_updates.append,
            )

        # 坏帧被转成 parse 警告，而不是让循环炸掉
        self.assertEqual(len(warning_updates), 1)
        self.assertEqual(warning_updates[0].kind, "parse")
        self.assertIn("IndexError", warning_updates[0].message)
        self.assertEqual(warning_updates[0].display_line, bad)
        # 关键断言：坏帧之后的正常帧仍然被解析并交给上层
        self.assertEqual(len(sentence_updates), 1)
        self.assertEqual(sentence_updates[0].parsed_sentence_id, "RMC")


if __name__ == "__main__":
    unittest.main()
