"""main 入口相关测试。"""

import argparse
import datetime as dt
import io
import unittest
from unittest.mock import Mock, patch

from nav_rtk.parser import cli as main
from nav_rtk.parser.cli import (
    DefaultConfig,
    build_argument_parser,
    build_wtrtk_summary,
    log_checksum_warning,
    print_state,
    resolve_config,
    write_wtrtk_log,
)
from nav_rtk.parser.models import WTRTKData
from nav_rtk.parser.wtrtk_runtime import SentenceUpdate, format_serial_output, format_system_time


class MainTestCase(unittest.TestCase):
    """验证 main.py 中的配置与输出格式化逻辑。"""

    def test_format_serial_output_plain_text(self) -> None:
        """默认按文本显示串口内容。"""

        raw = b"$GNGGA,070225.100,,,,,0,00,99.99,,,,,,*4B"
        text = "$GNGGA,070225.100,,,,,0,00,99.99,,,,,,*4B"
        self.assertEqual(format_serial_output(raw, text, output_hex=False), text)

    def test_format_serial_output_hex(self) -> None:
        """开启 --hex 后按十六进制显示原始字节。"""

        self.assertEqual(format_serial_output(b"$GN", "$GN", output_hex=True), "24 47 4E")

    def test_resolve_config_hex_default_false(self) -> None:
        """默认配置下不启用十六进制显示。"""

        args = build_argument_parser().parse_args([])
        config, used_defaults = resolve_config(args)
        self.assertEqual(config, DefaultConfig())
        self.assertIn("output_hex", used_defaults)

    def test_resolve_config_hex_flag(self) -> None:
        """传入 --hex 后启用十六进制显示。"""

        args = build_argument_parser().parse_args(["--hex"])
        config, _ = resolve_config(args)
        self.assertTrue(config.output_hex)

    def test_log_checksum_warning_keeps_current_display_format(self) -> None:
        """日志中的 checksum 警告应直接使用当前显示形式。"""

        fake_log = io.StringIO()
        log_checksum_warning(fake_log, "24 57 54 52 41 57 2C EF 2A 30 31")
        self.assertEqual(fake_log.getvalue(), "警告：checksum 校验失败: 24 57 54 52 41 57 2C EF 2A 30 31\n")

    def test_format_system_time(self) -> None:
        """系统时间应格式化为 HH:MM:SS.mmm。"""

        value = dt.datetime(2026, 7, 24, 10, 29, 32, 456789)
        self.assertEqual(format_system_time(value), "10:29:32.456")

    def test_build_wtrtk_summary(self) -> None:
        """WTRTK 摘要应输出用户友好的中文说明。"""

        data = WTRTKData(
            diff_x=0.1,
            diff_y=0.2,
            diff_z=0.3,
            diff_r=0.4,
            angle_x=15.44,
            angle_y=-6.31,
            angle_z=-0.62,
            positioning_status=4,
            mag_x=-217.68,
            mag_y=-149.553,
            mag_z=-170.513,
            rtk_mode=0,
            gps_heading=None,
            alignment_flag=1,
            reserved_1=0.0,
            temperature_c=26.6,
            base_distance_m=0.0,
            reserved_2=0.0,
            latitude=23.079175043666665,
            longitude=114.253874036,
            latitude_raw="2304.75050262",
            latitude_dir="N",
            longitude_raw="11415.23244216",
            longitude_dir="E",
            speed_kmh=0.0,
            heading_deg=0.62,
            altitude=69.73,
            raw="$WTRTK,...*75",
        )
        summary = build_wtrtk_summary(data)
        self.assertIn("WTRTK 解析", summary)
        self.assertIn("定位状态：固定解", summary)
        self.assertIn("RTK模式：流动站", summary)
        self.assertIn("安装对准：已对准", summary)
        self.assertIn("lat=23.07917504", summary)

    def test_write_wtrtk_log(self) -> None:
        """WTRTK 独立日志应写入接收系统时间、报文时间、原始报文和摘要。"""

        fake_log = io.StringIO()
        write_wtrtk_log(fake_log, "10:29:32.456", "06:54:48.300", "$WTRTK,...*75", "SUMMARY")
        self.assertEqual(
            fake_log.getvalue(),
            "接收系统时间：10:29:32.456\n时间：06:54:48.300\n原始报文：$WTRTK,...*75\nSUMMARY\n",
        )

    def test_print_state_with_receive_system_time(self) -> None:
        """WTRTK 终端输出应包含接收系统时间。"""

        with patch("sys.stdout", new_callable=io.StringIO) as stdout:
            print_state(
                SentenceUpdate(
                    display_line="$WTRTK,...*75",
                    state_json=None,
                    sentence_timestamp=None,
                    receive_system_time="10:29:32.456",
                    parsed_sentence_id="WTRTK",
                ),
                False,
            )
        self.assertEqual(stdout.getvalue(), "接收系统时间：10:29:32.456\n收：$WTRTK,...*75\n")

    def test_main_exit_safely_when_serial_not_inserted(self) -> None:
        """未检测到串口时打印友好提示并安全退出。"""

        fake_log = io.StringIO()
        fake_parser = Mock()
        fake_parser.parse_args.return_value = argparse.Namespace()

        with (
            patch("nav_rtk.parser.cli.build_argument_parser", return_value=fake_parser),
            patch.object(main.Path, "mkdir", return_value=None),
            patch.object(main.Path, "open", return_value=fake_log),
            patch("nav_rtk.parser.cli.run_serial_monitor", side_effect=main.SerialPortUnavailableError("当前没有检测到串口插入")),
            patch("sys.stdout", new_callable=io.StringIO) as stdout,
            patch("sys.stderr", new_callable=io.StringIO),
        ):
            exit_code = main.main()

        self.assertEqual(exit_code, 1)
        self.assertIn("================ 串口状态 ================", stdout.getvalue())
        self.assertIn("ERROR: 端口不存在", stdout.getvalue())


if __name__ == "__main__":
    unittest.main()
