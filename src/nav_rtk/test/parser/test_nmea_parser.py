"""NMEA 解析相关测试。"""

from pathlib import Path
import unittest

from nav_rtk.parser.checksum import calculate_nmea_checksum
from nav_rtk.parser.nmea_parser import parse_nmea_sentence, validate_wtraw_sentence_bytes
from nav_rtk.parser.state_store import RTKStateStore


FIXTURE_PATH = Path(__file__).parent / "fixtures" / "sample_nmea.txt"


class NMEAParserTestCase(unittest.TestCase):
    """验证 NMEA 解析与状态聚合逻辑。"""

    def test_parse_rmc_sentence(self) -> None:
        """验证 RMC 报文解析结果。"""

        sentence = "$GNRMC,074630.100,V,,,,,,,030726,,E,N,V*6B"
        parsed = parse_nmea_sentence(sentence)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.sentence_id, "RMC")
        self.assertEqual(parsed.data.status, "V")
        self.assertEqual(parsed.data.date, "2026-07-03")
        self.assertIsNone(parsed.data.latitude)
        self.assertIsNone(parsed.data.longitude)

    def test_parse_rmc_sentence_from_logged_line(self) -> None:
        """验证带日志前缀的串口行也能提取并解析。"""

        sentence = "[16:31:25.428]收←◆$GNRMC,083126.200,V,,,,,,,030726,,E,N,V*60"
        parsed = parse_nmea_sentence(sentence)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.sentence_id, "RMC")
        self.assertEqual(parsed.raw, "$GNRMC,083126.200,V,,,,,,,030726,,E,N,V*60")
        self.assertEqual(parsed.data.utc_time, "083126.200")
        self.assertEqual(parsed.data.date, "2026-07-03")

    def test_parse_gga_sentence(self) -> None:
        """验证 GGA 报文解析结果。"""

        sentence = "$GNGGA,074630.100,,,,,0,00,99.99,,,,,,*4F"
        parsed = parse_nmea_sentence(sentence)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.sentence_id, "GGA")
        self.assertEqual(parsed.data.fix_quality, 0)
        self.assertEqual(parsed.data.satellites_used, 0)
        self.assertEqual(parsed.data.hdop, 99.99)

    def test_parse_gsv_with_signal_id(self) -> None:
        """验证带扩展 signal_id 的 GSV 报文解析。"""

        sentence = "$GPGSV,2,1,7,05,23,111,00,15,43,041,00,18,62,240,00,20,19,055,00,8*5B"
        parsed = parse_nmea_sentence(sentence)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.sentence_id, "GSV")
        self.assertEqual(parsed.data.signal_id, "8")
        self.assertEqual(parsed.data.total_satellites, 7)
        self.assertEqual(len(parsed.data.satellites), 4)
        self.assertEqual(parsed.data.satellites[0].prn, "05")

    def test_validate_wtraw_sentence_bytes(self) -> None:
        """验证 WTRAW 使用原始字节校验时可以通过。"""

        sentence = bytes.fromhex(
            "24 57 54 52 41 57 2C EF 56 6B 00 D7 FF 07 08 00 00 00 00 FE FF F4 56 6A 00 D7 FF "
            "07 08 00 00 00 00 FD FF F9 56 69 00 D7 FF 07 08 00 00 00 00 FD FF FE 56 69 00 D7 "
            "FF 07 08 00 00 00 00 FD FF 03 57 6A 00 D7 FF 07 08 00 00 00 00 FD FF 08 57 6A 00 D6 "
            "FF 07 08 00 00 00 00 FD FF 0D 57 6A 00 D6 FF 07 08 00 00 00 00 FD FF 12 57 6A 00 D6 "
            "FF 07 08 00 00 00 00 FE FF 17 57 6A 00 D6 FF 07 08 00 00 00 00 FE FF 1C 57 6A 00 D7 "
            "FF 07 08 00 00 00 00 FE FF 76 FF 3D FF B3 FF 8E 02 DE 12 00 00 00 00 00 00 00 00 00 "
            "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0F 27 0F 27 00 "
            "00 00 00 00 00 00 00 00 00 00 00 00 00 2A 32 45"
        )
        self.assertTrue(validate_wtraw_sentence_bytes(sentence))

    def test_parse_wtrtk_sentence(self) -> None:
        """验证 WTRTK 报文解析结果。"""

        body = (
            "WTRTK,0.100,0.200,0.300,0.400,15.44,-6.31,-0.62,4,-217.680,-149.553,-170.513,"
            "0,--,1,0.0,26.6,0,0,2304.75050262,N,11415.23244216,E,0.00,0.62,69.73"
        )
        sentence = f"${body}*{calculate_nmea_checksum(body)}"
        parsed = parse_nmea_sentence(sentence)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.sentence_id, "WTRTK")
        self.assertEqual(parsed.data.positioning_status, 4)
        self.assertEqual(parsed.data.rtk_mode, 0)
        self.assertIsNone(parsed.data.gps_heading)
        self.assertEqual(parsed.data.alignment_flag, 1)
        self.assertAlmostEqual(parsed.data.latitude, 23.079175043666665)
        self.assertAlmostEqual(parsed.data.longitude, 114.253874036)
        self.assertEqual(parsed.data.speed_kmh, 0.0)
        self.assertEqual(parsed.data.heading_deg, 0.62)
        self.assertEqual(parsed.data.altitude, 69.73)

    def test_truncated_sentences_do_not_raise_index_error(self) -> None:
        """被截断的报文不应抛 IndexError。

        回归用例：早期 parse_rmc/parse_gga/parse_gsv 直接下标取字段，
        串口噪声造成的短报文会抛 IndexError，而 runtime 只捕获 ValueError，
        结果是 CLI 直接崩溃、Web 端实时线程静默死亡。
        """

        for body in ("GNRMC,,V", "GNGGA,1", "GPGSA,A", "GPGSV,1", "GNRMC"):
            with self.subTest(body=body):
                sentence = f"${body}*{calculate_nmea_checksum(body)}"
                try:
                    parse_nmea_sentence(sentence)
                except ValueError:
                    # GSV 这类结构性字段缺失会显式抛 ValueError，
                    # runtime 会把它归到 parse 警告，属于预期行为。
                    pass
                except Exception as exc:  # pragma: no cover - 仅失败时执行
                    self.fail(f"{sentence} 抛出了非 ValueError 异常: {type(exc).__name__}: {exc}")

    def test_wtrtk_does_not_clobber_gsa_fix_type(self) -> None:
        """WTRTK 的定位状态码写入独立的 wt_fix_type，不能覆盖 GSA 的 fix_type。

        两者的枚举含义不同（GSA 是 1/2/3，WTRTK 是 0/1/2/4/5），
        且 WTRTK 频率远高于 GSA，共用同一字段时实际总是 WTRTK 覆盖掉 GSA。
        """

        wtrtk_body = (
            "WTRTK,0.100,0.200,0.300,0.400,15.44,-6.31,-0.62,4,-217.680,-149.553,-170.513,"
            "0,--,1,0.0,26.6,0,0,2304.75050262,N,11415.23244216,E,0.00,0.62,69.73"
        )

        state_store = RTKStateStore()
        state_store.update(parse_nmea_sentence("$GPGSA,A,3,,,,,,,,,,,,,2.5,1.3,2.1*34"))
        state_store.update(
            parse_nmea_sentence(f"${wtrtk_body}*{calculate_nmea_checksum(wtrtk_body)}")
        )
        state = state_store.snapshot()

        self.assertEqual(state.fix_type, 3, "GSA 的 fix_type 应保持为 3（三维定位）")
        self.assertEqual(state.wt_fix_type, 4, "WTRTK 的固定解应写入 wt_fix_type")
        self.assertIn("wt_fix_type", state.to_dict())

    def test_state_store_from_fixture(self) -> None:
        """验证样本文件可以聚合出未定位状态。"""

        state_store = RTKStateStore()
        for line in FIXTURE_PATH.read_text(encoding="utf-8").splitlines():
            parsed = parse_nmea_sentence(line)
            if parsed is None:
                continue
            state_store.update(parsed)
        state = state_store.snapshot()
        self.assertFalse(state.fix_valid)
        self.assertEqual(state.fix_quality, 0)
        self.assertEqual(state.fix_type, 1)
        self.assertIsNone(state.latitude)
        self.assertIsNone(state.longitude)
        self.assertEqual(state.utc_date, "2026-07-03")


if __name__ == "__main__":
    unittest.main()
