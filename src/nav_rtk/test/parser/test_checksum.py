"""checksum 相关测试。"""

import unittest

from nav_rtk.parser.checksum import (
    calculate_nmea_checksum,
    calculate_nmea_checksum_bytes,
    validate_nmea_sentence,
    validate_nmea_sentence_bytes,
)


class ChecksumTestCase(unittest.TestCase):
    """验证 NMEA checksum 逻辑。"""

    def test_calculate_nmea_checksum(self) -> None:
        """验证 checksum 计算结果。"""

        body = "GNRMC,074630.100,V,,,,,,,030726,,E,N,V"
        self.assertEqual(calculate_nmea_checksum(body), "6B")

    def test_validate_nmea_sentence_success(self) -> None:
        """验证合法报文可以通过校验。"""

        sentence = "$GNGGA,074630.100,,,,,0,00,99.99,,,,,,*4F"
        self.assertTrue(validate_nmea_sentence(sentence))

    def test_validate_nmea_sentence_failure(self) -> None:
        """验证错误 checksum 会被识别。"""

        sentence = "$GNGGA,074630.100,,,,,0,00,99.99,,,,,,*00"
        self.assertFalse(validate_nmea_sentence(sentence))

    def test_validate_nmea_sentence_bytes_success(self) -> None:
        """验证带二进制负载的原始字节报文也能通过校验。"""

        body = b"WTRAW,\xef\x00A"
        checksum = calculate_nmea_checksum_bytes(body)
        sentence = b"$" + body + b"*" + checksum.encode("ascii")
        self.assertTrue(validate_nmea_sentence_bytes(sentence))


if __name__ == "__main__":
    unittest.main()
