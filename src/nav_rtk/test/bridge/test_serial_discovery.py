from pathlib import Path

import pytest

from nav_rtk.bridge.serial_discovery import list_rtk_serial_ports, require_explicit_port


def test_lists_usb_and_board_uart_without_chip_identification_or_open(tmp_path, monkeypatch):
    # Symlinks to a real character device avoid root-only mknod in this test.
    for name in ("ttyUSB10", "ttyTHS1", "ttyUSB2", "ttyACM0", "ttyS0", "ttyUSBbackup"):
        (tmp_path / name).symlink_to("/dev/null")
    (tmp_path / "ttyUSB3").touch()  # Ordinary files are not devices.
    (tmp_path / "ttyTHS2").symlink_to(tmp_path / "missing")
    def forbidden(*args, **kwargs):
        raise AssertionError("Search must not open any device")
    monkeypatch.setattr(Path, "open", forbidden)
    assert list_rtk_serial_ports(tmp_path) == [
        str(tmp_path / name) for name in ("ttyTHS1", "ttyUSB2", "ttyUSB10")]


def test_empty_list_and_missing_directory(tmp_path):
    assert list_rtk_serial_ports(tmp_path) == []
    with pytest.raises(FileNotFoundError):
        list_rtk_serial_ports(tmp_path / "missing")


@pytest.mark.parametrize("value", ["", " ", None, "auto", "AUTO", "autodetect", "自动检测", "ttyUSB0"])
def test_automatic_or_relative_paths_are_rejected(value):
    with pytest.raises(ValueError, match="搜索并选择串口"):
        require_explicit_port(value)


def test_explicit_path_never_falls_back_even_if_it_disappears():
    assert require_explicit_port(" /dev/ttyTHS1 ") == "/dev/ttyTHS1"
    assert require_explicit_port("/dev/ttyUSB99999") == "/dev/ttyUSB99999"
    assert require_explicit_port("/dev/pts/7") == "/dev/pts/7"
