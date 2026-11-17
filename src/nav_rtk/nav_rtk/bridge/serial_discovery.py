"""List serial paths for explicit user selection; never probe or choose a device."""
from pathlib import Path
import re


def list_rtk_serial_ports(dev_root: Path | str = "/dev") -> list[str]:
    """List existing ttyUSB*/ttyTHS* character devices, without opening them."""
    ports = []
    for path in Path(dev_root).iterdir():
        if re.fullmatch(r"tty(?:USB|THS)\d+", path.name) and path.is_char_device():
            ports.append(path)
    return [str(path) for path in sorted(
        ports, key=lambda path: (path.name.rstrip("0123456789"),
                                 int(re.search(r"\d+$", path.name).group())))]


def require_explicit_port(value: object) -> str:
    """Accept explicit paths (including PTYs); reject old automatic settings."""
    port = str(value or "").strip()
    if not port or not Path(port).is_absolute():
        raise ValueError("请先搜索并选择串口，不再支持自动识别")
    return port
