"""Read private map settings as data without executing shell content."""

import os
from pathlib import Path


MAP_SETTINGS = {
    "NAV_BAIDU_BROWSER_AK",
    "NAV_BAIDU_SERVER_AK",
    "NAV_BAIDU_SERVER_SK",
    "NAV_MAP_MODE",
    "NAV_MAP_GATEWAY",
    "NAV_MAP_TOKEN",
    "NAV_OUTDOOR_DATA",
    "NAV_OUTDOOR_CALIBRATION",
}


def load_map_environment(path=None, environ=None):
    environ = os.environ if environ is None else environ
    configured = path or environ.get("NAV_MAP_CONFIG")
    source = Path(configured).expanduser() if configured else Path.home() / ".config/nav/maps.env"
    if not source.is_file():
        return {}
    values = {}
    for line in source.read_text(encoding="utf-8-sig").splitlines():
        setting = line.strip()
        if not setting or setting.startswith("#"):
            continue
        name, separator, value = setting.partition("=")
        name = name.strip()
        if not separator or name not in MAP_SETTINGS:
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] in "\"'" and value[-1] == value[0]:
            value = value[1:-1]
        values[name] = value
    return values
