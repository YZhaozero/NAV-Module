"""Private map configuration is parsed as data and shared by launch paths."""

import json
import os
from pathlib import Path
import subprocess
import sys

from nav_app.map_environment import load_map_environment
from nav_app.outdoor_service import MapServices


def test_python_loader_and_map_services_read_private_file(tmp_path, monkeypatch):
    marker = tmp_path / "must-not-exist"
    config = tmp_path / "maps.env"
    config.write_text(
        "\ufeffNAV_BAIDU_BROWSER_AK='browser-example'\r\n"
        "NAV_BAIDU_SERVER_AK=server-example\r\n"
        "NAV_BAIDU_SERVER_SK=private-example\r\n"
        "NAV_MAP_MODE=direct\r\n"
        f"PATH=$(touch {marker})\r\n",
        encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_CONFIG", str(config))
    values = load_map_environment()
    assert values["NAV_BAIDU_BROWSER_AK"] == "browser-example"
    assert "PATH" not in values
    services = MapServices()
    assert services.browser_key == "browser-example"
    assert services.server_key == "server-example"
    assert services.signing_key == "private-example"
    assert services.map_mode == "direct"
    assert not marker.exists()


def test_bash_loader_treats_values_as_data(tmp_path):
    root = Path(__file__).resolve().parents[3]
    marker = tmp_path / "must-not-exist"
    config = tmp_path / "maps.env"
    values = {
        "NAV_BAIDU_BROWSER_AK": "browser-example",
        "NAV_BAIDU_SERVER_AK": "server-example",
        "NAV_BAIDU_SERVER_SK": "private-example",
        "NAV_MAP_MODE": "direct",
        "NAV_MAP_TOKEN": f"$(touch {marker}); `touch {marker}`",
    }
    config.write_bytes((
        "\ufeff" + "\r\n".join(f'{name}="{value}"' for name, value in values.items())
        + "\r\nLD_PRELOAD=untrusted").encode())
    probe = tmp_path / "probe.py"
    probe.write_text(
        "import json,os; print(json.dumps({name:os.environ.get(name) for name in "
        + repr([*values, "LD_PRELOAD"]) + "}))")
    environment = dict(os.environ, NAV_MAP_CONFIG=str(config))
    environment.pop("LD_PRELOAD", None)
    result = subprocess.run([
        "bash", "-c", 'source "$1"; exec "$2" "$3"', "test",
        str(root / "scripts/map_environment.sh"), sys.executable, str(probe),
    ], env=environment, text=True, capture_output=True, check=True)
    assert json.loads(result.stdout) == dict(values, LD_PRELOAD=None)
    assert not marker.exists()
