"""Provider-independent NTRIP account storage; saving never activates a paid service."""
import json
import os
from pathlib import Path
import re
import tempfile


def settings_path():
    return Path.home() / ".config/nav/ntrip.json"


def validate(value):
    if not isinstance(value, dict) or set(value) != {"provider", "host", "port", "mountpoint", "username", "password", "tls", "datum", "auto_start"}:
        raise ValueError("差分账号设置格式无效")
    if value["provider"] not in ("qianxun", "cmcc", "custom"):
        raise ValueError("请选择差分服务商")
    host = value["host"]
    if not isinstance(host, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.:-]{0,252}", host):
        raise ValueError("服务器填写域名或 IP，不含 http://、路径或空格")
    if type(value["port"]) is not int or not 1 <= value["port"] <= 65535:
        raise ValueError("端口须在 1–65535 之间")
    mount = value["mountpoint"]
    if not isinstance(mount, str) or not re.fullmatch(r"[A-Za-z0-9_.~-]{1,128}", mount):
        raise ValueError("挂载点请填写服务商提供的名称，不含斜杠或空格")
    for key in ("username", "password"):
        text = value[key]
        if not isinstance(text, str) or not 1 <= len(text) <= 256 or any(ord(c) < 32 or ord(c) == 127 for c in text):
            raise ValueError("请输入有效的账号和密码")
    if ":" in value["username"]:
        raise ValueError("账号不能包含冒号")
    if type(value["auto_start"]) is not bool or type(value["tls"]) is not bool or value["datum"] not in ("WGS84", "CGCS2000", "ITRF2008", "unknown"):
        raise ValueError("连接或坐标设置无效")
    if host.lower().rstrip(".") == "rtk.ntrip.qxwz.com":
        expected = {8001: "ITRF2008", 8002: "WGS84", 8003: "CGCS2000"}.get(value["port"])
        if expected and value["datum"] != expected:
            raise ValueError(f"千寻此端口对应 {expected}，请核对服务坐标系")
    return dict(value)


def load(path=None):
    path = Path(path) if path else settings_path()
    if not path.exists():
        return None
    if path.is_symlink():
        raise ValueError("账号文件路径异常，未读取")
    if os.name == "posix" and path.stat().st_mode & 0o077:
        raise ValueError("账号文件权限不正确，请重新保存设置")
    try:
        return validate(json.loads(path.read_text(encoding="utf-8")))
    except (OSError, ValueError) as exc:
        raise ValueError("账号文件无法读取，请重新填写并保存") from exc


def save(value, path=None):
    value = validate(value)
    path = Path(path) if path else settings_path()
    if path.is_symlink():
        raise ValueError("账号文件路径异常，未修改")
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".ntrip-", dir=path.parent)
    try:
        if os.name == "posix":
            os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def clear(path=None):
    path = Path(path) if path else settings_path()
    if path.is_symlink():
        raise ValueError("账号文件路径异常，未修改")
    path.unlink(missing_ok=True)


def control_path():
    default = f"/run/user/{os.getuid()}" if os.name == "posix" else tempfile.gettempdir()
    return Path(os.environ.get("XDG_RUNTIME_DIR", default)) / "nav-corrections.json"


def set_requested(enabled, path=None):
    path = Path(path) if path else control_path()
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".nav-corrections-", dir=path.parent)
    try:
        if os.name == "posix":
            os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w") as stream:
            json.dump({"enabled": bool(enabled)}, stream)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def requested(config, path=None):
    path = Path(path) if path else control_path()
    try:
        value = json.loads(path.read_text())
        return value.get("enabled") is True
    except FileNotFoundError:
        return bool(config and config.get("auto_start"))
    except (OSError, ValueError, AttributeError):
        return False
