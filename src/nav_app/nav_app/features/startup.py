"""Persistent user-service startup settings; never interrupt a running task."""
import configparser
import getpass
import json
import os
from pathlib import Path
import subprocess


SERVICE = "nav-app.service"


def run(*args):
    try:
        return subprocess.run(["systemctl", "--user", *args], text=True,
                              capture_output=True, timeout=8)
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError("暂时无法访问启动服务，请稍后重试") from exc


def preference_path():
    return Path.home() / ".config/nav/startup.json"


def preferred_enabled():
    path = preference_path()
    if not path.exists():
        return True
    value = json.loads(path.read_text(encoding="utf-8")).get("autostart")
    if type(value) is not bool:
        raise ValueError("开机启动设置无法读取，请重新保存设置")
    return value


def save_preference(enabled):
    path = preference_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump({"autostart": enabled}, stream)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def desktop_login_ready(path=None, username=None):
    parser = configparser.ConfigParser(interpolation=None, strict=False)
    try:
        parser.read(path or "/etc/gdm3/custom.conf")
        return (parser.getboolean("daemon", "AutomaticLoginEnable", fallback=False)
                and parser.get("daemon", "AutomaticLogin", fallback="").strip() == (username or getpass.getuser())
                and not parser.getboolean("daemon", "WaylandEnable", fallback=True))
    except (OSError, ValueError, configparser.Error):
        return False


def status():
    result = run("is-enabled", SERVICE)
    state = result.stdout.strip()
    if state not in ("enabled", "disabled"):
        raise RuntimeError("开机启动服务尚未安装或不可用，请先完成设备安装")
    return {"enabled": state == "enabled", "desktop_login_ready": desktop_login_ready()}


def restart_service(environ=None):
    """Queue a restart only when this process belongs to the installed service."""
    environment = os.environ if environ is None else environ
    if not environment.get("INVOCATION_ID"):
        raise RuntimeError("当前软件不是由设备服务启动，不能安全重启；请从桌面入口启动")
    active = run("is-active", SERVICE)
    if active.returncode or active.stdout.strip() != "active":
        raise RuntimeError("设备服务当前未运行，不能从软件内重启")
    result = run("--no-block", "restart", SERVICE)
    if result.returncode:
        raise RuntimeError("软件重启请求未提交，请检查设备服务")
    return {"requested": True, "service": SERVICE}


def set_enabled(enabled):
    if type(enabled) is not bool:
        raise ValueError("Invalid startup setting")
    status()  # Reject missing/masked services instead of reporting a false success.
    result = run("enable" if enabled else "disable", SERVICE)
    if result.returncode:
        raise RuntimeError("开机启动设置未保存，请检查设备服务")
    actual = status()
    if actual["enabled"] != enabled:
        raise RuntimeError("开机启动设置未生效，请重试")
    try:
        save_preference(enabled)
    except OSError as exc:
        raise RuntimeError("启动服务已更新，但设置文件保存失败，请重试") from exc
    return actual
