"""Startup choices must survive upgrades and must not stop an active capture."""
import configparser
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace
import time

import pytest
from nav_app.features import startup


def script(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).resolve().parents[3]/"scripts"/(name+".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def service(tmp_path, monkeypatch):
    monkeypatch.setattr(startup, "preference_path", lambda: tmp_path/"startup.json")
    monkeypatch.setattr(startup, "desktop_login_ready", lambda: True)
    state = SimpleNamespace(enabled=True, available=True, fail=False, calls=[])
    def run(*args):
        state.calls.append(args)
        if args[0] == "is-enabled":
            return SimpleNamespace(returncode=0 if state.enabled else 1,
                stdout=("enabled" if state.enabled else "disabled") if state.available else "not-found")
        if state.fail:
            return SimpleNamespace(returncode=1)
        assert args[0] in ("enable", "disable"), "Changing startup must not stop or restart the APP"
        assert args[1:] == (startup.SERVICE,), "No --now or shell commands allowed"
        state.enabled = args[0] == "enable"
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(startup, "run", run)
    return state


def test_default_enabled_then_disable_survives_upgrade_config(service, tmp_path):
    installer = script("device_service")
    config = tmp_path/"install.json"
    config.write_text(json.dumps({"autostart": True, "desktop_autologin": True}))
    assert installer.install_options(config) == (True, True)
    assert startup.set_enabled(False)["enabled"] is False
    assert startup.preferred_enabled() is False
    assert installer.install_options(config) == (False, True)
    assert installer.install_options(config, "on") == (True, True)
    assert startup.set_enabled(True)["enabled"] is True


def test_errors_never_persist_success(service):
    service.fail = True
    with pytest.raises(RuntimeError, match="未保存"):
        startup.set_enabled(False)
    assert not startup.preference_path().exists()
    service.available = False
    with pytest.raises(RuntimeError, match="尚未安装"):
        startup.set_enabled(True)
    assert len([x for x in service.calls if x[0] == "enable"]) == 0


def test_restart_requires_process_to_belong_to_device_service(monkeypatch):
    calls = []
    monkeypatch.setattr(startup, "run", lambda *args: calls.append(args))
    with pytest.raises(RuntimeError, match="不是由设备服务启动"):
        startup.restart_service({})
    assert not calls


def test_restart_checks_active_service_then_queues_nonblocking_restart(monkeypatch):
    calls = []

    def run(*args):
        calls.append(args)
        if args[0] == "is-active":
            return SimpleNamespace(returncode=0, stdout="active\n")
        return SimpleNamespace(returncode=0, stdout="")

    monkeypatch.setattr(startup, "run", run)
    assert startup.restart_service({"INVOCATION_ID": "test-service"}) == {
        "requested": True, "service": startup.SERVICE}
    assert calls == [
        ("is-active", startup.SERVICE),
        ("--no-block", "restart", startup.SERVICE),
    ]


def test_restart_rejects_inactive_service(monkeypatch):
    calls = []

    def run(*args):
        calls.append(args)
        return SimpleNamespace(returncode=3, stdout="inactive\n")

    monkeypatch.setattr(startup, "run", run)
    with pytest.raises(RuntimeError, match="当前未运行"):
        startup.restart_service({"INVOCATION_ID": "test-service"})
    assert calls == [("is-active", startup.SERVICE)]


@pytest.mark.parametrize("config", [{"autostart":"false"}, {"auto_start":True}, []])
def test_install_rejects_misspelled_or_non_boolean_options(service, tmp_path, config):
    path = tmp_path/"install.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        script("device_service").install_options(path)


def test_gdm_configuration_preserves_comments_and_other_sections(tmp_path):
    helper = script("desktop_autologin")
    original = "# vendor note\n[daemon]\nAutomaticLogin=old\nAutomaticLoginEnable=false\nTimedLoginEnable=true\nWaylandEnable=true\nDefaultSession=ubuntu\n[security]\nDisallowTCP=true\n"
    updated = helper.configured_text(original, "navrobot")
    assert helper.configured_text(updated, "navrobot") == updated
    assert "# vendor note" in updated and "DefaultSession=ubuntu" in updated
    parser = configparser.ConfigParser()
    parser.read_string(updated)
    assert parser.getboolean("security", "DisallowTCP")
    assert not parser.getboolean("daemon", "TimedLoginEnable")
    path = tmp_path/"custom.conf"
    path.write_text(updated)
    assert startup.desktop_login_ready(path, "navrobot")
    assert not startup.desktop_login_ready(path, "otheruser")
    path.write_text(original)
    assert not startup.desktop_login_ready(path, "navrobot")


def test_install_launchers_uses_localized_desktop_and_marks_it_trusted(tmp_path, monkeypatch):
    installer = script("device_service")
    desktop = tmp_path/"桌面"
    calls = []

    def run(args, **kwargs):
        calls.append((args, kwargs))
        if args[:2] == ["xdg-user-dir", "DESKTOP"]:
            return SimpleNamespace(returncode=0, stdout=str(desktop) + "\n", stderr="")
        assert args[:2] == ["gio", "set"]
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(installer, "command", run)
    result = installer.install_launchers(tmp_path)
    menu = tmp_path/".local/share/applications/nav-app.desktop"
    shortcut = desktop/"nav-app.desktop"
    assert result == {"application_menu": str(menu), "desktop": str(shortcut),
                      "desktop_trusted": True}
    assert menu.read_text(encoding="utf-8") == shortcut.read_text(encoding="utf-8")
    assert "Name[zh_CN]=识动 NAV" in shortcut.read_text(encoding="utf-8")
    assert 'Exec=/bin/sh -c "systemctl --user reset-failed nav-app.service; exec systemctl --user start nav-app.service"' in shortcut.read_text(encoding="utf-8")
    assert os.stat(menu).st_mode & 0o777 == 0o644
    assert os.stat(shortcut).st_mode & 0o777 == 0o755
    assert calls[-1][0] == ["gio", "set", str(shortcut), "metadata::trusted", "true"]


def test_desktop_directory_falls_back_without_xdg_support(tmp_path, monkeypatch):
    installer = script("device_service")
    localized = tmp_path/"桌面"
    localized.mkdir()
    monkeypatch.setattr(installer, "command", lambda *_args, **_kwargs:
                        SimpleNamespace(returncode=1, stdout="", stderr="missing"))
    assert installer.desktop_directory(tmp_path) == localized


@pytest.mark.parametrize("existing_service", [False, True])
def test_release_install_only_stops_an_existing_service(tmp_path, monkeypatch, existing_service):
    installer = script("device_service")
    release = tmp_path/"release"
    (release/"deployment").mkdir(parents=True)
    (release/"deployment/nav-app.service").write_text("[Service]\n")
    calls = []

    def run(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(Path, "home", lambda: tmp_path)
    monkeypatch.setattr(installer, "check", lambda _release: {
        "errors": [], "service_available": existing_service})
    monkeypatch.setattr(installer, "command", run)
    monkeypatch.setattr(installer, "install_launchers", lambda: None)
    monkeypatch.setattr(installer, "preserve_device_profiles", lambda: None)
    monkeypatch.setattr(startup, "set_enabled", lambda enabled: None)
    installer.install(release, tmp_path/"state", autostart=True)
    assert (["systemctl", "--user", "stop", "nav-app.service"] in calls) == existing_service
    assert (tmp_path/"nav-current").resolve() == release
    assert ["systemctl", "--user", "restart", "nav-app.service"] in calls


