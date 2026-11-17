import json
from pathlib import Path
import stat
import time
import pytest
from nav_app.device_network import Manager, NetworkError, AP_UUID, profile_text, split_nm

OLD = "7675b3a5-ed41-4b22-9218-b2766eceec78"


class NM:
    def __init__(self):
        self.calls = []
        self.current = OLD
        self.mode = "wifi"
        self.ap = "yes"
        self.fail_station = False
        self.known = {OLD}
        self.paths = {}
        self.routes = []
        self.auto = True
        self.priority = 0
        self.scan_value = 100
        self.scan_requested = False
        self.scan_reads = 0

    def __call__(self, *args):
        self.calls.append(args)
        if args[0] == 'busctl':
            self.scan_reads += 1
            return f"x {self.scan_value}"
        if 'rescan' in args:
            self.scan_requested = True
        if 'list' in args and 'wifi' in args:
            return 'gswifi1:59:WPA2' if self.scan_value > 100 else 'old-cache:1:WPA2'
        if 'UUID,TYPE,NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY' in args:
            return f"{OLD}:802-11-wireless:Old network:{'yes' if self.auto else 'no'}:{self.priority}"
        if 'modify' in args:
            self.auto = args[args.index('connection.autoconnect')+1] == 'yes'
            if 'connection.autoconnect-priority' in args:
                self.priority = int(args[-1])
        if args[:2] == ("ip", "-j"):
            return json.dumps(self.routes)
        if args[:2] == ("nmcli", "-g") and "device" in args:
            return {"GENERAL.STATE": "100 (connected)" if self.mode != "disconnected" else "30 (disconnected)",
                    "GENERAL.CON-UUID": self.current, "IP4.ADDRESS": "10.42.0.1/24" if self.mode == "hotspot" else "192.168.2.5/24" if self.mode == "wifi" else "",
                    "GENERAL.CONNECTION": "Test Wi-Fi", "WIFI-PROPERTIES.AP": self.ap, "GENERAL.TYPE": "wifi",
                    "GENERAL.DBUS-PATH": "/org/freedesktop/NetworkManager/Devices/5"}[args[2]]
        if args[1:3] == ("-g", "UUID"):
            return "\n".join(self.known)
        if "802-11-wireless.mode" in args:
            return "ap" if args[-1] == AP_UUID else "infrastructure"
        if "connection.type" in args:
            return "802-11-wireless"
        if "load" in args:
            path = Path(args[-1])
            ident = next(l[5:] for l in path.read_text().splitlines() if l.startswith("uuid="))
            self.known.add(ident)
            self.paths[ident] = path
        if "up" in args:
            ident = args[args.index("uuid") + 1]
            if self.fail_station and ident not in (AP_UUID, OLD):
                self.mode = "disconnected"
                raise NetworkError("test failure")
            self.current, self.mode = ident, "hotspot" if ident == AP_UUID else "wifi"
        if "delete" in args:
            self.known.remove(args[-1])
        return ""


def manager(tmp_path):
    nm = NM()
    now = [0.]
    service = Manager("wlan0", tmp_path / "settings.json", tmp_path / "profiles", nm, lambda: now[0])
    return service, nm, now


def test_boot_fallback_waits_and_does_not_require_internet(tmp_path):
    service, nm, now = manager(tmp_path)
    for t in (0, 60, 200):
        now[0] = t
        service.poll()
    assert not any("up" in c for c in nm.calls)
    nm.mode = "disconnected"
    now[0] = 259
    service.poll()
    assert nm.current == OLD
    now[0] = 261
    service.poll()
    assert nm.current == AP_UUID
    assert service.snapshot()["active_hotspot"]["ssid"] == "NAV Demo"
    now[0] = 800
    service.poll()
    assert sum("up" in c for c in nm.calls) == 1  # Does not disrupt a phone on the AP.


def test_bad_password_restores_ap_and_removes_only_failed_candidate(tmp_path):
    service, nm, now = manager(tmp_path)
    service.activate_hotspot()
    nm.fail_station = True
    with pytest.raises(NetworkError, match="恢复"):
        service.connect(dict(ssid="badwifi", password="badpassword"))
    assert nm.current == AP_UUID and nm.known == {OLD, AP_UUID}
    assert len(list(service.profiles.glob("*.nmconnection"))) == 1
    assert all("badpassword" not in " ".join(call) for call in nm.calls)
    assert stat.S_IMODE(next(service.profiles.glob("*.nmconnection")).stat().st_mode) == 0o600


def test_conflicting_lidar_route_is_preserved(tmp_path):
    service, nm, now = manager(tmp_path)
    nm.routes = [dict(dst="10.42.0.0/16", dev="eth0")]
    with pytest.raises(NetworkError, match="冲突"):
        service.activate_hotspot()
    assert not service.profiles.exists() and nm.current == OLD


def test_reconnect_saved_profile_does_not_need_password(tmp_path):
    service, nm, now = manager(tmp_path)
    service.activate_hotspot()
    service.connect(dict(uuid=OLD))
    assert nm.current == OLD and service.snapshot()["mode"] == "wifi"


def test_settings_disable_fallback_and_reject_invalid_types(tmp_path):
    service, nm, now = manager(tmp_path)
    service.settings["fallback"] = False
    nm.mode = "disconnected"
    now[0] = 600
    service.poll()
    assert nm.current == OLD
    with pytest.raises(NetworkError):
        service.validate_settings({**service.settings, "fallback": "false"})
    with pytest.raises(NetworkError):
        service.connect(dict(ssid="bad\nssid", password="12345678"))
    assert not service.profiles.exists()


def test_keyfile_escaping_and_nmcli_escaped_names():
    value = profile_text("wlan0", OLD, " Lab\\Wi-Fi ", " a;#\\12345 ")
    assert "ssid=\\sLab\\\\Wi-Fi\\s" in value
    assert "psk=\\sa;#\\\\12345\\s" in value
    assert split_nm(r"lab\:room:90:WPA2") == ["lab:room", "90", "WPA2"]


def test_alien_profile_not_overwritten(tmp_path):
    service, nm, now = manager(tmp_path)
    service.profiles.mkdir()
    p = service.profiles / f"nav-{AP_UUID}.nmconnection"
    p.write_text("keep")
    with pytest.raises(NetworkError, match="保留"):
        service.activate_hotspot()
    assert p.read_text() == "keep"


def test_scan_waits_for_completion_instead_of_reading_old_list(tmp_path):
    service, nm, _ = manager(tmp_path)
    waits = []
    def complete_after_wait(seconds):
        waits.append(seconds)
        nm.scan_value = 101
    service.sleep = complete_after_wait
    service.scan()
    assert waits and nm.scan_requested
    assert service.snapshot()['scan_fresh'] is True
    assert service.snapshot()['networks'][0]['ssid'] == 'gswifi1'
    assert not any('up' in c for c in nm.calls)


def test_scan_timeout_and_hotspot_are_explicit_cache_states(tmp_path):
    service, nm, _ = manager(tmp_path)
    service.sleep = lambda _: None
    service.scan()
    assert not service.snapshot()['scan_fresh']
    assert '缓存' in service.snapshot()['message']
    nm.mode='hotspot';nm.current=AP_UUID;nm.calls.clear()
    service.scan()
    assert not any('rescan' in c for c in nm.calls)
    assert '上次扫描' in service.snapshot()['message']


def test_preferences_never_disconnect_and_can_be_reenabled(tmp_path):
    service, nm, _ = manager(tmp_path)
    service.preference(dict(uuid=OLD, autoconnect=False))
    assert service.snapshot()['saved'][0]['autoconnect'] is False
    service.preference(dict(uuid=OLD, autoconnect=True, preferred=True))
    assert nm.auto and nm.priority == 10 and nm.current == OLD
    assert not any('up' in c or 'down' in c or 'delete' in c for c in nm.calls)
    with pytest.raises(NetworkError):
        service.preference(dict(uuid=AP_UUID, autoconnect=False))


def test_command_locale_preserves_chinese_wifi_names(monkeypatch):
    from nav_app import device_network
    from types import SimpleNamespace
    calls=[]
    def execute(*args, **kwargs):
        calls.append(kwargs)
        return SimpleNamespace(returncode=0, stdout='我的热点:70:WPA2\n')
    monkeypatch.setattr(device_network.subprocess,'run',execute)
    assert device_network.run('nmcli').startswith('我的热点')
    assert calls[0]['env']['LC_ALL']=='C.UTF-8'
