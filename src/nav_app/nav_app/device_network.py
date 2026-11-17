"""Single-adapter Wi-Fi provisioning, run by the restricted local system service.

New profiles are service-owned; saved Wi-Fi auto-connect preferences change only
on explicit selection. LiDAR links are untouched.
Internet reachability is deliberately not a condition for keeping a LAN connection.
"""
import copy
import configparser
import ipaddress
import json
import os
from pathlib import Path
import re
import socket
import socketserver
import struct
import subprocess
import tempfile
import threading
import time
import uuid

SOCKET = "/run/nav-network/control.sock"
AP_UUID = "7857c239-bb60-49e1-b03e-15c742ef90f9"
MARKER = "# Managed by nav-network\n"
DEFAULTS = dict(fallback=True, grace_seconds=60, ssid="NAV Demo", password="12345678")


class NetworkError(RuntimeError):
    pass


def atomic_write(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=".nav-", dir=path.parent)
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def run(*args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=40,
                                env={**os.environ, "LC_ALL": "C.UTF-8"})
    except (OSError, subprocess.SubprocessError) as exc:
        raise NetworkError("网络服务无响应，请稍后重试") from exc
    if result.returncode:
        # Never return command lines/stderr: they can include SSIDs or credentials.
        raise NetworkError("网络操作未成功，请检查密码、信号和网络类型")
    return result.stdout.strip()


def keyvalue(value):
    """GLib keyfile string escaping (including leading/trailing whitespace)."""
    return value.replace("\\", "\\\\").replace(" ", "\\s").replace("\t", "\\t")


def validate_wifi(ssid, password, secured=True):
    if not isinstance(ssid, str) or not 1 <= len(ssid.encode("utf-8")) <= 32:
        raise NetworkError("Wi-Fi 名称须为 1–32 字节")
    if any(ord(c) < 32 or ord(c) == 127 for c in ssid):
        raise NetworkError("Wi-Fi 名称含无效字符")
    if not isinstance(password, str):
        raise NetworkError("Wi-Fi 密码格式无效")
    if secured and not (8 <= len(password) <= 63 and all(32 <= ord(c) <= 126 for c in password)
                        or re.fullmatch(r"[a-fA-F0-9]{64}", password)):
        raise NetworkError("Wi-Fi 密码须为 8–63 位英文、数字或符号，或 64 位十六进制密钥")


def profile_text(interface, ident, ssid, password, ap=False, hidden=False):
    validate_wifi(ssid, password, bool(password) or ap)
    text = (MARKER + "[connection]\n" + f"id={keyvalue(ssid)}\nuuid={ident}\n"
            + f"type=wifi\ninterface-name={interface}\nautoconnect={'false' if ap else 'true'}\n"
            + "autoconnect-priority=10\n\n[wifi]\n"
            + f"mode={'ap' if ap else 'infrastructure'}\nssid={keyvalue(ssid)}\n"
            + ("band=bg\n" if ap else f"hidden={'true' if hidden else 'false'}\n"))
    if password:
        text += f"\n[wifi-security]\nkey-mgmt=wpa-psk\npsk={keyvalue(password)}\n"
    text += ("\n[ipv4]\nmethod=shared\naddress1=10.42.0.1/24\nnever-default=true\n" if ap
             else "\n[ipv4]\nmethod=auto\n")
    return text + "\n[ipv6]\nmethod=disabled\n"


def split_nm(line):
    fields, part, escape = [], "", False
    for char in line:
        if escape:
            part += char
            escape = False
        elif char == "\\":
            escape = True
        elif char == ":":
            fields.append(part)
            part = ""
        else:
            part += char
    fields.append(part)
    return fields


class Manager:
    def __init__(self, interface, config="/etc/nav/network.json",
                 profiles="/etc/NetworkManager/system-connections", runner=run, clock=time.monotonic,
                 sleep=time.sleep):
        if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9_.-]{0,14}", interface):
            raise NetworkError("无线网卡名称无效")
        self.interface, self.config_path, self.profiles = interface, Path(config), Path(profiles)
        self.run, self.clock = runner, clock
        self.sleep = sleep
        self.settings = dict(DEFAULTS)
        if self.config_path.exists():
            self.settings = self.validate_settings(json.loads(self.config_path.read_text()))
        self.lock = threading.RLock()
        self.operation = threading.Lock()
        self.offline_since = clock()
        self.last_auto_attempt = -1e9
        self.state = dict(mode="unknown", connection="", address="", supported=False,
                          busy=False, message="正在读取无线网络", networks=[], saved=[])

    @staticmethod
    def validate_settings(value):
        if not isinstance(value, dict) or set(value) != set(DEFAULTS):
            raise NetworkError("热点设置格式无效")
        if type(value["fallback"]) is not bool or type(value["grace_seconds"]) is not int or not 30 <= value["grace_seconds"] <= 300:
            raise NetworkError("热点等待时间须为 30–300 秒")
        validate_wifi(value["ssid"], value["password"])
        return dict(value)

    def field(self, name):
        return self.run("nmcli", "-g", name, "device", "show", self.interface)

    def read_link(self):
        supported = self.field("WIFI-PROPERTIES.AP") == "yes"
        state = self.field("GENERAL.STATE")
        current = self.field("GENERAL.CON-UUID")
        address = self.field("IP4.ADDRESS").splitlines()
        address = address[0].split("/")[0] if address else ""
        mode = "disconnected"
        connection = ""
        if state.startswith("100"):
            connection = self.field("GENERAL.CONNECTION")
            # Also respect an AP started manually outside this service.
            wifi_mode = self.run("nmcli", "-g", "802-11-wireless.mode", "connection", "show", "uuid", current)
            mode = "hotspot" if wifi_mode == "ap" else "wifi"
        elif state.startswith(("40", "50", "60", "70", "80", "90")):
            mode = "connecting"
        ap = {}
        path = self.profiles / f"nav-{AP_UUID}.nmconnection"
        if mode == "hotspot" and current == AP_UUID and path.is_file():
            parser = configparser.ConfigParser(interpolation=None)
            parser.read(path, encoding="utf-8")
            def decode(value):
                return re.sub(r"\\([snt\\])", lambda m: {"s": " ", "n": "\n", "t": "\t", "\\": "\\"}[m[1]], value)
            ap = dict(ssid=decode(parser.get("wifi", "ssid")), password=decode(parser.get("wifi-security", "psk")))
        return dict(mode=mode, connection=connection, address=address, supported=supported, current=current, active_hotspot=ap)

    def snapshot(self):
        with self.lock:
            return {**copy.deepcopy(self.state), "settings": dict(self.settings), "interface": self.interface}

    def poll(self):
        if not self.operation.acquire(False):
            return
        try:
            link = self.read_link()
            with self.lock:
                if self.state["mode"] == "unknown" or self.state["mode"] != link["mode"]:
                    self.state["message"] = {"wifi": "Wi-Fi 已连接，本地控制可用", "hotspot": "设备热点已开启",
                                             "disconnected": "正在等待 Wi-Fi 连接", "connecting": "正在连接 Wi-Fi"}.get(link["mode"], "正在读取网络")
                self.state.update(link)
            now = self.clock()
            if link["mode"] in ("wifi", "hotspot"):
                self.offline_since = now
                return
            # A permanently stuck DHCP/association also eventually falls back.
            if (self.settings["fallback"] and link["supported"]
                    and now - self.offline_since >= self.settings["grace_seconds"]
                    and now - self.last_auto_attempt >= 60):
                self.last_auto_attempt = now
                self.set_message("正在开启设备热点", True)
                self.activate_hotspot()
                self.set_message("热点已开启，可连接后设置 Wi-Fi")
        except NetworkError as exc:
            self.set_message(str(exc))
        finally:
            self.operation.release()

    def set_message(self, message, busy=False):
        with self.lock:
            self.state.update(message=message, busy=busy)

    def write_profile(self, ident, text):
        path = self.profiles / f"nav-{ident}.nmconnection"
        if path.is_symlink() or path.exists() and not path.read_text().startswith(MARKER):
            raise NetworkError("同名配置不属于导航应用，已保留原配置")
        existing = self.run("nmcli", "-g", "UUID", "connection", "show").splitlines()
        if ident in existing and not path.exists():
            raise NetworkError("网络配置标识冲突，已保留原配置")
        atomic_write(path, text)
        self.run("nmcli", "connection", "load", str(path))
        return path

    def activate(self, ident):
        self.run("nmcli", "--wait", "25", "connection", "up", "uuid", ident, "ifname", self.interface)

    def activate_hotspot(self):
        if self.field("GENERAL.TYPE") != "wifi" or self.field("WIFI-PROPERTIES.AP") != "yes":
            raise NetworkError("无线网卡不支持热点")
        subnet = ipaddress.ip_network("10.42.0.0/24")
        for route in json.loads(self.run("ip", "-j", "-4", "route", "show", "table", "all")):
            dst = route.get("dst", "default")
            if dst == "default" or route.get("dev") == self.interface:
                continue
            try:
                conflict = subnet.overlaps(ipaddress.ip_network(dst, strict=False))
            except ValueError:
                continue
            if conflict:
                raise NetworkError("热点网段与其他接口冲突，请联系维护人员")
        self.write_profile(AP_UUID, profile_text(self.interface, AP_UUID, self.settings["ssid"], self.settings["password"], ap=True))
        self.activate(AP_UUID)
        link = self.read_link()
        if link["mode"] != "hotspot" or link["address"] != "10.42.0.1":
            raise NetworkError("热点未取得地址，请重试")
        with self.lock:
            self.state.update(link)

    def last_scan(self):
        # RequestScan acknowledges scheduling, not completion. LastScan changes
        # only when NetworkManager has received the completed scan.
        path = self.field("GENERAL.DBUS-PATH")
        value = self.run("busctl", "get-property", "org.freedesktop.NetworkManager", path,
                         "org.freedesktop.NetworkManager.Device.Wireless", "LastScan")
        try:
            return int(value.split()[-1])
        except (ValueError, IndexError) as exc:
            raise NetworkError("无法确认扫描完成，请稍后刷新或手动填写 Wi-Fi 名称") from exc

    def saved_networks(self):
        saved = []
        fields = "UUID,TYPE,NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY"
        for line in self.run("nmcli", "-t", "-f", fields, "connection", "show").splitlines():
            parts = split_nm(line)
            if len(parts) == 5 and parts[1] in ("wifi", "802-11-wireless"):
                ident, _, name, auto, priority = parts
                mode = self.run("nmcli", "-g", "802-11-wireless.mode", "connection", "show", "uuid", ident)
                if mode != "ap":
                    saved.append(dict(uuid=ident, name=name, autoconnect=auto == "yes", priority=int(priority)))
        with self.lock:
            self.state["saved"] = saved
        return saved

    def scan(self):
        fresh = False
        hotspot = self.read_link()["mode"] == "hotspot"
        message = "设备热点模式显示上次扫描结果；可手动填写 Wi-Fi 名称"
        if not hotspot:
            try:
                before = self.last_scan()
                self.run("nmcli", "--wait", "8", "device", "wifi", "rescan", "ifname", self.interface)
                for _ in range(24):
                    if self.last_scan() > before:
                        fresh = True
                        break
                    self.sleep(.5)
                message = "附近 Wi-Fi 已重新扫描" if fresh else "扫描尚未完成，当前为缓存列表；请稍后刷新"
            except NetworkError:
                message = "扫描未成功，当前为缓存列表；请稍后刷新或手动填写 Wi-Fi 名称"
        lines = self.run("nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY", "device", "wifi", "list", "ifname", self.interface, "--rescan", "no")
        networks = {}
        for line in lines.splitlines():
            parts = split_nm(line)
            if len(parts) == 3 and parts[0]:
                name, signal, security = parts
                item = dict(ssid=name, signal=int(signal) if signal.isdigit() else 0, security=security)
                if name not in networks or item["signal"] > networks[name]["signal"]:
                    networks[name] = item
        self.saved_networks()
        with self.lock:
            self.state.update(networks=sorted(networks.values(), key=lambda x: -x["signal"]), scan_fresh=fresh)
        self.set_message(message)

    def preference(self, payload):
        try:
            ident = str(uuid.UUID(payload.get("uuid", "")))
        except (ValueError, TypeError, AttributeError) as exc:
            raise NetworkError("请先选择已保存的 Wi-Fi") from exc
        rows = self.saved_networks()
        if not any(row['uuid'] == ident for row in rows):
            raise NetworkError("请选择已保存的 Wi-Fi")
        interface = self.run("nmcli", "-g", "connection.interface-name", "connection", "show", "uuid", ident)
        if interface not in ("", "--", self.interface):
            raise NetworkError("该网络属于其他无线网卡，未修改")
        auto = payload.get('autoconnect')
        preferred = payload.get('preferred', False)
        if type(auto) is not bool or type(preferred) is not bool:
            raise NetworkError("自动连接设置无效")
        command = ["nmcli", "connection", "modify", "uuid", ident, "connection.autoconnect", "yes" if auto else "no"]
        if preferred:
            if not auto:
                raise NetworkError("优先连接需要开启自动连接")
            highest = max((row['priority'] for row in rows if row['uuid'] != ident and row['autoconnect']), default=0)
            if highest >= 999:
                raise NetworkError("已有网络达到最高优先级，请先关闭它的自动连接")
            command += ["connection.autoconnect-priority", str(max(10, highest+1))]
        self.run(*command)
        self.saved_networks()
        self.set_message("已设为优先自动连接；当前连接保持不变" if preferred else
                         "已开启自动连接；当前连接保持不变" if auto else
                         "已关闭此网络的自动连接；当前连接保持不变")

    def connect(self, payload):
        previous = self.read_link()
        created = None
        ident = payload.get("uuid")
        if ident:
            try:
                ident = str(uuid.UUID(ident))
            except (ValueError, TypeError, AttributeError) as exc:
                raise NetworkError("已保存的 Wi-Fi 标识无效") from exc
            kind = self.run("nmcli", "-g", "connection.type", "connection", "show", "uuid", ident)
            mode = self.run("nmcli", "-g", "802-11-wireless.mode", "connection", "show", "uuid", ident)
            if kind != "802-11-wireless" or mode == "ap":
                raise NetworkError("请选择已保存的 Wi-Fi")
        else:
            name, password = payload.get("ssid", ""), payload.get("password", "")
            security = payload.get("security", "wpa")
            if security not in ("wpa", "open") or type(payload.get("hidden", False)) is not bool:
                raise NetworkError("暂支持个人密码 Wi-Fi、开放网络和已保存的网络")
            validate_wifi(name, password, security == "wpa")
            ident = str(uuid.uuid4())
            created = self.write_profile(ident, profile_text(self.interface, ident, name,
                          password if security == "wpa" else "", hidden=payload.get("hidden", False)))
        try:
            self.activate(ident)
            link = self.read_link()
            if link["mode"] != "wifi" or not link["address"]:
                raise NetworkError("Wi-Fi 未分配地址")
            self.offline_since = self.clock()
            with self.lock:
                self.state.update(link)
            self.set_message("Wi-Fi 已连接。请让手机接入同一网络，使用屏幕上的新地址")
        except NetworkError:
            if created:
                try:
                    self.run("nmcli", "connection", "delete", "uuid", ident)
                    created.unlink(missing_ok=True)
                except (NetworkError, OSError):
                    # Failed candidates must not retry forever after reboot.
                    self.run("nmcli", "connection", "modify", "uuid", ident, "connection.autoconnect", "no")
            self.restore(previous)
            raise NetworkError("连接失败，已恢复原连接或设备热点；请核对 Wi-Fi 名称、密码和信号")

    def restore(self, previous):
        if previous["mode"] == "wifi" and previous.get("current"):
            try:
                self.activate(previous["current"])
                with self.lock:
                    self.state.update(self.read_link())
                return
            except NetworkError:
                pass
        self.activate_hotspot()

    def submit(self, action, payload):
        if action not in ("scan", "connect", "hotspot", "settings", "preference"):
            raise NetworkError("不支持的网络操作")
        if not self.operation.acquire(False):
            raise NetworkError("网络操作正在进行，请稍后重试")
        self.set_message("正在处理网络设置…", True)
        def work():
            try:
                # Give the phone HTTP request a chance to receive acknowledgement.
                if action in ("connect", "hotspot"):
                    time.sleep(2)
                if action == "scan":
                    self.scan()
                elif action == "connect":
                    self.connect(payload)
                elif action == "preference":
                    self.preference(payload)
                elif action == "hotspot":
                    previous = self.read_link()
                    try:
                        self.activate_hotspot()
                    except NetworkError:
                        self.restore(previous)
                        raise
                    self.set_message("热点已开启，请连接后打开 10.42.0.1:8000")
                else:
                    settings = self.validate_settings(payload)
                    atomic_write(self.config_path, json.dumps(settings, ensure_ascii=False))
                    with self.lock:
                        self.settings = settings
                    self.offline_since = self.clock()
                    self.set_message("设置已保存；新的热点名称和密码在下次开启热点时生效")
            except (NetworkError, OSError, ValueError):
                # Handler errors carry no arbitrary external output or secrets.
                import sys
                error = sys.exc_info()[1]
                self.set_message(str(error) if isinstance(error, NetworkError) else "设置失败，请检查网络服务")
            finally:
                with self.lock:
                    self.state["busy"] = False
                self.operation.release()
        threading.Thread(target=work, daemon=True, name="nav-network-operation").start()
        return {"accepted": True}


def request(action="status", payload=None, path=SOCKET):
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(3)
            client.connect(path)
            client.sendall(json.dumps(dict(action=action, payload=payload or {})).encode() + b"\n")
            data = b""
            while not data.endswith(b"\n") and len(data) < 65536:
                part = client.recv(4096)
                if not part:
                    break
                data += part
        result = json.loads(data)
    except (OSError, ValueError, AttributeError) as exc:
        raise NetworkError("设备联网服务未就绪，请稍后重试") from exc
    if "error" in result:
        raise NetworkError(result["error"])
    return result


def serve(manager, uid, gid, path=SOCKET):
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.request.settimeout(3)
            peer = struct.unpack("3i", self.request.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
            if peer[1] not in (0, uid):
                return
            try:
                raw = self.rfile.readline(16385)
                if len(raw) > 16384:
                    raise NetworkError("请求过长")
                data = json.loads(raw)
                if not isinstance(data, dict) or not isinstance(data.get("payload", {}), dict):
                    raise NetworkError("请求格式无效")
                result = manager.snapshot() if data.get("action") == "status" else manager.submit(data.get("action"), data.get("payload", {}))
            except (NetworkError, ValueError, OSError) as exc:
                result = {"error": str(exc) if isinstance(exc, NetworkError) else "请求未完成"}
            self.wfile.write(json.dumps(result, ensure_ascii=False).encode() + b"\n")
    Path(path).unlink(missing_ok=True)
    with socketserver.UnixStreamServer(path, Handler) as server:
        os.chown(path, 0, gid)
        os.chmod(path, 0o660)
        def poll():
            while True:
                manager.poll()
                time.sleep(3)
        threading.Thread(target=poll, daemon=True, name="nav-network-fallback").start()
        server.serve_forever(poll_interval=.5)
