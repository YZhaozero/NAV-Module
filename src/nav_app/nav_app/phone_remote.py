"""Opt-in LAN transport for the Qt window. No ROS or GUI calls on HTTP threads."""
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from http.cookies import SimpleCookie
from pathlib import Path
import hmac
import ipaddress
import json
import math
import secrets
import threading
import time
import ssl
import urllib.request
import urllib.error
from urllib.parse import urlsplit, parse_qs


class RemoteError(Exception):
    def __init__(self, status, message):
        self.status, self.message = status, message


class RemoteState:
    """One controller, bounded input, expiring frames and revocable credentials."""
    SESSION_SECONDS = 8 * 60 * 60
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.code = f"{secrets.randbelow(1000000):06d}"
        self.code_until = clock() + 300
        self.session = ""
        self.last_seen = 0.
        self.session_until = 0.
        self.attempts = deque(maxlen=5)
        self.commands = deque(maxlen=8)
        self.frames = deque(maxlen=24)
        self.image = b""
        self.serial = 0
        self.last_input = 0
        self.processed = 0
        self.notice = ""
        self.closed = False
        self.interruptions = 0
        self.suspended = False
        self.frame_requested_at = None

    def _expire(self):
        if self.session and self.clock() > self.session_until:
            self.revoke()
        elif self.session and self.clock() - self.last_seen > 5 and not self.suspended:
            # Authentication survives phone sleep; control and queued input do not.
            self.suspended = True
            self.interruptions += 1
            self.commands.clear()
            self.frames.clear()
            self.image = b""
            self.processed = max(self.processed, self.last_input)

    def live(self):
        with self.lock:
            self._expire()
            return bool(self.session and not self.suspended)

    def refresh_code(self):
        with self.lock:
            if self.clock() > self.code_until:
                self.code = f"{secrets.randbelow(1000000):06d}"
                self.code_until = self.clock() + 300

    def controller(self):
        with self.lock:
            self._expire()
            return self.session

    def revoke(self):
        with self.lock:
            self.session = ""
            self.commands.clear()
            self.frames.clear()
            self.image = b""
            self.code = f"{secrets.randbelow(1000000):06d}"
            self.code_until = self.clock() + 300
            self.last_input = self.serial
            self.suspended = False

    def pair(self, code, mirror=True):
        with self.lock:
            self._expire()
            now = self.clock()
            if self.closed:
                raise RemoteError(503, "手机遥控已关闭")
            if len(self.attempts) == 5 and now - self.attempts[0] < 60:
                raise RemoteError(429, "尝试过于频繁，请一分钟后重试")
            self.attempts.append(now)
            if not isinstance(code, str) or not hmac.compare_digest(code, self.code) or now > self.code_until:
                raise RemoteError(401, "配对码错误或已过期，请查看设备屏幕")
            if self.session and not self.suspended:
                raise RemoteError(409, "已有手机连接，请先在设备上断开")
            if self.session:
                # Someone with the current on-device code can replace an offline
                # phone. Revoke the old cookie and queued commands atomically.
                self.revoke()
            self.session = secrets.token_urlsafe(32)
            self.last_seen, self.session_until = now, now + self.SESSION_SECONDS
            self.suspended = False
            self.notice = ""
            self.frame_requested_at = now if mirror else None
            return self.session

    def _authorize(self, token):
        self._expire()
        if self.closed or not token or not self.session or not hmac.compare_digest(token, self.session):
            raise RemoteError(401, "连接已结束，请重新配对")
        self.last_seen = self.clock()
        self.suspended = False

    def publish(self, image, signature, width, height, editable=False, input_type="text", field_id=""):
        with self.lock:
            if not self.live():
                return
            self.serial += 1
            self.image = image
            self.frames.append((self.serial, self.clock(), signature, width, height, editable, self.processed,
                                "password" if input_type == "password" else "text", str(field_id)))

    def frame(self, token):
        with self.lock:
            self._authorize(token)
            self.frame_requested_at = self.clock()
            if not self.frames or self.clock() - self.frames[-1][1] > 2:
                raise RemoteError(503, "设备画面暂未更新，操作已暂停")
            return self.image, self.frames[-1], self.notice

    def wants_frame(self):
        with self.lock:
            return self.frame_requested_at is not None and self.clock()-self.frame_requested_at <= 2.

    def submit(self, token, command):
        with self.lock:
            self._authorize(token)
            if not isinstance(command, dict):
                raise RemoteError(400, "无效操作")
            frame_id = command.get("frame")
            if type(frame_id) is not int:
                raise RemoteError(400, "无效画面编号")
            frame = next((f for f in self.frames if f[0] == frame_id), None)
            if not frame or self.clock() - frame[1] > 2 or frame_id <= self.last_input:
                raise RemoteError(409, "画面已变化，请看清新画面后重试")
            kind = command.get("kind")
            if kind == "gesture":
                points = command.get("points")
                if not isinstance(points, list) or not 1 <= len(points) <= 64:
                    raise RemoteError(400, "无效触摸")
                for point in points:
                    if not isinstance(point, list) or len(point) != 2 or any(
                        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not 0 <= v <= 1 for v in point
                    ):
                        raise RemoteError(400, "触摸位置无效")
            elif kind == "text":
                if not isinstance(command.get("text"), str) or len(command["text"]) > 512 or not frame[5]:
                    raise RemoteError(400, "请先点选设备上的输入框")
                if any(type(command.get(key, False)) is not bool for key in ('replace', 'commit')):
                    raise RemoteError(400, "文字输入选项无效")
            elif kind == "key":
                if command.get("key") not in ("Enter", "Escape", "Backspace"):
                    raise RemoteError(400, "不支持此按键")
            else:
                raise RemoteError(400, "不支持此操作")
            if len(self.commands) == self.commands.maxlen:
                raise RemoteError(429, "设备正在处理，请稍后重试")
            self.last_input = frame_id
            self.commands.append((token, frame, command))

    def take(self):
        with self.lock:
            self._expire()
            commands = list(self.commands)
            self.commands.clear()
            return commands


class RemoteServer:
    def __init__(self, state, host, port, allowed_hosts, public_url="", outdoor=None, map_services=None):
        self.public_authority = ""
        if public_url:
            parsed = urlsplit(public_url)
            if (parsed.scheme != "http" or not parsed.hostname or parsed.username is not None
                    or parsed.password is not None or parsed.path not in ("", "/")
                    or parsed.query or parsed.fragment or any(c.isspace() for c in public_url)
                    or parsed.port == 0):
                raise ValueError("Expected an explicit HTTP origin for TCP forwarding")
            self.public_authority = parsed.netloc.lower()
        self.state = state
        self.outdoor, self.map_services = outdoor, map_services
        self.update_hosts(allowed_hosts)
        owner = self
        static = Path(__file__).with_name("phone_web")

        class Handler(BaseHTTPRequestHandler):
            # Frame polling reuses its connection, including through a TCP relay.
            protocol_version = "HTTP/1.1"

            def setup(self):
                super().setup()
                self.connection.settimeout(3)

            def handle(self):
                try:super().handle()
                except (ConnectionError,TimeoutError):
                    pass  # Phone sleep/roaming may close an idle keep-alive socket.

            def log_message(self, *_):
                pass  # Never log pairing codes, cookies, input or images.

            def send(self, status, body, content_type="application/json; charset=utf-8", headers=None):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                if self.close_connection:
                    self.send_header("Connection", "close")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                outdoor_page = self.path.split("?",1)[0] in ("/outdoor", "/navigation")
                control_image_sources = "'self' blob:"
                self.send_header("Referrer-Policy", "strict-origin" if outdoor_page else "no-referrer")
                self.send_header("Content-Security-Policy", (
                    # JSAPI 4.0 selects subresource protocol from the page origin.
                    # The existing LAN/FRP device page is HTTP; scope compatibility
                    # to Baidu resources on this map page, never the control page.
                    # The Baidu SDK evaluates downloaded/cached modules. Keep this
                    # exception on the dedicated map document, not Qt remote UI.
                    "default-src 'self'; script-src 'self' 'unsafe-eval' https://*.map.baidu.com https://*.bdimg.com https://*.bcebos.com http://api.map.baidu.com http://*.bdimg.com http://dlswbr.baidu.com; "
                    "img-src 'self' data: blob: https://*.baidu.com https://*.bdimg.com https://*.bcebos.com http://*.baidu.com http://*.bdimg.com; "
                    "style-src 'self' 'unsafe-inline' https://api.map.baidu.com; connect-src 'self' https://*.baidu.com https://*.bdimg.com https://*.bcebos.com http://*.baidu.com http://*.bdimg.com; "
                    "worker-src blob:; font-src 'self' data:; frame-ancestors 'none'; base-uri 'none'; form-action 'self'"
                    if outdoor_page else f"default-src 'self'; img-src {control_image_sources}; style-src 'self'; script-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'"))
                for key, value in (headers or {}).items():
                    self.send_header(key, value)
                self.end_headers()
                self.wfile.write(body)

            def token(self):
                cookie = SimpleCookie()
                try:
                    cookie.load(self.headers.get("Cookie", ""))
                    return cookie["nav_remote"].value if "nav_remote" in cookie else ""
                except Exception:
                    return ""

            def check_host(self, post=False):
                host = self.headers.get("Host", "").lower()
                parts = host.split(":")
                if host != owner.public_authority and (len(parts) > 2 or parts[0] not in owner.allowed_hosts or (len(parts) == 2 and parts[1] != str(self.server.server_port))):
                    raise RemoteError(403, "请使用设备屏幕上显示的地址")
                origin = self.headers.get("Origin")
                if origin is not None and origin.lower() != "http://" + host:
                    raise RemoteError(403, "请直接打开设备遥控页面")
                if self.headers.get("Sec-Fetch-Site") == "cross-site":
                    raise RemoteError(403, "请直接打开设备遥控页面")
                if post and self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    raise RemoteError(415, "请求格式无效")

            def dispatch(self, post=False):
                try:
                    self.check_host(post)
                    path = self.path.split("?", 1)[0]
                    if post and path == '/api/phone-gnss':
                        size=int(self.headers.get('Content-Length','0'))
                        if not 0 < size <= 3*1024*1024:raise RemoteError(413,'手机定位数据过大')
                        payload=self.rfile.read(size)
                        if len(payload)!=size:raise RemoteError(400,'数据接收不完整')
                        # Opaque authenticated ciphertext. The HTTP worker forwards
                        # to a separate process; it never calls Qt or runs RTK.
                        cert=Path.home()/'.config/nav/phone-gnss/device.crt'
                        if not cert.is_file():raise RemoteError(503,'手机定位接收服务未配置')
                        context=ssl.create_default_context(cafile=str(cert))
                        request=urllib.request.Request('https://127.0.0.1:8830/v1/sealed',data=payload,headers={'Content-Type':'application/json'})
                        try:
                            with urllib.request.urlopen(request,context=context,timeout=3) as response:
                                body=response.read(3*1024*1024+1)
                                if len(body)>3*1024*1024:raise RemoteError(502,'设备响应过大')
                                self.send(response.status,body)
                        except urllib.error.HTTPError as exc:self.send(exc.code,b'{"message":"Phone GNSS request rejected"}')
                        except (OSError,urllib.error.URLError):raise RemoteError(503,'手机定位接收服务暂不可用')
                        return
                    if path.startswith('/navigation/'):
                        path = '/outdoor' + path[len('/navigation'):]
                    if path.startswith("/outdoor") and path not in ("/outdoor", "/outdoor.js", "/outdoor.css"):
                        with state.lock:
                            state._authorize(self.token())
                        if owner.outdoor is None or owner.map_services is None:
                            raise RemoteError(503, "地图与路线尚未开启")
                    if not post and path == "/baidu_map.js":
                        self.send(200,(static/"baidu_map.js").read_bytes(),"text/javascript; charset=utf-8")
                    elif not post and path in ('/navigation', '/walking.js', '/walking.css'):
                        name,mime={'/navigation':('walking.html','text/html; charset=utf-8'),
                            '/walking.js':('walking.js','text/javascript; charset=utf-8'),
                            '/walking.css':('walking.css','text/css; charset=utf-8')}[path]
                        self.send(200,(static/name).read_bytes(),mime)
                    elif not post and path == '/outdoor/experience':
                        from .navigation_experience import experience_for
                        value=experience_for(owner.outdoor,owner.map_services).snapshot()
                        self.send(200,json.dumps(value,ensure_ascii=False,allow_nan=False).encode())
                    elif not post and path in ("/outdoor", "/outdoor.js", "/outdoor.css"):
                        name,mime={"/outdoor":("outdoor.html","text/html; charset=utf-8"),
                            "/outdoor.js":("outdoor.js","text/javascript; charset=utf-8"),
                            "/outdoor.css":("outdoor.css","text/css; charset=utf-8")}[path]
                        self.send(200,(static/name).read_bytes(),mime)
                    elif not post and path == "/outdoor/state":
                        self.send(200,json.dumps(owner.outdoor.snapshot(),ensure_ascii=False,allow_nan=False).encode())
                    elif not post and path == "/outdoor/config":
                        self.send(200,json.dumps({"browser_key":owner.map_services.browser_key}).encode())
                    elif not post and path == "/outdoor/export":
                        raw=json.dumps(owner.outdoor.export_document(),ensure_ascii=False,allow_nan=False).encode()
                        self.send(200,raw,headers={"Content-Disposition":"attachment; filename=nav-outdoor.json"})
                    elif not post and path in ("/", "/remote", "/app.js", "/style.css"):
                        name, mime = {"/": ("index.html", "text/html; charset=utf-8"),
                            "/remote": ("index.html", "text/html; charset=utf-8"),
                            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                            "/style.css": ("style.css", "text/css; charset=utf-8")}[path]
                        self.send(200, (static / name).read_bytes(), mime)
                    elif not post and path == "/frame":
                        image, frame, notice = state.frame(self.token())
                        self.send(200, image, "image/jpeg", {"X-Frame-Id": str(frame[0]), "X-Editable": "1" if frame[5] else "0", "X-Input-Processed": str(frame[6]), "X-Input-Rejected": "1" if notice else "0",
                                  "X-Input-Type": frame[7], "X-Input-Field": frame[8]})
                    elif post:
                        length = int(self.headers.get("Content-Length", "0"))
                        if not 0 < length <= (262144 if path in ("/outdoor/route", "/outdoor/journey") else 8192):
                            raise RemoteError(413, "请求过大或为空")
                        data = json.loads(self.rfile.read(length))
                        if not isinstance(data, dict):
                            raise RemoteError(400, "请求格式无效")
                        if path == '/outdoor/experience/control':
                            from .navigation_experience import experience_for
                            try:result=experience_for(owner.outdoor,owner.map_services).command(data)
                            except ValueError as exc:raise RemoteError(400,str(exc)) from None
                            self.send(200,json.dumps(result,ensure_ascii=False,allow_nan=False).encode())
                        elif path in ("/outdoor/route", "/outdoor/plan", "/outdoor/control", "/outdoor/simulation", "/outdoor/map", "/outdoor/journey"):
                            try:
                                if path == '/outdoor/journey':
                                    result=owner.outdoor.journey_command(data)
                                elif path == '/outdoor/map':
                                    action=data.get('action')
                                    if action=='select':
                                        owner.outdoor.select_geo_map(data.get('id'))
                                    elif action=='anchor':
                                        owner.outdoor.anchor_selected_map()
                                    elif action=='confirm':
                                        owner.outdoor.confirm_current_position()
                                    else:
                                        owner.outdoor.request_localization(action)
                                    result={'ok':True}
                                elif path == "/outdoor/route":
                                    result=owner.outdoor.set_route(data)
                                elif path == "/outdoor/plan":
                                    result=owner.map_services.plan(data.get("start"),data.get("end"),data.get("title","室外路线"))
                                    # Preview only: user explicitly saves before replacing the route.
                                    result=owner.outdoor.validate_route(result)
                                elif path == "/outdoor/simulation":
                                    owner.outdoor.simulate(data.get("action"))
                                    result={"ok":True}
                                else:
                                    owner.outdoor.control(data.get("action"))
                                    result={"ok":True}
                                self.send(200,json.dumps(result,ensure_ascii=False,allow_nan=False).encode())
                            except OSError:
                                raise RemoteError(400,'地图文件无法读取，请检查文件和保存空间') from None
                            except ValueError as exc:
                                raise RemoteError(400,str(exc)) from None
                        elif path == "/pair":
                            if data.get('mode') not in (None, 'journey'):
                                raise RemoteError(400, '配对方式无效')
                            token = state.pair(data.get("code"), mirror=data.get('mode') != 'journey')
                            self.send(200, b'{}', headers={"Set-Cookie": f"nav_remote={token}; HttpOnly; SameSite=Strict; Path=/; Max-Age={state.SESSION_SECONDS}"})
                        elif path == "/input":
                            state.submit(self.token(), data)
                            self.send(202, b'{}')
                        elif path == "/disconnect":
                            with state.lock:
                                state._authorize(self.token())
                                state.revoke()
                            self.send(200, b'{}', headers={"Set-Cookie": "nav_remote=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0"})
                        else:
                            raise RemoteError(404, "页面不存在")
                    else:
                        raise RemoteError(404, "页面不存在")
                except RemoteError as exc:
                    # Rejected requests may have an unread body.
                    self.close_connection = True
                    self.send(exc.status, json.dumps({"message": exc.message}, ensure_ascii=False).encode())
                except (ValueError, TypeError):
                    self.close_connection = True
                    self.send(400, b'{"message":"Invalid request"}')
                except (OSError, ConnectionError):
                    pass

            def do_GET(self):
                self.dispatch()

            def do_POST(self):
                self.dispatch(True)

        class BoundedServer(ThreadingHTTPServer):
            daemon_threads = True
            def __init__(self, *args):
                self.slots = threading.BoundedSemaphore(12)
                super().__init__(*args)

            def process_request(self, request, address):
                if not self.slots.acquire(blocking=False):
                    self.shutdown_request(request)
                    return
                try:
                    super().process_request(request, address)
                except Exception:
                    self.slots.release()
                    raise

            def process_request_thread(self, request, address):
                try:
                    super().process_request_thread(request, address)
                finally:
                    self.slots.release()

        # Only explicit IPv4 binds; no shell/network configuration is performed.
        ipaddress.IPv4Address(host)
        self.http = BoundedServer((host, port), Handler)
        self.port = self.http.server_port
        self.thread = threading.Thread(target=self.http.serve_forever, kwargs={"poll_interval": .1}, daemon=True)
        self.thread.start()

    def update_hosts(self, addresses):
        # Replace the immutable snapshot; HTTP readers never observe a partial update.
        self.allowed_hosts = frozenset(str(h).lower() for h in addresses) | {"localhost", "127.0.0.1"}

    def close(self):
        self.state.closed = True
        self.state.revoke()
        self.http.shutdown()
        self.http.server_close()
        self.thread.join(timeout=1)
