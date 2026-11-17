"""WitRTK 桥接话题的订阅适配器与界面状态。

移植自 `NAV-Module` 的 `src/nav_app/nav_app/features/rtk.py`（**未提交快照**，
内容取自 `backup/pre-merge-20260928` 的工作区，无 commit 可锚定）。类名与函数名逐字保留，
便于与上游直接 `diff`。

**为什么这个模块叫 `wtrtk` 而不是 `rtk`：** 本仓库里 `features/rtk.py` 已经被占用 ——
它服务的是 `rtk/` 那个 `nav_rtk` 包（订阅 `NavSatFix`，发 `/rtk/enu` + `/rtk/diagnostics`）。
本模块服务的是另一条链路：`nav_rtk_bridge` 从串口读 `$WTRTK` 报文，发
`/rtk/frame` + `/rtk/status`。两者话题不同、节点不同，不可合并。用协议名 `wtrtk` 区分，
`/rtk/` 这个话题根不足以区分它们。

对应 `nav_rtk_bridge` 节点发布的两条话题：

- `/rtk/frame`：每帧 WTRTK 报文的 JSON，字段与 web 仪表盘前端消费的完全一致
  （见 `nav_rtk.parser.wtrtk_runtime.WTRTKFrameEvent.to_dict()`）；
- `/rtk/status`：低频聚合状态，补上 WTRTK 帧里没有的字段（卫星数、HDOP、
  `fix_quality`）以及串口健康和解析告警。

这两个类刻意与 `features/sensors.py` 里的 `SensorPreviewRosAdapter` /
`UnavailableSensorPreviewAdapter` 保持同形，界面层不需要为 RTK 特殊处理。
"""

from __future__ import annotations

import json
import time
import uuid
from collections import deque
from typing import Optional

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# 界面上的诊断日志保留条数，与 web 仪表盘的 MAX_DIAG_LINES 对齐。
DEFAULT_WARNING_LIMIT = 500


def _qos() -> QoSProfile:
    # 与发布端的默认 QoS 保持一致：20 Hz 的遥测流不需要重传，但可靠传输
    # 在局域网上开销可以忽略，避免任何偶发丢帧。
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class RtkRosAdapter:
    """订阅 RTK 话题，把最新数据摊平成界面好读的属性。"""

    def __init__(
        self,
        node,
        frame_topic: str = "/rtk/frame",
        status_topic: str = "/rtk/status",
        warning_limit: int = DEFAULT_WARNING_LIMIT,
    ) -> None:
        self.node = node
        self.frame_topic = str(frame_topic)
        self.status_topic = str(status_topic)
        self.frame: Optional[dict] = None
        self.observation: Optional[dict] = None
        self.observation_received_at = None
        self.observation_revision = 0
        self.state: Optional[dict] = None
        self.serial: dict = {}
        self.stats: dict = {}
        self.warnings = deque(maxlen=warning_limit)
        # 累计追加过的告警条数。环形缓冲满了以后 len(warnings) 不再增长，
        # 界面只能靠这个单调计数器算出「这次新增了哪几条」。
        self.warning_count = 0
        # `revision` 只在真的收到新帧时自增，界面据此跳过无谓的重绘。
        self.revision = 0
        self.status_revision = 0
        self.corrections = {}
        self.receiver = {}
        self.raw_gnss = {}
        self.evidence = {}
        self._diagnostic_logged_at = None
        self.corrections_received_at = 0.
        self.frame_received_at = None
        self.status_received_at = None
        self.error = ""
        self.available = False
        self._seen_frame_index: Optional[int] = None
        self._seen_warning_total = 0
        self._seen_generation: Optional[int] = None
        # 进程级计数器，只用来判断「节点进程是不是换了一个」。见 _detect_restart。
        self._seen_frames_total: Optional[int] = None
        self._seen_uptime: Optional[float] = None
        # 识别到节点重启后置位：下一次 _note_serial_change 必须补发连接提示，
        # 不能因为 generation 恰好没变就跳过。
        self._announce_serial = False
        self._subscriptions = []
        self.debug = {}
        self.debug_received_at = 0.
        self.debug_client = uuid.uuid4().hex
        self.debug_busy_reason = lambda: '任务状态尚未就绪'
        self._debug_request_publisher = None
        self._debug_guard_publisher = None
        self._debug_guard_at = 0.
        self._debug_pending_at = 0.
        self._debug_pending_id = ''
        self._start()
        self._start_debug()

    def _start_debug(self):
        try:
            self._debug_request_publisher = self.node.create_publisher(String, '/rtk/debug/request', 10)
            self._debug_guard_publisher = self.node.create_publisher(String, '/rtk/debug/guard', 10)
            self._subscriptions.append(self.node.create_subscription(String, '/rtk/debug/state', self._on_debug, 10))
        except (AttributeError, RuntimeError, ValueError):
            # Older/test adapters stay read-only; never open a second serial port.
            self._debug_request_publisher = None

    def _on_debug(self, msg):
        if len(msg.data) > 300000:
            return
        value = _load_json(msg.data)
        if isinstance(value, dict) and isinstance(value.get('session'), str):
            changed = self.debug.get('session') != value['session']
            self.debug = value
            self.debug_received_at = time.monotonic()
            terminal = any(row.get('id') == self._debug_pending_id and row.get('state') not in ('queued', 'running')
                           for row in value.get('results', []) if isinstance(row, dict))
            if changed or terminal:
                self._debug_pending_at = 0.
                self._debug_pending_id = ''

    @property
    def debug_blocks_tasks(self):
        # Unknown after a disconnect is NOT permission to start a task.
        return bool(self.debug.get('exclusive') or self.debug.get('busy')
                    or self._debug_pending_id)

    def publish_debug_guard(self, force=False):
        now = time.monotonic()
        if not self._debug_guard_publisher or not self.debug.get('session'):
            return
        if not force and now-self._debug_guard_at < .4:
            return
        self._debug_guard_at = now
        reason = self.debug_busy_reason()
        message = String()
        message.data = json.dumps(dict(session=self.debug['session'], client=self.debug_client,
                                       allowed=not bool(reason), sent_at=time.time()))
        self._debug_guard_publisher.publish(message)

    def send_debug(self, **request):
        reason = self.debug_busy_reason()
        if reason:
            raise ValueError(reason)
        if (not self._debug_request_publisher or not self.status_fresh or not self.serial.get('ok')
                or time.monotonic()-self.debug_received_at > 2):
            raise ValueError('调试连接未就绪，请先启动RTK并等待状态更新')
        if self.debug.get('busy') or self._debug_pending_id:
            raise ValueError('上一条指令尚未结束，请稍候')
        self.publish_debug_guard(force=True)
        ident = uuid.uuid4().hex
        request.update(id=ident, client=self.debug_client, session=self.debug['session'], sent_at=time.time())
        message = String()
        message.data = json.dumps(request, ensure_ascii=False)
        self._debug_pending_at = time.monotonic()
        self._debug_pending_id = ident
        self._debug_request_publisher.publish(message)
        return ident

    def _start(self) -> None:
        try:
            self._subscriptions.append(self.node.create_subscription(
                String, self.frame_topic, self._on_frame, _qos()))
            self._subscriptions.append(self.node.create_subscription(
                String, self.status_topic, self._on_status, _qos()))
            self._subscriptions.append(self.node.create_subscription(
                String, '/rtk/observation', self._on_observation, _qos()))
        except (RuntimeError, ValueError) as exc:
            self.error = f"订阅 RTK 话题失败：{exc}"
            self._subscriptions = []
            return
        self.available = True

    def stop(self) -> None:
        """释放订阅。页面销毁或重开时调用。"""

        for subscription in self._subscriptions:
            try:
                self.node.destroy_subscription(subscription)
            except (RuntimeError, ValueError):
                pass
        self._subscriptions = []
        self.available = False
        for publisher in (self._debug_request_publisher, self._debug_guard_publisher):
            if publisher is not None:
                try:
                    self.node.destroy_publisher(publisher)
                except (AttributeError, RuntimeError, ValueError):
                    pass
        self._debug_request_publisher = self._debug_guard_publisher = None

    # ------------------------------------------------------------ 回调

    def _on_frame(self, msg: String) -> None:
        payload = _load_json(msg.data)
        if not isinstance(payload, dict):
            self.error = "RTK 帧 JSON 解析失败"
            return
        index = payload.get("index")
        # 与 web 前端的 applyLiveFrame 一致：按 index 去重，只有新帧才推进。
        if index is not None and index == self._seen_frame_index:
            return
        self._seen_frame_index = index
        self.frame = payload
        self.frame_received_at = time.monotonic()
        self.error = ""
        self.revision += 1

    def _on_status(self, msg: String) -> None:
        payload = _load_json(msg.data)
        if not isinstance(payload, dict):
            return
        self.serial = payload.get("serial") or {}
        self.corrections = payload.get("corrections") or {}
        self.receiver = payload.get("receiver") or {}
        self.raw_gnss = payload.get("raw_gnss") or {}
        self.evidence = payload.get("evidence") or {}
        self.status_received_at = time.monotonic()
        self.corrections_received_at = time.monotonic()
        self.stats = payload.get("stats") or {}
        state = payload.get("state")
        self.state = state if isinstance(state, dict) else None
        self._detect_restart()
        self._absorb_warnings(payload.get("warnings") or [])
        self._note_serial_change()
        self.status_revision += 1
        self._log_receiver_snapshot()

    def _on_observation(self, msg: String) -> None:
        if len(msg.data) > 8192:
            return
        payload = _load_json(msg.data)
        if not isinstance(payload, dict) or payload.get('schema') != 1:
            return
        self.observation = payload
        self.observation_received_at = time.monotonic()
        self.observation_revision += 1
        if not hasattr(self.node, "outdoor_adapter"):
            recorder = getattr(self.node, "trajectory_test", None)
            if recorder is not None:
                recorder.feed("gnss", dict(
                    payload,
                    app_received_at=self.node.get_clock().now().nanoseconds / 1e9,
                ))

    def _log_receiver_snapshot(self):
        now = time.monotonic()
        if self._diagnostic_logged_at is not None and now - self._diagnostic_logged_at < 1.:
            return
        self._diagnostic_logged_at = now
        from .log_store import log_store
        # One small snapshot per second, through the existing bounded writer.
        # No account config is copied into logs.
        correction_keys = ("enabled", "phase", "message", "received_bytes", "rtcm_packets",
                           "injected_bytes", "injected_packets", "last_injected_age_s",
                           "gga_received_age_s", "gga_rejection_reason")
        log_store().write("sensors", "rtk-observations", json.dumps(dict(
            receiver=self.receiver, serial=self.serial, stats=self.stats,
            raw_gnss=self.raw_gnss,
            corrections={k: self.corrections[k] for k in correction_keys if k in self.corrections},
            wtrtk=(self.frame or {}).get("rawSentence") if self.frame_fresh else None), ensure_ascii=False))

    def _detect_restart(self) -> bool:
        """判断节点进程是不是重启过；是的话把本地基准清掉。

        `serial.generation` 是**进程内**计数器（`rtk_node.py` 构造为 0，
        首次打开串口才 +1），所以每个新进程都会重新发布 `generation: 1`
        —— 与旧进程的初值一模一样。只比 generation 看不出重启，`_note_serial_change`
        会一直以为「没变化」，重启后再也不补「串口已连接／不可用」那条诊断。

        改用两个进程级计数器：`stats.framesTotal`（构造为 0）和 `stats.uptimeS`
        （`time.time() - _started_at`），两者在进程重启后必然回退。两个都用是因为
        它们各自有盲区：新进程若在适配器下一次收到状态前就跑过了旧进程的帧数，
        framesTotal 看不出回退；而旧进程一帧没收到时（串口被占），只有 uptimeS 会回退。
        """

        try:
            frames_total = int(self.stats.get("framesTotal"))
        except (TypeError, ValueError):
            frames_total = None
        try:
            uptime = float(self.stats.get("uptimeS"))
        except (TypeError, ValueError):
            uptime = None

        restarted = False
        if frames_total is not None and self._seen_frames_total is not None:
            restarted = restarted or frames_total < self._seen_frames_total
        if uptime is not None and self._seen_uptime is not None:
            restarted = restarted or uptime < self._seen_uptime

        self._seen_frames_total = frames_total
        self._seen_uptime = uptime
        if restarted:
            self.observation = None
            self.observation_received_at = None
            self.observation_revision += 1
            self.frame = None
            self.frame_received_at = None
            self._seen_frame_index = None
            self.revision += 1
            self._seen_warning_total = 0
            # 不能简单地把 _seen_generation 清成 None：那会让下一次状态走进
            # `first_time` 分支被静默当成「首次见到」，提示照样丢。
            self._announce_serial = True
        return restarted

    def _absorb_warnings(self, incoming) -> None:
        """只把新增的告警追加进日志。

        `stats.parseErrors` 是节点侧累计的告警总数（进程内单调递增），拿它和本地
        已见条数做差，就能算出这次状态里哪几条是新的——即使节点侧的环形
        缓冲已经把更早的告警挤掉了，也不会重复或遗漏。
        """

        try:
            total = int(self.stats.get("parseErrors", 0))
        except (TypeError, ValueError):
            return
        new_count = total - self._seen_warning_total
        if new_count < 0:
            # 兜底：`_detect_restart` 已经按 framesTotal/uptimeS 处理了正常路径，
            # 这里再按 parseErrors 自己回退一次。`stats.parseErrors` 同样是**进程内**
            # 累计（`rtk_node.py` 的 `_parse_error_total`），重启后从 0 重来；
            # 不重置基准的话 `_seen_warning_total` 会停在重启前的旧高水位，
            # 新进程的告警要等累计数**爬回那个旧值以上**才重新出现 ——
            # 中间这段时间的解析告警被静默丢弃，界面上看不出任何异常。
            # 留这条是因为上游节点可能不带 uptimeS/framesTotal，那时只能靠它。
            self._seen_warning_total = 0
            new_count = total
        if new_count <= 0:
            return
        if new_count >= len(incoming):
            fresh = list(incoming)
        else:
            fresh = list(incoming[-new_count:])
        for entry in fresh:
            if isinstance(entry, dict):
                self._append_warning(entry)
        self._seen_warning_total = total

    def _append_warning(self, entry: dict) -> None:
        from .log_store import log_store
        log_store().write("sensors", "rtk-diagnostics", str(entry))
        self.warnings.append(entry)
        self.warning_count += 1

    def _note_serial_change(self) -> None:
        """串口重新连上/断开时，往诊断日志里补一条本地提示。"""

        generation = self.serial.get("generation")
        if generation is None:
            return
        announce = self._announce_serial
        if generation == self._seen_generation and not announce:
            return
        first_time = self._seen_generation is None
        self._seen_generation = generation
        self._announce_serial = False
        # 首次见到 generation 时不必刷屏（状态条本来就写着串口状态）；
        # 但识别到进程重启就得补发，否则日志会缺掉最该记的那一次重连。
        if first_time and not announce:
            return
        if self.serial.get("ok"):
            self._append_warning({
                "ts": "", "level": "INFO", "kind": "serial",
                "message": (
                    f"串口已连接：{self.serial.get('port', '')} "
                    f"@ {self.serial.get('baudrate', '')}"),
            })
        else:
            self._append_warning({
                "ts": "", "level": "ERROR", "kind": "serial",
                "message": f"串口不可用：{self.serial.get('error') or '未知原因'}",
            })

    # ------------------------------------------------------------ 查询

    @property
    def observation_fresh(self) -> bool:
        return self.observation_received_at is not None and time.monotonic() - self.observation_received_at <= 1.5

    @property
    def frame_fresh(self) -> bool:
        return self.frame_received_at is not None and time.monotonic() - self.frame_received_at <= 2.

    @property
    def status_fresh(self) -> bool:
        return self.status_received_at is not None and time.monotonic() - self.status_received_at <= 3.

    @property
    def serial_ok(self) -> bool:
        return bool(self.serial.get("ok")) and self.status_fresh

    def serial_description(self) -> str:
        """给状态条用的一句话描述。"""

        port = self.serial.get("port", "")
        baudrate = self.serial.get("baudrate", "")
        where = f"{port} @ {baudrate}" if port else "串口"
        if not self.available:
            return self.error or "RTK 桥接接口未启用"
        if not self.serial:
            return f"等待 {self.status_topic} …"
        if not self.status_fresh:
            return "RTK 状态已过期，请检查接收器连接"
        if self.serial_ok:
            frames = self.stats.get("framesTotal")
            suffix = f" · 已接收 {frames} 帧" if frames else ""
            # 校验失败率高的时候必须说出来。报文被撕碎时界面看起来就是
            # 「什么都没收到」（经纬度 `--`、卫星 0 颗），用户会去怀疑接收机、
            # 天线、波特率——而真正的原因通常是**有第二个进程在读同一个串口**，
            # 两边各读走一半字节。实测两读者并存时失败率 62%，单读者 0%。
            # 这条提示是唯一能把人往那个方向指的线索。
            errors = self.stats.get("parseErrors") or 0
            if frames and errors and errors >= max(20, frames * 0.1):
                return (f"已连接 {where}{suffix} · ⚠ 校验失败 {errors} 条"
                        f"（{100.0 * errors / frames:.0f}%）——检查是否有"
                        f"另一个进程在读同一个串口")
            if not self.frame_fresh and not self.observation_fresh:
                return f"串口已连接 {where} · 定位报文未更新"
            return f"已连接 {where}{suffix}"
        return f"未连接 {where} · {self.serial.get('error') or '未知原因'}"


class UnavailableRtkAdapter:
    """惰性替身，供纯界面测试和 mock 节点使用。"""

    frame = None
    observation = None
    observation_fresh = False
    observation_revision = 0
    state = None
    serial: dict = {}
    stats: dict = {}
    warnings = ()
    warning_count = 0
    revision = 0
    status_revision = 0
    error = ""
    available = False
    serial_ok = False

    def stop(self) -> None:
        pass

    def serial_description(self) -> str:
        return "当前节点未启用 RTK 接口"


def _load_json(text: str):
    try:
        return json.loads(text)
    except (TypeError, ValueError):
        return None
