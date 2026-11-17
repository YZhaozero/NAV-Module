"""RTK 实时监看页签。

移植自 `NAV-Module` 的 `src/nav_app/nav_app/ui/rtk_page.py`（**未提交快照**，
内容取自 `backup/pre-merge-20260928` 的工作区，无 commit 可锚定）。类名与常量逐字保留，
便于与上游直接 `diff`；唯一的功能性改动是适配器改从 `..features.wtrtk` 导入
（见该模块开头解释为什么叫 `wtrtk` 而不是 `rtk`）。

实时指标按定位、方向、设备和基站分组；原始报文和诊断自动落盘。
刻意复用了主窗口已有的 objectName（`sidePanel` / `sensorPreviewStatus` /
`sensorLog` / `hint`），这样现有的全局样式表不需要为新页面改动一行。

本页只依赖 `features/wtrtk.py` 的适配器，不自己创建 ROS 订阅，也不自己起定时器
——`refresh()` 由主窗口那个 33 ms 定时器在切到本页签时调用。
"""

from __future__ import annotations

import html
import math
import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QLabel,
    QPlainTextEdit,
    QScrollArea,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..features.wtrtk import UnavailableRtkAdapter
from ..features.log_store import LogBuffer
from ..features.rtk_readiness import quality_label, receiver_coordinates
from ..features.rtk_diagnostics import diagnostics
from .touch_scroll import enable_touch_scroll
from .rtk_console import RtkCommandPanel, RtkTrafficPanel

GOOD_COLOR = "#66e09a"
WARN_COLOR = "#ffd166"
BAD_COLOR = "#ff7f88"
NEUTRAL_COLOR = "#91a2ad"

# 与 web 仪表盘 app.js 的 statusDotClass 同规则，另外把「未对准」也归到告警色
# ——安装没对准比定位状态更容易被忽略，值得显眼一点。
_STATUS_RULES = (
    (GOOD_COLOR, ("固定解", "浮点解", "已对准", "已二次对准")),
    (WARN_COLOR, ("码差分", "单点定位", "未对准")),
    (BAD_COLOR, ("初始化", "未知")),
)

# (frame 里的键, 卡片标题, 呈现方式)
# 顺序照搬 web 仪表盘的 metricsLayout，只把三个状态卡片提到了顶部——状态
# 是一眼要看的东西，压在磁场/差分中间没有道理。回放专用的「回放日志
# 时间」因为本页只做实时监看，没有迁移。
METRIC_LAYOUT = (
    ("clock", "系统时间", "clock"),
    ("timeText", "当前帧时间", "text"),
    ("alignmentText", "安装对准", "status"),
    ("positioningText", "定位状态", "status"),
    ("rtkModeText", "RTK 模式", "status"),
    ("latitude", "纬度", "coord"),
    ("longitude", "经度", "coord"),
    ("altitude", "海拔 (m)", "float"),
    ("speedKmh", "地速 (km/h)", "float"),
    ("headingDeg", "航向角 (°)", "float"),
    ("gpsHeading", "GPS 航向角 (°)", "float"),
    ("angleX", "角度X (°)", "float"),
    ("angleY", "角度Y (°)", "float"),
    ("angleZ", "角度Z (°)", "float"),
    ("magX", "磁场X (uT)", "float"),
    ("magY", "磁场Y (uT)", "float"),
    ("magZ", "磁场Z (uT)", "float"),
    ("temperatureC", "温度 (℃)", "float"),
    ("baseDistanceM", "基站距离 (m)", "float"),
    ("diffX", "差分X (m)", "float"),
    ("diffY", "差分Y (m)", "float"),
    ("diffR", "距离R (m)", "float"),
)

_VALUE_STYLE = "font-family: Monospace; font-size: 18px; font-weight: 700;"


def _safe_number(value):
    """把可能是 None / "" / "--" / 非数字的值收敛成 float 或 None。"""

    if value is None or value == "" or value == "--":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def format_coordinate(value) -> str:
    number = _safe_number(value)
    return "--" if number is None else f"{number:.8f}"


def format_measure(value) -> str:
    """与 app.js 的 float 规则一致：>=100 保留两位，否则三位。"""

    number = _safe_number(value)
    if number is None:
        return "--"
    return f"{number:.2f}" if abs(number) >= 100 else f"{number:.3f}"


def status_color(text: str) -> str:
    for color, keywords in _STATUS_RULES:
        if any(keyword in text for keyword in keywords):
            return color
    return NEUTRAL_COLOR


def _set_text(label: QLabel, text: str) -> None:
    """文本没变就不碰控件，省掉 20 Hz 下的大量无谓重绘。"""

    if label.text() != text:
        label.setText(text)


def _format_warning(entry) -> tuple[str, str]:
    """返回 (文本, 颜色)，格式与 web 终端一致。"""

    level = str(entry.get("level") or "INFO").upper()
    kind = str(entry.get("kind") or "").upper() or "RTK"
    timestamp = entry.get("ts") or time.strftime("%H:%M:%S")
    message = entry.get("message") or ""
    color = {"ERROR": BAD_COLOR, "WARN": WARN_COLOR}.get(level, "#b9d9c5")
    return f"{timestamp} [{level}] [{kind}] {message}", color


class RtkMonitorTab(QWidget):
    """RTK 页签：拉取适配器里的最新数据并就地更新控件。

    每次刷新只改已有控件的文本，不重建控件树——web 端每帧重建整张表
    是 DOM 的权宜之计，搬到 Qt 里 20 Hz 重建会很浪费。
    """

    def __init__(self, adapter=None, parent=None) -> None:
        super().__init__(parent)
        self.adapter = adapter or UnavailableRtkAdapter()
        self._tiles: dict = {}
        self._status_colors: dict = {}
        self._banner_color = ""
        self._last_revision = -1
        self._last_status_revision = -1
        self._rendered_warnings = 0
        self._diagnostics_cleared = False
        self._diagnostics_refreshed_at = 0.
        self._build_ui()

    # ------------------------------------------------------------- 构建

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        self.serial_label = QLabel("等待 RTK 数据…")
        self.serial_label.setObjectName("sensorPreviewStatus")
        self.serial_label.setWordWrap(True)
        root.addWidget(self.serial_label)

        self.hint = QLabel("在“定位状态”查看单点定位、浮点解或固定解；航向角不代表地图方向已确认。")
        self.hint.setObjectName("hint")
        self.hint.setWordWrap(True)
        self.hint.setStyleSheet("font-size: 15px; color: #b6c8d3;")
        root.addWidget(self.hint)

        root.addWidget(self._build_metric_area(), 1)
        self.raw_view = LogBuffer(persist=False)
        self.diag_view = LogBuffer(persist=False)

    def _build_metric_area(self) -> QWidget:
        self.metric_tabs = QTabWidget()
        self.metric_tabs.setStyleSheet(
            "QTabBar::tab { min-height: 32px; padding: 0 10px; font-size: 17px; }")
        self._metric_scrolls = []
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(4, 4, 4, 4)
        self.diagnostic_labels = {}
        for caption, _ in diagnostics(self.adapter)[0]:
            name = QLabel(caption)
            name.setObjectName('hint')
            value = QLabel('--')
            value.setWordWrap(True)
            value.setTextFormat(Qt.PlainText)
            value.setStyleSheet('font-size: 17px; padding-bottom: 6px;')
            layout.addWidget(name)
            layout.addWidget(value)
            self.diagnostic_labels[caption] = value
        explanation = QLabel('差分下载、写入接收器和获得固定解分别显示。搜星数按系统显示，各频段不重复相加。')
        explanation.setWordWrap(True)
        explanation.setObjectName('hint')
        layout.addWidget(explanation)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(panel)
        enable_touch_scroll(scroll)
        self.metric_tabs.addTab(scroll, '诊断')
        groups = (
            ("定位", ("positioningText", "rtkModeText", "latitude", "longitude", "altitude", "speedKmh")),
            ("方向", ("headingDeg", "gpsHeading", "alignmentText", "angleX", "angleY", "angleZ")),
            ("设备", ("clock", "timeText", "magX", "magY", "magZ", "temperatureC")),
            ("基站", ("baseDistanceM", "diffX", "diffY", "diffR")),
        )
        definitions = {key: (caption, kind) for key, caption, kind in METRIC_LAYOUT}
        for title, keys in groups:
            container = QWidget()
            grid = QGridLayout(container)
            grid.setContentsMargins(4, 4, 4, 4)
            grid.setHorizontalSpacing(6)
            grid.setVerticalSpacing(6)
            for position, key in enumerate(keys):
                caption, kind = definitions[key]
                tile, value_label = self._make_tile(caption)
                self._tiles[key] = (value_label, kind)
                grid.addWidget(tile, position // 2, position % 2)
            grid.setColumnStretch(0, 1)
            grid.setColumnStretch(1, 1)
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QFrame.NoFrame)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            scroll.setWidget(container)
            enable_touch_scroll(scroll)
            self._metric_scrolls.append(scroll)
            self.metric_tabs.addTab(scroll, title)
        self.receiver_raw = QPlainTextEdit()
        self.receiver_raw.setReadOnly(True)
        self.receiver_raw.setMaximumBlockCount(40)
        self.receiver_raw.setPlaceholderText('等待接收器卫星报文')
        self.metric_tabs.addTab(self.receiver_raw, '原始报文')
        self.traffic_panel = RtkTrafficPanel(self.adapter)
        self.command_panel = RtkCommandPanel(self.adapter)
        self.metric_tabs.addTab(self.traffic_panel, '原始收发')
        self.metric_tabs.addTab(self.command_panel, '指令调试')
        return self.metric_tabs

    def _make_tile(self, caption: str):
        tile = QFrame()
        tile.setObjectName("sidePanel")
        tile.setMinimumHeight(68)
        layout = QVBoxLayout(tile)
        layout.setContentsMargins(8, 5, 8, 5)
        layout.setSpacing(1)
        name = QLabel(caption)
        name.setObjectName("hint")
        name.setStyleSheet("font-size: 15px; color: #b6c8d3;")
        value = QLabel("--")
        value.setMinimumWidth(0)
        value.setWordWrap(True)
        value.setStyleSheet(_VALUE_STYLE)
        layout.addWidget(name)
        layout.addWidget(value)
        return tile, value


    # ------------------------------------------------------------- 刷新

    def refresh(self) -> None:
        """由主窗口的刷新定时器驱动；没有新数据时几乎不做功。"""

        adapter = self.adapter
        self.command_panel.refresh()
        if self.metric_tabs.currentWidget() is self.traffic_panel:
            self.traffic_panel.refresh()
        now = time.monotonic()
        if now - self._diagnostics_refreshed_at >= .5:
            self._diagnostics_refreshed_at = now
            rows, raw = diagnostics(adapter)
            for caption, value in rows:
                _set_text(self.diagnostic_labels[caption], value)
            if self.receiver_raw.toPlainText() != raw:
                self.receiver_raw.setPlainText(raw)
        # 系统时间独立于 RTK 数据：数据停了它也要继续走，否则界面像卡死。
        clock = self._tiles.get("clock")
        if clock is not None:
            _set_text(clock[0], time.strftime("%H:%M:%S"))

        self._refresh_banner(adapter)
        if adapter.status_revision != self._last_status_revision:
            self._last_status_revision = adapter.status_revision
            self._refresh_banner(adapter)
            self._refresh_warnings(adapter)

        revision = (adapter.revision, getattr(adapter, "frame_fresh", True),
                    getattr(adapter, 'observation_revision', 0), getattr(adapter, 'observation_fresh', False))
        if revision != self._last_revision:
            self._last_revision = revision
            self._refresh_frame(adapter)

    def _refresh_banner(self, adapter) -> None:
        text = adapter.serial_description()
        if not adapter.available:
            color = NEUTRAL_COLOR
        elif not adapter.serial:
            color = NEUTRAL_COLOR
        elif adapter.serial_ok:
            color = GOOD_COLOR
        else:
            color = BAD_COLOR
        if color != self._banner_color:
            self._banner_color = color
            self.serial_label.setStyleSheet(
                f"color: {color}; background: #0d1d27; border-radius: 7px;"
                " padding: 8px; font-size: 12px; font-weight: 700;")
        _set_text(self.serial_label, text)

    def _refresh_frame(self, adapter) -> None:
        frame = adapter.frame
        frame_fresh = isinstance(frame, dict) and getattr(adapter, "frame_fresh", True)
        values = dict(frame) if frame_fresh else {}
        observation = getattr(adapter, 'observation', None)
        if isinstance(observation, dict) and getattr(adapter, 'observation_fresh', False):
            lat, lon = receiver_coordinates(observation)
            values.update(positioningText=quality_label(observation.get('fix_quality')),
                          latitude=lat, longitude=lon,
                          altitude=observation.get('altitude_msl') if lat is not None else None)
        else:
            values.update(positioningText='过期' if observation is not None else '等待卫星位置',
                          latitude=None, longitude=None, altitude=None)
        for key, (label, kind) in self._tiles.items():
            if kind == "clock":
                continue
            value = values.get(key)
            if kind == "status":
                self._set_status(label, value or ('过期' if isinstance(frame, dict) else '等待'))
            elif kind == "coord":
                _set_text(label, format_coordinate(value))
            elif kind == "float":
                _set_text(label, format_measure(value))
            else:
                _set_text(label, "--" if value in (None, "") else str(value))

        raw = values.get("rawSentence") or ""
        if raw and self.raw_view.toPlainText() != raw:
            self.raw_view.setPlainText(raw)

    def _set_status(self, label: QLabel, value) -> None:
        text = str(value).strip() if value not in (None, "") else "未知"
        color = status_color(text)
        if self._status_colors.get(label) != color:
            self._status_colors[label] = color
            label.setStyleSheet(
                f"color: {color}; font-family: Monospace;"
                " font-size: 20px; font-weight: 700;")
        _set_text(label, f"● {text}")

    def _refresh_warnings(self, adapter) -> None:
        total = int(getattr(adapter, "warning_count", 0))
        if total <= self._rendered_warnings:
            return
        warnings = list(adapter.warnings)
        new_count = total - self._rendered_warnings
        fresh = warnings if new_count >= len(warnings) else warnings[-new_count:]
        if not self._diagnostics_cleared and fresh:
            self.diag_view.clear()
            self._diagnostics_cleared = True
        for entry in fresh:
            if not isinstance(entry, dict):
                continue
            text, color = _format_warning(entry)
            self.diag_view.appendHtml(
                f'<span style="color:{color}">{html.escape(text)}</span>')
        self._rendered_warnings = total
