"""Touch-friendly console. All UART I/O remains in nav_rtk.bridge."""
import base64
from collections import deque
import json
import os
import time

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QHBoxLayout, QLabel,
                             QLineEdit, QPlainTextEdit, QPushButton, QScrollArea,
                             QSizePolicy, QVBoxLayout, QWidget)
from nav_rtk.bridge.command_catalog import COMMANDS, BY_KEY, prepare, custom_command
from .touch_scroll import enable_touch_scroll
from ..features.log_store import log_store


# Match the adjacent satellite tabs instead of inheriting the handheld's 22px
# home-screen font. Keep this local so other pages and touch targets are unchanged.
_CONSOLE_STYLE = """
    QLabel { font-size: 15px; }
    QPlainTextEdit { font-family: Monospace; font-size: 15px; }
    QLabel#commandTranscript { font-family: Monospace; font-size: 15px; }
    QComboBox QAbstractItemView { font-size: 17px; }
"""


class CommandTranscript(QLabel):
    """Wrapped, selectable text with no nested scrolling viewport."""
    def __init__(self):
        super().__init__()
        self.setObjectName('commandTranscript')
        self.setTextFormat(Qt.PlainText)
        self.setWordWrap(True)
        self.setTextInteractionFlags(Qt.TextSelectableByMouse | Qt.TextSelectableByKeyboard)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.setAlignment(Qt.AlignLeft | Qt.AlignTop)

    def setPlainText(self, text):
        self.setText(text)

    def toPlainText(self):
        return self.text()


def result_summary(row):
    """Prefer the compact UI field; old bridges still get a terse summary."""
    if row.get('summary'):
        return row['summary']
    state = row.get('state')
    if state in ('queued', 'running'):
        return '正在执行' if state == 'running' else '等待发送'
    if state == 'unconfirmed':
        return (row.get('completed', '')+'；' if row.get('completed') else '') + (row.get('stage') or '指令结果')+'未确认'
    if state in ('acknowledged', 'verified'):
        return '读回值已确认' if state == 'verified' else '已收到回复'
    if state == 'ended':
        return '已结束调试'
    return row.get('message', '未收到回复').split('\n')[0]


class RtkCommandPanel(QScrollArea):
    def __init__(self, adapter, parent=None):
        super().__init__(parent)
        self.setStyleSheet(_CONSOLE_STYLE)
        self.adapter = adapter
        self.confirmed_request = None
        self.request_id = ''
        self.request_at = 0.
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        body = QWidget()
        layout = QVBoxLayout(body)
        self.state = QLabel('调试连接尚未就绪')
        self.state.setWordWrap(True)
        layout.addWidget(self.state)
        self.category = QComboBox()
        self.category.addItems(list(dict.fromkeys(c.group for c in COMMANDS)) + ['自定义'])
        self.command = QComboBox()
        self.value = QComboBox()
        self.custom = QLineEdit()
        self.custom.setMaxLength(2048)
        self.custom.setPlaceholderText('输入一条ASCII指令或HEX字节')
        self.encoding = QComboBox()
        self.encoding.addItem('文本', 'text')
        self.encoding.addItem('HEX', 'hex')
        self.crlf = QCheckBox('追加回车换行（CRLF）')
        self.crlf.setChecked(True)
        self.automatic = QCheckBox('自动进入/退出配置模式')
        self.automatic.setChecked(True)
        for widget in (self.category, self.command, self.value, self.custom, self.encoding, self.crlf, self.automatic):
            layout.addWidget(widget)
        self.preview = QLabel()
        self.preview.setWordWrap(True)
        self.preview.setTextFormat(Qt.PlainText)
        layout.addWidget(self.preview)
        buttons = QHBoxLayout()
        self.send_button = QPushButton('发送')
        self.confirm_button = QPushButton('确认并发送')
        self.send_button.setObjectName('primaryButton')
        self.confirm_button.setObjectName('primaryButton')
        self.confirm_button.hide()
        self.cancel_button = QPushButton('取消')
        self.cancel_button.hide()
        self.resume_button = QPushButton('结束调试并恢复接收')
        for widget in (self.send_button, self.confirm_button, self.cancel_button):
            buttons.addWidget(widget)
        layout.addLayout(buttons)
        layout.addWidget(self.resume_button)
        self.outcome = QLabel('尚未发送指令')
        self.outcome.setWordWrap(True)
        self.outcome.setTextFormat(Qt.PlainText)
        layout.addWidget(self.outcome)
        self.result = CommandTranscript()
        self.result.setPlainText('发送后显示回复')
        layout.addWidget(self.result)
        layout.addStretch(1)
        self.setWidget(body)
        enable_touch_scroll(self)
        self.category.currentIndexChanged.connect(self._category_changed)
        self.command.currentIndexChanged.connect(self._command_changed)
        self.value.currentIndexChanged.connect(self._preview)
        self.custom.textChanged.connect(self._preview)
        self.encoding.currentIndexChanged.connect(self._preview)
        self.crlf.toggled.connect(self._preview)
        self.automatic.toggled.connect(self._preview)
        self.send_button.clicked.connect(self.send)
        self.confirm_button.clicked.connect(self.confirm)
        self.cancel_button.clicked.connect(self.cancel)
        self.resume_button.clicked.connect(lambda: self._dispatch(dict(key='RESUME', automatic=True)))
        self._category_changed()

    def cancel(self):
        self.confirmed_request = None
        self.confirm_button.hide()
        self.cancel_button.hide()

    def _category_changed(self):
        self.command.blockSignals(True)
        self.command.clear()
        for row in COMMANDS:
            if row.group == self.category.currentText():
                self.command.addItem(row.title, row.key)
        self.command.blockSignals(False)
        custom = self.category.currentText() == '自定义'
        self.command.setVisible(not custom)
        for widget in (self.custom, self.encoding, self.crlf):
            widget.setVisible(custom)
        self._command_changed()

    def _command_changed(self):
        self.value.blockSignals(True)
        self.value.clear()
        row = BY_KEY.get(self.command.currentData())
        for title, value in row.choices if row else ():
            self.value.addItem(title, value)
        self.value.blockSignals(False)
        self.value.setVisible(bool(row and row.choices))
        self._preview()

    def request(self):
        result = dict(automatic=self.automatic.isChecked())
        if self.category.currentText() == '自定义':
            result.update(key='CUSTOM', text=self.custom.text(), encoding=self.encoding.currentData(), crlf=self.crlf.isChecked())
        else:
            result.update(key=self.command.currentData(), value=self.value.currentData() or '')
        return result

    def _preview(self):
        self.cancel()
        request = self.request()
        try:
            if request['key'] == 'CUSTOM':
                row, wire, _ = custom_command(request['text'], request['encoding'], request['crlf'], True)
            else:
                row, wire = prepare(request['key'], request['value'], True)
            preview = wire.decode('ascii', errors='backslashreplace').replace('\r', '<CR>').replace('\n', '<LF>')
            self.preview.setText(f'发送：{preview}' + ('\n'+row.warning if row.warning else ''))
        except ValueError as exc:
            self.preview.setText(str(exc))

    def send(self):
        request = self.request()
        row = BY_KEY.get(request['key'])
        if row and row.unavailable:
            self.result.setPlainText(row.unavailable)
            return
        if request['key'] == 'CUSTOM' or (row and row.warning):
            self.confirmed_request = request
            self.confirm_button.show()
            self.cancel_button.show()
            self.result.setPlainText('请核对上方发送内容及风险，再点击“确认并发送”。')
            self.ensureWidgetVisible(self.confirm_button)
            return
        self._dispatch(request)

    def confirm(self):
        request = self.confirmed_request
        self.cancel()
        if request:
            self._dispatch(dict(request, confirmed=True))

    def _dispatch(self, request):
        try:
            if not hasattr(self.adapter, 'send_debug'):
                raise ValueError('当前连接只支持查看，无法发送指令')
            self.request_id = self.adapter.send_debug(**request)
            self.request_at = time.monotonic()
            self.outcome.setText('指令已提交，等待执行结果')
            self.result.setPlainText('已提交，等待串口执行；请勿重复发送。')
        except ValueError as exc:
            self.outcome.setText('未提交：'+str(exc))
            self.result.setPlainText(str(exc))
        self.ensureWidgetVisible(self.outcome)

    def refresh(self):
        debug = getattr(self.adapter, 'debug', {})
        fresh = time.monotonic()-getattr(self.adapter, 'debug_received_at', 0) < 2
        mode = {'normal': '正常输出模式', 'config': '配置模式', 'unknown': '模式未确认'}.get(debug.get('mode'), '模式未确认')
        serial = getattr(self.adapter, 'serial', {})
        reason = getattr(self.adapter, 'debug_busy_reason', lambda: '当前连接只支持查看')()
        self.state.setText(f'{serial.get("port", "未选串口")} · {serial.get("baudrate", "--")} bps · {mode}\n'
                           + ('连接状态已过期' if not fresh else reason or ''))
        enabled = fresh and not reason and not debug.get('busy') and bool(serial.get('ok'))
        self.send_button.setEnabled(enabled)
        self.confirm_button.setEnabled(enabled)
        self.resume_button.setEnabled(enabled and bool(debug.get('exclusive')))
        rows = [row for row in debug.get('results', []) if row.get('id') == self.request_id]
        if rows:
            last = rows[-1]
            self.outcome.setText(result_summary(last))
            lines, replies = [], set()
            for row in rows:
                stamp = time.strftime('%H:%M:%S', time.localtime(row.get('wall_ns', 0)/1e9))
                if row.get('sent'):
                    lines.append(f'{stamp} 发送：{row["sent"]}')
                if row.get('reply'):
                    reply = row['reply']
                    # Terminal records retain the main reply as evidence, not
                    # an extra UART receipt. Show each actual step reply once.
                    if row.get('state') == 'running' or reply not in replies:
                        lines.append(f'{stamp} 收到：{reply}')
                    replies.add(reply)
            text = '\n'.join(lines) or result_summary(last)
            if self.result.toPlainText() != text:
                self.result.setPlainText(text)
        elif self.request_id and time.monotonic()-self.request_at > 5:
            self.outcome.setText('执行结果未收到；请勿重复危险操作')
            self.result.setPlainText('尚未收到执行结果，不能确定是否发送；请查看原始收发，不要重复危险操作。')


class RtkTrafficPanel(QWidget):
    def __init__(self, adapter, parent=None):
        super().__init__(parent)
        self.setStyleSheet(_CONSOLE_STYLE)
        self.adapter, self.session, self.sequence = adapter, '', 0
        self.records = deque(maxlen=1000)
        self.missed = 0
        self.last_render = 0.
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        self.direction = QComboBox()
        self.direction.addItems(['全部', 'RX', 'TX'])
        self.encoding = QComboBox()
        self.encoding.addItems(['文本', 'HEX'])
        self.filter = QLineEdit()
        self.filter.setPlaceholderText('筛选数据块内容，如 GGA / +VERSION')
        self.pause = QCheckBox('暂停显示')
        for widget in (self.direction, self.encoding, self.pause):
            controls.addWidget(widget)
        layout.addLayout(controls)
        layout.addWidget(self.filter)
        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(1100)
        layout.addWidget(self.view, 1)
        buttons = QHBoxLayout()
        for label, action in (('复制显示', self.copy), ('清空显示', self.clear), ('导出近期收发', self.export)):
            button = QPushButton(label)
            button.clicked.connect(action)
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.note = QLabel('按读取/写入数据块显示；完整原始数据另行落盘，暂停显示不停止接收。')
        self.note.setWordWrap(True)
        layout.addWidget(self.note)

    def copy(self):
        QApplication.clipboard().setText(self.view.toPlainText())

    def clear(self):
        self.records.clear()
        self.view.clear()

    def export(self):
        try:
            root = log_store().root.parent / 'log-exports'
            root.mkdir(parents=True, exist_ok=True, mode=0o700)
            path = root / f'rtk-preview-{time.time_ns()}.jsonl'
            with os.fdopen(os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'w') as stream:
                stream.write(json.dumps(dict(scope='recent_preview_only', session=self.session, missed=self.missed))+'\n')
                for row in self.records:
                    stream.write(json.dumps(row)+'\n')
            self.note.setText(f'已导出近期收发：{path}；完整会话请在日志管理中导出。')
        except OSError as exc:
            self.note.setText(f'导出失败：{exc}')

    def refresh(self):
        debug = getattr(self.adapter, 'debug', {})
        session = debug.get('session', '')
        if session != self.session:
            self.session, self.sequence = session, 0
            self.records.clear()
            self.missed = 0
        for row in debug.get('records', []):
            seq = row.get('seq', 0)
            if seq <= self.sequence:
                continue
            self.missed += max(0, seq-self.sequence-1)
            self.sequence = seq
            self.records.append(row)
        if self.pause.isChecked() or time.monotonic()-self.last_render < .25:
            return
        self.last_render = time.monotonic()
        lines = []
        for row in self.records:
            if self.direction.currentText() != '全部' and row.get('direction') != self.direction.currentText():
                continue
            try:
                data = base64.b64decode(row['base64'], validate=True)
                text = data.hex(' ') if self.encoding.currentText() == 'HEX' else data.decode('ascii', errors='backslashreplace').replace('\r', '\\r').replace('\n', '\\n')
                if self.filter.text() not in text:
                    continue
                stamp = row['wall_ns']/1e9
                clock = time.strftime('%H:%M:%S', time.localtime(stamp)) + f'.{row["wall_ns"]//1000000%1000:03d}'
                lines.append(f'{clock} {row["direction"]} {text}')
            except (ValueError, KeyError, TypeError):
                continue
        if self.missed:
            lines.insert(0, f'[实时预览缺少 {self.missed} 个数据块；完整性以落盘status.json为准]')
        self.view.setPlainText('\n'.join(lines))
        self.view.verticalScrollBar().setValue(self.view.verticalScrollBar().maximum())
