"""GUI-thread capture and input bridge; the web server never touches QWidget."""
import time
import os
import uuid
from pathlib import Path
from PyQt5 import sip
from PyQt5.QtCore import QObject, QTimer, Qt, QEvent, QPoint, QPointF, QBuffer, QIODevice
from PyQt5.QtGui import QMouseEvent, QKeyEvent, QInputMethodEvent
from PyQt5.QtNetwork import QNetworkInterface, QAbstractSocket
from PyQt5.QtWidgets import QApplication, QWidget, QAbstractScrollArea, QLineEdit, QTextEdit, QPlainTextEdit
from ..phone_remote import RemoteState, RemoteServer


def configured_public_url():
    """Public relay address for the phone page.

    The value is device configuration, not a per-launch secret: a manual
    ``ros2 launch`` does not inherit the service's Environment, so fall back
    to the same runtime.env that scripts/start_device_app.sh sources. Both
    startup paths then accept the same public Host on screen.
    """
    value = os.environ.get("NAV_PHONE_PUBLIC_URL", "").strip()
    if value:
        return value
    path = os.environ.get("NAV_RUNTIME_CONFIG", "") or str(
        Path.home() / ".config" / "nav" / "runtime.env")
    try:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line.startswith("export "):
                line = line[len("export "):].lstrip()
            if not line.startswith("NAV_PHONE_PUBLIC_URL="):
                continue
            return line.split("=", 1)[1].strip().strip('"').strip("'").strip()
    except OSError:
        pass
    return ""


class PhoneRemote(QObject):
    def __init__(self, window):
        super().__init__(window)
        self.window = window
        self.state = None
        self.server = None
        self.urls = []
        self.error = ""
        self.revision = 0
        self.injecting = False
        self.remote_robot = False
        self.remote_robot_generation = None
        self.previous_session = ""
        self.previous_interruptions = 0
        self.last_capture = 0.
        self.last_network_check = 0.
        self.bind_host = "0.0.0.0"
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)

    def owns(self, widget):
        if not isinstance(widget, QWidget) or sip.isdeleted(self.window) or sip.isdeleted(widget):
            return False
        roots = (self.window,)
        while widget is not None:
            if widget in roots:
                return True
            widget = widget.parentWidget()
        return False

    def eventFilter(self, obj, event):
        if (not self.injecting or event.spontaneous()) and event.type() in (QEvent.MouseButtonPress, QEvent.KeyPress, QEvent.Wheel, QEvent.TouchBegin) and self.owns(obj):
            self.revision += 1  # Local input wins over an older phone frame.
        return False

    @staticmethod
    def network_addresses():
        addresses = []
        for interface in QNetworkInterface.allInterfaces():
            if interface.flags() & QNetworkInterface.IsUp and not interface.flags() & QNetworkInterface.IsLoopBack:
                for entry in interface.addressEntries():
                    if entry.ip().protocol() == QAbstractSocket.IPv4Protocol:
                        addresses.append(entry.ip().toString())
        return sorted(set(addresses), key=lambda ip: (ip != "10.42.0.1", ip))

    def refresh_addresses(self):
        addresses = self.network_addresses() if self.bind_host == "0.0.0.0" else [self.bind_host]
        self.server.update_hosts(addresses)
        self.urls = [f"http://{ip}:{self.server.port}" for ip in addresses]
        if self.server.public_authority:
            self.urls.append("http://" + self.server.public_authority)
        self.last_network_check = time.monotonic()

    def start(self, host="0.0.0.0", port=8765):
        if self.server:
            return True
        self.bind_host = host
        addresses = self.network_addresses() if host == "0.0.0.0" else [host]
        self.state = RemoteState()
        try:
            self.server = RemoteServer(self.state, host, port, addresses + [host],
                                       public_url=configured_public_url(),
                                       outdoor=getattr(self.window.node,"outdoor",None),
                                       map_services=getattr(self.window.node,"map_services",None))
        except (OSError, ValueError, OverflowError):
            self.error = "无法开启，请检查监听地址和端口是否被占用"
            self.state = None
            return False
        self.refresh_addresses()
        self.error = ""
        self.previous_session = ""
        self.previous_interruptions = 0
        QApplication.instance().installEventFilter(self)
        self.timer.start(50)
        return True

    def target(self):
        # QFileDialog's completion popup can be a parentless Qt QListView.
        # If it is not ours, still inspect the active modal dialog before
        # considering the main window; the modal has not been dismissed.
        for target in (QApplication.activePopupWidget(), QApplication.activeModalWidget(),
                       QApplication.activeWindow()):
            if self.owns(target) and target.isVisible():
                return target
        return self.window if self.window.isVisible() else None

    @staticmethod
    def focused_widget(target):
        focus = QApplication.focusWidget()
        if focus is not None and (focus is target or target.isAncestorOf(focus)):
            return focus
        # Offscreen Qt and freshly shown modal dialogs can retain their child
        # focus locally before QApplication.focusWidget() catches up.
        focus = target.focusWidget()
        return focus if focus is not None and (focus is target or target.isAncestorOf(focus)) else None

    @staticmethod
    def widget_identity(widget):
        if widget is None:
            return ""
        # Qt-created children (for example QFileDialog's filename edit) may
        # acquire a new SIP wrapper on every lookup. A Python id is therefore
        # not a widget identity. Keep the token on the underlying QObject so
        # rewrapping is stable and a replacement widget still gets a new token.
        name = "_nav_phone_remote_identity"
        identity = widget.property(name)
        if not identity:
            identity = uuid.uuid4().hex
            widget.setProperty(name, identity)
        return identity

    def signature(self, target):
        # Focus identity is deliberately excluded: opening a dialog or applying
        # a remote tap moves focus, and a gesture aimed at visible coordinates
        # stays valid across that move. Text/key input is still bound to the
        # published field token (frame[8]) in dispatch_input.
        node = self.window.node
        return (self.widget_identity(target), target.width(), target.height(), self.revision,
                self.window.pages.currentIndex(), getattr(node, "operating_mode", "walking"),
                getattr(node.walking, "phase", "idle"), getattr(getattr(node, "robot", None), "phase", "idle"),
                self.window.operations_page.controller.recording,
                self.window.operations_page.controller.playing,
                self.window.mapping_controller.process.active,
                self.window.navigation_stack_controller.overall_state)

    def editable(self, target):
        focus = self.focused_widget(target)
        return (isinstance(focus, (QLineEdit, QTextEdit, QPlainTextEdit))
                and focus.isEnabled() and not focus.isReadOnly())

    def cancel_remote_robot(self):
        if self.remote_robot:
            self.remote_robot = False
            if self.window.node.robot.busy and self.remote_robot_generation == self.window.node.robot.generation:
                self.window.node.cancel_navigation()
                self.window.pages.setCurrentWidget(self.window.active_page)

    def tick(self):
        if not self.server:
            return
        if time.monotonic() - self.last_network_check >= 2:
            self.refresh_addresses()
        robot = getattr(self.window.node, "robot", None)
        if self.remote_robot and robot and (not robot.busy or robot.generation != self.remote_robot_generation):
            self.remote_robot = False
        session = self.state.controller()
        if ((self.previous_session and session != self.previous_session)
                or self.previous_interruptions != self.state.interruptions):
            self.cancel_remote_robot()
        self.previous_interruptions = self.state.interruptions
        self.previous_session = session
        self.state.refresh_code()
        if not self.state.live():
            return
        for token, frame, command in self.state.take():
            # A QTimer cannot fire again while its timeout slot is in exec_().
            # Dispatch outside this timer so modal dialogs keep receiving both
            # frame captures and subsequent remote input in their nested loop.
            QTimer.singleShot(0, lambda t=token, f=frame, c=command: self.dispatch_input(t, f, c))
        now = time.monotonic()
        if not self.state.wants_frame() or now - self.last_capture < .125:
            return
        target = self.target()
        if target is None:
            return
        buffer = QBuffer()
        buffer.open(QIODevice.WriteOnly)
        # Capture this application only, including its Qt modal dialogs/popups.
        pixmap = target.grab()
        if pixmap.width() > 1280 or pixmap.height() > 1280:
            pixmap = pixmap.scaled(1280, 1280, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        if pixmap.save(buffer, "JPEG", 75):
            focus = self.focused_widget(target)
            editable = self.editable(target)
            sensitive = (editable and isinstance(focus, QLineEdit) and
                         (focus.echoMode() != QLineEdit.Normal or focus.property("navSensitive")))
            self.state.publish(bytes(buffer.data()), self.signature(target), target.width(), target.height(), editable,
                               "password" if sensitive else "text",
                               self.widget_identity(focus) if focus is not None else "")
        self.last_capture = now

    def dispatch_input(self, token, frame, command):
        if self.state is None or sip.isdeleted(self.window):
            return
        target = self.target()
        if (token != self.state.controller() or not target or frame[2] != self.signature(target)
                or time.monotonic() - frame[1] > 2):
            self.state.notice = "画面已变化，请重试"
            self.state.processed = max(self.state.processed, frame[0])
            return
        if command.get("kind") in ("text", "key"):
            # Text and keys act on the focused field rather than on visible
            # coordinates, so they still require the exact published field.
            if frame[8] != self.widget_identity(self.focused_widget(target)):
                self.state.notice = "画面已变化，请重试"
                self.state.processed = max(self.state.processed, frame[0])
                return
        before = bool(getattr(self.window.node, "robot", None) and self.window.node.robot.busy)
        self.state.processed = max(self.state.processed, frame[0])
        self.revision += 1
        self.state.notice = ""
        previous_injecting, self.injecting = self.injecting, True
        try:
            self.apply(target, command)
        finally:
            self.injecting = previous_injecting
        if self.state is None:
            return
        if not before and getattr(self.window.node, "robot", None) and self.window.node.robot.busy:
            self.remote_robot = True
            self.remote_robot_generation = self.window.node.robot.generation

    def apply(self, target, command):
        kind = command["kind"]
        if kind == "gesture":
            points = [QPoint(round(p[0] * (target.width() - 1)), round(p[1] * (target.height() - 1))) for p in command["points"]]
            widget = target.childAt(points[0]) or target
            if not widget.isEnabled():
                return
            # A QWebEngineView does not forward mouse events itself; page input
            # is handled by an internal delegate that is recreated while a page
            # (re)loads. Resolve through it so a tap in that window is not lost.
            if widget.metaObject().className() == "QWebEngineView":
                local = widget.mapFrom(target, points[0])
                widget = widget.childAt(local) or widget.focusProxy() or widget
            # Lists and maintenance forms scroll without a synthetic long press.
            area = widget
            while area is not None and not isinstance(area, QAbstractScrollArea) and area is not target:
                area = area.parentWidget()
            delta = points[-1] - points[0]
            if isinstance(area, QAbstractScrollArea) and delta.manhattanLength() > 12:
                area.verticalScrollBar().setValue(area.verticalScrollBar().value() - delta.y())
                area.horizontalScrollBar().setValue(area.horizontalScrollBar().value() - delta.x())
                return
            # sendEvent bypasses QApplication's native mouse focus assignment.
            # Without this, a remote tap moves the cursor but cannot type in a field.
            QApplication.setActiveWindow(target.window())
            if widget.focusPolicy() & Qt.ClickFocus:
                widget.setFocus(Qt.MouseFocusReason)
            editor = getattr(self.window, 'touch_input', None)
            if editor is not None and delta.manhattanLength() <= 12 and editor.eligible(widget):
                # The phone has already delivered a complete tap. Open now,
                # without a second delayed mouse/scroll gesture recognition.
                editor.open_editor(widget)
                return
            # A complete gesture is delivered in one GUI turn: never leave a held button after a lost packet.
            def mouse(kind, point, buttons):
                local = widget.mapFrom(target, point)
                event = QMouseEvent(kind, QPointF(local), QPointF(widget.mapToGlobal(local)),
                                    Qt.NoButton if kind == QEvent.MouseMove else Qt.LeftButton,
                                    buttons, Qt.NoModifier)
                # Chromium counts rapid nearby presses as one double click when
                # their timestamps look simultaneous. Without a real timestamp
                # every synthetic event reads as 0, so consecutive multi-point
                # selections became map zooms. Stamp the actual delivery time.
                event.setTimestamp(int(time.monotonic() * 1000))
                QApplication.sendEvent(widget, event)
            mouse(QEvent.MouseButtonPress, points[0], Qt.LeftButton)
            for point in points[1:]:
                mouse(QEvent.MouseMove, point, Qt.LeftButton)
            mouse(QEvent.MouseButtonRelease, points[-1], Qt.NoButton)
        else:
            focus = self.focused_widget(target)
            if focus is None:
                return
            if kind == "text" and self.editable(target):
                if command.get('replace'):
                    focus.selectAll()
                    if not command['text']:
                        # Qt ignores an empty IME commit even with a selection.
                        focus.clear()
                event = QInputMethodEvent()
                event.setCommitString(command["text"])
                QApplication.sendEvent(focus, event)
                if command.get('commit'):
                    QApplication.sendEvent(focus, QKeyEvent(QEvent.KeyPress, Qt.Key_Return, Qt.NoModifier))
                    # Saving a touch draft may close its dialog; never send a
                    # follow-up key to a new focused field or default button.
            elif kind == "key":
                key = {"Enter": Qt.Key_Return, "Escape": Qt.Key_Escape, "Backspace": Qt.Key_Backspace}[command["key"]]
                QApplication.sendEvent(focus, QKeyEvent(QEvent.KeyPress, key, Qt.NoModifier))
                QApplication.sendEvent(focus, QKeyEvent(QEvent.KeyRelease, key, Qt.NoModifier))

    def revoke(self):
        if self.state:
            self.state.revoke()
            self.cancel_remote_robot()

    def close(self):
        self.timer.stop()
        QApplication.instance().removeEventFilter(self)
        self.cancel_remote_robot()
        if self.server:
            self.server.close()
            self.server = None
        self.state = None
        self.previous_session = ""
