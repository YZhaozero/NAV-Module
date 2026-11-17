"""RTK trajectory recording with geographic and local-map previews."""

import math
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QComboBox, QHBoxLayout, QLabel, QPushButton, QSlider, QStackedWidget,
    QTabWidget, QVBoxLayout, QWidget)

from .background_task import BackgroundTask
from .trajectory_map import TrajectoryMap
from ..features.trajectory_test import export_session, load_session, tracks_for


class TrajectoryCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(240, 180)
        self.tracks = {}
        self.grid = None
        self.cloud = None
        self.zoom = 1.
        self.pan = QPointF()
        self.down = None
        self.grid_identity = None
        self.grid_image = None

    def mousePressEvent(self, event):
        self.down = event.localPos()

    def mouseMoveEvent(self, event):
        if self.down is not None:
            self.pan += event.localPos() - self.down
            self.down = event.localPos()
            self.update()

    def mouseReleaseEvent(self, event):
        del event
        self.down = None

    def fit(self):
        self.zoom = 1.
        self.pan = QPointF()
        self.update()

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#071923"))
        points = [
            point[:2] for track in self.tracks.values()
            for point in track if point is not None]
        if self.cloud is not None:
            points.extend(self.cloud[::max(1, len(self.cloud) // 2000), :2].tolist())
        corners = None
        if self.grid is not None:
            grid, origin, resolution = self.grid
            height, width = grid.shape
            angle = origin[2]
            rotation = np.array([
                [math.cos(angle), -math.sin(angle)],
                [math.sin(angle), math.cos(angle)],
            ])
            corners = np.array([
                [0, 0], [width * resolution, 0],
                [width * resolution, height * resolution],
                [0, height * resolution],
            ]) @ rotation.T + origin[:2]
            points.extend(corners.tolist())
        if not points:
            painter.setPen(QColor("#bdd8df"))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待轨迹或地图数据")
            return
        values = np.asarray(points)
        lower, upper = values.min(0), values.max(0)
        center = (lower + upper) / 2
        scale = min(
            (self.width() - 30) / max(1., upper[0] - lower[0]),
            (self.height() - 30) / max(1., upper[1] - lower[1]),
        ) * self.zoom

        def screen(point):
            return QPointF(
                self.width() / 2 + (point[0] - center[0]) * scale + self.pan.x(),
                self.height() / 2 - (point[1] - center[1]) * scale + self.pan.y(),
            )

        if corners is not None:
            grid, origin, resolution = self.grid
            height, width = grid.shape
            if self.grid_identity is not grid:
                rgb = np.full((*grid.shape, 3), 70, dtype=np.uint8)
                rgb[grid == 0] = 190
                rgb[grid > 50] = 15
                rgb = np.ascontiguousarray(np.flipud(rgb))
                self.grid_image = QImage(
                    rgb.data, width, height, rgb.strides[0],
                    QImage.Format_RGB888).copy()
                self.grid_identity = grid
            painter.save()
            painter.translate(screen(corners[3]))
            painter.rotate(-math.degrees(origin[2]))
            painter.scale(scale * resolution, scale * resolution)
            painter.drawImage(QRectF(0, 0, width, height), self.grid_image)
            painter.restore()
        if self.cloud is not None:
            painter.setPen(QPen(QColor("#66818c"), 1))
            step = max(1, len(self.cloud) // 15000)
            painter.drawPoints(QPolygonF([screen(point) for point in self.cloud[::step]]))
        for key, color in (
            ("odometry", "#5eb6ff"),
            ("localization", "#ffca69"),
            ("gnss", "#42e5b4"),
        ):
            painter.setPen(QPen(QColor(color), 2))
            last = None
            for point in self.tracks.get(key, []):
                if point is None:
                    last = None
                    continue
                value = screen(point)
                if last is not None:
                    painter.drawLine(last, value)
                last = value
            if last is not None:
                painter.drawEllipse(last, 4, 4)
        painter.setPen(QColor("#cde5ef"))
        painter.drawText(10, self.height() - 8, f"比例：100 px ≈ {100 / scale:.1f} m")


class TrajectoryPage(QWidget):
    def __init__(self, operations, parent=None):
        super().__init__(parent)
        self.operations = operations
        self.window = operations.window
        self.node = self.window.node
        self.recorder = getattr(self.node, "trajectory_test", None)
        self.worker = BackgroundTask(self)
        self.preview_worker = BackgroundTask(self)
        self.replay_data = None
        self.session_path = None
        self.assets = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)

        live = QWidget()
        body = QVBoxLayout(live)
        body.setContentsMargins(0, 0, 0, 0)
        self.tabs.addTab(live, "轨迹")

        controls = QHBoxLayout()
        self.view = QComboBox()
        self.view.addItems(["百度地图", "地图 / 里程计对比"])
        self.context = QComboBox()
        self.context.addItems(["地图包坐标", "建图 / 里程计坐标"])
        self.context.setCurrentIndex(1)
        controls.addWidget(self.view)
        controls.addWidget(self.context)
        self.start_button = self._button("开始记录", self.start, "primaryButton")
        self.align_button = self._button("冻结对齐", self.align, "secondaryButton")
        self.stop_button = self._button("结束记录", self.stop, "dangerButton")
        controls.addWidget(self.start_button)
        controls.addWidget(self.align_button)
        controls.addWidget(self.stop_button)
        body.addLayout(controls)

        map_row = QHBoxLayout()
        self.maps = QComboBox()
        map_row.addWidget(self.maps, 1)
        self.load_map_button = self._button("加载地图", self.load_selected_map, "secondaryButton")
        map_row.addWidget(self.load_map_button)
        body.addLayout(map_row)

        self.status = QLabel("未记录 · 轨迹实验不参与导航融合")
        self.status.setWordWrap(True)
        self.status.setObjectName("statusBar")
        body.addWidget(self.status)

        self.canvas = TrajectoryCanvas(self)
        self.baidu_map = TrajectoryMap(operations)
        self.views = QStackedWidget()
        self.views.addWidget(self.baidu_map)
        self.views.addWidget(self.canvas)
        body.addWidget(self.views, 1)

        self.legend = QLabel("绿：GNSS · 蓝：里程计 · 黄：地图定位")
        self.legend.setWordWrap(True)
        body.addWidget(self.legend)

        view_row = QHBoxLayout()
        self.layer = QComboBox()
        self.layer.addItems(["栅格", "点云", "仅轨迹"])
        view_row.addWidget(self.layer)
        view_row.addWidget(self._button("−", lambda: self.zoom(.8), "secondaryButton"))
        view_row.addWidget(self._button("+", lambda: self.zoom(1.25), "secondaryButton"))
        self.fit_button = self._button("刷新位置", self.fit_view, "secondaryButton")
        view_row.addWidget(self.fit_button)
        body.addLayout(view_row)

        self.time_slider = QSlider(Qt.Horizontal)
        self.time_slider.setRange(0, 1000)
        self.time_slider.setValue(1000)
        self.time_slider.valueChanged.connect(lambda _value: self.refresh(force=True))
        body.addWidget(self.time_slider)
        self.summary = QLabel("")
        self.summary.setWordWrap(True)
        body.addWidget(self.summary)

        history = QWidget()
        history_layout = QVBoxLayout(history)
        self.tabs.addTab(history, "记录与回看")
        self.tabs.currentChanged.connect(
            lambda index: self.scan() if index == 1 else None)
        help_text = QLabel(
            "百度地图用于查看卫星位置和轨迹；地图 / 里程计对比可加载所选地图包的栅格或点云。"
            "底图需联网，地图加载失败不影响原始轨迹记录。")
        help_text.setWordWrap(True)
        history_layout.addWidget(help_text)
        self.sessions = QComboBox()
        history_layout.addWidget(self.sessions)
        history_actions = QHBoxLayout()
        history_actions.addWidget(self._button("刷新记录", self.scan, "secondaryButton"))
        history_actions.addWidget(self._button("回看", self.replay, "secondaryButton"))
        history_actions.addWidget(self._button("返回实时", self.live, "secondaryButton"))
        history_actions.addWidget(self._button("导出 ZIP", self.export, "secondaryButton"))
        history_layout.addLayout(history_actions)
        self.file_status = QLabel("")
        self.file_status.setWordWrap(True)
        history_layout.addWidget(self.file_status)
        history_layout.addStretch(1)

        self.view.currentIndexChanged.connect(self.change_view)
        self.context.currentIndexChanged.connect(self.context_changed)
        self.layer.currentIndexChanged.connect(lambda _value: self.refresh(force=True))
        self.refresh_maps()
        self.change_view(0)
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        self.scan()

    @staticmethod
    def _button(text, callback, object_name):
        button = QPushButton(text)
        button.setObjectName(object_name)
        button.clicked.connect(callback)
        return button

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh_maps()
        self.scan()
        self.refresh(force=True)

    def refresh_maps(self):
        self.operations.library.refresh(preserve_edits=True)
        selected = self.maps.currentData()
        selected_manifest = str(selected.manifest) if selected and selected.manifest else ""
        self.maps.clear()
        for scene in self.operations.library.scenes:
            self.maps.addItem(f"{scene.title} · {scene.summary}", scene)
            if scene.manifest and str(scene.manifest) == selected_manifest:
                self.maps.setCurrentIndex(self.maps.count() - 1)

    def selected_scene(self):
        scene = self.maps.currentData()
        if scene is None:
            raise ValueError("没有可加载的地图包，请先在地图包页面登记地图")
        if not scene.grid and not scene.cloud:
            raise ValueError("所选地图包没有栅格或点云")
        return scene

    def context_changed(self):
        show = self.view.currentIndex() == 1 and self.context.currentIndex() == 0
        self.maps.setVisible(show)
        self.load_map_button.setVisible(show)

    def change_view(self, index):
        self.views.setCurrentIndex(index)
        local = index == 1
        for widget in (self.context, self.layer, self.legend, self.summary):
            widget.setVisible(local)
        self.time_slider.setVisible(local or self.replay_data is not None)
        self.fit_button.setText("刷新位置" if index == 0 else "全览")
        self.context_changed()
        self.refresh(force=True)

    def zoom(self, factor):
        if self.view.currentIndex() == 0:
            self.baidu_map.reset_view(1 if factor > 1 else -1)
            self.refresh(force=True)
            return
        self.canvas.zoom = max(.2, min(20., self.canvas.zoom * factor))
        self.canvas.update()

    def fit_view(self):
        if self.view.currentIndex() == 0:
            self.baidu_map.reset_view()
            self.refresh(force=True)
        else:
            self.canvas.fit()

    def _operation(self, work, done):
        if self.worker.busy:
            return
        self.file_status.setText("正在处理…")

        def finish(value, error):
            self.file_status.setText(error or "完成")
            if error:
                self.status.setText(error)
            else:
                done(value)
            self.refresh(force=True)

        self.worker.start(work, finish)

    @staticmethod
    def _scene_config(scene):
        return dict(
            context="map",
            map_frame=scene.frame,
            map_title=scene.title,
            cloud_path=scene.cloud,
            grid_path=scene.grid,
        )

    def load_selected_map(self):
        try:
            scene = self.selected_scene()
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        config = self._scene_config(scene)

        def done(assets):
            self.assets = assets
            self.status.setText(f"已加载地图：{scene.title}")
            self._apply_assets("map")

        self._operation(lambda: self.load_assets(config), done)

    def start(self):
        if self.recorder is None:
            self.status.setText("轨迹记录服务未连接")
            return
        scene = None
        if self.view.currentIndex() == 1 and self.context.currentIndex() == 0:
            try:
                scene = self.selected_scene()
            except ValueError as exc:
                self.status.setText(str(exc))
                return
        source = getattr(self.node, "_odometry_backend", "fast_livo")
        if self.context.currentIndex() == 1:
            backend = getattr(self.window.mapping_controller, "backend", None)
            source = getattr(backend, "key", source)
        config = self._scene_config(scene) if scene else dict(context="mapping")
        config.update(
            source=source,
            recorder_version=1,
            protocol="initial-anchor/frozen-heading/evaluation",
            calibration_note=(
                "Time offset 0 s is unverified; antenna lever arm unknown"),
        )
        outdoor = getattr(self.node, "outdoor", None)
        if outdoor is not None:
            calibration = dict(outdoor.calibration)
            config["calibration_metadata"] = calibration
            if calibration.get("time_verified"):
                config["gnss_to_odom_offset_s"] = calibration["gnss_to_odom_offset_s"]
            if calibration.get("lever_arm_verified"):
                config["antenna_in_body_m"] = calibration["antenna_in_body_m"]
                config["odom_child_frame"] = calibration["odom_child_frame"]

        def work():
            if scene and scene.cloud:
                from ..geo_maps import file_hash
                config["map_sha256"] = file_hash(scene.cloud)
            path = self.recorder.call("start", config)
            return path, self.load_assets(config, path)

        def done(value):
            self.session_path, self.assets = value
            self.replay_data = None
            self.time_slider.setValue(1000)
            if config["context"] == "mapping":
                self.layer.setCurrentIndex(1)
            self.status.setText(f"正在记录：{self.session_path}")

        self._operation(work, done)

    @staticmethod
    def load_assets(config, path=None):
        from ..map_processing import load_grid_map, load_pointcloud
        result = {}
        snapshot = Path(path) / "map-preview.npz" if path else None
        if snapshot and snapshot.is_file() and not snapshot.is_symlink():
            with np.load(snapshot, allow_pickle=False) as assets:
                if "occupancy" in assets:
                    result["grid"] = (
                        assets["occupancy"], assets["origin"],
                        float(assets["resolution"]))
                if "cloud" in assets:
                    result["cloud"] = assets["cloud"]
            return result
        if path:
            return result
        if config.get("grid_path"):
            grid = load_grid_map(config["grid_path"])
            result["grid"] = (grid.occupancy, grid.origin, grid.resolution)
        if config.get("cloud_path"):
            cloud = load_pointcloud(config["cloud_path"]).points
            result["cloud"] = cloud[::max(1, len(cloud) // 20000)].copy()
        return result

    def align(self):
        if self.recorder is not None:
            self._operation(
                lambda: self.recorder.call("align"),
                lambda value: self.status.setText(
                    f"地理对齐已冻结 · 残差 {value['residual_m']:.2f} m"))

    def stop(self):
        if self.recorder is None:
            return
        mapping = getattr(self.node, "mapping", None)
        cloud = getattr(mapping, "cloud", None)
        frame = getattr(mapping, "cloud_frame", "")

        def work():
            if cloud is not None:
                self.recorder.call("preview", dict(cloud=cloud, frame=frame))
            report = self.recorder.call("stop")
            assets = self.load_assets(
                self.recorder.manifest, str(self.recorder.session))
            return report, assets

        def done(value):
            self.assets = value[1]
            self.status.setText("记录已结束")
            self.scan()

        self._operation(work, done)

    def scan(self):
        if self.recorder is None or self.worker.busy:
            return

        def read():
            return sorted((
                path for path in self.recorder.root.glob("*")
                if path.is_dir() and not path.is_symlink()
                and (path / "manifest.json").is_file()), reverse=True)

        def update(paths):
            selected = self.sessions.currentData()
            self.sessions.clear()
            for path in paths:
                self.sessions.addItem(path.name, str(path))
                if str(path) == selected:
                    self.sessions.setCurrentIndex(self.sessions.count() - 1)

        self._operation(read, update)

    def replay(self):
        path = self.sessions.currentData()
        if not path:
            return
        if self.recorder is not None and self.recorder.active:
            self.file_status.setText("请先结束当前测试，再回看历史")
            return

        def work():
            data = load_session(path)
            return data, self.load_assets(data["manifest"], path)

        def done(value):
            self.replay_data, self.assets = value
            self.session_path = path
            self.time_slider.setValue(1000)
            self.tabs.setCurrentIndex(0)
            self.file_status.setText(path)

        self._operation(work, done)

    def live(self):
        self.replay_data = None
        self.time_slider.setValue(1000)
        self.tabs.setCurrentIndex(0)
        if self.recorder is not None and self.recorder.session:
            path = str(self.recorder.session)
            manifest = dict(self.recorder.manifest)
            self._operation(
                lambda: self.load_assets(manifest, path),
                lambda assets: setattr(self, "assets", assets))

    def export(self):
        path = self.sessions.currentData() or self.session_path
        if not path:
            return
        if self.recorder is not None and self.recorder.active:
            self.file_status.setText("请先结束记录，再导出完整测试包")
            return
        self._operation(
            lambda: export_session(path),
            lambda target: self.file_status.setText(f"已导出：{target}"))

    def _apply_assets(self, frame):
        self.canvas.grid = (
            self.assets.get("grid")
            if frame == "map" and self.layer.currentIndex() == 0 else None)
        self.canvas.cloud = (
            self.assets.get("cloud")
            if frame == "map" and self.layer.currentIndex() == 1 else None)
        self.canvas.update()

    def refresh(self, force=False):
        if not self.isVisible() and not force:
            return
        if self.preview_worker.busy:
            return
        snapshot = self.recorder.snapshot() if self.recorder is not None else dict(
            active=False, manifest={}, samples={}, error="轨迹记录服务未连接",
            dropped=0, session="")
        active = snapshot["active"]
        manifest = snapshot["manifest"]
        self.start_button.setEnabled(not active and not self.worker.busy and self.recorder is not None)
        self.align_button.setEnabled(active and not manifest.get("alignment") and not self.worker.busy)
        self.stop_button.setEnabled(active and not self.worker.busy)
        self.context.setEnabled(not active)
        self.maps.setEnabled(not active)
        self.time_slider.setEnabled(not active)
        data = self.replay_data or dict(snapshot["samples"], manifest=manifest)
        if self.view.currentIndex() == 0:
            history = self.replay_data is not None
            self.time_slider.setVisible(history)
            self.baidu_map.refresh(
                data, history, self.time_slider.value() / 1000,
                self.session_path if history else snapshot.get("session"))
            state = "回看记录" if history else "记录中" if active else "实时位置 · 未记录"
            self.status.setText(snapshot["error"] or f"{state} · 丢失 {snapshot['dropped']} 条")
            return
        self.time_slider.setVisible(True)
        if not manifest:
            self._apply_assets("map")
            return
        slider = self.time_slider.value()

        def work():
            stamps = [row["stamp"] for row in data["gnss"]
                      if isinstance(row.get("stamp"), (float, int))]
            until = (min(stamps) + (max(stamps) - min(stamps)) * slider / 1000
                     if stamps else None)
            return tracks_for(data, until)

        def done(value, error):
            if error:
                self.status.setText(error)
                return
            tracks, report, frame = value
            self.canvas.tracks = tracks
            self._apply_assets(frame)
            label = {"map": "地图包坐标", "odom": "建图 / 里程计坐标",
                     "enu": "GNSS ENU（方向待对齐）"}[frame]
            state = "回看" if self.replay_data else "记录中" if active else "已结束"
            self.status.setText(snapshot["error"] or f"{state} · {label} · 丢失 {snapshot['dropped']} 条")
            detail = (f"时间配对 {report['paired']}/{report['total']}"
                      f" · 评估 {report['evaluation_count']} 点")
            if "p95_m" in report:
                detail += f" · 相对差异 P95 {report['p95_m']:.2f} m"
            if manifest.get("antenna_in_body_m") is None:
                detail += " · 杆臂未标定"
            if manifest.get("epoch_changed") or manifest.get("map_changed"):
                self.status.setText("地图或里程计坐标已变化，请结束并重新开始测试")
            self.summary.setText(detail)

        self.preview_worker.start(work, done)

    def shutdown(self):
        self.timer.stop()
