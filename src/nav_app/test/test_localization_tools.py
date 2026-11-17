import os
import hashlib
import json
import math
from pathlib import Path
import struct
import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtCore import QObject, pyqtSignal  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

import nav_app.features.localization_tools as tools_module  # noqa: E402
from nav_app.features.localization_tools import (  # noqa: E402
    DEFAULT_GLOBAL_RELOCALIZE_SERVICE,
    LocalizationToolsController,
    LocalizationRosAdapter,
    initial_pose_from_points,
    scan_context_path,
    validate_scan_context_cache,
    validate_scan_context_source,
)


class FakeProcess(QObject):
    log_received = pyqtSignal(str)
    state_changed = pyqtSignal(str)

    def __init__(self, _label, parent=None):
        super().__init__(parent)
        self.active = False
        self.started = []

    def start_launch(self, arguments):
        self.started.append(list(arguments))
        self.active = True
        self.state_changed.emit("running")
        return True

    def finish(self, state="stopped"):
        self.active = False
        self.state_changed.emit(state)

    def stop_launch(self):
        self.state_changed.emit("stopping")
        self.active = False
        self.state_changed.emit("stopped")

    def shutdown(self):
        self.active = False


class FakeRosAdapter:
    def __init__(self):
        self.calls = []
        self.done = None
        self.initial_calls = []
        self.initial_done = None
        self.timeout_calls = []

    def trigger_global_relocalization(self, service_name, done, timeout_s=120):
        self.calls.append(service_name)
        self.timeout_calls.append(("global", timeout_s))
        self.done = done
        return True, f"已请求全局重定位：{service_name}"

    def cancel_global_relocalization_requests(self, reason):
        if self.done is None:
            return False
        done, self.done = self.done, None
        done(False, reason)
        return True

    def initial_pose_readiness(self, frame_id, expected_map_sha256=""):
        return True, "ready"

    def publish_initial_pose(
        self, frame_id, x, y, yaw, expected_map_sha256, done, timeout_s=120,
    ):
        self.initial_calls.append(
            (frame_id, x, y, yaw, expected_map_sha256))
        self.initial_done = done
        self.timeout_calls.append(("manual", timeout_s))
        return True, "已提交大概位置和方向，等待点云匹配确认"


def test_scan_context_source_requires_existing_pcd(tmp_path):
    valid, message = validate_scan_context_source("")
    assert not valid
    assert "定位 PCD" in message

    ply = tmp_path / "map.ply"
    ply.write_bytes(b"ply")
    valid, message = validate_scan_context_source(str(ply))
    assert not valid
    assert "PCD" in message

    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd")
    valid, resolved = validate_scan_context_source(str(pcd))
    assert valid
    assert resolved == str(pcd.resolve())
    assert scan_context_path(resolved) == Path(str(pcd.resolve()) + ".sc.v2")


def test_two_point_initial_pose_uses_first_point_and_forward_direction():
    assert initial_pose_from_points((2, 3), (3, 3)) == (2.0, 3.0, 0.0)
    assert initial_pose_from_points((2, 3), (2, 4))[2] == pytest.approx(math.pi / 2)
    assert abs(initial_pose_from_points((2, 3), (1, 3))[2]) == pytest.approx(math.pi)
    with pytest.raises(ValueError, match="至少"):
        initial_pose_from_points((1, 1), (1.1, 1.1), .3)
    with pytest.raises(ValueError, match="有效"):
        initial_pose_from_points((float("nan"), 1), (2, 2))


def write_scan_context_cache(path, pcd):
    rings, sectors, entries = 20, 60, 1
    payload = (
        struct.pack("<fffii", 1.0, 2.0, 0.0, rings, sectors)
        + np.zeros(rings * sectors, dtype="<f8").tobytes()
    )
    header = struct.pack(
        "<8sII32sddIIQ32s",
        b"GSSCC002",
        2,
        0x01020304,
        hashlib.sha256(pcd.read_bytes()).digest(),
        80.0,
        2.0,
        rings,
        sectors,
        entries,
        hashlib.sha256(payload).digest(),
    )
    path.write_bytes(header + payload)


def test_scan_context_cache_must_belong_to_selected_pcd(tmp_path):
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd-map")
    cache = tmp_path / "chosen.sc.v2"
    write_scan_context_cache(cache, pcd)

    valid, resolved = validate_scan_context_cache(str(pcd), str(cache))
    assert valid and resolved == str(cache.resolve())

    pcd.write_bytes(b"changed-map")
    valid, message = validate_scan_context_cache(str(pcd), str(cache))
    assert not valid and "不匹配" in message


def test_scan_context_progress_cancellation_and_changed_file_do_not_cache_success(tmp_path):
    pcd = tmp_path/'map.pcd';pcd.write_bytes(b'a' * (2*1024*1024))
    cache = tmp_path/'map.sc.v2';write_scan_context_cache(cache, pcd)
    progress = []
    valid, reason = validate_scan_context_cache(str(pcd), str(cache),
        progress=lambda done, total: progress.append((done, total)),
        cancelled=lambda: len(progress) > 2)
    assert not valid and '取消' in reason
    assert progress[-1][0] < progress[-1][1]
    # A cancelled attempt must never populate a success cache.
    valid, _ = validate_scan_context_cache(str(pcd), str(cache))
    assert valid


def test_scan_context_cache_rejects_entry_shape_and_parameter_mismatch(tmp_path):
    pcd = tmp_path / "map.pcd"
    pcd.write_bytes(b"pcd-map")
    cache = tmp_path / "chosen.sc.v2"
    write_scan_context_cache(cache, pcd)

    document = bytearray(cache.read_bytes())
    struct.pack_into("<ii", document, 112 + 12, 0, 0)
    document[80:112] = hashlib.sha256(document[112:]).digest()
    cache.write_bytes(document)
    valid, message = validate_scan_context_cache(str(pcd), str(cache))
    assert not valid and "条目" in message

    write_scan_context_cache(cache, pcd)
    valid, message = validate_scan_context_cache(
        str(pcd), str(cache), expected_radius=40.0, expected_grid=2.0)
    assert not valid and "最大半径" in message


def test_controller_runs_generate_sc_tool_and_verifies_output(
    tmp_path, monkeypatch,
):
    app = QApplication.instance() or QApplication([])
    assert app is not None
    monkeypatch.setattr(tools_module, "RosLaunchProcess", FakeProcess)
    pcd = tmp_path / "location.pcd"
    pcd.write_bytes(b"pcd")
    controller = LocalizationToolsController(FakeRosAdapter())
    states = []
    controller.operation_state_changed.connect(
        lambda operation, state: states.append((operation, state)))

    success, _message = controller.generate_scan_context(str(pcd))
    assert success
    assert controller.sc_process.started == [[
        "run", "localizer", "generate_sc_tool", str(pcd.resolve())]]

    scan_context_path(str(pcd)).write_bytes(b"scan-context")
    controller.sc_process.finish()
    assert states[-1] == ("scan_context", "success")
    assert ".pcd.sc" in controller.status


def test_controller_calls_configurable_global_relocalize_service(monkeypatch):
    app = QApplication.instance() or QApplication([])
    assert app is not None
    monkeypatch.setattr(tools_module, "RosLaunchProcess", FakeProcess)
    adapter = FakeRosAdapter()
    controller = LocalizationToolsController(adapter)
    states = []
    controller.operation_state_changed.connect(
        lambda operation, state: states.append((operation, state)))

    success, _message = controller.trigger_global_relocalization(
        DEFAULT_GLOBAL_RELOCALIZE_SERVICE)
    assert success
    assert adapter.calls == [DEFAULT_GLOBAL_RELOCALIZE_SERVICE]
    assert controller.global_relocalize_pending

    adapter.done(True, "重定位任务已开始")
    assert not controller.global_relocalize_pending
    assert states[-1] == ("global_relocalize", "success")
    assert controller.status == "重定位任务已开始"


def test_controller_tracks_manual_two_point_relocalization(monkeypatch):
    app = QApplication.instance() or QApplication([])
    assert app is not None
    monkeypatch.setattr(tools_module, "RosLaunchProcess", FakeProcess)
    adapter = FakeRosAdapter()
    controller = LocalizationToolsController(adapter)
    states = []
    controller.operation_state_changed.connect(
        lambda operation, state: states.append((operation, state)))

    accepted, message = controller.submit_initial_pose(
        "map", (4.0, 5.0), (4.0, 7.0), "a" * 64)

    assert accepted and "等待点云" in message
    assert controller.manual_relocalize_pending
    frame, x, y, yaw, digest = adapter.initial_calls[-1]
    assert (frame, x, y, digest) == ("map", 4.0, 5.0, "a" * 64)
    assert yaw == pytest.approx(math.pi / 2)
    assert states[-1] == ("manual_relocalize", "starting")

    adapter.initial_done(True, "点云匹配通过，已进入连续跟踪")
    assert not controller.manual_relocalize_pending
    assert controller.manual_relocalize_state == "matched"
    assert states[-1] == ("manual_relocalize", "success")


def test_initial_pose_publisher_sets_frame_quaternion_and_covariance(monkeypatch):
    from types import SimpleNamespace as NS

    class Message:
        def __init__(self):
            self.header = NS(frame_id="", stamp=None)
            self.pose = NS(
                pose=NS(
                    position=NS(x=0.0, y=0.0, z=0.0),
                    orientation=NS(x=0.0, y=0.0, z=0.0, w=0.0),
                ),
                covariance=[0.0] * 36,
            )

    class Publisher:
        def __init__(self):
            self.messages = []
            self.subscriptions = 1

        def get_subscription_count(self):
            return self.subscriptions

        def publish(self, message):
            self.messages.append(message)

    class Now:
        nanoseconds = 1_000_000_000

        def to_msg(self):
            return NS(sec=1, nanosec=0)

    monkeypatch.setattr(tools_module, "PoseWithCovarianceStamped", Message)
    monkeypatch.setattr(tools_module.time, "monotonic", lambda: 10.0)
    adapter = object.__new__(LocalizationRosAdapter)
    adapter.node = NS(get_clock=lambda: NS(now=lambda: Now()))
    adapter.health = tools_module.LocalizationHealth()
    adapter.health.received = 10.0
    adapter.health.sample = dict(
        instance_id="localizer-a",
        map_frame="map",
        map_sha256="b" * 64,
        published_stamp_ns=1_000_000_000,
        match_sequence=3,
    )
    adapter._initial_pose_pub = Publisher()
    adapter._pending_initial_poses = []

    adapter._initial_pose_pub.subscriptions = 0
    accepted, message = adapter.publish_initial_pose(
        "map", 2.5, -1.0, 0.0, "b" * 64, lambda *_: None)
    assert not accepted and "等待定位服务" in message
    adapter._initial_pose_pub.subscriptions = 1
    accepted, message = adapter.publish_initial_pose(
        "map", 2.5, -1.0, 0.0, "", lambda *_: None)
    assert not accepted and "身份" in message
    accepted, message = adapter.publish_initial_pose(
        "map", 2.5, -1.0, 0.0, "c" * 64, lambda *_: None)
    assert not accepted and "不是当前" in message

    adapter._last_initial_pose_stamp_ns = 1_000_000_000
    accepted, _message = adapter.publish_initial_pose(
        "map", 2.5, -1.0, math.pi / 2, "b" * 64, lambda *_: None)

    assert accepted and len(adapter._initial_pose_pub.messages) == 1
    message = adapter._initial_pose_pub.messages[0]
    assert message.header.frame_id == "map"
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (1, 1)
    assert adapter._pending_initial_poses[0]["pose_stamp_ns"] == 1_000_000_001
    assert adapter._pending_initial_poses[0]["started_ns"] == 1_000_000_000
    assert (message.pose.pose.position.x, message.pose.pose.position.y) == (2.5, -1.0)
    assert message.pose.pose.orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert message.pose.pose.orientation.w == pytest.approx(math.cos(math.pi / 4))
    assert [message.pose.covariance[index] for index in (0, 7, 14)] == [1.0, 1.0, 4.0]
    assert all(message.pose.covariance[index] > 0 for index in (21, 28, 35))

    assert not adapter.publish_rtk_initial_pose(Message(), "localizer-a")
    assert adapter.cancel_initial_pose_requests("cancelled")
    assert not adapter.publish_rtk_initial_pose(Message(), "localizer-a")
    # A vanished/restarted owner cannot later deliver the cancelled result.
    adapter.health.sample["instance_id"] = "localizer-b"
    adapter._poll_cancellations()

    class GlobalClient:
        @staticmethod
        def service_is_ready():
            return True

        @staticmethod
        def call_async(_request):
            return NS(done=lambda: False)

    global_outcomes = []
    adapter._trigger_clients = {"/global": GlobalClient()}
    accepted, _message = adapter.trigger_global_relocalization(
        "/global", lambda *args: global_outcomes.append(args))
    assert accepted
    assert not adapter.publish_rtk_initial_pose(Message(), "localizer-b")
    assert adapter.cancel_global_relocalization_requests("global cancelled")
    assert len(global_outcomes) == 1 and not global_outcomes[0][0]
    assert global_outcomes[0][1].startswith("global cancelled")
    assert not adapter.publish_rtk_initial_pose(Message(), "localizer-a")
    adapter.health.sample["instance_id"] = "localizer-c"
    adapter._poll_cancellations()
    assert adapter.publish_rtk_initial_pose(Message(), "localizer-c")
    assert len(adapter._initial_pose_pub.messages) == 2


def test_outdoor_rtk_seed_waits_for_global_operation_to_release():
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time
    from nav_app.features.outdoor import OutdoorAdapter

    class Publisher:
        def __init__(self):
            self.messages = []

        @staticmethod
        def get_subscription_count():
            return 1

        def publish(self, message):
            self.messages.append(message)

    class State:
        selected_map = {"sha256": "a" * 64}
        map_reason = ""

        @staticmethod
        def feed_map_match(_data):
            return None

        @staticmethod
        def initial_map_pose():
            return dict(
                frame="map",
                position_m=(1.0, 2.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                horizontal_variance_m2=0.04,
                yaw_variance_rad2=0.01,
            )

    manager = object.__new__(LocalizationRosAdapter)
    manager._initial_pose_pub = Publisher()
    manager._global_reservation = (9, "localizer-a")
    manager._pending_results = [dict(
        token=9,
        done=lambda *_args: None,
    )]
    outdoor = object.__new__(OutdoorAdapter)
    outdoor.initial_pose_adapter = manager
    outdoor.initial_publisher = manager._initial_pose_pub
    outdoor.seeded_instance = None
    outdoor.state = State()
    outdoor.node = NS(
        get_clock=lambda: NS(
            now=lambda: NS(to_msg=lambda: Time(sec=1, nanosec=0))))
    quality = NS(data=json.dumps(dict(
        instance_id="localizer-a",
        map_sha256="a" * 64,
        valid=False,
        match_sequence=0,
    )))

    outdoor.map_quality(quality)
    assert outdoor.seeded_instance is None
    assert manager._initial_pose_pub.messages == []

    manager._pending_results = []
    manager._release_global_reservation(9)
    outdoor.map_quality(quality)
    assert outdoor.seeded_instance == "localizer-a"
    assert len(manager._initial_pose_pub.messages) == 1


def test_initial_pose_result_needs_new_fresh_match_and_reports_failure(monkeypatch):
    from types import SimpleNamespace as NS

    now_ns = [1_100_000_000]
    adapter = object.__new__(LocalizationRosAdapter)
    adapter.node = NS(get_clock=lambda: NS(
        now=lambda: NS(nanoseconds=now_ns[0])))
    adapter.health = tools_module.LocalizationHealth()
    adapter.health.received = 10.0
    adapter.health.sample = dict(
        instance_id="localizer-a",
        map_sha256="d" * 64,
        published_stamp_ns=1_050_000_000,
        match_sequence=4,
        valid=False,
        match_age_s=0.1,
        data_age_s=0.1,
        failures=2,
        match_in_progress=False,
        initial_pose_required=True,
        manual_request_id=6,
        manual_pose_stamp_ns=900_000_000,
        completed_manual_request_id=6,
        completed_manual_pose_stamp_ns=900_000_000,
        manual_request_succeeded=False,
    )
    outcomes = []
    adapter._pending_initial_poses = [dict(
        done=lambda *args: outcomes.append(args),
        deadline=30.0,
        started_ns=1_000_000_000,
        pose_stamp_ns=1_000_000_000,
        instance_id="localizer-a",
        map_sha256="d" * 64,
        match_sequence=4,
        baseline_manual_request_id=6,
        manual_request_id=None,
        acknowledged=False,
    )]
    monkeypatch.setattr(tools_module.time, "monotonic", lambda: 10.0)

    adapter._poll_initial_pose_results()
    assert not outcomes and len(adapter._pending_initial_poses) == 1
    adapter.health.sample.update(
        manual_request_id=7,
        manual_pose_stamp_ns=1_000_000_000,
        failures=0,
        initial_pose_required=False,
        match_in_progress=True,
    )
    adapter._poll_initial_pose_results()
    assert not outcomes and len(adapter._pending_initial_poses) == 1
    adapter.health.sample.update(
        completed_manual_request_id=7,
        completed_manual_pose_stamp_ns=1_000_000_000,
        manual_request_succeeded=True,
        valid=True,
        match_sequence=5,
        match_in_progress=False,
    )
    adapter._poll_initial_pose_results()
    assert outcomes == [(True, "点云匹配通过，已进入连续跟踪")]

    outcomes.clear()
    adapter.health.sample.update(
        valid=False,
        match_sequence=5,
        failures=1,
        match_in_progress=False,
        initial_pose_required=True,
        published_stamp_ns=1_090_000_000,
        manual_request_id=8,
        manual_pose_stamp_ns=1_080_000_000,
        completed_manual_request_id=8,
        completed_manual_pose_stamp_ns=1_080_000_000,
        manual_request_succeeded=False,
    )
    adapter._pending_initial_poses = [dict(
        done=lambda *args: outcomes.append(args),
        deadline=30.0,
        started_ns=1_080_000_000,
        pose_stamp_ns=1_080_000_000,
        instance_id="localizer-a",
        map_sha256="d" * 64,
        match_sequence=5,
        baseline_manual_request_id=7,
        manual_request_id=None,
        acknowledged=False,
    )]
    adapter._poll_initial_pose_results()
    assert outcomes and outcomes[0][0] is False
    assert "未通过点云确认" in outcomes[0][1]


def test_controller_can_cancel_scan_context_without_reporting_success(
    tmp_path, monkeypatch,
):
    app = QApplication.instance() or QApplication([])
    assert app is not None
    monkeypatch.setattr(tools_module, "RosLaunchProcess", FakeProcess)
    pcd = tmp_path / "location.pcd"
    pcd.write_bytes(b"pcd")
    controller = LocalizationToolsController(FakeRosAdapter())
    states = []
    controller.operation_state_changed.connect(
        lambda operation, state: states.append((operation, state)))

    assert controller.generate_scan_context(str(pcd))[0]
    success, message = controller.stop_scan_context()

    assert success
    assert "正在停止" in message
    assert states[-1] == ("scan_context", "cancelled")
    assert "已停止" in controller.status


def test_initial_pose_readiness_rejects_stale_quality_and_wrong_map_frame(
    monkeypatch,
):
    from types import SimpleNamespace as NS

    class Publisher:
        @staticmethod
        def get_subscription_count():
            return 1

    monkeypatch.setattr(tools_module, "PoseWithCovarianceStamped", object)
    monkeypatch.setattr(tools_module.time, "monotonic", lambda: 10.0)
    adapter = object.__new__(LocalizationRosAdapter)
    adapter.node = NS(get_clock=lambda: NS(
        now=lambda: NS(nanoseconds=2_000_000_000)))
    adapter._initial_pose_pub = Publisher()
    adapter.health = tools_module.LocalizationHealth()
    adapter.health.received = 8.0
    adapter.health.sample = dict(
        instance_id="localizer-a",
        map_frame="map",
        map_sha256="a" * 64,
        published_stamp_ns=1_900_000_000,
    )

    ready, message = adapter.initial_pose_readiness("map", "a" * 64)
    assert not ready and "定位服务状态" in message

    adapter.health.received = 10.0
    adapter.health.sample["map_frame"] = "different-map-frame"
    ready, message = adapter.initial_pose_readiness("map", "a" * 64)
    assert not ready and "地图坐标系" in message


def test_initial_pose_result_times_out_and_clears_pending(monkeypatch):
    from types import SimpleNamespace as NS

    outcomes = []
    monkeypatch.setattr(tools_module.time, "monotonic", lambda: 10.0)
    adapter = object.__new__(LocalizationRosAdapter)
    adapter.node = NS(get_clock=lambda: NS(
        now=lambda: NS(nanoseconds=2_000_000_000)))
    adapter.health = tools_module.LocalizationHealth()
    adapter._pending_initial_poses = [dict(
        done=lambda *args: outcomes.append(args),
        deadline=9.0,
        started_ns=1_000_000_000,
        instance_id="localizer-a",
        map_sha256="a" * 64,
        match_sequence=1,
    )]

    adapter._poll_initial_pose_results()

    assert outcomes == [(
        False, "重定位超过 120 秒：Localizer 未确认收到人工初始位姿")]
    assert adapter._pending_initial_poses == []


@pytest.mark.parametrize(
    ("quality_change", "expected_message"),
    [
        ({"instance_id": "localizer-b"}, "定位服务已重启"),
        ({"map_sha256": "b" * 64}, "定位服务地图已变化"),
    ],
)
def test_initial_pose_result_rejects_localizer_or_map_identity_change(
    monkeypatch, quality_change, expected_message,
):
    from types import SimpleNamespace as NS

    outcomes = []
    monkeypatch.setattr(tools_module.time, "monotonic", lambda: 10.0)
    adapter = object.__new__(LocalizationRosAdapter)
    adapter.node = NS(get_clock=lambda: NS(
        now=lambda: NS(nanoseconds=2_000_000_000)))
    adapter.health = tools_module.LocalizationHealth()
    adapter.health.received = 10.0
    adapter.health.sample = dict(
        instance_id="localizer-a",
        map_sha256="a" * 64,
        published_stamp_ns=1_900_000_000,
        match_sequence=1,
        valid=False,
        failures=0,
        match_in_progress=True,
        initial_pose_required=False,
    )
    adapter.health.sample.update(quality_change)
    adapter._pending_initial_poses = [dict(
        done=lambda *args: outcomes.append(args),
        deadline=30.0,
        started_ns=1_000_000_000,
        instance_id="localizer-a",
        map_sha256="a" * 64,
        match_sequence=1,
    )]

    adapter._poll_initial_pose_results()

    assert outcomes and outcomes[0][0] is False
    assert expected_message in outcomes[0][1]
    assert adapter._pending_initial_poses == []


def test_controller_keeps_manual_and_global_relocalization_mutually_exclusive(
    monkeypatch,
):
    app = QApplication.instance() or QApplication([])
    assert app is not None
    monkeypatch.setattr(tools_module, "RosLaunchProcess", FakeProcess)
    adapter = FakeRosAdapter()
    controller = LocalizationToolsController(adapter)

    accepted, _message = controller.submit_initial_pose(
        "map", (0.0, 0.0), (1.0, 0.0), "a" * 64)
    assert accepted and controller.manual_relocalize_pending
    accepted, message = controller.trigger_global_relocalization()
    assert not accepted and "两点重定位" in message
    assert adapter.calls == []
    ready, message = controller.initial_pose_readiness("map", "a" * 64)
    assert not ready and "两点重定位" in message

    adapter.initial_done(True, "点云匹配通过")
    accepted, _message = controller.trigger_global_relocalization()
    assert accepted and controller.global_relocalize_pending
    assert controller.manual_relocalize_state == "idle"
    initial_call_count = len(adapter.initial_calls)
    accepted, message = controller.submit_initial_pose(
        "map", (0.0, 0.0), (1.0, 0.0), "a" * 64)
    assert not accepted and "自动重定位" in message
    assert len(adapter.initial_calls) == initial_call_count
    ready, message = controller.initial_pose_readiness("map", "a" * 64)
    assert not ready and "自动重定位" in message

    assert controller.cancel_global_relocalization("地图已切换")
    assert not controller.global_relocalize_pending
    assert controller.status == "地图已切换"

    accepted, _message = controller.trigger_global_relocalization()
    assert accepted and controller.global_relocalize_pending
    adapter.done(True, "全局重定位匹配成功")
    assert not controller.global_relocalize_pending


def test_controller_can_resubmit_after_initial_pose_publish_is_rejected(
    monkeypatch,
):
    app = QApplication.instance() or QApplication([])
    assert app is not None
    monkeypatch.setattr(tools_module, "RosLaunchProcess", FakeProcess)

    class RejectOnceAdapter(FakeRosAdapter):
        def __init__(self):
            super().__init__()
            self.publish_attempts = 0

        def publish_initial_pose(
            self, frame_id, x, y, yaw, expected_map_sha256, done,
        ):
            self.publish_attempts += 1
            self.initial_calls.append(
                (frame_id, x, y, yaw, expected_map_sha256))
            if self.publish_attempts == 1:
                return False, "定位服务尚未就绪"
            self.initial_done = done
            return True, "已提交大概位置和方向，等待点云匹配确认"

    adapter = RejectOnceAdapter()
    controller = LocalizationToolsController(adapter)

    accepted, message = controller.submit_initial_pose(
        "map", (1.0, 2.0), (2.0, 2.0), "a" * 64)
    assert not accepted and "尚未就绪" in message
    assert not controller.manual_relocalize_pending
    assert controller.manual_relocalize_state == "error"

    accepted, message = controller.submit_initial_pose(
        "map", (1.0, 2.0), (2.0, 2.0), "a" * 64)
    assert accepted and "等待点云" in message
    assert adapter.publish_attempts == 2
    assert controller.manual_relocalize_pending
    assert controller.manual_relocalize_state == "pending"

    adapter.initial_done(False, "本次先验未通过点云确认")
    assert not controller.manual_relocalize_pending


def test_controller_passes_temporary_relocalization_timeout(monkeypatch):
    app = QApplication.instance() or QApplication([])
    assert app is not None
    monkeypatch.setattr(tools_module, "RosLaunchProcess", FakeProcess)
    adapter = FakeRosAdapter()
    controller = LocalizationToolsController(adapter)

    accepted, message = controller.submit_initial_pose(
        "map", (1.0, 2.0), (2.0, 2.0), "a" * 64,
        timeout_s=300)
    assert accepted
    assert adapter.timeout_calls[-1] == ("manual", 300)
    assert controller.active_relocalization_timeout_s == 300
    adapter.initial_done(False, "test complete")

    accepted, message = controller.trigger_global_relocalization(
        timeout_s=600)
    assert accepted
    assert adapter.timeout_calls[-1] == ("global", 600)
    adapter.done(False, "test complete")

    accepted, message = controller.trigger_global_relocalization(
        timeout_s=601)
    assert not accepted and "30–600" in message


def test_relocalization_recorder_writes_bounded_structured_timeline(tmp_path):
    recorder = tools_module.RelocalizationDiagnosticsRecorder(tmp_path)
    attempt = recorder.begin(
        "manual", 300, frame_id="map", map_sha256="a" * 64,
        position=[1.0, 2.0], direction=[2.0, 2.0], yaw=0.0)
    recorder.quality(dict(
        stage="refine_gicp", reason_code="", progress_sequence=3,
        input_points=1200, refine_points=800,
        refine_fitness=.08, refine_threshold=.1,
        instance_id="localizer-a", map_sha256="a" * 64,
        relocalization_timeout_s=300))
    recorder.finish(False, "精匹配未收敛")
    from nav_app.features.log_store import log_store
    assert log_store().flush()

    latest = json.loads(
        (tmp_path / "relocalization-latest.json").read_text())
    rows = [json.loads(line) for line in (
        tmp_path / "relocalization-attempts.jsonl").read_text().splitlines()]
    assert latest["attempt_id"] == attempt
    assert latest["timeout_s"] == 300
    assert latest["latest_quality"]["stage"] == "refine_gicp"
    assert latest["result"]["success"] is False
    assert [row["event"] for row in rows] == [
        "started", "quality", "finished"]


def test_stale_input_reports_seconds_instead_of_passing_geometric_score():
    text=tools_module.format_relocalization_failure(dict(reason_code='RESULT_STALE',
        data_age_s=2.203,refine_fitness=.00191,refine_threshold=.1),'定位失败')
    assert '2.2秒' in text and '阈值' not in text and '0.002' not in text


def test_ambiguous_geometry_offers_a_new_view_instead_of_a_raw_error_code():
    text = tools_module.format_relocalization_failure(
        dict(reason_code='GLOBAL_GEOMETRY_AMBIGUOUS'), '定位失败')
    assert '换个视角' in text and '重试定位' in text
    assert 'GLOBAL_GEOMETRY_AMBIGUOUS' not in text
