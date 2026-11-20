"""Action discovery cannot substitute for confirmed active Nav2 lifecycle."""
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest

from nav_app.features.navigation_readiness import NavigationReadiness


class Reply(Future):
    def cancel(self):return False  # An RPC response may already be on its way.


class Client:
    def __init__(self):self.available=True;self.requests=[];self.removed=[]
    def service_is_ready(self):return self.available
    def call_async(self, request):
        future=Reply();self.requests.append((request,future));return future
    def remove_pending_request(self, future):self.removed.append(future)
    def respond(self, state):
        self.requests[-1][1].set_result(NS(current_state=NS(id=state)))


@pytest.fixture
def runtime():
    clients={};now=[100.];located=[False]
    manager=Client()
    helper=NavigationReadiness(lambda name:clients.setdefault(name,Client()),lambda:NS(),
        manager_client=lambda name:manager,resume_request=lambda:NS(command=2),
        localization_ready=lambda:located[0],clock=lambda:now[0])
    return helper,clients,manager,now,located


def states(runtime, planner, map_state=None):
    helper,clients,manager,now,located=runtime
    helper.ready(True)
    clients['/planner_server/get_state'].respond(planner)
    if map_state is not None:clients['/map_server/get_state'].respond(map_state)


@pytest.mark.parametrize('backend',['walking','nav2'])
def test_action_visible_inactive_is_not_ready_active_is_required(runtime, backend):
    helper,clients,manager,now,located=runtime
    helper.configure(backend)
    assert not helper.ready(True)
    assert '等待 Nav2 状态回执' in helper.reason
    states(runtime,2,2 if backend=='walking' else None)
    assert not helper.ready(True)
    assert 'Nav2 尚未激活' in helper.reason
    for _ in range(100):assert not helper.ready(True)
    assert all(len(client.requests)==1 for client in clients.values())
    now[0]+=.6
    states(runtime,3,3 if backend=='walking' else None)
    assert helper.ready(True) and not helper.ready(False)
    assert not manager.requests


def test_robot_requires_map_planner_controller_and_bt_active(runtime):
    helper,clients,manager,now,located=runtime
    helper.configure('nav2','robot');assert not helper.ready(True,True)
    clients['/planner_server/get_state'].respond(3)
    clients['/map_server/get_state'].respond(2)
    clients['/controller_server/get_state'].respond(3)
    clients['/bt_navigator/get_state'].respond(3)
    assert not helper.ready(True,True)
    now[0]+=.6;helper.ready(True,True)
    for client in clients.values():client.respond(3)
    assert helper.ready(True,True) and not helper.ready(True,False)
    assert not manager.requests


def test_normal_startup_and_mixed_states_never_trigger_resume(runtime):
    helper,clients,manager,now,located=runtime
    helper.configure('walking');located[0]=True
    states(runtime,2,2);helper.ready(True)
    now[0]+=2;states(runtime,13,3);assert not helper.ready(True)
    now[0]+=.6;states(runtime,3,3);assert helper.ready(True)
    assert not manager.requests


def test_aborted_walking_start_resumes_only_both_inactive_after_real_pose(runtime):
    helper,clients,manager,now,located=runtime
    helper.configure('walking');helper.observe_log('Failed to bring up all requested nodes. Aborting bringup.')
    states(runtime,2,2);assert not helper.ready(True) and not manager.requests
    located[0]=True;assert not helper.ready(True)
    assert len(manager.requests)==1 and manager.requests[0][0].command==2
    manager.requests[0][1].set_result(NS(success=True))
    assert not helper.ready(True)  # RESUME acknowledgement is not active proof.
    now[0]+=.6;states(runtime,3,3);assert helper.ready(True)
    assert len(manager.requests)==1


def test_resume_failure_is_visible_and_not_retried_indefinitely(runtime):
    helper,clients,manager,now,located=runtime
    helper.configure('walking');located[0]=True
    helper.observe_log('Failed to change state for node: planner_server')
    states(runtime,2,2);helper.ready(True)
    manager.requests[0][1].set_result(NS(success=False))
    now[0]+=.6;states(runtime,2,2)
    assert not helper.ready(True) and '未能激活' in helper.reason
    assert len(manager.requests)==1


def test_without_abort_log_stable_inactive_waits_beyond_startup_timeout(runtime):
    helper,clients,manager,now,located=runtime
    helper.configure('walking');located[0]=True
    states(runtime,2,2);helper.ready(True)
    for _ in range(48):
        now[0]+=.6;states(runtime,2,2);helper.ready(True)
    assert not manager.requests
    for _ in range(3):
        now[0]+=.6;states(runtime,2,2);helper.ready(True)
    assert len(manager.requests)==1


def test_resume_timeout_is_bounded_and_previous_generation_cannot_mutate(runtime):
    helper,clients,manager,now,located=runtime
    helper.configure('walking');located[0]=True
    helper.observe_log('Aborting bringup.');states(runtime,2,2);helper.ready(True)
    late=manager.requests[0][1]
    now[0]+=8.1;states(runtime,2,2)
    assert not helper.ready(True) and '超时' in helper.reason
    assert late in manager.removed and len(manager.requests)==1
    helper.configure('custom');late.set_result(NS(success=True))
    assert helper.ready(True) and helper.reason=='正在确认路线服务状态'


def test_timeouts_and_stop_or_backend_changes_reject_old_responses(runtime):
    helper,clients,manager,now,located=runtime
    helper.configure('walking');helper.ready(True)
    old=clients['/planner_server/get_state'].requests[-1][1]
    now[0]+=2.1;helper.ready(True)
    assert old in clients['/planner_server/get_state'].removed
    old.set_result(NS(current_state=NS(id=3)))
    assert not helper.ready(True)
    pending=clients['/planner_server/get_state'].requests[-1][1]
    helper.reset();pending.set_result(NS(current_state=NS(id=3)))
    assert not helper.ready(True)
    helper.configure('scan_planner');assert helper.ready(True)
    helper.configure('custom');assert helper.ready(True)
    helper.configure('outdoor');assert helper.ready(True)
    helper.configure('walking');assert not helper.ready(True)


def test_real_planning_entry_rejects_discovered_but_inactive_nav2(runtime):
    from nav_app.qt_nav_node import QtNavRosNode
    from nav_app.features.walking_guidance import WalkingGuidance
    helper,clients,manager,now,located=runtime
    helper.configure('walking');states(runtime,2,2)
    action=NS(server_is_ready=lambda:True,send_goal_async=lambda *_:pytest.fail('inactive goal sent'))
    node=NS(_navigation_readiness=helper,navigation_client=action,robot_client=action,_planning_inputs=(),_meshnav_active=False)
    node.navigation_route_ready=lambda:QtNavRosNode.navigation_route_ready(node)
    assert not QtNavRosNode.navigation_component_ready(node,'nav2')
    assert not QtNavRosNode.send_navigation_waypoints(node,[[1.,2.]])
    walking=WalkingGuidance(action,lambda *_:None,ready=node.navigation_route_ready)
    assert not walking.plan(NS())





def test_real_lifecycle_replies_arrive_while_gui_executor_is_not_spinning(tmp_path, monkeypatch):
    """Image/map work must not prevent the startup guard seeing real Active replies."""
    import time
    import rclpy
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer
    from nav_app.qt_nav_node import QtNavRosNode
    from nav_app.transform_listener import DisplayTransformListener
    monkeypatch.setenv("NAV_RUNTIME_DIR", str(tmp_path / "runtime"))
    rclpy.init()
    server = DisplayTransformListener(Buffer())
    node = QtNavRosNode()
    state = [3]
    def respond(request, response):
        response.current_state.id = state[0]
        response.current_state.label = "active" if state[0] == 3 else "inactive"
        return response
    service = server.node.create_service(GetState, "/planner_server/get_state", respond)
    helper = node._navigation_readiness
    helper.configure("nav2")
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not helper.ready(True):
            time.sleep(.02)  # Deliberately no rclpy.spin_once(node), no GUI event pump.
        assert helper.ready(True), helper.reason
        time.sleep(2.)  # Idle UI longer than the 1.5 s cache lifetime.
        assert helper.ready(True), helper.reason
        state[0] = 2
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            helper.ready(True)
            if helper.rows["planner_server"]["state"] == 2:break
            time.sleep(.02)
        assert not helper.ready(True)
        assert helper.rows["planner_server"]["state"] == 2
    finally:
        node.destroy_node()
        server.close()
        rclpy.shutdown()



def test_camera_bandwidth_split_preserves_all_existing_bridge_rows(monkeypatch):
    import sys
    from pathlib import Path
    script = (Path(__file__).parents[3] / "deployment/woosh_bridge_autostart.sh").read_text()
    topics = [{"topic": name, "queue_size": 1} for name in (
        "livox/imu_192_168_128_3", "camera_1/color/image_raw",
        "livox/imu_192_168_128_4", "camera_1/color/camera_info", "scan", "robot_status")]
    params = {"/topics": topics}
    monkeypatch.setitem(sys.modules, "rosparam", NS(get_param=params.__getitem__, set_param_raw=params.__setitem__))
    source = script.split("<<'PARAMS'\n", 1)[1].split("\nPARAMS", 1)[0]
    exec(compile(source, "woosh bridge parameters", "exec"), {})
    combined = params["/topics"] + params["/nav_imu_bridge/topics"] + [topics[1]]
    assert sorted(combined, key=lambda row:row["topic"]) == sorted(topics, key=lambda row:row["topic"])
    assert params["/nav_camera_bridge/topics"][0]["topic"] == "nav/camera_1/image/compressed"
    assert params["/nav_camera_bridge/topics"][0]["type"] == "sensor_msgs/msg/CompressedImage"
    assert params["/nav/camera_1/image/compressed/format"] == "png"
    assert topics[2] in params["/topics"] and topics[3] in params["/topics"]
    assert "throttle messages /camera_1/color/image_raw 5.0 /nav/camera_1/image_throttled" in script
    assert "republish raw in:=/nav/camera_1/image_throttled compressed" in script



def test_lossless_png_receiver_preserves_pixels_encoding_and_header():
    import cv2
    import numpy as np
    from array import array
    from sensor_msgs.msg import Image, CompressedImage
    from nav_app.features.image_archive import decode_image
    pixels = np.arange(17*23*3, dtype=np.uint8).reshape(17, 23, 3)
    message = CompressedImage()
    message.header.frame_id = "camera_1_color_optical_frame"
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.format = "rgb8; png compressed bgr8"
    ok, payload = cv2.imencode(".png", cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))
    assert ok
    message.data = array("B", payload.tobytes())
    decoded = decode_image(message)
    assert decoded.header == message.header
    assert decoded.encoding == "rgb8"
    assert (decoded.height, decoded.width, decoded.step) == (17, 23, 69)
    assert bytes(decoded.data) == pixels.tobytes()



def test_navigation_freshness_uses_real_ros_receipt_while_gui_is_busy(tmp_path, monkeypatch):
    import json
    import time
    import rclpy
    from array import array
    from sensor_msgs.msg import Image, PointCloud2, LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer
    from nav_app.qt_nav_node import QtNavRosNode
    from nav_app.transform_listener import DisplayTransformListener
    from nav_app.features.navigation_stack import NavigationStackConfig
    monkeypatch.setenv("NAV_RUNTIME_DIR", str(tmp_path / "runtime"))
    rclpy.init()
    server = DisplayTransformListener(Buffer())
    node = QtNavRosNode()
    config = NavigationStackConfig(navigation_map="", localization_map="",
        dlio_pointcloud_topic="/nav/test_lidar", scan_topic="/nav/test_scan")
    node.configure_navigation(config)
    publishers = [
        server.node.create_publisher(Image, node.camera_topic, 1),
        server.node.create_publisher(PointCloud2, config.dlio_pointcloud_topic, 1),
        server.node.create_publisher(LaserScan, config.scan_topic, 1),
        server.node.create_publisher(String, "/localizer/localizer_node/quality", 1)]
    try:
        deadline = time.monotonic()+5
        while time.monotonic()<deadline:
            stamp = server.node.get_clock().now()
            image = Image();image.header.stamp=stamp.to_msg()
            image.width=image.height=2;image.step=6;image.encoding="rgb8"
            image.data=array("B",bytes(12))
            cloud=PointCloud2();cloud.header.stamp=stamp.to_msg()
            scan=LaserScan();scan.header.stamp=stamp.to_msg()
            quality=String(data=json.dumps(dict(schema_version=1,valid=True,
                published_stamp_ns=stamp.nanoseconds,match_age_s=.1,data_age_s=.1,
                match_sequence=1,request_id=0,completed_request_id=0)))
            for publisher,message in zip(publishers,[image,cloud,scan,quality]):publisher.publish(message)
            time.sleep(.03)  # No GUI executor spinning.
            if (node.navigation_sensor_data_active("lidar") and node.navigation_sensor_data_active("camera")
                    and node.navigation_component_ready("scan") and node.localization_tools.health.valid(stamp.nanoseconds)):
                break
        assert node.navigation_sensor_data_active("lidar")
        assert node.navigation_sensor_data_active("camera")
        assert node.navigation_component_ready("scan")
        assert node.localization_tools.health.valid(node.get_clock().now().nanoseconds)
        # DDS may deliver the last queued sample after publishing stops.
        for publisher in publishers:server.node.destroy_publisher(publisher)
        deadline = time.monotonic()+5
        while time.monotonic()<deadline and (
                node.navigation_sensor_data_active("lidar") or node.navigation_sensor_data_active("camera")
                or node.navigation_component_ready("scan")
                or node.localization_tools.health.valid(node.get_clock().now().nanoseconds)):
            time.sleep(.03)
        assert not node.navigation_sensor_data_active("lidar")
        assert not node.navigation_sensor_data_active("camera")
        assert not node.navigation_component_ready("scan")
        assert not node.localization_tools.health.valid(node.get_clock().now().nanoseconds)
    finally:
        node.configure_navigation(None)
        node.destroy_node()
        server.close()
        rclpy.shutdown()
