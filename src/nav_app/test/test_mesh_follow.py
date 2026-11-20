from types import SimpleNamespace as NS

import numpy as np
import pytest

from nav_app.features.mesh_follow import MeshFollower
from sensor_msgs.msg import PointCloud2, PointField
from nav_app.features.meshnav_maps import MeshSurface
from nav_app.features.mesh_drive import MeshDrive


def ramp():
    return MeshSurface([[-2,-2,-.4], [3,-2,.6], [3,3,.6], [-2,3,-.4]], [[0,1,2],[0,2,3]])


def test_closed_loop_tracks_confirmed_ramp_then_turns_and_arrives():
    route = np.array([[0,0,0], [1,0,.2], [1,1,.2]], dtype=float)
    follower = MeshFollower(route)
    route[1:] = 50  # A new preview cannot change the accepted path.
    surface = ramp()
    position = np.zeros(3)
    yaw = 0.
    for tick in range(900):
        command = follower.command(position, yaw, surface, 0, np.empty((0, 2)), [0.,0.,0.])
        if command is None:
            break
        speed, turn = command
        assert 0 <= speed <= .10 and abs(turn) <= .35
        position[:2] += .1 * speed * np.array([np.cos(yaw), np.sin(yaw)])
        position[2] = position[0] * .2
        yaw += .1 * turn
    assert command is None and np.linalg.norm(position - [1,1,.2]) <= .10
    with pytest.raises(ValueError, match="偏离"):
        MeshFollower([[0,0,0], [1,0,.2]]).command([0,0,1.5], 0, surface, 0, [], [0,0,0])


def test_swept_footprint_rejects_obstacle_and_mesh_boundary():
    surface = ramp()
    for y, obstacles, reason in ((0., [[.52,.05,.20]], "障碍"), (-1.8, [[8,8,0]], "网格")):
        follower = MeshFollower([[0,y,0], [1,y,.2]])
        with pytest.raises(ValueError, match=reason):
            follower.command(np.array([0,y,0]), 0, surface, 0, np.array(obstacles), [0.,0.,0.])


def cloud(points):
    msg = PointCloud2(height=1, width=len(points), point_step=12, row_step=len(points)*12)
    msg.header.frame_id = "base_link"
    msg.header.stamp.sec = 10
    msg.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate("xyz")]
    msg.data = np.asarray(points, dtype="<f4").tobytes()
    return msg


def test_ground_relative_clearance_on_ramp_and_separate_levels():
    surface = ramp()
    for x in (.3, -.3):
        ground_z = .2*x
        for dz, blocked in ((0., False), (.12, True), (.7, True), (1.7, False)):
            assert surface.obstacle_in_clearance(np.array([[x, 0., ground_z+dz]]), 0., 0) == blocked
    vertices = np.vstack((surface.vertices, surface.vertices+[0.,0.,1.9]))
    levels = MeshSurface(vertices, np.vstack((surface.faces, surface.faces+4)))
    assert not levels.obstacle_in_clearance(np.array([[.2,0.,1.94]]), 0., 0)
    assert levels.obstacle_in_clearance(np.array([[.2,0.,2.14]]), 1.9, 1)


@pytest.mark.parametrize("background", [
    [[4, 4, 0], [-4, 4, 0], [-4, -4, 0], [4, -4, 0]],
    [[4, 4, 0], [-4, 4, 0], [4, -4, 0]],
    [[2, 0, 0]],
], ids=["all-directions", "missing-rear-right", "single-direction"])
def test_obstacle_latches_and_takeover_waits_for_new_measured_stop(background):
    stamp = [10_000_000_000]
    sent, permits, modes = [], [], []
    surface = ramp()
    session = NS(unavailable_reason=lambda:"", fresh=lambda:True, base_reason=lambda:"",
                 armed=True, poll=lambda *args:"", state=dict(ready_to_start=True,
                 stop_confirmed=True, stamp_ns=stamp[0], measured_velocity=[0,0,0]))
    n = NS(create_publisher=lambda *args:NS(publish=lambda m:sent.append(m)),
           get_logger=lambda:NS(info=lambda msg:None),
           motion_node=NS(create_timer=lambda *args, **kwargs:None), motion_permit_pub=NS(publish=lambda m:permits.append(m.data)),
           walking=NS(phase="preview", path=np.array([[0.,0.,0.], [1.,0.,.2]])),
           meshnav_start=lambda:(np.zeros(3), 0), _meshnav_surface=surface, motion_session=session,
           get_clock=lambda:NS(now=lambda:NS(nanoseconds=stamp[0])), navigation_route_ready=lambda:True,
           lookup_matrix=lambda *args:np.eye(4), base_frame="base_link", map_frame="map",
           set_woosh_control_mode=modes.append)
    drive = MeshDrive(n)
    drive.surface = surface
    for pts in ([], [[float("nan")]*3], [[.02, .01, 0]]):
        drive.clouds.append(cloud(pts))
        assert not drive.begin()
    drive.clouds.append(cloud(background))
    drive.tick()
    assert not sent and not permits
    assert drive.begin()
    drive.tick()
    assert sent[-1].linear.x > 0 and permits[-1]
    drive.clouds.append(cloud(background + [[.47,0,.24]]))
    drive.tick()
    assert sent[-1].linear.x == 0 and not permits[-1] and drive.phase == "stopping"
    assert "障碍" in n.navigation_status
    drive.clouds.append(cloud(background))
    drive.tick()
    assert not drive.begin()
    stamp[0] += 400_000_000
    session.state["stamp_ns"] = stamp[0]
    drive.tick()
    assert drive.phase == "paused" and not permits[-1]
    assert not drive.begin() and "过期" in n.navigation_status
    drive.clouds[-1].header.stamp.nanosec = 400_000_000
    assert drive.begin()
    drive.tick()
    drive.stop("交还手柄", "manual")
    drive.tick()
    assert not modes
    stamp[0] += 400_000_000
    session.state["stamp_ns"] = stamp[0]
    drive.tick()
    assert modes == [0] and not permits[-1]


def test_organized_xyz_preserves_padded_rows_and_endianness():
    from nav_app.map_processing import pointcloud2_to_xyz
    msg = cloud([[1,2,3],[4,5,6]])
    msg.height, msg.width, msg.row_step, msg.is_bigendian = 2, 1, 16, True
    msg.data = np.asarray([1,2,3,99,4,5,6,99],dtype=">f4").tobytes()
    np.testing.assert_array_equal(pointcloud2_to_xyz(msg), [[1,2,3],[4,5,6]])


def test_status_and_mesh_timer_progress_without_spinning_gui(monkeypatch):
    import json
    import threading
    import time
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from nav_app.qt_nav_node import QtNavRosNode

    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init(args=["--ros-args", "-p", "robot_profile:=woosh"], domain_id=183,
               signal_handler_options=SignalHandlerOptions.NO)
    node = QtNavRosNode()
    source = rclpy.create_node("test_motion_status_source")
    publisher = source.create_publisher(String, "/nav/motion_status", 1)
    ticks = []
    monkeypatch.setattr(MeshDrive, "tick", lambda drive: ticks.append(threading.get_ident()))
    try:
        node.mesh_drive = MeshDrive(node)
        deadline = time.monotonic() + 3.
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(.02)
        start = time.monotonic()
        while time.monotonic() - start < .8:
            stamp = source.get_clock().now().nanoseconds
            publisher.publish(String(data=json.dumps(dict(schema=1, armed=False,
                                     base_connected=True, stamp_ns=stamp))))
            time.sleep(.04)
        assert node.motion_session.fresh() and node.motion_session.state["stamp_ns"] > stamp - 100_000_000
        assert len(ticks) >= 6 and set(ticks) == {node._motion_thread.ident}
        time.sleep(.35)
        assert not node.motion_session.fresh() and "底盘状态超时" in node.motion_session.unavailable_reason()
    finally:
        node.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def test_dynamic_cloud_uses_side_clearance_and_checks_rotation():
    flat = MeshSurface([[-2,-2,0],[2,-2,0],[2,2,0],[-2,2,0]], [[0,1,2],[0,2,3]])
    position = np.zeros(3)
    for side, blocked in ((.41, False), (.37, True)):
        tracker = MeshFollower([position, [1.,0.,0.]])
        obstacle = np.array([[0.,side,.25]])
        if blocked:
            with pytest.raises(ValueError, match="障碍"):
                tracker.command(position, 0., flat, 0, obstacle, [0.,0.,0.])
        else:
            assert tracker.command(position, 0., flat, 0, obstacle, [0.,0.,0.])[0] > 0
    # The front corner moves sideways during a commanded in-place turn.
    tracker = MeshFollower([position, [0.,1.,0.]])
    with pytest.raises(ValueError, match="障碍"):
        tracker.command(position, 0., flat, 0, np.array([[.23,.40,.22]]), [0.,0.,0.])
