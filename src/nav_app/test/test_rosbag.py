from pathlib import Path

import yaml

from nav_app.features.rosbag import (
    BASE_SENSOR_TOPICS,
    BagTopic,
    PlaybackRequest,
    RosbagController,
    build_play_arguments,
    build_record_arguments,
    default_record_selected,
    read_bag_metadata,
    safe_algorithm_topic,
)


def write_metadata(root: Path) -> Path:
    document = {
        "rosbag2_bagfile_information": {
            "storage_identifier": "sqlite3",
            "duration": {"nanoseconds": 2_500_000_000},
            "starting_time": {"nanoseconds_since_epoch": 123_000_000_000},
            "message_count": 13,
            "topics_with_message_count": [
                {
                    "topic_metadata": {
                        "name": "/livox/lidar",
                        "type": "livox_ros_driver2/msg/CustomMsg",
                    },
                    "message_count": 10,
                },
                {
                    "topic_metadata": {
                        "name": "/aft_mapped_to_init",
                        "type": "nav_msgs/msg/Odometry",
                    },
                    "message_count": 3,
                },
            ],
        }
    }
    (root / "metadata.yaml").write_text(yaml.safe_dump(document))
    return root


def test_metadata_and_safe_algorithm_topic_filter(tmp_path):
    metadata = read_bag_metadata(write_metadata(tmp_path))
    assert metadata.storage_id == "sqlite3"
    assert metadata.duration_seconds == 2.5
    assert metadata.message_count == 13
    assert [topic.name for topic in metadata.topics] == [
        "/aft_mapped_to_init", "/livox/lidar"]
    assert safe_algorithm_topic(metadata.topics[0]) is False
    assert safe_algorithm_topic(metadata.topics[1]) is True
    assert default_record_selected(metadata.topics[1]) is True
    assert safe_algorithm_topic(BagTopic(
        "/tf_static", "tf2_msgs/msg/TFMessage")) is True
    assert safe_algorithm_topic(BagTopic(
        "/tf", "tf2_msgs/msg/TFMessage")) is False


def test_phone_solution_is_recorded_and_replayed_without_replaying_status():
    solution = BagTopic("/phone_gnss/rtk_solution", "std_msgs/msg/String")
    assert default_record_selected(solution)
    assert safe_algorithm_topic(solution)
    for name in ("/phone_gnss/observation", "/phone_gnss/status"):
        topic = BagTopic(name, "std_msgs/msg/String")
        assert default_record_selected(topic)
        assert not safe_algorithm_topic(topic)
    assert not safe_algorithm_topic(BagTopic("/phone_gnss/rtk_solution", "std_msgs/msg/Bool"))


def test_base_catalog_covers_mapping_lidar_imu_and_camera_inputs():
    catalog = {(topic.name, topic.message_type) for topic in BASE_SENSOR_TOPICS}
    assert (
        "/livox/lidar", "livox_ros_driver2/msg/CustomMsg") in catalog
    assert (
        "/livox/lidar/pointcloud", "sensor_msgs/msg/PointCloud2") in catalog
    assert ("/livox/imu", "sensor_msgs/msg/Imu") in catalog
    assert (
        "/camera/camera/color/image_raw", "sensor_msgs/msg/Image") in catalog
    assert any(name.startswith("/camera/") for name, _type in catalog)


def test_capture_satellite_checkbox_overrides_discovery(tmp_path, monkeypatch):
    from types import SimpleNamespace
    protocol = tmp_path/'capture.yaml'
    protocol.write_text(yaml.safe_dump(dict(roles=dict(imu='/imu'), required_roles=['imu'])))
    monkeypatch.setenv('NAV_CAPTURE_CONFIG', str(protocol))
    controller = RosbagController()
    controller.node = SimpleNamespace(get_publishers_info_by_topic=lambda _: [object()])
    controller.available_topics = lambda: (
        BagTopic('/rtk/fix', 'sensor_msgs/msg/NavSatFix'),
        BagTopic('/rtk/observation', 'std_msgs/msg/String'),
        BagTopic('/tf_static', 'tf2_msgs/msg/TFMessage'))
    assert controller.capture_topics(False, False) == ('/imu', '/tf_static')
    assert set(controller.capture_topics(False, True)) == {'/imu', '/tf_static', '/rtk/fix', '/rtk/observation'}
    controller.available_topics = lambda: ()
    assert '/rtk/fix' in controller.capture_topics(False, True)


def test_record_and_play_commands_keep_paths_and_remaps_as_single_arguments(
    tmp_path,
):
    output = tmp_path / "bag with spaces"
    record = build_record_arguments(
        output, ["/livox/lidar", "/livox/imu"])
    assert record == [
        "bag", "record", "--output", str(output), "--storage", "sqlite3",
        "--compression-mode", "message", "--compression-format", "zstd",
        "--compression-queue-size", "128", "--compression-threads", "1",
        "--max-cache-size", "134217728", "--max-bag-size", "1000000000",
        "/livox/lidar", "/livox/imu",
    ]
    plain = build_record_arguments(
        output, ["/livox/imu"], storage_id="mcap", compression_mode="none",
        max_cache_size=0, max_bag_size=0)
    assert plain == [
        "bag", "record", "--output", str(output), "--storage", "mcap",
        "/livox/imu",
    ]
    small = build_record_arguments(
        output, ["/livox/imu"], compression_format="zstd",
        max_cache_size=50_000_000, max_bag_size=2_000_000_000)
    assert "--compression-format" in small
    assert small[small.index("--max-cache-size") + 1] == "50000000"
    assert small[small.index("--max-bag-size") + 1] == "2000000000"
    request = PlaybackRequest(
        bag_path=output,
        topics=("/old/lidar", "/imu"),
        remaps=(("/old/lidar", "/livox/lidar"),),
        rate=0.5,
        clock_hz=40.0,
        start_paused=True,
    )
    play = build_play_arguments(request)
    assert play[:3] == ["bag", "play", str(output)]
    assert "--start-paused" in play
    assert play[play.index("--topics") + 1:play.index("--remap")] == [
        "/old/lidar", "/imu"]
    assert play[-1] == "/old/lidar:=/livox/lidar"
    assert request.effective_topics["/old/lidar"] == "/livox/lidar"


def test_controller_creates_record_parent_without_overwriting(tmp_path):
    controller = RosbagController()
    calls = []
    controller.record_process.start_launch = lambda arguments: (
        calls.append(arguments) or True)
    parent = tmp_path / "new" / "bags"
    success, _status = controller.start_recording(
        parent, "sample", ["/livox/imu"])
    assert success
    assert parent.is_dir()
    assert calls[0][-1] == "/livox/imu"

    (parent / "existing").mkdir()
    success, status = controller.start_recording(
        parent, "existing", ["/livox/imu"])
    assert not success
    assert "不会覆盖" in status

    success, status = controller.start_recording(
        parent, "existing", ["/livox/imu"], unique_if_exists=True)
    assert success
    assert (parent / "existing").is_dir()
    created = Path(calls[-1][calls[-1].index("--output") + 1])
    assert created.parent == parent
    assert created.name.startswith("existing_")
    assert created != parent / "existing"


def test_handheld_mapping_record_requires_original_estimator_snapshot(tmp_path):
    controller = RosbagController()
    calls=[]
    controller.record_process.start_launch=lambda arguments: calls.append(arguments) or True
    ok,_=controller.start_recording(tmp_path,'mapped',['/livox/imu'],require_mapping_parameters=True)
    assert ok
    assert calls[0][:7]==['run','nav_app','reliable_record','--require-mapping-parameters','--image-storage','raw','--']
    assert "--compression-mode" in calls[-1]


def test_default_selection_matches_fast_livo_plus_depth_recording():
    """默认勾选应当正好是"FAST-LIVO2 建图 + 深度"那一组话题。"""
    live = {
        "/livox/lidar": "livox_ros_driver2/msg/CustomMsg",
        "/livox/lidar/pointcloud": "sensor_msgs/msg/PointCloud2",
        "/livox/imu": "sensor_msgs/msg/Imu",
        "/camera/camera/color/image_raw": "sensor_msgs/msg/Image",
        "/camera/camera/color/camera_info": "sensor_msgs/msg/CameraInfo",
        "/camera/camera/depth/image_rect_raw": "sensor_msgs/msg/Image",
        "/camera/camera/depth/camera_info": "sensor_msgs/msg/CameraInfo",
        "/camera/camera/aligned_depth_to_color/image_raw": "sensor_msgs/msg/Image",
        "/camera/camera/aligned_depth_to_color/camera_info": "sensor_msgs/msg/CameraInfo",
        "/camera/camera/extrinsics/depth_to_color":
            "realsense2_camera_msgs/msg/Extrinsics",
        "/tf_static": "tf2_msgs/msg/TFMessage",
        "/camera/camera/depth/metadata":
            "realsense2_camera_msgs/msg/Metadata",
        "/camera/camera/color/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
        "/camera/camera/depth/image_rect_raw/compressed": "sensor_msgs/msg/CompressedImage",
        "/camera/camera/aligned_depth_to_color/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
        "/aft_mapped_to_init": "nav_msgs/msg/Odometry",
    }
    selected = sorted(
        name for name, message_type in live.items()
        if default_record_selected(BagTopic(name, message_type)))
    assert selected == [
        "/camera/camera/aligned_depth_to_color/camera_info",
        "/camera/camera/aligned_depth_to_color/image_raw",
        "/camera/camera/color/camera_info",
        "/camera/camera/color/image_raw",
        "/camera/camera/depth/camera_info",
        "/camera/camera/depth/image_rect_raw",
        "/camera/camera/depth/metadata",
        "/camera/camera/extrinsics/depth_to_color",
        "/livox/imu",
        "/livox/lidar",
        "/tf_static",
    ]
    # 逐帧曝光元数据默认保留，算法派生话题不勾选。
    assert default_record_selected(
        BagTopic("/camera/camera/depth/metadata",
                 "realsense2_camera_msgs/msg/Metadata"))
    assert not default_record_selected(
        BagTopic("/aft_mapped_to_init", "nav_msgs/msg/Odometry"))


def test_record_defaults_match_verified_cli_flags():
    arguments = build_record_arguments(Path("/tmp/x"), ["/livox/lidar"])
    assert arguments[arguments.index("--compression-format") + 1] == "zstd"
    assert arguments[arguments.index("--max-cache-size") + 1] == "134217728"
    assert arguments[arguments.index("--max-bag-size") + 1] == "1000000000"


def test_message_level_compression_is_available():
    """逐消息压缩: 写入时逐条压, 不产生"待压缩分片"积压。"""
    arguments = build_record_arguments(
        Path("/tmp/x"), ["/livox/lidar"], compression_mode="message")
    assert arguments[arguments.index("--compression-mode") + 1] == "message"
    assert arguments[arguments.index("--compression-format") + 1] == "zstd"
    assert arguments[
        arguments.index("--compression-queue-size") + 1] == "128"
    assert arguments[arguments.index("--compression-threads") + 1] == "1"
    # 文件级的队列单位是整片, 必须保持小值
    file_arguments = build_record_arguments(
        Path("/tmp/x"), ["/livox/lidar"], compression_mode="file")
    assert file_arguments[
        file_arguments.index("--compression-mode") + 1] == "file"
    assert file_arguments[
        file_arguments.index("--compression-queue-size") + 1] == "2"
    # 显式指定时优先生效
    custom = build_record_arguments(
        Path("/tmp/x"), ["/livox/lidar"], compression_queue_size=50,
        compression_threads=1)
    assert custom[custom.index("--compression-queue-size") + 1] == "50"
    assert custom[custom.index("--compression-threads") + 1] == "1"
