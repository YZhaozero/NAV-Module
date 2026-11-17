import json
from pathlib import Path
import yaml
import pytest
from nav_app.features.bag_calibration import restore_bag_calibration


def fixture_bag(tmp_path, selector="laserMapping"):
    bag = tmp_path/"bag"; bag.mkdir()
    values = {"extrin_calib.Rcl": [1.,0.,0.,0.,1.,0.,0.,0.,1.], "extrin_calib.Pcl": [1.,2.,3.],
              "extrin_calib.extrinsic_R": [1.,0.,0.,0.,1.,0.,0.,0.,1.], "extrin_calib.extrinsic_T": [0.,0.,0.],
              "time_offset.img_time_offset": .1, "time_offset.imu_time_offset": 0., "time_offset.exposure_time_init": 0.}
    parameters = {k: {"type": 8 if isinstance(v, list) else 3, "double_array_value" if isinstance(v, list) else "double_value": v} for k,v in values.items()}
    camera = {"cam_model": {"type":4, "string_value":"Pinhole"}}
    camera.update({k: {"type":3,"double_value":v} for k,v in dict(cam_fx=910.,cam_fy=911.,cam_width=1280.,cam_height=720.).items()})
    (bag/"nav_manifest.json").write_text(json.dumps({"parameters_start": {
        "/laserMapping": {"parameters":parameters}, "/parameter_blackboard":{"parameters":camera}}}))
    base = tmp_path/"base.yaml"
    base.write_text(yaml.safe_dump({selector:{"ros__parameters":{
        "extrin_calib":{"Rcl":[0.]*9}, "time_offset":{"img_time_offset":0.},
        "vio":{"max_visual_frames":150}}}}))
    return bag, base


def test_old_bag_calibration_is_restored_but_runtime_limits_are_retained(tmp_path):
    bag, base = fixture_bag(tmp_path)
    original = (bag/"nav_manifest.json").read_bytes()
    algorithm, camera = restore_bag_calibration(bag, base, tmp_path/"cache")
    params = yaml.safe_load(open(algorithm))["laserMapping"]["ros__parameters"]
    assert params["extrin_calib.Pcl"] == [1.,2.,3.]
    assert params["time_offset.img_time_offset"] == .1
    assert params["vio"]["max_visual_frames"] == 150
    assert "extrin_calib" not in params
    assert (bag/"nav_manifest.json").read_bytes() == original
    assert restore_bag_calibration(bag, base, tmp_path/"cache") == (algorithm, camera)


@pytest.mark.parametrize("selector", ["laserMapping", "/laserMapping", "/**"])
def test_supported_fast_livo_parameter_selectors(selector, tmp_path):
    bag, base = fixture_bag(tmp_path, selector)
    algorithm, _camera = restore_bag_calibration(
        bag, base, tmp_path/"cache")
    document = yaml.safe_load(open(algorithm))
    params = document[selector]["ros__parameters"]
    assert params["extrin_calib.Pcl"] == [1., 2., 3.]
    assert params["vio"]["max_visual_frames"] == 150


def test_invalid_fast_livo_base_layout_has_actionable_error(tmp_path):
    bag, base = fixture_bag(tmp_path)
    base.write_text(yaml.safe_dump({"other_node": {"ros__parameters": {}}}))
    with pytest.raises(ValueError, match="ros__parameters"):
        restore_bag_calibration(bag, base, tmp_path/"cache")


def test_identical_calibration_from_different_bags_keeps_separate_provenance(tmp_path):
    bag, base = fixture_bag(tmp_path)
    other_bag = tmp_path/"other-bag"
    other_bag.mkdir()
    (other_bag/"nav_manifest.json").write_bytes(
        (bag/"nav_manifest.json").read_bytes())
    first, _ = restore_bag_calibration(bag, base, tmp_path/"cache")
    second, _ = restore_bag_calibration(other_bag, base, tmp_path/"cache")
    assert first != second
    first_source = json.loads((Path(first).parent/"source.json").read_text())
    second_source = json.loads((Path(second).parent/"source.json").read_text())
    assert first_source["bag"] == str(bag.resolve())
    assert second_source["bag"] == str(other_bag.resolve())


def test_missing_recorded_time_is_not_replaced_by_live_value(tmp_path):
    bag, base = fixture_bag(tmp_path)
    path = bag/"nav_manifest.json"; doc = json.loads(path.read_text())
    del doc["parameters_start"]["/laserMapping"]["parameters"]["time_offset.img_time_offset"]
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="时间参数"):
        restore_bag_calibration(bag, base, tmp_path/"cache")


def test_untracked_bag_does_not_silently_inherit_device_calibration(tmp_path):
    with pytest.raises(ValueError, match="标定快照"):
        restore_bag_calibration(tmp_path/"unknown", tmp_path/"base", tmp_path/"cache")


def test_malformed_snapshot_reports_data_error(tmp_path):
    bag, base = fixture_bag(tmp_path)
    path = bag/"nav_manifest.json"
    document = json.loads(path.read_text())
    document["parameters_start"]["/laserMapping"]["parameters"][
        "extrin_calib.Rcl"] = {"type": 8}
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="缺少参数值"):
        restore_bag_calibration(bag, base, tmp_path/"cache")


def test_lidar_only_replay_does_not_require_or_activate_camera_intrinsics(tmp_path):
    bag, base = fixture_bag(tmp_path)
    path = bag / 'nav_manifest.json'
    document = json.loads(path.read_text())
    del document['parameters_start']['/parameter_blackboard']
    path.write_text(json.dumps(document))
    algorithm, camera = restore_bag_calibration(bag, base, tmp_path / 'cache', require_camera=False)
    assert Path(algorithm).is_file() and camera == ''
    with pytest.raises(ValueError, match='相机参数'):
        restore_bag_calibration(bag, base, tmp_path / 'visual-cache', require_camera=True)
