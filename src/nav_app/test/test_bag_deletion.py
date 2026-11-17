"""Deletion must never cross a bag boundary or race a changed confirmation."""
import os
from pathlib import Path
import pytest
from nav_app.features.bag_library import prepare_bag_deletion, delete_bag
from test_bag_library import bag


def test_deletes_one_confirmed_bag_and_preserves_sibling_map_and_bag(tmp_path):
    target = tmp_path/'capture'
    sibling = tmp_path/'keep'
    bag(target); bag(sibling)
    (tmp_path/'scene.pcd').write_bytes(b'original-map')
    plan = prepare_bag_deletion(target, [tmp_path])
    assert delete_bag(plan) == plan.size_bytes > 0
    assert not target.exists()
    assert (sibling/'segment.db3').read_bytes() == b'test-payload'
    assert (tmp_path/'scene.pcd').read_bytes() == b'original-map'


def test_refuses_root_outside_and_non_bag(tmp_path):
    target = tmp_path/'capture'; bag(target)
    with pytest.raises(ValueError): prepare_bag_deletion(target, [target])
    with pytest.raises(ValueError): prepare_bag_deletion(target, [tmp_path/'other'])
    with pytest.raises(ValueError): prepare_bag_deletion(tmp_path, [tmp_path.parent])
    assert target.exists()


@pytest.mark.parametrize('change', ['append', 'new_file', 'replace_directory'])
def test_changed_snapshot_requires_new_confirmation(tmp_path, change):
    target = tmp_path/'capture'; bag(target)
    plan = prepare_bag_deletion(target, [tmp_path])
    if change == 'append':
        with (target/'segment.db3').open('ab') as f: f.write(b'new')
    elif change == 'new_file':
        (target/'new.db3').write_bytes(b'new')
    else:
        target.rename(tmp_path/'old'); bag(target)
    with pytest.raises(ValueError, match='已变化'): delete_bag(plan)
    assert target.exists()


def test_rejects_nested_bag_or_map(tmp_path):
    target = tmp_path/'capture'; bag(target)
    (target/'map.pcd').write_bytes(b'map')
    with pytest.raises(ValueError, match='地图'): prepare_bag_deletion(target, [tmp_path])
    (target/'map.pcd').unlink()
    bag(target/'nested')
    with pytest.raises(ValueError, match='其他录包'): prepare_bag_deletion(target, [tmp_path])


def test_rejects_links_without_touching_external_target(tmp_path):
    target = tmp_path/'capture'; bag(target)
    external = tmp_path/'keep'; external.mkdir(); (external/'precious').write_bytes(b'keep')
    try:
        (target/'linked').symlink_to(external, target_is_directory=True)
    except OSError:
        pytest.skip('symlink creation unavailable')
    with pytest.raises(ValueError, match='链接'): prepare_bag_deletion(target, [tmp_path])
    assert (external/'precious').read_bytes() == b'keep'


def test_failed_deletion_reports_partial_result(tmp_path, monkeypatch):
    import nav_app.features.bag_library as lib
    target = tmp_path/'capture'; bag(target)
    plan = prepare_bag_deletion(target, [tmp_path])
    def fail(_path): raise PermissionError('read-only')
    monkeypatch.setattr(lib.shutil, 'rmtree', fail)
    with pytest.raises(OSError, match='删除未完成'): delete_bag(plan)
    assert target.exists()
