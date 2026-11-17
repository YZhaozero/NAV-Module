"""Task list follows the current map library without deleting task history."""
import json

from nav_app.ui.map_tasks import MapTasksPage


def job(identifier, manifest, *, state='succeeded'):
    return dict(id=identifier, kind='navigation', state=state, attempt=1,
                created=1794636350., payload=dict(title='nav_map', cloud=str(manifest.with_suffix('.pcd'))),
                result=dict(scene=str(manifest)), progress={}, error='')


def manifest(path, title):
    path.write_text(json.dumps(dict(version=2, title=title)))
    return path


def test_current_title_and_source_distinguish_jobs_and_follow_rename(application, tmp_path):
    first = manifest(tmp_path / 'first.scene.json', '916')
    second = manifest(tmp_path / 'second.scene.json', '916')
    jobs = [job('first', first), job('second', second)]
    page = MapTasksPage(lambda _: None, lambda _: None, lambda _: None)
    page.refresh(jobs, directory=tmp_path)
    assert page.rows.count() == 2
    texts = [page.rows.item(i).text() for i in range(2)]
    assert texts[0] != texts[1]
    assert all(text.startswith('916 ·') and '提交于' in text for text in texts)
    assert 'first.scene.pcd' in texts[0]
    page.rows.setCurrentRow(1)
    manifest(second, '新名称')
    page.refresh(jobs, directory=tmp_path)
    assert page.selected()['id'] == 'second'
    assert page.rows.currentItem().text().startswith('新名称 ·')
    assert jobs[1]['payload']['title'] == 'nav_map'


def test_deleted_and_old_directory_results_disappear_but_live_and_failed_tasks_remain(application, tmp_path):
    current = tmp_path / 'nav_maps'; current.mkdir()
    old = tmp_path / 'nav-maps'; old.mkdir()
    present = manifest(current / 'current.scene.json', '当前地图')
    archived = manifest(old / 'old.scene.json', '旧目录地图')
    jobs = [job('current', present), job('old', archived),
            job('live', current / 'pending.scene.json', state='running'),
            job('failed', current / 'failed.scene.json', state='failed')]
    page = MapTasksPage(lambda _: None, lambda _: None, lambda _: None)
    page.refresh(jobs, directory=current)
    assert set(page.jobs) == {'current', 'live', 'failed'}
    assert page.use.isEnabled()
    present.unlink()
    page.refresh(jobs, directory=current)
    assert set(page.jobs) == {'live', 'failed'}
    assert page.pause.isEnabled()
    assert not page.use.isEnabled()
    assert len(jobs) == 4 and archived.is_file()
    page.refresh(jobs, directory=old)
    assert set(page.jobs) == {'old', 'live', 'failed'}


def test_deleting_last_result_clears_selection_and_navigation_action(application, tmp_path):
    path = manifest(tmp_path / 'map.scene.json', '地图')
    jobs = [job('map', path)]
    page = MapTasksPage(lambda _: None, lambda _: None, lambda _: None)
    page.refresh(jobs, directory=tmp_path)
    assert page.use.isEnabled()
    path.unlink()
    page.refresh(jobs, directory=tmp_path)
    assert page.rows.count() == 0
    assert page.selected() is None
    assert not page.use.isEnabled()
