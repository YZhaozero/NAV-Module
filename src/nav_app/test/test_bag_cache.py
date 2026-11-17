import hashlib
import os
import pytest
from nav_app.features import bag_cache


@pytest.mark.parametrize("extension", ["db3", "mcap"])
def test_closed_shards_numeric_order_active_is_never_synced(tmp_path, monkeypatch, extension):
    calls = []
    monkeypatch.setattr(bag_cache, "release_file_cache", lambda p: calls.append(p.name) or True)
    for index in (0, 1, 2, 10):
        (tmp_path / f"capture_{index}.{extension}").write_bytes(b"data")
    worker = bag_cache.BagCacheWriter(tmp_path)
    worker.sweep()
    assert calls == [f"capture_{i}.{extension}" for i in (0, 1, 2)]
    worker.sweep()
    assert len(calls) == 3
    (tmp_path / f"capture_11.{extension}").write_bytes(b"new")
    worker.sweep()
    assert calls[-1] == f"capture_10.{extension}"
    assert f"capture_11.{extension}" not in calls
    worker.sweep(final=True)
    assert calls[-1] == f"capture_11.{extension}"


@pytest.mark.skipif(not hasattr(os, "posix_fadvise"), reason="Linux page cache API")
def test_sync_and_evict_keeps_all_bytes_and_rejects_symlink(tmp_path):
    path = tmp_path / "capture_0.db3"
    data = os.urandom(1024 * 1024)
    path.write_bytes(data)
    assert bag_cache.release_file_cache(path)
    assert hashlib.sha256(path.read_bytes()).digest() == hashlib.sha256(data).digest()
    alias = tmp_path / "alias.db3"
    alias.symlink_to(path)
    with pytest.raises(OSError):
        bag_cache.release_file_cache(alias)


def test_sync_failure_is_reported_without_marking_shard_done(tmp_path, monkeypatch):
    for index in (0, 1):
        (tmp_path / f"capture_{index}.db3").write_bytes(b"data")
    def fail(_path):
        raise OSError("disk writeback failed")
    monkeypatch.setattr(bag_cache, "release_file_cache", fail)
    worker = bag_cache.BagCacheWriter(tmp_path, interval=.001).start()
    worker._thread.join(timeout=2)
    result = worker.close()
    assert "disk writeback failed" in result["error"]
    assert result["released_shards"] == 0
