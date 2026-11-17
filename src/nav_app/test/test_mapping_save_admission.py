"""Saving the live mapper shares its reservation; other owners still block it."""
import time
from types import SimpleNamespace

import pytest

from nav_app.features.mapping import MappingController, MAPPING_BACKENDS
from nav_app.features.task_coordinator import TaskCoordinator
from nav_app.ui.operations import OperationsController


@pytest.mark.parametrize("owner,allowed", [("mapping", True), ("navigation", False), ("recording", False)])
def test_current_map_save_respects_foreground_owner(tmp_path, monkeypatch, owner, allowed):
    monkeypatch.setenv("NAV_RUNTIME_DIR", str(tmp_path / "runtime"))
    coordinator = TaskCoordinator(tmp_path / "tasks", systemd=False)
    calls = []
    mapper = SimpleNamespace(
        _recovery_path=None, process=SimpleNamespace(active=True),
        backend=MAPPING_BACKENDS["woosh_dlio"],
        ros=SimpleNamespace(save_map=lambda *args: calls.append(args) or (True, "saved")))
    operations = SimpleNamespace(
        window=SimpleNamespace(mapping_controller=mapper), task_coordinator=coordinator,
        controller=SimpleNamespace(library_delete_in_progress=False),
        library=SimpleNamespace(worker=SimpleNamespace(busy=False)),
        audit_process=SimpleNamespace(active=False),
        rtk=SimpleNamespace(adapter=SimpleNamespace(available=False)))
    mapper.admission_guard = lambda operation: OperationsController.algorithm_admission(operations, operation)
    try:
        deadline = time.monotonic() + 5
        while not coordinator.snapshot()["loaded"] and time.monotonic() < deadline:
            time.sleep(.01)
        assert coordinator.reserve_idle(owner)
        success, message = MappingController.save_map(mapper, str(tmp_path), "current", .05)
        assert success is allowed, message
        assert len(calls) == int(allowed)
        assert coordinator.snapshot()["foreground"] == owner
        if allowed:
            assert calls[0][1:] == (str(tmp_path), "current", .05)
        assert not coordinator.reserve_idle("unrelated-task")
    finally:
        coordinator.close()
        coordinator.thread.join(timeout=5)
        assert not coordinator.thread.is_alive()
