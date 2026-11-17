import json
from types import SimpleNamespace

from nav_app.features.rtk_readiness import summary


def outdoor(**observation):
    observation.setdefault('timestamp_source', 'receiver_utc')
    return SimpleNamespace(clock=lambda: 100., observation=observation, alignment_reason="等待行走对齐")


def test_no_fix_is_not_misreported_as_a_time_problem():
    adapter = SimpleNamespace(frame_fresh=True, frame={'positioningText': '初始化'})
    for stamp in (None, 100., 100.1):
        value = summary(adapter, outdoor(received_at=100., stamp=stamp, fix_quality=0, satellites=7))
        assert '未定位' in value and '报告卫星数 7' in value
        assert '接收器尚未输出有效位置' in value
        assert '时间缺失' not in value and '时间相差' not in value


def test_new_fixed_observation_separates_time_failure_from_solution_type():
    for stamp, expected in ((None, '时间缺失'), (101., '1.000 秒'), (98., '2.000 秒')):
        value = summary(None, outdoor(received_at=100., stamp=stamp, fix_quality=4, satellites=12))
        assert 'RTK 固定解' in value and expected in value and '暂不用于导航' in value


def test_pending_epoch_and_missing_date_keep_receiver_coordinates_visible():
    for stamp, expected in ((100.1, '等待同一时刻'), (None, '时间缺失')):
        value = summary(None, outdoor(received_at=100., stamp=stamp, fix_quality=1,
                                     latitude=29., longitude=119.))
        assert '纬度 29.0000000' in value and expected in value


def test_stale_fixed_observation_is_not_displayed_as_current():
    value = summary(None, outdoor(received_at=90., stamp=90., fix_quality=4, satellites=12))
    assert '固定解' not in value and '等待新的' in value


def test_fresh_wtrtk_does_not_restore_a_stale_gga_solution():
    adapter = SimpleNamespace(frame_fresh=True, frame={'positioningText': 'RTK 固定解'})
    value = summary(adapter, outdoor(received_at=90., stamp=90., fix_quality=4, satellites=12))
    assert '固定解' not in value and '已过期' in value


def test_valid_position_and_coordinate_rejection_are_visible():
    obs = dict(received_at=100., stamp=100., fix_quality=4, satellites=12,
               latitude=29., longitude=119., valid_position=True)
    value = summary(None, outdoor(**obs))
    assert '纬度 29.0000000' in value and '经度 119.0000000' in value
    obs.update(valid_position=False, coordinate_reason='坐标系尚未转换')
    assert '坐标系尚未转换' in summary(None, outdoor(**obs))


def test_adapter_logs_receiver_evidence_without_secrets_and_at_bounded_rate(monkeypatch):
    import nav_app.features.wtrtk as module
    from nav_app.features import log_store
    writes = []
    monkeypatch.setattr(log_store, 'log_store', lambda: SimpleNamespace(write=lambda *args: writes.append(args)))
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    node = SimpleNamespace(create_subscription=lambda *a: None)
    adapter = module.RtkRosAdapter(node)
    msg = SimpleNamespace(data=json.dumps(dict(receiver={'messages': {'GNGGA': {'raw': 'test-gga'}}},
        corrections={'phase': 'waiting', 'password': 'never-log-this'}, stats={}, serial={})))
    for _ in range(20): adapter._on_status(msg)
    assert len(writes) == 1 and 'test-gga' in writes[0][2]
    assert 'never-log-this' not in writes[0][2]
    clock[0] += 1.
    adapter._on_status(msg)
    assert len(writes) == 2
