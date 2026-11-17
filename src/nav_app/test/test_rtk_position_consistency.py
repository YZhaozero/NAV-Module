import json
import math
import threading
from collections import deque
from types import SimpleNamespace

import pytest
from PyQt5.QtWidgets import QApplication
from std_msgs.msg import String

from nav_app.features.wtrtk import RtkRosAdapter
from nav_app.ui.wtrtk_page import RtkMonitorTab
from nav_rtk.bridge.rtk_node import RtkBridge
from nav_rtk.bridge.epochs import EpochBuffer


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


def observation(stamp=100.1, received=100., quality=1):
    return dict(schema=1, stamp=stamp, received_at=received, timestamp_source='receiver_utc',
        latitude=29. if quality else None, longitude=119. if quality else None,
        valid_position=bool(quality), fix_quality=quality, satellites=7,
        altitude_ellipsoid=20., altitude_msl=20., covariance=None)


def bridge_for(data, hold_s=0):
    return SimpleNamespace(_lock=threading.Lock(), _pending_frame=None,
        _observations=deque(data), _pending_fixes=deque(maxlen=100),
        _epochs=EpochBuffer(hold_s=hold_s),
        observation_publisher=Publisher(), fix_publisher=Publisher())


def test_bridge_waits_for_small_future_epoch_without_rewriting_timestamp(monkeypatch):
    import nav_rtk.bridge.rtk_node as module
    now = [100.]
    monkeypatch.setattr(module.time, 'time', lambda: now[0])
    bridge = bridge_for([observation()])
    RtkBridge._publish_frame(bridge)
    assert len(bridge.observation_publisher.messages) == 1
    assert not bridge.fix_publisher.messages, 'wait for the epoch instead of publishing an invalid fix'
    now[0] = 100.12
    RtkBridge._publish_frame(bridge)
    fix = bridge.fix_publisher.messages[-1]
    assert fix.status.status >= 0 and fix.latitude == 29.
    assert fix.header.stamp.sec + fix.header.stamp.nanosec/1e9 == pytest.approx(100.1)
    raw = json.loads(bridge.observation_publisher.messages[0].data)
    assert raw['received_at'] == 100. and raw['stamp'] == 100.1


@pytest.mark.parametrize('data', [observation(stamp=None), observation(stamp=105.),
    observation(stamp=98.), observation(stamp=100., quality=0)])
def test_bridge_still_rejects_invalid_missing_stale_or_far_future(monkeypatch, data):
    import nav_rtk.bridge.rtk_node as module
    monkeypatch.setattr(module.time, 'time', lambda: 100.)
    bridge = bridge_for([data])
    RtkBridge._publish_frame(bridge)
    if data['stamp'] is None:
        assert not bridge.fix_publisher.messages
        assert json.loads(bridge.observation_publisher.messages[0].data)['stamp'] is None
    elif data['stamp'] in (105., 98.):
        assert not bridge.fix_publisher.messages and not bridge.observation_publisher.messages
        assert bridge._epochs.counts['clock_mismatch'] == 1
    else:
        assert bridge.fix_publisher.messages[-1].status.status < 0




def test_bridge_publishes_one_merged_fix_and_observation_per_epoch(monkeypatch):
    import nav_rtk.bridge.rtk_node as module
    now = [100.]
    mono = [1.]
    monkeypatch.setattr(module.time, 'time', lambda: now[0])
    monkeypatch.setattr(module.time, 'monotonic', lambda: mono[0])
    raw = observation(stamp=100.)
    bridge = bridge_for([raw], hold_s=.2)
    RtkBridge._publish_frame(bridge)
    assert not bridge.observation_publisher.messages
    bridge._observations.extend([dict(raw, received_at=100.1),
        dict(raw, covariance_update=True, covariance=[.01]*9,
             horizontal_std_m=.1, covariance_source='receiver_gst')])
    now[0] = 100.15
    mono[0] = 1.15
    RtkBridge._publish_frame(bridge)
    assert not bridge.fix_publisher.messages
    now[0] = 100.21
    mono[0] = 1.21
    RtkBridge._publish_frame(bridge)
    assert len(bridge.fix_publisher.messages) == len(bridge.observation_publisher.messages) == 1
    message = json.loads(bridge.observation_publisher.messages[0].data)
    assert message['stamp'] == message['received_at'] == 100.
    assert message['covariance_source'] == 'receiver_gst'
    assert bridge.fix_publisher.messages[0].position_covariance_type == 2
    bridge._observations.extend([raw, observation(stamp=99.)])
    RtkBridge._publish_frame(bridge)
    assert len(bridge.fix_publisher.messages) == len(bridge.observation_publisher.messages) == 1




def test_realtime_coordinates_follow_gga_even_when_wtrtk_disagrees(monkeypatch):
    import nav_app.features.wtrtk as module
    from nav_app.features import log_store
    monkeypatch.setattr(log_store, 'log_store', lambda: SimpleNamespace(write=lambda *a: None))
    callbacks = {}
    node = SimpleNamespace(create_subscription=lambda typ, topic, callback, qos: callbacks.setdefault(topic, callback))
    adapter = RtkRosAdapter(node)
    app = QApplication.instance() or QApplication([])
    tab = RtkMonitorTab(adapter)
    assert '/rtk/observation' in callbacks
    callbacks['/rtk/frame'](String(data=json.dumps(dict(index=1, latitude=None, longitude=None, positioningText='初始化'))))
    callbacks['/rtk/observation'](String(data=json.dumps(observation(quality=4))))
    tab.refresh()
    assert tab._tiles['latitude'][0].text() == '29.00000000'
    assert tab._tiles['longitude'][0].text() == '119.00000000'
    assert '固定解' in tab._tiles['positioningText'][0].text()
    # Loss of GGA validity must clear coordinates even while WTRTK reports an old fix.
    callbacks['/rtk/frame'](String(data=json.dumps(dict(index=2, latitude=29., longitude=119., positioningText='固定解'))))
    callbacks['/rtk/observation'](String(data=json.dumps(observation(quality=0))))
    tab.refresh()
    assert tab._tiles['latitude'][0].text() == '--'
    assert '未定位' in tab._tiles['positioningText'][0].text()
    adapter.observation_received_at -= 3.
    tab.refresh()
    assert '过期' in tab._tiles['positioningText'][0].text()
    tab.close()
