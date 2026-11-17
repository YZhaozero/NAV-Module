import pytest
from nav_rtk.parser.observation_time import time_state


def fix(stamp=100.1, received=100.):
    return dict(stamp=stamp, received_at=received, timestamp_source='receiver_utc')


def test_bounded_wait_keeps_acquisition_time_and_measurement_time_separate():
    data = fix()
    assert time_state(data, 100.) == 'waiting'
    assert time_state(data, 100.11) == 'ready'
    assert data['stamp'] == 100.1 and data['received_at'] == 100.
    assert time_state(data, 101.7) == 'stale'
    assert time_state(fix(101.), 101.1) == 'clock_mismatch', 'waiting cannot bless a large original discrepancy'


@pytest.mark.parametrize('changes', [{'stamp':None}, {'stamp':float('nan')}, {'stamp':float('inf')},
    {'received_at':float('nan')}, {'timestamp_source':'receive'}])
def test_unavailable_timing_never_uses_receipt_as_measurement(changes):
    assert time_state(dict(fix(), **changes), 100.) == 'unavailable'


def test_clock_rollback_and_old_packet_fail_closed():
    assert time_state(fix(), 90.) == 'clock_mismatch'
    assert time_state(fix(90.), 100.) == 'clock_mismatch'
