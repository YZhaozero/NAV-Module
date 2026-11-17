from nav_rtk.bridge.epochs import EpochBuffer
from nav_rtk.bridge.observation import ObservationBuilder
from nav_rtk.parser.checksum import calculate_nmea_checksum
from datetime import datetime, timezone


def value(stamp=100., **fields):
    return dict(stamp=stamp, received_at=stamp+.01 if stamp else 100.01, latitude=29., longitude=119.,
                fix_quality=4, valid_position=True, covariance=None, **fields)


def test_epoch_waits_and_merges_without_changing_original_time():
    buffer = EpochBuffer()
    observation = value()
    assert not buffer.add(observation, 1.)
    assert not buffer.add(dict(observation, received_at=100.09), 1.1)
    update = dict(observation, covariance=[1.]*9, covariance_update=True,
                  horizontal_std_m=1., covariance_source='receiver_gst')
    buffer.add(update, 1.15)
    assert not buffer.drain(1.19)
    merged, = buffer.drain(1.21)
    assert merged['received_at'] == 100.01 and merged['stamp'] == 100.
    assert merged['covariance'] == [1.]*9 and 'covariance_update' not in merged
    assert not buffer.add(observation, 2.)
    assert not buffer.add(update, 2.)
    assert not buffer.add(value(99.), 2.)
    assert not buffer.drain(3.)
    assert buffer.counts['coalesced'] == buffer.counts['gst_merged'] == 1
    assert buffer.counts['late_gst'] == buffer.counts['backward'] == 1


def test_no_fix_duplicate_cannot_be_resurrected_by_old_gst():
    buffer = EpochBuffer()
    good = value()
    buffer.add(good, 1.)
    buffer.add(dict(good, valid_position=False, fix_quality=0, latitude=None, longitude=None), 1.1)
    buffer.add(dict(good, covariance_update=True, covariance=[1.]*9), 1.15)
    result, = buffer.drain(1.3)
    assert not result['valid_position'] and result['covariance'] is None


def test_queue_is_bounded_and_reconnect_preserves_watermark():
    buffer = EpochBuffer(capacity=2)
    buffer.add(value(101.), 1.)
    buffer.add(value(100.), 1.05)
    buffer.add(value(102.), 1.1)
    assert buffer.counts['overflow'] == 1
    assert not buffer.drain(1.21)  # wait for the earlier epoch as well
    assert [v['stamp'] for v in buffer.drain(1.3)] == [100., 101.]
    buffer.add(value(102.), 2.)
    buffer.clear_pending()
    buffer.add(value(99.), 3.)
    assert not buffer.drain(4.)


def test_gst_after_next_gga_joins_its_original_epoch():
    def sentence(body):
        return '$' + body + '*' + calculate_nmea_checksum(body)
    builder = ObservationBuilder()
    now = datetime(2026, 10, 21, 12, tzinfo=timezone.utc).timestamp()
    builder.feed(sentence('GNRMC,120000.00,A,3954.0000,N,11618.0000,E,0,0,211026,,,A'), now)
    first = builder.feed(sentence('GNGGA,120000.00,3954.0000,N,11618.0000,E,4,20,0.7,50,M,-10,M,1.2,01'), now+.01)
    second = builder.feed(sentence('GNGGA,120000.10,3954.0000,N,11618.0000,E,4,20,0.7,50,M,-10,M,1.2,01'), now+.11)
    late = builder.feed(sentence('GNGST,120000.00,1,1,1,0,0.1,0.2,0.3'), now+.15)
    assert late['stamp'] == first['stamp'] != second['stamp']
    assert late['received_at'] == first['received_at']
    buffer = EpochBuffer()
    for row in (first, second, late):
        buffer.add(row, 1.)
    rows = buffer.drain(1.3)
    assert len(rows) == 2 and rows[0]['covariance'] and rows[1]['covariance'] is None


def test_unknown_utc_stays_unknown_not_synthetic():
    buffer = EpochBuffer()
    result, = buffer.add(value(None), 1.)
    assert result['stamp'] is None
    assert buffer.snapshot()['counts']['missing_utc'] == 1


def test_invalid_future_date_does_not_poison_watermark():
    buffer = EpochBuffer()
    assert not buffer.add(dict(value(1e10), received_at=100.), 1.)
    buffer.add(value(100.), 1.1)
    assert [row['stamp'] for row in buffer.drain(1.4)] == [100.]
    assert buffer.counts['clock_mismatch'] == 1
