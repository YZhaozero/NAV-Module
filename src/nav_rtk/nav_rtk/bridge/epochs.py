"""Bounded GST join and strictly increasing receiver epochs, never restamped."""
from collections import Counter, OrderedDict, deque
import math
from nav_rtk.parser.observation_time import MAX_EARLY_S, MAX_AGE_S


class EpochBuffer:
    def __init__(self, hold_s=.2, capacity=64):
        self.hold_s = hold_s
        self.capacity = capacity
        self.pending = OrderedDict()
        self.last_stamp = None
        self.counts = Counter()
        self.events = deque(maxlen=32)

    def _reject(self, reason, observation):
        self.counts[reason] += 1
        self.events.append(dict(reason=reason, stamp=observation.get('stamp'),
                                received_at=observation.get('received_at'),
                                raw=observation.get('_raw_sentence')))

    def add(self, observation, now):
        value = dict(observation)
        stamp = value.get('stamp')
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= 0:
            self._reject('missing_utc', value)
            value.pop('_raw_sentence', None)
            return [value] if not value.get('covariance_update') else []
        received = value.get('received_at')
        if (type(received) not in (int, float) or not math.isfinite(received)
                or not -MAX_EARLY_S <= received - stamp <= MAX_AGE_S):
            # A bad future date must not poison the monotonic watermark and
            # suppress all subsequent correctly timed fixes.
            self._reject('clock_mismatch', value)
            return []
        # NMEA precision is below a microsecond; avoid float representation noise.
        key = round(stamp * 1_000_000)
        if self.last_stamp is not None and key <= self.last_stamp:
            self._reject('late_gst' if value.get('covariance_update') else
                         'duplicate' if key == self.last_stamp else 'backward', value)
            return []
        if key in self.pending:
            deadline, previous = self.pending[key]
            if value.get('covariance_update'):
                # GST can enrich precision, not change position, validity or age.
                fields = ('latitude', 'longitude', 'fix_quality', 'valid_position', 'coordinate_system')
                if all(previous.get(field) == value.get(field) for field in fields):
                    for field in ('covariance', 'horizontal_std_m', 'covariance_source'):
                        previous[field] = value.get(field)
                    self.counts['gst_merged'] += 1
            else:
                # Latest validity wins within the bounded epoch, but never renew age.
                value['received_at'] = previous['received_at']
                if value.get('covariance') is None and all(
                        previous.get(field) == value.get(field)
                        for field in ('latitude', 'longitude', 'fix_quality', 'coordinate_system')):
                    for field in ('covariance', 'horizontal_std_m', 'covariance_source'):
                        value[field] = previous.get(field)
                self.pending[key] = (deadline, value)
                self.counts['coalesced'] += 1
            return []
        if value.get('covariance_update'):
            self._reject('orphan_gst', value)
            return []
        if len(self.pending) >= self.capacity:
            self._reject('overflow', value)
            return []
        self.pending[key] = (now + self.hold_s, value)
        return []

    def drain(self, now):
        result = []
        for key in sorted(self.pending):
            deadline, value = self.pending[key]
            if now < deadline:
                break
            del self.pending[key]
            if self.last_stamp is not None and key <= self.last_stamp:
                self._reject('backward', value)
                continue
            value.pop('covariance_update', None)
            value.pop('_raw_sentence', None)
            self.last_stamp = key
            self.counts['published'] += 1
            result.append(value)
        return result

    def clear_pending(self):
        # Retain watermark across reconnects so an old UART batch cannot rewind UTC.
        self.pending.clear()

    def snapshot(self):
        return dict(counts=dict(self.counts), pending=len(self.pending),
                    hold_s=self.hold_s, events=list(self.events))
