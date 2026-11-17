"""Keep receiver UTC intact; briefly queue early epochs instead of erasing fixes."""
import math

# Five 20 Hz receiver cycles. This is a bounded scheduling allowance, not a
# measured clock correction or a claim of time/position accuracy.
MAX_EARLY_S = .25
MAX_AGE_S = 1.5


def time_state(observation, now):
    stamp = observation.get('stamp')
    received = observation.get('received_at', now)
    if (observation.get('timestamp_source') != 'receiver_utc'
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in (stamp, received, now))
            or stamp <= 0):
        return 'unavailable'
    # A clock jump or a large original discrepancy cannot become valid merely
    # by waiting in a queue. Preserve the acquisition-time check separately.
    if not -MAX_EARLY_S <= received-stamp <= MAX_AGE_S or now < received-.05:
        return 'clock_mismatch'
    if now-stamp > MAX_AGE_S:
        return 'stale'
    if now < stamp:
        return 'waiting'
    return 'ready'
