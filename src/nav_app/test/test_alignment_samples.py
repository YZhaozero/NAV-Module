"""Synthetic selection/recovery checks; not a field accuracy certificate."""
import math
from nav_app.alignment_samples import StableAlignmentSamples


def observation(samples, t, x, error=0.):
    # Sideways device motion and a rotated geographic frame remain valid.
    return samples.add(t, (x, .2*x, 0.), (20.-.2*x+error, 30.+x, 0.))


def test_consistent_motion_retains_a_baseline_without_knowing_yaw():
    samples = StableAlignmentSamples()
    for i in range(80):
        observation(samples, i*.5, i*.4)
    assert len(samples.pairs) >= 30
    assert samples.rejected == 0
    for p, q in samples.pairs:
        assert math.dist((20.-p[1], 30.+p[0]), q[:2]) < 1e-10


def test_frozen_satellite_position_is_not_stable_when_lidar_moves():
    samples = StableAlignmentSamples()
    for i in range(50):
        samples.add(i*.5, (i*.5, 0., 0.), (10., 20., 0.))
    assert not samples.pairs
    assert samples.rejected > 0


def test_bad_window_recovers_without_manual_reset_or_loss_of_earlier_points():
    samples = StableAlignmentSamples()
    for i in range(30):
        observation(samples, i*.5, i*.4)
    earlier = list(samples.keyframes)
    observation(samples, 15., 12., error=10.)
    assert samples.rejected == 1
    assert list(samples.keyframes) == earlier
    for i in range(31, 50):
        observation(samples, i*.5, i*.4)
    assert len(samples.keyframes) > len(earlier)
    assert list(samples.keyframes)[:len(earlier)] == earlier


def test_stationary_jitter_does_not_fill_or_evict_travel_history():
    samples = StableAlignmentSamples()
    for i in range(50):
        observation(samples, i*.5, i*.4)
    for i in range(50, 60):
        observation(samples, i*.5, 19.6)
    size = len(samples.keyframes)
    first = samples.keyframes[0]
    for i in range(60, 200):
        observation(samples, i*.5, 19.6, error=.05*math.sin(i))
    assert len(samples.keyframes) == size
    assert samples.keyframes[0] == first


def test_data_gap_requires_new_consistency_window_but_retains_travel_points():
    samples = StableAlignmentSamples()
    for i in range(20):
        observation(samples, i*.5, i*.4)
    size = len(samples.keyframes)
    assert not observation(samples, 30., 20.)
    assert len(samples.recent) == 1
    assert len(samples.keyframes) == size


def test_memory_and_retained_age_are_bounded():
    samples = StableAlignmentSamples()
    for i in range(3000):
        observation(samples, i*.5, i*.4)
    assert len(samples.recent) == 6
    assert len(samples.keyframes) <= 100
    assert samples.keyframes[-1][0]-samples.keyframes[0][0] <= 180.
