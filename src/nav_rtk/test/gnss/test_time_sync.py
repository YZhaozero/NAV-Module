import numpy as np
import pytest
from nav_rtk.gnss.time_sync import estimate_offset


def speed(t):
    return 2 + np.sin(t * .73) + .4 * np.sin(t * 1.81) + .25 * np.cos(t * .23)


@pytest.mark.parametrize("offset", [.37, -.62])
def test_known_clock_offset_recovered_on_held_out_motion(offset):
    lio = np.arange(0, 40, .02)
    gnss = np.arange(3, 37, .5)
    result = estimate_offset(lio, speed(lio), gnss, speed(gnss + offset))
    assert result.accepted
    assert abs(result.offset_seconds - offset) <= .011
    assert result.validation_correlation > .99


def test_stationary_and_constant_speed_are_not_a_clock_calibration():
    times = np.arange(0, 40, .1)
    for speed_value in (0., 2.):
        result = estimate_offset(times, times*0 + speed_value, times, times*0 + speed_value)
        assert not result.accepted and result.offset_seconds is None


def test_long_lio_outage_cannot_be_interpolated_into_evidence():
    lio = np.r_[np.linspace(0., .2, 20), np.linspace(39., 40., 20)]
    gt = np.arange(3, 37, .5)
    assert not estimate_offset(lio, speed(lio), gt, speed(gt)).accepted


def test_small_overlap_cannot_look_like_a_good_clock_calibration():
    lio = np.arange(18., 40., .02)
    gnss = np.arange(3., 37., .5)
    assert not estimate_offset(lio, speed(lio), gnss, speed(gnss + .37)).accepted


def test_periodic_motion_and_boundary_offset_are_ambiguous():
    lio = np.arange(0., 40., .02)
    gnss = np.arange(3., 37., .1)
    periodic = lambda t: 2. + np.sin(2 * np.pi * t)
    assert not estimate_offset(lio, periodic(lio), gnss, periodic(gnss + .3)).accepted
    assert not estimate_offset(lio, speed(lio), gnss, speed(gnss + 2.)).accepted
