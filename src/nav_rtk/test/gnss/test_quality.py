from dataclasses import replace
import numpy as np
import pytest
from nav_rtk.gnss.quality import Fix, EnuOrigin, validate_fix


GOOD = Fix(100, 39.9, 116.4, 45, 0, 2, (0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.04))


@pytest.mark.parametrize("changes", [
    {"status": -1}, {"stamp": 95}, {"stamp": 101}, {"stamp": 0},
    {"latitude": 91}, {"longitude": float("nan")}, {"covariance_type": 0},
    {"covariance": (0,) * 9}, {"covariance": (1, 0, 0, 0, 1, 0, 0, 0, 1)},
    {"covariance": (0.01, 1, 0, 0, 0.01, 0, 0, 0, 0.04)},
    {"covariance": (0.01, 0, 0, 0, -0.01, 0, 0, 0, 0.04)},
])
def test_reject_unusable_observations(changes):
    assert not validate_fix(replace(GOOD, **changes), 100.1)[0]


def test_valid_fix_does_not_imply_rtk_fixed():
    valid, reason = validate_fix(GOOD, 100.1)
    assert valid
    assert "RTK 固定解状态尚未接入" in reason


def test_enu_directions_and_altitude():
    ref = EnuOrigin(0, 0, 0)
    assert np.allclose(ref.convert(0, 0, 0), 0)
    assert np.allclose(ref.convert(0, 0, 10), [0, 0, 10], atol=1e-6)
    east = ref.convert(0, 0.00001, 0)
    north = ref.convert(0.00001, 0, 0)
    assert 1.11 < east[0] < 1.12 and abs(east[1]) < 1e-6
    assert 1.10 < north[1] < 1.11 and abs(north[0]) < 1e-6
