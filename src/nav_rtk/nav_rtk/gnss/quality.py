"""ROS-independent GNSS measurement gate and WGS84 ECEF/ENU conversion."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Fix:
    stamp: float
    latitude: float
    longitude: float
    altitude: float
    status: int
    covariance_type: int
    covariance: tuple


def validate_fix(fix, now, max_age=1.0, max_horizontal_std=0.5):
    if not all(math.isfinite(x) for x in (now, fix.stamp, fix.latitude, fix.longitude, fix.altitude)):
        return False, "定位数据包含无效数值"
    if not -90 <= fix.latitude <= 90 or not -180 <= fix.longitude <= 180:
        return False, "经纬度超出范围"
    if fix.status < 0:
        return False, "GNSS 未定位"
    if fix.stamp <= 0 or now - fix.stamp > max_age or fix.stamp - now > 0.05:
        return False, "定位时间过期或时钟不一致"
    if fix.covariance_type not in (1, 2, 3):
        return False, "定位协方差未知"
    cov = np.asarray(fix.covariance, dtype=float)
    if cov.size != 9 or not np.isfinite(cov).all():
        return False, "定位协方差无效"
    cov = cov.reshape(3, 3)
    if (not np.allclose(cov, cov.T) or np.min(np.linalg.eigvalsh(cov)) < -1e-9
            or np.any(np.diag(cov) <= 0)):
        return False, "定位协方差无效"
    if np.max(np.linalg.eigvalsh(cov[:2, :2])) > max_horizontal_std ** 2:
        return False, "水平定位不确定度过大"
    return True, "GNSS 观测有效；RTK 固定解状态尚未接入"


def ecef(latitude, longitude, altitude):
    lat, lon = math.radians(latitude), math.radians(longitude)
    a, e2 = 6378137.0, 6.6943799901413165e-3
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    return np.array([(n + altitude) * math.cos(lat) * math.cos(lon),
                     (n + altitude) * math.cos(lat) * math.sin(lon),
                     (n * (1 - e2) + altitude) * math.sin(lat)])


class EnuOrigin:
    def __init__(self, latitude, longitude, altitude):
        self.origin = (latitude, longitude, altitude)
        self.ecef = ecef(*self.origin)
        lat, lon = math.radians(latitude), math.radians(longitude)
        self.rotation = np.array([
            [-math.sin(lon), math.cos(lon), 0],
            [-math.sin(lat)*math.cos(lon), -math.sin(lat)*math.sin(lon), math.cos(lat)],
            [math.cos(lat)*math.cos(lon), math.cos(lat)*math.sin(lon), math.sin(lat)]])

    def convert(self, latitude, longitude, altitude):
        return self.rotation @ (ecef(latitude, longitude, altitude) - self.ecef)
