"""Small WGS84/ECEF/ENU conversion used by local trajectory analysis."""

import math


def ecef(longitude, latitude, altitude=0.0):
    longitude = math.radians(longitude)
    latitude = math.radians(latitude)
    eccentricity = 6.69437999014e-3
    radius = 6378137.0 / math.sqrt(
        1.0 - eccentricity * math.sin(latitude) ** 2)
    return (
        (radius + altitude) * math.cos(latitude) * math.cos(longitude),
        (radius + altitude) * math.cos(latitude) * math.sin(longitude),
        (radius * (1.0 - eccentricity) + altitude) * math.sin(latitude),
    )


class GeoOrigin:
    def __init__(self, longitude, latitude, altitude=0.0):
        self.values = [longitude, latitude, altitude]
        self.xyz = ecef(*self.values)
        longitude = math.radians(longitude)
        latitude = math.radians(latitude)
        self.rotation = (
            (-math.sin(longitude), math.cos(longitude), 0.0),
            (-math.sin(latitude) * math.cos(longitude),
             -math.sin(latitude) * math.sin(longitude), math.cos(latitude)),
            (math.cos(latitude) * math.cos(longitude),
             math.cos(latitude) * math.sin(longitude), math.sin(latitude)),
        )

    def enu(self, longitude, latitude, altitude=0.0):
        delta = [
            value - origin
            for value, origin in zip(
                ecef(longitude, latitude, altitude), self.xyz)]
        return tuple(
            sum(axis * value for axis, value in zip(row, delta))
            for row in self.rotation)

    def wgs84(self, east, north, up=0.0):
        xyz = [
            self.xyz[index] + sum(
                self.rotation[axis][index] * value
                for axis, value in enumerate((east, north, up)))
            for index in range(3)
        ]
        x, y, z = xyz
        rho = math.hypot(x, y)
        eccentricity = 6.69437999014e-3
        latitude = math.atan2(z, rho * (1.0 - eccentricity))
        for _ in range(8):
            radius = 6378137.0 / math.sqrt(
                1.0 - eccentricity * math.sin(latitude) ** 2)
            latitude = math.atan2(
                z + eccentricity * radius * math.sin(latitude), rho)
        return [math.degrees(math.atan2(y, x)), math.degrees(latitude)]
