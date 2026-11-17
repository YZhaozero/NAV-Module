"""One GGA epoch, not last-good fields from an aggregated UI state.

Receipt time, receiver UTC, MSL/ellipsoid altitude and reported GST uncertainty
are distinct. Quality=4 is not a covariance estimate. No receiver writes.
"""
from datetime import datetime, timezone, timedelta
from collections import OrderedDict
import math
from nav_rtk.parser.checksum import validate_nmea_sentence
from nav_rtk.parser.nmea_parser import parse_nmea_sentence


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def seconds(value):
    try:
        h, m, s = int(value[:2]), int(value[2:4]), float(value[4:])
        if not 0 <= h < 24 or not 0 <= m < 60 or not 0 <= s < 60:
            return None
        return h*3600+m*60+s
    except (ValueError, TypeError):
        return None


def covariance_fields(values):
    # GST latitude/longitude/altitude std -> ENU covariance diagonal.
    lat, lon, alt = values
    east, north, up = lon*lon, lat*lat, alt*alt
    if not all(math.isfinite(value) and value > 0 for value in (east, north, up)):
        return None
    return dict(covariance=[east,0.,0.,0.,north,0.,0.,0.,up],
                horizontal_std_m=max(lat, lon),covariance_source="receiver_gst")


class ObservationBuilder:
    def __init__(self):
        self.date = None
        self.date_received = 0.
        self.gst = None
        self.recent_gga = OrderedDict()

    def feed(self, text, received):
        if not validate_nmea_sentence(text):
            return None
        fields = text.partition("*")[0].lstrip("$").split(",")
        kind = fields[0][-3:]
        if kind == "GST" and len(fields) >= 9:
            utc = seconds(fields[1])
            values = [number(v) for v in fields[6:9]]
            if utc is not None and all(v is not None and v > 0 for v in values):
                accuracy = covariance_fields(values)
                if accuracy is None:
                    return None
                self.gst = (utc, values, received)
                if utc in self.recent_gga:
                    observation = self.recent_gga[utc]
                    if (observation["covariance"] is None
                            and 0 <= received-observation["received_at"] < 1):
                        # Enrich this epoch once; GST must not renew position age.
                        update = dict(observation, **accuracy, covariance_update=True)
                        self.recent_gga[utc] = dict(update)
                        return update
            return None
        if kind not in ("GGA", "RMC"):
            return None
        parsed = parse_nmea_sentence(text)
        if parsed is None:
            return None
        data = parsed.data
        if kind == "RMC":
            if data.date and seconds(data.utc_time) is not None:
                try:
                    self.date = datetime.strptime(data.date, "%Y-%m-%d").replace(tzinfo=timezone.utc)
                    self.date_received = received
                except ValueError:
                    pass
            return None
        utc = seconds(data.utc_time)
        stamp = None
        if self.date and utc is not None and 0 <= received-self.date_received < 2:
            candidate = self.date + timedelta(seconds=utc)
            # Adjacent midnight GGA may arrive before the new RMC date.
            choices = [candidate+timedelta(days=d) for d in (-1,0,1)]
            stamp = min(choices,key=lambda d:abs(d.timestamp()-received)).timestamp()
        field = lambda i: fields[i] if i < len(fields) else ""
        geoid = number(field(11)) if field(12) == "M" else None
        msl = number(data.altitude) if field(10) == "M" else None
        accuracy = dict(covariance=None,horizontal_std_m=None,covariance_source="unknown")
        if (self.gst and utc is not None and utc == self.gst[0]
                and 0 <= received-self.gst[2] < 1):
            accuracy = covariance_fields(self.gst[1])
        lat,lon=data.latitude,data.longitude
        coordinate_fields_valid=(field(3) in ("N","S") and field(5) in ("E","W"))
        for index in (2,4):
            raw=number(field(index))
            if raw is None or raw<0 or raw%100 >= 60:
                coordinate_fields_valid=False
        valid=(lat is not None and lon is not None and math.isfinite(lat) and math.isfinite(lon)
               and coordinate_fields_valid and abs(lat)<85 and abs(lon)<=180 and data.fix_quality in (1,2,4,5))
        observation = dict(schema=1,stamp=stamp,received_at=received,timestamp_source="receiver_utc" if stamp else "unavailable",
                    latitude=lat if valid else None,longitude=lon if valid else None,
                    fix_quality=data.fix_quality or 0,satellites=data.satellites_used,hdop=number(data.hdop),
                    altitude_msl=msl,geoid_separation_m=geoid,
                    altitude_ellipsoid=msl+geoid if msl is not None and geoid is not None else None,
                    differential_age_s=number(field(13)),station_id=field(14)[:32],
                    **accuracy,valid_position=valid)
        # Downstream qualification may mutate the returned dictionary.
        self.recent_gga[utc] = dict(observation)
        self.recent_gga.move_to_end(utc)
        while len(self.recent_gga) > 32:
            self.recent_gga.popitem(last=False)
        return observation
