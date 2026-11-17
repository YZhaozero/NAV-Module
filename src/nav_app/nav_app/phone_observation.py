"""Translate phone RTKLIB solutions at their measurement epochs, not arrival time."""
import math


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def observation(solution, received):
    if (not isinstance(solution, dict) or solution.get('source') != 'phone_software_rtk'
            or solution.get('valid') is not True or solution.get('fresh') is not True
            or type(solution.get('quality')) is not int or solution['quality'] not in (1, 2)):
        return None
    stamp = solution.get('utc_ms')
    lon, lat = solution.get('longitude'), solution.get('latitude')
    sigma = solution.get('horizontal_sigma_m')
    if (not all(number(v) for v in (stamp, lon, lat, sigma, received))
            or stamp <= 0 or abs(lon) > 180 or abs(lat) >= 85 or sigma <= 0):
        return None
    stamp /= 1000.
    if not 0 <= received-stamp <= 10.:
        return None
    # RTKLIB: 1=fixed,2=float. Receiver GGA: 4=fixed,5=float.
    return dict(schema=1, source='phone_software_rtk', timestamp_source='phone_gnss_utc',
        stamp=stamp, received_at=received, longitude=lon, latitude=lat,
        coordinate_system='WGS84', fix_quality={1:4, 2:5}[solution['quality']],
        horizontal_std_m=sigma, covariance_source='phone_rtklib',
        altitude_ellipsoid=solution.get('altitude_ellipsoid_m'),
        differential_age_s=solution.get('correction_age_s'),
        source_id=solution.get('source_id'), session_id=solution.get('session_id'))


def historical_time_ready(data, now):
    stamp, received = data.get('stamp'), data.get('received_at')
    return (data.get('source') in ('phone_software_rtk','phone_continuity')
        and data.get('timestamp_source') == 'phone_gnss_utc'
        and all(number(v) for v in (stamp, received, now))
        and stamp > 0 and 0 <= received-stamp <= 10.
        and -.05 <= now-received <= 2. and 0 <= now-stamp <= 10.)


def motion_observation(solution, received):
    """An explicitly selected experimental position, with native quality separate.

    Prediction, a changed absolute origin, or merely finite numerical output
    cannot initialize/recalibrate the geographic transform.
    """
    scalar=scalar_observation(solution,received)
    if scalar is not None:return scalar
    if (not isinstance(solution,dict) or solution.get('schema_version')!=1
            or solution.get('source')!='phone_continuity' or solution.get('fresh') is not True
            or solution.get('absolute_accuracy_verified') is not False
            or solution.get('coordinate_frame')!='phone_gnss_enu'
            or solution.get('status')!='observed' or 'quality' in solution
            or type(solution.get('position_observed')) is not bool
            or solution.get('raw_motion_observed') is not True
            or solution.get('delta_observed') is not True
            or not number(solution.get('gap_seconds')) or solution['gap_seconds']!=0
            or solution.get('coarse_origin_update') is not False
            or solution.get('native_error_reinitialized') is not False):return None
    quality=solution.get('native_quality')
    stamp,lon,lat,sigma=(solution.get(k) for k in ('utc_ms','longitude','latitude','horizontal_sigma_m'))
    delta_cov=solution.get('delta_covariance_enu_m2')
    if isinstance(delta_cov,list) and len(delta_cov)==9 and all(number(v) for v in delta_cov):
        delta_cov=[delta_cov[i:i+3] for i in (0,3,6)]
    local=solution.get('local_enu_m')
    if (not isinstance(local,list) or len(local)!=3 or not all(number(v) for v in local)
            or not isinstance(delta_cov,list) or len(delta_cov)!=3
            or any(not isinstance(row,list) or len(row)!=3 or not all(number(v) for v in row) for row in delta_cov)
            or any(delta_cov[i][i]<0 for i in range(3))
            or not number(solution.get('delta_dt_s')) or not 0<solution['delta_dt_s']<=2.):return None
    for i in range(3):
        for j in range(i):
            if (abs(delta_cov[i][j]-delta_cov[j][i])>1e-6
                    or delta_cov[i][i]*delta_cov[j][j]-delta_cov[i][j]**2 < -1e-9):return None
    a,b,c=delta_cov
    if a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]) < -1e-9:return None
    relative_sigma=math.sqrt(delta_cov[0][0]+delta_cov[1][1])
    if (type(quality) is not int or quality not in (0,1,2,4,5)
            or not all(number(v) for v in (stamp,lon,lat,sigma,received))
            or stamp<=0 or abs(lon)>180 or abs(lat)>=85 or not 0<sigma<=100000.
            or not 0<relative_sigma<=2.
            or not 0<=received-stamp/1000<=10.):return None
    if (not isinstance(solution.get('continuity_id'),str) or not solution['continuity_id']
            or type(solution.get('origin_revision')) is not int or solution['origin_revision']<0):return None
    return dict(schema=1,source='phone_continuity',timestamp_source='phone_gnss_utc',
        stamp=stamp/1000,received_at=received,longitude=lon,latitude=lat,
        coordinate_system='WGS84',native_fix_quality={1:4,2:5}.get(quality),
        horizontal_std_m=relative_sigma,covariance_source='phone_raw_motion_step',
        absolute_horizontal_std_m=sigma,relative_position_m=list(local),
        absolute_position_observed=solution['position_observed'],
        altitude_ellipsoid=solution.get('altitude_ellipsoid_m'),
        source_id=solution.get('source_id'),session_id=solution.get('session_id'),
        continuity_id=solution['continuity_id'],origin_revision=solution['origin_revision'],
        absolute_accuracy_verified=False)


def scalar_observation(solution,received):
    """Raw scalar constraints need no already-integrated 3D phone trajectory."""
    from .phone_heading import constraints
    if (not isinstance(solution,dict) or solution.get('schema_version')!=1
            or solution.get('source')!='phone_continuity' or solution.get('fresh') is not True
            or solution.get('absolute_accuracy_verified') is not False or 'quality' in solution
            or solution.get('coordinate_frame')!='phone_gnss_enu'):return None
    if 'raw_scalar_observed' in solution:
        # A partial set can constrain one LiVO heading without observing an
        # independent 3D phone displacement. Its numerical rank is checked by
        # the heading fit, not by relabelling the integrated position measured.
        if (solution['raw_scalar_observed'] is not True
                or solution.get('status') not in ('observed','predicted')):return None
    elif solution.get('status')!='observed' or solution.get('raw_motion_observed') is not True:
        return None
    raw=solution.get('raw_constraints')
    if constraints(raw) is None:return None
    quality=solution.get('native_quality')
    stamp,lon,lat,sigma=(solution.get(k) for k in ('utc_ms','longitude','latitude','horizontal_sigma_m'))
    if (type(quality) is not int or quality not in (0,1,2,4,5)
            or not all(number(v) for v in (stamp,lon,lat,sigma,received))
            or stamp<=0 or abs(lon)>180 or abs(lat)>=85 or not 0<sigma<=100000
            or not 0<=received-stamp/1000<=10. or abs(raw['end_utc_ms']-stamp)>1.
            or not isinstance(solution.get('continuity_id'),str) or not solution['continuity_id']
            or type(solution.get('origin_revision')) is not int or solution['origin_revision']<0
            or any(type(solution.get(k)) is not bool for k in (
                'position_observed','coarse_origin_update','native_error_reinitialized'))):return None
    return dict(schema=1,source='phone_continuity',timestamp_source='phone_gnss_utc',
        stamp=stamp/1000,received_at=received,longitude=lon,latitude=lat,
        coordinate_system='WGS84',native_fix_quality={1:4,2:5}.get(quality),
        horizontal_std_m=None,covariance_source='raw_scalar_heading_only',
        absolute_horizontal_std_m=sigma,raw_constraints=raw,
        absolute_position_observed=solution['position_observed'] and not solution['coarse_origin_update']
            and not solution['native_error_reinitialized'],
        altitude_ellipsoid=solution.get('altitude_ellipsoid_m'),
        source_id=solution.get('source_id'),session_id=solution.get('session_id'),
        continuity_id=solution['continuity_id'],origin_revision=solution['origin_revision'],
        absolute_accuracy_verified=False)


def alignment_quality(data):
    # A native label controls the selected observation subset; it is never
    # copied onto the fused estimate as a measured RTK quality.
    return data.get('native_fix_quality') if data.get('source')=='phone_continuity' else data.get('fix_quality')


class PhoneFallback:
    """Prefer usable dedicated RTK; never interleave two antennas in one fit."""
    def __init__(self):
        self.receiver_stamp = None
        self.phone_stamp = None
        self.phone_identity = None
        self.motion_stamp = None
        self.motion_identity = None

    def receiver(self, data, now):
        from nav_rtk.parser.observation_time import time_state
        lon, lat, sigma = (data.get(k) for k in ('longitude','latitude','horizontal_std_m'))
        usable = (data.get('fix_quality') in (4, 5) and time_state(data, now) in ('ready', 'waiting')
            and data.get('coordinate_system', 'WGS84') == 'WGS84'
            and all(number(v) for v in (lon,lat,sigma))
            and abs(lon) <= 180 and abs(lat) < 85 and 0 < sigma <= 2.)
        if usable:
            self.receiver_stamp = data['stamp']
            return True
        stamps=[stamp for stamp in (self.phone_stamp,self.motion_stamp) if stamp is not None]
        return not stamps or now-max(stamps)>10.

    def phone(self, solution, now):
        data = observation(solution, now)
        if data is None:
            return None
        from nav_rtk.parser.observation_time import MAX_EARLY_S, MAX_AGE_S
        if self.receiver_stamp is not None and -MAX_EARLY_S <= now-self.receiver_stamp <= MAX_AGE_S:
            return None
        identity=(data.get('source_id'),data.get('session_id'))
        if self.phone_identity==identity and self.phone_stamp is not None and data['stamp'] <= self.phone_stamp:
            return None
        self.phone_identity=identity
        self.phone_stamp = data['stamp']
        return data

    def motion(self, solution, now):
        data=motion_observation(solution,now)
        if data is None:return None
        from nav_rtk.parser.observation_time import MAX_EARLY_S, MAX_AGE_S
        if self.receiver_stamp is not None and -MAX_EARLY_S<=now-self.receiver_stamp<=MAX_AGE_S:return None
        identity=(data.get('source_id'),data.get('session_id'),data['continuity_id'],data['origin_revision'])
        if identity==self.motion_identity and self.motion_stamp is not None and data['stamp']<=self.motion_stamp:return None
        self.motion_identity=identity;self.motion_stamp=data['stamp']
        return data
