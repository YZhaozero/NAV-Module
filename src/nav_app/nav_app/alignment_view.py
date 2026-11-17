"""Bounded presentation of independent GNSS/LIO evidence; previews never navigate."""
import math


def preview_fit(pairs, weights=None):
    if len(pairs) < 3:
        return None
    n=len(pairs)
    weights=list(weights) if weights is not None else [1.]*n
    if len(weights)!=n or any(not math.isfinite(w) or w<=0 for w in weights):return None
    mass=sum(weights)
    a=[sum(w*p[i] for w,(p,q) in zip(weights,pairs))/mass for i in (0,1)]
    b=[sum(w*q[i] for w,(p,q) in zip(weights,pairs))/mass for i in (0,1)]
    x=[(p[0]-a[0],p[1]-a[1]) for p,q in pairs]
    y=[(q[0]-b[0],q[1]-b[1]) for p,q in pairs]
    spread=math.sqrt(sum(u*u+v*v for u,v in x)/n)
    if spread < .3:
        return None
    dot=sum(w*(u*s+v*t) for w,(u,v),(s,t) in zip(weights,x,y))
    cross=sum(w*(u*t-v*s) for w,(u,v),(s,t) in zip(weights,x,y))
    yaw=math.atan2(cross,dot);c,s=math.cos(yaw),math.sin(yaw)
    rms=math.sqrt(sum(w*((c*u-s*v-r)**2+(s*u+c*v-t)**2) for w,(u,v),(r,t) in zip(weights,x,y))/mass)
    return dict(yaw=yaw,tx=b[0]-c*a[0]+s*a[1],ty=b[1]-s*a[0]-c*a[1],
                residual_m=rms,spread_m=spread,observations=n,preview_only=True)


def _segments(rows, field):
    result=[]
    for row in rows:
        if (not result or row['stamp']-result[-1][-1]['stamp'] > 2.
                or row.get('source')!=result[-1][-1].get('source')
                or row.get('segment')!=result[-1][-1].get('segment')):
            result.append([])
        result[-1].append(row)
    return [[r[field] for r in group] for group in result]


def _velocity(rows, field, now):
    if len(rows)<2 or not -.05 <= now-rows[-1]['stamp'] <= 1.5:
        return None
    end=rows[-1]
    # Absolute origin updates are not walking velocity. Physical motion remains
    # visible through the independently measured LiVO velocity beside it.
    if end.get('source')=='phone_continuity':return None
    start=next((r for r in reversed(rows[:-1]) if .5 <= end['stamp']-r['stamp'] <= 2.), None)
    if start is None:
        return None
    dt=end['stamp']-start['stamp']
    v=[(end[field][i]-start[field][i])/dt for i in (0,1)]
    return dict(components=v,speed_m_s=math.hypot(*v),dt_s=dt,source='position_difference')


def alignment_view(state, trial):
    from .navigation_trial import transform
    now=state.clock()
    local=[dict(stamp=r['stamp'],point=list(r['position_m'][:2])) for r in state.local_track[-600:]
           if r['epoch']==state.odom_epoch and r.get('continuity',state.odom_continuity)==state.odom_continuity]
    if state.odom and (not local or state.odom[-1][0]>local[-1]['stamp']):
        stamp,p,_=state.odom[-1];local.append(dict(stamp=stamp,point=list(p[:2])))
    gnss=[]
    if state.origin:
        for r in list(state.track)[-600:]:
            if not trial.accepts_quality(r.get('native_quality',r.get('quality'))):continue
            p=state.origin.enu(*r['wgs84'],state.origin.values[2])
            gnss.append(dict(stamp=r['stamp'],point=list(p[:2]),quality=r.get('quality'),
                             source=r.get('source'),segment=r.get('segment'),selected=True))
        r=state.position
        if r and trial.accepts_quality(r.get('native_quality',r.get('quality'))) and (not gnss or r['stamp']>gnss[-1]['stamp']):
            p=state.origin.enu(*r['wgs84'],state.origin.values[2])
            gnss.append(dict(stamp=r['stamp'],point=list(p[:2]),quality=r.get('quality'),
                             source=r.get('source'),segment=r.get('segment'),selected=True))
    verified=bool(state.alignment and not state.alignment_conflict and
                  state.alignment.get('odom_continuity')==state.odom_continuity)
    a=state.alignment if verified else (trial.preview or trial.display_anchor)
    lio=[dict(r,point=transform(a,r['point'])) for r in local] if a else local
    velocity_lio=_velocity(lio,'point',now)
    velocity_gnss=_velocity(gnss,'point',now)
    projection=trial.projection
    def on_map(rows):
        if not projection:return []
        result=[]
        for r in rows:
            try:result.append(dict(r,point=projection.to_bd09(r['point'])))
            except ValueError:continue
        return result
    route=[]
    if projection and state.route:
        try:route=[projection.to_enu(p) for p in state.route['points']]
        except ValueError:pass
    return dict(gnss=gnss,lidar=lio,gnss_segments=_segments(gnss,'point'),lidar_segments=_segments(lio,'point'),
        gnss_map=on_map(gnss),lidar_map=on_map(lio) if a else [],route=route,
        aligned=verified,preview=bool(a and not verified),unoriented=bool(a and a.get('unoriented')),separate_axes=a is None,
        transform=a,quality_policy=trial.quality_policy,velocity_gnss=velocity_gnss,velocity_lidar=velocity_lio,
        velocity_frame='east_north' if a else 'separate_local_axes',
        gnss_fresh=bool(gnss and -.05<=now-gnss[-1]['stamp']<=1.5),
        lidar_fresh=bool(local and -.05<=now-local[-1]['stamp']<=.5))
