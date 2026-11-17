"""Polyline walking cues. The measured location is never snapped to the route."""
import bisect
import math


def look_ahead(points, position, progress, scale):
    xy=[((p[0]-position[0])*scale,(p[1]-position[1])*111195) for p in points]
    lengths=[0.]
    for a,b in zip(xy,xy[1:]):lengths.append(lengths[-1]+math.dist(a,b))

    def at(s):
        i=min(len(xy)-2,max(0,bisect.bisect_right(lengths,s)-1))
        rate=max(0.,min(1.,(s-lengths[i])/max(1e-9,lengths[i+1]-lengths[i])))
        return [xy[i][j]+rate*(xy[i+1][j]-xy[i][j]) for j in (0,1)]

    target=at(min(lengths[-1],progress+3.))
    result=dict(bearing_deg=round(math.degrees(math.atan2(*target))%360))
    for i in range(1,len(xy)-1):
        remaining=lengths[i]-progress
        if not 1. <= remaining <= 20.:continue
        before,after=at(max(0.,lengths[i]-3.)),at(min(lengths[-1],lengths[i]+3.))
        a=[xy[i][j]-before[j] for j in (0,1)]
        b=[after[j]-xy[i][j] for j in (0,1)]
        if min(math.hypot(*a),math.hypot(*b))<.5:continue
        # Clockwise compass turn: east then north is left (negative).
        turn=(math.degrees(math.atan2(*b)-math.atan2(*a))+180)%360-180
        if abs(turn)<35:continue
        direction='掉头' if abs(turn)>150 else ('右转' if turn>0 else '左转')
        result.update(instruction=f'前方 {remaining:.0f} 米{direction}',
                      maneuver=direction,turn_distance_m=round(remaining,1),route_turn_deg=round(turn))
        break
    return result
