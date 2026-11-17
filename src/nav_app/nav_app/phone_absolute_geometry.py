"""ECEF range and signal-arc models for raw differential GNSS windows."""
from collections import defaultdict
import numpy as np
from scipy.sparse import coo_matrix


def frame(x):
    lon=np.arctan2(x[1],x[0]);lat=np.arctan2(x[2],np.hypot(x[0],x[1])*(1-6.69437999014e-3))
    for _ in range(10):
        n=6378137/np.sqrt(1-6.69437999014e-3*np.sin(lat)**2)
        lat=np.arctan2(x[2]+6.69437999014e-3*n*np.sin(lat),np.hypot(x[0],x[1]))
    s,c,sl,cl=np.sin(lat),np.cos(lat),np.sin(lon),np.cos(lon)
    return np.array([[-sl,cl,0],[-s*cl,-s*sl,c],[c*cl,c*sl,s]])


def geodetic(x):
    e=frame(x);lat=np.arcsin(e[2,2]);lon=np.arctan2(x[1],x[0]);n=6378137/np.sqrt(1-6.69437999014e-3*np.sin(lat)**2)
    return float(np.degrees(lat)),float(np.degrees(lon)),float(np.hypot(x[0],x[1])/np.cos(lat)-n)


def geometric_range(rs,x):
    d=x-rs;r=np.linalg.norm(d,axis=1);k=7.2921151467e-5/299792458.
    correction=k*(rs[:,0]*x[:,1]-rs[:,1]*x[:,0])
    grad=d/r[:,None]+k*np.column_stack([-rs[:,1],rs[:,0],np.zeros(len(rs))])
    return r+correction,grad


class AbsoluteModel:
    def __init__(self,records):
        self.models={};self._cached=None
        for r in records:
            for q in r['rows']:
                key=(round(r['utc_ms']/1000,6),tuple(q['satellite_ecef_m']))
                self.models[key]=(np.asarray(r['x0']),q['troposphere_m'],np.asarray(q['troposphere_gradient_ecef']))
    def __call__(self,t,rs,x):
        g,h=geometric_range(rs,x)
        if self._cached is None or not np.array_equal(t,self._cached[0]) or not np.array_equal(rs,self._cached[1]):
            entries=[self.models[(round(float(ti),6),tuple(ri))] for ti,ri in zip(t,rs)]
            self._cached=(t.copy(),rs.copy(),np.array([v[0] for v in entries]).reshape(-1,3),np.array([v[1] for v in entries]),np.array([v[2] for v in entries]).reshape(-1,3))
        _,_,ref,trop,grad=self._cached
        g+=trop+np.sum(grad*(x-ref),axis=1);h+=grad
        return g,h


def prepare(records,model,kind='phase',seconds=None,exclude=None,systems=None):
    t0=records[0]['utc_ms']/1000;prior={};arc=defaultdict(int);rows=[];keys=[]
    for epoch,r in enumerate(records):
        t=r['utc_ms']/1000
        if seconds is not None and t-t0>seconds:break
        for q in r['rows']:
            if q['id'] in (exclude or []) or systems and q['system'] not in systems:continue
            if not q[kind+'_valid']:continue
            key=(q['system'],q['frequency_hz'],q['code'],q['id'],r.get('clock_segment',0))
            if kind=='phase':
                if key not in prior or t-prior[key]>1.5 or q['lli_rover']&3 or q['lli_base']&3:arc[key]+=1
                prior[key]=t
            group=(q['system'],q['frequency_hz'],q['code'])
            keys.append((('clock',epoch,group),('arc',key,arc[key])) if kind=='phase' else (('clock',epoch,group),))
            rows.append((t,r,q))
    index={k:i for i,k in enumerate(dict.fromkeys(k for row in keys for k in row))};ri=[];ci=[]
    for i,ks in enumerate(keys):
        for k in ks:ri.append(i);ci.append(index[k])
    b=coo_matrix((np.ones(len(ri)),(ri,ci)),shape=(len(keys),len(index))).tocsr()
    t=np.array([v[0] for v in rows]);rs=np.array([v[2]['satellite_ecef_m'] for v in rows]).reshape(-1,3)
    z=np.array([v[2][kind+'_observation_m'] for v in rows]);sigma=np.array([max(.05 if kind=='phase' else 2.,v[2][kind+'_sigma_m']) for v in rows])
    return dict(t=t,rs=rs,z=z,sigma=sigma,B=b,rows=rows,index=index,keys=keys)
