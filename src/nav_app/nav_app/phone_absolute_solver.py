"""Shared sparse GNSS/LiVO absolute-window solver.

No native/system position prior enters the objective. Code biases are correlated
within each signal, carrier ambiguities within each uninterrupted arc.
"""
import time
import numpy as np
from scipy.sparse import coo_matrix, block_diag, hstack, vstack, csr_matrix, diags
from scipy.sparse.linalg import lsmr,splu
from .phone_absolute_geometry import frame, prepare

def rotation(y):
    c,s=np.cos(y),np.sin(y)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]]),np.array([[-s,-c,0],[c,-s,0],[0,0,0.]])


def phase_gauge_fixed(data):
    """Remove one ambiguity offset per connected clock/arc component.

    This fixes only the exact clock/ambiguity gauge; it supplies no position,
    heading or ambiguity-integer constraint.
    """
    b=data['B'];parent=list(range(b.shape[1]))
    def find(i):
        while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
        return i
    for row in data['keys']:
        if len(row)==2:
            a,c=(data['index'][k] for k in row);parent[find(a)]=find(c)
    drops={}
    for k,i in data['index'].items():
        if k[0]=='arc':drops.setdefault(find(i),i)
    dropped=set(drops.values());return b[:,[i for i in range(b.shape[1]) if i not in dropped]]


def solve(records,poses,model,seed_yaw=0.,exclude=None,edge_sigma=.05,phase=True,robust=True,code_bias_scale=3.,seed_enu=None,deadline_monotonic=None,lever_sigma_m=0.,compute_covariance=True,warm_start=None,linear_backend='direct'):
    started=time.monotonic()
    if not np.all(np.diff(poses[:,0])>0):raise ValueError('LiVO UTC must increase strictly')
    selected=[]
    for r in records:
        q=np.searchsorted(poses[:,0],r['utc_ms']/1000)
        if q==0 or q==len(poses):continue
        if poses[q,0]-poses[q-1,0]>.25:continue
        selected.append(r)
    rejected=len(records)-len(selected);records=selected
    if len(records)<20:raise ValueError('Insufficient LiVO poses with valid time brackets')
    seed=np.array(records[0]['x0']);E=frame(seed);times=np.array([r['utc_ms']/1000 for r in records]);nt=len(times)
    odom=np.column_stack([np.interp(times,poses[:,0],poses[:,i]) for i in (1,2,3)]);odom-=odom[0];delta=np.diff(odom,axis=0)
    R,_=rotation(seed_yaw);x=seed[None,:]+odom@R.T@E;yaw=seed_yaw
    if seed_enu is not None:x+=np.array(seed_enu)@E
    kinds=['code','phase'] if phase else ['code'];dd=[prepare(records,model,kind,exclude=exclude) for kind in kinds]
    B=block_diag([phase_gauge_fixed(d) if kind=='phase' and linear_backend=='direct' else d['B'] for kind,d in zip(kinds,dd)],format='csr');z=np.concatenate([d['z'] for d in dd]);rs=np.concatenate([d['rs'] for d in dd]);t=np.concatenate([d['t'] for d in dd]);sigma=np.concatenate([d['sigma'] for d in dd]);
    indices=np.array([np.argmin(abs(times-rr[0])) for d in dd for rr in d['rows']]);nr=len(z);basew=1/sigma;w=basew.copy()
    code_keys=[(r[2]['system'],r[2]['frequency_hz'],r[2]['code'],r[2]['id'],r[1].get('clock_segment',0)) for r in dd[0]['rows']]
    ci={k:i for i,k in enumerate(dict.fromkeys(code_keys))} if code_bias_scale else {}
    C=coo_matrix((np.ones(len(code_keys)),(np.arange(len(code_keys)),[ci[k] for k in code_keys])),shape=(nr,len(ci))).tocsr() if ci else csr_matrix((nr,0))
    code_bias=np.zeros(len(ci));lever=np.zeros((nt,3));lever_start=3*nt+1;lever_count=3*nt if lever_sigma_m>0 else 0;param=lever_start+lever_count;edge_sd=edge_sigma*np.sqrt(np.maximum(np.diff(times),.05))+.005*np.linalg.norm(delta,axis=1)
    if warm_start is not None:
        if not np.array_equal(times,np.asarray(warm_start['utc_seconds'])):raise ValueError('warm_start_time_mismatch')
        x=np.array(warm_start['trajectory_ecef'],dtype=float);yaw=np.radians(warm_start['yaw_deg'])
        if lever_count:lever=np.array(warm_start['phone_offset_enu_m'],dtype=float)
        code_bias=np.array([warm_start.get('code_bias_m',{}).get(str(k),0.) for k in ci])
    end_matrix=None;residual=None;direct_fallbacks=0
    for iteration in range(16 if code_bias_scale else 8):
        if deadline_monotonic is not None and time.monotonic()>deadline_monotonic:raise TimeoutError('absolute window computation deadline exceeded')
        g,jac=model(t,rs,x[indices]);res=z-g-C@code_bias
        # Project away large unknown clock/ambiguity constants before the full
        # least-squares solve; this changes no position information.
        BW=diags(w)@B
        if linear_backend=='direct':bias0=splu((BW.T@BW).tocsc()).solve(np.asarray(BW.T@(res*w)))
        else:bias0=lsmr(BW,res*w,atol=1e-10,btol=1e-10,maxiter=5000)[0]
        small=res*w-BW@bias0
        ii=np.repeat(np.arange(nr),3);jj=np.repeat(indices*3,3)+np.tile(np.arange(3),nr)
        H=coo_matrix(((jac*w[:,None]).ravel(),(ii,jj)),shape=(nr,param)).tocsr();HG=hstack([H,BW,diags(w)@C],format='csr')
        R,dR=rotation(yaw);expected=delta@R.T;actual=np.diff(x,axis=0)@E.T-np.diff(lever,axis=0);er=(expected-actual).ravel()/np.repeat(edge_sd,3)
        ei=[];ej=[];ev=[]
        for k in range(nt-1):
            for axis in range(3):
                row=3*k+axis
                for j in range(3):
                    ei.extend([row,row]);ej.extend([3*k+j,3*(k+1)+j]);ev.extend([-E[axis,j]/edge_sd[k],E[axis,j]/edge_sd[k]])
                ei.append(row);ej.append(3*nt);ev.append(-(dR@delta[k])[axis]/edge_sd[k])
                if lever_count:
                    ei.extend([row,row]);ej.extend([lever_start+3*k+axis,lever_start+3*(k+1)+axis]);ev.extend([1/edge_sd[k],-1/edge_sd[k]])
        HE=coo_matrix((ev,(ei,ej)),shape=(3*(nt-1),param+B.shape[1]+len(ci))).tocsr()
        mats=[HG,HE];rhs=[small,er]
        if lever_count:
            LP=coo_matrix((np.full(lever_count,1/lever_sigma_m),(np.arange(lever_count),lever_start+np.arange(lever_count))),shape=(lever_count,HG.shape[1])).tocsr()
            mats.append(LP);rhs.append(-lever.ravel()/lever_sigma_m)
        if ci:
            pw=np.sqrt(1/(code_bias_scale*np.sqrt(code_bias**2+1.)))
            HP=coo_matrix((pw,(np.arange(len(ci)),param+B.shape[1]+np.arange(len(ci)))),shape=(len(ci),HG.shape[1])).tocsr()
            mats.append(HP);rhs.append(-pw*code_bias)
        A=vstack(mats,format='csr');b=np.concatenate(rhs)
        norms=np.sqrt(np.asarray(A.power(2).sum(axis=0)).ravel());norms=np.maximum(norms,1e-8)
        As=A@diags(1/norms);factor=None
        if linear_backend=='direct':
            try:
                factor=splu((As.T@As).tocsc());pivots=np.abs(factor.U.diagonal())
                if np.min(pivots)<np.max(pivots)*1e-13:factor=None
            except RuntimeError:factor=None
        if factor is not None:sol=(factor.solve(np.asarray(As.T@b)),2,1)
        else:
            sol=lsmr(As,b,atol=1e-9,btol=1e-9,maxiter=8000)
            if linear_backend=='direct':direct_fallbacks+=1
        dx=sol[0]/norms
        x+=dx[:3*nt].reshape(-1,3);yaw+=dx[3*nt]
        if lever_count:lever+=dx[lever_start:param].reshape(-1,3)
        if ci:code_bias+=dx[param+B.shape[1]:]
        residual=(small-HG@dx)/w
        if robust:
            # Code NLOS and phase slips are separate errors; preserve declared
            # observation sigma and use Cauchy influence for both modalities.
            w=basew/np.sqrt(1+(residual/(3*sigma))**2)
        end_matrix=As
        if iteration>=3 and np.max(abs(dx[:param]))<1e-4:break
    covariance=[];final_covariance=[];marginal_supported=[];body_covariance=[];body_supported=[]
    # Marginal covariance via minimum-norm A^T u=e. No numeric prior is added to
    # the clock/ambiguity gauge. These remain model sigmas, not calibrated truth.
    for axis in range(3) if compute_covariance else ():
        if deadline_monotonic is not None and time.monotonic()>deadline_monotonic:raise TimeoutError('absolute window covariance deadline exceeded')
        target=np.zeros(A.shape[1]);target[:3]=E[axis,:]/norms[:3]
        if factor is not None:
            v=factor.solve(target);u=(As@v,)
        else:u=lsmr(As.T,target,atol=1e-8,btol=1e-8,maxiter=10000)
        covariance.append(float(np.linalg.norm(u[0])))
        target=np.zeros(A.shape[1]);target[3*(nt-1):3*nt]=E[axis,:]/norms[3*(nt-1):3*nt]
        if factor is not None:u=(As@factor.solve(target),)
        else:u=lsmr(As.T,target,atol=1e-8,btol=1e-8,maxiter=10000)
        final_covariance.append(float(np.linalg.norm(u[0])))
        marginal_supported.append(float(np.linalg.norm(As.T@u[0]-target))/max(1e-20,np.linalg.norm(target))<1e-3)
        # x is the phone antenna. The LiVO reference point is x-E.T*offset;
        # its marginal includes the x/offset cross covariance, not the sum of
        # two independent sigmas. A common unknown installation offset remains
        # unobservable and is accounted for separately by the public floor.
        if lever_count:target[lever_start+3*(nt-1)+axis]=-1/norms[lever_start+3*(nt-1)+axis]
        if factor is not None:u=(As@factor.solve(target),)
        else:u=lsmr(As.T,target,atol=1e-8,btol=1e-8,maxiter=10000)
        body_covariance.append(float(np.linalg.norm(u[0])))
        body_supported.append(float(np.linalg.norm(As.T@u[0]-target))/max(1e-20,np.linalg.norm(target))<1e-3)
    yaw_sigma=None;yaw_supported=None
    if compute_covariance:
        target=np.zeros(A.shape[1]);target[3*nt]=1/norms[3*nt]
        if factor is not None:u=(As@factor.solve(target),)
        else:u=lsmr(As.T,target,atol=1e-8,btol=1e-8,maxiter=10000)
        yaw_sigma=float(np.linalg.norm(u[0]));yaw_supported=float(np.linalg.norm(As.T@u[0]-target))/max(1e-20,np.linalg.norm(target))<1e-3
    body=x-lever@E
    return {'start_utc_ms':times[0]*1000,'end_utc_ms':times[-1]*1000,'epochs':nt,'rows':nr,'rejected_pose_brackets':rejected,'initial_ecef':x[0].tolist(),'final_ecef':x[-1].tolist(),'final_body_ecef':body[-1].tolist(),'body_trajectory_ecef':body.tolist(),'body_model_sigma_enu_m':body_covariance,'body_position_rank':int(sum(body_supported)) if compute_covariance else None,'yaw_deg':float(np.degrees(yaw)%360),'origin_model_sigma_enu_m':covariance,'final_model_sigma_enu_m':final_covariance,'position_rank':int(sum(marginal_supported)) if compute_covariance else None,'yaw_rank':int(yaw_supported) if compute_covariance else None,'yaw_model_sigma_rad':yaw_sigma,'lever_sigma_m':lever_sigma_m,'lever_estimate_rms_m':float(np.sqrt(np.mean(lever**2))),'phone_offset_enu_m':lever.tolist(),'code_bias_scale_m':code_bias_scale,'code_bias_m':{str(k):float(code_bias[i]) for k,i in ci.items()},'iterations':iteration+1,'lsmr_stop':int(sol[1]),'lsmr_iterations':int(sol[2]),'linear_backend':linear_backend,'direct_fallbacks':direct_fallbacks,'matrix_rows':A.shape[0],'matrix_columns':A.shape[1],'matrix_nonzeros':A.nnz,'phase_gauge_columns_removed':sum(d['B'].shape[1] for d in dd)-B.shape[1],'gnss_residual_rms_m':float(np.sqrt(np.mean(residual**2))),'edge_residual_rms_m':float(np.sqrt(np.mean((np.diff(x,axis=0)@E.T-np.diff(lever,axis=0)-delta@rotation(yaw)[0].T)**2))),'trajectory_ecef':x.tolist(),'utc_seconds':times.tolist(),'wall_seconds':time.monotonic()-started}
