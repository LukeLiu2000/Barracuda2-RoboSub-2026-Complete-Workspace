"""Pure numpy/scipy geometry. No ROS, SDK, scene poses or simulator truth input."""
import itertools
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


def transform(T, p):
    return np.asarray(p) @ T[:3, :3].T + T[:3, 3]


def matrix(x):
    T = np.eye(4); T[:3,:3] = Rotation.from_rotvec(x[3:]).as_matrix(); T[:3,3] = x[:3]
    return T


def parameters(T):
    return np.r_[T[:3,3], Rotation.from_matrix(T[:3,:3]).as_rotvec()]


def surface_distance(q, part):
    v = q - part['center']
    if part['shape'] == 'box':
        d = np.abs(v) - np.asarray(part['size']) / 2
        # Absolute signed distance to finite box SURFACE (inside != zero).
        return np.abs(np.linalg.norm(np.maximum(d,0),axis=1) + np.minimum(np.max(d,axis=1),0))
    if part['shape'] == 'cylinder':
        a = int(part.get('axis',0)); radial = np.linalg.norm(np.delete(v,a,axis=1),axis=1)
        d = np.c_[radial-part['radius'], np.abs(v[:,a])-part['length']/2]
        return np.abs(np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(d.max(axis=1),0))
    raise ValueError('Unsupported primitive: '+part['shape'])


def clean_points(points, limit=450, seed=0):
    p=np.asarray(points,float); p=p[np.isfinite(p).all(axis=1)]
    p=p[(p[:,2]>.15)&(p[:,2]<25)]
    if len(p)<8:return p
    # Conservative local-density trimming; do not assume nearest/largest cluster is the target.
    d=cKDTree(p).query(p,k=min(6,len(p)))[0][:,-1]
    med=np.median(d); mad=1.4826*np.median(np.abs(d-med))
    p=p[d <= max(.08,med+4*max(.005,mad))]
    if len(p)>limit:p=p[np.random.default_rng(seed).choice(len(p),limit,replace=False)]
    return p


def principal_axis(p):
    # RANSAC axis candidate with a sleeve-sized tube, then PCA on consensus.
    rng=np.random.default_rng(9); best=np.ones(len(p),bool); score=0
    for _ in range(70):
        a,b=p[rng.choice(len(p),2,replace=False)]; v=b-a; n=np.linalg.norm(v)
        if n<.08:continue
        v/=n; d=np.linalg.norm(np.cross(p-a,v),axis=1); good=d<.10
        if good.sum()>score:score=good.sum();best=good
    q=p[best]; _,_,V=np.linalg.svd(q-np.median(q,axis=0),full_matrices=False)
    return V[0]


class RigidPrimitiveSolver:
    def __init__(self, template, options=None):
        self.template=template; self.parts=template['parts']; self.opt=options or {}
        for part in self.parts.values():surface_distance(np.zeros((1,3)),part)

    def fit(self, observations, prior=None):
        obs=[]
        for o in observations:
            if o['label'] not in self.parts and o['label']!='__whole__':continue
            p=clean_points(o['points'],self.opt.get('points_per_part',300))
            if len(p)>=12:obs.append(dict(o,points=p))
        if not obs:return None
        seeds=[o for o in obs if o['label'] in self.parts]
        labels={o['label'] for o in seeds}
        if not seeds:return None
        # Cold start needs two nonparallel structures or three separated parts.
        if prior is None and not ('HorizontalBar' in labels and bool(labels & {'LeftPillar','RightPillar','MiddleRedPillar'})) and len(labels)<3:
            return None
        total=np.concatenate([o['points'] for o in obs]); sigma=max(.015,.0015*np.median(total[:,2])**2)
        counts={k:sum(o['label']==k for o in obs) for k in labels|{'__whole__'}}
        weights=[np.sqrt(max(.05,o['confidence'])*(.25 if o['label']=='__whole__' else 1)/counts[o['label']]/len(o['points'])) for o in obs]
        def distances(x):
            T=matrix(x); R=T[:3,:3]; t=T[:3,3]
            out=[]
            for o in obs:
                q=(o['points']-t)@R
                out.append(surface_distance(q,self.parts[o['label']]) if o['label'] in self.parts else
                           np.min([surface_distance(q,p) for p in self.parts.values()],axis=0))
            return out
        def residual(x):
            # Apply robustification BEFORE part weighting, preserving a common metric cutoff.
            out=[]
            for d,w in zip(distances(x),weights):
                a=d/sigma; hub=np.where(a<=2,a*a,4*a-4)
                out.append(w*np.sqrt(np.maximum(hub,0)))
            return np.concatenate(out)
        candidates=[]
        if prior is not None:candidates.append(prior)
        centers=np.array([self.parts[o['label']]['center'] for o in seeds]); targets=np.array([np.median(o['points'],axis=0) for o in seeds])
        if len(seeds)>=3 and np.linalg.matrix_rank(centers-centers.mean(0),tol=.03)>=2:
            U,_,V=np.linalg.svd((centers-centers.mean(0)).T@(targets-targets.mean(0)));R=V.T@U.T
            if np.linalg.det(R)<0:V[-1]*=-1;R=V.T@U.T
            T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.median(targets-centers@R.T,axis=0);candidates.append(T)
        horizontal=next((o for o in obs if o['label']=='HorizontalBar'),None)
        vertical=next((o for o in obs if o['label'] in {'LeftPillar','RightPillar','MiddleRedPillar'}),None)
        if horizontal is not None and vertical is not None:
            u=principal_axis(horizontal['points']);v=principal_axis(vertical['points']);v-=u*(u@v)
            if np.linalg.norm(v)>.2:
                v/=np.linalg.norm(v)
                for sx,sy in itertools.product([-1,1],repeat=2):
                    R=np.column_stack([sx*u,sy*v,np.cross(sx*u,sy*v)])
                    T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.median(targets-centers@R.T,axis=0);candidates.append(T)
        best=None
        for T in candidates:
            fit=least_squares(residual,parameters(T),method='trf',max_nfev=65,x_scale=[1,1,1,.4,.4,.4],diff_step=1e-4)
            ds=distances(fit.x); cut=max(.06,2.5*sigma)
            fractions=[float(np.mean(d<cut)) for d in ds]
            score=float(np.average(fractions,weights=[.25 if o['label']=='__whole__' else 1. for o in obs]))
            if best is None or score>best['score'] or (abs(score-best['score'])<.01 and fit.cost<best['cost']):
                # Data-only finite differences, scale angle columns by inverse gate length.
                J=fit.jac.copy(); J[:,3:]/=max(.5,float(self.opt.get('characteristic_length',1.5)))
                sv=np.linalg.svd(J,compute_uv=False); condition=float(sv[-1]/max(sv[0],1e-12)) if len(sv)==6 else 0
                rms=float(np.sqrt(np.mean(np.concatenate([d[d<cut] for d in ds])**2))) if score>0 else 1e9
                best={'T':matrix(fit.x),'score':score,'rms_m':rms,'condition_ratio':condition,
                      'labels':sorted(labels),'cost':float(fit.cost),'counts':[len(o['points']) for o in obs],
                      'part_inliers':fractions,'sigma_m':sigma}
        if best is None:return None
        best['accepted']=bool(best['score']>=self.opt.get('min_inlier_fraction',.65) and best['rms_m']<=self.opt.get('max_rms_m',.09)
                              and best['condition_ratio']>self.opt.get('min_condition_ratio',.001) and len(labels)>=2)
        return best


SOLVERS={'rigid_primitives':RigidPrimitiveSolver}


def project_membership(points, masks, P, width, height):
    p=np.asarray(points); good=np.isfinite(p).all(axis=1)&(p[:,2]>.15)
    p=p[good]; uv=np.rint(np.c_[P[0]*p[:,0]/p[:,2]+P[2],P[5]*p[:,1]/p[:,2]+P[6]]).astype(int)
    valid=(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
    p=p[valid];uv=uv[valid]
    # Keep nearest sample for each projected pixel (no background overwrite).
    order=np.argsort(p[:,2]); ids=uv[order,1]*width+uv[order,0]
    _,first=np.unique(ids,return_index=True);keep=order[first];p=p[keep];uv=uv[keep]
    return [p[np.asarray(mask,dtype=bool)[uv[:,1],uv[:,0]]] for mask in masks]


class StateBuffer:
    def __init__(self):self.samples=[]
    def add(self,t,T):
        if self.samples and t<self.samples[-1][0]-.1:self.samples=[]
        self.samples.append((t,T));self.samples=self.samples[-1200:]
    def at(self,t,tolerance=.10):
        if not self.samples:return None
        ts=np.array([s[0] for s in self.samples]);j=int(np.searchsorted(ts,t))
        if j==0 or j==len(ts):
            k=0 if j==0 else -1
            return self.samples[k][1].copy() if abs(ts[k]-t)<tolerance else None
        a,A=self.samples[j-1];b,B=self.samples[j]
        if b-a>2*tolerance:return None
        T=np.eye(4);w=(t-a)/(b-a);T[:3,3]=(1-w)*A[:3,3]+w*B[:3,3]
        if b>a:T[:3,:3]=Slerp([a,b],Rotation.from_matrix([A[:3,:3],B[:3,:3]]))([t]).as_matrix()[0]
        else:T[:3,:3]=B[:3,:3]
        return T


class LandmarkStore:
    def __init__(self):self.records={};self.pending={}
    def update(self,key,T,quality,t):
        if not quality.get('accepted',False):return False
        old=self.records.get(key)
        if old:
            jump=np.linalg.norm(T[:3,3]-np.array(old['T'])[:3,3]);ang=np.linalg.norm(Rotation.from_matrix(np.array(old['T'])[:3,:3].T@T[:3,:3]).as_rotvec())
            if jump>.45 or ang>.4:return False
        else:
            prev=self.pending.get(key)
            if prev is None or np.linalg.norm(T[:3,3]-prev[0][:3,3])>.15:
                self.pending[key]=(T.copy(),1);return False
            count=prev[1]+1;self.pending[key]=(T.copy(),count)
            if count<3:return False
        self.records[key]={'T':T.tolist(),'last_accepted':float(t),'quality':{k:v for k,v in quality.items() if k!='T'}}
        return True
