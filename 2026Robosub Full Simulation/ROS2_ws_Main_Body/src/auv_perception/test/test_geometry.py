from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from auv_perception.core import RigidPrimitiveSolver,LandmarkStore,StateBuffer,project_membership,transform

TEMPLATE=yaml.safe_load((Path(__file__).parents[1]/'config/templates/task1_gate.yaml').read_text())


def samples(part,n,rng):
    if part['shape']=='box':
        p=rng.uniform(-.5,.5,(n,3))*part['size'];p[:,2]=part['size'][2]/2
    else:
        p=np.zeros((n,3));p[:,0]=rng.uniform(-part['length']/2,part['length']/2,n)
        a=rng.uniform(0,np.pi*2,n);p[:,1]=part['radius']*np.cos(a);p[:,2]=part['radius']*np.sin(a)
    return p+part['center']


def test_noisy_semantic_registration_and_partial():
    rng=np.random.default_rng(32);T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',[.08,-.15,.04]).as_matrix();T[:3,3]=[.2,-.6,4]
    obs=[]
    for label,part in TEMPLATE['parts'].items():
        p=transform(T,samples(part,180,rng))+rng.normal(0,.007,(180,3))
        p=np.r_[p,rng.uniform([-3,-2,7],[3,2,10],(30,3))]
        obs.append({'label':label,'points':p,'confidence':.9})
    solver=RigidPrimitiveSolver(TEMPLATE,{'points_per_part':220})
    result=solver.fit(obs);assert result and result['accepted'],result
    assert np.linalg.norm(result['T'][:3,3]-T[:3,3])<.08
    assert np.linalg.norm(Rotation.from_matrix(T[:3,:3].T@result['T'][:3,:3]).as_rotvec())<.08
    partial=[o for o in obs if o['label'] in ['RightPillar','HorizontalBar']]
    result2=solver.fit(partial,prior=result['T']);assert result2 and result2['rms_m']<.07
    # A single feature must not initialize a confirmed map object.
    assert solver.fit([obs[0]]) is None


def test_projection_z_buffer():
    p=np.array([[0,0,2],[0,0,4],[.1,0,2],[np.nan,0,2]])
    mask=np.zeros((10,10),bool);mask[5,5]=True
    P=[10,0,5,0,0,10,5,0,0,0,1,0]
    out=project_membership(p,[mask],P,10,10)[0]
    assert np.all(out[:,2]==2)


def test_map_freezes_and_rejects_jump():
    s=LandmarkStore();T=np.eye(4);q={'accepted':True}
    assert not s.update('gate',T,q,1);assert not s.update('gate',T,q,2);assert s.update('gate',T,q,3)
    old=s.records['gate'].copy();bad=T.copy();bad[0,3]=5
    assert not s.update('gate',bad,q,4)
    assert not s.update('gate',T,{'accepted':False},5)
    assert s.records['gate']==old


def test_timestamp_interpolation():
    s=StateBuffer();A=np.eye(4);B=np.eye(4);B[0,3]=1
    s.add(1,A);s.add(1.1,B)
    assert abs(s.at(1.05) [0,3]-.5)<1e-6
    assert s.at(3) is None


def test_whole_mask_is_a_union_surface_constraint():
    rng=np.random.default_rng(4);T=np.eye(4);T[2,3]=4
    obs=[{'label':label,'points':transform(T,samples(part,100,rng)), 'confidence':.9}
         for label,part in TEMPLATE['parts'].items()]
    whole=np.concatenate([o['points'][::5] for o in obs])
    obs.append({'label':'__whole__','points':whole,'confidence':.8})
    result=RigidPrimitiveSolver(TEMPLATE).fit(obs)
    assert result and result['score']>.99 and result['rms_m']<.001
    assert np.linalg.norm(result['T'][:3,3]-T[:3,3])<.03
    # Exact face-interior samples have no measured boundary support in some
    # directions. A tiny residual must not override the observability rejection.
    assert not result['accepted']
