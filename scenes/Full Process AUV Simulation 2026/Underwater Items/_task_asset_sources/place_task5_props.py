"""Populate saved pool with four independent referenced Task5 props only."""
from pathlib import Path
import datetime,hashlib,json,os,shutil
from pxr import Usd,UsdGeom,UsdPhysics,Gf,Sdf

ROOT=Path('/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026')
FILE=ROOT/'Pool Enviroment/pool.usd'
NAMES=('repair_bolt','repair_plug','rescue_pill','rescue_bandage')
PATHS={'/World/'+n for n in NAMES}
s=Usd.Stage.Open(str(FILE)); assert s and UsdGeom.GetStageUpAxis(s)=='Z'
assert UsdGeom.GetStageMetersPerUnit(s)==1
table=s.GetPrimAtPath('/World/restore_table');assert table,'Save current table layout before running'
top=s.GetPrimAtPath('/World/restore_table/Tabletop');assert top
xc=UsdGeom.XformCache();tm=xc.GetLocalToWorldTransform(table)
normal=tm.TransformDir(Gf.Vec3d(0,0,1));normal.Normalize();assert normal[2]>.999,'Table must be horizontal'
assert all(abs(tm.TransformDir(Gf.Vec3d(*v)).GetLength()-1)<1e-5 for v in ((1,0,0),(0,1,0),(0,0,1)))
gravity=s.GetPrimAtPath('/World/PhysicsScene');assert gravity
assert abs(UsdPhysics.Scene(gravity).GetGravityMagnitudeAttr().Get())<1e-6,'This profile preserves existing zero gravity'

def snapshot(stage):
 data={}
 for p in stage.Traverse():
  if any(str(p.GetPath())==x or str(p.GetPath()).startswith(x+'/') for x in PATHS):continue
  data[str(p.GetPath())]={'type':p.GetTypeName(),'meta':str(p.GetAllMetadata()),
   'attrs':[(a.GetName(),str(a.Get()),[(float(t),str(a.Get(t))) for t in a.GetTimeSamples()]) for a in p.GetAttributes()],
   'rels':[(r.GetName(),list(map(str,r.GetTargets()))) for r in p.GetRelationships()]}
 return data

before=snapshot(s);digest=hashlib.sha256(FILE.read_bytes()).hexdigest()
stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S');backup=ROOT/'Underwater Items/_task_asset_sources/backups'/('pool_props_'+stamp)
backup.mkdir(parents=True);shutil.copy2(FILE,backup/'pool.usd')
top_world=xc.GetLocalToWorldTransform(top).Transform(Gf.Vec3d(0,0,.5))
orientation=Gf.Transform(tm).GetRotation().GetQuat()
entries=[]
for name,(x,y) in zip(NAMES,((-.14,-.14),(.14,-.14),(-.14,.14),(.14,.14))):
 path='/World/'+name;prim=s.GetPrimAtPath(path);created=not bool(prim)
 if created:
  prim=UsdGeom.Xform.Define(s,path).GetPrim()
  prim.GetReferences().AddReference('../Underwater Items/Task5_Objects/'+name+'.usd')
 assert prim.HasAPI(UsdPhysics.RigidBodyAPI),path
 target=tm.Transform(Gf.Vec3d(x,y,0));target[2]=top_world[2]+.20
 xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();xf.SetResetXformStack(False)
 xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(target)
 xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble,opSuffix='task5Placement').Set(orientation)
 # Set no scale op: original assets already have their real physical dimensions.
 rb=UsdPhysics.RigidBodyAPI.Apply(prim);rb.CreateRigidBodyEnabledAttr(True);rb.CreateKinematicEnabledAttr(False)
 rb.CreateVelocityAttr(Gf.Vec3f(0));rb.CreateAngularVelocityAttr(Gf.Vec3f(0))
 UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(.2)
 # Author native installed PhysX schema directly. Offline stock USD lacks its registry;
 # Isaac Kit recognizes these API names and standard schema attributes on reload.
 prim.AddAppliedSchema('PhysxForceAPI');prim.AddAppliedSchema('PhysxRigidBodyAPI')
 for attr,typ,val in [('physxForce:forceEnabled',Sdf.ValueTypeNames.Bool,True),
  ('physxForce:worldFrameEnabled',Sdf.ValueTypeNames.Bool,True),
  ('physxForce:mode',Sdf.ValueTypeNames.Token,'force'),
  ('physxForce:force',Sdf.ValueTypeNames.Vector3f,Gf.Vec3f(0,0,-1.962)),
  ('physxForce:torque',Sdf.ValueTypeNames.Vector3f,Gf.Vec3f(0)),
  ('physxRigidBody:disableGravity',Sdf.ValueTypeNames.Bool,True),
  ('physxRigidBody:enableCCD',Sdf.ValueTypeNames.Bool,True)]:
  prim.CreateAttribute(attr,typ,custom=False).Set(val)
 for q in Usd.PrimRange(prim):
  if q.HasAPI(UsdPhysics.CollisionAPI):
   UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(True)
   q.AddAppliedSchema('PhysxCollisionAPI')
   q.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float,custom=False).Set(.002)
   q.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float,custom=False).Set(0.0)
 entries.append({'name':name,'created':created,'world_bottom':list(target),'clearance_m':.2,'mass_kg':.2,'down_force_N':1.962})

assert snapshot(s)==before,'Unrelated composed scene state changed'
assert hashlib.sha256(FILE.read_bytes()).hexdigest()==digest,'Scene changed concurrently; aborting'
tmp=FILE.with_name('pool.task5_props_pending.usd');s.GetRootLayer().Export(str(tmp));os.replace(tmp,FILE)
check=Usd.Stage.Open(str(FILE));xc=UsdGeom.XformCache()
for e in entries:
 p=check.GetPrimAtPath('/World/'+e['name']);assert p
 assert abs(UsdPhysics.MassAPI(p).GetMassAttr().Get()-.2)<1e-6
 assert not UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Get()
 assert abs(xc.GetLocalToWorldTransform(p).ExtractTranslation()[2]-top_world[2]-.2)<1e-6
 assert sum(q.HasAPI(UsdPhysics.RigidBodyAPI) for q in Usd.PrimRange(p))==1
assert snapshot(check)==before
assert UsdPhysics.Scene(check.GetPrimAtPath('/World/PhysicsScene')).GetGravityMagnitudeAttr().Get()==0
report={'scene':str(FILE),'backup':str(backup),'tabletop_world_z':top_world[2], 'props':entries,
 'unchanged_other_prims':len(before),'gravity':'Global unchanged at 0; four props only get world -Z force 1.962N each at mass 0.2kg, equivalent to g=9.81m/s^2. Built-in gravity disabled on these props to avoid future double-counting.',
 'validation':'Scene reopened, properties and placements verified; no live scene playback triggered.'}
(backup/'placement_report.json').write_text(json.dumps(report,indent=2))
(ROOT/'Underwater Items/task5_pool_placement.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
