"""Author a candidate robot; original robot is committed only after physics validation."""
from pathlib import Path
import shutil,math,json,datetime,hashlib
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf

B=Path('/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026')
D=B/'robot_barracuda2';A=D/'accessories/front_gripper';A.mkdir(parents=True,exist_ok=True)
V=A/'vendor/Robotiq_2F140';(V/'Props').mkdir(parents=True,exist_ok=True)
for rel in ('2f140_instanceable.usd','Props/instanceable_meshes.usd'):
 shutil.copy2(Path('/tmp/gripper_official_probe')/rel,V/rel)
(V/'SOURCE.txt').write_text('NVIDIA Isaac Sim 5.1 official asset. Source: https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/Robotiq/2F-140/ . Original downloaded files retained without changes. Local robot overrides provide mounting and drives.\n')

original=D/'Barracuda 2.usd';candidate=D/'Barracuda 2.gripper_candidate.usd'
stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S');backup=A/'backups'/stamp;backup.mkdir(parents=True)
shutil.copy2(original,backup/original.name);shutil.copy2(B/'Pool Enviroment/pool.usd',backup/'pool.usd')
source_hash=hashlib.sha256(original.read_bytes()).hexdigest()
s0=Usd.Stage.Open(str(original));s0.GetRootLayer().Export(str(candidate))
s=Usd.Stage.Open(str(candidate));parent='/World/barracuda2';body=parent+'/base_link';gp=parent+'/front_gripper'
assert not s.GetPrimAtPath(gp),'Gripper already present'
orig_attrs={a.GetName():str(a.Get()) for a in s.GetPrimAtPath(body).GetAttributes()}

# Original body coordinates are effectively mm in the composed metre pool stage.
mount_mm=Gf.Vec3d(300,55,701.16790771)
theta=math.radians(15);c=math.cos(theta);h=math.sin(theta)
rod_end_mm=mount_mm+Gf.Vec3d(0,300*h,300*c)
tool_R=Gf.Matrix3d(0,0,1, 1,0,0, 0,1,0)
grip_R=Gf.Matrix3d(0,-c,h, 1,0,0, 0,h,c)
grip_q=grip_R.ExtractRotation().GetQuat()

visual=A/'front_tool_visual.usd'
vs=Usd.Stage.CreateInMemory();vr=UsdGeom.Xform.Define(vs,'/FrontTool');vs.SetDefaultPrim(vr.GetPrim());UsdGeom.SetStageMetersPerUnit(vs,1);UsdGeom.SetStageUpAxis(vs,UsdGeom.Tokens.z)
def material(name,col):
 m=UsdShade.Material.Define(vs,'/FrontTool/Looks/'+name);sh=UsdShade.Shader.Define(vs,str(m.GetPath())+'/Surface');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*col));sh.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(.65);sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.3);sh.CreateOutput('surface',Sdf.ValueTypeNames.Token);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return m
metal=material('Metal',(.46,.50,.55));dark=material('Mount',(.08,.09,.11))
rod=UsdGeom.Cylinder.Define(vs,'/FrontTool/Rod');rod.CreateRadiusAttr(.0085);rod.CreateHeightAttr(.30);direction=Gf.Vec3d(c,0,h)
rod.AddTranslateOp().Set(.15*direction);rod.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Rotation(Gf.Vec3d(0,0,1),direction).GetQuat());UsdShade.MaterialBindingAPI.Apply(rod.GetPrim()).Bind(metal)
plate=UsdGeom.Cube.Define(vs,'/FrontTool/MountPlate');plate.CreateSizeAttr(1);plate.AddTranslateOp().Set(Gf.Vec3d(-.012,0,-.004));plate.AddScaleOp().Set(Gf.Vec3f(.035,.07,.008));UsdShade.MaterialBindingAPI.Apply(plate.GetPrim()).Bind(dark)
tip=UsdGeom.Xform.Define(vs,'/FrontTool/rod_tip');tip.AddTranslateOp().Set(.30*direction)
vs.GetRootLayer().Export(str(visual))

tool=UsdGeom.Xform.Define(s,body+'/tool_mount');tool.GetPrim().GetReferences().AddReference('accessories/front_gripper/front_tool_visual.usd')
tool.AddTranslateOp().Set(mount_mm);tool.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(tool_R.ExtractRotation().GetQuat());tool.AddScaleOp().Set(Gf.Vec3d(1000))
tool.GetPrim().CreateAttribute('tool:rodLengthMeters',Sdf.ValueTypeNames.Double).Set(.30)
tool.GetPrim().CreateAttribute('tool:rodPitchDegrees',Sdf.ValueTypeNames.Double).Set(15)

g=UsdGeom.Xform.Define(s,gp);g.GetPrim().GetReferences().AddReference('accessories/front_gripper/vendor/Robotiq_2F140/2f140_instanceable.usd')
g.ClearXformOpOrder();g.AddTranslateOp().Set(rod_end_mm);g.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(grip_q);g.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1000))
g.GetPrim().CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool,custom=False).Set(False)
g.GetPrim().CreateAttribute('physxArticulation:solverPositionIterationCount',Sdf.ValueTypeNames.Int,custom=False).Set(32)
g.GetPrim().CreateAttribute('physxArticulation:solverVelocityIterationCount',Sdf.ValueTypeNames.Int,custom=False).Set(4)

# The official fixture-to-world joint becomes a robot-body-to-gripper fixed joint.
j=UsdPhysics.FixedJoint(s.GetPrimAtPath(gp+'/root_joint'));j.CreateBody0Rel().SetTargets([body]);j.CreateBody1Rel().SetTargets([gp+'/robotiq_arg2f_base_link'])
j.CreateLocalPos0Attr(Gf.Vec3f(rod_end_mm));j.CreateLocalRot0Attr(Gf.Quatf(grip_q));j.CreateLocalPos1Attr(Gf.Vec3f(0));j.CreateLocalRot1Attr(Gf.Quatf(1));j.CreateJointEnabledAttr(True)
j.CreateExcludeFromArticulationAttr(True)
# Keep the original vehicle a free rigid body, connected to the independent
# gripper articulation through a fixed constraint. Put rod collision in SI
# coordinates on the gripper base, avoiding mixed-scale compounds on the vehicle.
rc=UsdGeom.Cylinder.Define(s,gp+'/robotiq_arg2f_base_link/rod_collider');rc.CreateRadiusAttr(.0085);rc.CreateHeightAttr(.30);rc.AddTranslateOp().Set(Gf.Vec3d(0,0,-.15));rc.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(rc.GetPrim())

signs={'finger_joint':1,'left_inner_knuckle_joint':-1,'right_inner_knuckle_joint':-1,'right_outer_knuckle_joint':-1,'left_inner_finger_joint':1,'right_inner_finger_joint':1}
for p in Usd.PrimRange(g.GetPrim()):
 if p.HasAPI(UsdPhysics.RigidBodyAPI):
  p.AddAppliedSchema('PhysxRigidBodyAPI');p.CreateAttribute('physxRigidBody:disableGravity',Sdf.ValueTypeNames.Bool,custom=False).Set(True)
  p.CreateAttribute('physxRigidBody:angularDamping',Sdf.ValueTypeNames.Float,custom=False).Set(.1);p.CreateAttribute('physxRigidBody:linearDamping',Sdf.ValueTypeNames.Float,custom=False).Set(.1)
  # In the metre pool profile the installed gripper links have world scale 1.
  # Retain the manufacturer's authored inertias; recomputing from instance geometry
  # can produce poor articulation conditioning when attached to the mm-authored body.
 if p.IsA(UsdPhysics.RevoluteJoint):
  drive=UsdPhysics.DriveAPI.Apply(p,'angular');drive.CreateStiffnessAttr(40);drive.CreateDampingAttr(2);drive.CreateMaxForceAttr(10);drive.CreateTargetPositionAttr(0);drive.CreateTargetVelocityAttr(0)
  p.CreateAttribute('gripper:closingSign',Sdf.ValueTypeNames.Int).Set(signs[p.GetName()])
g.GetPrim().CreateAttribute('gripper:openFraction',Sdf.ValueTypeNames.Double).Set(1.0)
g.GetPrim().CreateAttribute('gripper:nominalMaximumOpeningMm',Sdf.ValueTypeNames.Double).Set(140)

assert {a.GetName():str(a.Get()) for a in s.GetPrimAtPath(body).GetAttributes()}==orig_attrs
assert abs(UsdPhysics.MassAPI(s.GetPrimAtPath(body)).GetMassAttr().Get()-40)<1e-6
s.GetRootLayer().Save()
config={'candidate':str(candidate),'original':str(original),'source_sha256':source_hash,'backup':str(backup),'parent':parent,'body':body,'gripper':gp,
 'mount_body_mm':list(mount_mm),'rod_end_body_mm':list(rod_end_mm),'rod_length_m':.30,'rod_pitch_deg':15,'body_mesh_length_m':.69743799,
 'closing_signs':signs,'open_angle_deg':0,'closed_angle_deg':40,'official_open_inner_pad_gap_m':.1359784,
 'notes':'Official nominal 140mm asset; measured original pad spacing approx 136mm at zero-angle open. Original body mass40 and all original body attributes preserved; added gripper link masses add to total robot assembly mass. No ROS or torpedo-ray code modified. tool_mount Xforward/Yleft/Zup.'}
(A/'installation.json').write_text(json.dumps(config,indent=2));print(json.dumps(config,indent=2))
