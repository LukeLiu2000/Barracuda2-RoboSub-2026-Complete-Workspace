from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'hide_ui':True,'disable_viewport_updates':True,'multi_gpu':False})
from pathlib import Path
import json,traceback,math
try:
 from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
 import omni.usd
 from isaacsim.core.api import World
 omni.usd.get_context().new_stage();w=World(physics_dt=1/120,rendering_dt=1/60,stage_units_in_meters=1);w.get_physics_context().set_gravity(0)
 s=omni.usd.get_context().get_stage();UsdGeom.SetStageUpAxis(s,UsdGeom.Tokens.z)
 f='/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/robot_barracuda2/Barracuda 2.gripper_candidate.usd'
 vehicle=UsdGeom.Xform.Define(s,'/World/Vehicle');vehicle.GetPrim().GetReferences().AddReference(f)
 vehicle.ClearXformOpOrder();placement=Gf.Matrix4d().SetScale(.001)*Gf.Matrix4d().SetRotate(Gf.Matrix3d(0,1,0,0,0,1,1,0,0).ExtractRotation())*Gf.Matrix4d().SetTranslate(Gf.Vec3d(0,0,1));vehicle.AddTransformOp(opSuffix='testPlacement').Set(placement)
 body='/World/Vehicle/barracuda2/base_link';grip='/World/Vehicle/barracuda2/front_gripper'
 zed=s.GetPrimAtPath(body+'/zed_link')
 if zed:zed.SetActive(False) # No SDK sender/ports in the isolated test.
 joints=[p for p in Usd.PrimRange(s.GetPrimAtPath(grip)) if p.IsA(UsdPhysics.RevoluteJoint)]
 assert len(joints)==6

 UsdPhysics.RigidBodyAPI(s.GetPrimAtPath(body)).GetKinematicEnabledAttr().Set(True)
 xc=UsdGeom.XformCache();gm=xc.GetLocalToWorldTransform(s.GetPrimAtPath(grip+'/robotiq_arg2f_base_link'))
 centre=gm.Transform(Gf.Vec3d(0,0,.195));obj=UsdGeom.Cube.Define(s,'/World/GraspObject');obj.CreateSizeAttr(1);obj.AddTranslateOp().Set(centre);obj.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Transform(gm).GetRotation().GetQuat());obj.AddScaleOp().Set(Gf.Vec3f(.04,.07,.04))
 UsdPhysics.RigidBodyAPI.Apply(obj.GetPrim());UsdPhysics.MassAPI.Apply(obj.GetPrim()).CreateMassAttr(.2);UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
 w.reset()
 for _ in range(60):w.step(render=False)
 for p in joints:UsdPhysics.DriveAPI(p,'angular').GetTargetPositionAttr().Set(p.GetAttribute('gripper:closingSign').Get()*40)
 for _ in range(360):w.step(render=False)
 before=UsdGeom.XformCache().GetLocalToWorldTransform(obj.GetPrim()).ExtractTranslation()
 force=PhysxSchema.PhysxForceAPI.Apply(obj.GetPrim());force.CreateWorldFrameEnabledAttr(True);force.CreateModeAttr('force');force.CreateForceAttr(Gf.Vec3f(0,0,-1.962))
 for _ in range(360):w.step(render=False)
 held=UsdGeom.XformCache().GetLocalToWorldTransform(obj.GetPrim()).ExtractTranslation();slip=(held-before).GetLength()
 assert slip<.06,('object slipped',slip,list(before),list(held))
 for p in joints:UsdPhysics.DriveAPI(p,'angular').GetTargetPositionAttr().Set(0)
 for _ in range(180):w.step(render=False)
 released=UsdGeom.XformCache().GetLocalToWorldTransform(obj.GetPrim()).ExtractTranslation()
 assert held[2]-released[2]>.2,('release failed',list(held),list(released))
 result={'passed':True,'test_object_mass_kg':.2,'test_object_dimensions_m':[.04,.07,.04],'down_force_N':1.962,'hold_seconds':3,'slip_m':slip,'release_drop_m':held[2]-released[2]}
except Exception:result={'passed':False,'traceback':traceback.format_exc()}
Path('/tmp/barracuda_grasp_test.json').write_text(json.dumps(result,indent=2));print('GRASP_TEST',json.dumps(result),flush=True);app.close()
