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
 w.reset();results=[]
 from isaacsim.core.prims import SingleArticulation
 art=SingleArticulation(prim_path=grip,name='test_gripper',reset_xform_properties=False);art.initialize();print('INERTIAS',art._articulation_view._physics_view.get_inertias(),flush=True)
 def record(phase):
  xc=UsdGeom.XformCache();bw=xc.GetLocalToWorldTransform(s.GetPrimAtPath(body));gw=xc.GetLocalToWorldTransform(s.GetPrimAtPath(grip+'/robotiq_arg2f_base_link'))
  rel=(gw*bw.GetInverse()).ExtractTranslation();expect=Gf.Vec3d(300,132.6457135307562,990.9456555967205)
  error=(rel-expect).GetLength()*.001
  left=xc.GetLocalToWorldTransform(s.GetPrimAtPath(grip+'/left_inner_finger_pad')).ExtractTranslation()
  right=xc.GetLocalToWorldTransform(s.GetPrimAtPath(grip+'/right_inner_finger_pad')).ExtractTranslation()
  l=gw.GetInverse().Transform(left);r=gw.GetInverse().Transform(right)
  item={'dof_names':art.dof_names,'dof_positions':art.get_joint_positions().tolist(),'phase':phase,'fixed_joint_error_m':error,'pad_centre_gap_m':abs(l[1]-r[1]),'body_world_origin':list(bw.ExtractTranslation())}
  assert error<.005,item
  results.append(item)
 for _ in range(180):w.step(render=False)
 record('open')
 for p in joints:UsdPhysics.DriveAPI(p,'angular').GetTargetPositionAttr().Set(p.GetAttribute('gripper:closingSign').Get()*40)
 for _ in range(960):w.step(render=False)
 record('closed')
 print('CLOSED_DIAGNOSTIC',json.dumps(results),flush=True)
 for p in joints:UsdPhysics.DriveAPI(p,'angular').GetTargetPositionAttr().Set(0)
 for _ in range(960):w.step(render=False)
 record('reopened');assert results[-1]['pad_centre_gap_m']>.13,results
 force=PhysxSchema.PhysxForceAPI.Apply(s.GetPrimAtPath(body));force.CreateWorldFrameEnabledAttr(True);force.CreateModeAttr('force');force.CreateForceAttr(Gf.Vec3f(5,0,0))
 for _ in range(960):w.step(render=False)
 record('vehicle_moving')
 delta=(Gf.Vec3d(*results[-1]['body_world_origin'])-Gf.Vec3d(*results[-2]['body_world_origin'])).GetLength()
 assert delta>.005,delta
 assert UsdPhysics.MassAPI(s.GetPrimAtPath(body)).GetMassAttr().Get()==40
 # Rod endpoint and direction in body frame are geometric requirements independent of motion.
 xc=UsdGeom.XformCache();tool=xc.GetLocalToWorldTransform(s.GetPrimAtPath(body+'/tool_mount'));tip=xc.GetLocalToWorldTransform(s.GetPrimAtPath(body+'/tool_mount/rod_tip')).ExtractTranslation();start=tool.ExtractTranslation()
 length=(tip-start).GetLength();assert abs(length-.3)<1e-5,length
 assert results[1]['pad_centre_gap_m']<.025,results
 result={'passed':True,'rod_length_m':length,'pitch_deg':15,'body_mass_kg':40,'motion_displacement_m':delta,'results':results}
except Exception:result={'passed':False,'traceback':traceback.format_exc()}
Path('/tmp/barracuda_gripper_test.json').write_text(json.dumps(result,indent=2));print('BARRACUDA_GRIPPER_TEST',json.dumps(result),flush=True);app.close()
