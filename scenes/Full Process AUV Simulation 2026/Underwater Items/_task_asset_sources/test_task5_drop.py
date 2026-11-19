"""Isolated Isaac physics test; never opens/edits the user's live stage."""
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'hide_ui':True,'disable_viewport_updates':True,'multi_gpu':False})
import json,traceback
from pathlib import Path
try:
 from pxr import UsdGeom,UsdPhysics,Gf
 import omni.usd
 from isaacsim.core.api import World
 omni.usd.get_context().new_stage()
 w=World(physics_dt=1/120,rendering_dt=1/60,stage_units_in_meters=1)
 w.get_physics_context().set_gravity(0.0)
 s=omni.usd.get_context().get_stage();UsdGeom.SetStageUpAxis(s,UsdGeom.Tokens.z)
 f='/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Pool Enviroment/pool.usd'
 names=['repair_bolt','repair_plug','rescue_pill','rescue_bandage']
 for name in ['restore_table']+names:
  p=UsdGeom.Xform.Define(s,'/World/'+name).GetPrim();p.GetReferences().AddReference(f,'/World/'+name)
 control=UsdGeom.Cube.Define(s,'/World/ZeroGravityControl');control.CreateSizeAttr(.02);control.AddTranslateOp().Set(Gf.Vec3d(0,0,2))
 UsdPhysics.RigidBodyAPI.Apply(control.GetPrim());UsdPhysics.MassAPI.Apply(control.GetPrim()).CreateMassAttr(.2)
 UsdPhysics.CollisionAPI.Apply(control.GetPrim())
 for i in range(4):app.update()
 w.reset()
 for i in range(360):w.step(render=False)
 xc=UsdGeom.XformCache();results={}
 for name in names:
  z=xc.GetLocalToWorldTransform(s.GetPrimAtPath('/World/'+name)).ExtractTranslation()[2]
  assert .83<z<.86,(name,z)
  results[name]=float(z)
 z=xc.GetLocalToWorldTransform(control.GetPrim()).ExtractTranslation()[2];assert abs(z-2)<.002,z
 result={'passed':True,'sim_seconds':3,'settled_bottom_z':results,'tabletop_z':.8408476157511366,'zero_gravity_control_z':float(z)}
except Exception:
 result={'passed':False,'traceback':traceback.format_exc()}
Path('/tmp/task5_drop_test_result.json').write_text(json.dumps(result,indent=2))
print('DROP_TEST_RESULT',json.dumps(result),flush=True)
app.close()
