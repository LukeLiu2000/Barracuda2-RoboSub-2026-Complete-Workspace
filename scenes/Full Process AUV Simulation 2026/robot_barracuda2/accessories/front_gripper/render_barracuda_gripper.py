from pathlib import Path
from pxr import Usd,UsdGeom,Gf
B=Path('/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026')
A=B/'robot_barracuda2/accessories/front_gripper'
file=A/'preview_scene.usd';s=Usd.Stage.CreateInMemory();root=UsdGeom.Xform.Define(s,'/Robot');s.SetDefaultPrim(root.GetPrim());UsdGeom.SetStageMetersPerUnit(s,1);UsdGeom.SetStageUpAxis(s,'Z')
root.GetPrim().GetReferences().AddReference(str(B/'robot_barracuda2/Barracuda 2.gripper_candidate.usd'))
root.ClearXformOpOrder();m=Gf.Matrix4d().SetScale(.001)*Gf.Matrix4d().SetRotate(Gf.Matrix3d(0,1,0,0,0,1,1,0,0).ExtractRotation());root.AddTransformOp(opSuffix='preview').Set(m)
s.GetPrimAtPath('/Robot/barracuda2/base_link/zed_link').SetActive(False);s.GetRootLayer().Export(str(file))
src=(B/'Underwater Items/_task_asset_sources/revision_v2/render_task_assets.py').read_text().split('canvas=Image.new')[0]
src=src.replace('for p in s.Traverse():','for p in Usd.PrimRange.Stage(s, Usd.TraverseInstanceProxies()):')
src=src.replace('if not p.IsA(UsdGeom.Gprim):continue',"if not p.IsA(UsdGeom.Gprim):continue\n  if '/collisions' in str(p.GetPath()) or UsdGeom.Imageable(p).ComputeVisibility()=='invisible':continue")
src=src.replace('if mat:',"if mat and s.GetPrimAtPath(str(mat.GetPath())+'/Surface'):")
src=src.replace('[-.20,1,-.65]','[-.4,1,-.45]')
exec(src)
render(str(file),1000).save(A/'robot_gripper_preview.png')
print('Robot gripper CPU preview saved')
