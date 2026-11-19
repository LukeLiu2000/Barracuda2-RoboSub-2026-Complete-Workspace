"""Create disposable overlay, never edit/save original simulation USD assets."""
from pxr import Usd,UsdGeom,UsdPhysics,Sdf,Gf
from pathlib import Path
import numpy as np
source='/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Pool Enviroment/pool.usd'
path=Path('/tmp/auv_perception_vision_scene.usda')
if path.exists():path.unlink()
s=Usd.Stage.CreateNew(str(path));s.GetRootLayer().subLayerPaths=[source]
UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z')
root='/World/Barracuda_2/barracuda2/base_link';zed=root+'/zed_link/ZED_X'
z=s.GetPrimAtPath(zed);z.GetReferences().SetReferences([Sdf.Reference('/isaacsim/exts/sl.sensor.camera/data/usd/ZED_X.usdc')])
z.GetAttribute('physics:rigidBodyEnabled').Set(False)
s.GetPrimAtPath('/World/ActionGraph').SetActive(False)
sc=UsdPhysics.Scene.Define(s,'/PerceptionZeroGravity');sc.CreateGravityDirectionAttr((0,0,-1));sc.CreateGravityMagnitudeAttr(0.)
cache=UsdGeom.XformCache();M=cache.GetLocalToWorldTransform(s.GetPrimAtPath(root));A=np.array(M)[:3,:3].T;U,_,V=np.linalg.svd(A);R=U@V
com=np.array([.29171041,.16131385,.37127125]);pc=np.array(M.ExtractTranslation())+R@com
for p in Usd.PrimRange(z,Usd.TraverseInstanceProxies()):
 if p.IsA(UsdGeom.Camera):
  x=UsdGeom.Xformable(p);x.ClearXformOpOrder();op=x.AddTransformOp();x.SetResetXformStack(True)
  offset=np.array([.008289,.058686,.36873])+([.06,0,0] if 'Left' in p.GetName() else [-.06,0,0])
  W=np.eye(4);W[:3,:3]=R@np.diag([-1,1,-1]);W[:3,3]=pc+R@offset;op.Set(Gf.Matrix4d(W.T.tolist()))
for _ in range(4):
 for prim in list(s.TraverseAll()):
  if prim.IsInstanceable():prim.SetInstanceable(False)
flat=s.Flatten()
# Saved render/graph execution caches are not portable across Kit sessions.
for obsolete in ['/Render','/World/ActionGraph','/OmniverseKit_Persp']:
 if flat.GetPrimAtPath(obsolete):
  edit=Sdf.BatchNamespaceEdit();edit.Add(obsolete,Sdf.Path.emptyPath);flat.Apply(edit)
flat.Export(str(path));print('Disposable flattened scene (render cache removed):',path)
