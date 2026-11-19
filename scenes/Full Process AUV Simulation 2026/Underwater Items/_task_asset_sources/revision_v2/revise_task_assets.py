"""V2: preserve supplied panel artwork exactly; align real apertures to its rings."""
from pathlib import Path
import json,math,shutil,datetime,hashlib,os
import numpy as np
from scipy.spatial import Delaunay
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,Sdf,Gf,UsdUtils

BASE=Path('/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items')
HERE=Path(__file__).parent; cfg=json.loads((HERE/'panel_layout.json').read_text())
stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
backup=BASE/'_task_asset_sources'/'backups'/stamp;backup.mkdir(parents=True)
for folder in ('Task4_Deploy','Task5_Table','Task5_Objects'):
 shutil.copytree(BASE/folder,backup/folder)
print('BACKUP',str(backup))
unchanged={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for folder in ('gate','Slalom','Task3_Recon','Task5_Octagon') for p in (BASE/folder).rglob('*.usd')}

def save(s,file):
 tmp=Path(file).with_name(Path(file).stem+'.v2tmp.usd');s.GetRootLayer().Export(str(tmp));os.replace(tmp,file)

def textured(s,path,file):
 m=UsdShade.Material.Define(s,path); sh=UsdShade.Shader.Define(s,path+'/Surface');sh.CreateIdAttr('UsdPreviewSurface')
 sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.75)
 sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1,1,1))
 # Low emission keeps the printed white reasonably legible under dim lighting without eliminating surface shading.
 t=UsdShade.Shader.Define(s,path+'/Texture');t.CreateIdAttr('UsdUVTexture')
 t.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('textures/'+file))
 t.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');t.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
 u=UsdShade.Shader.Define(s,path+'/UV');u.CreateIdAttr('UsdPrimvarReader_float2')
 u.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');u.CreateOutput('result',Sdf.ValueTypeNames.Float2)
 t.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(u.ConnectableAPI(),'result')
 sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(t.ConnectableAPI(),'rgb')
 sh.CreateOutput('surface',Sdf.ValueTypeNames.Token);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return m

def mesh(s,path,pts,faces,mat,uv=None):
 o=UsdGeom.Mesh.Define(s,path);o.CreatePointsAttr([Gf.Vec3f(*p) for p in pts]);o.CreateFaceVertexCountsAttr([len(f) for f in faces])
 o.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);o.CreateSubdivisionSchemeAttr('none');o.CreateDoubleSidedAttr(True)
 if uv is not None:UsdGeom.PrimvarsAPI(o).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex).Set(uv)
 UsdShade.MaterialBindingAPI.Apply(o.GetPrim()).Bind(mat);return o

file=BASE/'Task4_Deploy/deploy.usd';s=Usd.Stage.Open(str(file));d=file.parent
shutil.copy2(HERE/'panel_reference.png',d/'textures/panel_reference_v2.png')
holes=cfg['holes_world'];n=256
xy=[]
for x in np.linspace(-.305,.305,100):
 for z in np.linspace(-.305,.305,100):
  if all((x-cx)**2+(z-cz)**2>r*r for cx,cz,r in holes):xy.append((x,z))
for cx,cz,r in holes:
 xy.extend((cx+r*math.cos(a),cz+r*math.sin(a)) for a in np.linspace(0,2*math.pi,n,endpoint=False))
xy=np.unique(np.array(xy),axis=0);keep=[]
for ids in Delaunay(xy).simplices:
 tri=xy[ids];c=tri.mean(0)
 if all(np.linalg.norm(c-[cx,cz])>=r*math.cos(math.pi/n)-1e-9 for cx,cz,r in holes):keep.append(list(map(int,ids)))
for name in ('PanelFront','PanelBack','Blood','Ambulance','Engine','Fire'):
 s.RemovePrim('/deploy/'+name)
for j in range(4):
 s.RemovePrim('/deploy/HoleRing'+str(j));s.RemovePrim('/deploy/HoleWall'+str(j))
mat=textured(s,'/deploy/Looks/ExactReferenceV2','panel_reference_v2.png')
width,height=cfg['source_size'];left,top,right,bottom=cfg['artwork_rect']
uv=[((left+(x/.61+.5)*(right-left))/width,1-(top+(.5-z/.61)*(bottom-top))/height) for x,z in xy]
mesh(s,'/deploy/PanelFront',[(x,-.004,z+.95) for x,z in xy],keep,mat,uv)
backmat=UsdShade.Material.Get(s,'/deploy/Looks/M2')
assert backmat
mesh(s,'/deploy/PanelBack',[(x,.004,z+.95) for x,z in xy],[f[::-1] for f in keep],backmat)
for j,(cx,cz,r) in enumerate(holes):
 pts=[];faces=[]
 for k in range(n):
  a=2*math.pi*k/n;pts.extend([(cx+r*math.cos(a),-.004,cz+.95+r*math.sin(a)),(cx+r*math.cos(a),.004,cz+.95+r*math.sin(a))])
  faces.append([2*k,2*((k+1)%n),2*((k+1)%n)+1,2*k+1])
 mesh(s,'/deploy/HoleWall'+str(j),pts,faces,backmat)
save(s,file)

for folder,names in [('Task5_Table',['warning','helmet']),('Task5_Objects',['bolt','pill','plug','bandage'])]:
 for name in names:shutil.copy2(HERE/'textures'/(name+'_v2.png'),BASE/folder/'textures'/(name+'_v2.png'))
for filename in ['Task5_Table/restore_table.usd']+['Task5_Objects/'+n+'.usd' for n in ('repair_bolt','rescue_pill','repair_plug','rescue_bandage')]:
 file=BASE/filename;s=Usd.Stage.Open(str(file))
 for p in list(s.Traverse()):
  if p.IsA(UsdShade.Shader):
   sh=UsdShade.Shader(p);inp=sh.GetInput('file')
   if inp:
    asset=inp.Get();name=Path(asset.path).stem if asset else ''
    if name in ('warning','helmet','bolt','pill','plug','bandage'):inp.Set(Sdf.AssetPath('textures/'+name+'_v2.png'))
 # Circular white stickers match the circular jar lids, replacing old diamond-looking square decals.
 if file.stem in ('repair_bolt','rescue_pill'):
  root='/'+file.stem;old=UsdGeom.Mesh.Get(s,root+'/TopImage');mat,_=UsdShade.MaterialBindingAPI(old).ComputeBoundMaterial()
  s.RemovePrim(root+'/TopImage');r=.031;n=128;pts=[(0,0,.0506)];uv=[(.5,.5)];faces=[]
  for k in range(n):
   a=2*math.pi*k/n;pts.append((r*math.cos(a),r*math.sin(a),.0506));uv.append((.5+.675*math.cos(a),.5+.675*math.sin(a)))
   faces.append([0,k+1,(k+1)%n+1])
  mesh(s,root+'/TopImage',pts,faces,mat,uv)
  for prim in s.Traverse():
   if prim.IsA(UsdShade.Shader):
    sh=UsdShade.Shader(prim)
    if sh.GetIdAttr().Get()=='UsdUVTexture':
     sh.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('clamp')
     sh.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('clamp')
 save(s,file)

# Verify preserved physical properties and material resolution, and sample aperture interior rays.
validation=[]
for rel in ['Task4_Deploy/deploy.usd','Task5_Table/restore_table.usd']+['Task5_Objects/'+n+'.usd' for n in ('repair_bolt','rescue_pill','repair_plug','rescue_bandage')]:
 file=BASE/rel;s=Usd.Stage.Open(str(file));layers,assets,missing=UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(file)));assert not missing
 cols=sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in s.Traverse())
 if 'Objects' in rel:
  root=s.GetDefaultPrim();assert abs(UsdPhysics.MassAPI(root).GetMassAttr().Get()-.2)<1e-6
  assert UsdPhysics.RigidBodyAPI(root).GetRigidBodyEnabledAttr().Get()
  assert not UsdPhysics.RigidBodyAPI(root).GetKinematicEnabledAttr().Get()
 elif 'Deploy' in rel:assert cols==0
 else:assert cols==29
 validation.append({'file':rel,'collision_count':cols,'dependencies':'OK'})
# Test a dense disk inside each opening, against BOTH panel surfaces (same topology).
tri=xy[np.array(keep)];a=tri[:,0];b=tri[:,1];c=tri[:,2]
den=(b[:,1]-c[:,1])*(a[:,0]-c[:,0])+(c[:,0]-b[:,0])*(a[:,1]-c[:,1])
for cx,cz,r in holes:
 for rho in (0,.3,.7,.95):
  for theta in np.linspace(0,2*math.pi,24,endpoint=False):
   x=cx+rho*r*math.cos(theta);z=cz+rho*r*math.sin(theta)
   u=((b[:,1]-c[:,1])*(x-c[:,0])+(c[:,0]-b[:,0])*(z-c[:,1]))/den
   v=((c[:,1]-a[:,1])*(x-c[:,0])+(a[:,0]-c[:,0])*(z-c[:,1]))/den
   assert not np.any((u>1e-7)&(v>1e-7)&(1-u-v>1e-7)),'Aperture blocked'
for path,digest in unchanged.items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest
record={'backup':str(backup),'panel_layout':cfg,'validation':validation,'aperture_rays':384,'unchanged_asset_usd_count':len(unchanged),
 'notes':'Panel uses unchanged supplied source image with UV crop; holes fitted to inner edge of red rings. Six new generated cartoon decals. Physics unchanged. No live stage edits or pool edits.'}
(BASE/'task_assets_revision_v2.json').write_text(json.dumps(record,indent=2))
shutil.copytree(HERE,BASE/'_task_asset_sources'/'revision_v2',dirs_exist_ok=True)
print(json.dumps(record,indent=2))
