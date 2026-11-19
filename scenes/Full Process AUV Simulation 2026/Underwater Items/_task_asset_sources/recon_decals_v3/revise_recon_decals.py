"""Task3 high-resolution decals only; preserve placement, frame and physics."""
from pathlib import Path
import hashlib,json,shutil,datetime,os
from pxr import Usd,UsdGeom,UsdShade,Sdf,Gf,UsdUtils

B=Path('/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026')
D=B/'Underwater Items/Task3_Recon';FILE=D/'recon.usd';HERE=Path(__file__).parent
stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S');backup=B/'Underwater Items/_task_asset_sources/backups'/('recon_decals_'+stamp)
backup.mkdir(parents=True);shutil.copytree(D,backup/'Task3_Recon')
pool=B/'Pool Enviroment/pool.usd';pool_hash=hashlib.sha256(pool.read_bytes()).hexdigest()
for name in ('fire_recon_hd_v3.png','blood_recon_hd_v3.png'):shutil.copy2(HERE/name,D/'textures'/name)
s=Usd.Stage.Open(str(FILE));targets={f'/recon/Bin{i}/Image' for i in range(4)}
matpaths=set()
for path in targets:
 mat,_=UsdShade.MaterialBindingAPI(s.GetPrimAtPath(path)).ComputeBoundMaterial();matpaths.add(str(mat.GetPath()))
def snap(p):return (p.GetTypeName(),str(p.GetAllMetadata()),[(a.GetName(),str(a.Get())) for a in p.GetAttributes()],[(r.GetName(),str(r.GetTargets())) for r in p.GetRelationships()])
def excluded(p):
 path=str(p.GetPath());return path in targets or any(path==m or path.startswith(m+'/') for m in matpaths)
before={str(p.GetPath()):snap(p) for p in s.Traverse() if not excluded(p)}
record=[]
for i in range(4):
 p=s.GetPrimAtPath(f'/recon/Bin{i}/Image');o=UsdGeom.Mesh(p);mat,_=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
 tex=UsdShade.Shader.Get(s,str(mat.GetPath())+'/Texture');file='blood_recon_hd_v3.png' if i in (0,3) else 'fire_recon_hd_v3.png'
 tex.GetInput('file').Set(Sdf.AssetPath('textures/'+file))
 tex.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('clamp');tex.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('clamp')
 # 305 mm square prints per handbook. Preserve decal height and centre.
 pts=o.GetPointsAttr().Get();cx=sum(v[0] for v in pts)/4;cy=sum(v[1] for v in pts)/4;z=pts[0][2];half=.305/2
 o.GetPointsAttr().Set([Gf.Vec3f(cx-half,cy-half,z),Gf.Vec3f(cx+half,cy-half,z),Gf.Vec3f(cx+half,cy+half,z),Gf.Vec3f(cx-half,cy+half,z)])
 # Handbook plan: left drop points down, low flame up, high flame right, right drop left.
 uv=[(0,0),(1,0),(1,1),(0,1)] if i in (0,1) else [(1,0),(1,1),(0,1),(0,0)]
 UsdGeom.PrimvarsAPI(p).GetPrimvar('st').Set(uv)
 assert all(0<=a<=1 for pair in uv for a in pair)
 record.append({'prim':str(p.GetPath()),'texture':file,'decal_size_m':.305,'uv':uv})
assert {str(p.GetPath()):snap(p) for p in s.Traverse() if not excluded(p)}==before
tmp=FILE.with_name('recon.decals_pending.usd');s.GetRootLayer().Export(str(tmp));os.replace(tmp,FILE)
c=Usd.Stage.Open(str(FILE));assert {str(p.GetPath()):snap(p) for p in c.Traverse() if not excluded(p)}==before
_,_,missing=UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(FILE)));assert not missing
assert hashlib.sha256(pool.read_bytes()).hexdigest()==pool_hash
report={'asset':str(FILE),'backup':str(backup),'images':record,'preserved_other_prims':len(before),'pool_unchanged':True,
 'validation':'Reopened USD, 4 full-coverage UVs and image dependencies checked. Geometry outside decal quads, materials outside decal materials, physics and pool transforms unchanged.',
 'texture_notes':'Two separately generated high-resolution cartoon icons, each fully visible with wide white margins. Source textures stored with native generated resolution. Two fire and two blood decals; no other task textures touched.'}
(B/'Underwater Items/recon_decals_v3.json').write_text(json.dumps(report,indent=2))
rpath=B/'Underwater Items/task_assets_manifest.json'
if rpath.exists():
 r=json.loads(rpath.read_text())
 for item in r['assets']:
  if Path(item['file'])==FILE:item['sha256']=hashlib.sha256(FILE.read_bytes()).hexdigest();item['description']='Four open bins; two bright HD fire and two blood decals, full-coverage UVs, handbook-inspired orientations; no collisions.'
 r['recon_revision']='v3 high-resolution fire/blood symbols, 305mm decals, full shapes and wide margins';rpath.write_text(json.dumps(r,indent=2))
shutil.copytree(HERE,B/'Underwater Items/_task_asset_sources/recon_decals_v3',dirs_exist_ok=True)
print(json.dumps(report,indent=2))
