"""CPU preview of composed USD geometry/materials, without opening a Kit app."""
from pathlib import Path
import math,json
import numpy as np
from PIL import Image,ImageDraw
from pxr import Usd,UsdGeom,UsdShade,Gf
BASE=Path('/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items')
report=json.loads((BASE/'task_assets_manifest.json').read_text()); cache={}

def render(file,size=480):
 s=Usd.Stage.Open(file); xc=UsdGeom.XformCache(); geometry=[]
 for p in s.Traverse():
  if not p.IsA(UsdGeom.Gprim):continue
  m=xc.GetLocalToWorldTransform(p)
  if p.IsA(UsdGeom.Cube):
   v=np.array([(x,y,z) for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])
   fs=[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]];uv=None
  elif p.IsA(UsdGeom.Cylinder):
   o=UsdGeom.Cylinder(p);r=o.GetRadiusAttr().Get();h=o.GetHeightAttr().Get();n=32
   v=np.array([(r*math.cos(i*2*math.pi/n),r*math.sin(i*2*math.pi/n),z) for z in (-h/2,h/2) for i in range(n)])
   fs=[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]+[list(range(n-1,-1,-1)),list(range(n,2*n))];uv=None
  elif p.IsA(UsdGeom.Mesh):
   o=UsdGeom.Mesh(p);v=np.array(o.GetPointsAttr().Get());cs=o.GetFaceVertexCountsAttr().Get();ix=o.GetFaceVertexIndicesAttr().Get();fs=[];start=0
   for n in cs:fs.append(list(ix[start:start+n]));start+=n
   u=UsdGeom.PrimvarsAPI(p).GetPrimvar('st');uv=np.array(u.Get()) if u else None
  else:continue
  v=np.array([m.Transform(Gf.Vec3d(*map(float,q))) for q in v])
  mat,_=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial(); color=np.array([.8,.8,.8]);tex=None
  if mat:
   surface=UsdShade.Shader(s.GetPrimAtPath(str(mat.GetPath())+'/Surface'))
   color=np.array(surface.GetInput('diffuseColor').Get())
   texture=UsdShade.Shader(s.GetPrimAtPath(str(mat.GetPath())+'/Texture'))
   if texture:
    asset=texture.GetInput('file').Get();path=str(Path(file).parent/asset.path)
    if path not in cache:cache[path]=np.array(Image.open(path).convert('RGB'))
    tex=cache[path]
  for f in fs:
   for j in range(1,len(f)-1):
    ids=[f[0],f[j],f[j+1]];geometry.append((v[ids],color,uv[ids] if uv is not None else None,tex))
 # Camera from front (-Y) and above; frame to real composed world bounds.
 forward=np.array([-.20,1,-.65]);forward/=np.linalg.norm(forward)
 right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);up=np.cross(right,forward)
 basis=np.array([right,up,forward]);allv=np.concatenate([g[0] for g in geometry]); projected=allv@basis.T
 lo=projected.min(0);hi=projected.max(0);scale=(size-50)/max(hi[0]-lo[0],hi[1]-lo[1]);center=(lo+hi)/2
 img=np.full((size,size,3),[28,35,45],dtype=np.uint8);depth=np.full((size,size),np.inf)
 for verts,color,uv,tex in geometry:
  q=(verts@basis.T-center)*scale;q[:,0]+=size/2;q[:,1]=size/2-q[:,1]
  minx,maxx=max(0,int(np.floor(q[:,0].min()))),min(size-1,int(np.ceil(q[:,0].max())))
  miny,maxy=max(0,int(np.floor(q[:,1].min()))),min(size-1,int(np.ceil(q[:,1].max())))
  if minx>maxx or miny>maxy:continue
  x,y=np.meshgrid(np.arange(minx,maxx+1)+.5,np.arange(miny,maxy+1)+.5)
  a,b,c=q;den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
  if abs(den)<1e-10:continue
  w1=((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/den
  w2=((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/den;w3=1-w1-w2
  z=w1*a[2]+w2*b[2]+w3*c[2];ds=depth[miny:maxy+1,minx:maxx+1];ok=(w1>=-1e-7)&(w2>=-1e-7)&(w3>=-1e-7)&(z<ds)
  if not ok.any():continue
  if tex is not None and uv is not None:
   st=w1[...,None]*uv[0]+w2[...,None]*uv[1]+w3[...,None]*uv[2]
   xx=np.clip((st[...,0]*(tex.shape[1]-1)).astype(int),0,tex.shape[1]-1)
   yy=np.clip(((1-st[...,1])*(tex.shape[0]-1)).astype(int),0,tex.shape[0]-1);rgb=tex[yy,xx]
  else:
   normal=np.cross(verts[1]-verts[0],verts[2]-verts[0]);normal/=max(np.linalg.norm(normal),1e-12)
   shade=.55+.45*abs(np.dot(normal,np.array([.2,-.5,.84])));rgb=np.broadcast_to(np.clip(color*255*shade,0,255).astype(np.uint8),(*x.shape,3))
  img[miny:maxy+1,minx:maxx+1][ok]=rgb[ok];ds[ok]=z[ok]
 return Image.fromarray(img)

canvas=Image.new('RGB',(1920,1040),(28,35,45));d=ImageDraw.Draw(canvas)
for i,a in enumerate(report['assets']):
 img=render(a['file']);x=(i%4)*480;y=(i//4)*520;canvas.paste(img,(x,y));d.text((x+15,y+485),Path(a['file']).stem,fill='white')
canvas.save(BASE/'task_assets_preview.png')
print('Preview saved; all 8 composed USDs rendered on CPU')
