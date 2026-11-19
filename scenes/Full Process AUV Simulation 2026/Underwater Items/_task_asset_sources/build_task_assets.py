"""Offline USD asset authoring. Units metres, Z up; no edits to pool.usd."""
from pathlib import Path
import math, json, hashlib
import numpy as np
from scipy.spatial import Delaunay
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Gf, Sdf, Vt

BASE=Path('/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026/Underwater Items')
TEX=Path(__file__).parent/'textures'
REPORT=[]

def new(folder,name):
    global stage, root, dest, serial
    dest=BASE/folder; dest.mkdir(exist_ok=True)
    import shutil
    shutil.copytree(TEX,dest/'textures',dirs_exist_ok=True)
    output=dest/(name+'.usd')
    if output.exists(): raise RuntimeError('Refusing overwrite: '+str(output))
    stage=Usd.Stage.CreateNew(str(output)); root='/'+name; serial=0
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,root).GetPrim())
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage,1)
    return output

def material(color=(.85,.85,.82),texture=None):
    global serial
    serial+=1; p=root+'/Looks/M'+str(serial)
    m=UsdShade.Material.Define(stage,p); sh=UsdShade.Shader.Define(stage,p+'/Surface')
    sh.CreateIdAttr('UsdPreviewSurface'); sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65)
    sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    if texture:
        uv=UsdShade.Shader.Define(stage,p+'/UV'); uv.CreateIdAttr('UsdPrimvarReader_float2')
        uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st'); uv.CreateOutput('result',Sdf.ValueTypeNames.Float2)
        t=UsdShade.Shader.Define(stage,p+'/Texture'); t.CreateIdAttr('UsdUVTexture')
        t.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('textures/'+texture+'.png'))
        t.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
        t.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
        t.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
        sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(t.ConnectableAPI(),'rgb')
    sh.CreateOutput('surface',Sdf.ValueTypeNames.Token)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface'); return m

def bind(obj,mat,collision=False):
    UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
    if collision: UsdPhysics.CollisionAPI.Apply(obj.GetPrim()).CreateCollisionEnabledAttr(True)
    return obj

def box(name,center,size,mat,collision=False):
    o=UsdGeom.Cube.Define(stage,root+'/'+name); o.CreateSizeAttr(1)
    o.AddTranslateOp().Set(Gf.Vec3d(*center)); o.AddScaleOp().Set(Gf.Vec3f(*size))
    return bind(o,mat,collision)

def pipe(name,a,b,r,mat,collision=False):
    a=np.array(a,dtype=float); b=np.array(b,dtype=float); d=b-a
    o=UsdGeom.Cylinder.Define(stage,root+'/'+name); o.CreateRadiusAttr(r); o.CreateHeightAttr(float(np.linalg.norm(d)))
    o.AddTranslateOp().Set(Gf.Vec3d(*((a+b)/2)))
    q=Gf.Rotation(Gf.Vec3d(0,0,1),Gf.Vec3d(*(d/np.linalg.norm(d)))).GetQuat()
    o.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(q)
    return bind(o,mat,collision)

def mesh(name,pts,faces,mat,uv=None):
    o=UsdGeom.Mesh.Define(stage,root+'/'+name); o.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    o.CreateFaceVertexCountsAttr([len(f) for f in faces]); o.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    o.CreateSubdivisionSchemeAttr('none'); o.CreateDoubleSidedAttr(True)
    if uv is not None:
        UsdGeom.PrimvarsAPI(o).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex).Set(uv)
    return bind(o,mat)

def label(name,center,w,h,tex,horizontal=False):
    x,y,z=center
    if horizontal: pts=[(x-w/2,y-h/2,z),(x+w/2,y-h/2,z),(x+w/2,y+h/2,z),(x-w/2,y+h/2,z)]
    else: pts=[(x-w/2,y,z-h/2),(x+w/2,y,z-h/2),(x+w/2,y,z+h/2),(x-w/2,y,z+h/2)]
    return mesh(name,pts,[[0,1,2,3]],material(texture=tex),[(0,0),(1,0),(1,1),(0,1)])

def tray(name,c,w,d,h,mat,collision=False,tex=None):
    x,y,z=c; t=.007
    box(name+'/Bottom',(x,y,z+t/2),(w,d,t),mat,collision)
    for k,xx in enumerate((x-w/2+t/2,x+w/2-t/2)):
        box(name+'/SideX'+str(k),(xx,y,z+h/2),(t,d,h),mat,collision)
    for k,yy in enumerate((y-d/2+t/2,y+d/2-t/2)):
        box(name+'/SideY'+str(k),(x,yy,z+h/2),(w-2*t,t,h),mat,collision)
    if tex: label(name+'/Image',(x,y,z+t+.0005),w-.018,d-.018,tex,True)

def save(output,description):
    stage.GetRootLayer().Save()
    REPORT.append({'file':str(output),'description':description})

def recon():
    p=new('Task3_Recon','recon'); white=material(); gray=material((.55,.59,.65))
    # Two raised rails with a central step and three floor stands.
    for i,x in enumerate((-.85,0,.85)):
        z=.55 if x<=0 else .85
        pipe('Stand'+str(i),(x,0,.025),(x,0,z),.016,white)
        pipe('Foot'+str(i),(x,-.25,.025),(x,.25,.025),.016,white)
    pipe('RailLow',(-.85,0,.55),(0,0,.55),.016,white)
    pipe('Riser',(0,0,.55),(0,0,.85),.016,white)
    pipe('RailHigh',(0,0,.85),(.85,0,.85),.016,white)
    for i,(x,y,z,tex) in enumerate([(-.62,-.24,.55,'blood'),(-.22,.24,.55,'fire'),(.25,-.24,.85,'fire'),(.65,.24,.85,'blood')]):
        pipe('Branch'+str(i),(x,0,z),(x,y,z),.016,white)
        tray('Bin'+str(i),(x,y,z+.02),.335,.335,.28,gray,False,tex)
    save(p,'Four open bins with 2 fire and 2 blood images. Visual only; approximate PVC layout.')

HOLES=[(-.21,.055,.070),(.04,.19,.048),(.04,-.20,.048),(.23,-.19,.070)]
def torpedoes():
    p=new('Task4_Deploy','deploy'); white=material(); board=material((.8,.81,.79)); red=material((.8,.02,.025))
    # Board in XZ, normal Y. Constrained-by-dense-boundary triangulation.
    n=128; xy=[]
    for x in np.linspace(-.305,.305,65):
        for z in np.linspace(-.305,.305,65):
            if all((x-cx)**2+(z-cz)**2>r*r for cx,cz,r in HOLES): xy.append((x,z))
    for cx,cz,r in HOLES:
        xy.extend((cx+r*math.cos(a),cz+r*math.sin(a)) for a in np.linspace(0,2*math.pi,n,endpoint=False))
    xy=np.unique(np.array(xy),axis=0); tris=Delaunay(xy).simplices; keep=[]
    for ids in tris:
        triangle=xy[ids]; mid=triangle.mean(axis=0)
        if all(np.linalg.norm(mid-[cx,cz])>=r*math.cos(math.pi/n)-1e-8 for cx,cz,r in HOLES): keep.append(list(map(int,ids)))
    pts=[(float(x),-.004,float(z)+.95) for x,z in xy]
    mesh('PanelFront',pts,keep,board)
    mesh('PanelBack',[(x,.004,z) for x,y,z in pts],[f[::-1] for f in keep],board)
    for j,(cx,cz,r) in enumerate(HOLES):
        # Real aperture wall plus red annulus, no hidden plane or collider.
        q=[]; f=[]; wall=[]
        for k in range(n):
            a=2*math.pi*k/n
            q.extend([(cx+r*math.cos(a),-.0045,cz+.95+r*math.sin(a)),(cx+(r+.006)*math.cos(a),-.0045,cz+.95+(r+.006)*math.sin(a))])
            wall.extend([(cx+r*math.cos(a),-.004,cz+.95+r*math.sin(a)),(cx+r*math.cos(a),.004,cz+.95+r*math.sin(a))])
            f.append([2*k,2*((k+1)%n),2*((k+1)%n)+1,2*k+1])
        mesh('HoleRing'+str(j),q,f,red); mesh('HoleWall'+str(j),wall,f,board)
    for name,x,z,tex in [('Blood',-.21,.23,'blood'),('Ambulance',.21,.23,'ambulance'),('Engine',-.20,-.18,'engine'),('Fire',.23,-.02,'fire')]:
        label(name,(x,-.006,z+.95),.12,.12,tex)
    for i,x in enumerate((-.33,.33)):
        pipe('Leg'+str(i),(x,0,.02),(x,0,1.29),.016,white)
        pipe('Foot'+str(i),(x,-.28,.02),(x,.28,.02),.016,white)
    for i,z in enumerate((.62,1.28)): pipe('Cross'+str(i),(-.33,0,z),(.33,0,z),.016,white)
    # Mathematical aperture test: no retained triangle contains the central ray.
    for cx,cz,r in HOLES:
        for ids in keep:
            t=xy[ids]; v=t-[cx,cz]
            signs=[np.cross(v[(i+1)%3]-v[i],-v[i]) for i in range(3)]
            assert not (min(signs)>1e-10 or max(signs)<-1e-10), 'Aperture obstructed'
    save(p,'0.61 m board, four true polygonal apertures (128 sides); visual only. Hole sizes/layout are explicit modeling assumptions.')

def octagon():
    p=new('Task5_Octagon','octagon'); white=material()
    # 2.7 m across parallel flats. Origin at ring centre; caller places at water surface.
    R=1.35/math.cos(math.pi/8)
    pts=[(R*math.cos(math.pi/8+i*math.pi/4),R*math.sin(math.pi/8+i*math.pi/4),0) for i in range(8)]
    for i in range(8): pipe('Edge'+str(i),pts[i],pts[(i+1)%8],.0107,white)
    save(p,'White horizontal octagonal PVC ring only; 2.7 m across flats; no signs/collisions/physics.')

def table():
    p=new('Task5_Table','restore_table'); white=material(); gray=material((.62,.65,.69))
    box('Tabletop',(0,0,.75),(.61,.61,.012),gray,True)
    for i,x in enumerate((-.31,.31)):
        for j,y in enumerate((-.31,.31)): pipe('Leg'+str(i)+str(j),(x,y,.025),(x,y,.76),.0107,white,True)
        pipe('Sled'+str(i),(x,-.50,.025),(x,.50,.025),.0107,white,True)
        pipe('TopX'+str(i),(x,-.31,.76),(x,.31,.76),.0107,white,True)
        pipe('BottomX'+str(i),(x,-.31,.12),(x,.31,.12),.0107,white,True)
    for j,y in enumerate((-.31,.31)):
        pipe('TopY'+str(j),(-.31,y,.76),(.31,y,.76),.0107,white,True)
        pipe('BottomY'+str(j),(-.31,y,.12),(.31,y,.12),.0107,white,True)
    for i,(x,tex) in enumerate([(-.435,'warning'),(.435,'helmet')]):
        tray('Basket'+str(i),(x,0,.755),.24,.28,.10,gray,True,tex)
        for j,y in enumerate((-.10,.10)): pipe('BasketSupport'+str(i)+str(j),(np.sign(x)*.30,y,.74),(x,y,.74),.0107,white,True)
    save(p,'Static collidable table and two open receiving trays. 0.61 m tabletop, height 0.75 m. No loose props.')

def objects():
    for name,tex,round_ in [('repair_bolt','bolt',True),('rescue_pill','pill',True),('repair_plug','plug',False),('rescue_bandage','bandage',False)]:
        p=new('Task5_Objects',name); body=material((.78,.82,.85)); black=material((.035,.035,.04))
        rb=UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(root)); rb.CreateRigidBodyEnabledAttr(True); rb.CreateKinematicEnabledAttr(False)
        UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(root)).CreateMassAttr(.2)
        if round_:
            pipe('Body',(0,0,0),(0,0,.043),.035,body,True)
            pipe('Lid',(0,0,.043),(0,0,.050),.035,black,True)
            label('TopImage',(0,0,.0506),.049,.049,tex,True)
        else:
            box('Body',(0,0,.015),(.084,.084,.030),body,True)
            label('TopImage',(0,0,.0306),.075,.075,tex,True)
        save(p,'Dynamic rigid body, mass 0.2 kg, gravity enabled by default; bottom origin. '+('70 mm diameter x 50 mm' if round_ else '84 x 84 x 30 mm'))

recon(); torpedoes(); octagon(); table(); objects()
for item in REPORT:
    s=Usd.Stage.Open(item['file']); prims=list(s.Traverse())
    item['collision_count']=sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims)
    item['rigid_body_count']=sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims)
    if 'Task5_Objects' in item['file']: assert item['rigid_body_count']==1 and item['collision_count']>=1
    elif 'Task5_Table' in item['file']: assert item['collision_count']>=10 and item['rigid_body_count']==0
    else: assert item['collision_count']==0 and item['rigid_body_count']==0
    item['sha256']=hashlib.sha256(Path(item['file']).read_bytes()).hexdigest()
    assert s.GetDefaultPrim() and UsdGeom.GetStageMetersPerUnit(s)==1
    for prim in prims:
        for attr in prim.GetAttributes():
            if attr.GetTypeName()==Sdf.ValueTypeNames.Asset:
                v=attr.Get()
                if v and v.path: assert (Path(item['file']).parent/v.path).exists(),v.path
report={'assets':REPORT,'coordinates':'Z up, metres. Props/table bottom at Z=0; octagon origin at ring centre. Panel faces -Y.',
        'physics_note':'Dynamic props fall only if containing scene gravity is nonzero. Existing pool was previously zero-gravity; this generator does not change it. No buoyancy/underwater drag authored.',
        'assumptions':'Recon support layout, tray dimensions, panel aperture diameters (0.14/0.096 m) and stand dimensions are adjustable visual approximations, not certified official CAD. Octagon is ring only as requested; hanging signs not included.',
        'texture_source':'Cropped illustrations supplied by user; previews may include source raster shading. Not official vector artwork.'}
(BASE/'task_assets_manifest.json').write_text(json.dumps(report,indent=2),encoding='utf8')
for d in {Path(x['file']).parent for x in REPORT}:
    (d/'README.txt').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report,indent=2))
