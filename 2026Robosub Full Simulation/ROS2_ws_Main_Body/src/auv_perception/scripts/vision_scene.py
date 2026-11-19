"""Optional standalone Isaac integration profile. Session layer only; NEVER saves assets.

Run with /isaacsim/python.sh. No controller is started. Runtime cameras are
attached rigidly to the measured ZED mount with an explicit 0.12m baseline.
This profile must be paired with config/perception.yaml camera_to_body.
"""
import argparse
import os
import signal
import sys
import time
from pathlib import Path

parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=3600)
args=parser.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':720,'create_new_stage':False,
                   'open_usd':'/tmp/auv_perception_vision_scene.usda'})
from isaacsim.core.utils.extensions import enable_extension
enable_extension('isaacsim.ros2.bridge')
import omni.usd
import omni.timeline
import omni.graph.core as og
from pxr import Usd,UsdGeom,UsdPhysics,Gf,Sdf
import numpy as np

base=Path('/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace')
ws=base/'2026Robosub Full Simulation/ROS2_ws_Main_Body'
scene=base/'scenes/Full Process AUV Simulation 2026/Pool Enviroment/pool.usd'
prepared=Path('/tmp/auv_perception_vision_scene.usda')
if not prepared.is_file():raise RuntimeError('Run scripts/prepare_scene.py using USD Python first')
for _ in range(15):app.update()
enable_extension('sl.sensor.camera')
s=omni.usd.get_context().get_stage()
# Runtime-only root edits: OmniGraph generated node schemas require its root edit target.
# The stage is never saved or exported by this launcher.
s.SetEditTarget(s.GetRootLayer())
# Repair only this session's unavailable CDN reference with the locally bundled asset.
root='/World/Barracuda_2/barracuda2/base_link'
zed=root+'/zed_link/ZED_X';z=s.GetPrimAtPath(zed)
# Remove this session's old stream graph, then configure the same registered helper explicitly.
graph,node,_,_=og.Controller.edit({'graph_path':'/World/PerceptionZEDGraph','evaluator_name':'execution'},
 {og.Controller.Keys.CREATE_NODES:[('tick','omni.graph.action.OnPlaybackTick'),('zed','sl.sensor.camera.ZED_Camera')],
  og.Controller.Keys.CONNECT:[('tick.outputs:tick','zed.inputs:execIn')],
  og.Controller.Keys.SET_VALUES:[('zed.inputs:cameraPrim',[Sdf.Path(zed)]),('zed.inputs:cameraModel','ZED_X'),
      ('zed.inputs:resolution','HD1200'),('zed.inputs:fps',30),('zed.inputs:streamingPort',30002),('zed.inputs:transportLayerMode','IPC')]})
physics=UsdPhysics.Scene.Define(s,'/PerceptionZeroGravity')
physics.CreateGravityDirectionAttr((0,0,-1));physics.CreateGravityMagnitudeAttr(0.)
import runpy,builtins
sys.path.insert(0,str(ws/'src/auv_common'))
runpy.run_path(str(ws/'src/auv_isaac_bridge/isaac/auv_isaac_bridge.py'),run_name='__main__',
              init_globals={'AUV_BRIDGE_CONFIG_PATH':str(ws/'src/auv_isaac_bridge/config/isaac_bridge_ros_headless.json')})
bridge=builtins._AUV_ISAAC_BRIDGE_INSTANCE
body=s.GetPrimAtPath(root);cache=UsdGeom.XformCache()
com=np.array([.29171041,.16131385,.37127125]);mount=np.array([.008289,.058686,.36873])
camera_paths=[p for p in Usd.PrimRange(z,Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Camera)]
if len(camera_paths)!=2:raise RuntimeError('Expected two camera prims')
camera_ops=[]
for p in camera_paths:
    x=UsdGeom.Xformable(p);x.ClearXformOpOrder();op=x.AddTransformOp();x.SetResetXformStack(True)
    camera_ops.append((str(p.GetPath()),op))
def update_cameras():
    cache.Clear();M=cache.GetLocalToWorldTransform(body);A=np.array(M)[:3,:3].T
    U,_,V=np.linalg.svd(A);R=U@V
    origin=np.array(M.ExtractTranslation());pc=origin+R@com
    for name,op in camera_ops:
        offset=mount+([.06,0,0] if 'Left' in name else [-.06,0,0])
        # USD camera axes x right / y up / -z forward; body +x left,+y up,+z forward.
        Rcam=R@np.diag([-1,1,-1]);W=np.eye(4);W[:3,:3]=Rcam;W[:3,3]=pc+R@offset
        op.Set(Gf.Matrix4d(W.T.tolist()))
update_cameras()
timeline=omni.timeline.get_timeline_interface();timeline.set_end_time(1000000.);timeline.set_looping(False);timeline.play()
stop=False
def stopping(*_):
    global stop;stop=True
signal.signal(signal.SIGTERM,stopping);signal.signal(signal.SIGINT,stopping)
start=time.monotonic()
print('PERCEPTION_SCENE_READY camera baseline=.12m; no source USD saved',flush=True)
try:
    while not stop and time.monotonic()-start<args.duration:
        update_cameras();app.update();bridge.drain_main_ops()
finally:
    bridge.reset_output();bridge.shutdown();app.close()
