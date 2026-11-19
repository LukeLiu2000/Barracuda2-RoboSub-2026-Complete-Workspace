"""Only disable the two explicitly approved task collision shapes.

Offline: --apply-files (backs up physics layers, verifies whole-stage diff).
Isaac Script Editor: runpy.run_path(..., run_name='__main__') (reload only these layers).
Never saves the live root/session layer or changes robot state.
"""
from pathlib import Path
import hashlib
import json
import difflib
import datetime
import shutil
import sys
from pxr import Usd, UsdPhysics, Sdf

BASE = Path('/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/scenes/Full Process AUV Simulation 2026')
SCENE = BASE/'Pool Enviroment/pool.usd'
WS = Path('/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/2026Robosub Full Simulation/ROS2_ws_Main_Body')
TARGETS = [
    ('/World/gate_01/gate/base_link/collisions/gate/Scene',
     BASE/'Underwater Items/gate/configuration/gate_physics.usd', '/colliders/base_link/gate/Scene'),
    ('/World/Slalom/channel/base_link/collisions/channel_full/Scene',
     BASE/'Underwater Items/Slalom/configuration/channel_physics.usd', '/colliders/base_link/channel_full/Scene'),
]


def snapshot(stage):
    out = {}
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        p = str(prim.GetPath())
        out[p+'|metadata'] = str(prim.GetAllMetadata())
        for attr in prim.GetAttributes():
            key = p+'.'+attr.GetName()
            out[key] = (str(attr.GetTypeName()), str(attr.Get()),
                        str(attr.GetConnections()), tuple((t, str(attr.Get(t))) for t in attr.GetTimeSamples()))
        for rel in prim.GetRelationships():out[p+'.'+rel.GetName()] = str(rel.GetTargets())
    return out


def verify(stage, before):
    after = snapshot(stage)
    diffs = sorted(k for k in set(before)|set(after) if before.get(k) != after.get(k))
    allowed = {p+'.physics:collisionEnabled' for p, _, _ in TARGETS}
    unexpected = [k for k in diffs if k not in allowed]
    if unexpected:raise RuntimeError('Unexpected scene changes: '+str(unexpected[:15]))
    for p, _, _ in TARGETS:
        prim = stage.GetPrimAtPath(p)
        if not prim or not prim.HasAPI(UsdPhysics.CollisionAPI):raise RuntimeError('Missing collider '+p)
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False:
            raise RuntimeError('Collision still enabled: '+p)
    robot = stage.GetPrimAtPath('/World/Barracuda_2/barracuda2/base_link/mesh_')
    if not robot or not UsdPhysics.CollisionAPI(robot).GetCollisionEnabledAttr().Get():
        raise RuntimeError('Robot collider missing/disabled')
    return diffs, len(before)


def apply_files():
    stage = Usd.Stage.Open(str(SCENE))
    before = snapshot(stage)
    # Exact observed topology; do not accidentally leave another collider active.
    for root in ('/World/gate_01', '/World/Slalom'):
        colliders = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI)]
        expected = [p for p, _, _ in TARGETS if p.startswith(root+'/')]
        if colliders != expected:raise RuntimeError('Unexpected collider topology under '+root)
    folder = WS/'runs/scene_collision_backups'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    folder.mkdir(parents=True)
    original = {}
    texts = {}
    for _, path, _ in TARGETS:
        original[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        shutil.copy2(path, folder/path.name)
        texts[str(path)] = Sdf.Layer.FindOrOpen(str(path)).ExportToString()
    try:
        for _, path, prim_path in TARGETS:
            layer = Sdf.Layer.FindOrOpen(str(path))
            prim = layer.GetPrimAtPath(prim_path)
            if not prim:raise RuntimeError('Missing source prim '+prim_path)
            attr = prim.attributes.get('physics:collisionEnabled')
            if attr is None:attr = Sdf.AttributeSpec(prim, 'physics:collisionEnabled', Sdf.ValueTypeNames.Bool)
            attr.default = False
        changes, property_count = verify(stage, before)
        for _, path, _ in TARGETS:Sdf.Layer.FindOrOpen(str(path)).Save()
        # Verify serialized physics layers were saved, without modifying visual assets.
        diffs = []
        for _, path, _ in TARGETS:
            layer = Sdf.Layer.FindOrOpen(str(path)); layer.Reload()
            diffs += list(difflib.unified_diff(texts[str(path)].splitlines(), layer.ExportToString().splitlines(), fromfile=path.name+':before', tofile=path.name+':after'))
        verify(stage, before)
    except Exception:
        for _, path, _ in TARGETS:
            shutil.copy2(folder/path.name, path)
            Sdf.Layer.FindOrOpen(str(path)).Reload()
        raise
    report = dict(scene=str(SCENE), backup=str(folder), original_sha256=original,
                  changes=changes, checked_properties=property_count,
                  robot_collision_enabled=True, visual_and_other_properties_unchanged=True,
                  live_applied=False, usd_diff=diffs)
    (folder/'manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    (WS/'runs/scene_collision_backups/latest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))


def apply_live():
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    if not stage or Path(stage.GetRootLayer().realPath).resolve() != SCENE.resolve():
        raise RuntimeError('Current stage is not the approved pool.usd')
    before = snapshot(stage)
    layers = []
    for _, path, _ in TARGETS:
        layer = Sdf.Layer.Find(str(path))
        if not layer:raise RuntimeError('Expected physics layer is not loaded: '+str(path))
        if layer.dirty:raise RuntimeError('Physics source has unsaved changes; refusing reload: '+str(path))
        layers.append(layer)
    for layer in layers:layer.Reload()
    changes, count = verify(stage, before)
    report = dict(changes=changes, checked_properties=count, live_applied=True,
                  robot_collision_enabled=True, visual_and_other_properties_unchanged=True)
    output = WS/'runs/scene_collision_backups/live_applied.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2))
    print('DONE: Gate + Slalom collisions OFF. Robot/visual/mass/transforms unchanged. No root/session save.')


if __name__ == '__main__':
    if '--apply-files' in sys.argv:apply_files()
    else:apply_live()
