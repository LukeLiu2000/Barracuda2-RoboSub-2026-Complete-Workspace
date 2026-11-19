"""Run in Isaac Script Editor with runpy. USD joint drive control only; no ROS changes."""
import omni.usd
from pxr import UsdPhysics

_watcher=None
_last=None

def _root(stage=None):
 stage=stage or omni.usd.get_context().get_stage()
 for p in ('/World/Barracuda_2/barracuda2/front_gripper','/World/barracuda2/front_gripper'):
  prim=stage.GetPrimAtPath(p)
  if prim:return prim
 for prim in stage.Traverse():
  if prim.GetAttribute('gripper:nominalMaximumOpeningMm'):return prim
 raise RuntimeError('No Barracuda front_gripper found in current stage')

def set_open_fraction(value):
 """1=fully open, 0=fully closed. Intermediate values are angle fractions, not calibrated mm."""
 global _last
 value=float(value)
 if not 0<=value<=1:raise ValueError('open fraction must be between 0 and 1')
 stage=omni.usd.get_context().get_stage();root=_root(stage);angle=40*(1-value);count=0
 from pxr import Usd
 for p in Usd.PrimRange(root):
  if p.IsA(UsdPhysics.RevoluteJoint):
   sign=p.GetAttribute('gripper:closingSign').Get()
   if sign is not None:UsdPhysics.DriveAPI(p,'angular').GetTargetPositionAttr().Set(sign*angle);count+=1
 if count!=6:raise RuntimeError(f'Expected six coupled joint drives, found {count}')
 root.GetAttribute('gripper:openFraction').Set(value);_last=value
 return {'open_fraction':value,'joint_drives':count,'closing_angle_deg':angle}

def open_gripper():return set_open_fraction(1)
def close_gripper():return set_open_fraction(0)

def enable_gui_control():
 """Watch the front_gripper custom openFraction field during current Isaac session."""
 global _watcher,_last
 import omni.kit.app
 _last=None
 def update(event):
  global _last
  try:
   value=_root().GetAttribute('gripper:openFraction').Get()
   if value!=_last:set_open_fraction(value)
  except (RuntimeError,ValueError):pass
 _watcher=omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(update,name='BarracudaGripperControl')
 return _watcher

if __name__=='__main__':
 print('Ready. open_gripper(), close_gripper(), set_open_fraction(0..1), enable_gui_control(). Play is required for physical motion.')
