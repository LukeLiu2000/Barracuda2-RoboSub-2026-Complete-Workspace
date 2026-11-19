from copy import deepcopy
from pathlib import Path
import math
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from auv_mission_control.visual_approach import VisualApproach,select_image_target,world_ray
from auv_mission_control.waypoint_mission import BODY_TO_TASK,Mission
from test_waypoint_mission import TEMPLATE,gate_pose

ROOT=Path(__file__).resolve().parents[1]
CFG=yaml.safe_load((ROOT/'config/autonomy.yaml').read_text())
TASK=yaml.safe_load((ROOT/'config/tasks/task1_gate.yaml').read_text())
OPT=CFG['visual_approach']


def pose():
    T=np.eye(4);T[:3,:3]=BODY_TO_TASK;return T


def target(t,ray=(1,0,0),source='whole',width=.2):
    return dict(stamp=t,ray_world=list(ray),source=source,labels=['Gate'],center=[320,180],width_fraction=width,height_fraction=.2)


def test_whole_gate_priority_and_bbox_center_not_mask_centroid():
    msg=dict(width=640,height=360,detections=[
        dict(label='Gate',confidence=.8,bbox=[100,50,500,300]),
        dict(label='LeftPillar',confidence=.99,bbox=[100,50,130,300]),
        dict(label='RightPillar',confidence=.99,bbox=[480,60,500,290])])
    result=select_image_target(msg,TASK,OPT)
    assert result['source']=='whole' and result['center']==[300,175]


def test_parts_use_center_rod_horizontal_anchor_and_task_config():
    msg=dict(width=640,height=360,detections=[
        dict(label='LeftPillar',confidence=.9,bbox=[100,50,130,300]),
        dict(label='MiddleRedPillar',confidence=.8,bbox=[300,50,310,160])])
    legacy=deepcopy(TASK);legacy['visual_target'].pop('part_selection',None)
    legacy['visual_target']['part_labels'].append('MiddleRedPillar')
    result=select_image_target(msg,legacy,OPT)
    assert result['source']=='parts' and result['center'][0]==305
    other=deepcopy(TASK);other['visual_target']={'whole_labels':['Task2'],'part_labels':[]}
    assert select_image_target(msg,other,OPT) is None
    msg['detections'].append(dict(label='Task2',confidence=.9,bbox=[200,80,400,280]))
    assert select_image_target(msg,other,OPT)['labels']==['Task2']


def test_camera_right_pixel_turns_right_and_below_pixel_descends():
    extrinsic=np.eye(4);extrinsic[:3,:3]=np.diag([-1,-1,1])
    P=[500,0,320,0,0,500,180,0,0,0,1,0]
    center=world_ray([320,180],P,extrinsic,pose())
    right=world_ray([420,180],P,extrinsic,pose())
    below=world_ray([320,280],P,extrinsic,pose())
    assert np.allclose(center,[1,0,0])
    assert right[1]<0 and below[2]<0


def test_no_depth_target_moves_ten_meters_after_two_frames():
    v=VisualApproach(OPT,BODY_TO_TASK);T=pose();v.reset(0,T)
    assert v.step(.1,T) is None and v.phase=='SEARCH'
    v.observe(target(.2),.2);goal=v.step(.2,T)
    assert v.phase=='ALIGN_2D' and np.allclose(goal[:3,3],T[:3,3])
    v.observe(target(.5),.5);goal=v.step(.5,T)
    assert v.phase=='APPROACH_2D' and np.allclose(goal[:3,3],[10,0,0])


def test_alignment_first_and_lost_target_holds_before_search():
    v=VisualApproach(OPT,BODY_TO_TASK);T=pose();v.reset(0,T)
    for t in (.1,.3):v.observe(target(t,(1,1,0)),t)
    goal=v.step(.3,T)
    assert v.phase=='ALIGN_2D' and np.allclose(goal[:3,3],T[:3,3])
    T[:3,:3]=Rotation.from_euler('z',45,degrees=True).as_matrix()@BODY_TO_TASK
    goal=v.step(.4,T);assert v.phase=='APPROACH_2D'
    T[:3,3]=[.2,.2,0]
    lost=.3+OPT['max_age_s']+.1
    goal=v.step(lost,T);assert v.phase=='WAIT_2D' and np.allclose(goal[:3,3],T[:3,3])
    assert v.step(lost+OPT['loss_grace_s']+.1,T) is None and v.phase=='SEARCH'


def test_close_visual_target_and_travel_budget_stop_forward_motion():
    v=VisualApproach(OPT,BODY_TO_TASK);T=pose();v.reset(0,T)
    v.observe(target(.1,width=.8),.1)
    assert np.allclose(v.step(.1,T)[:3,3],T[:3,3]) and v.phase=='WAIT_3D'
    T[:3,3]=[13,0,0];v.observe(target(.2),.2);v.step(.2,T)
    assert v.phase=='FAILED'


def test_image_target_stale_or_jumps_cannot_command_immediate_advance():
    v=VisualApproach(OPT,BODY_TO_TASK);T=pose();v.reset(0,T)
    v.observe(target(0),OPT['max_age_s']+1);assert v.target is None
    for t in (2.1,2.3):v.observe(target(t),t)
    v.step(2.3,T);assert v.phase=='APPROACH_2D'
    v.observe(target(2.5,(1,1,0)),2.5)
    assert v.samples==1
    assert v.step(2.5,T) is not None and v.phase=='ALIGN_2D'


def test_3d_confirmation_preempts_visual_approach_and_freezes_waypoints():
    m=Mission(deepcopy(CFG),deepcopy(TASK),TEMPLATE)
    T=pose();T[:3,3]=[7,4,2];m.begin(0,T)
    for t in (.1,.3):m.observe_2d(target(t),t);m.step(t,T,np.zeros(3),np.zeros(3))
    assert m.phase=='APPROACH_2D'
    for t in (.5,1.1,1.7):
        m.observe({'T':gate_pose().tolist(),'last_accepted':t,'quality':{'accepted':True}},t)
        m.step(t,T,np.zeros(3),np.zeros(3))
    assert m.phase=='EXECUTE'
    locked=m.target.copy()
    m.observe_2d(target(1.8,(0,1,0)),1.8)
    assert np.allclose(m.target,locked)


def test_alignment_does_not_exhaust_forward_motion_budget():
    options=deepcopy(OPT);options['timeout_s']=1.
    v=VisualApproach(options,BODY_TO_TASK);T=pose();v.reset(0,T)
    for t in np.arange(.2,10,.2):
        v.observe(target(t,(1,1,0)),t);v.step(t,T)
    assert v.phase=='ALIGN_2D' and v.active_time==0.
    T[:3,:3]=Rotation.from_euler('z',45,degrees=True).as_matrix()@BODY_TO_TASK
    for t in np.arange(10,11.6,.2):
        v.observe(target(t,(1,1,0)),t);v.step(t,T)
        if v.phase=='FAILED':break
    assert v.phase=='FAILED' and 'motion-time limit' in v.reason
    assert v.metrics['visual_active_s']>=1.


def test_search_timeout_is_not_an_overall_2d_approach_timeout():
    config=deepcopy(CFG);config['search']['timeout_s']=1.
    m=Mission(config,deepcopy(TASK),TEMPLATE);T=pose();m.begin(0,T)
    for t in np.arange(.2,5,.2):
        m.observe_2d(target(t),t);m.step(t,T,np.zeros(3),np.zeros(3))
    assert m.phase=='APPROACH_2D'
    assert m.search_elapsed==0.


def test_live_low_frame_rate_keeps_consistent_target():
    v=VisualApproach(OPT,BODY_TO_TASK);T=pose();v.reset(0,T)
    v.observe(target(.1),.1);v.step(.1,T)
    v.observe(target(1.8),1.8);goal=v.step(1.8,T)
    assert v.samples==2 and v.phase=='APPROACH_2D'
    assert np.allclose(goal[:3,3],[10,0,0])


def test_each_requested_single_component_is_sufficient_and_best_instance_wins():
    for label in ('LeftPillar','RightPillar','FlagSOS','FlagRepair'):
        msg=dict(width=1920,height=1200,detections=[dict(label=label,confidence=.75,bbox=[950,580,970,600])])
        result=select_image_target(msg,TASK,OPT)
        assert result['labels']==[label] and result['center']==[960,590]
    msg['detections'] += [dict(label='RightPillar',confidence=.94,bbox=[100,100,140,200]),
                          dict(label='FlagSOS',confidence=.70,bbox=[1500,500,1540,540])]
    result=select_image_target(msg,TASK,OPT)
    assert result['labels']==['RightPillar'] and result['center']==[120,150]
    assert result['confidence']==.94


def test_confidence_075_rejects_false_parts_and_low_gate_cannot_override_good_part():
    def d(label,confidence,box=None):
        return dict(label=label,confidence=confidence,bbox=box or [200,100,240,200])
    for label in ('Gate','LeftPillar','RightPillar','FlagSOS','FlagRepair'):
        msg=dict(width=640,height=360,detections=[d(label,.749999)])
        assert select_image_target(msg,TASK,OPT) is None
        msg['detections'][0]['confidence']=.75
        assert select_image_target(msg,TASK,OPT)['labels']==[label]
    msg=dict(width=640,height=360,detections=[d('Gate',.4),d('FlagSOS',.8),d('LeftPillar',.9)])
    assert select_image_target(msg,TASK,OPT)['labels']==['LeftPillar']
    msg['detections'].append(d('Gate',.75))
    assert select_image_target(msg,TASK,OPT)['labels']==['Gate']


def test_low_confidence_only_keeps_searching_without_forward_target():
    engine=Mission(CFG,TASK,TEMPLATE);T=pose();engine.begin(0,T)
    for t in (.1,.4,.8):
        msg=dict(width=640,height=360,detections=[dict(label='FlagRepair',confidence=.40,bbox=[300,150,340,190])])
        assert select_image_target(msg,TASK,OPT) is None
        goal=engine.step(t,T,np.zeros(3),np.zeros(3))
        assert engine.phase=='SEARCH'
        assert np.allclose(goal[:3,3],T[:3,3])
