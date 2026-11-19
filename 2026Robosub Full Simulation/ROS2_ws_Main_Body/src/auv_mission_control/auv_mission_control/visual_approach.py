"""Reusable 2D target selection and pre-localization state machine (no ROS)."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def select_image_target(message, task, options):
    """Prefer a trusted whole-object bbox; otherwise estimate from semantic components.

    No mask-pixel centroid: frame/leg areas are asymmetric. Distinct task label sets
    allow the acquisition controller to be shared with future geometric planners.
    """
    width, height = int(message['width']), int(message['height'])
    if width <= 0 or height <= 0: raise ValueError('Invalid image size')
    labels = task['visual_target']
    valid = []
    for d in message.get('detections', []):
        bbox = np.asarray(d['bbox'], float)
        conf = float(d['confidence'])
        if (bbox.shape != (4,) or not np.isfinite(np.r_[bbox, conf]).all()
                or bbox[0]<0 or bbox[1]<0 or bbox[2]>width or bbox[3]>height
                or bbox[2]<=bbox[0] or bbox[3]<=bbox[1]):continue
        if conf < options['min_confidence']: continue
        if (bbox[2]-bbox[0])*(bbox[3]-bbox[1])/(width*height) < options['min_bbox_fraction']:continue
        valid.append(dict(d, bbox=bbox))
    whole = [d for d in valid if d['label'] in labels.get('whole_labels', [])]
    source = 'whole'
    if whole:
        d = max(whole, key=lambda d:d['confidence'])
        bbox = d['bbox']; center = (bbox[:2]+bbox[2:])/2
        confidence = d['confidence']; selected = [d['label']]
    else:
        # At most one best instance per semantic part; never union same-class instances.
        parts = {}
        for d in valid:
            if d['label'] in labels.get('part_labels', []):
                if d['label'] not in parts or d['confidence']>parts[d['label']]['confidence']:parts[d['label']]=d
        if not parts:return None
        source = 'parts'
        if labels.get('part_selection') == 'highest_confidence':
            d = max(parts.values(), key=lambda item: float(item['confidence']))
            bbox = d['bbox']; center = (bbox[:2]+bbox[2:])/2
            confidence = float(d['confidence']); selected = [d['label']]
        else:
            boxes = np.array([d['bbox'] for d in parts.values()])
            bbox = np.r_[boxes[:, :2].min(axis=0), boxes[:, 2:].max(axis=0)]
            center = (bbox[:2]+bbox[2:])/2
            for preferred in (labels.get('center_labels', []), labels.get('horizontal_labels', [])):
                candidates = [parts[x] for x in preferred if x in parts]
                if candidates:
                    d=max(candidates,key=lambda d:d['confidence']); center[0]=(d['bbox'][0]+d['bbox'][2])/2
                    break
            confidence = float(np.mean([d['confidence'] for d in parts.values()]))
            selected = list(parts)
    return dict(center=center.tolist(), bbox=bbox.tolist(), confidence=float(confidence), source=source,
                labels=selected, width_fraction=float((bbox[2]-bbox[0])/width),
                height_fraction=float((bbox[3]-bbox[1])/height))


def world_ray(center, projection, camera_to_body, body_to_world):
    P = np.asarray(projection, float)
    if P.shape != (12,) or not np.isfinite(P).all() or P[0]<=0 or P[5]<=0:raise ValueError('Invalid camera projection')
    u,v = center
    ray = np.array([(u-P[2])/P[0], (v-P[6])/P[5], 1.])
    R = np.asarray(body_to_world)[:3, :3]@np.asarray(camera_to_body)[:3, :3]
    ray = R@ray
    return ray/np.linalg.norm(ray)


class VisualApproach:
    """SEARCH -> ALIGN_2D -> APPROACH_2D; stops on stale/lost/too-close targets.

    Returns None when rotation search should run, otherwise a world-frame pose.
    3D confirmation and waypoint planning take priority in the parent mission.
    """
    def __init__(self, options, body_to_task):
        self.opt = options; self.body_to_task = body_to_task
        self.target = None; self.samples = 0; self.last_stamp = None
        self.last_ray = None; self.phase = 'SEARCH'; self.hold_pose = None
        self.last_seen = None; self.travel = 0.; self.last_position = None; self.started = None
        self.active_time=0.;self.last_step=None;self.reason='searching for image target'
        self.loss_started=None
        self.metrics = {}

    def reset(self, now, pose):
        self.__init__(self.opt, self.body_to_task)
        self.last_position = pose[:3, 3].copy()

    def observe(self, target, now):
        if target is None:return
        stamp = float(target['stamp'])
        if not -self.opt['max_future_s'] <= now-stamp <= self.opt['max_age_s']:return
        if self.last_stamp is not None and stamp <= self.last_stamp:return
        ray = np.asarray(target['ray_world'],float)
        if ray.shape != (3,) or not np.isfinite(ray).all() or np.linalg.norm(ray)<1e-6:return
        ray /= np.linalg.norm(ray)
        jump = self.last_ray is not None and np.dot(ray,self.last_ray)<math.cos(math.radians(self.opt['max_bearing_jump_deg']))
        gap = self.last_stamp is not None and stamp-self.last_stamp>self.opt['max_age_s']
        self.samples = 1 if jump or gap else self.samples+1
        self.target = dict(target,ray_world=ray); self.last_ray=ray; self.last_stamp=stamp
        self.last_seen=now
        self.loss_started=None

    def step(self, now, pose):
        p,R=pose[:3,3],pose[:3,:3]
        dt=max(0.,min(.3,now-self.last_step)) if self.last_step is not None else 0.
        self.last_step=now
        if self.phase=='APPROACH_2D':self.active_time+=dt
        if self.last_position is not None:self.travel += float(np.linalg.norm(p-self.last_position))
        self.last_position=p.copy()
        self.metrics.update(visual_travel_m=self.travel,visual_active_s=self.active_time,
                            visual_time_limit_s=self.opt['timeout_s'],visual_distance_limit_m=self.opt['max_travel_m'],
                            visual_lookahead_m=self.opt['lookahead_m'])
        fresh=self.target is not None and -self.opt['max_future_s'] <= now-self.target['stamp'] <= self.opt['max_age_s']
        if not fresh:
            if self.last_seen is not None and self.loss_started is None:
                self.loss_started=now;self.hold_pose=pose.copy()
            if self.loss_started is not None and now-self.loss_started <= self.opt['loss_grace_s']:
                self.phase='WAIT_2D';self.reason='2D target missing/stale; holding'; return self.hold_pose.copy()
            self.phase='SEARCH'; self.samples=0;self.reason='searching for image target'; return None
        target=self.target; ray=target['ray_world']
        if self.started is None:self.started=now
        if self.hold_pose is None:self.hold_pose=pose.copy()
        self.metrics.update(visual_confidence=target.get('confidence'),visual_source=target['source'],visual_labels=target['labels'],visual_center=target['center'],
                            visual_samples=self.samples,visual_image_age_s=now-target['stamp'],
                            visual_width_fraction=target['width_fraction'],visual_height_fraction=target['height_fraction'])
        if self.travel>=self.opt['max_travel_m'] or self.active_time>=self.opt['timeout_s']:
            if self.phase!='FAILED':self.hold_pose=pose.copy()
            if self.travel>=self.opt['max_travel_m']:
                self.reason='2D approach distance limit reached: %.2f / %.2f m'%(self.travel,self.opt['max_travel_m'])
            else:self.reason='2D approach motion-time limit reached: %.1f / %.1f s'%(self.active_time,self.opt['timeout_s'])
            self.phase='FAILED'; return self.hold_pose.copy()
        if target['width_fraction']>=self.opt['stop_width_fraction'] or target['height_fraction']>=self.opt['stop_height_fraction']:
            if self.phase!='WAIT_3D':self.hold_pose=pose.copy()
            self.reason='2D target too large; waiting for 3D (width=%.2f height=%.2f)'%(target['width_fraction'],target['height_fraction'])
            self.phase='WAIT_3D'; return self.hold_pose.copy()
        if math.hypot(ray[0],ray[1])<.1:
            self.reason='2D bearing is nearly vertical; holding for 3D'
            self.phase='WAIT_3D'; return self.hold_pose.copy()
        heading=math.atan2(ray[1],ray[0]); actual=math.atan2(R[1,2],R[0,2])
        yaw_error=math.atan2(math.sin(heading-actual),math.cos(heading-actual))
        goal=pose.copy()
        goal[:3,:3]=Rotation.from_euler('z',heading).as_matrix()@self.body_to_task
        aligned=abs(yaw_error)<=math.radians(self.opt['align_tolerance_deg'])
        ready=self.samples>=self.opt['stable_frames']
        old_phase=self.phase
        if not aligned or not ready:
            if old_phase!='ALIGN_2D':self.hold_pose=pose.copy()
            goal[:3,3]=self.hold_pose[:3,3]; self.phase='ALIGN_2D'
            self.reason='confirming 2D frames: %d / %d'%(self.samples,self.opt['stable_frames']) if not ready else 'aligning 2D target: yaw error %.1f deg'%math.degrees(yaw_error)
        else:
            self.phase='APPROACH_2D'
            self.reason='approaching 2D target with %.1f m virtual lookahead'%self.opt['lookahead_m']
            direction=ray.copy()
            direction[2]=np.clip(direction[2],-self.opt['max_vertical_ray'],self.opt['max_vertical_ray'])
            direction/=np.linalg.norm(direction)
            if target['source']=='parts' and not self.opt.get('part_vertical_enabled',False):direction[2]=0.
            direction/=np.linalg.norm(direction)
            distance=self.opt['lookahead_m']
            goal[:3,3]=p+distance*direction
        self.metrics.update(visual_yaw_error_deg=math.degrees(yaw_error))
        return goal
