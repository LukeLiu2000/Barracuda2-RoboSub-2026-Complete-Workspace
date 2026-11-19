"""Task-relative waypoint planning/execution. No ROS, camera or actuator dependencies.

Task axes: X through gate, Y left, Z up. AUV body: X left, Y up, Z forward.
Gate model axes are explicitly converted; they are not vehicle axes.
"""
from dataclasses import dataclass, field
from collections import deque
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .visual_approach import VisualApproach


BODY_TO_TASK = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])


def validate_config(config, task):
    if config.get('schema_version') != 1 or task.get('schema_version') != 1:
        raise ValueError('Unknown config schema version')
    if task.get('planner') not in PLANNERS: raise ValueError('Unsupported task planner')
    for section in ('arrival', 'search', 'acquisition', 'reference', 'progress'):
        for key, value in config[section].items():
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(section+'.'+key+' must be positive and finite')
    for key in ('goal_rate_hz', 'state_timeout_wall_s'):
        if not math.isfinite(config[key]) or config[key] <= 0: raise ValueError('Invalid '+key)
    if config['arrival']['exit_ratio'] < 1: raise ValueError('exit_ratio must be >= 1')
    if config['acquisition']['samples'] < 2 or int(config['acquisition']['samples']) != config['acquisition']['samples']:
        raise ValueError('acquisition.samples must be an integer >= 2')
    if config['acquisition']['window_s'] < config['acquisition']['stable_s']:
        raise ValueError('acquisition window shorter than stable duration')
    visual=config.get('visual_approach',{})
    if visual.get('enabled',False):
        for key,value in visual.items():
            if key in ('enabled','part_vertical_enabled'):continue
            if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:raise ValueError('Invalid visual_approach.'+key)
        for key in ('stop_width_fraction','stop_height_fraction','part_speed_factor','min_confidence'):
            if visual[key]>1:raise ValueError('visual_approach.'+key+' must be <= 1')
        if int(visual['stable_frames'])!=visual['stable_frames'] or visual['stable_frames']<2:raise ValueError('stable_frames must be integer >=2')
        target=task.get('visual_target',{})
        if not target.get('whole_labels') and not target.get('part_labels'):raise ValueError('Task requires visual target labels')
    if not task.get('waypoints'): raise ValueError('No waypoints')
    names = [w['name'] for w in task['waypoints']]
    if len(names) != len(set(names)): raise ValueError('Waypoint names must be unique')
    if not isinstance(task.get('max_laps', 0), int) or task.get('max_laps', 0) < 0:
        raise ValueError('max_laps must be a nonnegative integer')
    for w in task['waypoints']:
        for key, default in [('position_m', None), ('rpy_deg', [0, 0, 0])]:
            values = np.asarray(w.get(key, default), float)
            if values.shape != (3,) or not np.isfinite(values).all(): raise ValueError('Invalid waypoint '+key)
        if w.get('mode', 'settle') not in ('settle', 'pass'): raise ValueError('Invalid waypoint mode')
        if w.get('lateral_anchor') not in (None, 'center', 'sos', 'repair'): raise ValueError('Invalid lateral_anchor')
        if w.get('height_anchor') not in (None, 'beam', 'middle_bottom'): raise ValueError('Invalid height_anchor')
        for key, value in w.get('arrival', {}).items():
            if key not in config['arrival'] or not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid waypoint arrival override '+key)
            if key == 'exit_ratio' and value < 1: raise ValueError('exit_ratio must be >= 1')


def transform(value):
    T = np.asarray(value, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-4)):
        raise ValueError('Expected finite rigid 4x4 transform')
    return T.copy()


def rotation_error(a, b):
    return float(Rotation.from_matrix(a.T @ b).magnitude())


@dataclass
class Waypoint:
    name: str
    T: np.ndarray
    mode: str = 'settle'
    tolerances: dict = field(default_factory=dict)


def make_waypoint(name, task_to_world, position, rpy_deg=(0, 0, 0), mode='settle', **tolerances):
    """Reusable single-waypoint API. RPY are fixed task X/Y/Z, degrees (Rz Ry Rx)."""
    parent = transform(task_to_world)
    p = np.asarray(position, float)
    rpy = np.asarray(rpy_deg, float)
    if p.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[p, rpy]).all():
        raise ValueError('position and rpy_deg must be finite triples')
    if mode not in ('settle', 'pass'): raise ValueError('Unknown arrival mode')
    T = np.eye(4)
    T[:3, 3] = parent[:3, 3] + parent[:3, :3] @ p
    T[:3, :3] = parent[:3, :3] @ Rotation.from_euler('xyz', rpy, degrees=True).as_matrix() @ BODY_TO_TASK
    return Waypoint(name, T, mode, tolerances)


def plan_task(task, template, gate_to_world, robot_position):
    """Lock entry side once; derive semantic lateral anchors from the geometry template."""
    gate = transform(gate_to_world)
    # Existing visual template: x right pillar -> left pillar, y down, z normal.
    if np.dot(-gate[:3, 1], [0, 0, 1]) < task.get('min_up_alignment', .85):
        raise ValueError('Gate orientation is inconsistent with upright task model')
    side = task.get('entry_side', 'auto')
    dot = float((np.asarray(robot_position) - gate[:3, 3]) @ gate[:3, 2])
    if side == 'auto':
        if abs(dot) < task.get('min_entry_distance_m', .3):
            raise ValueError('Too close to gate plane to choose entry side')
        sign = 1 if dot < 0 else -1
    elif side in ('negative_model_z', 'positive_model_z'):
        sign = 1 if side == 'negative_model_z' else -1
    else: raise ValueError('Invalid entry_side')
    frame = np.eye(4)
    frame[:3, 3] = gate[:3, 3]
    frame[:3, :3] = gate[:3, :3] @ np.array([[0, -sign, 0], [0, 0, -1], [sign, 0, 0]])
    parts = template['parts']
    short = parts['MiddleRedPillar']
    height = -(short['center'][1] + short['size'][1]/2)
    anchors = {'center': 0., 'sos': -sign * parts['FlagSOS']['center'][0],
               'repair': -sign * parts['FlagRepair']['center'][0]}
    points = []
    for item in task['waypoints']:
        pos = list(item['position_m'])
        if item.get('lateral_anchor'): pos[1] += anchors[item['lateral_anchor']]
        if item.get('height_anchor') == 'middle_bottom': pos[2] += height
        elif item.get('height_anchor') not in (None, 'beam'): raise ValueError('Invalid height_anchor')
        points.append(make_waypoint(item['name'], frame, pos, item.get('rpy_deg', [0, 0, 0]),
                                    item.get('mode', 'settle'), **item.get('arrival', {})))
    if not points: raise ValueError('Task requires at least one waypoint')
    return frame, points


# Other tasks register a geometric planner without replacing 2D acquisition/control.
PLANNERS={'gate_pass':plan_task}


class Arrival:
    """Separate translation/SO(3) gates, hysteresis, dwell and velocity limits."""
    def __init__(self, options):
        self.opt = options
        self.since = None

    def check(self, now, state, waypoint, previous=None):
        o = dict(self.opt, **waypoint.tolerances)
        p, R, v, w = state
        delta = p - waypoint.T[:3, 3]
        ep = float(np.linalg.norm(delta))
        er = rotation_error(R, waypoint.T[:3, :3])
        tol = float(o['position_tolerance_m']); angle = math.radians(o['orientation_tolerance_deg'])
        ratio = float(o.get('exit_ratio', 1.4)) if self.since is not None else 1.
        inside = ep <= tol*ratio and er <= angle*ratio
        info = {'position_error_m': ep, 'orientation_error_deg': math.degrees(er)}
        if waypoint.mode == 'pass':
            # Either enter the waypoint region or cross its normal plane in the right corridor.
            crossed = False
            if previous is not None:
                direction = waypoint.T[:3, 2]
                a = float((previous-waypoint.T[:3, 3]) @ direction)
                b = float(delta @ direction)
                if a <= 0 <= b and b-a > 1e-9:
                    hit = previous + (-a/(b-a))*(p-previous)
                    crossed = np.linalg.norm(hit-waypoint.T[:3, 3]) <= o.get('pass_corridor_m', tol)
            return bool((ep <= tol or crossed) and er <= angle), info
        inside = inside and np.linalg.norm(v) <= o['linear_speed_mps'] and np.linalg.norm(w) <= math.radians(o['angular_speed_deg_s'])
        if not inside:
            self.since = None
        elif self.since is None:
            self.since = now
        # At the actual completion instant require strict tolerances, not just hysteresis.
        return bool(self.since is not None and now-self.since >= o['dwell_s'] and ep <= tol and er <= angle), info


class Mission:
    """Inputs use simulation time; ROS adapter separately rejects wall-clock stale data."""
    def __init__(self, config, task, template):
        validate_config(config, task)
        self.cfg, self.task, self.template = config, task, template
        self.phase = 'IDLE'; self.reason = 'not started'
        self.target = None; self.points = []; self.index = 0; self.laps = 0
        self.history = deque(maxlen=20); self.frame = None; self.gate = None
        self.previous = None; self.last_time = None; self.metrics = {}
        self.arrival = Arrival(config['arrival'])
        self.visual = VisualApproach(config['visual_approach'], BODY_TO_TASK) if config.get('visual_approach',{}).get('enabled',False) else None

    def begin(self, now, state):
        self.phase = 'SEARCH'; self.reason = 'searching for stable gate'
        self.history.clear(); self.points = []; self.index = 0; self.laps = 0
        self.target = transform(state); self.previous = state[:3, 3].copy()
        self.last_time = now; self.phase_start = now
        self.search_elapsed=0.
        forward = state[:3, 2]
        self.heading = math.atan2(forward[1], forward[0])
        if self.visual:self.visual.reset(now,state)

    def hold(self, state, reason, phase='HOLD'):
        self.target = transform(state); self.phase = phase; self.reason = reason
        self.arrival.since = None

    def observe(self, record, now):
        if self.phase not in ('SEARCH', 'CONFIRM', 'ALIGN_2D', 'APPROACH_2D', 'WAIT_2D', 'WAIT_3D'): return
        quality = record.get('quality', {})
        stamp = float(record.get('last_accepted', -1e9))
        if not quality.get('accepted', False) or not -self.cfg['acquisition'].get('max_future_s', .05) <= now-stamp <= self.cfg['acquisition']['max_age_s']: return
        gate = transform(record['T'])
        if self.history and stamp <= self.history[-1][0]: return
        if self.history:
            prev = self.history[-1][1]
            if (np.linalg.norm(gate[:3, 3]-prev[:3, 3]) > self.cfg['acquisition']['position_spread_m']
                or rotation_error(gate[:3, :3], prev[:3, :3]) > math.radians(self.cfg['acquisition']['angle_spread_deg'])):
                self.history.clear()
        self.history.append((stamp, gate))

    def observe_2d(self, target, now):
        if self.visual and self.phase in ('SEARCH','CONFIRM','ALIGN_2D','APPROACH_2D','WAIT_2D','WAIT_3D'):
            self.visual.observe(target,now)

    def set_waypoint(self, waypoint, now):
        """Single entry point for changing the frozen PID destination."""
        self.active = waypoint; self.target = waypoint.T.copy()
        self.arrival = Arrival(self.cfg['arrival']); self.waypoint_start = now
        self.best = float('inf'); self.progress_time = now; self.reason = waypoint.name
        self.previous = None

    def step(self, now, T, linear_speed, angular_speed):
        T = transform(T)
        if self.last_time is not None and now < self.last_time-1e-3:
            self.history.clear(); self.points = []
            self.hold(T, 'simulation time reset; explicit restart required', 'IDLE')
        dt = max(0., min(.2, now-(self.last_time if self.last_time is not None else now)))
        self.last_time = now
        p, R = T[:3, 3], T[:3, :3]
        if self.phase in ('SEARCH', 'CONFIRM', 'ALIGN_2D', 'APPROACH_2D', 'WAIT_2D', 'WAIT_3D'):
            acq = self.cfg['acquisition']
            while self.history and now-self.history[0][0] > acq['window_s']: self.history.popleft()
            fresh = self.history and now-self.history[-1][0] <= acq['max_age_s']
            if fresh:
                if self.phase!='CONFIRM':self.target=T.copy()
                self.phase = 'CONFIRM'
                self.reason = 'confirming stable 3D target'
            else:
                visual_goal=self.visual.step(now,T) if self.visual else None
                if visual_goal is not None:
                    self.target=visual_goal; self.phase=self.visual.phase
                    self.reason=self.visual.reason
                else:
                    actual = math.atan2(R[1, 2], R[0, 2])
                    if self.phase!='SEARCH':self.target=T.copy();self.heading=actual
                    self.phase = 'SEARCH'; self.reason='searching for 2D or stable 3D target'
                    self.heading += math.radians(self.cfg['search']['yaw_rate_deg_s'])*dt
                    lag = math.atan2(math.sin(self.heading-actual), math.cos(self.heading-actual))
                    limit = math.radians(self.cfg['search']['max_yaw_lead_deg'])
                    self.heading = actual + np.clip(lag, -limit, limit)
                    self.target[:3, :3] = Rotation.from_euler('z', self.heading).as_matrix() @ BODY_TO_TASK
            self.search_elapsed=self.search_elapsed+dt if self.phase=='SEARCH' else 0.
            if self.phase=='SEARCH' and self.search_elapsed > self.cfg['search']['timeout_s']:
                self.hold(T, 'gate search timed out', 'FAILED')
            elif fresh and len(self.history) >= acq['samples'] and self.history[-1][0]-self.history[0][0] >= acq['stable_s']:
                centers = np.array([g[:3, 3] for _, g in self.history])
                mean = centers.mean(axis=0)
                rotation = Rotation.from_matrix(np.array([g[:3, :3] for _, g in self.history])).mean().as_matrix()
                spread = np.max(np.linalg.norm(centers-mean, axis=1))
                angles = max(rotation_error(rotation, g[:3, :3]) for _, g in self.history)
                if spread <= acq['position_spread_m'] and angles <= math.radians(acq['angle_spread_deg']):
                    gate = np.eye(4); gate[:3, 3] = mean; gate[:3, :3] = rotation
                    try: self.frame, self.points = PLANNERS[self.task['planner']](self.task, self.template, gate, p)
                    except ValueError as e:
                        self.hold(T, str(e), 'FAILED'); return self.target
                    self.gate = gate; self.index = 0; self.phase = 'EXECUTE'
                    self.set_waypoint(self.points[0], now)
        elif self.phase == 'EXECUTE':
            done, self.metrics = self.arrival.check(now, (p, R, linear_speed, angular_speed), self.active, self.previous)
            score = self.metrics['position_error_m'] + self.cfg['progress']['angle_weight_m_per_rad'] * math.radians(self.metrics['orientation_error_deg'])
            if score < self.best-self.cfg['progress']['min_improvement_m']:
                self.best = score; self.progress_time = now
            if done:
                self.index += 1
                if self.index < len(self.points): self.set_waypoint(self.points[self.index], now)
                else:
                    self.laps += 1
                    if self.task.get('repeat', True) and (self.task.get('max_laps', 0) == 0 or self.laps < self.task['max_laps']):
                        self.index = 0; self.set_waypoint(self.points[0], now)
                    else: self.hold(T, 'task complete', 'DONE')
            elif now-self.waypoint_start > self.cfg['progress']['waypoint_timeout_s']:
                self.hold(T, 'waypoint timeout: '+self.active.name, 'FAILED')
            elif now-self.progress_time > self.cfg['progress']['no_progress_s']:
                self.hold(T, 'no progress: '+self.active.name, 'FAILED')
            self.previous = p.copy()
        return self.target

    def status(self):
        return dict(phase=self.phase, reason=self.reason, index=self.index, laps=self.laps,
                    waypoint=self.points[self.index].name if self.points and self.index < len(self.points) else None,
                    **(self.visual.metrics if self.visual else {}),
                    **self.metrics)
