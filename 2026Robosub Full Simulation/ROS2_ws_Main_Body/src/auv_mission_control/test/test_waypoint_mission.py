"""Numerical regression tests for the nonstandard body/model axes and task sequencing."""
from pathlib import Path
from copy import deepcopy
import math
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from auv_mission_control.waypoint_mission import (
    Arrival, Mission, BODY_TO_TASK, make_waypoint, plan_task, rotation_error,
)

ROOT = Path(__file__).resolve().parents[1]
CFG = yaml.safe_load((ROOT/'config/autonomy.yaml').read_text())
TASK = yaml.safe_load((ROOT/'config/tasks/task1_gate.yaml').read_text())
# Independent measured geometry fixture: model x lateral, y down, z normal.
TEMPLATE = {'parts': {
    'MiddleRedPillar': {'center': [.04570, .33305, .00203], 'size': [.05079, .60960, .00406]},
    'FlagSOS': {'center': [-.79116, .17610, 0.]},
    'FlagRepair': {'center': [.75496, .17610, -.002]},
}}


def gate_pose():
    # Model x world+Y, y world-Z, z world-X. Translation deliberately nonzero.
    T = np.eye(4); T[:3, :3] = [[0, 0, -1], [1, 0, 0], [0, -1, 0]]
    T[:3, 3] = [10, 4, 3]
    return T


def activated():
    mission = Mission(deepcopy(CFG), deepcopy(TASK), TEMPLATE)
    G = gate_pose(); robot = np.eye(4); robot[:3, :3] = BODY_TO_TASK
    robot[:3, 3] = [7, 4, 2]
    mission.begin(0., robot)
    for t in (.1, .7, 1.3):
        mission.observe({'T': G.tolist(), 'last_accepted': t, 'quality': {'accepted': True}}, t)
        mission.step(t, robot, np.zeros(3), np.zeros(3))
    assert mission.phase == 'EXECUTE'
    return mission, robot


@pytest.mark.parametrize('robot_x,expected_forward', [(7., [1, 0, 0]), (13., [-1, 0, 0])])
def test_semantic_path_on_both_entry_sides(robot_x, expected_forward):
    G = gate_pose()
    frame, points = plan_task(TASK, TEMPLATE, G, [robot_x, 4, 2])
    assert len(points) == 11
    assert np.linalg.det(frame[:3, :3]) == pytest.approx(1)
    assert np.allclose(points[0].T[:3, 2], expected_forward)
    assert points[0].T[2, 3] == pytest.approx(3-.63785)
    # SOS lies on world Y=4-.79116 regardless of approach side.
    assert points[1].T[1, 3] == pytest.approx(4-.79116)
    assert points[6].T[1, 3] == pytest.approx(4+.75496)
    assert np.allclose(points[4].T[:3, 3], points[5].T[:3, 3])
    assert rotation_error(points[4].T[:3, :3], points[5].T[:3, :3]) == pytest.approx(math.pi)
    assert np.allclose(points[-1].T, points[0].T)
    for wp in points:
        assert np.allclose(wp.T[:3, 1], [0, 0, 1])  # AUV +Y is up, not conventional +Z.


def test_arbitrary_world_rotation_translation():
    G = gate_pose(); W = np.eye(4); W[:3, :3] = Rotation.from_euler('z', 39, degrees=True).as_matrix()
    W[:3, 3] = [-8, 3, 2]
    p = np.array([7, 4, 2]); _, a = plan_task(TASK, TEMPLATE, G, p)
    _, b = plan_task(TASK, TEMPLATE, W@G, W[:3, :3]@p+W[:3, 3])
    for x, y in zip(a, b): assert np.allclose(y.T, W@x.T)


def test_so3_wrap_dwell_velocity_hysteresis():
    waypoint = make_waypoint('test', np.eye(4), [0, 0, 0], [0, 0, 179])
    actual = make_waypoint('actual', np.eye(4), [0, 0, 0], [0, 0, -179])
    a = Arrival(CFG['arrival'])
    state = (np.zeros(3), actual.T[:3, :3], np.zeros(3), np.zeros(3))
    assert not a.check(0, state, waypoint)[0]
    assert a.check(.6, state, waypoint)[0]
    fast = (*state[:2], np.array([.4, 0, 0]), state[3])
    assert not a.check(.7, fast, waypoint)[0]
    assert a.since is None
    assert not a.check(.8, state, waypoint)[0]
    # Hysteresis retains dwell for small excursions but strict error at completion.
    edge = (np.array([.20, 0, 0]), *state[1:])
    assert not a.check(1.4, edge, waypoint)[0]
    assert a.check(1.5, state, waypoint)[0]


def test_pass_plane_requires_correct_direction_and_corridor():
    wp = make_waypoint('pass', np.eye(4), [0, 0, 0], mode='pass')
    a = Arrival(CFG['arrival'])
    R = wp.T[:3, :3]
    assert a.check(1, (np.array([.3, .02, 0]), R, np.ones(3), np.zeros(3)), wp, np.array([-.3, .02, 0]))[0]
    assert not a.check(1, (np.array([.3, .4, 0]), R, np.ones(3), np.zeros(3)), wp, np.array([-.3, .4, 0]))[0]
    assert not a.check(1, (np.array([-.3, .02, 0]), R, np.ones(3), np.zeros(3)), wp, np.array([.3, .02, 0]))[0]


def test_gate_requires_distinct_fresh_stable_observations():
    m = Mission(deepcopy(CFG), deepcopy(TASK), TEMPLATE); T = np.eye(4); T[:3, :3] = BODY_TO_TASK
    m.begin(10, T)
    record = {'T': gate_pose().tolist(), 'last_accepted': 10, 'quality': {'accepted': True}}
    for t in (10, 10.5, 11): m.observe(record, t); m.step(t, T, np.zeros(3), np.zeros(3))
    assert m.phase != 'EXECUTE' and len(m.history) == 1
    record['last_accepted'] = 11.5; record['quality']['accepted'] = False
    m.observe(record, 11.5); assert len(m.history) == 1
    m.step(15, T, np.zeros(3), np.zeros(3)); assert m.phase == 'SEARCH'


def test_full_round_order_freeze_and_repeat():
    m, _ = activated(); frozen = [wp.T.copy() for wp in m.points]
    new = gate_pose(); new[0, 3] += 8
    m.observe({'T': new.tolist(), 'last_accepted': 2, 'quality': {'accepted': True}}, 2)
    assert all(np.allclose(a, b.T) for a, b in zip(frozen, m.points))
    now = 2.; names = []
    for _ in range(11):
        wp = m.points[m.index]; names.append(wp.name)
        m.step(now, wp.T, np.zeros(3), np.zeros(3))
        now += .6
        if m.active.name == wp.name: m.step(now, wp.T, np.zeros(3), np.zeros(3))
        now += .2
    assert names == [w['name'] for w in TASK['waypoints']]
    assert m.laps == 1 and m.index == 0 and m.phase == 'EXECUTE'


def test_timeout_and_clock_reset_do_not_skip_waypoints():
    m, state = activated()
    m.step(40., state, np.zeros(3), np.zeros(3))
    m.step(76., state, np.zeros(3), np.zeros(3))
    assert m.phase == 'FAILED' and m.index == 0
    m.step(0., state, np.zeros(3), np.zeros(3))
    assert m.phase == 'IDLE' and not m.points


def test_bad_threshold_and_model_rejected():
    c = deepcopy(CFG); c['arrival']['position_tolerance_m'] = -1
    with pytest.raises(ValueError): Mission(c, TASK, TEMPLATE)
    G = gate_pose(); G[:3, 1] *= -1
    with pytest.raises(ValueError): plan_task(TASK, TEMPLATE, G, [0, 0, 0])


def test_single_waypoint_function_full_pose():
    wp = make_waypoint('custom', np.eye(4), [1, 2, 3], [15, -10, 45])
    assert np.allclose(wp.T[:3, 3], [1, 2, 3])
    assert rotation_error(wp.T[:3, :3], Rotation.from_euler('xyz', [15, -10, 45], degrees=True).as_matrix()@BODY_TO_TASK) < 1e-8
