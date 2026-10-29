"""The illustrated O→A→B→C→D scene uses grid planning and real command integration."""
import math
from pathlib import Path

import pytest
import yaml

from stair_nav.core.config import load_route, segment_config
from stair_nav.core.engine import State
from stair_nav.core.mission import MissionCore, MissionState, Readiness, load_mission
from stair_nav.core.models import wrap_angle
from stair_nav.core.planning import OccupancyGrid
from stair_nav.demo import Harness


@pytest.fixture
def scene():
    config = Path(__file__).resolve().parents[1] / 'stair_nav' / 'demo' / 'config'
    route = load_route(config / 'route.yaml')
    mission = load_mission(config / 'mission.yaml', route)
    return route, mission


def grid_for(route, sid):
    segment = segment_config(route, sid)
    mapping = yaml.safe_load((Path(route['_base_dir']) / segment['grid_file']).read_text())
    return OccupancyGrid.from_mapping(mapping, segment['planning'])


def at_start(route, pose):
    start = route['nodes']['A']
    limits = start['arrival']
    return (math.hypot(pose.x - start['position'][0], pose.y - start['position'][1]) <= limits['xy']
            and abs(pose.z - start['position'][2]) <= limits['z']
            and abs(wrap_angle(pose.yaw - start['yaw'])) <= limits['yaw'])


def test_scene_shape_and_stair_entry_alignment_are_explicit(scene):
    route, mission = scene
    nodes = route['nodes']
    assert nodes['O']['position'] == [0., 0., 0.]
    assert route['simulation']['initial_node'] == 'O'
    assert nodes['A']['position'][0] < nodes['O']['position'][0]
    assert nodes['A']['position'][1] < nodes['O']['position'][1]
    assert nodes['A']['position'][0] == nodes['B']['position'][0]
    assert nodes['B']['position'][1] == nodes['C']['position'][1]
    assert nodes['C']['position'][0] == nodes['D']['position'][0]
    assert nodes['A']['position'][2] < nodes['B']['position'][2]
    assert nodes['B']['position'][2] == nodes['C']['position'][2]
    assert nodes['C']['position'][2] < nodes['D']['position'][2]
    assert mission['approach_segment'] == route['approach_segment'] == 'APPROACH_A'
    for sid in ('AB', 'CD'):
        segment = segment_config(route, sid)
        assert segment['backend'] == 'height_grid_astar'
        assert segment['kind'] == 'stair' and not segment['tracking']['allow_turn_in_place']
        entry = nodes[segment['start']]
        assert entry['arrival']['xy'] == .04
        assert entry['arrival']['yaw'] == .05
        assert entry['yaw'] == pytest.approx(math.pi / 2)
    assert segment_config(route, 'BC')['backend'] == 'grid_astar'
    assert segment_config(route, 'BC')['mode'] == 'flat'


def test_complete_scene_reaches_every_stage_without_reset_or_descending(scene):
    route, _ = scene
    harness = Harness(route)
    try:
        for sid in ('APPROACH_A', 'AB', 'BC', 'CD'):
            first = harness.sim.pose
            index = len(harness.trajectory)
            assert harness.start(sid)
            output = harness.until_terminal()
            assert output.status.state == State.SUCCEEDED, output.status
            assert output.status.hold_confirmed and output.status.navigation_stopped
            segment = segment_config(route, sid)
            target = route['nodes'][segment['end']]
            arrival = target['arrival']
            last = harness.sim.pose
            assert math.hypot(last.x - target['position'][0], last.y - target['position'][1]) <= arrival['xy']
            assert abs(last.z - target['position'][2]) <= arrival['z']
            assert abs(wrap_angle(last.yaw - target['yaw'])) <= arrival['yaw']
            samples = [(first.x, first.y, first.z), *harness.trajectory[index:]]
            grid = grid_for(route, sid)
            assert all(grid.line_is_free(a[:2], b[:2]) for a, b in zip(samples, samples[1:]))
            assert all(b[2] >= a[2] - 1e-9 for a, b in zip(samples, samples[1:]))
            if sid in ('AB', 'CD'):
                axis_x = target['position'][0]
                assert max(abs(x - axis_x) for x, y, z in samples) < .08
                assert all(b[1] >= a[1] - 1e-9 for a, b in zip(samples, samples[1:]))
                assert last.z - first.z == pytest.approx(1.2)
            else:
                assert all(z == pytest.approx(first.z) for x, y, z in samples)
        assert harness.sim.pose.z == pytest.approx(2.4)
        assert harness.now < 80.
    finally:
        harness.engine.close()


def test_headless_default_sequence_includes_the_origin_approach(scene):
    route, _ = scene
    harness = Harness(route)
    try:
        report = harness.run_sequence()
        assert report['success'], report
        assert report['approach_result']['segment_id'] == 'APPROACH_A'
        assert report['approach_result']['state'] == State.SUCCEEDED
        assert [row['segment_id'] for row in report['results']] == ['AB', 'BC', 'CD']
        assert report['final_pose']['z'] == pytest.approx(2.4)
    finally:
        harness.engine.close()


def test_mission_and_grid_executor_run_all_four_stages_from_O(scene):
    route, config = scene
    harness = Harness(route)
    core = MissionCore(route, config)
    result_ids = set()
    sent, state_events = [], []
    try:
        assert core.start(config['route_id'], 'scene-task', harness.now)
        while core.active and harness.now < 90.:
            motion = harness.sim.motion_status(harness.now)
            idle = not harness.engine.active
            fresh_release = bool(idle and motion.stable and
                                 (not harness.engine.status.execution_id or motion.hold_confirmed))
            facts = Readiness(ready=fresh_release, released=fresh_release,
                              execution_id=harness.engine.status.execution_id,
                              stamp=harness.now, server_instance='scene-simulator',
                              target_segment_id=core.status.segment_id,
                              at_start=at_start(route, harness.sim.pose))
            effects = core.tick(harness.now, facts, server_ready=True)
            for effect in effects:
                if effect.kind == 'TRANSITION':
                    state_events.append(effect.snapshot.state)
                if effect.kind == 'SEND':
                    sent.append(effect.segment_id)
                    harness.engine.update_pose(harness.sim.pose)
                    assert harness.engine.start(effect.task_id, effect.execution_id,
                                                effect.segment_id, harness.now)
                    core.post('ACCEPTED', effect.task_id, effect.execution_id, accepted=True)
            output = harness.step()
            status = output.status
            if status.execution_id:
                core.post('FEEDBACK', status.task_id, status.execution_id,
                          stamp=harness.now, segment_state=status.state,
                          phase=status.phase, progress=status.progress)
                if not harness.engine.active and status.execution_id not in result_ids:
                    result_ids.add(status.execution_id)
                    core.post('RESULT', status.task_id, status.execution_id,
                              status=4 if status.state == State.SUCCEEDED else 6,
                              success=status.state == State.SUCCEEDED,
                              navigation_stopped=status.navigation_stopped,
                              hold_confirmed=status.hold_confirmed,
                              error_code=status.error_code, error_message=status.error_message)
        assert core.status.state == MissionState.SUCCEEDED, core.status
        assert sent == ['APPROACH_A', 'AB', 'BC', 'CD']
        assert MissionState.APPROACHING_START in state_events
        assert core.status.completed_segments == core.status.segment_count == 3
        assert harness.sim.pose.z == pytest.approx(2.4)
    finally:
        harness.engine.close()
