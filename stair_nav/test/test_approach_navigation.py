"""Approach A through the production planner/tracker and command-driven simulator."""
import math
from pathlib import Path

import pytest
import yaml

from stair_nav.core.config import load_route, segment_config
from stair_nav.core.engine import State
from stair_nav.core.models import NavError, Pose
from stair_nav.core.planning import GridAStarBackend, OccupancyGrid
from stair_nav.core.simulation import Simulator
from stair_nav.demo import Harness


@pytest.fixture
def route():
    return load_route(Path(__file__).resolve().parents[1] / 'config' / 'demo_route.yaml')


def approach_grid(route):
    segment = segment_config(route, 'APPROACH_A')
    document = yaml.safe_load((Path(route['_base_dir']) / segment['grid_file']).read_text())
    return OccupancyGrid.from_mapping(document, segment['planning'])


def assert_trajectory_clear(grid, initial, trajectory):
    points = [tuple(initial[:3]), *trajectory]
    assert all(grid.line_is_free(a[:2], b[:2]) for a, b in zip(points, points[1:]))


def assert_held_command_clear(grid, pose, command):
    """Independently check a lost-update hold using the exact unicycle arc."""
    if not command.valid:
        return
    horizon = command.valid_until - command.stamp
    count = max(1, math.ceil(horizon / .005))
    previous = (pose.x, pose.y)
    for index in range(1, count + 1):
        elapsed = horizon * index / count
        yaw = pose.yaw + command.wz * elapsed
        if abs(command.wz) < 1e-10:
            current = (pose.x + command.vx * elapsed * math.cos(pose.yaw),
                       pose.y + command.vx * elapsed * math.sin(pose.yaw))
        else:
            radius = command.vx / command.wz
            current = (pose.x + radius * (math.sin(yaw) - math.sin(pose.yaw)),
                       pose.y - radius * (math.cos(yaw) - math.cos(pose.yaw)))
        assert grid.line_is_free(previous, current), (pose, command, previous, current)
        previous = current


@pytest.mark.parametrize('initial', [
    [-1.0, 0.4, 0.0, 0.6],
    [-1.8, -0.8, 0.0, math.pi],
    [-0.5, 0.9, 0.0, -1.2],
    [0.0, 0.0, 0.0, math.pi / 2],
])
def test_approach_tracks_from_live_pose_and_confirms_position_heading_hold(route, initial):
    route['simulation']['initial_pose'] = initial
    harness = Harness(route)
    try:
        assert harness.start('APPROACH_A')
        result = harness.until_terminal()
        assert result.status.state == State.SUCCEEDED, result.status
        assert result.status.hold_confirmed and result.status.navigation_stopped
        pose = harness.sim.pose
        goal = route['nodes']['A']
        limits = goal['arrival']
        assert math.hypot(pose.x, pose.y) <= limits['xy']
        assert abs(pose.z) <= limits['z']
        assert abs(pose.yaw) <= limits['yaw']
        assert harness.sim.motion_status(harness.now).stable
        assert harness.command_count > 0
        # The simulated initial position is actually driven to A, including
        # in-place heading correction when initial XY already equals A.
        assert harness.engine.path.points[0].x == initial[0]
        assert harness.engine.path.points[0].y == initial[1]
        assert_trajectory_clear(approach_grid(route), initial, harness.trajectory)
    finally:
        harness.engine.close()


@pytest.mark.parametrize('initial', [
    [-1.9, 0.9, 0.0, -1.0],
    [-2.0, 0.5, 0.0, 0.0],
])
@pytest.mark.parametrize('dt,ttl', [
    (.025, .1), (.05, .1),
    (.025, .2), (.05, .2), (.1, .2),
    (.025, .4), (.05, .4), (.1, .4),
])
def test_approach_obstacle_clearance_covers_motion_and_entire_command_ttl(route, initial, dt, ttl):
    route['simulation']['initial_pose'] = initial
    route['command_ttl'] = ttl
    harness = Harness(route, dt=dt)
    grid = approach_grid(route)
    tracking = segment_config(route, 'APPROACH_A')['tracking']
    try:
        harness.start('APPROACH_A')
        deadline = harness.now + 120.
        previous_command = None
        while harness.engine.active and harness.now < deadline:
            before = harness.sim.pose
            result = harness.step()
            # A publisher may disappear immediately after this output. The
            # adapter can keep the velocity until its TTL, not just for dt.
            assert_held_command_clear(grid, before, result.command)
            if previous_command is not None and previous_command.valid and result.command.valid:
                assert abs(result.command.vx - previous_command.vx) <= tracking['max_accel'] * dt + 1e-9
                assert abs(result.command.wz - previous_command.wz) <= tracking['max_angular_accel'] * dt + 1e-9
            previous_command = result.command
        assert result.status.state == State.SUCCEEDED, result.status
        assert_trajectory_clear(grid, initial, harness.trajectory)
        assert result.status.hold_confirmed and harness.sim.motion_status(harness.now).stable
    finally:
        harness.engine.close()


def test_approach_then_original_route_runs_without_pose_reset(route):
    route['simulation']['initial_pose'] = [-1.0, 0.4, 0.0, 0.6]
    harness = Harness(route)
    try:
        harness.start('APPROACH_A')
        approach = harness.until_terminal()
        assert approach.status.state == State.SUCCEEDED
        at_a = harness.sim.pose
        report = harness.run_sequence()
        assert report['success'], report
        assert [row['segment_id'] for row in report['results']] == ['AB', 'BC', 'CD']
        assert harness.events[0]['segment_id'] == 'APPROACH_A'
        assert math.hypot(at_a.x, at_a.y) <= route['nodes']['A']['arrival']['xy']
        assert abs(harness.sim.pose.x - 9.0) < .15
        assert abs(harness.sim.pose.z - 2.0) < .12
    finally:
        harness.engine.close()


@pytest.mark.parametrize('initial, code', [
    ([-4.0, 0.0, 0.0, 0.0], 'INVALID_START'),
    ([-1.0, 0.0, 1.0, 0.0], 'WRONG_PLATFORM_HEIGHT'),
    ([-1.5, 0.5, 0.0, 0.0], 'INVALID_START'),
])
def test_invalid_approach_start_fails_without_motion_or_floor_change(route, initial, code):
    route['simulation']['initial_pose'] = initial
    harness = Harness(route)
    try:
        harness.start('APPROACH_A')
        result = harness.until_terminal()
        assert result.status.state == State.FAILED
        assert result.status.error_code == code
        for _ in range(12):
            harness.step()
        assert (harness.sim.pose.x, harness.sim.pose.y, harness.sim.pose.z) == tuple(initial[:3])
        assert harness.command_count == 0
    finally:
        harness.engine.close()


def test_approach_uses_obstacle_grid_and_rejects_disconnected_start(route, tmp_path):
    segment = segment_config(route, 'APPROACH_A')
    grid = yaml.safe_load((Path(route['_base_dir']) / segment['grid_file']).read_text())
    # A solid wall divides the approved lower platform into two components.
    for row in grid['data']:
        row[17] = 100
    (tmp_path / 'blocked.yaml').write_text(yaml.safe_dump(grid))
    segment['grid_file'] = str(tmp_path / 'blocked.yaml')
    with pytest.raises(NavError) as caught:
        GridAStarBackend().plan(route, segment, Pose(-1.0, 0.0, 0.0, 0.0))
    assert caught.value.code == 'NO_PATH'


def test_approach_planned_path_respects_lower_landing_obstacle(route):
    segment = segment_config(route, 'APPROACH_A')
    grid = OccupancyGrid.from_mapping(yaml.safe_load(
        (Path(route['_base_dir']) / segment['grid_file']).read_text()), segment['planning'])
    path = GridAStarBackend().plan(route, segment, Pose(-2.0, 0.5, 0.0, 0.0))
    assert len(path.points) > 2
    assert all(grid.line_is_free((a.x, a.y), (b.x, b.y))
               for a, b in zip(path.points, path.points[1:]))
    # The grid ends before the stair incline; a stair position cannot be used
    # as an unvalidated flat-floor approach start.
    with pytest.raises(NavError) as caught:
        GridAStarBackend().plan(route, segment, Pose(1.0, 0.0, 0.0, 0.0))
    assert caught.value.code == 'INVALID_START'


@pytest.mark.parametrize('initial', [[], [1, 2, 3], [0, 0, 0, 0, 0],
                                      [0, 0, math.nan, 0], [0, math.inf, 0, 0],
                                      [0, 0, 0, True], ['0', 0, 0, 0], 'invalid'])
def test_simulator_initial_pose_is_strictly_validated(route, initial):
    route['simulation']['initial_pose'] = initial
    with pytest.raises(NavError) as caught:
        Simulator(route)
    assert caught.value.code == 'INVALID_INITIAL_POSE'


def test_simulator_explicit_initial_pose_overrides_configuration(route):
    route['simulation']['initial_pose'] = [-1.0, 0.4, 0.0, 0.6]
    sim = Simulator(route, initial_pose=[-0.5, -0.4, 0.0, -0.2])
    assert sim.pose == Pose(-0.5, -0.4, 0.0, -0.2, 0.0, 'map')
