"""The grid guard checks the command lifetime, including curved motion."""
import math
from pathlib import Path

import pytest
import yaml

from stair_nav.core.config import load_route, segment_config
from stair_nav.core.engine import State
from stair_nav.core.models import Pose, Waypoint
from stair_nav.core.planning import GridAStarBackend, OccupancyGrid
from stair_nav.core.tracking import TrackingOutput
from stair_nav.demo import Harness


def grid_with_obstacle():
    rows = [[0] * 20 for _ in range(20)]
    rows[5][5] = 100
    return OccupancyGrid.from_mapping(dict(
        width=20, height=20, resolution=.1, origin=[0., 0.],
        height_z=0., frame='map', platform_id='test', data=rows))


def test_velocity_safe_for_one_tick_can_be_unsafe_before_ttl_expires():
    grid = grid_with_obstacle()
    pose = Pose(.2, .55, 0., 0.)
    assert grid.motion_is_free(pose, 1., 0., .05)
    assert not grid.motion_is_free(pose, 1., 0., .5)


def test_curved_sweep_hits_obstacle_even_when_start_to_endpoint_chord_is_free():
    grid = grid_with_obstacle()
    pose = Pose(.3, .3, 0., 0.)
    # A half circle centred at (.3, .6) passes through the occupied square;
    # checking only the vertical chord between its endpoints misses it.
    assert grid.line_is_free((.3, .3), (.3, .9))
    assert not grid.motion_is_free(pose, .3 * math.pi, math.pi, 1.)


def test_reverse_motion_and_map_boundary_are_checked():
    grid = grid_with_obstacle()
    assert not grid.motion_is_free(Pose(.9, .55, 0., 0.), -1., 0., .5)
    assert not grid.motion_is_free(Pose(.1, .2, 0., math.pi), 1., 0., .2)


def test_tiny_nonzero_velocity_is_not_rotation_when_the_lease_is_long():
    # A tiny velocity still traverses an obstacle over a sufficiently long
    # finite command lifetime; a fixed speed epsilon must not skip its sweep.
    assert not grid_with_obstacle().motion_is_free(Pose(.2, .55, 0., 0.), 1e-12, 0., 1e12)


def test_excessive_sweep_horizon_is_rejected_with_a_bounded_check_budget():
    # The tiny circular orbit fits in free space, but checking an arbitrarily
    # long lease must fail closed rather than monopolize the control callback.
    assert not grid_with_obstacle().motion_is_free(Pose(.3, .3, 0., 0.), .001, 1., 1e6)


def test_rotation_cannot_authorize_motion_from_an_occupied_start():
    grid = grid_with_obstacle()
    assert grid.motion_is_free(Pose(.3, .3, 0., 0.), 0., 1., .5)
    assert not grid.motion_is_free(Pose(.55, .55, 0., 0.), 0., 1., .5)
    assert not grid.motion_is_free(Pose(.55, .55, 0., 0.), 0., 0., 0.)


@pytest.mark.parametrize('vx,wz,duration', [
    (math.nan, 0., .2), (0., math.inf, .2), (0., 0., math.nan),
    (0., 0., math.inf), (0., 0., -.1),
])
def test_invalid_twist_or_horizon_is_never_authorized(vx, wz, duration):
    assert not grid_with_obstacle().motion_is_free(Pose(.3, .3, 0., 0.), vx, wz, duration)


def test_planned_grid_path_carries_the_inflated_map_for_command_checks():
    route = load_route(Path(__file__).resolve().parents[1] / 'config' / 'demo_route.yaml')
    segment = segment_config(route, 'APPROACH_A')
    path = GridAStarBackend().plan(route, segment, Pose(-1., .4, 0., .6))
    document = yaml.safe_load((Path(route['_base_dir']) / segment['grid_file']).read_text())
    inflated = OccupancyGrid.from_mapping(document, segment['planning'])
    raw = OccupancyGrid.from_mapping(document)
    assert path.collision_grid.blocked == inflated.blocked
    assert raw.blocked < path.collision_grid.blocked
    assert path.collision_grid.platform_id == segment['platform_id']


def test_executor_rejects_unsafe_tracker_output_before_publishing_velocity():
    route = load_route(Path(__file__).resolve().parents[1] / 'config' / 'demo_route.yaml')
    route['simulation']['initial_pose'] = [-1.95, .5, 0., 0.]
    harness = Harness(route)
    try:
        harness.start('APPROACH_A')
        for _ in range(30):
            output = harness.step()
            if output.status.state == State.EXECUTING:
                break
        assert output.status.state == State.EXECUTING
        pose = harness.sim.pose
        grid = harness.engine.path.collision_grid
        # This deliberately broken tracker output is safe for the next tick,
        # but the adapter could hold it until the command lease expires.
        assert grid.motion_is_free(pose, .35, 0., harness.dt)
        assert not grid.motion_is_free(pose, .35, 0., route['command_ttl'])
        harness.engine._tracker.update = lambda *_: TrackingOutput(
            .35, 0., .1, 0., 0., Waypoint(0., 0., 0.), 'platform', False)
        output = harness.step()
        assert output.status.state == State.FAILED
        assert output.status.error_code == 'TRACKING_COLLISION'
        assert output.status.navigation_stopped
        assert not output.command.valid
        assert output.command.behavior == 'HOLD'
        assert output.command.vx == output.command.wz == 0.
        assert (harness.sim.pose.x, harness.sim.pose.y) == (pose.x, pose.y)
    finally:
        harness.engine.close()


@pytest.mark.parametrize('initial', [
    [5.9, -.03, 2., -.15],
    [5.92, .03, 2., .15],
    [6., 0., 2., 0.],
])
def test_grid_endpoint_join_reaches_exact_goal_without_terminal_backtracking(initial):
    route = load_route(Path(__file__).resolve().parents[1] / 'config' / 'demo_route.yaml')
    route['simulation']['initial_pose'] = initial
    segment = segment_config(route, 'CD')
    document = yaml.safe_load((Path(route['_base_dir']) / segment['grid_file']).read_text())
    grid = OccupancyGrid.from_mapping(document, segment['planning'])
    harness = Harness(route)
    try:
        harness.start('CD')
        output = harness.until_terminal()
        assert output.status.state == State.SUCCEEDED, output.status
        assert output.status.hold_confirmed and output.status.geometry_reached
        points = [tuple(initial[:3]), *harness.trajectory]
        assert all(grid.line_is_free(a[:2], b[:2]) for a, b in zip(points, points[1:]))
        goal = route['nodes'][segment['end']]
        assert math.hypot(harness.sim.pose.x - goal['position'][0],
                          harness.sim.pose.y - goal['position'][1]) <= goal['arrival']['xy']
    finally:
        harness.engine.close()
