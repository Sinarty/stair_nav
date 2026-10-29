"""A* searches supported stair cells and the simulator samples their terrain."""
from copy import deepcopy
from dataclasses import replace
import math
from pathlib import Path
from threading import Event

import pytest
import yaml

from stair_nav.core.config import load_route, segment_config, validate_route
from stair_nav.core.models import Command, NavError, Pose
from stair_nav.core.planning import HeightGridAStarBackend, GridAStarBackend, OccupancyGrid
from stair_nav.demo.simulation import Simulator
from stair_nav.demo import Harness


@pytest.fixture
def surface(tmp_path):
    width, height = 9, 13
    rows = [[100 if x in (0, width-1) or y in (0, height-1) else 0
             for x in range(width)] for y in range(height)]
    levels = [min(.6, max(0, (y-2)//3)*.2) for y in range(height)]
    document = dict(frame='map', platform_id='stairs', height_z=0., width=width,
                    height=height, resolution=.1, origin=[0., 0.], data=rows,
                    heights=[[levels[y]]*width for y in range(height)],
                    phases=[['stair']*width for _ in range(height)])
    filename = tmp_path/'stairs.yaml'
    filename.write_text(yaml.safe_dump(document))
    route = dict(_base_dir=str(tmp_path), frame='map', nodes={
        'B': dict(position=[.45, 1.15, .6], yaw=math.pi/2)})
    segment = dict(kind='stair', backend='height_grid_astar', grid_file='stairs.yaml',
                   platform_id='stairs', start='A', end='B', mode='stair',
                   planning=dict(max_step_height=.2, inflation_radius=0.))
    return route, segment, Pose(.45, .15, 0., math.pi/2), document, filename


def test_height_astar_preserves_steps_and_straight_stair_direction(surface):
    route, segment, pose, _, _ = surface
    path = HeightGridAStarBackend().plan(route, segment, pose)
    assert path.collision_grid.heights is not None
    assert path.points[0].z == 0. and path.points[-1].z == pytest.approx(.6)
    assert all(point.x == pytest.approx(.45) for point in path.points)
    assert all(point.mode == 'stair' for point in path.points)
    assert all(b.z >= a.z-1e-9 for a, b in zip(path.points, path.points[1:]))
    assert all(abs(b.z-a.z) <= .2+1e-9 for a, b in zip(path.points, path.points[1:]))
    assert all(path.collision_grid.line_is_free((a.x, a.y), (b.x, b.y))
               for a, b in zip(path.points, path.points[1:]))


@pytest.mark.parametrize('changes,code', [
    (dict(z=1.), 'WRONG_PLATFORM_HEIGHT'),
    (dict(frame='odom'), 'FRAME_MISMATCH'),
    (dict(x=-1.), 'INVALID_START'),
])
def test_height_astar_rejects_wrong_start(surface, changes, code):
    route, segment, pose, _, _ = surface
    with pytest.raises(NavError) as caught:
        HeightGridAStarBackend().plan(route, segment, replace(pose, **changes))
    assert caught.value.code == code


def test_height_astar_rejects_wrong_goal_height(surface):
    route, segment, pose, _, _ = surface
    route['nodes']['B']['position'][2] = 2.
    with pytest.raises(NavError) as caught:
        HeightGridAStarBackend().plan(route, segment, pose)
    assert caught.value.code == 'WRONG_PLATFORM_HEIGHT'


def test_height_astar_cannot_jump_a_tall_riser(surface):
    route, segment, pose, document, filename = surface
    for y in range(6, document['height']):
        document['heights'][y] = [1.]*document['width']
    route['nodes']['B']['position'][2] = 1.
    filename.write_text(yaml.safe_dump(document))
    with pytest.raises(NavError) as caught:
        HeightGridAStarBackend().plan(route, segment, pose)
    assert caught.value.code == 'NO_PATH'
    grid = OccupancyGrid.from_mapping(document, segment['planning'])
    assert not grid.line_is_free((.45, .45), (.45, .85))
    assert not grid.motion_is_free(Pose(.45, .45, 0., math.pi/2), .4, 0., 1.)


def test_height_astar_does_not_climb_diagonally(surface):
    route, segment, pose, _, _ = surface
    route['nodes']['B']['position'][0] = .55
    path = HeightGridAStarBackend().plan(route, segment, pose)
    for a, b in zip(path.points, path.points[1:]):
        if abs(b.z-a.z) > 1e-9:
            assert abs(b.x-a.x) < 1e-9 or abs(b.y-a.y) < 1e-9


def test_height_astar_obeys_closed_corridor_and_cancellation(surface):
    route, segment, pose, document, filename = surface
    document['data'][6] = [100]*document['width']
    filename.write_text(yaml.safe_dump(document))
    with pytest.raises(NavError) as caught:
        HeightGridAStarBackend().plan(route, segment, pose)
    assert caught.value.code == 'NO_PATH'
    cancel = Event()
    cancel.set()
    with pytest.raises(NavError) as caught:
        HeightGridAStarBackend().plan(route, segment, pose, cancel)
    assert caught.value.code == 'PLANNING_CANCELLED'


@pytest.mark.parametrize('matrix', [None, [], [[0.]], [[math.nan]*9]*13, [[True]*9]*13])
def test_height_grid_requires_finite_grid_sized_elevations(surface, matrix):
    route, segment, pose, document, filename = surface
    if matrix is None:
        document.pop('heights')
    else:
        document['heights'] = matrix
    filename.write_text(yaml.safe_dump(document))
    with pytest.raises(NavError) as caught:
        HeightGridAStarBackend().plan(route, segment, pose)
    assert caught.value.code == 'INVALID_GRID'


def test_flat_backend_cannot_silently_flatten_stair_heights(surface):
    route, segment, pose, _, _ = surface
    with pytest.raises(NavError) as caught:
        GridAStarBackend().plan(route, segment, pose)
    assert caught.value.code == 'INVALID_GRID'


def test_new_staircase_route_uses_supported_height_maps_and_rejects_invalid_step_setting():
    route = load_route(Path(__file__).resolve().parents[1]/'stair_nav/demo/config/route.yaml')
    for sid in ('AB', 'CD'):
        assert segment_config(route, sid)['backend'] == 'height_grid_astar'
    invalid = deepcopy(route)
    invalid['segments']['AB']['planning'] = dict(max_step_height=0.)
    with pytest.raises(NavError, match='max_step_height'):
        validate_route(invalid)


def test_stair_hold_does_not_pull_an_initially_wrong_height_onto_the_surface():
    route = load_route(Path(__file__).resolve().parents[1]/'stair_nav/demo/config/route.yaml')
    start = route['nodes']['A']['position']
    sim = Simulator(route, initial_pose=[*start[:2], 3., math.pi/2])
    command = Command(0., .2, 'task', 'run', 'AB', 'checking', 'stair', 'HOLD')
    assert sim.accept(command)
    for index in range(1, 14):
        sim.step(.05, index*.05)
    assert sim.pose.z == 3.


def test_staircase_height_grids_close_the_production_control_loop():
    route = load_route(Path(__file__).resolve().parents[1]/'stair_nav/demo/config/route.yaml')
    harness = Harness(route)
    previous_height = harness.sim.pose.z
    try:
        for sid in ('APPROACH_A', 'AB', 'BC', 'CD'):
            assert harness.start(sid)
            while harness.engine.active:
                output = harness.step()
                assert harness.sim.pose.z >= previous_height-1e-9
                assert harness.sim.pose.z-previous_height <= .2+1e-9
                previous_height = harness.sim.pose.z
            assert output.status.state == 'SUCCEEDED', output.status
            assert output.status.hold_confirmed
        assert harness.sim.pose.z == pytest.approx(2.4)
        assert math.hypot(harness.sim.pose.x-2., harness.sim.pose.y-6.5) <= .12
    finally:
        harness.engine.close()
