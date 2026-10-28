import math
import threading

import pytest
import yaml

from stair_nav.core.models import NavError, PlannedPath, Pose, Waypoint
from stair_nav.core.planning import GridAStarBackend, OccupancyGrid, PresetPathBackend
from stair_nav.core.tracking import PurePursuit


def grid_setup(tmp_path, rows, start=(.5, .5), goal=(6.5, 5.5), **options):
    document = dict(width=len(rows[0]), height=len(rows), resolution=1.0,
                    origin=[0.0, 0.0], height_z=2., frame='map', platform_id='floor2', data=rows)
    (tmp_path/'grid.yaml').write_text(yaml.safe_dump(document))
    route = {'_base_dir': str(tmp_path), 'frame': 'map', 'nodes': {'D': {'position': [*goal, 2.]}}}
    segment = {'backend': 'grid_astar', 'platform_id': 'floor2', 'grid_file': 'grid.yaml',
               'end': 'D', 'mode': 'flat', 'planning': options}
    return route, segment, Pose(*start, 2., 0), document


def test_astar_detours_and_simplified_path_is_collision_free(tmp_path):
    rows = [[0]*8 for _ in range(8)]
    for y in range(6):
        rows[y][3] = 100
    route, segment, pose, document = grid_setup(tmp_path, rows, simplify=True)
    path = GridAStarBackend().plan(route, segment, pose)
    grid = OccupancyGrid.from_mapping(document)
    assert max(point.y for point in path.points) >= 6.5
    assert path.points[0].x == pose.x
    assert (path.points[-1].x, path.points[-1].y) == (6.5, 5.5)
    assert all(grid.line_is_free((a.x, a.y), (b.x, b.y)) for a, b in zip(path.points, path.points[1:]))


def test_astar_unreachable_target(tmp_path):
    rows = [[0]*8 for _ in range(8)]
    for y in range(8):
        rows[y][3] = 100
    route, segment, pose, _ = grid_setup(tmp_path, rows)
    with pytest.raises(NavError, match='unreachable') as caught:
        GridAStarBackend().plan(route, segment, pose)
    assert caught.value.code == 'NO_PATH'


def test_astar_corner_cutting_is_forbidden(tmp_path):
    route, segment, pose, _ = grid_setup(tmp_path, [[0, 100], [100, 0]], goal=(1.5, 1.5))
    with pytest.raises(NavError) as caught:
        GridAStarBackend().plan(route, segment, pose)
    assert caught.value.code == 'NO_PATH'


def test_unknown_default_and_explicit_override(tmp_path):
    route, segment, pose, _ = grid_setup(tmp_path, [[0, -1, 0]], goal=(2.5, .5))
    with pytest.raises(NavError) as caught:
        GridAStarBackend().plan(route, segment, pose)
    assert caught.value.code == 'NO_PATH'
    segment['planning']['unknown_is_obstacle'] = False
    assert GridAStarBackend().plan(route, segment, pose).length > 0


def test_astar_checks_height_occupied_goal_and_inflation(tmp_path):
    rows = [[0]*7 for _ in range(7)]
    rows[3][3] = 100
    route, segment, pose, document = grid_setup(tmp_path, rows, goal=(3.5, 3.5))
    with pytest.raises(NavError) as caught:
        GridAStarBackend().plan(route, segment, pose)
    assert caught.value.code == 'INVALID_GOAL'
    with pytest.raises(NavError) as caught:
        GridAStarBackend().plan(route, segment, Pose(.5, .5, 0, 0))
    assert caught.value.code == 'WRONG_PLATFORM_HEIGHT'
    grid = OccupancyGrid.from_mapping(document, {'inflation_radius': .1})
    assert not grid.free((3, 2))
    assert grid.free((1, 1))


def test_planning_cancellation(tmp_path):
    route, segment, pose, _ = grid_setup(tmp_path, [[0]*8 for _ in range(8)])
    cancelled = threading.Event()
    cancelled.set()
    with pytest.raises(NavError) as caught:
        GridAStarBackend().plan(route, segment, pose, cancelled)
    assert caught.value.code == 'PLANNING_CANCELLED'


def preset_setup(points):
    route = {'frame': 'map', 'reference_point': 'contact_projection',
             'nodes': {'B': {'position': [1., 0., .5], 'arrival': {'xy': .1, 'z': .1}}}}
    segment = {'path': points, 'end': 'B', 'allowed_modes': ['stair'],
               'planning': {'join_xy': .2, 'join_z': .1, 'max_jump': 2,
                            'entry_corridor_confirmed': True}}
    return route, segment


@pytest.mark.parametrize('points,code', [
    ([], 'EMPTY_PATH'),
    ([{'position': [0., 0., 0.], 'phase': 'entry', 'mode': 'stair'},
      {'position': [0., 0., 0.], 'phase': 'exit', 'mode': 'stair'}], 'DUPLICATE_POINT'),
    ([{'position': [0., 0., 0.], 'phase': 'entry', 'mode': 'stair'},
      {'position': [math.nan, 0., .5], 'phase': 'exit', 'mode': 'stair'}], 'INVALID_PATH'),
    ([{'position': [0., 0., 0.], 'phase': 'entry', 'mode': 'stair'},
      {'position': [10., 0., .5], 'phase': 'exit', 'mode': 'stair'}], 'PATH_JUMP'),
])
def test_preset_rejects_invalid_paths(points, code):
    route, segment = preset_setup(points)
    with pytest.raises(NavError) as caught:
        PresetPathBackend().plan(route, segment, Pose(0, 0, 0, 0))
    assert caught.value.code == code


def test_preset_keeps_annotations_and_does_not_fabricate_join():
    route, segment = preset_setup([
        {'position': [0., 0., 0.], 'phase': 'entry', 'mode': 'stair'},
        {'position': [1., 0., .5], 'phase': 'exit', 'mode': 'stair'}])
    path = PresetPathBackend().plan(route, segment, Pose(-.1, 0, 0, 0))
    assert len(path.points) == 2 and path.points[0].x == 0
    assert path.points[-1].phase == 'exit'
    with pytest.raises(NavError) as caught:
        PresetPathBackend().plan(route, segment, Pose(-.5, 0, 0, 0))
    assert caught.value.code == 'JOIN_OUT_OF_RANGE'
    segment['planning']['entry_corridor_confirmed'] = False
    with pytest.raises(NavError) as caught:
        PresetPathBackend().plan(route, segment, Pose(-.1, 0, 0, 0))
    assert caught.value.code == 'JOIN_UNVERIFIED'


def test_grid_start_equals_goal_does_not_create_a_detour(tmp_path):
    route, segment, pose, _ = grid_setup(tmp_path, [[0]*3 for _ in range(3)],
                                        start=(.3, .7), goal=(.3, .7))
    path = GridAStarBackend().plan(route, segment, pose)
    assert path.length == 0 and len(path.points) == 1


def test_nonadjacent_preset_duplicate_is_explicitly_rejected():
    route, segment = preset_setup([
        {'position': [0., 0., 0.], 'phase': 'entry', 'mode': 'stair'},
        {'position': [1., 0., .5], 'phase': 'exit', 'mode': 'stair'},
        {'position': [0., 0., 0.], 'phase': 'entry', 'mode': 'stair'}])
    with pytest.raises(NavError) as caught:
        PresetPathBackend().plan(route, segment, Pose(0, 0, 0, 0))
    assert caught.value.code == 'DUPLICATE_POINT'


def tracker(path, **config):
    goal = path.points[-1]
    return PurePursuit(path, dict(lookahead=.3, match_forward=10., max_cross_track=.4,
                                 max_height_error=.2, max_vx=.5, max_wz=1., max_accel=.3,
                                 max_angular_accel=.6, **config),
                       {'position': [goal.x, goal.y, goal.z], 'yaw': 0.,
                        'arrival': {'xy': .1, 'z': .1, 'yaw': .1, 'progress': .97}})


def test_height_overlap_and_progress_do_not_jump_to_other_level():
    path = PlannedPath([Waypoint(0, 0, 0), Waypoint(2, 0, 0),
                        Waypoint(2, 1, 1), Waypoint(0, 1, 1),
                        Waypoint(0, 0, 2), Waypoint(2, 0, 2)])
    follow = tracker(path)
    result = follow.update(Pose(.4, .02, 0, 0), .05)
    assert result.progress < .1
    assert result.target.z == 0
    # A pose on the overlapping upper floor cannot use a later path edge to hide
    # the impossible localization/history jump.
    with pytest.raises(NavError) as caught:
        follow.update(Pose(.4, 0, 2, 0), .05)
    assert caught.value.code in ('HEIGHT_DEVIATION', 'PATH_DEVIATION')


def test_tracker_commands_are_slew_limited_and_slow_near_goal():
    path = PlannedPath([Waypoint(0, 0, 0), Waypoint(2, 0, 0)])
    follow = tracker(path)
    first = follow.update(Pose(0, 0, 0, .1), .05)
    assert 0 <= first.vx <= .3*.05+1e-9
    assert abs(first.wz) <= .6*.05+1e-9
    outputs = []
    for x in [i*.02 for i in range(1, 91)]:
        outputs.append(follow.update(Pose(x, 0, 0, 0), .05))
    assert outputs[-1].vx < max(out.vx for out in outputs)
    assert all(abs(b.vx-a.vx) <= .3*.05+1e-9 for a, b in zip(outputs, outputs[1:]))


def test_no_unconfigured_stair_rotation_and_geometry_includes_yaw():
    path = PlannedPath([Waypoint(0, 0, 0, 'stair', 'stair'), Waypoint(1, 0, .5, 'exit', 'stair')])
    follow = tracker(path, allow_stair_correction=False, allow_turn_in_place=False)
    result = follow.update(Pose(.1, .02, .05, 0), .05)
    assert result.wz == 0
    # Advance through a sequence so history permits matching the final edge.
    for i in range(2, 10):
        follow.update(Pose(i*.1, 0, i*.05, 0), .05)
    with pytest.raises(NavError) as caught:
        follow.update(Pose(1, 0, .5, .3), .05)
    assert caught.value.code == 'FINAL_HEADING_UNREACHABLE'


def test_geometry_requires_height_and_path_progress():
    path = PlannedPath([Waypoint(0, 0, 0), Waypoint(1, 0, .5)])
    follow = tracker(path)
    assert not follow.update(Pose(0, 0, 0, 0), .05).geometry_reached
    for i in range(1, 11):
        result = follow.update(Pose(i/10, 0, i/20, 0), .05)
    assert result.geometry_reached
