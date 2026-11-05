"""Obstacle geometry and clearing require fresh, continuous observations."""
from dataclasses import replace
import math

import pytest

from stair_nav.core.obstacles import ObstacleConfig, ObstacleDetector, corridor_obstacles


FLOOR = [(0.8, 0., 0.)]
PERSON = [(0.8, 0., .3), (0.8, 0., .5), (0.8, 0., .7)]


def test_floor_robot_and_people_beside_or_behind_corridor_are_clear():
    points = FLOOR + [(0.1, 0., .4), (-.2, .1, .6), (.8, .6, .7),
                      (-.6, 0., .8), (1.4, 0., .8), (.8, 0., 2.)]
    result = corridor_obstacles(points, ObstacleConfig())
    assert result.valid and not result.blocked and result.points == 0
    assert math.isinf(result.nearest_distance)


def test_person_in_corridor_blocks_in_first_observation():
    detector = ObstacleDetector()
    result = detector.observe(PERSON, 1.)
    assert result.valid and result.blocked and result.reason == 'OBSTACLE'
    assert result.points == 3 and result.nearest_distance == pytest.approx(.8)


def test_duplicate_returns_and_single_voxel_noise_do_not_meet_evidence_threshold():
    points = [(0.81, .01, .41)]*100 + [(0.82, .02, .42)]
    result = corridor_obstacles(points, ObstacleConfig())
    assert result.valid and not result.blocked and result.points == 1
    assert corridor_obstacles(points, replace(ObstacleConfig(), min_points=1)).blocked


def test_nonfinite_points_do_not_create_obstacles_or_hide_valid_person():
    invalid = [(math.nan, 0., .5), (.8, math.inf, .5), (.8, 0., -math.inf)]
    assert corridor_obstacles(invalid+FLOOR, ObstacleConfig()).valid
    assert corridor_obstacles(invalid+PERSON, ObstacleConfig()).blocked
    assert not corridor_obstacles(invalid, ObstacleConfig()).valid


@pytest.mark.parametrize('points', [None, [], [(1., 2.)], [('invalid', 0., 0.)]])
def test_missing_empty_or_malformed_cloud_is_invalid_and_blocked(points):
    result = corridor_obstacles(points, ObstacleConfig())
    assert not result.valid and result.blocked


def test_path_corridor_follows_a_corner_while_robot_velocity_is_zero():
    config = replace(ObstacleConfig(), half_width=.2, forward_distance=1.4)
    path = [(0., 0.), (.6, 0.), (.6, 1.)]
    after_turn = [(.6, .6, .3), (.6, .6, .5), (.6, .6, .7)]
    assert not corridor_obstacles(after_turn, config).blocked
    result = corridor_obstacles(after_turn, config, path)
    assert result.blocked and result.nearest_distance == pytest.approx(1.2)
    # A stationary pause does not change the intended path-based region.
    detector = ObstacleDetector(config)
    assert detector.observe(after_turn, 1., path_xy=path).blocked
    assert detector.observe(after_turn, 1.1, path_xy=path).blocked


def test_path_swept_width_covers_corner_and_clips_forward_horizon():
    config = replace(ObstacleConfig(), half_width=.2, forward_distance=1.)
    path = [(0., 0.), (.6, 0.), (.6, 2.)]
    corner = [(.7, -.1, .3), (.7, -.1, .5), (.7, -.1, .7)]
    beyond_horizon = [(.6, .8, .3), (.6, .8, .5), (.6, .8, .7)]
    assert corridor_obstacles(corner, config, path).blocked
    assert not corridor_obstacles(beyond_horizon, config, path).blocked


@pytest.mark.parametrize('path', [[(math.nan, 0.)], [(0.,)]])
def test_invalid_path_does_not_authorize_motion(path):
    result = corridor_obstacles(FLOOR, ObstacleConfig(), path)
    assert not result.valid and result.blocked and result.reason == 'INVALID_PATH'


@pytest.mark.parametrize('path', [[], [(0., 0.)], [(0., 0.), (0., 0.)]])
def test_degenerate_path_retains_default_forward_corridor(path):
    assert corridor_obstacles(PERSON, ObstacleConfig(), path).blocked


def test_initial_clear_and_obstacle_clear_require_continuous_frames():
    detector = ObstacleDetector(replace(ObstacleConfig(), clear_seconds=.6))
    assert detector.result.blocked and not detector.result.valid
    assert detector.observe(FLOOR, 1.).blocked
    assert detector.observe(FLOOR, 1.3).blocked
    assert not detector.observe(FLOOR, 1.6).blocked
    assert detector.observe(PERSON, 1.7).blocked
    assert detector.observe(FLOOR, 1.8).blocked
    assert detector.observe(FLOOR, 2.1).blocked
    assert not detector.observe(FLOOR, 2.4).blocked


@pytest.mark.parametrize('kind', ['invalid', 'gap', 'backward', 'duplicate', 'nan', 'empty'])
def test_invalid_observations_and_time_discontinuities_reset_clearance(kind):
    detector = ObstacleDetector(replace(ObstacleConfig(), clear_seconds=.4))
    assert detector.observe(FLOOR, 1.).blocked
    assert not detector.observe(FLOOR, 1.4).blocked
    if kind == 'invalid':
        result = detector.observe(FLOOR, 1.5, valid=False)
    elif kind == 'gap':
        result = detector.observe(FLOOR, 2.)
    elif kind == 'backward':
        result = detector.observe(FLOOR, 1.3)
    elif kind == 'duplicate':
        result = detector.observe(FLOOR, 1.4)
    elif kind == 'nan':
        result = detector.observe(FLOOR, math.nan)
    else:
        result = detector.observe([], 1.5)
    assert result.blocked
    assert detector.observe(FLOOR, 2.1).blocked
    assert not detector.observe(FLOOR, 2.5).blocked


def test_invalid_frame_during_clear_countdown_cannot_contribute_wait_time():
    detector = ObstacleDetector(replace(ObstacleConfig(), clear_seconds=.6))
    assert detector.observe(FLOOR, 1.).blocked
    assert detector.observe(FLOOR, 1.3).blocked
    assert detector.observe(FLOOR, 1.4, valid=False).blocked
    assert detector.observe(FLOOR, 1.5).blocked
    assert detector.observe(FLOOR, 1.8).blocked
    assert not detector.observe(FLOOR, 2.1).blocked


def test_zero_clear_duration_still_requires_valid_fresh_clear_frame():
    detector = ObstacleDetector(replace(ObstacleConfig(), clear_seconds=0.))
    assert detector.observe(PERSON, 1.).blocked
    assert not detector.observe(FLOOR, 1.1).blocked
    assert detector.observe(FLOOR, 1.1).blocked


@pytest.mark.parametrize('options', [
    dict(forward_distance=0.), dict(half_width=-.1), dict(min_height=2.),
    dict(max_height=math.inf), dict(self_front=-1.), dict(voxel_size=0.),
    dict(min_points=0), dict(min_points=1.5), dict(min_points=True),
    dict(clear_seconds=-1.), dict(cloud_timeout=0.),
])
def test_invalid_configuration_is_rejected(options):
    with pytest.raises(ValueError):
        ObstacleConfig(**options)
