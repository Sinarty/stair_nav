"""Point-cloud frame transforms and the future section of a local route."""
import math

import pytest

from stair_nav.core.cloud_geometry import remaining_path, rigid_transform


def test_identity_and_sensor_translation_place_points_in_ground_frame():
    assert rigid_transform((0., 0., 0.), (0., 0., 0., 1.))((1., 2., 3.)) == (1., 2., 3.)
    to_ground = rigid_transform((.2, -.1, .6), (0., 0., 0., 1.))
    assert to_ground((.8, .1, -.6)) == pytest.approx((1., 0., 0.))
    assert to_ground((.8, .1, .4)) == pytest.approx((1., 0., 1.))


@pytest.mark.parametrize('quaternion,point,expected', [
    ((0., 0., math.sqrt(.5), math.sqrt(.5)), (1., 0., 0.), (0., 1., 0.)),
    ((math.sqrt(.5), 0., 0., math.sqrt(.5)), (0., 1., 0.), (0., 0., 1.)),
    ((0., math.sqrt(.5), 0., math.sqrt(.5)), (1., 0., 0.), (0., 0., -1.)),
    ((.5, .5, .5, .5), (1., 2., 3.), (3., 1., 2.)),
])
def test_full_quaternion_rotation_includes_roll_pitch_and_yaw(quaternion, point, expected):
    apply = rigid_transform((2., 3., 4.), quaternion)
    assert apply(point) == pytest.approx(tuple(value+offset for value, offset in zip(expected, (2., 3., 4.))))


def test_nearly_unit_quaternion_is_normalized_instead_of_scaling_points():
    angle = math.pi/4
    q = (0., 0., 1.02*math.sin(angle), 1.02*math.cos(angle))
    assert rigid_transform((0., 0., 0.), q)((1., 0., 0.)) == pytest.approx((0., 1., 0.))


@pytest.mark.parametrize('translation,quaternion', [
    ((math.nan, 0., 0.), (0., 0., 0., 1.)),
    ((0., math.inf, 0.), (0., 0., 0., 1.)),
    ((0., 0., 0.), (0., 0., math.nan, 1.)),
    ((0., 0., 0.), (0., 0., 0., 0.)),
    ((0., 0., 0.), (0., 0., 0., 2.)),
    ((0., 0.), (0., 0., 0., 1.)),
    ((0., 0., 0.), (0., 0., 1.)),
])
def test_invalid_transforms_raise_instead_of_moving_obstacles_to_free_space(translation, quaternion):
    with pytest.raises(ValueError):
        rigid_transform(translation, quaternion)


def test_future_route_discards_passed_section_but_retains_turn():
    path = [(-2., 0.), (-1., 0.), (1., 0.), (1., 2.)]
    assert remaining_path(path) == [(0., 0.), (1., 0.), (1., 2.)]


def test_off_route_robot_includes_connector_to_projected_remaining_path():
    path = [(-1., .3), (1., .3), (1., 2.)]
    assert remaining_path(path) == [(0., 0.), (0., .3), (1., .3), (1., 2.)]
    assert path == [(-1., .3), (1., .3), (1., 2.)]


def test_route_projection_clamps_to_endpoint_instead_of_extending_past_goal():
    assert remaining_path([(1., .3), (2., .3)]) == [(0., 0.), (1., .3), (2., .3)]
    assert remaining_path([(-2., 0.), (-1., 0.)]) == [(0., 0.), (-1., 0.)]


def test_missing_or_repeated_path_points_are_handled_without_division_by_zero():
    assert remaining_path([]) is None
    assert remaining_path([(.5, .2)]) == [(0., 0.), (.5, .2)]
    assert remaining_path([(.5, .2), (.5, .2)]) == [(0., 0.), (.5, .2)]
    assert remaining_path([(-1., 0.), (-1., 0.), (1., 0.)]) == [(0., 0.), (1., 0.)]


@pytest.mark.parametrize('path', [
    [(math.nan, 0.)], [(0., math.inf)], [(0.,)], [(0., 0., 0.)],
])
def test_invalid_path_coordinates_do_not_silently_select_a_clear_corridor(path):
    with pytest.raises(ValueError):
        remaining_path(path)


def test_progress_window_keeps_current_branch_at_a_path_self_intersection():
    path = [(-1., 0.), (1., 0.), (1., -1.), (0., -1.), (0., 1.)]
    assert remaining_path(path)[1] == (1., 0.)  # Earlier horizontal crossing.
    # The current crossing is five metres into this six-metre route.
    assert remaining_path(path, progress=5./6) == [(0., 0.), (0., 1.)]


def test_progress_window_does_not_jump_to_a_closer_passed_parallel_segment():
    path = [(-1., 0.), (1., 0.), (1., .05), (-1., .05)]
    assert remaining_path(path)[1] == (1., 0.)
    result = remaining_path(path, progress=3.05/4.05)
    assert result == [(0., 0.), (0., .05), (-1., .05)]


def test_progress_window_limits_projection_even_inside_a_long_segment():
    result = remaining_path([(-10., 0.), (10., 0.)], progress=.9, backtrack=.3, forward=1.)
    assert result[1] == pytest.approx((7.7, 0.))


@pytest.mark.parametrize('progress', [-.1, 1.1, math.nan, math.inf])
def test_invalid_progress_cannot_select_a_different_obstacle_corridor(progress):
    with pytest.raises(ValueError):
        remaining_path([(-1., 0.), (1., 0.)], progress=progress)
